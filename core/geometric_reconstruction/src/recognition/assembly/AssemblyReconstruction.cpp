#include "AssemblyInternal.h"
#include "preprocessing/detail/KdTree.h"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cstring>
#include <map>
#include <unordered_map>
#include <numeric>
#include <unordered_set>
#include <stdexcept>

namespace reconstruction {
namespace {
using namespace detail::assembly;
double quantile(std::vector<double> values,double fraction){if(values.empty())return 0;const auto i=std::min(values.size()-1,size_t(fraction*(values.size()-1)));std::nth_element(values.begin(),values.begin()+i,values.end());return values[i];}
bool finite(Vec3 p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
std::uint64_t sampleKey(const PointSample& p){std::uint64_t h=1469598103934665603ULL;for(double x:{p.position.x,p.position.y,p.position.z}){std::uint64_t bits=0;std::memcpy(&bits,&x,8);h^=bits;h*=1099511628211ULL;h^=h>>29;}return h;}
void checkCancel(const std::function<bool()>& cancelled){if(cancelled&&cancelled())throw std::runtime_error("Cancelled");}

void fitOpenParts(const PointStore& source, const AssemblyOptions& options,
                  AssemblyReconstructionResult& output, const std::function<bool()>& cancelled) {
    if (!options.buildSolids || !options.maximumProjectionParts) return;
    std::vector<AssemblyPart*> pending;
    for (auto& part : output.parts)
        if (!part.closed && part.patchIds.size() >= 2 && part.pointIds.size() >= 60)
            pending.push_back(&part);
    std::stable_sort(pending.begin(),pending.end(),[](const auto* a,const auto* b) {
        return a->pointIds.size() > b->pointIds.size();
    });
    if (pending.size() > options.maximumProjectionParts) pending.resize(options.maximumProjectionParts);
    std::unordered_map<PointId,const PointSample*> observations;
    for (const auto& p : source.points()) observations.emplace(p.id,&p);
    for (auto* part : pending) {
        checkCancel(cancelled);
        std::vector<PointSample> points;
        for (const auto id : part->pointIds) points.push_back(*observations.at(id));
        std::vector<PrimitiveEvidence> evidence;
        for (const auto id : part->patchIds) {
            const auto& patch = output.patches[id-1];
            if (!patch.planar) continue;
            PlaneEvidence plane;
            plane.id = id;
            plane.plane = {patch.normal,-dot(patch.normal,patch.origin)};
            plane.supportPointIds = patch.pointIds;
            evidence.emplace_back(std::move(plane));
        }
        ProjectionExtrusionOptions fitOptions;
        fitOptions.distanceTolerance = options.distanceTolerance;
        part->projection.emplace();
        if (fitProjectionExtrusion(PointStore(std::move(points)),fitOptions,
                                   *part->projection,evidence,cancelled))
            output.candidate.projectionPrograms.push_back(part->projection->operations);
        checkCancel(cancelled);
    }
}
struct UnionFind {
    std::vector<size_t> p;
    explicit UnionFind(size_t n):p(n){std::iota(p.begin(),p.end(),0);}
    size_t root(size_t x){while(p[x]!=x){p[x]=p[p[x]];x=p[x];}return x;}
    void join(size_t a,size_t b){a=root(a);b=root(b);if(a!=b)p[std::max(a,b)]=std::min(a,b);}
};
}
namespace detail::assembly {
namespace {
bool crossesObservedBoundary(const WorkingData& data, const AssemblyOptions& options, size_t a, size_t b) {
    const auto& first=data.points[a];const auto& second=data.points[b];
    for(auto endpoint:{a,b})for(const auto& neighbor:data.neighbors[endpoint]){
        const auto k=neighbor.second;const auto& wall=data.points[k];if(!wall.normal||!first.normal)continue;
        if(std::abs(dot(*wall.normal,*first.normal))>.5)continue;
        const double da=dot(subtract(first.position,wall.position),*wall.normal),db=dot(subtract(second.position,wall.position),*wall.normal);
        if(!((da>data.tolerance&&db<-data.tolerance)||(db>data.tolerance&&da<-data.tolerance)))continue;
        const auto intersection=add(first.position,scale(subtract(second.position,first.position),da/(da-db)));
        if(length(subtract(intersection,wall.position))<=data.spacing[k]*options.connectivityFactor)return true;
    }
    return false;
}
}

PatchExtraction extractPatches(const PointStore& source, const AssemblyOptions& options,
    const std::function<bool()>& cancelled) {
    if(source.unit()!=LengthUnit::Millimeter || source.size()<options.minimumPatchPoints || options.minimumPatchPoints<6
        || options.neighbors<6 || options.neighbors>128 || options.maximumWorkingPoints<options.minimumPatchPoints
        || !std::isfinite(options.distanceTolerance) || options.distanceTolerance<0
        || !std::isfinite(options.normalAngleRadians) || options.normalAngleRadians<=0 || options.normalAngleRadians>.7854
        || !std::isfinite(options.connectivityFactor) || options.connectivityFactor<=0 || !std::isfinite(options.alphaFactor) || options.alphaFactor<=0)
        throw std::runtime_error("Invalid metric assembly observations or options");
    PatchExtraction result;
    auto& data = result.work;
    auto& patches = result.patches;
    data.points=source.points();std::unordered_set<PointId> ids;
    for(auto& p:data.points){if(!p.id || !finite(p.position) || !ids.insert(p.id).second)throw std::runtime_error("Assembly observations contain invalid or duplicate point IDs");
        if(p.normal){if(!finite(*p.normal)||length(*p.normal)<1e-12)p.normal.reset();else p.normal=normalized(*p.normal);}}
    // Coordinate hashing keeps the budget independent of file order and IDs.
    if(data.points.size()>options.maximumWorkingPoints) {
        std::nth_element(data.points.begin(),data.points.begin()+options.maximumWorkingPoints,data.points.end(),[](const auto& a,const auto& b){return sampleKey(a)<sampleKey(b);});data.points.resize(options.maximumWorkingPoints);
    }
    std::sort(data.points.begin(),data.points.end(),[](const auto& a,const auto& b){return std::tie(a.position.x,a.position.y,a.position.z)<std::tie(b.position.x,b.position.y,b.position.z);});
    if(std::any_of(data.points.begin(),data.points.end(),[](const auto& p){return !p.normal;})) {
        PointCloudPreprocessingOptions normals;normals.normalNeighborCount=options.neighbors;PointCloudPreprocessingReport report;
        detail::preprocessing::estimateNormalsAndCurvature(data.points,normals,report);
    }
    checkCancel(cancelled);
    const detail::preprocessing::KdTree tree(data.points);const size_t count=data.points.size();
    data.neighbors.resize(count);data.spacing.resize(count);std::vector<double> noise;Vec3 low=data.points.front().position,high=low;
    const double angleCos=std::cos(options.normalAngleRadians);
    for(size_t i=0;i<count;++i) {
        if(i%1024==0)checkCancel(cancelled);
        const auto& p=data.points[i];low={std::min(low.x,p.position.x),std::min(low.y,p.position.y),std::min(low.z,p.position.z)};
        high={std::max(high.x,p.position.x),std::max(high.y,p.position.y),std::max(high.z,p.position.z)};
        data.neighbors[i]=tree.nearest(i,std::min(options.neighbors,count-1));std::vector<double> tangentDistances;
        for(const auto& neighbor:data.neighbors[i]){const auto& q=data.points[neighbor.second];if(!p.normal||!q.normal)continue;
            const double alignment=dot(*p.normal,*q.normal);if((p.normalOrientationKnown&&q.normalOrientationKnown?alignment:std::abs(alignment))<angleCos)continue;
            tangentDistances.push_back(std::sqrt(neighbor.first));noise.push_back(std::abs(dot(subtract(q.position,p.position),*p.normal)));}
        if(tangentDistances.empty())data.spacing[i]=std::sqrt(data.neighbors[i][std::min<size_t>(3,data.neighbors[i].size()-1)].first);
        else data.spacing[i]=tangentDistances[std::min<size_t>(2,tangentDistances.size()-1)];
    }
    const double diagonal=length(subtract(high,low));
    if(!std::isfinite(diagonal)||diagonal<=1e-12)throw std::runtime_error("Unresolved or nonfinite observation extent");
    data.tolerance=options.distanceTolerance>0?options.distanceTolerance:std::max(diagonal*1e-6,3*quantile(std::move(noise),.25));
    data.tolerance=std::max(data.tolerance,diagonal*1e-9);
    for(auto& spacing:data.spacing)spacing=std::max(spacing,data.tolerance*2);
    data.labels.assign(count,-1);
    for(size_t seed=0;seed<count;++seed) {
        if(data.labels[seed]!=-1||!data.points[seed].normal)continue;checkCancel(cancelled);
        const int label=int(patches.size());const auto normal=*data.points[seed].normal,origin=data.points[seed].position;
        std::vector<size_t> queue{seed};data.labels[seed]=label;
        for(size_t head=0;head<queue.size();++head){const auto i=queue[head];for(const auto& pair:data.neighbors[i]){
            const auto j=pair.second;const auto& p=data.points[j];if(data.labels[j]!=-1||!p.normal)continue;
            const double alignment=dot(normal,*p.normal);if((data.points[seed].normalOrientationKnown&&p.normalOrientationKnown?alignment:std::abs(alignment))<angleCos)continue;
            const double reach=options.connectivityFactor*std::max(data.spacing[i],data.spacing[j]);
            if(pair.first>reach*reach || std::abs(dot(subtract(p.position,origin),normal))>data.tolerance)continue;
            if(crossesObservedBoundary(data,options,i,j))continue;
            data.labels[j]=label;queue.push_back(j);
        }}
        if(queue.size()<options.minimumPatchPoints){for(auto i:queue)data.labels[i]=-2;continue;}
        FiniteSurfacePatch patch;patch.id=patches.size()+1;patch.orientationKnown=true;
        Vec3 meanNormal{};std::vector<double> spacing;
        for(auto i:queue){const auto& p=data.points[i];patch.origin=add(patch.origin,p.position);meanNormal=add(meanNormal,*p.normal);patch.orientationKnown &= p.normalOrientationKnown;spacing.push_back(data.spacing[i]);}
        patch.origin=scale(patch.origin,1.0/queue.size());Eigen::Matrix3d covariance=Eigen::Matrix3d::Zero();
        for(auto i:queue){const auto d=subtract(data.points[i].position,patch.origin);const Eigen::Vector3d v(d.x,d.y,d.z);covariance+=v*v.transpose();}
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(covariance);const auto n=eigen.eigenvectors().col(0);patch.normal={n[0],n[1],n[2]};
        if(dot(patch.normal,meanNormal)<0)patch.normal=scale(patch.normal,-1);
        const Vec3 reference=std::abs(patch.normal.x)<.8?Vec3{1,0,0}:Vec3{0,1,0};patch.xDirection=normalized(subtract(reference,scale(patch.normal,dot(reference,patch.normal))));
        patch.lower=patch.upper=data.points[queue.front()].position;double sum=0;
        for(auto i:queue){const auto p=data.points[i].position;const double d=dot(subtract(p,patch.origin),patch.normal);sum+=d*d;
            patch.lower={std::min(patch.lower.x,p.x),std::min(patch.lower.y,p.y),std::min(patch.lower.z,p.z)};patch.upper={std::max(patch.upper.x,p.x),std::max(patch.upper.y,p.y),std::max(patch.upper.z,p.z)};}
        patch.residual=std::sqrt(sum/queue.size());patch.planar=patch.residual<=data.tolerance;patch.spacing=quantile(std::move(spacing),.5);
        patches.push_back(std::move(patch));data.groups.push_back(std::move(queue));
    }
    // PCA normals blur at sharp edges. For unsigned estimates, retain the
    // observed positions and extend a nearby fitted plane by geometric
    // residual, rather than misclassifying a crisp edge as a curved strip.
    for(int pass=0;pass<3;++pass){size_t changed=0;
        for(size_t i=0;i<count;++i){auto& p=data.points[i];if(data.labels[i]>=0||p.normalOrientationKnown)continue;int best=-1;double residual=data.tolerance;
            for(const auto& neighbor:data.neighbors[i]){const int label=data.labels[neighbor.second];if(label<0||!patches[label].planar)continue;const auto& plane=patches[label];
                const double reach=options.connectivityFactor*std::max(data.spacing[i],data.spacing[neighbor.second]);if(neighbor.first>reach*reach)continue;
                const double distance=std::abs(dot(subtract(p.position,plane.origin),plane.normal));if(distance<residual){best=label;residual=distance;}}
            if(best>=0){data.labels[i]=best;data.groups[best].push_back(i);p.normal=patches[best].normal;++changed;}}
        if(!changed)break;
    }
    // Planar tests deliberately leave curved strips behind. Grow those
    // residuals by local smoothness, without forcing a fictitious plane.
    for(size_t seed=0;seed<count;++seed){if(data.labels[seed]>=0||data.labels[seed]==-3||!data.points[seed].normal)continue;
        checkCancel(cancelled);const int label=int(patches.size());std::vector<size_t> queue{seed};data.labels[seed]=label;
        for(size_t head=0;head<queue.size();++head){const auto i=queue[head];for(const auto& pair:data.neighbors[i]){const auto j=pair.second;
            if(data.labels[j]>=0||data.labels[j]==-3||!data.points[j].normal)continue;double alignment=dot(*data.points[i].normal,*data.points[j].normal);
            if(!data.points[i].normalOrientationKnown||!data.points[j].normalOrientationKnown)alignment=std::abs(alignment);
            const double reach=options.connectivityFactor*std::min(data.spacing[i],data.spacing[j]);
            if(alignment<angleCos||pair.first>reach*reach)continue;data.labels[j]=label;queue.push_back(j);}}
        if(queue.size()<options.minimumPatchPoints){for(auto i:queue)data.labels[i]=-3;continue;}
        FiniteSurfacePatch patch;patch.id=patches.size()+1;patch.normal=*data.points[seed].normal;patch.orientationKnown=true;
        patch.lower=patch.upper=data.points[seed].position;std::vector<double> spacing;
        for(auto i:queue){const auto& p=data.points[i];patch.origin=add(patch.origin,p.position);patch.orientationKnown &= p.normalOrientationKnown;spacing.push_back(data.spacing[i]);
            patch.lower={std::min(patch.lower.x,p.position.x),std::min(patch.lower.y,p.position.y),std::min(patch.lower.z,p.position.z)};
            patch.upper={std::max(patch.upper.x,p.position.x),std::max(patch.upper.y,p.position.y),std::max(patch.upper.z,p.position.z)};}
        patch.origin=scale(patch.origin,1.0/queue.size());patch.spacing=quantile(std::move(spacing),.5);
        patches.push_back(std::move(patch));data.groups.push_back(std::move(queue));
    }
    for (std::size_t index = 0; index < patches.size(); ++index)
        for (const auto sample : data.groups[index])
            patches[index].workingPointIds.push_back(data.points[sample].id);
    for (std::size_t i = 0; i < count; ++i)
        if (data.labels[i] < 0) result.unassignedPointIds.push_back(data.points[i].id);
    return result;
}

PatchGrouping groupPatches(const WorkingData& data, const std::vector<FiniteSurfacePatch>& patches,
    const AssemblyOptions& options) {
    PatchGrouping result;
    const auto count = data.points.size();
    struct Link{size_t all=0,convex=0;double distance=INFINITY;};std::map<std::pair<size_t,size_t>,Link> links;
    for(size_t i=0;i<count;++i){if(data.labels[i]<0)continue;for(const auto& pair:data.neighbors[i]){
        const auto j=pair.second;const auto a=data.labels[i],b=data.labels[j];if(b<0||a==b)continue;
        const auto& pa=patches[a];const auto& pb=patches[b];
        const auto na=pa.planar?pa.normal:*data.points[i].normal,nb=pb.planar?pb.normal:*data.points[j].normal;
        if(std::abs(dot(na,nb))>.995)continue;
        const double reach=options.connectivityFactor*std::max(data.spacing[i],data.spacing[j]);if(pair.first>reach*reach)continue;
        if(crossesObservedBoundary(data,options,i,j))continue;
        const auto d=subtract(data.points[j].position,data.points[i].position);auto& link=links[std::minmax(size_t(a),size_t(b))];++link.all;link.distance=std::min(link.distance,std::sqrt(pair.first));
        if(pa.orientationKnown&&pb.orientationKnown && dot(na,d)<=data.tolerance*2 && dot(nb,scale(d,-1))<=data.tolerance*2)++link.convex;
    }}
    UnionFind groups(patches.size());
    for(const auto& item:links){const auto& l=item.second;const bool known=patches[item.first.first].orientationKnown&&patches[item.first.second].orientationKnown;
        const bool convex=l.all>=3 && l.convex>=.7*l.all;const bool accepted=convex||(!known&&l.all>=3);
        result.connections.push_back({item.first.first+1,item.first.second+1,convex,l.all,l.distance,accepted,known});if(accepted)groups.join(item.first.first,item.first.second);}
    std::map<size_t,size_t> parts;result.patchParts.resize(patches.size());
    for(size_t i=0;i<patches.size();++i){const auto root=groups.root(i);auto inserted=parts.emplace(root,result.parts.size());
        if(inserted.second){AssemblyPart p;p.id=result.parts.size()+1;p.orientationKnown=true;p.lower=patches[i].lower;p.upper=patches[i].upper;result.parts.push_back(p);}
        auto& part=result.parts[inserted.first->second];result.patchParts[i]=inserted.first->second;part.patchIds.push_back(i+1);part.orientationKnown &= patches[i].orientationKnown;
        const auto& patch=patches[i];part.lower={std::min(part.lower.x,patch.lower.x),std::min(part.lower.y,patch.lower.y),std::min(part.lower.z,patch.lower.z)};
        part.upper={std::max(part.upper.x,patch.upper.x),std::max(part.upper.y,patch.upper.y),std::max(part.upper.z,patch.upper.z)};}
    for (auto& part : result.parts)
        for (const auto id : part.patchIds) {
            const auto& ids = patches[id - 1].workingPointIds;
            part.workingPointIds.insert(part.workingPointIds.end(), ids.begin(), ids.end());
        }
    return result;
}

std::vector<PointId> assignSourceObservations(const PointStore& source, const WorkingData& data,
    const AssemblyOptions& options, std::vector<FiniteSurfacePatch>& patches, PatchGrouping& grouping,
    const std::function<bool()>& cancelled) {
    std::vector<PointId> unassigned;
    const detail::preprocessing::KdTree tree(data.points);
    const double angleCos = std::cos(options.normalAngleRadians);
    for(const auto& p:source.points()) {
        if(unassigned.size()%2048==0)checkCancel(cancelled);int selected=-1;
        for(const auto& pair:tree.nearest(p.position,4)){const auto j=pair.second;if(data.labels[j]<0)continue;
            const double reach=options.connectivityFactor*data.spacing[j]*2;if(pair.first>reach*reach)continue;
            if(p.normal&&finite(*p.normal)&&length(*p.normal)>1e-12&&data.points[j].normal){double alignment=dot(normalized(*p.normal),*data.points[j].normal);if(!p.normalOrientationKnown)alignment=std::abs(alignment);if(alignment<angleCos)continue;}
            selected=data.labels[j];break;}
        if(selected<0)unassigned.push_back(p.id);
        else{patches[selected].pointIds.push_back(p.id);grouping.parts[grouping.patchParts[selected]].pointIds.push_back(p.id);}
    }
    return unassigned;
}
} // namespace detail::assembly

bool reconstructAssembly(const PointStore& source, const AssemblyOptions& options,
    AssemblyReconstructionResult& output, std::string& error, const std::function<bool()>& cancelled) {
    output = {};
    error.clear();
    try {
        auto extraction = extractPatches(source, options, cancelled);
        auto& work = extraction.work;
        auto& patches = extraction.patches;
        auto grouping = groupPatches(work, patches, options);
        recoverBoundaries(work, options, grouping.connections, patches, cancelled);
        checkCancel(cancelled);
        output.unassignedPointIds = assignSourceObservations(source, work, options, patches, grouping, cancelled);
        output.candidate = closeParts(work, options, patches, grouping.parts, source.size(), cancelled);
        output.inputPoints = source.size();
        output.workingPoints = work.points.size();
        output.distanceTolerance = work.tolerance;
        output.unassignedWorkingPointIds = std::move(extraction.unassignedPointIds);
        output.patches = std::move(patches);
        output.connections = std::move(grouping.connections);
        output.parts = std::move(grouping.parts);
        fitOpenParts(source, options, output, cancelled);
        if (output.patches.empty()) {
            error = "No finite surface patches with sufficient neighborhood support";
            return false;
        }
        return true;
    } catch (const std::exception& e) {
        output = {};
        error = e.what();
        return false;
    }
}
}
