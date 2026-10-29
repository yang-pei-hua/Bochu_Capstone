#include "StepCloudSampler.h"
#include <BRepAdaptor_Surface.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepClass_FaceClassifier.hxx>
#include <BRep_Tool.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <random>
#include <stdexcept>
#include <vector>
#include <algorithm>
#include <cmath>

namespace {
void check(bool ok, const char* message) { if(!ok) throw std::runtime_error(message); }
void save(const QString& path,const QJsonObject& object) {
    QFile file(path);check(file.open(QIODevice::WriteOnly),"Cannot write sampler metadata");
    const auto data=QJsonDocument(object).toJson();check(file.write(data)==data.size(),"Cannot save sampler metadata");
}

const char* kind(GeomAbs_SurfaceType t);
void inspectStepTopologyImpl(const QStringList& args,QJsonObject& report) {
    check(args.size()==4,"Usage: inspect-step source.step report.json");
    STEPControl_Reader reader;std::ifstream input(std::filesystem::path(args[2].toStdWString()),std::ios::binary);
    check(input.good() && reader.ReadStream("source.step",input)==IFSelect_RetDone,"Cannot read STEP");
    check(reader.TransferRoots()>0,"No STEP roots");QJsonArray solids;int id=0;
    auto vector=[](auto v){return QJsonArray{v.X(),v.Y(),v.Z()};};
    for(TopExp_Explorer si(reader.OneShape(),TopAbs_SOLID);si.More();si.Next()) {
        QJsonArray faces;int faceId=0;
        for(TopExp_Explorer fi(si.Current(),TopAbs_FACE);fi.More();fi.Next()) {
            const auto face=TopoDS::Face(fi.Current());BRepAdaptor_Surface surface(face);
            GProp_GProps props;BRepGProp::SurfaceProperties(face,props);
            QJsonObject f{{"id",++faceId},{"kind",kind(surface.GetType())},{"area_mm2",props.Mass()},
                {"center_mm",vector(props.CentreOfMass())},{"reversed",face.Orientation()==TopAbs_REVERSED}};
            f["occt_surface_type"] = int(surface.GetType());
            if(surface.GetType()==GeomAbs_Plane){const auto p=surface.Plane();f["normal"]=vector(p.Axis().Direction());}
            if(surface.GetType()==GeomAbs_Cylinder){const auto c=surface.Cylinder();f["axis"]=vector(c.Axis().Direction());f["axis_point"]=vector(c.Location());f["radius_mm"]=c.Radius();}
            if(surface.GetType()==GeomAbs_Cone){const auto c=surface.Cone();f["axis"]=vector(c.Axis().Direction());f["angle_rad"]=c.SemiAngle();}
            faces.append(f);
        }
        Bnd_Box bounds;BRepBndLib::AddOptimal(si.Current(),bounds,false,false);double x0,y0,z0,x1,y1,z1;bounds.Get(x0,y0,z0,x1,y1,z1);
        solids.append(QJsonObject{{"id",++id},{"bounds_mm",QJsonArray{x0,y0,z0,x1,y1,z1}},{"faces",faces}});
    }
    report["status"]="inspected";report["solids"]=solids;
}
const char* kind(GeomAbs_SurfaceType t) {
    switch(t) {case GeomAbs_Plane:return "plane";case GeomAbs_Cylinder:return "cylinder";
    case GeomAbs_Cone:return "cone";case GeomAbs_Sphere:return "sphere";
    case GeomAbs_Torus:return "torus";default:return "other";}
}
}

void inspectStepTopology(const QStringList& args,QJsonObject& report) {inspectStepTopologyImpl(args,report);}

void sampleStepCloud(const QStringList& args,QJsonObject& report) {
    check(args.size()==6,"Usage: sample source.step report.json cloud.ply spacing_mm");
    bool valid=false;const double spacing=args[5].toDouble(&valid);
    check(valid && std::isfinite(spacing) && spacing>0,"Sampling spacing must be positive");
    STEPControl_Reader reader;
    std::ifstream source(std::filesystem::path(args[2].toStdWString()),std::ios::binary);
    check(source.good() && reader.ReadStream("source.step",source)==IFSelect_RetDone,"Cannot read STEP");
    check(reader.TransferRoots()>0,"STEP has no transferable roots");
    const auto shape=reader.OneShape();
    // Preserve source placement. Evaluate the analytic surface at interpolated
    // UV coordinates rather than emitting faceted triangle positions/normals.
    BRepMesh_IncrementalMesh mesh(shape,std::min(spacing*0.15,0.05),false,0.15,true);
    check(mesh.IsDone(),"STEP triangulation failed");
    struct Point {gp_Pnt p;gp_Vec n;};std::vector<Point> cloud;
    QJsonArray faces,solids;int solidId=0,faceId=0;
    std::mt19937 random(4500);std::uniform_real_distribution<double> unit(0,1);
    for(TopExp_Explorer si(shape,TopAbs_SOLID);si.More();si.Next()) {
        ++solidId;const auto start=cloud.size();const int firstFace=faceId+1;
        GProp_GProps props;BRepGProp::VolumeProperties(si.Current(),props);
        for(TopExp_Explorer fi(si.Current(),TopAbs_FACE);fi.More();fi.Next()) {
            ++faceId;const auto face=TopoDS::Face(fi.Current());BRepAdaptor_Surface surface(face);
            TopLoc_Location location;const auto triangles=BRep_Tool::Triangulation(face,location);
            check(!triangles.IsNull() && triangles->HasUVNodes(),"Face has no UV triangulation");
            const auto begin=cloud.size();std::vector<double> areas;
            double total=0;
            for(int i=1;i<=triangles->NbTriangles();++i) {
                int a,b,c;triangles->Triangle(i).Get(a,b,c);
                const auto p=triangles->Node(a),q=triangles->Node(b),r=triangles->Node(c);
                total+=gp_Vec(p,q).Crossed(gp_Vec(p,r)).Magnitude()*0.5;areas.push_back(total);
            }
            check(total>0,"Degenerate source face");
            // Minimum support preserves small chamfers and thin hole walls.
            const auto count=static_cast<std::size_t>(std::clamp(std::ceil(total/(spacing*spacing)),256.0,2048.0));
            std::size_t rejected=0;
            for(std::size_t i=0;i<count;) {
                const double target=(i+unit(random))*total/count;
                const int triangle=int(std::lower_bound(areas.begin(),areas.end(),target)-areas.begin())+1;
                int a,b,c;triangles->Triangle(triangle).Get(a,b,c);
                const double r=std::sqrt(unit(random)),t=unit(random);
                const auto uv=triangles->UVNode(a).XY()*(1-r)+triangles->UVNode(b).XY()*(r*(1-t))+triangles->UVNode(c).XY()*(r*t);
                BRepClass_FaceClassifier classifier(face,gp_Pnt2d(uv),1e-7);
                if(classifier.State()!=TopAbs_IN && classifier.State()!=TopAbs_ON) {
                    check(++rejected<count*100,"Cannot sample the trimmed face interior");continue;
                }
                gp_Pnt p;gp_Vec du,dv;surface.D1(uv.X(),uv.Y(),p,du,dv);
                auto n=du.Crossed(dv);check(n.SquareMagnitude()>1e-24,"Singular surface sample");
                n.Normalize();if(face.Orientation()==TopAbs_REVERSED)n.Reverse();
                cloud.push_back({p,n});
                ++i;
            }
            faces.append(QJsonObject{{"id",faceId},{"solid_id",solidId},{"kind",kind(surface.GetType())},
                {"first_point_id",double(begin+1)},{"point_count",double(cloud.size()-begin)},{"area_mm2",total},
                {"effective_spacing_mm",std::sqrt(total/count)},{"outside_trim_rejected",double(rejected)}});
        }
        solids.append(QJsonObject{{"id",solidId},{"first_face_id",firstFace},{"face_count",faceId-firstFace+1},
            {"first_point_id",double(start+1)},{"point_count",double(cloud.size()-start)},{"volume_mm3",props.Mass()}});
    }
    check(!cloud.empty(),"STEP contains no sampled solids");
    std::ofstream ply(std::filesystem::path(args[4].toStdWString()));check(ply.good(),"Cannot create PLY");
    ply<<"ply\nformat ascii 1.0\ncomment analytic STEP samples; units mm\nelement vertex "<<cloud.size()
       <<"\nproperty double x\nproperty double y\nproperty double z\nproperty double nx\nproperty double ny\nproperty double nz\nend_header\n"<<std::setprecision(17);
    for(const auto& p:cloud)ply<<p.p.X()<<' '<<p.p.Y()<<' '<<p.p.Z()<<' '<<p.n.X()<<' '<<p.n.Y()<<' '<<p.n.Z()<<'\n';
    ply.close();check(ply.good(),"Cannot finish PLY");
    const QFileInfo path(args[4]);
    save(path.dir().filePath(path.completeBaseName()+".json"),QJsonObject{{"unit","mm"},{"source","analytic_step_sampling"}});
    report["status"]="sampled";report["source_step"]=QFileInfo(args[2]).absoluteFilePath();
    report["unit"]="mm";report["spacing_mm"]=spacing;report["points"]=double(cloud.size());
    report["solid_count"]=solidId;report["face_count"]=faceId;report["faces"]=faces;report["solids"]=solids;
    report["minimum_samples_per_face"]=256;report["maximum_samples_per_face"]=2048;
    report["reference_labels_used_by_recognition"]=false;
}
