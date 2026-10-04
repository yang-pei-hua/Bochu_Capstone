#include "io/PointCloudAnalysis.h"
#include "io/PlyReader.h"

#include <QCoreApplication>
#include <algorithm>
#include <iostream>
#include <set>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
}
int main(int argc, char** argv)
{
    QCoreApplication application(argc,argv);
    try {
        PlyCloud cloud;
        for (int axis=0; axis<3; ++axis) for (float sign : {-1.0F,1.0F}) {
            for (int i=0;i<20;++i) for (int j=0;j<20;++j) {
                std::array<float,3> p{}, n{};
                p[static_cast<size_t>(axis)]=sign*3; n[static_cast<size_t>(axis)]=sign;
                p[static_cast<size_t>((axis+1)%3)]=-3+(i+0.5F)*0.3F;
                p[static_cast<size_t>((axis+2)%3)]=-3+(j+0.5F)*0.3F;
                cloud.positions.push_back(p); cloud.normals.push_back(n);
            }
        }
        std::vector<vtkIdType> ids;
        for (size_t i=0;i<cloud.positions.size();++i) ids.push_back(static_cast<vtkIdType>(i*2+7));
        auto result=analysePointCloud(cloud,ids,0.01);
        require(result.error.isEmpty() && result.surfaces.size()==6, "analytic box exposes six fitted faces");
        require(result.assignedPoints>=cloud.positions.size()*0.99, "face support maps to original observations");
        std::set<vtkIdType> valid(ids.begin(),ids.end()), assigned;
        for(const auto& surface:result.surfaces) for(auto id:surface.points) {
            require(valid.count(id) && assigned.insert(id).second, "stable source IDs are mapped once");
        }
        PlyCloud dense;
        for(int repeat=0;repeat<24;++repeat) {
            dense.positions.insert(dense.positions.end(),cloud.positions.begin(),cloud.positions.end());
            dense.normals.insert(dense.normals.end(),cloud.normals.begin(),cloud.normals.end());
        }
        ids.clear(); for(size_t i=0;i<dense.positions.size();++i) ids.push_back(static_cast<vtkIdType>(i+100));
        result=analysePointCloud(dense,ids,0.01);
        require(result.error.isEmpty() && result.processedPoints<dense.positions.size()
            && result.assignedPoints>=dense.positions.size()*0.99, "voxel downsampling does not lose original point memberships");
        if(argc>1) {
            PlyCloud input; QString error;
            require(PlyReader::read(QString::fromLocal8Bit(argv[1]),input,error), "load sample cloud");
            ids.clear(); for(size_t i=0;i<input.positions.size();++i) ids.push_back(static_cast<vtkIdType>(i));
            result=analysePointCloud(input,ids,0.08);
            require(result.error.isEmpty() && !result.surfaces.empty(), "sample cloud surfaces detected");
            std::cout << "Sample: " << result.surfaces.size() << " surfaces, " << result.assignedPoints << "/" << input.positions.size() << " points.\n";
        }
        std::cout<<"Point cloud analysis tests passed.\n"; return 0;
    } catch(const std::exception& error) { std::cerr<<error.what()<<'\n';return 1; }
}
