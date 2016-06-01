//
// Created by Maria Ruxandra Robu on 12/05/2016.
//

#ifndef NEXTVIEWPLANNING_DEFS_H
#define NEXTVIEWPLANNING_DEFS_H

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

namespace nvp {
    struct MyTraits: public OpenMesh::DefaultTraits
    {
        typedef OpenMesh::Vec3d Point;
        typedef OpenMesh::Vec3d Normal;
        VertexAttributes( OpenMesh::Attributes::Normal);
    };
    typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  MyMesh;

    static const int IMAGEPLANE_SIZE = 600;
    static const int FOCAL_LENGTH = 800;


    static std::string MODEL_1_FILENAME = "../models/_armadillo.ply";
    static std::string MODEL_1_OUTPUT_FILENAME = "../output/_out_armadillo.ply";

    static std::string MODEL_2_FILENAME = "../models/_bunny.ply";
    static std::string MODEL_2_OUTPUT_FILENAME = "../output/_out_bunny.ply";

    static std::string MODEL_3_FILENAME = "../models/_dragon.ply";
    static std::string MODEL_3_OUTPUT_FILENAME = "../output/_out_dragon.ply";




#define SUCCESS 0;
#define ERROR 1;

}// namespace nvp



#endif //NEXTVIEWPLANNING_DEFS_H
