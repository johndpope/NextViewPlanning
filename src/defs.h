//
// Created by Maria Ruxandra Robu on 12/05/2016.
//

#ifndef NEXTVIEWPLANNING_DEFS_H
#define NEXTVIEWPLANNING_DEFS_H

#include <Eigen/Dense>
#include <Eigen/Sparse>


//#include <OpenMesh/Core/IO/MeshIO.hh>
//#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

namespace nvp {
    /* struct MyTraits: public OpenMesh::DefaultTraits
     {
         typedef OpenMesh::Vec3d Point;
 //        typedef OpenMesh::Vec3d Normal;
 //        VertexAttributes( OpenMesh::Attributes::Normal |
 //                          OpenMesh::Attributes::Color );
 //        FaceAttributes( OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color );
     };
     typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  MyMesh;
 */
    static double IMAGEPLANE_SIZE = 800;

    static std::string MODEL_1_FILENAME = "../models/_armadillo.ply";
    static std::string MODEL_1_OUTPUT_FILENAME = "../output/_out_armadillo.ply";

    static std::string MODEL_2_FILENAME = "../models/_cube.ply";
    static std::string MODEL_2_OUTPUT_FILENAME = "../output/_out_cube.ply";

    static std::string MODEL_3_FILENAME = "_bunny.ply";
    static std::string MODEL_3_OUTPUT_FILENAME = "_out_bunny.ply";

    // declares a column-major sparse matrix type of double
    // typedef Eigen::SparseMatrix<double> SparseMatrix;
    // typedef Eigen::Triplet<double> Triplet;


#define SUCCESS 0;
#define ERROR 1;

}// namespace nvp


#endif //NEXTVIEWPLANNING_DEFS_H
