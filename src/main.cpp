#include <iostream>
#include "PointCloud.h"
#include "defs.h"
#include "utilities.h"
#include "Camera.h"
#include <Eigen/Dense>
#include <Eigen/LU>


using namespace nvp;

int main() {

    std::cout << "Reading 3D model..." << std::endl;
    //read point cloud in
    PointCloud pc(MODEL_3_FILENAME);

    // get the point set - dimensions: 3 x numPts
    Eigen::MatrixXd myPointSetBefore;
    pc.getPoints(myPointSetBefore);

    //vector of cameras
    std::vector<Camera> cameras;

    // set the rotation and translation for the extrinsics of the camera
    double rotationX_deg = 0.0, rotationY_deg = 30.0, rotationZ_deg = 0.0;
    double translationX, translationY, translationZ;

    for (auto camNo = 0; camNo < 5 ; camNo++)
    {
        // create a new camera object
        Camera currentCamera(rotationX_deg,
                             rotationY_deg,
                             rotationZ_deg,
                             translationX,
                             translationY,
                             translationZ);

        cameras.push_back(currentCamera);
        std::cout<<"Cameras created : "<<cameras.size()<<std::endl;

        // use this to obtain all of the projected points, irrespective of their depth
        //pc.computeProjectedCoordinates(currentCamera);

        // use this to compute the nearest points as seen from the camera
        // a discretization step of 500 is used for now

#if 1
        pc.computeNearestProjectedPts(currentCamera);
        std::cout<<"yey it passed"<<std::endl;

        // center the camera wrt to the point cloud
        pc.getCenterXY(translationX, translationY);
        translationZ = pc.computeRadiusFromCentroid();

        // convert the projected points back to the world coordinate frame
        pc.computeWorldCoordinates(currentCamera);

        std::cout<<"This scan has "<<pc.m_numPoints<<" points"<<std::endl;
#endif
        //write each scan to a different file
        std::string outputFile = "scan_"+
                     std::to_string(camNo+1)+".ply";
        std::cout << "Writing file "<<outputFile<<std::endl;
        pc.writeCloud(outputFile);
        std::cout << "File "<<outputFile<<" written"<<std::endl;
        rotationY_deg += 30.0;
    }

    //this matrix concatenates all the points from the different scans
    Eigen::MatrixXd all_vertices; // 3x camNo x numPoints
    std::string reconstructedMesh = "reconstruction.ply";

    for (auto  camNo = 0; camNo < cameras.size(); ++camNo )
    {
        //for every scan create a point cloud and read the points in
        std::string inputFile = "scan_" +
                     std::to_string(camNo+1)+".ply";
        PointCloud pc(inputFile);

        //this function takes each point cloud and concatenates them into a single matrix
        pc.mergeClouds(all_vertices);
    }

   // output to a single file using the write function
    std::cout<<"Finished merging the clouds"<<std::endl;
    std::cout<<"Writing reconstructed cloud to file ..."<<std::endl;
    pc.writeCloud(reconstructedMesh);
    std::cout << "File "<<reconstructedMesh<<" written"<<std::endl;

    // clip points by setting a threshold at half the distance between minZ and maxZ
    // this is not really what we want, since we don't get all of the points
    // that would be seen first from the camera
    // pc.clipPointsByZ();

    return SUCCESS;
}