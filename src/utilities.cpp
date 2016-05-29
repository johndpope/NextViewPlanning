//
// Created by Maria Ruxandra Robu on 13/05/2016.
//
#include <iostream>
#include "defs.h"
#include "utilities.h"
#include "converter.h"
#include <stdlib.h>


namespace nvp {

    Eigen::Matrix4d createRotationMatrix(double rotRadiansX,
                                         double rotRadiansY,
                                         double rotRadiansZ) {

        Eigen::Matrix4d rotationMat(Eigen::Matrix4d::Identity());
        // awesome website: http://ksimek.github.io/2012/08/22/extrinsic/


        Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(rotRadiansX,
                                                               Eigen::Vector3d(1, 0, 0)));
        Eigen::Affine3d ry = Eigen::Affine3d(Eigen::AngleAxisd(rotRadiansY,
                                                               Eigen::Vector3d(0, 1, 0)));
        Eigen::Affine3d rz = Eigen::Affine3d(Eigen::AngleAxisd(rotRadiansZ,
                                                               Eigen::Vector3d(0, 0, 1)));
        Eigen::Affine3d rotMat = rz * ry * rx;

        rotationMat = rotMat.matrix();

//        // set translations to the origin of the mesh
//        rotationMat(0, 3) = 0.0;
//        rotationMat(1, 3) = 0.0;
//        rotationMat(2, 3) = 164.0;

//        std::cout << "rotationMat: \n" << rotMat.matrix() << std::endl;
        return rotationMat;
    }

    int getValueInRange(double value,
                        double currentMin,
                        double currentMax,
                        double rangeMin,
                        double rangeMax) {
        // This function brings the *value* which is in the range [currentMin, currentMax]
        // into [rangeMin,rangeMax]

        return int((rangeMin + ((rangeMax - rangeMin) *
                                (value - currentMin)) / (currentMax - currentMin)));
    }

    bool isPointCloser(double newZ, double currentZ) {
        return newZ <= currentZ;
    }

    void createZBuffer(Eigen::MatrixXd &ptsCoord,
                       Eigen::MatrixXd &zBuffer,
                       Eigen::MatrixXd &idxBuffer,
                       int &numNearestPts,
                       int zbufferSideSize) {
        numNearestPts = 0;
        double xMin = ptsCoord.row(0).minCoeff();
        double xMax = ptsCoord.row(0).maxCoeff();
        double yMin = ptsCoord.row(1).minCoeff();
        double yMax = ptsCoord.row(1).maxCoeff();
        double zMin = ptsCoord.row(2).minCoeff();
        double zMax = ptsCoord.row(2).maxCoeff();
        int widthBuffer = zbufferSideSize;
        int heightBuffer = zbufferSideSize;

        zBuffer = zMax * Eigen::MatrixXd::Ones(widthBuffer, heightBuffer);
        idxBuffer = -1 * Eigen::MatrixXd::Ones(widthBuffer, heightBuffer);

        int currXIdx, currYIdx;
        for (int i = 0; i < ptsCoord.cols(); i++) {
            currXIdx = getValueInRange(ptsCoord.col(i)[0],
                                       xMin,
                                       xMax,
                                       0,
                                       widthBuffer - 1);
            currYIdx = getValueInRange(ptsCoord.col(i)[1],
                                       yMin,
                                       yMax,
                                       0,
                                       heightBuffer - 1);
//            std::cout << "xIdx = " << currXIdx << " & yIdx = " << currYIdx << std::endl;
            if (isPointCloser(ptsCoord(2, i),
                              zBuffer(currXIdx, currYIdx))) {
                // save new z depth in the buffer
                zBuffer(currXIdx, currYIdx) = ptsCoord(2, i);
                // save the corresponding idx for the point in the idxBuffer
                if (idxBuffer(currXIdx, currYIdx) == -1)
                    numNearestPts++;
                idxBuffer(currXIdx, currYIdx) = i;

            }
        }
        numNearestPts--;

    }

    void getNearestPointsToCamera(Eigen::MatrixXd &projectedPts,
                                  Eigen::MatrixXd &out_nearestProjectedPts,
                                  int zbufferSideSize) {
        Eigen::MatrixXd zBuffer, idxBuffer;
        int numNearestPts = 0;
        createZBuffer(projectedPts,
                      zBuffer,
                      idxBuffer,
                      numNearestPts,
                      zbufferSideSize);
        out_nearestProjectedPts = Eigen::MatrixXd::Zero(3, numNearestPts);
        int colIdx = 0;

        for (int i = 0; i < idxBuffer.rows(); i++) {
            for (int j = 0; j < idxBuffer.cols(); j++) {
                if (idxBuffer(i, j) != -1) {
                    out_nearestProjectedPts.col(colIdx) = projectedPts.col(idxBuffer(i, j));
                    colIdx++;
                }
            }
        }

    }

    void mergePointClouds(PointCloud &pc1, PointCloud &pc2, Eigen::MatrixXd &out_pointSet) {
        // Author: Karina Mady

        Eigen::MatrixXd pc1_pointSet, pc2_pointSet;
        pc1.getPoints(pc1_pointSet);
        pc2.getPoints(pc2_pointSet);

        out_pointSet = Eigen::MatrixXd::Zero(3, pc1.m_numPoints + pc2.m_numPoints);
        out_pointSet << pc1_pointSet, pc2_pointSet;

        std::cout << "Merged pc1 (" << pc1.m_numPoints <<
        ") with pc2(" << pc2.m_numPoints << ")\n";
    }

    void mergePointCloudsNoDuplicates(PointCloud &pc, Eigen::MatrixXd &points_all_scans) {
        // Author: Karina Mady
        Eigen::MatrixXd currentPointSet;
        pc.getPoints(currentPointSet);
        //std::cout << "The current scan matrix has " << currentPointSet.cols() << " points" << std::endl;
        //std::cout << "The reconstruction matrix before this merge has "
        //<< points_all_scans.cols() << " points" << std::endl;

        Eigen::MatrixXd unique_m_vertices(3, currentPointSet.cols());
        int pIdx = 0;
        int numberDuplicates = 0;

        //take every point in the current scan matrix
        for (int j = 0; j < currentPointSet.cols(); j++) {
            bool newPoint = true;
            //take every point in the reconstruction matrix
            for (int i = 0; i < points_all_scans.cols(); i++) {
                //and match them to see if there are duplicates
//              // compute the euclidean distance between the 2 vectors
                double distVect = (currentPointSet.col(j) - points_all_scans.col(i)).norm();
                // is the distance close to 0? if yes, they are duplicates
                if (std::abs(distVect) < std::numeric_limits<double>::epsilon()) {
                    //this point already exists
                    newPoint = false;
                    numberDuplicates++;
                }
            }
            //if the point has not been found, it's new, so add it as unique
            if (newPoint) {
                //save the unique points into a temporary matrix to be used later in merge
                unique_m_vertices.col(pIdx) = currentPointSet.col(j);
                pIdx++;
            }
        }
        pIdx--;

        //temporary matrix to store the unique points between the reconstruction matrix and current scan matrix
        Eigen::MatrixXd temporaryMat(3, points_all_scans.cols() + pIdx);
        temporaryMat << points_all_scans, unique_m_vertices.block(0, 0, 3, pIdx);
        points_all_scans = temporaryMat;
//        std::cout << "Found " << numberDuplicates << " duplicates" << std::endl;
//        std::cout << "The reconstruction matrix after merge has "
//        << points_all_scans.cols() << " points" << std::endl;
    }

    void printScoreToConsole(double score) {
        // Author: Karina Mady
        std::cout << "***\n";
        if (std::abs(score - 1.0) < std::numeric_limits<double>::epsilon())
            std::cout << "Congratulations, you have achieved the impossible.\n"
                    "You have created a perfect reconstruction ->"
            << score << std::endl;
        else if (std::abs(score - 0.0) < std::numeric_limits<double>::epsilon())
            std::cout << "Your value as a programmer is equal to your score ->"
            << score << std::endl;
        else if (score > 0.0 && score < 0.5)
            std::cout << "Rubbish result ->"
            << score << ". You didn't even get half the vertices" << std::endl;
        else if (score >= 0.5 && score < 0.8)
            std::cout << "Decent result ->" << score << std::endl;
        else if (score >= 0.8 && score < 0.9)
            std::cout << "Not bad, human ->" << score << std::endl;
        else if (score >= 0.9 && score < 1.0)
            std::cout << "Bow before me, mortal. I got ->" << score << std::endl;
        else
            std::cout << "You are the ultimate failure of mankind :(" << std::endl;
        std::cout << "***\n";

    }

    void computeNormals(Eigen::MatrixXd &pEIG,
                        Eigen::MatrixXd &pNormals,
                        int kNN) {
        int nPts = int(pEIG.cols()); // actual number of data points
        // int k = 0; //num of NN to return
        int dim = 3; // number of dimensions
        double eps = 0.0; // eps value for the kd search
        ANNdist sqRad = 0.00005; // returns around 250 - 350 neighbours

        ANNpointArray pANN; // fixed point set
        ANNpoint queryP;// query point
        ANNidxArray nnIdx; // near neighbor indices
        ANNdistArray dists; // near neighbor distances
        ANNkd_tree *kdTree; // search structure

        pANN = convertEigenMatToANNarray(pEIG);
        queryP = annAllocPt(dim);
        nnIdx = new ANNidx[kNN];
        dists = new ANNdist[kNN];
        kdTree = new ANNkd_tree(pANN, nPts, dim);
        // search for the closest neighbour in the kdtree
        // nPts = 1;
        for (int i = 0; i < nPts; i++) {
            // std::cout << "*** i = " << i << std::endl;
            Eigen::VectorXd thisPoint = pEIG.col(i);
            queryP = convertEigenVecToANNpoint(thisPoint);
            // approx fixed-radius kNN search

            // how many points are in my sqRad search?
            int numNN = kdTree->annkFRSearch(queryP, sqRad, 0);

            if (numNN > kNN) {
                kdTree->annkFRSearch(queryP, sqRad, kNN, nnIdx, dists, eps);

                // my local point cloud is
                Eigen::MatrixXd pLocal(3, kNN);
                for (int j = 0; j < kNN; j++) {
                    pLocal.col(j) = pEIG.col(nnIdx[j]);
                }
//                Eigen::Vector3d meanPLocal(pLocal.row(0).mean(), pLocal.row(1).mean(), pLocal.row(2).mean());
                Eigen::Vector3d meanPLocal = pLocal.rowwise().mean();
                Eigen::MatrixXd diff_p = pLocal - meanPLocal.replicate(1, kNN);

                // std::cout << "*** pLocal = " << pLocal <<  "\n";
                Eigen::Matrix3d qMat(Eigen::Matrix3d::Zero());
                for (int j = 0; j < kNN; j++) {
                    qMat += pLocal.col(j) * pLocal.col(j).transpose();
                }
                // qMat = qMat * (1/kNN); // normalize the cov matrix

                Eigen::JacobiSVD<Eigen::MatrixXd> svd(qMat, Eigen::ComputeFullU | Eigen::ComputeFullV);

                // Eigen::EigenSolver<Eigen::Matrix3d> es(qMat);
                // std::cout << "The eigenvalues of A are:" << std::endl << es.eigenvalues() << std::endl;
                // std::cout << "The singular values of A are:" << std::endl << svd.singularValues() << std::endl;
                // std::cout << "Its right singular vectors are the columns of the full V matrix:" << std::endl << svd.matrixV() << std::endl;

                // singular values are always sorted in decreasing order!
                Eigen::Vector3d normalVec = svd.matrixV().col(2);
                normalVec.normalize();
                pNormals.col(i) = normalVec;
            } // else increase the search radius

        }
        delete[] nnIdx;
        delete[] dists;
        delete kdTree;
        annDeallocPts(pANN);
        annDeallocPt(queryP);
        annClose(); // deallocate any shared memory used for the kd search
    }

    std::vector<Camera> getKplus1ViewVector(std::vector<Camera> &kViewVect,
                                            Camera &kplus1View) {
        std::vector<Camera> kplus1Views(kViewVect.size() + 1);

        for (int i = 0; i < kViewVect.size(); i++) {
            kplus1Views.push_back(kViewVect[i]);
        }
        kplus1Views.push_back(kplus1View);
        return kplus1Views;
    }


} //namespace nvp





