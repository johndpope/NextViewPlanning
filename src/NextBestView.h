//
// Created by Maria Ruxandra Robu on 26/05/2016.
//

#ifndef NEXTVIEWPLANNING_NEXTBESTVIEW_H
#define NEXTVIEWPLANNING_NEXTBESTVIEW_H

#include <Eigen/Dense>
#include <vector>

namespace nvp {
    //*************** Basic Framework *******************

    void generateScansFromDegrees(std::vector<Camera> &scans,
                                  PointCloud &pc);

    void getEstimatedReconstruction(std::vector<Camera> const &scans,
                                    Eigen::MatrixXd &out_mergedScans);

    void compareOriginalWithReconstruction(PointCloud &ic,
                                           PointCloud &rc);

    double evaluateNBV(std::vector<Camera> &kplus1Views,
                       PointCloud &originalPCD,
                       int zbufferSideSize = 200);

    //*************** Individual Section *******************

    // this function doesn't write all the intermediary scans
    void getEstimatedReconstructionFromKViews(PointCloud &pc,
                                              std::vector<Camera> const &kViews,
                                              Eigen::MatrixXd &out_mergedScans,
                                              int zbufferSideSize = 100);

    Camera getCameraFromDegrees(PointCloud &pc,
                                double &rotYDegrees);

    // this function generates the vector of Cameras for the k views
    void getCameraVecFromDegrees(PointCloud &pc,
                          Eigen::VectorXd &kYDegrees,
                          std::vector<Camera> &out_kCamVect);

    void getCandidateViewsDegrees(std::vector<Camera> &kViews,
                           Eigen::VectorXd &out_candidateYDegrees);


    int getNumNewPointsFromNewScan(PointCloud &pc,
                                   std::vector<Camera> &kViews,
                                   Camera &newView,
                                   int numPoints_kViews,
                                   int zbufferSideSize = 50);

    void evaluateEachCandidateView(PointCloud &pc,
                                   std::vector<Camera> &kViews,
                                   Eigen::VectorXd &candidateYDegrees,
                                   Eigen::VectorXd &out_viewScores);


}// namespace nvp
#endif //NEXTVIEWPLANNING_NEXTBESTVIEW_H
