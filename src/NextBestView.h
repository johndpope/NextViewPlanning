//
// Created by Maria Ruxandra Robu on 26/05/2016.
//

#ifndef NEXTVIEWPLANNING_NEXTBESTVIEW_H
#define NEXTVIEWPLANNING_NEXTBESTVIEW_H

#include <Eigen/Dense>
#include <vector>

namespace nvp {
    void generateScansFromDegrees(std::vector<Camera> &scans,
                                  PointCloud &pc);

    void getEstimatedReconstruction(std::vector<Camera> const &scans,
                                    Eigen::MatrixXd &out_mergedScans);

    void compareOriginalWithReconstruction(PointCloud &ic,
                                           PointCloud &rc);


}// namespace nvp
#endif //NEXTVIEWPLANNING_NEXTBESTVIEW_H
