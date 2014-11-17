#ifndef MATCH_GHDISTANCE_HH
#define MATCH_GHDISTANCE_HH

#include <iostream>
#include <vector>
#include <algorithm>

#include <pcl/common/common_headers.h>

#include <match/Metric.hpp>

namespace match {

    class GHDistance {
    public:
        GHDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr shapeA,
            pcl::PointCloud<pcl::PointXYZ>::Ptr shapeB);
        ~GHDistance() {}

        void findMinDistortion(double *d, std::vector<int>::iterator perm);

    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr shape1;
        pcl::PointCloud<pcl::PointXYZ>::Ptr shape2;

        double distortion(const std::vector<int> &permutation,
                const Eigen::MatrixXf &dX, const Eigen::MatrixXf &dY, const size_t N);
    };

}   // namespace match

#endif
