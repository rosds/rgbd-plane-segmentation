#ifndef MATCH_METRIC_HH
#define MATCH_METRIC_HH

#include <Eigen/Core>
#include <pcl/common/common_headers.h>
#include <pcl/registration/distances.h>


namespace match {

    class Metric
    {
    public:
        Metric() {}
        ~Metric() {}

        void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr);
        Eigen::MatrixXf getEuclideanDistanceMatrix();

    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    };

}   // namespace match

#endif // MATCH_METRIC_HH
