#include <match/Metric.hpp>


void match::Metric::setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr) {
    cloud = cloudPtr;
}


Eigen::MatrixXf match::Metric::getEuclideanDistanceMatrix() {
    const size_t n = cloud->size();
    Eigen::MatrixXf distanceMat(n, n);

    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < n; j++) {
            distanceMat(i, j) = pcl::distances::l2(
                    cloud->points[i].getVector4fMap(),  
                    cloud->points[j].getVector4fMap());
        }
    } 


    return distanceMat;
}
