#include <match/GHDistance.hpp>


match::GHDistance::GHDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr shapeA,
pcl::PointCloud<pcl::PointXYZ>::Ptr shapeB) {
    shape1 = shapeA;
    shape2 = shapeB;
}


double match::GHDistance::distortion(const std::vector<int> &permutation,
    const Eigen::MatrixXf &dX, const Eigen::MatrixXf &dY, const size_t N) {

    double max = 0.0f;
    double dist;
    for (size_t i = 0; i < N; i++) {
        for (size_t j = 0; j < N; j++) {
            dist = fabs(dX(i, j) - dY(permutation[i], permutation[j])); 
            if (dist > max) {
                max = dist;
            }
        }
    }
    return max;
}

void match::GHDistance::findMinDistortion(double *d, std::vector<int>::iterator perm) {
    const size_t n = shape1->size();
    const size_t m = shape2->size();

    match::Metric X, Y;

    X.setPointCloud(shape1);
    Y.setPointCloud(shape2);

    Eigen::MatrixXf dX = X.getEuclideanDistanceMatrix();
    Eigen::MatrixXf dY = Y.getEuclideanDistanceMatrix();


    const size_t N = 10;

    // index vector
    std::vector<int> index(N);
    for (size_t i = 0; i < N; i++) {
        index[i] = i;
    }
    std::copy(index.begin(), index.end(), perm);

    *d = distortion(index, dX, dY, N);
    double dist;

    do {
        dist = distortion(index, dX, dY, N);
        if (dist < *d) {
            *d = dist;
            std::copy(index.begin(), index.end(), perm);
        }
    } while(std::next_permutation(index.begin(), index.end()));
}
