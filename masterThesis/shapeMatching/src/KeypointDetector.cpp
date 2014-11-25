#include <match/KeypointDetector.hpp>

match::KeypointDetector::KeypointDetector(DetectorType type) {
    detectorType = type;


    double iss_gamma_21_ (0.975);
    double iss_gamma_32_ (0.975);
    double iss_min_neighbors_ (5);
    int iss_threads_ (4);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr iss_tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::search::KdTree<pcl::PointXYZI>::Ptr sift_tree(new pcl::search::KdTree<pcl::PointXYZI>());

    switch (detectorType) {

        case ISS_DETECTOR:
            iss_detector = boost::make_shared<ISSDetector>(ISSDetector());

            iss_detector->setSearchMethod(iss_tree);
            iss_detector->setThreshold21 (iss_gamma_21_);
            iss_detector->setThreshold32 (iss_gamma_32_);
            iss_detector->setMinNeighbors (iss_min_neighbors_);
            iss_detector->setNumberOfThreads (iss_threads_);

            modelResFactor = 2.0;
            break;

        case SIFT_DETECTOR:
            sift_detector = boost::make_shared<SIFTDetector>(SIFTDetector());
            sift_detector->setSearchMethod(sift_tree);
            sift_detector->setMinimumContrast(0.0);
            sift_detector->setScales(0.01, 3, 2);
            break;
    }
}


double match::KeypointDetector::computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices(2);
    std::vector<float> sqr_distances(2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size (); ++i)
    {
        if (! pcl_isfinite ((*cloud)[i].x))
        {
            continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}


void match::KeypointDetector::setCloudResolutionFactor(const double f) {
    modelResFactor = f;
}


void match::KeypointDetector::inputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    double model_resolution = computeCloudResolution(cloud);

    this->cloud = cloud;

    switch (detectorType) {
        case ISS_DETECTOR:
            iss_detector->setSalientRadius(3 * modelResFactor * model_resolution);
            iss_detector->setNonMaxRadius(2 * modelResFactor * model_resolution);

            iss_detector->setInputCloud(cloud);
            break;
        case SIFT_DETECTOR:
            pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZI>());
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
            pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

            ne.setInputCloud(cloud);
            ne.setSearchMethod(tree);
            ne.setRadiusSearch(0.03);
            ne.compute(*cloud_normals);

            new_cloud->resize(cloud->size());

            for (int i = 0; i < cloud->size(); i++) {
                new_cloud->points[i].x = cloud->points[i].x;                
                new_cloud->points[i].y = cloud->points[i].y;                
                new_cloud->points[i].z = cloud->points[i].z;                
                new_cloud->points[i].intensity = cloud_normals->points[i].curvature;                
            }

            sift_detector->setInputCloud(new_cloud);
            break;
    }
}

void match::KeypointDetector::getKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints) {
    switch (detectorType) {
        case ISS_DETECTOR:
            iss_detector->compute(*keypoints);
            break;
        case SIFT_DETECTOR:
            sift_detector->compute(*keypoints);
            break;
    }
}
