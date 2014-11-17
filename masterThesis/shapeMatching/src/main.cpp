#include <iostream>
#include <ctime>

#include <Eigen/Core>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/keypoints/iss_3d.h>

#include <match/GHDistance.hpp>


double computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud) {
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud (cloud);

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


int main (int argc, char** argv) {

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters ();

    pcl::PointCloud<pcl::PointXYZ>::Ptr bunny1(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr bunny2(new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::PointCloud<pcl::PointXYZ>::Ptr c1(new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr c2(new pcl::PointCloud<pcl::PointXYZ> ());

    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    float theta = 0.35; // M_PI / 6; // The angle of rotation in radians
    transform_1 (0,0) = cos (theta);
    transform_1 (0,1) = -sin(theta);
    transform_1 (1,0) = sin (theta);
    transform_1 (1,1) = cos (theta);
    //    (row, column)

    // Define a translation of 2.5 meters on the x axis.
    transform_1 (2,3) = 0.5;

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read("bunny_downsampled.pcd", *c1); 
    reader.read("bunny_downsampled.pcd", *c2); 

    reader.read("bunny.pcd", *bunny1); 
    reader.read("bunny.pcd", *bunny2); 

    pcl::transformPointCloud (*bunny2, *bunny2, transform_1);
    pcl::transformPointCloud (*c2, *c2, transform_1);


    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints2(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector; 

    double model_resolution = computeCloudResolution(bunny1);
    double iss_gamma_21_ (0.975);
    double iss_gamma_32_ (0.975);
    double iss_min_neighbors_ (20);
    int iss_threads_ (4);

    iss_detector.setSearchMethod(tree);
    iss_detector.setSalientRadius (20 * model_resolution);
    iss_detector.setNonMaxRadius (10 * model_resolution);
    iss_detector.setThreshold21 (iss_gamma_21_);
    iss_detector.setThreshold32 (iss_gamma_32_);
    iss_detector.setMinNeighbors (iss_min_neighbors_);
    iss_detector.setNumberOfThreads (iss_threads_);
    iss_detector.setInputCloud (bunny1);
    iss_detector.compute (*keypoints1);

    iss_detector.setInputCloud (bunny2);
    iss_detector.compute (*keypoints2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(keypoints1, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(keypoints1, single_color, "keypoints1");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(keypoints2, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(keypoints2, single_color2, "keypoints2");


    match::GHDistance ghd(keypoints1, keypoints2);

    double distortion;
    std::vector<int> perm(11);


    clock_t begin = clock();
    ghd.findMinDistortion(&distortion, perm.begin());
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Time: " << elapsed_secs << std::endl;

    std::cout << distortion << "  ";
    for (size_t i = 0; i < 10; i++) {
        viewer->addLine<pcl::PointXYZ>(keypoints1->points[i], keypoints2->points[perm[i]], 1.0, 0.0, 0.0, "line" + i);
        std::cout << perm[i] << " ";
    }
    std::cout << std::endl;


    viewer->addPointCloud<pcl::PointXYZ>(bunny1, "bunny1");
    viewer->addPointCloud<pcl::PointXYZ>(bunny2, "bunny2");


    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}
