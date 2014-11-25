#include <iostream>
#include <ctime>
#include <sstream>

#include <Eigen/Core>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/keypoints/iss_3d.h>

#include <match/GHDistance.hpp>
#include <match/KeypointDetector.hpp>



int main (int argc, char** argv) {

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->initCameraParameters ();

    char* args[3];
    char option[5] = "-cam";
    char params[130] = "0.289216,4.36774/-0.0598384,-0.165688,0.48632/-0.0919335,-0.218632,0.798878/-0.0516432,0.985499,0.161629/0.8575/1918,1183/1,31";

    args[1] = option;
    args[2] = params;
    viewer->getCameraParameters(3, args);
    viewer->updateCamera();

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
    //reader.read("bunny_downsampled.pcd", *c1); 
    //reader.read("bunny_downsampled.pcd", *c2); 

    //reader.read("bunny.pcd", *bunny1); 
    //reader.read("bunny.pcd", *bunny2); 


    //pcl::transformPointCloud (*bunny2, *bunny2, transform_1);
    //pcl::transformPointCloud (*c2, *c2, transform_1);


    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints2(new pcl::PointCloud<pcl::PointXYZ>());

    match::KeypointDetector det(match::KeypointDetector::SIFT_DETECTOR);
    
    for (int i = 1; i < 9; i++) {

        c1->clear();
        std::stringstream pclName;
        pclName << "../models/BigChair/BigChair0" << i << ".pcd";

        reader.read(pclName.str(), *c1); 
        det.inputCloud(c1);

        viewer->removeAllPointClouds();

        //det.setCloudResolutionFactor(i); 

        clock_t begin = clock();
        det.getKeypoints(keypoints1);
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "Time: " << elapsed_secs << std::endl;

        //det.inputCloud(bunny2);
        //det.getKeypoints(keypoints2);

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(c1, 100, 100, 100);
        viewer->addPointCloud<pcl::PointXYZ>(c1, single_color2, "bunny2");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(keypoints1, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(keypoints1, single_color, "keypoints1");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "keypoints1");

        //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(keypoints2, 0, 255, 0);
        //viewer->addPointCloud<pcl::PointXYZ>(keypoints2, single_color2, "keypoints2");

        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));

        std::stringstream str;
        str << "partial" << i << ".png";
        viewer->saveScreenshot(str.str());
    }

/*
 *    match::GHDistance ghd(keypoints1, keypoints2);
 *
 *    double distortion;
 *    std::vector<int> perm(11);
 *
 *    clock_t begin = clock();
 *    ghd.findMinDistortion(&distortion, perm.begin());
 *    clock_t end = clock();
 *    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
 *    std::cout << "Time: " << elapsed_secs << std::endl;
 *
 *    std::cout << distortion << "  ";
 *    for (size_t i = 0; i < 10; i++) {
 *        viewer->addLine<pcl::PointXYZ>(keypoints1->points[i], keypoints2->points[perm[i]], 1.0, 0.0, 0.0, "line" + i);
 *        std::cout << perm[i] << " ";
 *    }
 *    std::cout << std::endl;
 */

    //viewer->addPointCloud<pcl::PointXYZ>(bunny1, "bunny1");


    /*
     *while (!viewer->wasStopped ())
     *{
     *    viewer->spinOnce (100);
     *    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
     *}
     */

    return 0;
}
