#include <Frame.hpp>


// Global static pointer used to ensure a single instance of class.
CameraParameters* Frame::depth_parameters = NULL;


CameraParameters* Frame::getDepthParameters() 
{
    if (!depth_parameters) {
        depth_parameters = new CameraParameters;
    }
    return depth_parameters;
}


void Frame::setCameraParameters(CameraParameters &parameters)
{
    depth_parameters = new CameraParameters(parameters);
}


Frame::Frame(std::string file_path)
{
    cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    /** Read the depth information from the file **/
    Mat depth_image = imread(file_path, CV_LOAD_IMAGE_ANYDEPTH);
    Size s = depth_image.size();

    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > points(s.width * s.height);

#ifndef WITH_CUDA
    ushort* depth_value = (ushort*)depth_image.data;
    cloud -> width = depth_image.cols;
    cloud -> height = depth_image.rows;
    cloud -> points.resize(cloud -> width * cloud -> height);

    int i = 0;
    for (ushort v = 0; v < depth_image.rows; v++) {
        for (ushort u = 0; u < depth_image.cols; u++, i++, depth_value++) {
            double depth = *depth_value / 5000.0;
            cloud -> points[i].z = depth;
            cloud -> points[i].x = (u - depth_parameters -> cx) * depth / depth_parameters -> fx;  
            cloud -> points[i].y = (v - depth_parameters -> cy) * depth / depth_parameters -> fy;
        }   
    }
#else
    cudaReadPointCloud((ushort*)depth_image.data, (float*)points.data(), s.width, s.height);
    cloud -> points = points;
#endif
}


Frame::Frame(pcl::PointCloud<pcl::PointXYZ>::Ptr cl)
{
    cloud = cl; 
    normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
}


/**
 *  segment_planes uses the OrganizedMultiPlaneDetection from PCL library
 */
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Frame::segment_planes(double dth, double ath, double cur)
{
    /** 
     * First step: calculate surface normals
     * Using Integral images normal estimation
     * Page: http://pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php
     */
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    ne.compute(*normals);

    pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>());

    /** Plane segmentation parameters */
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers(1000);
    mps.setAngularThreshold(0.017453 * ath); // degrees
    //mps.setMaximumCurvature(cur);
    mps.setDistanceThreshold(dth); // meters
    mps.setInputNormals(normals);
    mps.setInputCloud(cloud);

    // Plane segmentation
    mps.segmentAndRefine(
            regions,
            model_coefficients, 
            inlier_indices, 
            labels, 
            label_indices, 
            boundary_indices
    );

    //mps.segment(model_coefficients, inlier_indices);
    
    // Filter inliers
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planes(regions.size());

    extract.setInputCloud(cloud);
    extract.setNegative(false);
    for (size_t i = 0; i < regions.size(); i++) {
        pcl::IndicesPtr inliers = boost::make_shared<std::vector<int> >(inlier_indices[i].indices);
        extract.setIndices(inliers);
        planes[i] = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>());
        extract.filter(*planes[i]);
    }

    return planes;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr Frame::transform(const Eigen::Transform<float, 3, Eigen::Affine> &transformation)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *trans, transformation);
    cloud = trans;
    return trans;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr Frame::transform(const Transform3D &transformation)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *trans, transformation.transform);
    cloud = trans;
    return trans;
}
