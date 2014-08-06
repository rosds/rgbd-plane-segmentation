#ifndef CROP_PLANE_INLIERS_HH
#define CROP_PLANE_INLIERS_HH

#include <iostream>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#define DISTANCE_THRESHOLD 0.03


class CropPlaneInliers 
{
public:
    typedef boost::shared_ptr<CropPlaneInliers> Ptr;
    typedef boost::shared_ptr<const CropPlaneInliers> ConstPtr;

    CropPlaneInliers() : distance_threshold(DISTANCE_THRESHOLD) {}
    ~CropPlaneInliers() {}
    
    /** \brief Set the plane coefficients to realize the projection 
     * \param[in] plane_coefficients The plane coefficients 
     */
    void 
    setPlaneCoefficients(const Eigen::Vector4f &plane_coefficients);

    /** \brief Set the plane contour or boundary
     * \param[in] boundary The boundary of the plane polygon
     */
    void
    setPlaneBoundary(const pcl::PointCloud<pcl::PointXYZ>::VectorType &boundary);

    /** \brief Set the source pointcloud from where to extract the inlier points
     * \param[in] cloud The source point cloud
     */
    void
    setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    /** \brief Set the point to plane distance threshold
     * \param[in] threshold The distance threshold
     */
    inline void
    setDistanceThreshold(double threshold)
    {
        distance_threshold = threshold;
    }

    /** \brief Filter the points in the plane laying inside the boundary
     * \param[out] cloud The inliers
     */
    void
    filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &inliers);

    void print();

private:
    double distance_threshold;
    Eigen::Vector4f plane;
    Eigen::Vector3f origin;
    Eigen::Vector3f normal;
    pcl::PointCloud<pcl::PointXYZ>::VectorType boundary;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};

#endif
