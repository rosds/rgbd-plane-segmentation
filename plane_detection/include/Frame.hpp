#ifndef FRAME_HH
#define FRAME_HH

#ifdef __CUDACC__
#define CUDA_CALLABLE_MEMBER __host__ __device__
#else
#define CUDA_CALLABLE_MEMBER
#endif

#include <Transform3D.hpp>

#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/registration/transforms.h>

#define DEFAULT_DISTANCE_THRESHOLD 0.07f
#define DEFAULT_ANGLE_THRESHOLD 3.5f
#define DEFAULT_CURVATURE 0.0

using namespace cv;

/** Intrinsic camera parameters **/
struct depth_cam_params {
    float fx; // focal length 
    float fy;
    float cx; // center of camera
    float cy;
};

/** Cuda functions **/
extern void cudaReadPointCloud(ushort* depth, float* points, const int width, const int height);


class Frame 
{
    public:
        Frame(std::string file_path, depth_cam_params params);
        Frame(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        ~Frame() {}

        /** \brief Getter for the point cloud. */
        pcl::PointCloud<pcl::PointXYZ>::Ptr getPointCloud()
        { 
            return cloud; 
        }

        /** \brief Getter for surface normals */
        pcl::PointCloud<pcl::Normal>::Ptr getSurfaceNormals()
        { 
            return normals;
        }
        
        /** \brief Use the pcl::OrganizedMultiPlaneSegmentation to segment the
          * planes in the point cloud of this frame
          */
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> segment_planes(
                double dth=DEFAULT_DISTANCE_THRESHOLD, 
                double ath=DEFAULT_ANGLE_THRESHOLD,
                double cur=DEFAULT_CURVATURE
        );

        std::vector< pcl::PlanarRegion<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZ> > > regions;

        std::vector<pcl::ModelCoefficients> model_coefficients;
        std::vector<pcl::PointIndices> inlier_indices;
        std::vector<pcl::PointIndices> boundary_indices;
        std::vector<pcl::PointIndices> label_indices;

        /** \brief Transform the cloud with the given affine transformation **/
        pcl::PointCloud<pcl::PointXYZ>::Ptr transform(const Eigen::Transform<float, 3, Eigen::Affine> &trans);

        /** \brief Transform the cloud with the given affine transformation **/
        pcl::PointCloud<pcl::PointXYZ>::Ptr transform(const Transform3D &trans);

    private:
        /** \brief Point cloud corresponding to this frame. **/
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

        /** \brief Surface normals **/
        pcl::PointCloud<pcl::Normal>::Ptr normals;

        /** \brief Use CUDA parallelization to create point cloud **/

};

#endif
