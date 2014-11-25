#ifndef SHAPE_MATCHING_KEYPOINT_DETECTOR_HH
#define SHAPE_MATCHING_KEYPOINT_DETECTOR_HH

#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/sift_keypoint.h>

#include <boost/make_shared.hpp>

namespace match {

class KeypointDetector {
    public:
        typedef pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> ISSDetector;
        typedef pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointXYZ> SIFTDetector;

        enum DetectorType {
            ISS_DETECTOR,
            SIFT_DETECTOR,
        };

        /** Constructor **/
        KeypointDetector(DetectorType type);

        /** Input cloud **/
        void inputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        
        void setCloudResolutionFactor(const double f);

        void getKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints);

    private:
        /** Intrinsic Shape Signature keypoint detector **/
        ISSDetector::Ptr iss_detector; 

        /** SIFT detector **/
        SIFTDetector::Ptr sift_detector;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

        double modelResFactor;

        double computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

        DetectorType detectorType;
};

} // namespace match

#endif // SHAPE_MATCHING_KEYPOINT_DETECTOR_HH
