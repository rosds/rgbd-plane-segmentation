#ifndef DATASET_HH
#define DATASET_HH

#include <Frame.hpp>
#include <Transform3D.hpp>
#include <CropPlaneInliers.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/registration/transforms.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/impl/extract_polygonal_prism_data.hpp>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/crop_hull.h>
#include <boost/format.hpp>

using namespace std;
using namespace cv;

#define DEPTH_FILE "depth.txt"
#define GROUNDTRUTH_FILE "combined_file.txt"

/**
 * This Enumeration is only to name the camera parameters corresponding to the
 * specific data set.
 */
enum set_id {
    FREIBURG_TWO_DESK,
};


class DataSet 
{
    private:
        depth_cam_params depth_params;
        string data_set_directory;
        map<string, string> depth_paths;
        map<string, Transform3D> groundtruth_transforms;

        /** Private Methods **/
        void load_dimg_paths();
        void load_groundtruth();
        boost::shared_ptr<pcl::visualization::PCLVisualizer> get_pcl_visualizer();

    public:
        /** Constructors **/
        DataSet(set_id id);
        DataSet(set_id id, string dataset_dir);

        /** Destructor **/
        ~DataSet() {}

        /** Intrinsic parameters for the freiburg2 dataset **/
        static const depth_cam_params freiburg2;

        void set_directory(string dir);
        Frame get_frame(string ts);
        void displayFrame(string ts);
        void displaySegmentedFrame(string ts, bool normals=false);
        void showGroundTruth(int number_of_frames);
        void propagate_segmentation_1();
        void process_entire_data_set(bool screenshots=false);
};

#endif
