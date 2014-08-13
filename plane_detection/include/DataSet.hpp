#ifndef DATASET_HH
#define DATASET_HH

#include <Frame.hpp>
#include <Transform3D.hpp>
#include <CropPlaneInliers.hpp>

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdio.h>
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
 *  RGB-D dataset manipulation class. 
 *  This class is meant for manipulation of the RGB-D datasets.
 */
class DataSet 
{
    public:

        /**
         * @brief Benchmark datasets identifier.
         *
         * Enumerate the 
         * <a href="http://vision.in.tum.de/data/datasets/rgbd-dataset/download">
         * benchmark datasets</a> from the CVPR group at TU München. It is use
         * to set the particular intrinsic camera information when working with
         * the selected dataset.
         */
        enum dataset_id {
            FREIBURG_TWO_DESK, /**< Freiburg 2 intrinsic camera parameters */
        };

        /**
         *  @brief Only constructs the class using the camera parameters.
         *
         *  Builds the class just with the information corresponding to the
         *  cameras of the dataset specified.
         *  Notice that for working with the actual images the path to the 
         *  dataset directory should be specified.
         *
         *  @param id The dataset identification number according to the
         *  DataSet::dataset_id enumerator.
         *
         *  @see DataSet::set_directory
         */
        DataSet(dataset_id id);
        
        /**
         *  @brief Fully constructs the class with the dataset ID and path to
         *  the image files.
         *
         *  This constructor fully sets the required information to work with
         *  the specified benchmark dataset.
         *
         *  @param id The dataset identification number according to the
         *  DataSet::dataset_id enumerator.
         *  @param dataset_dir Path to the directory containing the dataset
         *  images.
         */
        DataSet(dataset_id id, string dataset_dir);

        /** Destructor **/
        ~DataSet() {}

        /** Intrinsic parameters for the freiburg2 dataset **/
        static const CameraParameters freiburg2;

        void set_directory(string dir);
        Frame get_frame(string ts);
        void displayFrame(string ts);
        void displaySegmentedFrame(string ts, bool normals=false);
        void showGroundTruth(int number_of_frames);
        void propagate_segmentation_1();
        void process_entire_data_set(bool screenshots=false);

    private:
        CameraParameters depth_params;
        string data_set_directory;
        map<string, string> depth_paths;
        map<string, Transform3D> groundtruth_transforms;

        /** Private Methods **/
        void load_dimg_paths();
        void load_groundtruth();
        boost::shared_ptr<pcl::visualization::PCLVisualizer> get_pcl_visualizer();
};

#endif
