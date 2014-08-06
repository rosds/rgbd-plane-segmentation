#include <DataSet.hpp>
#include <Frame.hpp>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/normal_3d.h>

#include <iostream>

using namespace std;

int main(int argc, const char *argv[])
{
    DataSet ds(FREIBURG_TWO_DESK, "../res/datasets/rgbd_dataset_freiburg2_desk/");
    //ds.displaySegmentedFrame("1311868164.338541");
    Frame f = ds.get_frame("1311868164.338541");
    //ds.process_entire_data_set();
    //ds.showGroundTruth(4);
    //ds.propagate_segmentation_1();

    return 0;
}
