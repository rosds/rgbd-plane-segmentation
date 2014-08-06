#include "Frame.cu"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>

using namespace cv;

int main(int argc, const char *argv[])
{

    Mat depth_image =
        imread("../res/datasets/rgbd_dataset_freiburg2_desk/depth/1311868164.338541.png", CV_LOAD_IMAGE_ANYDEPTH);
    Size s = depth_image.size();
    Mat points(s.width * s.height * 4, 1, CV_32F);

    depth_image.at<ushort>(0, 0) = 9753;
    cudaReadPointCloud((ushort*)depth_image.data, (float*)points.data, s.width, s.height);

    printf("%hu\n", ((ushort*)depth_image.data)[0]);
    printf("%f\n", ((float*)points.data)[0]);
    printf("%f\n", ((float*)points.data)[1]);
    printf("%f\n", ((float*)points.data)[2]);
    printf("%f\n", ((float*)points.data)[3]);

    //cloud -> points = points;

    return 0;
}
