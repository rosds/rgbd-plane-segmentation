#include <DataSet.hpp>

// Camera parameters for freiburg2 Dataset
const CameraParameters DataSet::freiburg2 = {
    517.3f,  // fx 
    516.5f,  // fy
    318.6f,  // cx
    255.3f   // cy
};


// Some colors for the planes
static double color[][3] = {
    {1.0, 0.501961, 0.501961}, 
    {1.0, 0.627451, 0.501961}, 
    {1.0, 1.0, 0.596078}, 
    {0.596078, 1.0, 0.596078}, 
    {0.501961, 1.0, 0.815686}, 
    {0.501961, 1.0, 1.0}, 
    {0.501961, 0.627451, 1.0}, 
    {0.501961, 0.501961, 1.0}, 
    {0.752941, 0.501961, 1.0}, 
    {1.0, 0.501961, 1.0}
};


DataSet::DataSet(DataSet::dataset_id id)
{
    switch (id) {
        case DataSet::FREIBURG_TWO_DESK:
            depth_params = freiburg2; 
            break;
        default:
            depth_params = freiburg2; 
    }

    Frame::setCameraParameters(depth_params);
}


DataSet::DataSet(DataSet::dataset_id id, string dataset_dir)
{
    // Set the camera parameters for the specified dataet
    switch (id) {
        case DataSet::FREIBURG_TWO_DESK:
            depth_params = freiburg2; 
            break;
        default:
            depth_params = freiburg2; 
    }

    // Set the fs directorey containing the data set
    set_directory(dataset_dir);

    Frame::setCameraParameters(depth_params);
}


void DataSet::set_directory(string dir)
{
    data_set_directory = dir;
    load_dimg_paths();
    load_groundtruth();
}


/**
 * Inits the pcl visualizer if it was not initialized before
 */
boost::shared_ptr<pcl::visualization::PCLVisualizer> DataSet::get_pcl_visualizer()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer -> setBackgroundColor(0, 0, 0);
    viewer -> addCoordinateSystem(0.01);

    // Set camera position
    char* args[3];
    char option[5] = "-cam";
    char params[130] = "0.0294884,29.4884/2.06455,-1.14778,0.846397/-3.76698,-7.46711,3.81319/0.275027,0.18879,0.942719/0.523599/1054,1183/1,31";
    args[1] = option;
    args[2] = params;
    viewer -> getCameraParameters(3, args);
    viewer -> updateCamera();

    return viewer;
}


Frame DataSet::get_frame(string ts)
{
    return Frame(data_set_directory + depth_paths[ts]);
}


void DataSet::load_dimg_paths()
{
    ifstream file;
    string str;
    file.open((data_set_directory + DEPTH_FILE).c_str());

    // Clear map before proceeding
    depth_paths.clear();

    // Read the file system path to each depth image file
    while(getline(file, str)) {
        if (str[0] != '#') {
            istringstream line(str);
            string ts, path;
            line >> ts >> path;

            // Assign the paths to the map: time_stamps -> paths
            depth_paths[ts] = path;
        }
    }
}


void DataSet::load_groundtruth()
{
    ifstream file;
    string str;
    file.open((data_set_directory + GROUNDTRUTH_FILE).c_str());

    // Clear map before proceeding
    groundtruth_transforms.clear();

    // Read the transformation for each frame
    while(getline(file, str)) {
        if (str[0] != '#') {
            istringstream line(str);
            string ts, dummy1, dummy2;
            float tx, ty, tz, qx, qy, qz, qw;
            line >> ts >> dummy1 >> dummy2 >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

            // Assign the transformations to the frames: time_stamps -> transformation
            groundtruth_transforms[ts] = Transform3D(tx, ty, tz, qx, qy, qz, qw);
        }
    }
}


/**
 * Function displayFrame
 * This function use the PCLVisualizer to display the point cloud corresponding
 * to the supplied time stamp
 * @param ts time stamp of the frame
 */
void DataSet::displayFrame(string ts)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = get_pcl_visualizer();

    Frame frame = get_frame(ts);
    Transform3D t = groundtruth_transforms[ts];
    pcl::PointCloud<pcl::PointXYZ>::Ptr trans = frame.transform(t);

    viewer -> addPointCloud(trans);

    while (!viewer -> wasStopped()) {
        viewer -> spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}


struct frame_viewer {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    Frame frame;
    double dth;
    double ath;
    double cur;
};


void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* f)
{
    frame_viewer* fv = (frame_viewer*)f;
    Frame frame = fv -> frame;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>((void*)&(fv -> viewer));

    bool reload = false;
    if (event.getKeySym () == "a" && event.keyDown ()) {
        fv -> ath += 0.05f;
        reload = true;
    }
    else if (event.getKeySym () == "s" && event.keyDown ()) {
        fv -> ath -= 0.05f;
        reload = true;
    }
    else if (event.getKeySym () == "d" && event.keyDown ()) {
        fv -> dth += 0.01f;
        reload = true;
    }
    else if (event.getKeySym () == "w" && event.keyDown ()) {
        fv -> dth -= 0.01f;
        reload = true;
    }
    else if (event.getKeySym () == "n" && event.keyDown ()) {
        fv -> cur += 0.02f;
        reload = true;
    }
    else if (event.getKeySym () == "p" && event.keyDown ()) {
        fv -> cur -= 0.02f;
        reload = true;
    }

    if (reload) {
        cout << "Distance threshold: " << fv -> dth << " Angle threshold: " << fv -> ath << " Curvature: " << fv -> cur << endl;

        viewer -> removeAllPointClouds();
        viewer -> addPointCloud(frame.getPointCloud());

        // Segment the frame
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planes;
        planes = frame.segment_planes(fv -> dth, fv -> ath, fv -> cur);

        // Load the segments on the viewer
        for (size_t i = 0; i < planes.size(); i++) {
            viewer -> addPointCloud<pcl::PointXYZ> (planes[i], "plane" + i);
            viewer -> setPointCloudRenderingProperties (
                    pcl::visualization::PCL_VISUALIZER_COLOR,
                    color[i][0],
                    color[i][1],
                    color[i][2], 
                    "plane" + i
            ); 
        }
    }
}


void
displayPlanarRegions (std::vector<pcl::PlanarRegion<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZ> > > &regions,
                      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
  char name[1024];
  unsigned char red [6] = {255, 0, 0, 255, 255, 0};
  unsigned char grn [6] = { 0, 255, 0, 255, 0, 255};
  unsigned char blu [6] = { 0, 0, 255, 0, 255, 255};

  pcl::PointCloud<pcl::PointXYZ>::Ptr contour (new pcl::PointCloud<pcl::PointXYZ>);

  for (size_t i = 0; i < regions.size (); i++)
  {
    Eigen::Vector3f centroid = regions[i].getCentroid ();
    Eigen::Vector4f model = regions[i].getCoefficients ();
    pcl::PointXYZ pt1 = pcl::PointXYZ (centroid[0], centroid[1], centroid[2]);
    pcl::PointXYZ pt2 = pcl::PointXYZ (centroid[0] + (0.5f * model[0]),
                                       centroid[1] + (0.5f * model[1]),
                                       centroid[2] + (0.5f * model[2]));
    sprintf (name, "normal_%d", unsigned (i));
    viewer->addArrow (pt2, pt1, 1.0, 0, 0, false, name);
    
    contour->points = regions[i].getContour ();
    sprintf (name, "plane_%02d", int (i));
    pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> color (contour, red[i%6], grn[i%6], blu[i%6]);
    if(!viewer->updatePointCloud(contour, color, name))
      viewer->addPointCloud (contour, color, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);
  }
}


/**
 * Function displaySegmentedFrame
 * This function use the PCLVisualizer to display the point cloud corresponding
 * to the supplied time stamp plus it is segmented
 * @param ts time stamp of the frame
 */
void DataSet::displaySegmentedFrame(string ts, bool normals)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = get_pcl_visualizer();

    Frame f = get_frame(ts);
    Transform3D t = groundtruth_transforms[ts];
    Frame frame(f.transform(t));

    /*
     *viewer -> addPointCloud(frame.getPointCloud());
     */

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = frame.segmentPlanes();

/*
 *    // Segment the frame
 *    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planes;
 *    planes = frame.segment_planes();
 *
 *    displayPlanarRegions(frame.regions, viewer);
 *
 *    // Load the segments on the viewer
 *    for (size_t i = 0; i < planes.size(); i++) {
 *        viewer -> addPointCloud<pcl::PointXYZ> (planes[i], "plane" + i);
 *        viewer -> setPointCloudRenderingProperties (
 *                pcl::visualization::PCL_VISUALIZER_COLOR,
 *                color[i][0],
 *                color[i][1],
 *                color[i][2], 
 *                "plane" + i
 *        ); 
 *    }
 */ 
    viewer -> addPointCloud(cloud);

    // Visualize normals
    if (normals) {
        viewer -> addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(frame.getPointCloud(), frame.getSurfaceNormals(),  100, 0.01, "normals");
        viewer -> setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "normals");
    }

    frame_viewer fv = {viewer, frame, 0.02f, 2.0f, 0.0f};

    viewer -> registerKeyboardCallback(keyboardEventOccurred, (void*)&fv);

    while (!viewer -> wasStopped()) {
        viewer -> spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}


void DataSet::showGroundTruth(int number_of_frames)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = get_pcl_visualizer();

    int i = 0;
    int j = 0;
    // Segment each frame
    for (map<string, string>::iterator it = depth_paths.begin(); (it != depth_paths.end()) && (i < number_of_frames); ++it) {

        if (j % 50 != 0) {
            j++;
            continue;
        }

        // Get the frame
        Frame frame = get_frame(it -> first);
        Transform3D t = groundtruth_transforms[it -> first];

        pcl::PointCloud<pcl::PointXYZ>::Ptr trans = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*frame.getPointCloud(), *trans, t.transform);

        // Adds the initial point cloud in white
        viewer -> addPointCloud(trans, it -> first);
        i++;
        j++;
    }


    while (!viewer -> wasStopped()) {
        viewer -> spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}


void DataSet::propagate_segmentation_1()
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = get_pcl_visualizer();

    /** First step is to segment the first frame and get the plane coefficients **/
    map<string, string>::iterator it = depth_paths.begin();
        
    // Segment the frame
    Frame f = get_frame(it -> first);
    Frame frame(f.transform(groundtruth_transforms[it -> first]));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planes;
    planes = frame.segment_planes(0.07, 3.5);

    char name[1024];
    unsigned char red [6] = {255, 0, 0, 255, 255, 0};
    unsigned char grn [6] = { 0, 255, 0, 255, 0, 255};
    unsigned char blu [6] = { 0, 0, 255, 0, 255, 255};

/*
 *    // Adds the initial point cloud in white
 *    viewer -> addPointCloud(frame.getPointCloud());
 *
 *    // Load the segments on the viewer
 *    for (size_t i = 0; i < planes.size(); i++) {
 *        viewer -> addPointCloud<pcl::PointXYZ> (planes[i], "plane" + i);
 *        viewer -> setPointCloudRenderingProperties (
 *                pcl::visualization::PCL_VISUALIZER_COLOR,
 *                color[i][0],
 *                color[i][1],
 *                color[i][2], 
 *                "plane" + i
 *        ); 
 *    }
 *
 *    displayPlanarRegions(frame.regions, viewer);
 *
 *    while (!viewer -> wasStopped()) {
 *        viewer -> spinOnce (100);
 *        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 *    }
 */

    //std::vector<pcl::ModelCoefficients> model_coefficients = frame.model_coefficients;

    ++it;

    //double epsilon = 0.03;
  
    int j = 1;
    for (; it != depth_paths.end(); j++) {
        viewer -> removeAllPointClouds();

        // get frame
        Frame f = get_frame(it -> first);
        f.transform(groundtruth_transforms[it -> first]);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = f.getPointCloud();

        //pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr 
            //optimize_model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(frame.getPointCloud()));

        /** Refine the model for each plane **/
        //for (size_t i = 0; i < model_coefficients.size(); i++) {

            //Eigen::VectorXf plane_coeff = Eigen::VectorXf::Map(model_coefficients[i].values.data(), 4);
            //Eigen::VectorXf new_plane_coeff;
            //std::vector<int> inliers;

            //for (j = 0; j < 5; j++) {
                //inliers.clear();
                //optimize_model -> selectWithinDistance(plane_coeff, epsilon, inliers);
                //optimize_model -> optimizeModelCoefficients(inliers, plane_coeff, new_plane_coeff);
                //plane_coeff = new_plane_coeff;
            //}

            //model_coefficients[i].values.assign(new_plane_coeff.data(), new_plane_coeff.data() + 4);

        //}

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr contour (new pcl::PointCloud<pcl::PointXYZ>);

        /** Assign the color of the point depending if it is inlier or not **/
        for (size_t i = 0; i < frame.regions.size(); i++) {

            pcl::PointCloud<pcl::PointXYZ>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZ>());
            CropPlaneInliers::Ptr cropper(new CropPlaneInliers());
            cropper -> setPlaneCoefficients(frame.regions[i].getCoefficients());
            cropper -> setPlaneBoundary(frame.regions[i].getContour());
            cropper -> setPointCloud(cloud);
            cropper -> filter(inliers);

            //contour -> points = frame.regions[i].getContour();
            //pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());

            //Eigen::Vector4f plane_coeff = frame.regions[i].getCoefficients();
            //coeff -> values.assign(plane_coeff.data(), plane_coeff.data() + 4);
            //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);

            //// Project inliers
            //pcl::ProjectInliers<pcl::PointXYZ>::Ptr projection(new pcl::ProjectInliers<pcl::PointXYZ>());
            //projection -> setModelType(pcl::SACMODEL_PLANE);
            //projection -> setInputCloud(cloud);
            //projection -> setIndices();
            //projection -> setModelCoefficients(coeff);
            //projection -> filter(*cloud_projected);

            //// CropHull
            //pcl::CropHull<pcl::PointXYZ>::Ptr crop_hull(new pcl::CropHull<pcl::PointXYZ>());
            //crop_hull -> setHullCloud(frame.getPointCloud());
            //crop_hull -> setHullIndices(frame.inliers)
            //crop_hull -> setInputCloud(cloud);

            //sprintf (name, "plane_%02d", int(i));
            //pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZ> col (cloud_projected, red[i%6], grn[i%6], blu[i%6]);
            //if(!viewer->updatePointCloud(cloud_projected, col, name))
              //viewer->addPointCloud (cloud_projected, col, name);
            //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, name);

            //for (pcl::PointCloud<pcl::PointXYZ>::iterator pit = cloud -> begin(); pit != cloud -> end(); ++pit) {
                //pcl::PointXYZRGB point(255, 255, 255);

                //if (pcl::isPointIn2DPolygon(*pit, *contour)) {
                    //cout << "WOOOH!" << endl;
                    //point = pcl::PointXYZRGB(color[i][0] * 255, color[i][1] * 255, color[i][2] * 255);
                    //break;
                //}

                //point.x = pit -> x;
                //point.y = pit -> y;
                //point.z = pit -> z;

                //color_cloud -> push_back(point);
            //}
            //
            viewer -> addPointCloud(inliers, "plane" + i);
            viewer -> setPointCloudRenderingProperties (
                    pcl::visualization::PCL_VISUALIZER_COLOR,
                    color[i][0],
                    color[i][1],
                    color[i][2], 
                    "plane" + i
            ); 

        }

        // Adds the initial point cloud in white
        //viewer -> addPointCloud(color_cloud);

        //boost::format fmt("%05d");
        //fmt  % j;
        //string out("./imgs/depth/" + fmt.str() + ".png");
        //cout << "Saving: " << out << endl;
        //viewer -> saveScreenshot(out);
        //j++;

        viewer -> spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));

        if (viewer -> wasStopped()) break;

        for(int i = 0; i < 10; ++i) ++it;
        cout << "frame " << j << endl;
    }
}


void DataSet::process_entire_data_set(bool screenshots)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = get_pcl_visualizer();

    int j = 1;
    // Segment each frame
    for (map<string, string>::iterator it = depth_paths.begin(); it != depth_paths.end(); ++it) {
        viewer -> removeAllPointClouds();

        // Segment the frame
        Frame frame = get_frame(it -> first);
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planes;
        planes = frame.segment_planes(DEFAULT_DISTANCE_THRESHOLD, DEFAULT_ANGLE_THRESHOLD);


        // Adds the initial point cloud in white
        viewer -> addPointCloud(frame.getPointCloud());

        // Load the segments on the viewer
        for (size_t i = 0; i < planes.size(); i++) {
            viewer -> addPointCloud<pcl::PointXYZ> (planes[i], "plane" + i);
            viewer -> setPointCloudRenderingProperties (
                    pcl::visualization::PCL_VISUALIZER_COLOR,
                    color[i][0],
                    color[i][1],
                    color[i][2], 
                    "plane" + i
            ); 
        }

        if (screenshots) {
            // Take the screenshot
            boost::format fmt("%05d");
            fmt  % j;
            string out("./imgs/depth/" + fmt.str() + ".png");
            cout << "Saving: " << out << endl;
            viewer -> saveScreenshot(out);
            j++;
        }
        else {
            viewer -> spinOnce (100);
            boost::this_thread::sleep (boost::posix_time::microseconds (100000));
            cout << "Displaying frame: " << it -> first << endl;
        }

        if (viewer -> wasStopped()) break;
    }
}
