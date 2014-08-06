#include <CropPlaneInliers.hpp>

void
CropPlaneInliers::setPlaneCoefficients(const Eigen::Vector4f &plane_coefficients)
{
    plane = plane_coefficients;
    normal = plane_coefficients.head(3);
    origin = normal * (-plane_coefficients(3));
}

void
CropPlaneInliers::setPlaneBoundary(const pcl::PointCloud<pcl::PointXYZ>::VectorType &b)
{
    boundary = b;
}

void
CropPlaneInliers::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &c)
{
    cloud = c;
}

int 
pnpoly(pcl::PointXYZ point, 
std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > boundary_2D,
Eigen::Matrix<float, 3, 2> B)
{
    Eigen::Vector3f point3d(point.x, point.y, point.z);
    Eigen::Vector2f p = B.transpose() * point3d;

    int i, j, c = 0;
    for (i = 0, j = boundary_2D.size() - 1; i < boundary_2D.size(); j = i++) {
        Eigen::Vector2f edge_a = boundary_2D[i];
        Eigen::Vector2f edge_b = boundary_2D[j];
        if (((edge_a(1) > p(1)) != (edge_b(1) > p(1))) &&
                (p(0) < (edge_b(0) - edge_a(0)) * (p(1) - edge_a(1)) / (edge_b(1) - edge_a(1)) + edge_a(0)) )
            c = !c;
    }
    return c;
}

void
CropPlaneInliers::filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &inliers)
{
    pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundary_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_inliers(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients());

    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr 
        optimize_model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud));

    //for (int j = 0; j < 5; j++) { 
        // Find the inliers
        optimize_model -> selectWithinDistance(plane, distance_threshold, inlier_indices -> indices);

        plane_coefficients -> values.assign(plane.data(), plane.data() + 4);
        pcl::ProjectInliers<pcl::PointXYZ>::Ptr projection(new pcl::ProjectInliers<pcl::PointXYZ>());
        projection -> setModelType(pcl::SACMODEL_PLANE);
        projection -> setModelCoefficients(plane_coefficients);

        // Project the boundary
        boundary_cloud -> points = boundary;
        boundary_cloud -> width = boundary.size();
        boundary_cloud -> height = 1;
        projection -> setInputCloud(boundary_cloud);
        projection -> filter(*boundary_cloud);

        boundary = boundary_cloud -> points;

        // Project inliers
        projection -> setInputCloud(cloud);
        projection -> setIndices(inlier_indices);
        projection -> filter(*cloud_projected);

        // Get the first point of the boundary as a reference point
        Eigen::Vector3f refPoint(boundary[0].x, boundary[0].y, boundary[0].z);
        Eigen::Vector3f u = (refPoint - origin).normalized();
        Eigen::Vector3f v = u.cross(normal);

        // Change of basis Matrix
        Eigen::Matrix<float, 3, 2> B;
        B.col(0) = u;
        B.col(1) = v;

        // Convert the pcl::Points vector to Eigen::Vector2f 
        std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > boundary_2D;
        for (size_t i = 0; i < boundary.size(); i++) {
            Eigen::Vector3f point3d(boundary[i].x, boundary[i].y, boundary[i].z);
            Eigen::Vector2f point2d = B.transpose() * point3d;
            boundary_2D.push_back(point2d);
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_projected);
        extract.setIndices(inlier_indices);
        extract.filter(*plane_inliers);

        inliers -> clear();
        pcl::PointCloud<pcl::PointXYZ>::VectorType::iterator it; 
        for (it = plane_inliers -> begin(); it != plane_inliers -> end(); ++it) {
            pcl::PointXYZ p = *it; 
            if (pnpoly(p, boundary_2D, B)) {
                inliers -> push_back(p);
            }
        }

        //Eigen::VectorXf plane_coeff = Eigen::VectorXf::Map(model_coefficients[i].values.data(), 4);
        //Eigen::VectorXf new_plane;
        //optimize_model -> optimizeModelCoefficients(inliers, plane_coeff, new_plane);
        //plane = new_plane;

    //}
}


void
CropPlaneInliers::print()
{
    std::cout << plane << std::endl;
    std::cout << origin << std::endl;
}
