#ifndef TRANSFORM3D_HH
#define TRANSFORM3D_HH

#include <Eigen/Geometry>

class Transform3D 
{
    public:
        Transform3D() {}
        Transform3D(float tx, float ty, float tz, float qx, float qy, float qz, float qw);
        ~Transform3D() {}
        Eigen::Transform<float, 3, Eigen::Affine> transform;
        Eigen::Vector3f offset;
        Eigen::Quaternion<float> rotation;

    private:
};

#endif
