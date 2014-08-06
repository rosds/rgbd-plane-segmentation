#include <Transform3D.hpp>

Transform3D::Transform3D(float tx, float ty, float tz, float qx, float qy, float qz, float qw)
{
    rotation = Eigen::Quaternion<float>(qw, qx, qy, qz);
    offset = Eigen::Vector3f();
    offset[0] = tx;
    offset[1] = ty;
    offset[2] = tz;
    transform = Eigen::Transform<float, 3, Eigen::Affine>();
    transform.setIdentity();
    transform.translate(offset);
    transform.rotate(rotation);
}
