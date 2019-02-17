#include "../inc/utils.h"

float rot2yaw(Eigen::Quaternionf yaw)
{
    Eigen::Vector3f dst_vec = yaw * Eigen::Vector3f(1, 0, 0);
    return atan2(dst_vec(1), dst_vec(0));
}

float rad2deg(float rad)
{
    return rad * 180 / pi;
}

float deg2rad(float deg)
{
    return deg * pi / 180;
}
