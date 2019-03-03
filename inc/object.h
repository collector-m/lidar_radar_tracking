#ifndef OBJECT_H
#define OBJECT_H

#include "global.h"
#include "utils.h"

const float box_object_len = 4;
const float box_object_wid = 1.5;
const float box_object_len_large = 8;
const float box_object_wid_large = 2.5;

struct Object
{
    uint64_t id;
};

struct BoxObject : Object
{
    float rx;
    float ry;
    float vx;
    float vy;
    float ax;
    float ay;

    float rx_cov;
    float ry_cov;
    float vx_cov;
    float vy_cov;

    cv::Point2f corner[4];  // left-top, left-down, right-down, right-top in 2d
    Eigen::Quaternionf yaw;
};

struct RadarObject : Object
{
    float r;
    float theta;
    float vt;
};

struct LidarPoint
{
    float rx;
    float ry;
};

#endif // OBJECT_H
