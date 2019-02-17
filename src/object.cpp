#include "../inc/object.h"

Object::Object()
{
    // do nothing
}

BoxObject::BoxObject()
{
    // do nothing
}

RadarObject::RadarObject()
{
    // do nothing
}

void BoxObject::PrintObjInfo()
{
    printf("[id: %ld][pos]: rx %f, ry %f\n",
           id, rx, ry);
    printf("[id: %ld][pos_cov]: rx_cov %f, ry_cov %f\n",
           id, rx_cov, ry_cov);
    printf("[id: %ld][vel]: vx %f, vy %f\n",
           id, vx, vy);
    printf("[id: %ld][vel_cov]: vx_cov %f, vy_cov %f\n",
           id, vx_cov, vy_cov);
    printf("[id: %ld][acc]: ax %f, ay %f\n",
           id, ax, ay);
    printf("[id: %ld][size]: l %f, w %f\n",
           id, (corner[0].x - corner[1].x)/2, (corner[1].y - corner[2].y)/2);
    printf("[id: %ld][yaw(q)]: w %f, x %f, y %f, z %f\n",
           id, yaw.w(), yaw.x(), yaw.y(), yaw.z());
    printf("[id: %ld][yaw(deg)]: %f\n",
           id, rad2deg(rot2yaw(yaw)));
}

void RadarObject::PrintObjInfo()
{
    printf("[id: %ld][range: %f][theta(deg): %f][velocity: %f]\n",
           id, r, rad2deg(theta), vt);
}



