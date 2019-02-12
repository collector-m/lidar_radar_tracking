#ifndef TRACKER_H
#define TRACKER_H

#include "global.h"
#include "object.h"
#include "utils.h"

const float ts = 0.05;
const float max_acc = 2;
const float range_accuracy_rate = 0.05;
const float range_accuracy = 0.5;
const float theta_accuracy_rate = 0;
const float theta_accuracy = 1 * pi / 180;
const float velocity_accuracy_rate = 0;
const float velocity_accuracy = 2;
const float velocity_lateral_xinit = 0;
const float velocity_lateral_pinit = 30;

class Tracker
{
public:
    Tracker();

    void init_obj(const RadarObject &obj);
    void match_nn(std::vector<RadarObject> &src);
    void ekf(std::vector<RadarObject> &src,
             std::vector<BoxObject> &dst);

private:
    std::vector<Eigen::VectorXf> X;  // rx ry vx vy
    std::vector<Eigen::MatrixXf> P;

    uint64_t id_cnt;

    std::vector<std::pair<int, int>> matched_pair;
    std::vector<bool> prev_matched;
    std::vector<bool> src_matched;
};

#endif // TRACKER_H
