#ifndef OBJECTSIMULATOR_H
#define OBJECTSIMULATOR_H

#include "global.h"
#include "object.h"

const float update_period = 0.05;

const float generate_obj_rate = 0.05;  // debug
const float constant_velocity_rate = 0.6;
const float small_car_rate = 0.6;
const float constant_acc = 2;
const float max_velocity = 30;
const float max_range_x = 100;
const float max_range_y = 50;

const float radar_loss_rate = 0.1;  // debug
const float radar_range_noise_bias_rate = 0.05;
const float radar_range_noise_cov = 0.5;
const float radar_theta_noise_bias_rate = 0;
const float radar_theta_noise_cov = 1 * pi / 180;
const float radar_velocity_noise_bias_rate = 0;
const float radar_velocity_noise_cov = 2;


class ObjectSimulator
{
public:
    ObjectSimulator();

    void GenerateGT(std::vector<BoxObject> &v);
    void GenerateRadarObsv(std::vector<BoxObject> &gt, std::vector<RadarObject> &radarobjs);

private:
    uint64_t id_cnt;
    uint64_t id_cnt_radar;
};

#endif // OBJECTSIMULATOR_H
