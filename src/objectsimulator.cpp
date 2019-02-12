#include "../inc/objectsimulator.h"

static void erase_obj(std::vector<BoxObject> &v, uint64_t id)
{
    for(auto iter=v.begin(); iter!=v.end(); )
    {
        if ((*iter).id == id)
            iter = v.erase(iter);
        else
            iter ++ ;
    }
}

ObjectSimulator::ObjectSimulator()
{
    id_cnt = 0;
}

void ObjectSimulator::GenerateGT(std::vector<BoxObject> &v)
{
    std::random_device e;
    std::uniform_real_distribution<double> u_double(0,1);

    // update old obj
    for (auto &obj : v)
    {
        bool constant_velocity = (u_double(e) < constant_velocity_rate);
        if (!constant_velocity)
        {
            bool positive_acc = (u_double(e) > 0.5);
            obj.ax = (positive_acc) ? constant_acc : -constant_acc;

            positive_acc = (u_double(e) > 0.5);
            obj.ay = (positive_acc) ? constant_acc : -constant_acc;
        }
        else
        {
            obj.ax = obj.ay = 0;
        }
        obj.vx += obj.ax * update_period;
        obj.vx = std::max(obj.vx, -max_velocity);
        obj.vx = std::min(obj.vx, max_velocity);

        obj.vy += obj.ay * update_period;
        obj.vy = std::max(obj.vy, -max_velocity);
        obj.vy = std::min(obj.vy, max_velocity);

        obj.rx += obj.vx * update_period + 0.5 * obj.ax * update_period * update_period;
        obj.ry += obj.vy * update_period + 0.5 * obj.ay * update_period * update_period;

        if (obj.rx < 0
            || obj.rx > max_range_x
            || fabs(obj.ry) > max_range_y)  // exceed roi
        {
            erase_obj(v, obj.id);
        }

        Eigen::Vector3f v_be(1,0,0);
        Eigen::Vector3f v_en(obj.vx, obj.vy, 0);
        obj.yaw.setFromTwoVectors(v_be, v_en);
        obj.yaw = obj.yaw.normalized();
    }

    // generate new obj (one obj)
    bool if_generate_new_obj = (u_double(e) < generate_obj_rate);
    if (if_generate_new_obj)
    {
        double where_to_generate = u_double(e);  // TODO: support generate from side
        BoxObject obj;
        obj.id = id_cnt++;

        if (where_to_generate < 0.6)  // from down
        {
            obj.rx = 0;
            obj.ry = u_double(e) * 100 - 50;  // -50~50 m
            obj.vx = u_double(e) * 20 + 10;  // 10~30 m/s
            obj.vy = u_double(e) * 10 - 5;  // -5~5 m/s
        }
        else if (where_to_generate < 0.8)  // from top
        {
            obj.rx = 100;
            obj.ry = u_double(e) * 100 - 50;  // -50~50 m
            obj.vx = u_double(e) * -20 - 10;  // -10~-30 m/s
            obj.vy = u_double(e) * 10 - 5;  // -5~5 m/s
        }
        else if (where_to_generate < 0.9)  // from left
        {
            obj.rx = u_double(e) * 100;  // 0~100 m
            obj.ry = -50;
            obj.vx = ((u_double(e) < 0.5) * 2 - 1) * (u_double(e) * 20 + 10);  // 10~30 m/s or -10~-30 m/s
            obj.vy = u_double(e) * 5;  // 0~5 m/s
        }
        else  // from right
        {
            obj.rx = u_double(e) * 100;  // 0~100 m
            obj.ry = 50;
            obj.vx = ((u_double(e) < 0.5) * 2 - 1) * (u_double(e) * 20 + 10);  // 10~30 m/s or -10~-30 m/s
            obj.vy = u_double(e) * -5;  // 0~-5 m/s
        }

        Eigen::Vector3f v_be(1,0,0);
        Eigen::Vector3f v_en(obj.vx, obj.vy, 0);
        obj.yaw.setFromTwoVectors(v_be, v_en);
        obj.yaw = obj.yaw.normalized();

        if (u_double(e) < 0.7)
        {
            obj.corner[0] = cv::Point2f(box_object_len/2, -box_object_wid/2);
            obj.corner[1] = cv::Point2f(-box_object_len/2, -box_object_wid/2);
            obj.corner[2] = cv::Point2f(-box_object_len/2, box_object_wid/2);
            obj.corner[3] = cv::Point2f(box_object_len/2, box_object_wid/2);
        }
        else
        {
            obj.corner[0] = cv::Point2f(box_object_len_large/2, -box_object_wid_large/2);
            obj.corner[1] = cv::Point2f(-box_object_len_large/2, -box_object_wid_large/2);
            obj.corner[2] = cv::Point2f(-box_object_len_large/2, box_object_wid_large/2);
            obj.corner[3] = cv::Point2f(box_object_len_large/2, box_object_wid_large/2);
        }

        v.push_back(obj);
    }

//    printf("gt num %d\n", v.size());
//    for (auto obj : v)
//    {
//        printf("id %ld, rx %f, ry %f, vx %f, vy %f, ax %f, ay %f\n",
//               obj.id, obj.rx, obj.ry, obj.vx, obj.vy, obj.ax, obj.ay);
//    }
}

static void search_min_dist_pt(const cv::Point2f &pt1, const cv::Point2f &pt2, float &min_dist, cv::Point2f &min_dist_pt)
{
    float x1 = pt1.x, y1 = pt1.y;
    float x2 = pt2.x, y2 = pt2.y;

//    std::cout << "be: " << pt1 << std::endl;
//    std::cout << "en: " << pt2 << std::endl;

    if (fabs((y1 - y2)) < 1e-6)
    {
        float x = std::min(x1, x2);
        float y = std::min(y1, y2);
        float tmp_dist = sqrt(pow(x, 2) + pow(y, 2));
        if (tmp_dist < min_dist)
        {
            min_dist = tmp_dist;
            min_dist_pt = cv::Point2f(x,y);
        }
    }
    else
    {
        float k = (y1-y2) / (x1-x2);
        float b = y1 - k*x1;
        for (float x = std::min(x1, x2); x <= std::max(x1, x2); x += 0.0001)
        {
            float y = k*x + b;
            float tmp_dist = sqrt(pow(x, 2) + pow(y, 2));
            if (tmp_dist < min_dist)
            {
                min_dist = tmp_dist;
                min_dist_pt = cv::Point2f(x,y);
            }
        }
//        printf("line params: k=%f, b=%f\n", k, b);
//        std::cout << "nn: " << min_dist_pt << std::endl;
    }

}

static cv::Point2f find_nearest_point(const BoxObject &obj)
{
    cv::Point2f rotated_corner[4];
    for (int i=0; i<4; ++i)
    {
        Eigen::Vector3f vec(obj.corner[i].x, obj.corner[i].y, 0);
        vec = obj.yaw * vec;  // rotate by z axis
        rotated_corner[i].x = vec(0) + obj.rx;  // offset
        rotated_corner[i].y = vec(1) + obj.ry;
    }

    float min_dist = FLT_MAX;
    cv::Point2f min_dist_pt(0,0);
    search_min_dist_pt(rotated_corner[0], rotated_corner[1], min_dist, min_dist_pt);
    search_min_dist_pt(rotated_corner[1], rotated_corner[2], min_dist, min_dist_pt);
    search_min_dist_pt(rotated_corner[2], rotated_corner[3], min_dist, min_dist_pt);
    search_min_dist_pt(rotated_corner[3], rotated_corner[0], min_dist, min_dist_pt);
    return min_dist_pt;
}

void ObjectSimulator::GenerateRadarObsv(std::vector<BoxObject> &gt, std::vector<RadarObject> &radarobjs)
{
    radarobjs.clear();

    for (auto obj : gt)
    {
        // track loss
        std::random_device e;
        std::uniform_real_distribution<double> radar_loss(0,1);
        if (radar_loss(e) < radar_loss_rate)  continue;

        RadarObject robj;
        robj.id = 0;  // invalid
        cv::Point2f ref_position = find_nearest_point(obj);
//        cv::Point2f ref_position(obj.rx, obj.ry);
        robj.r = sqrt(pow(ref_position.x, 2) + pow(ref_position.y, 2));
        robj.theta = atan2(ref_position.y, ref_position.x);
        robj.vt = (ref_position.x * obj.vx + ref_position.y * obj.vy) / robj.r;

        // add noise
        std::normal_distribution<double> r_noise_sign(0,1);

        int sign = (r_noise_sign(e) < 0) * 2 - 1;  // -1 or 1
        std::normal_distribution<double> r_noise(sign*radar_range_noise_bias_rate*robj.r, radar_range_noise_cov);
        robj.r += r_noise(e);

        sign = (r_noise_sign(e) < 0) * 2 - 1;  // -1 or 1
        std::normal_distribution<double> theta_noise(sign*radar_theta_noise_bias_rate*robj.theta, radar_theta_noise_cov);
        robj.theta += theta_noise(e);

        sign = (r_noise_sign(e) < 0) * 2 - 1;  // -1 or 1
        std::normal_distribution<double> vt_noise(sign*radar_velocity_noise_bias_rate*robj.vt, radar_velocity_noise_cov);
        robj.vt += vt_noise(e);

        radarobjs.push_back(robj);
    }
}
