#include "../inc/tracker.h"

template <class T>
static void erase_obj(std::vector<T> &v, int index)
{
    assert(index+1 <= v.size());
    v.erase(v.begin() + index);
}

Tracker::Tracker()
{
    id_cnt = 0;
    matched_pair.clear();
    prev_matched.clear();
    src_matched.clear();
}

void Tracker::match_nn(std::vector<RadarObject> &src)
{
    // TODO: several srcs matches prevs

    int prev_obj_num = X.size();
    int src_obj_num = src.size();
    printf("[da_nn] prev_num %d, src_num %d\n", prev_obj_num, src_obj_num);

    matched_pair.clear();
    prev_matched.clear();
    prev_matched.resize(prev_obj_num, 0);
    src_matched.clear();
    src_matched.resize(src_obj_num, 0);

    for (int i=0; i<src_obj_num; ++i)
    {
        RadarObject robj = src[i];
        float r = robj.r;
        float theta = robj.theta;
        float vt = robj.vt;

        bool find_match = 0;
        int match_id = -1;
        float cost = FLT_MAX;
        float matched_r, matched_theta, matched_vt;

        for (int j=0; j<prev_obj_num; ++j)
        {
            float rx = X[j](0);
            float ry = X[j](1);
            float vx = X[j](2);
            float vy = X[j](3);
            float r_ = sqrt(rx * rx + ry * ry);
            float theta_ = atan2(ry, rx);
            float vt_ = (vx * rx + vy * ry) / r_;

            if (fabs(r - r_) < r * 0.3
                && fabs(theta - theta_) < 5 * pi / 180
                && fabs(vt - vt_) < 10)
            {
                float tmp_cost = pow((r-r_)/r, 2) + pow((theta-theta_)/theta, 2) + pow((vt-vt_)/vt, 2);
                if (tmp_cost < cost)
                {
                    cost = tmp_cost;
                    find_match = 1;
                    match_id = j;
                    matched_r = r_;
                    matched_theta = theta_;
                    matched_vt = vt_;
                }
            }
        }

        if (find_match)
        {
            if (!prev_matched[match_id])
            {
                std::pair<int, int> assignment(i, match_id);
                matched_pair.push_back(assignment);
                prev_matched[match_id] = 1;
                src_matched[i] = 1;

                printf("[da_nn] matched: src %d -> prev %d, with r %f -> %f, theta %f -> %f, vt %f -> %f, cost %f\n",
                       i, match_id, r, matched_r, theta, matched_theta, vt, matched_vt, cost);
            }
            else
            {
                int new_match_id = X.size();
                Eigen::VectorXf X_copy(X[match_id]);
                Eigen::MatrixXf P_copy(P[match_id]);
                X.push_back(X_copy);
                P.push_back(P_copy);

                std::pair<int, int> assignment(i, new_match_id);
                matched_pair.push_back(assignment);
                prev_matched.push_back(1);
                src_matched[i] = 1;

                printf("[da_nn] repeated matched: src %d -> prev %d, with r %f -> %f, theta %f -> %f, vt %f -> %f, cost %f\n",
                       i, match_id, r, matched_r, theta, matched_theta, vt, matched_vt, cost);
            }
        }
    }

    prev_obj_num = X.size();
    src_obj_num = src.size();
    printf("[da_nn] after match, prev_num %d, src_num %d\n", prev_obj_num, src_obj_num);

    for (int i=0; i<prev_obj_num; ++i)
    {
        if (!prev_matched[i])
        {
            float rx = X[i](0);
            float ry = X[i](1);
            float vx = X[i](2);
            float vy = X[i](3);
            float r_ = sqrt(rx * rx + ry * ry);
            float theta_ = atan2(ry, rx);
            float vt_ = (vx * rx + vy * ry) / r_;
            printf("[da_nn] prev not matched %d: r %f, theta %f, vt %f\n",
                   i, r_, theta_, vt_);
        }
    }
    for (int i=0; i<src_obj_num; ++i)
    {
        if (!src_matched[i])
        {
            float r = src[i].r;
            float theta = src[i].theta;
            float vt = src[i].vt;
            printf("[da_nn] src not matched %d: r %f, theta %f, vt %f\n",
                   i, r, theta, vt);
        }
    }
}

void Tracker::init_obj(const RadarObject &obj)
{
    Eigen::VectorXf tmpX = Eigen::VectorXf::Zero(4);
    tmpX(0) = obj.r * cos(obj.theta);
    tmpX(1) = obj.r * sin(obj.theta);
    tmpX(2) = obj.vt / cos(obj.theta);
    tmpX(3) = velocity_lateral_xinit;
    X.push_back(tmpX);

    Eigen::MatrixXf tmpP = Eigen::MatrixXf::Zero(4,4);
    tmpP(0,0) = pow(range_accuracy_rate * obj.r + range_accuracy, 2);
    tmpP(1,1) = pow(theta_accuracy_rate * obj.theta + theta_accuracy * obj.r / 2, 2);
    tmpP(2,2) = pow(velocity_accuracy_rate * obj.vt + velocity_accuracy, 2);
    tmpP(3,3) = pow(velocity_lateral_pinit, 2);
    P.push_back(tmpP);
}

void Tracker::ekf(std::vector<RadarObject> &src,
                  std::vector<BoxObject> &dst)
{
    printf("[ekf] receive %d input\n", src.size());
    dst.clear();

    if (!X.size())
    {
        printf("[ekf] X empty!\n");
        for (auto &obj : src)
        {
            init_obj(obj);
        }
    }
    else
    {
        // predict
        assert(X.size() == P.size());
        int prev_obj_num = X.size();
        for (int i=0; i<prev_obj_num; ++i)
        {
            Eigen::MatrixXf F = Eigen::MatrixXf::Zero(4,4);
            F(0,0) = 1; F(0,2) = ts;
            F(1,1) = 1; F(1,3) = ts;
            F(2,2) = 1;
            F(3,3) = 1;

            Eigen::MatrixXf Q = Eigen::MatrixXf::Zero(4,4);
            Q(0,0) = pow(0.5 * max_acc * ts * ts, 2);
            Q(1,1) = pow(0.5 * max_acc * ts * ts, 2);
            Q(2,2) = pow(max_acc * ts, 2);
            Q(3,3) = pow(max_acc * ts, 2);

            X[i] = F * X[i];
            P[i] = F * P[i] * F.transpose() + Q;
        }

        // da
        match_nn(src);

        // upgrade
        // a. upgrade matched
        for (int i=0; i<matched_pair.size(); ++i)
        {
            int src_index = matched_pair[i].first;
            int prev_index = matched_pair[i].second;

            float rx = X[prev_index](0);
            float ry = X[prev_index](1);
            float vx = X[prev_index](2);
            float vy = X[prev_index](3);
            float r_ = sqrt(rx * rx + ry * ry);
            float theta_ = atan2(ry, rx);
            float vt_ = (vx * rx + vy * ry) / r_;

            float r = src[src_index].r;
            float theta = src[src_index].theta;
            float vt = src[src_index].vt;

            Eigen::Vector3f Y(r-r_, theta-theta_, vt-vt_);

            Eigen::MatrixXf H = Eigen::MatrixXf::Zero(3,4);
            H(0,0) = rx / r_;
            H(0,1) = ry / r_;
            H(1,0) = -ry / r_ / r_;
            H(1,1) = rx / r_ / r_;
            H(2,0) = -ry * (rx * vy - ry * vx) / r_ / r_ / r_;
            H(2,1) = rx * (rx * vy - ry * vx) / r_ / r_ / r_;
            H(2,2) = rx / r_;
            H(2,3) = ry / r_;

            Eigen::MatrixXf R = Eigen::MatrixXf::Zero(3,3);
            R(0,0) = pow(range_accuracy_rate * r + range_accuracy, 2);
            R(1,1) = pow(theta_accuracy_rate * theta + theta_accuracy, 2);
            R(2,2) = pow(velocity_accuracy_rate * vt + velocity_accuracy, 2);

            Eigen::MatrixXf S = H * P[prev_index] * H.transpose() + R;
            Eigen::MatrixXf K = P[prev_index] * H.transpose() * S.inverse();

            X[prev_index] = X[prev_index] + K * Y;
            P[prev_index] = (Eigen::MatrixXf::Identity(4,4) - K * H) * P[prev_index];
        }

        // b. upgrade prev unmatched
        // TODO: according to loss count
        assert(X.size() == P.size());
        prev_obj_num = X.size();
        for (int i=prev_obj_num-1; i>=0; --i)  // from back to front to avoid wrongly removing
        {
            if (!prev_matched[i])
            {
               erase_obj(X, i);
               erase_obj(P, i);
            }
        }

        // c. upgrade src unmatched
        // TODO: according to exist count
        int src_obj_num = src.size();
        for (int i=0; i<src_obj_num; ++i)
        {
            if (!src_matched[i])
            {
                init_obj(src[i]);
            }
        }

        // output
        assert(X.size() == P.size());
        for (int i=0; i<X.size(); ++i)
        {
            BoxObject obj;
            obj.rx = X[i](0);
            obj.ry = X[i](1);
            obj.vx = X[i](2);
            obj.vy = X[i](3);
            obj.ax = obj.ay = 0;

            obj.corner[0] = cv::Point2f(box_object_len/2, -box_object_wid/2);
            obj.corner[1] = cv::Point2f(-box_object_len/2, -box_object_wid/2);
            obj.corner[2] = cv::Point2f(-box_object_len/2, box_object_wid/2);
            obj.corner[3] = cv::Point2f(box_object_len/2, box_object_wid/2);

            Eigen::Vector3f v_be(1,0,0);
            Eigen::Vector3f v_en(obj.vx, obj.vy, 0);
            obj.yaw.setFromTwoVectors(v_be, v_en);
            obj.yaw = obj.yaw.normalized();

            dst.push_back(obj);
        }
    }
    printf("[ekf] filter %d output\n", dst.size());
}
