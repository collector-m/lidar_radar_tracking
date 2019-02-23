#include "../inc/tracker.h"

template <class T>
static void erase_from_vector(std::vector<T> &v, int index)
{
    assert(index < v.size());
    v.erase(v.begin() + index);
}

Tracker::Tracker()
{
    id_cnt = 1;  // gid start from 1, so 0 means invalid or init status
    matched_pair.clear();
    prev_matched.clear();
    src_matched.clear();
    track_info.clear();
    X.clear();
    P.clear();
}

void Tracker::PrintMeasurements(std::vector<RadarObject> &src)
{
    printf("[MeasurementInfo: num %d]\n", src.size());
    for (auto &obj : src)
    {
        obj.PrintObjInfo();
    }
}

void Tracker::PrintOutputTracks(std::vector<BoxObject> &dst)
{
    printf("[OutputInfo: num %d]\n", X.size());
    for (int i=0; i<X.size(); ++i)
    {
        if (track_info[i].second.exist_cnt < min_exist_cnt)  continue;
        if (!IsConverged(i))  continue;

        BoxObject obj;
        obj.id = track_info[i].first;

        obj.rx = X[i](0);
        obj.ry = X[i](1);
        obj.vx = X[i](2);
        obj.vy = X[i](3);
        obj.ax = obj.ay = 0;

        obj.rx_cov = P[i](0,0);
        obj.ry_cov = P[i](1,1);
        obj.vx_cov = P[i](2,2);
        obj.vy_cov = P[i](3,3);

        obj.corner[0] = cv::Point2f(box_object_len/2, -box_object_wid/2);
        obj.corner[1] = cv::Point2f(-box_object_len/2, -box_object_wid/2);
        obj.corner[2] = cv::Point2f(-box_object_len/2, box_object_wid/2);
        obj.corner[3] = cv::Point2f(box_object_len/2, box_object_wid/2);

        Eigen::Vector3f v_be(1,0,0);
        Eigen::Vector3f v_en(obj.vx, obj.vy, 0);
        obj.yaw.setFromTwoVectors(v_be, v_en);
        obj.yaw = obj.yaw.normalized();

        printf("[gid: %ld][trackinfo]: loss_cnt %d, exist_cnt %d\n",
               obj.id, track_info[i].second.loss_cnt, track_info[i].second.exist_cnt);
        printf("[gid: %ld][pos]: rx %f, ry %f\n",
               obj.id, obj.rx, obj.ry);
        printf("[gid: %ld][pos_cov]: rx_cov %f, ry_cov %f\n",
               obj.id, obj.rx_cov, obj.ry_cov);
        printf("[gid: %ld][vel]: vx %f, vy %f\n",
               obj.id, obj.vx, obj.vy);
        printf("[gid: %ld][vel_cov]: vx_cov %f, vy_cov %f\n",
               obj.id, obj.vx_cov, obj.vy_cov);
        printf("[gid: %ld][size]: l %f, w %f\n",
               obj.id, (obj.corner[0].x - obj.corner[1].x)/2, (obj.corner[1].y - obj.corner[2].y)/2);
        printf("[gid: %ld][yaw(q)]: w %f, x %f, y %f, z %f\n",
               obj.id, obj.yaw.w(), obj.yaw.x(), obj.yaw.y(), obj.yaw.z());
        printf("[gid: %ld][yaw(deg)]: %f\n",
               obj.id, rad2deg(rot2yaw(obj.yaw)));

        dst.push_back(obj);
    }
}

void Tracker::MatchNN(std::vector<RadarObject> &src)
{
    int prev_track_num = X.size();
    int src_obj_num = src.size();
    printf("[MatchInfo] prev_track %d, src_num %d\n", prev_track_num, src_obj_num);

    matched_pair.clear();
    src_matched.clear();
    src_matched.resize(src_obj_num, 0);
    prev_matched.clear();
    prev_matched.resize(prev_track_num, 0);

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

        for (int j=0; j<prev_track_num; ++j)
        {
            float rx = X[j](0);
            float ry = X[j](1);
            float vx = X[j](2);
            float vy = X[j](3);
            float r_ = sqrt(rx * rx + ry * ry);
            float theta_ = atan2(ry, rx);
            float vt_ = (vx * rx + vy * ry) / r_;

//            if (fabs(r*cos(theta) - rx) < rx * 0.3
//                && fabs(r*sin(theta) - ry) < ry * 0.3
//                && fabs(vt - vt_) < 10)

            printf("[MatchNN] diff btw src %d -> prev %d: diff_r %f, diff_theta %f, diff_vt %f\n",
                   i, j, fabs(r - r_), fabs(theta - theta_), fabs(vt - vt_));

            float last_rx = rx - vx * ts;
            float last_ry = ry - vy * ts;
            float auto_theta_thres = (fabs(atan2(ry, rx) - atan2(last_ry, last_rx)) + 5) * 180 / pi;
            auto_theta_thres = std::min(auto_theta_thres, (float)10.0);
//            printf("debug_theta_threshold(deg): %f, gid %ld, rx %f, ry %f\n", auto_theta_thres, track_info[j].first, rx, ry);

            if (fabs(r - r_) < r * 0.3
                && fabs(theta - theta_) < auto_theta_thres * pi / 180
                && fabs(vt - vt_) < 10)
            {
                float tmp_cost = pow((r-r_)/r_, 2) + pow((theta-theta_)/theta_, 2) + pow((vt-vt_)/vt_, 2);
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

                track_info[match_id].second.loss_cnt = 0;
                track_info[match_id].second.exist_cnt++;

                printf("[MatchNN] matched: src %d -> prev %d, with r %f -> %f, theta %f -> %f, vt %f -> %f, cost %f\n",
                       i, match_id, r, matched_r, theta, matched_theta, vt, matched_vt, cost);
            }
            else
            {
                int new_match_id = X.size();
                Eigen::VectorXf X_copy(X[match_id]);
                Eigen::MatrixXf P_copy(P[match_id]);
                X.push_back(X_copy);
                P.push_back(P_copy);

//                std::pair<uint64_t, TrackCount> track_info_copy(track_info[match_id]);
//                track_info_copy.first = id_cnt++;

                std::pair<uint64_t, TrackCount> track_info_copy(id_cnt++, {0,1});
                track_info.push_back(track_info_copy);

                assert(X.size() == P.size());
                assert(X.size() == track_info.size());

                std::pair<int, int> assignment(i, new_match_id);
                matched_pair.push_back(assignment);
                prev_matched.push_back(1);
                src_matched[i] = 1;

                printf("[MatchNN] repeated matched: src %d -> prev %d, with r %f -> %f, theta %f -> %f, vt %f -> %f, cost %f\n",
                       i, match_id, r, matched_r, theta, matched_theta, vt, matched_vt, cost);
            }
        }
    }

    prev_track_num = X.size();
    src_obj_num = src.size();

    int unmatched_track_num = 0;
    for (int i=0; i<prev_track_num; ++i)
    {
        if (!prev_matched[i])
        {
            unmatched_track_num++;
            track_info[i].second.loss_cnt++;
//            track_info[i].second.exist_cnt = 0;

            float rx = X[i](0);
            float ry = X[i](1);
            float vx = X[i](2);
            float vy = X[i](3);
            float r_ = sqrt(rx * rx + ry * ry);
            float theta_ = atan2(ry, rx);
            float vt_ = (vx * rx + vy * ry) / r_;
            printf("[MatchNN] prev not matched %d: r %f, theta %f, vt %f\n",
                   i, r_, theta_, vt_);
//            printf("[MatchNN] prev not matched %d: rx %f, ry %f, vx %f, vy %f\n",
//                   i, rx, ry, vx, vy);
        }
    }
    int unmatched_obj_num = 0;
    for (int i=0; i<src_obj_num; ++i)
    {
        if (!src_matched[i])
        {
            unmatched_obj_num++;
            float r = src[i].r;
            float theta = src[i].theta;
            float vt = src[i].vt;
            printf("[MatchNN] src not matched %d: r %f, theta %f, vt %f\n",
                   i, r, theta, vt);
        }
    }

    printf("[MatchNN][matched_track: %d][unmatched_track: %d][unmatched_obj: %d]\n",
           matched_pair.size(), unmatched_track_num, unmatched_obj_num);
}

void Tracker::InitTrack(const RadarObject &obj)
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

    std::pair<uint64_t, TrackCount> initial_info(id_cnt++, {0,1});
    track_info.push_back(initial_info);

    assert(X.size() == P.size());
    assert(X.size() == track_info.size());
}

void Tracker::RemoveTrack(int index)
{
    erase_from_vector(X, index);
    erase_from_vector(P, index);
    erase_from_vector(track_info, index);
}

void Tracker::Predict()
{
    assert(X.size() == P.size());
    int prev_track_num = X.size();
    for (int i=0; i<prev_track_num; ++i)
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
}

void Tracker::Update(std::vector<RadarObject> &src)
{
    assert(X.size() == P.size());

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
    int prev_track_num = X.size();
    for (int i=prev_track_num-1; i>=0; --i)  // from back to front to avoid wrongly removing
    {
        if (!prev_matched[i])
        {
            if (track_info[i].second.loss_cnt > max_loss_cnt
                || !IsConverged(i))
            {
                RemoveTrack(i);
            }
        }
    }

    // c. upgrade src unmatched
    int src_obj_num = src.size();
    for (int i=0; i<src_obj_num; ++i)
    {
        if (!src_matched[i])
        {
            InitTrack(src[i]);
        }
    }
}

void Tracker::EKF(std::vector<RadarObject> &src,
                  std::vector<BoxObject> &dst)
{
    PrintMeasurements(src);
    dst.clear();

    if (!X.size())
    {
        printf("[EKF] X empty!\n");
        for (auto &obj : src)
        {
            InitTrack(obj);
        }
    }
    else
    {
        Predict();
        MatchNN(src);
        Update(src);
        PrintOutputTracks(dst);
    }
//    printf("[EKF][summary] X size %d, P size %d, matched_pair size %d, prev_matched size %d, src_matched size %d, track_info size %d\n",
//           X.size(), P.size(), matched_pair.size(), prev_matched.size(), src_matched.size(), track_info.size());
}

bool Tracker::IsConverged(int track_index)
{
    bool converged = 0;
    float rx_cov = P[track_index](0,0);
    float ry_cov = P[track_index](1,1);
    float vx_cov = P[track_index](2,2);
    float vy_cov = P[track_index](3,3);
    if (rx_cov < 25
        && ry_cov < 25
        && vx_cov < 5
        && vy_cov < 5)
    {
        converged = 1;
    }
    return converged;
}
