#include "inc/global.h"
#include "inc/objectsimulator.h"
#include "inc/visualizer.h"
#include "inc/tracker.h"
#include "inc/radarhelper.h"
#include "inc/pointcloudhelper.h"

int main()
{
    ObjectSimulator sim;
    Visualizer viser;
    Tracker radar_tracker;

    uint64_t frame_cnt = 0;  
    std::vector<BoxObject> gt;
    std::vector<RadarObject> radarobjs;
    std::vector<LidarPoint> lidarpts;
    std::vector<BoxObject> filtered_radarobjs;
    while (frame_cnt++ < 60 / update_period)
    {
        sim.GenerateGT(gt);
        sim.GenerateRadarObsv(gt, radarobjs);
        sim.GenerateLidarPts(gt, lidarpts);
        pointcloud_labelling(lidarpts);

        radar_tracker.EKF(radarobjs, filtered_radarobjs);
        anchor2center(filtered_radarobjs);

        viser.Init();
        viser.DrawGT(gt, cv::Scalar(0,255,0));
        viser.DrawRadarObjs(radarobjs, cv::Scalar(0,0,255));
        // viser.DrawLidarPts(lidarpts, cv::Scalar(0,255,255));
        viser.DrawFT(filtered_radarobjs, cv::Scalar(255,255,0));

        viser.ShowMap();

        usleep(50000);
    }

    return 0;
}
