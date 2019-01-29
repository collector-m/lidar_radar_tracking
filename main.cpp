#include "inc/global.h"
#include "inc/objectsimulator.h"
#include "inc/visualizer.h"

int main()
{
    uint64_t frame_cnt = 0;
    ObjectSimulator sim;
    Visualizer viser;
    std::vector<BoxObject> gt;
    std::vector<RadarObject> radarobjs;
    while (frame_cnt++ < 60 / update_period)
    {
        sim.GenerateGT(gt);
        sim.GenerateRadarObsv(gt, radarobjs);

        viser.Init();
        viser.DrawGT(gt);
        viser.DrawRadarObjs(radarobjs);

        viser.ShowMap();
        usleep(50000);
    }

    return 0;
}
