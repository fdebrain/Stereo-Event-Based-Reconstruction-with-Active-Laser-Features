#include <EBV_DFF_Visualizer.h>
#include <EBV_DVS128USB.h>
#include <EBV_DAVIS240C.h>
#include <EBV_Filter.h>
#include <EBV_Matcher.h>
#include <EBV_Triangulator.h>
#include <EBV_LaserController.h>
#include <EBV_Stereo_Calibration.h>

int main(int argc, char** argv)
{
    DAVIS240C    davis_master;
    DAVIS240C    davis_slave;

    AdaptiveFilter       filter0(&davis_master);
    AdaptiveFilter       filter1(&davis_slave);
    Matcher      matcher(&filter0,&filter1);
    LaserController laser;
    Triangulator triangulator(&matcher,
                              &laser);
    StereoCalibrator calibrator(&laser,
                                &filter0,
                                &filter1,
                                &triangulator);
    Visualizer   visu(davis_master.m_nbCams,
                      &davis_master, &davis_slave,
                      &filter0, &filter1,
                      &calibrator,
                      &triangulator,
                      &laser);

    visu.run();

    return 0;
}
