#include <EBV_DFF_Visualizer.h>
#include <EBV_DVS128USB.h>
#include <EBV_DAVIS240C.h>
#include <EBV_Filter.h>
#include <EBV_Matcher.h>
#include <EBV_Triangulator.h>
#include <EBV_LaserController.h>

int main(int argc, char** argv)
{
    // Resolution
    unsigned int rows = 180;
    unsigned int cols = 240;

    DAVIS240C    davis_master;
    DAVIS240C    davis_slave;
    Filter       filter0(rows,cols,&davis_master);
    Filter       filter1(rows,cols,&davis_slave);
    Matcher      matcher(rows,cols,&filter0,&filter1);
    LaserController laser;
    Triangulator triangulator(rows,cols,
                              &davis_master,
                              &davis_slave,
                              &matcher,&laser);
    Visualizer   visu(rows,cols,davis_master.m_nbCams,
                      &davis_master, &davis_slave,
                      &filter0, &filter1,
                      &triangulator,
                      &laser);

    visu.run();

    return 0;
}
