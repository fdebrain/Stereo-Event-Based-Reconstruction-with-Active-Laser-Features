#include <EBV_DFF_Visualizer.h>
#include <EBV_DVS128USB.h>
#include <EBV_DAVIS240C.h>
#include <EBV_Filter.h>
#include <EBV_Matcher.h>
#include <EBV_Triangulator.h>

int main(int argc, char** argv)
{
    // Resolution
    int rows = 180;
    int cols = 240;

    DAVIS240C    davis_master;
    DAVIS240C    davis_slave;

    Filter       filter0(rows,cols,&davis_master);
    Filter       filter1(rows,cols,&davis_slave);
    Matcher      matcher(rows,cols,&filter0,&filter1);
    Triangulator triangulator(rows,cols,&matcher);
    Visualizer   visu(rows,cols,davis_master.m_nbCams,
                      &davis_master, &davis_slave,
                      &filter0, &filter1,
                      &triangulator);

    // Open Master
    davis_master.init();
    davis_master.start();
    davis_master.listen();

    // Open Slave
    davis_slave.init();
    davis_slave.start();
    davis_slave.listen();

    // Visualize
    visu.run();

    // Close Slave
    davis_slave.stopListening();
    davis_slave.stop();
    davis_slave.close();

    // Close Master
    davis_master.stopListening();
    davis_master.stop();
    davis_master.close();

    return 0;
}
