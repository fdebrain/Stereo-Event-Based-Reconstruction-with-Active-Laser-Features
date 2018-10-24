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
    Filter       filter0(rows,cols,0);
    Filter       filter1(rows,cols,1);
    Matcher      matcher(rows,cols,&filter0,&filter1);
    Triangulator triangulator(rows,cols,&matcher);
    Visualizer   visu(rows,cols,davis_master.m_nbCams,
                      &filter0, &filter1);

    // Open Master
    davis_master.init();
    davis_master.start();
    davis_master.registerEventListener(&visu);
    davis_master.registerFrameListener(&visu);
    davis_master.registerEventListener(&filter0);
    filter0.registerFilterListener(&visu);
    filter0.registerFilterListener(&matcher);
    davis_master.listen();

    // Open Slave
    davis_slave.init();
    davis_slave.start();
    davis_slave.registerEventListener(&visu);
    davis_slave.registerFrameListener(&visu);
    davis_slave.registerEventListener(&filter1);
    filter1.registerFilterListener(&visu);
    filter1.registerFilterListener(&matcher);
    davis_slave.listen();

    // Visualize
    matcher.registerMatcherListener(&triangulator);
    triangulator.registerTriangulatorListener(&visu);
    visu.run();
    matcher.deregisterMatcherListener(&triangulator);
    triangulator.deregisterTriangulatorListener(&visu);

    // Close Slave
    davis_slave.stopListening();
    davis_slave.deregisterEventListener(&visu);
    davis_slave.deregisterFrameListener(&visu);
    davis_slave.deregisterEventListener(&filter1);
    filter1.deregisterFilterListener(&visu);
    filter1.deregisterFilterListener(&matcher);
    davis_slave.stop();
    davis_slave.close();

    // Close Master
    davis_master.stopListening();
    davis_master.deregisterEventListener(&visu);
    davis_master.deregisterFrameListener(&visu);
    davis_master.deregisterEventListener(&filter0);
    filter0.deregisterFilterListener(&visu);
    filter0.deregisterFilterListener(&matcher);
    davis_master.stop();
    davis_master.close();

    return 0;
}
