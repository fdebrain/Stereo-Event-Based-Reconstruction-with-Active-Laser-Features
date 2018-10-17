#include <EBV_DFF_Visualizer.h>
#include <EBV_DVS128USB.h>
#include <EBV_DAVIS240C.h>
#include <EBV_Filter.h>

int main(int argc, char** argv)
{
    //DVS128USB    davis_master;
    DAVIS240C    davis_master;
    //DAVIS240C    davis_slave;
    Visualizer   visu(180,240,davis_master.m_nbCams);
    Filter       filter(180,240);
    visu.setFilter(&filter);


    // Open Master
    davis_master.init();
    davis_master.start();
    davis_master.registerEventListener(&visu);
    davis_master.registerFrameListener(&visu);
    //====
    davis_master.registerEventListener(&filter);
    filter.registerFilterListener(&visu);
    //====
    davis_master.listen();

    // Open Slave
    /*
    davis_slave.init();
    davis_slave.start();
    davis_slave.registerEventListener(&visu);
    davis_slave.registerFrameListener(&visu);
    davis_slave.listen();
    */

    // Visualize
    visu.run();

    /*
    // Close Slave
    davis_slave.stopListening();
    davis_slave.deregisterEventListener(&visu);
    davis_master.deregisterFrameListener(&visu);
    davis_slave.stop();
    davis_slave.close();
    */

    // Close Master
    davis_master.stopListening();
    davis_master.deregisterEventListener(&visu);
    davis_master.deregisterFrameListener(&visu);
    davis_master.stop();
    davis_master.close();

    return 0;
}
