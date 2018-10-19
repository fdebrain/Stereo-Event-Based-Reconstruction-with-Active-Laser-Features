#include <EBV_DFF_Visualizer.h>
#include <EBV_DVS128USB.h>
#include <EBV_DAVIS240C.h>
#include <EBV_Filter.h>

int main(int argc, char** argv)
{
    //DVS128USB    davis_master;
    DAVIS240C    davis_master;
    DAVIS240C    davis_slave;
    Filter       filter0(180,240,0);
    Filter       filter1(180,240,1);
    Visualizer   visu(180,240,davis_master.m_nbCams,
                      &filter0, &filter1);

    // Open Master
    davis_master.init();
    davis_master.start();
    davis_master.registerEventListener(&visu);
    davis_master.registerFrameListener(&visu);
    davis_master.registerEventListener(&filter0);
    filter0.registerFilterListener(&visu);
    davis_master.listen();

    // Open Slave
    davis_slave.init();
    davis_slave.start();
    davis_slave.registerEventListener(&visu);
    davis_slave.registerFrameListener(&visu);
    davis_slave.registerEventListener(&filter1);
    filter1.registerFilterListener(&visu);
    davis_slave.listen();

    // Visualize
    visu.run();

    // Close Slave
    davis_slave.stopListening();
    davis_slave.deregisterEventListener(&visu);
    davis_slave.deregisterFrameListener(&visu);
    davis_slave.deregisterEventListener(&filter1);
    filter1.deregisterFilterListener(&visu);
    davis_slave.stop();
    davis_slave.close();

    // Close Master
    davis_master.stopListening();
    davis_master.deregisterEventListener(&visu);
    davis_master.deregisterFrameListener(&visu);
    davis_master.deregisterEventListener(&filter0);
    filter0.deregisterFilterListener(&visu);
    davis_master.stop();
    davis_master.close();

    return 0;
}
