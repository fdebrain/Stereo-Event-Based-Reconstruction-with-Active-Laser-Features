#include <EBV_DFF_Visualizer.h>
#include <EBV_DVS128USB.h>
#include <EBV_DAVIS240C.h>

int main(int argc, char** argv)
{
    DAVIS240C    davis_master;
    DAVIS240C    davis_slave;
    //DVS128USB    davis_master;
    Visualizer   visu_master(180,240,0);
    //Visualizer   visu_slave(180,240,1);

    // Open Master
    davis_master.init();
    davis_master.start();
    davis_master.registerEventListener(&visu_master);
    davis_master.registerFrameListener(&visu_master);
    davis_master.listen();

    // Open Slave
    /*
    davis_slave.init();
    davis_slave.start();
    davis_slave.registerEventListener(&visu_slave);
    //davis_slave.registerFrameListener(&visu);
    davis_slave.listen();
    */

    visu_master.run();
    /*
    visu_slave.run();

    // Close Slave
    davis_slave.stopListening();
    davis_slave.deregisterEventListener(&visu_slave);
    //davis_master.deregisterFrameListener(&visu);
    davis_slave.stop();
    davis_slave.close();
    */

    // Close Master
    davis_master.stopListening();
    davis_master.deregisterEventListener(&visu_master);
    davis_master.deregisterFrameListener(&visu_master);
    davis_master.stop();
    davis_master.close();

    return 0;
}
