#ifndef EBV_MAGNETICMIRRORLASER_H
#define EBV_MAGNETICMIRRORLASER_H

#include <iostream>
#include <cstring>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

class MagneticMirrorLaser
{

public:
    MagneticMirrorLaser();
    ~MagneticMirrorLaser();

    // Life cycle
    int init(std::string device);
    void close(void);

    // Actions on the laser
    int toggle(bool toggle);
    int blink(unsigned int us);
    int pos(unsigned int x,unsigned int y);
    int vel(unsigned int x,unsigned int y);

private:
    int sendCommand(std::string cmd);

private:
    // File descriptors
    int m_ttyFd;

    // Device descriptors
    std::string m_device;
    struct termios m_oldStdio;
};

#endif //EBV_MAGNETICMIRRORLASER_H
