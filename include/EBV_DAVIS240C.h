#ifndef EBV_DAVIS240C_H
#define EBV_DAVIS240C_H

#include <iostream>
#include <string>
#include <list>
#include <atomic>
#include <csignal>
#include <thread>
#include <vector>
#include <mutex>

#include <EBV_LaserController.h>

#include <libcaercpp/devices/davis.hpp>

namespace cv {class Mat;}

static std::atomic_bool globalShutdown(false);

struct DAVIS240CEvent {
public:
    DAVIS240CEvent()
        : m_x(0),
          m_y(0),
          m_pol(0),
          m_timestamp(0) {}

    DAVIS240CEvent(const int x, const int y, const bool pol,
                   const int timestamp)
        : m_x(x),
          m_y(y),
          m_pol(pol),
          m_timestamp(timestamp) {}

    DAVIS240CEvent(const int x, const int y, const bool pol,
                   const int timestamp, const int laser_x, const int laser_y)
        : m_x(x),
          m_y(y),
          m_pol(pol),
          m_timestamp(timestamp),
          m_laser_x(laser_x),
          m_laser_y(laser_y) {}

    int m_x;
    int m_y;
    bool m_pol;
    int m_timestamp;

    // For laser synchronization
    int m_laser_x{-1};
    int m_laser_y{-1};
};

struct DAVIS240CFrame {
public:
    DAVIS240CFrame()
        : m_frame(240*180),
          m_timestamp(0) {}

    DAVIS240CFrame(const std::vector<uchar> frame,
                   const int timestamp)
        : m_frame(frame),
          m_timestamp(timestamp) {}

     std::vector<uchar> m_frame;
     int m_timestamp;
};

class DAVIS240CEventListener {
public:
    DAVIS240CEventListener(void) {}
    virtual void receivedNewDAVIS240CEvent(DAVIS240CEvent& event,
                                           const int id) = 0;
};

class DAVIS240CFrameListener
{
public:
    DAVIS240CFrameListener(void) {}
    virtual void receivedNewDAVIS240CFrame(DAVIS240CFrame& frame,
                                           const int id) = 0;
};

class DAVIS240C
{
public:
    DAVIS240C();
    DAVIS240C(LaserController *laser);
    ~DAVIS240C();

    static int m_nbCams;
    static libcaer::devices::davis* m_davis_master_handle;

    // Life cycle of a DAVIS240C
    void resetMasterClock();
    int init();
    int start();
    int listen();
    int stopListening();
    void readThread();
    int stop();

    // Life cycle - events listening
    void registerEventListener(DAVIS240CEventListener* listener);
    void warnEvent(std::vector<DAVIS240CEvent>& events);
    void deregisterEventListener(DAVIS240CEventListener* listener);

    // Life cycle - frames listening
    void registerFrameListener(DAVIS240CFrameListener* listener);
    void warnFrame(DAVIS240CFrame& frame);
    void deregisterFrameListener(DAVIS240CFrameListener* listener);

public:
    // Camera settings
    const int m_rows{180};
    const int m_cols{240};
    const int m_id;

    // Who camera listens to
    LaserController *m_laser;

private:
    libcaer::devices::davis m_davis_handle;

    std::thread m_read_thread;
    bool m_stop_read_thread{true};

    // Registered listeners
    std::list<DAVIS240CEventListener*> m_event_listeners;
    std::list<DAVIS240CFrameListener*> m_frame_listeners;
};

#endif // EBV_DAVIS240C_H
