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

#include <libcaercpp/devices/davis.hpp>

namespace cv {class Mat;}

static std::atomic_bool globalShutdown(false);

struct DAVIS240CEvent
{
public:
    DAVIS240CEvent()
        : m_x(0),
          m_y(0),
          m_pol(0),
          m_timestamp(0)
    {}

    DAVIS240CEvent(int x, int y, bool pol, int timestamp)
        : m_x(x),
          m_y(y),
          m_pol(pol),
          m_timestamp(timestamp)
    {}

    int m_x;
    int m_y;
    bool m_pol;
    int m_timestamp;
};

struct DAVIS240CFrame
{
public:
    DAVIS240CFrame(): m_frame(240*180), m_timestamp(0)
    {
    }

    DAVIS240CFrame(const std::vector<uchar> frame,
                   const int timestamp)
        : m_frame(frame), m_timestamp(timestamp)
    {
        //m_frame.resize(240*180);
    }

     std::vector<uchar> m_frame;
     int m_timestamp;
};

class DAVIS240CEventListener
{
public:
    DAVIS240CEventListener(void) {}
    virtual void receivedNewDAVIS240CEvent(DAVIS240CEvent& event,
                                           const uint id) = 0;
};

class DAVIS240CFrameListener
{
public:
    DAVIS240CFrameListener(void) {}
    virtual void receivedNewDAVIS240CFrame(DAVIS240CFrame& frame,
                                           const uint id) = 0;
};

class DAVIS240C
{
public:
    DAVIS240C();
    ~DAVIS240C();

    static uint m_nbCams;
    static libcaer::devices::davis* m_davisMasterHandle;

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
    // Id of the camera
    const uint m_id;

    // Device resolution
    const int m_rows;
    const int m_cols;

private:
    //Device handle
    libcaer::devices::davis m_davisHandle;

    // Reading thread related variables
    std::thread m_readThread;
    bool m_stopreadThread;

    // Registered listeners
    std::list<DAVIS240CEventListener*> m_eventListeners;
    std::list<DAVIS240CFrameListener*> m_frameListeners;
};

#endif // EBV_DAVIS240C_H
