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

    DAVIS240CEvent(const unsigned int x, const unsigned int y,
                   const unsigned int pol,
                   const int timestamp)
        : m_x(x),
          m_y(y),
          m_pol(pol),
          m_timestamp(timestamp)
    {}

    unsigned int m_x;
    unsigned int m_y;
    unsigned int m_pol;
    int m_timestamp;
};

struct DAVIS240CFrame
{
public:
    DAVIS240CFrame()
        : m_frame(0),
          m_timestamp(0)
    {
        m_frame.resize(240*180);
    }

    DAVIS240CFrame(const std::vector<unsigned char> frame,
                   const int timestamp)
        : m_frame(frame),
          m_timestamp(timestamp)
    {
        m_frame.resize(240*180);
    }

     std::vector<unsigned char> m_frame;
     int m_timestamp;
};

class DAVIS240CEventListener
{
    public:
        DAVIS240CEventListener(void) {}
        virtual void receivedNewDAVIS240CEvent(DAVIS240CEvent& event,
                                               const unsigned int id) = 0;
};

class DAVIS240CFrameListener
{
    public:
        DAVIS240CFrameListener(void) {}
        virtual void receivedNewDAVIS240CFrame(DAVIS240CFrame& frame,
                                               const unsigned int id) = 0;
};

class DAVIS240C
{
public:
    DAVIS240C();
    ~DAVIS240C();

    static unsigned int m_nbCams;
    static libcaer::devices::davis* m_davisMasterHandle;

    // Life cycle of a DAVIS240C
    void resetMasterClock();
    int init();
    int start();
    int stop();

    // Life cycle - events listening
    void registerEventListener(DAVIS240CEventListener* listener);
    int listenEvents();
    void readThreadEvents();
    void warnEvent(std::vector<DAVIS240CEvent>& events);
    int stopListeningEvents();
    void deregisterEventListener(DAVIS240CEventListener* listener);

    // Life cycle - frames listening
    void registerFrameListener(DAVIS240CFrameListener* listener);
    int listenFrames();
    void readThreadFrames();
    void warnFrame(DAVIS240CFrame& frame);
    int stopListeningFrames();
    void deregisterFrameListener(DAVIS240CFrameListener* listener);

public:
    // Id of the camera
    const unsigned int m_id;

private:
    // Device resolution
    const unsigned int m_rows;
    const unsigned int m_cols;

    //Device handle
    libcaer::devices::davis m_davisHandle;

    // Reading thread related variables
    std::thread m_readThreadEvents;
    std::thread m_readThreadFrames;

    bool m_stopreadThreadEvents;
    bool m_stopreadThreadFrames;

    std::mutex m_lockerEvent;
    std::mutex m_lockerFrame;

    // Registered listeners
    std::list<DAVIS240CEventListener*> m_eventListeners;
    std::list<DAVIS240CFrameListener*> m_frameListeners;
};

#endif // EBV_DAVIS240C_H
