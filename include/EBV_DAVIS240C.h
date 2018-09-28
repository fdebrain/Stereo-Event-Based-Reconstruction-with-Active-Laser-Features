#ifndef EBV_DAVIS240C_H
#define EBV_DAVIS240C_H

#include <iostream>
#include <string>
#include <list>
#include <atomic>
#include <csignal>
#include <thread>
#include <vector>

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
                   const int pol,
                   const unsigned long timestamp)
        : m_x(x),
          m_y(y),
          m_pol(pol),
          m_timestamp(timestamp)
    {}

    unsigned int m_x;
    unsigned int m_y;
    int m_pol;
    unsigned int m_timestamp;
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

    /*
    DAVIS240CFrame(const cv::Mat frame,
                   const int timestamp)
        : m_frame(frame),
          m_timestamp(timestamp)
    {
        m_frame.resize(240*180);
    }
    */

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


class DAVIS240CListener
{
    public:
        DAVIS240CListener(void) {}
        virtual void receivedNewDAVIS240CEvent(DAVIS240CEvent& event) = 0;
        virtual void receivedNewDAVIS240CFrame(DAVIS240CFrame& frame) = 0;
};

class DAVIS240C
{
public:
    DAVIS240C();
    ~DAVIS240C();

    // Life cycle of a DAVIS240C
    int init();
    int start();
    int listen();

    int stopListening();
    int stop();
    int close();

    // Reading thread & related stuff
    void readThread();
    void warnEvent(std::vector<DAVIS240CEvent>& events);
    void warnFrame(DAVIS240CFrame& frame);

    // Register a listener to receive the new events
    void registerEventListener(DAVIS240CListener* listener);
    void deregisterEventListener(DAVIS240CListener* listener);

    // Register a listener to receive the new frames
    void registerFrameListener(DAVIS240CListener* listener);
    void deregisterFrameListener(DAVIS240CListener* listener);

private:
    //Device handle
    libcaer::devices::davis m_davisHandle;

    // Device resolution
    int m_rows = 180;
    int m_cols = 240;

    // Reading thread related variables
    std::thread m_readThread;
    bool m_stopreadThread;

    // Registered listeners
    std::list<DAVIS240CListener*> m_frameListeners;
    std::list<DAVIS240CListener*> m_eventListeners;
};

#endif // EBV_DAVIS240C_H
