#ifndef EBV_DVS128USB_H
#define EBV_DVS128USB_H

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <string>
#include <list>
#include <map>
#include <vector>
#include <pthread.h>

struct libusb_device_handle;
struct libusb_device;
struct libusb_device_descriptor;
struct libusb_config_descriptor;

class DVS128USB;

struct DVS128USBEvent
{
public:
    DVS128USBEvent()
        : m_x(0),
          m_y(0),
          m_pol(0),
          m_timestamp(0),
          m_rawdata()
    {}

    DVS128USBEvent(const unsigned int x, const unsigned int y,
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
    char m_rawdata[8];
};

struct DVS128USBBiases
{
    DVS128USBBiases()
       : cas(1992),
         injGnd(1108364),
         reqPd(16777215),
         puX(8159221),
         diffOff(132),
         req(309590),
         refr(969),
         puY(16777215),
         diffOn(209996),
         diff(13125),
         foll(271),
         pr(217)
    {}

    int cas;
    int injGnd;
    int reqPd;
    int puX;
    int diffOff;
    int req;
    int refr;
    int puY;
    int diffOn;
    int diff;
    int foll;
    int pr;
};

class DVS128USBListener
{
    public:
        DVS128USBListener(void) {}
        virtual void receivedNewDVS128USBEvent(DVS128USBEvent& event) = 0;
};

class DVS128USBSpecListener
{
    public:
        DVS128USBSpecListener(void) {}
        virtual void receivedNewDVS128USBSpecEvent(unsigned int ts) = 0;
};


class DVS128USBStatusListener
{
    public:
        DVS128USBStatusListener(void) {}
        virtual void receivedNewDVS128USBStatus(std::string& string) = 0;
};

struct DVS128USBArgs
{
    struct libusb_device_handle*    devh;
    DVS128USB*                      device;
    int*                            threadStatus;
    std::list<DVS128USBEvent>*      events;
};

class DVS128USB
{
public:
    static const int TERMINATE_THREAD = -1;
    static const int RUN_THREAD = 1;

    DVS128USB(void);
    ~DVS128USB(void);

    // Discover all the devices plugged to the USB (for debug or curiosity)
    int discover(void);

    // Step by step life of a DVS128USB object
    int init(void);
    int start(void);
    int stop(void);
    int listen(void);
    int stopListening(void);

    int close(void);

    // If you need it, resets the USB driver
    int reset(void);

    // Conbfigure the biases
    int sendBiases(DVS128USBBiases b = DVS128USBBiases());
    DVS128USBBiases getBiases();

    // Register a listener to receive the new events
    void registerListener(DVS128USBListener* listener);
    void deregisterListener(DVS128USBListener* listener);

    // Warn all the listeners that an event has occured
    void warnEvent(std::vector<DVS128USBEvent>& events);

    // Register a listener to receive the new special events
    void registerSpecListener(DVS128USBSpecListener* listener);
    void deregisterSpecListener(DVS128USBSpecListener* listener);

    // Warn all the listeners that a special event has occured
    void warnSpecEvent(std::vector<unsigned int>& ts);

    // Register a listener to receive new status
    void registerStatusListener(DVS128USBStatusListener* listener);
    void deregisterStatusListener(DVS128USBStatusListener* listener);

    // Warn all the listeners that a status has been output
    void warnStatus(std::string status);

    // Getters & Setters
    int getStatus(void);
    int getNumBus(void);
    int getAddrBus(void);
    int getSpeed(void);

    void setMaxCtrlTransTimeout(unsigned int to);
    void setVerboseOutput(FILE* f);

private:
    void logg(std::string toLogg);

    int vendorRequest(libusb_device_handle* devh,
                      uint8_t bRequest,
                      uint16_t wValue,
                      uint16_t wIndex,
                      unsigned char *data,
                      uint16_t wLength);

    int findDevice(void);
    int initFailedWithInterface(void);
    int initFailed(void);

    // Object status
    int m_status;
    char m_buffStatus[512];
    bool m_init;
    bool m_interfaced;
    bool m_started;
    bool m_toggleListening;

    // Output for errors and info
    FILE* m_verboseOutput;

    // Device structures and info
    struct libusb_device_handle* m_devh;
    struct libusb_device* m_dev;
    struct libusb_config_descriptor* m_activeConfigDesc;
    struct libusb_device_descriptor*  m_devDesc;
    int m_devBusNum;
    int m_devBusAddr;
    int m_devSpeed;
    int m_activeConfig;

    // Threading
    pthread_t             m_readThread;
    int                   m_readThreadStatus;
    struct DVS128USBArgs  m_readThreadArgs;

    pthread_t             m_warnThread;
    int                   m_warnThreadStatus;
    struct DVS128USBArgs  m_warnThreadArgs;

    std::list<DVS128USBEvent>   m_events;

    // Bias Configuration
    DVS128USBBiases m_biases;

    // User parameters
    unsigned int m_ctrlTransTimeout;

    // Registered listeners
    std::list<DVS128USBListener*> m_listeners;
    std::map<int,void*> m_eventData;

    // Registered special listeners
    std::list<DVS128USBSpecListener*> m_specListeners;

    // Registered status listeners
    std::list<DVS128USBStatusListener*> m_statusListeners;
};

#endif // EBV_DVS128USB_H
