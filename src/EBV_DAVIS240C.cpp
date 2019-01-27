#include <EBV_DAVIS240C.h>

#include <stdio.h>
#include <stdint.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

const std::vector<std::string> explode(const std::string& s, const char& c)
{
    std::string buff{""};
    std::vector<std::string> v;

    for(auto n:s)
    {
        if(n != c) buff+=n; else
        if(n == c && buff != "") { v.push_back(buff); buff = ""; }
    }
    if(buff != "") v.push_back(buff);

    return v;
}

// Simply set the running flag to false on SIGTERM and SIGINT (CTRL+C) for global shutdown.
static void globalShutdownSignalHandler(int signal)
{
    if (signal == SIGTERM || signal == SIGINT) { globalShutdown.store(true); }
}

static void usbShutdownHandler(void *ptr)
{
    (void) (ptr); // UNUSED.
    globalShutdown.store(true);
}

int DAVIS240C::m_nbCams=0;
libcaer::devices::davis* DAVIS240C::m_davis_master_handle = nullptr;

DAVIS240C::DAVIS240C()
    // Open a DAVIS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
    : m_id(m_nbCams),
      m_davis_handle(libcaer::devices::davis(m_id))
{
    m_nbCams += 1;

    struct sigaction shutdownAction;

    shutdownAction.sa_handler = &globalShutdownSignalHandler;
    shutdownAction.sa_flags   = 0;
    sigemptyset(&shutdownAction.sa_mask);
    sigaddset(&shutdownAction.sa_mask, SIGTERM);
    sigaddset(&shutdownAction.sa_mask, SIGINT);

    if (sigaction(SIGTERM, &shutdownAction, nullptr) == -1) {
        libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
            "Failed to set signal handler for SIGTERM. Error: %d.", errno);
        return;
    }

    if (sigaction(SIGINT, &shutdownAction, nullptr) == -1) {
        libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
            "Failed to set signal handler for SIGINT. Error: %d.", errno);
        return;
    }
}

DAVIS240C::DAVIS240C(LaserController *laser)
    : DAVIS240C()
{
     m_laser = laser;
}

DAVIS240C::~DAVIS240C()
{
    m_nbCams -= 1;

    if (m_record)
    {
        m_recorder_davis.close();
        if (m_id==0) { m_recorder_laser.close(); }
    }
}


//=== INITIALIZING ===//
void DAVIS240C::resetMasterClock()
{
    if (m_davis_master_handle->infoGet().deviceIsMaster)
    {
        m_davis_master_handle->configSet(DAVIS_CONFIG_MUX,
                                         DAVIS_CONFIG_MUX_TIMESTAMP_RESET, 2);
        printf("Reset Master clock. \n\r");
    }
}

int DAVIS240C::init()
{
    // Let's take a look at the information we have on the device.
    struct caer_davis_info davis_info = m_davis_handle.infoGet();

    printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Logic: %d.\n\r",
           davis_info.deviceString, davis_info.deviceID,
           davis_info.deviceIsMaster, davis_info.dvsSizeX,
           davis_info.dvsSizeY, davis_info.logicVersion);

    // Send the default configuration before using the device.
    // No configuration is sent automatically!
    m_davis_handle.sendDefaultConfig();

    // Tweak some biases, to increase bandwidth in this case.
    struct caer_bias_coarsefine coarseFineBias;

    coarseFineBias.coarseValue        = 2;
    coarseFineBias.fineValue          = 116;
    coarseFineBias.enabled            = true;
    coarseFineBias.sexN               = false;
    coarseFineBias.typeNormal         = true;
    coarseFineBias.currentLevelNormal = true;

    m_davis_handle.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP,
                            caerBiasCoarseFineGenerate(coarseFineBias));

    coarseFineBias.coarseValue        = 1;
    coarseFineBias.fineValue          = 33;
    coarseFineBias.enabled            = true;
    coarseFineBias.sexN               = false;
    coarseFineBias.typeNormal         = true;
    coarseFineBias.currentLevelNormal = true;

    m_davis_handle.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP,
                            caerBiasCoarseFineGenerate(coarseFineBias));

//    m_davis_handle.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);
//    m_davis_handle.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER, false);
//    m_davis_handle.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE, false);
//    m_davis_handle.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, 4200);

    return 0;
}

int DAVIS240C::start()
{
    // Now let's get start getting some data from the device. We just loop in blocking mode,
    // no notification needed regarding new events. The shutdown notification, for example if
    // the device is disconnected, should be listened to.
    m_davis_handle.dataStart(nullptr, nullptr, nullptr, &usbShutdownHandler, nullptr);

    // Let's turn on blocking data-get mode to avoid wasting resources.
    m_davis_handle.configSet(CAER_HOST_CONFIG_DATAEXCHANGE,
                             CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

    // Reset master clock when new device is added
    if (m_id==0) { m_davis_master_handle = &m_davis_handle; }
    else { resetMasterClock(); }

    // Recorder
    if (m_record)
    {
        m_recorder_davis.open(m_eventRecordFileDavis);
        if (m_id==0) { m_recorder_laser.open(m_eventRecordFileLaser); }
    }

    return 0;
}

//=== EVENT LISTENING ===//
void DAVIS240C::registerEventListener(DAVIS240CEventListener* listener)
{
    m_event_listeners.push_back(listener);
}

int DAVIS240C::listen()
{
    m_stop_read_thread = false;
    m_read_thread = std::thread(&DAVIS240C::readThread,this);
    return 0;
}

int DAVIS240C::listenRecording()
{
    m_stop_read_thread = false;
    m_read_thread = std::thread(&DAVIS240C::readData,this);
    return 0;
}

void DAVIS240C::readThread()
{
    std::vector<DAVIS240CEvent> events;
    while (!globalShutdown.load(std::memory_order_relaxed) & !m_stop_read_thread)
    {
        std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = m_davis_handle.dataGet();

        // Skip if container empty
        if (packetContainer == nullptr) { continue; }

        for (auto &packet : *packetContainer)
        {
            if (packet == nullptr) { continue; }

            // Events
            if (packet->getEventType() == POLARITY_EVENT)
            {
                std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity
                    = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);

                int nEvents = packet->getEventNumber();
                events.clear();
                for(int n=0; n<nEvents ; n++)
                {
                    const libcaer::events::PolarityEvent &polarityEvent = (*polarity)[n];

                    int y = polarityEvent.getX();
                    int x = polarityEvent.getY();
                    int t = polarityEvent.getTimestamp();
                    bool p = polarityEvent.getPolarity(); // {0,1}
                    if (m_record) { m_recorder_davis << p << '\t' << x << '\t' << y << '\t' << t << '\n'; }

                    if (m_laser)
                    {
                        int laser_x = m_laser->getX();
                        int laser_y = m_laser->getY();
                        if (m_record && m_id==0) { m_recorder_laser << laser_x << '\t' << laser_y << '\n'; }
                        events.emplace_back(x,y,p,t,laser_x,laser_y);
                    }
                    else
                    {
                        events.emplace_back(x,y,p,t);
                    }
                }
                this->warnEvent(events);
            }
            else if (packet == nullptr) { continue; }

            // Frame
            if (packet->getEventType() == FRAME_EVENT)
            {
                std::shared_ptr<const libcaer::events::FrameEventPacket> frame
                    = std::static_pointer_cast<libcaer::events::FrameEventPacket>(packet);

                const libcaer::events::FrameEvent &frameEvent = (*frame)[0];

                DAVIS240CFrame f;
                f.m_timestamp = frameEvent.getTimestamp();

                for (int row = 0; row < frameEvent.getLengthY(); row++)
                {
                    for (int col = 0; col < frameEvent.getLengthX(); col++)
                    {
                        f.m_frame[row*m_cols + col] = static_cast<uchar>(255.*frameEvent.getPixel(col, row)/65535.);
                    }
                }

                this->warnFrame(f);
            }
        }
    }
}

int DAVIS240C::stopListening()
{
    m_stop_read_thread = true;
    return 0;
}

void DAVIS240C::warnEvent(std::vector<DAVIS240CEvent>& events)
{
    for(auto it = m_event_listeners.begin(); it!=m_event_listeners.end(); it++)
    {
        for(int i=0; i<events.size(); i++)
            (*it)->receivedNewDAVIS240CEvent(events[i],m_id);
    }
}

void DAVIS240C::deregisterEventListener(DAVIS240CEventListener* listener)
{
    m_event_listeners.remove(listener);
}

//=== FRAME LISTENING ===//
// QUESTION: I cannot launch both events and frames threads (double free or corruption)
void DAVIS240C::registerFrameListener(DAVIS240CFrameListener* listener)
{
    m_frame_listeners.push_back(listener);
}

void DAVIS240C::warnFrame(DAVIS240CFrame& frame)
{
    for(auto it = m_frame_listeners.begin(); it!=m_frame_listeners.end(); it++)
    {
        (*it)->receivedNewDAVIS240CFrame(frame,m_id);
    }
}

void DAVIS240C::deregisterFrameListener(DAVIS240CFrameListener* listener)
{
    m_frame_listeners.remove(listener);
}

//=== CLOSING ===/
int DAVIS240C::stop()
{
    m_davis_handle.dataStop();
    return 0;
}


void DAVIS240C::readData()
{
    std::fstream eventReader;
    eventReader.open(m_eventRecordFileDavis);

    std::string output;
    if (eventReader.is_open())
    {
        while (!eventReader.eof())
        {
            std::getline(eventReader,output);
            std::vector<std::string> v{explode(output, '\t')};

            DAVIS240CEvent e;
            e.m_pol = std::atoi(v[0].c_str());
            e.m_x = std::atoi(v[1].c_str());
            e.m_y = std::atoi(v[2].c_str());
            e.m_timestamp = std::atoi(v[3].c_str());

            std::vector<DAVIS240CEvent> events;
            events.push_back(e);
            this->warnEvent(events);
            std::cout << e.m_pol << " - " << e.m_x << " - " << e.m_y << " - " << e.m_timestamp << '\n';
        }
    }
    eventReader.close();
    std::cout << "Finished! \n";
}
