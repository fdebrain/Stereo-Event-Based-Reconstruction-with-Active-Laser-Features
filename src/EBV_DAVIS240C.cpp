#include <EBV_DAVIS240C.h>

#include <stdio.h>
#include <stdint.h>

#include <libcaercpp/devices/davis.hpp>

static void globalShutdownSignalHandler(int signal)
{
    // Simply set the running flag to false on SIGTERM and SIGINT (CTRL+C) for global shutdown.
    if (signal == SIGTERM || signal == SIGINT)
    {
        globalShutdown.store(true);
    }
}

static void usbShutdownHandler(void *ptr)
{
    (void) (ptr); // UNUSED.
    globalShutdown.store(true);
}

unsigned int DAVIS240C::m_nbCams=0;
libcaer::devices::davis* DAVIS240C::m_davisMasterHandle = nullptr;

DAVIS240C::DAVIS240C()
    // Open a DAVIS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
    : m_id(m_nbCams),
      m_davisHandle(libcaer::devices::davis(m_id)),
      m_stopreadThreadEvents(false),
      m_stopreadThreadFrames(false)
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

DAVIS240C::~DAVIS240C()
{
    m_nbCams -= 1;
}


//=== INITIALIZING ===//
void DAVIS240C::resetMasterClock()
{
    if (m_davisMasterHandle->infoGet().deviceIsMaster)
    {
        m_davisMasterHandle->configSet(DAVIS_CONFIG_MUX,
                                 DAVIS_CONFIG_MUX_TIMESTAMP_RESET, 2);
        printf("Reset Master clock. \n\r");
    }
}

int DAVIS240C::init()
{
    // Let's take a look at the information we have on the device.
    struct caer_davis_info davis_info = m_davisHandle.infoGet();

    printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Logic: %d.\n\r", davis_info.deviceString,
        davis_info.deviceID, davis_info.deviceIsMaster, davis_info.dvsSizeX, davis_info.dvsSizeY,
        davis_info.logicVersion);

    // Send the default configuration before using the device.
    // No configuration is sent automatically!
    m_davisHandle.sendDefaultConfig();

    // Tweak some biases, to increase bandwidth in this case.
    struct caer_bias_coarsefine coarseFineBias;

    coarseFineBias.coarseValue        = 2;
    coarseFineBias.fineValue          = 116;
    coarseFineBias.enabled            = true;
    coarseFineBias.sexN               = false;
    coarseFineBias.typeNormal         = true;
    coarseFineBias.currentLevelNormal = true;

    m_davisHandle.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(coarseFineBias));

    coarseFineBias.coarseValue        = 1;
    coarseFineBias.fineValue          = 33;
    coarseFineBias.enabled            = true;
    coarseFineBias.sexN               = false;
    coarseFineBias.typeNormal         = true;
    coarseFineBias.currentLevelNormal = true;

    m_davisHandle.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(coarseFineBias));

    return 0;
}

int DAVIS240C::start()
{
    // Now let's get start getting some data from the device. We just loop in blocking mode,
    // no notification needed regarding new events. The shutdown notification, for example if
    // the device is disconnected, should be listened to.
    m_davisHandle.dataStart(nullptr, nullptr, nullptr, &usbShutdownHandler, nullptr);

    // Let's turn on blocking data-get mode to avoid wasting resources.
    m_davisHandle.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

    // Reset master clock when new device is added
    if (m_id==0) { m_davisMasterHandle = &m_davisHandle; }
    else { resetMasterClock(); }

    return 0;
}

//=== EVENT LISTENING ===//
void DAVIS240C::registerEventListener(DAVIS240CListener* listener)
{
    m_eventListeners.push_back(listener);
}

int DAVIS240C::listenEvents()
{
    m_stopreadThreadEvents = false;
    m_readThreadEvents = std::thread(&DAVIS240C::readThreadEvents,this);
    return 0;
}

void DAVIS240C::readThreadEvents()
{
    std::vector<DAVIS240CEvent> events;

    while (!globalShutdown.load(std::memory_order_relaxed) & !m_stopreadThreadEvents)
    {
        std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = m_davisHandle.dataGet();

        // Skip if nothing there.
        if (packetContainer == nullptr) { continue; }

        for (auto &packet : *packetContainer)
        {
            if (packet == nullptr) { continue; }

            if (packet->getEventType() == POLARITY_EVENT)
            {
                std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity
                    = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);

                int nEvents = packet->getEventNumber();
                events.clear();
                for(int n=0; n<nEvents ; n++)
                {
                    const libcaer::events::PolarityEvent &theEvent = (*polarity)[n];

                    DAVIS240CEvent e;
                    e.m_y           = theEvent.getX();
                    e.m_x           = theEvent.getY();
                    e.m_timestamp   = theEvent.getTimestamp();
                    e.m_pol         = theEvent.getPolarity(); // {0,1}

                    events.push_back(e);
                }
                this->warnEvent(events);
            }
        }
    }
}

void DAVIS240C::warnEvent(std::vector<DAVIS240CEvent>& events)
{
    std::list<DAVIS240CListener*>::iterator it;
    for(it = m_eventListeners.begin(); it!=m_eventListeners.end(); it++)
    {
        for(unsigned int i=0; i<events.size(); i++)
            (*it)->receivedNewDAVIS240CEvent(events[i],m_id);
    }
}

int DAVIS240C::stopListeningEvents()
{
    m_stopreadThreadEvents = true;
    return 0;
}

void DAVIS240C::deregisterEventListener(DAVIS240CListener* listener)
{
    m_eventListeners.remove(listener);
}

//=== FRAME LISTENING ===//
void DAVIS240C::registerFrameListener(DAVIS240CListener* listener)
{
    m_frameListeners.push_back(listener);
}

int DAVIS240C::listenFrames()
{
    m_stopreadThreadFrames= false;
    m_readThreadFrames= std::thread(&DAVIS240C::readThreadFrames,this);
    m_readThreadFrames.join();
    return 0;
}

void DAVIS240C::readThreadFrames()
{
    std::vector<DAVIS240CEvent> events;

    while (!globalShutdown.load(std::memory_order_relaxed) & !m_stopreadThreadFrames)
    {
        // Get data
        std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = m_davisHandle.dataGet();

        // Skip if nothing there.
        if (packetContainer == nullptr) { continue; }

        for (auto &packet : *packetContainer)
        {
            // Skip if nothing there.
            if (packet == nullptr) { continue; }

            if (packet->getEventType() == FRAME_EVENT)
            {
                std::shared_ptr<const libcaer::events::FrameEventPacket> frame
                    = std::static_pointer_cast<libcaer::events::FrameEventPacket>(packet);

                const libcaer::events::FrameEvent &theFrame = (*frame)[0];

                DAVIS240CFrame f;
                f.m_timestamp = theFrame.getTimestamp();

                for (int row = 0; row < theFrame.getLengthY(); row++)
                {
                    for (int col = 0; col < theFrame.getLengthX(); col++)
                    {
                        f.m_frame[row*m_cols + col] = static_cast<unsigned char>(255.*theFrame.getPixel(col, row)/65535.);
                    }
                }
                this->warnFrame(f);
            }
        }
    }
}

void DAVIS240C::warnFrame(DAVIS240CFrame& frame)
{
    std::list<DAVIS240CListener*>::iterator it;
    for(it = m_frameListeners.begin(); it!=m_frameListeners.end(); it++)
    {
        (*it)->receivedNewDAVIS240CFrame(frame,m_id);
    }
}

int DAVIS240C::stopListeningFrames()
{
    m_stopreadThreadFrames = true;
    return 0;
}

void DAVIS240C::deregisterFrameListener(DAVIS240CListener* listener)
{
    m_frameListeners.remove(listener);
}


//=== CLOSING ===/
int DAVIS240C::stop()
{
    m_davisHandle.dataStop();
    return 0;
}
