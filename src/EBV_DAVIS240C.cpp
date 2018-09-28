#include <EBV_DAVIS240C.h>

#include <stdio.h>
#include <stdint.h>

#include <libcaercpp/devices/davis.hpp>

static void globalShutdownSignalHandler(int signal) {
    // Simply set the running flag to false on SIGTERM and SIGINT (CTRL+C) for global shutdown.
    if (signal == SIGTERM || signal == SIGINT) {
        globalShutdown.store(true);
    }
}

static void usbShutdownHandler(void *ptr) {
    (void) (ptr); // UNUSED.

    globalShutdown.store(true);
}


DAVIS240C::DAVIS240C()
    // Open a DAVIS, give it a device ID of 1, and don't care about USB bus or SN restrictions.
    : m_davisHandle(libcaer::devices::davis(1)),
      m_stopreadThread(false)
{
    struct sigaction shutdownAction;

    shutdownAction.sa_handler = &globalShutdownSignalHandler;
    shutdownAction.sa_flags   = 0;
    sigemptyset(&shutdownAction.sa_mask);
    sigaddset(&shutdownAction.sa_mask, SIGTERM);
    sigaddset(&shutdownAction.sa_mask, SIGINT);

    if (sigaction(SIGTERM, &shutdownAction, NULL) == -1) {
        libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
            "Failed to set signal handler for SIGTERM. Error: %d.", errno);
        return;
    }

    if (sigaction(SIGINT, &shutdownAction, NULL) == -1) {
        libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
            "Failed to set signal handler for SIGINT. Error: %d.", errno);
        return;
    }
}

DAVIS240C::~DAVIS240C()
{
    // Close automatically done by destructor.
}

int DAVIS240C::init()
{
    // Let's take a look at the information we have on the device.
    struct caer_davis_info davis_info = m_davisHandle.infoGet();

    printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Logic: %d.\n", davis_info.deviceString,
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

    // Let's verify they really changed!
    uint32_t prBias   = m_davisHandle.configGet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP);
    uint32_t prsfBias = m_davisHandle.configGet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP);

    printf("New bias values --- PR-coarse: %d, PR-fine: %d, PRSF-coarse: %d, PRSF-fine: %d.\n",
        caerBiasCoarseFineParse(prBias).coarseValue, caerBiasCoarseFineParse(prBias).fineValue,
        caerBiasCoarseFineParse(prsfBias).coarseValue, caerBiasCoarseFineParse(prsfBias).fineValue);

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

    return 0;
}

void DAVIS240C::readThread()
{
    std::vector<DAVIS240CEvent> events;

    while (!globalShutdown.load(std::memory_order_relaxed) & !m_stopreadThread) {
        std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = m_davisHandle.dataGet();
        if (packetContainer == nullptr) {
            continue; // Skip if nothing there.
        }

        printf("\nGot event container with %d packets (allocated).\n", packetContainer->size());

        for (auto &packet : *packetContainer) {
            if (packet == nullptr) {
                printf("Packet is empty (not present).\n");
                continue; // Skip if nothing there.
            }

            printf("Packet of type %d -> %d events, %d capacity.\n", packet->getEventType(), packet->getEventNumber(),
                packet->getEventCapacity());

            if (packet->getEventType() == POLARITY_EVENT) {
                std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity
                    = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);

                // DEBUG ===
                // Get full timestamp and addresses of first event.
                /*
                const libcaer::events::PolarityEvent &firstEvent = (*polarity)[0];

                int32_t ts = firstEvent.getTimestamp();
                uint16_t x = firstEvent.getX();
                uint16_t y = firstEvent.getY();
                bool pol   = firstEvent.getPolarity();

                printf("First polarity event - ts: %d, x: %d, y: %d, pol: %d.\n", ts, x, y, pol);
                */

                int nEvents = packet->getEventNumber();
                events.clear();
                for(int n=0; n<nEvents ; n++)
                {
                    const libcaer::events::PolarityEvent &theEvent = (*polarity)[n];

                    DAVIS240CEvent e;
                    e.m_y           = theEvent.getX();
                    e.m_x           = theEvent.getY();
                    e.m_timestamp   = theEvent.getTimestamp();
                    e.m_pol         = theEvent.getPolarity();

                    events.push_back(e);
                }
                this->warnEvent(events);
            }

            if (packet->getEventType() == FRAME_EVENT) {
                std::shared_ptr<const libcaer::events::FrameEventPacket> frame
                    = std::static_pointer_cast<libcaer::events::FrameEventPacket>(packet);

                /* DEBUG
                // Get full timestamp, and sum all pixels of first frame event.
                const libcaer::events::FrameEvent &theFrame = (*frame)[0];

                int32_t ts   = theFrame.getTimestamp();
                uint64_t sum = 0;
                int32_t m_cols = this->m_cols;


                for (int32_t y = 0; y < theFrame.getLengthY(); y++) {
                    for (int32_t x = 0; x < theFrame.getLengthX(); x++) {
                        sum += theFrame.getPixel(x, y);
                        frame[x*m_rows+y] = theFrame.getPixel(x, y);
                    }
                }

                printf("First frame event - ts: %d, sum: %" PRIu64 ".\n", ts, sum);
                */

                const libcaer::events::FrameEvent &theFrame = (*frame)[0];

                DAVIS240CFrame f;
                f.m_timestamp = theFrame.getTimestamp();

                // Problem is here:
                //std::memcpy(&(f.m_frame),&(theFrame.pixels), theFrame.getPixelsSize());
                //f.m_frame = theFrame.pixels;
                //theFrame.getPixel()

                for (int row = 0; row < theFrame.getLengthY(); row++) {
                    for (int col = 0; col < theFrame.getLengthX(); col++) {
                        f.m_frame[row*m_cols + col] = (unsigned char)(255.*theFrame.getPixel(col, row)/65535.);
                    }
                }

                this->warnFrame(f);
            }
        }
    }
}

int DAVIS240C::listen()
{
    // Spawn the thread
    m_stopreadThread = false;
    m_readThread = std::thread(&DAVIS240C::readThread,this);
    return 0;
}

int DAVIS240C::stopListening()
{
    //Communicate with the thread
    m_stopreadThread = true;
    return 0;
}

int DAVIS240C::stop()
{
    m_davisHandle.dataStop();
    return 0;
}

int DAVIS240C::close()
{
    return 0;
}

// Register a listener to receive the new events
void DAVIS240C::registerEventListener(DAVIS240CListener* listener)
{
    m_eventListeners.push_back(listener);
}

void DAVIS240C::deregisterEventListener(DAVIS240CListener* listener)
{
    m_eventListeners.remove(listener);
}

// Send new event to all listeners
void DAVIS240C::warnEvent(std::vector<DAVIS240CEvent>& events)
{
    std::list<DAVIS240CListener*>::iterator it;
    for(it = m_eventListeners.begin(); it!=m_eventListeners.end(); it++)
    {
        for(int i=0;i<events.size();i++)
            (*it)->receivedNewDAVIS240CEvent(events[i]);
    }
}

// Register a listener to receive the new frames
void DAVIS240C::registerFrameListener(DAVIS240CListener* listener)
{
    m_frameListeners.push_back(listener);
}

void DAVIS240C::deregisterFrameListener(DAVIS240CListener* listener)
{
    m_frameListeners.remove(listener);
}

// Send new frame to all listeners
void DAVIS240C::warnFrame(DAVIS240CFrame& frame)
{
    std::list<DAVIS240CListener*>::iterator it;
    for(it = m_frameListeners.begin(); it!=m_frameListeners.end(); it++)
    {
        (*it)->receivedNewDAVIS240CFrame(frame);
    }
}
