#include <EBV_DVS128USB.h>
#include <EBV_UtilsUSB.h>

#include <libusb-1.0/libusb.h>
#include <arpa/inet.h>
#include <iostream>

/* usb device descriptor for the retina DVS128 by INILABS */
#define USB_USBIO_VENDOR_ID             0x152a
#define USB_USBIO_DVS128_PRODUCT_ID     0x8400
/* bits to set int the requests to act on the DVS128 by INILABS */
#define VENDOR_REQUEST_START_TRANSFER	0xb3
#define VENDOR_REQUEST_STOP_TRANSFER	0xb4
#define VENDOR_REQUEST_SEND_BIASES      0xB8
#define VENDOR_REQUEST_SET_LED			0xbf
/* endpoint for getting status info: 0x81  EP 1 BULK IN */
#define USB_ENDPOINT_STATUS_IN          0x81
/* endpoint for reading data: 0x02  EP 2 BULK OUT */
#define USB_ENDPOINT_OUT                0x02
/* endpoint for reading address events: 0x86  EP 6 BULK IN */
#define USB_ENDPOINT_ADDRESS_EVENTS_IN	0x86

/* Dummy but usefull macros */
#define U8T(X) ((uint8_t) (X))
#define U16T(X) ((uint16_t) (X))
#define U32T(X) ((uint32_t) (X))
#define U64T(X) ((uint64_t) (X))

DVS128USB::DVS128USB(void)
    : m_init(false),
      m_interfaced(false),
      m_started(false),
      m_toggleListening(false),
      m_devSpeed(-1),
      m_activeConfig(-1),
      m_devBusNum(-1),
      m_devBusAddr(-1),
      m_activeConfigDesc(NULL),
      m_devDesc(NULL),
      m_devh(NULL),
      m_dev(NULL),
      m_readThread(0),
      m_verboseOutput(stderr),
      m_ctrlTransTimeout(20000),
      m_readThreadStatus(DVS128USB::TERMINATE_THREAD),
      m_biases()
{
}

DVS128USB::~DVS128USB(void)
{
    this->close();

    if(m_activeConfigDesc)
        libusb_free_config_descriptor(m_activeConfigDesc);
    if(m_devDesc)
        delete m_devDesc;
}

int DVS128USB::getStatus(void)
{
    return m_status;
}

int DVS128USB::getNumBus(void)
{
    return m_devBusNum;
}

int DVS128USB::getAddrBus(void)
{
    return m_devBusAddr;
}

int DVS128USB::getSpeed(void)
{
    return m_devSpeed;
}

void DVS128USB::setMaxCtrlTransTimeout(unsigned int to)
{
    m_ctrlTransTimeout = to;
}

void DVS128USB::setVerboseOutput(FILE* f)
{
    m_verboseOutput = f;
}

int DVS128USB::findDevice(void)
{
    m_devh = libusb_open_device_with_vid_pid(NULL,
                                             USB_USBIO_VENDOR_ID,
                                             USB_USBIO_DVS128_PRODUCT_ID
                                             );
    if(!m_devh)
    {
        libusb_device **devs;
        int numDevices = libusb_get_device_list(NULL, &devs);
        if (numDevices < 0)
        {
            logg("\t Could not find any device connected to USB\n");
            this->initFailed();
        }

        int i=0;
        libusb_device* dev;
        while ((dev = devs[i++]) != NULL)
        {
            struct libusb_device_descriptor desc;
            int r = libusb_get_device_descriptor(dev, &desc);
            if (r < 0)
            {
                logg("failed to get device descriptor");
                return -1;
            }

            if((desc.idVendor==USB_USBIO_VENDOR_ID) && (desc.idProduct==USB_USBIO_DVS128_PRODUCT_ID))
            {
                logg("\t Device found:\n");
                m_dev = dev;
                int r = libusb_open(m_dev,&m_devh);
                switch(r)
                {
                    case LIBUSB_ERROR_NO_MEM:
                        logg("\t   Device not alloced (not enough memory)\n");
                    break;
                    case LIBUSB_ERROR_ACCESS:
                        logg("\t   Device not opened (insufficient permissions)\n");
                    break;
                    case LIBUSB_ERROR_NO_DEVICE:
                        logg("\t   Device not opened (disconnected)\n");
                    break;
                    case 0:
                        logg("\t   Device successfully opened\n");
                    break;
                    default:
                        logg("\t   Device not opened (unknown error)\n");
                }
            }
        }
        libusb_free_device_list(devs, 1);
    }

    if (m_devh) {
        m_dev = libusb_get_device(m_devh);
        if(m_dev)
        {
            m_devBusNum = libusb_get_bus_number(m_dev);
            m_devBusAddr = libusb_get_device_address(m_dev);
//            m_devSpeed = libusb_get_device_speed (m_dev);

            m_devDesc = new libusb_device_descriptor();
            libusb_get_device_descriptor(m_dev,m_devDesc);
            logg("\t Device descriptor:\n");
            print_dev_desc(m_buffStatus,m_devh,m_devDesc);

            libusb_get_configuration(m_devh,&m_activeConfig);
            libusb_get_active_config_descriptor(m_dev,&m_activeConfigDesc);
            if(m_activeConfigDesc)
            {
                logg("\t Current configuration:\n");
                print_config_desc(m_buffStatus,m_devh,m_activeConfigDesc);
                logg(m_buffStatus);

                logg("\t Available interfaces:\n");
                for(int i=0;i<m_activeConfigDesc->bNumInterfaces;i++)
                {
                    sprintf(m_buffStatus,"\t - interface-%d\n",i);
                    logg(m_buffStatus);
                    sprintf(m_buffStatus,"\t     #settings: %d\n",m_activeConfigDesc->interface[i].num_altsetting);
                    logg(m_buffStatus);

                    for(int j=0;j<m_activeConfigDesc->interface[i].num_altsetting;j++)
                    {
                        sprintf(m_buffStatus,"\t  - setting-%d\n",j);
                        logg(m_buffStatus);
                        const libusb_interface_descriptor* interface =
                            &(m_activeConfigDesc->interface[i].altsetting[j]);
                        print_interface_desc(m_buffStatus,m_devh,interface);
                        logg(m_buffStatus);

                        for(int k=0;k<interface->bNumEndpoints;k++)
                        {
                            sprintf(m_buffStatus,"\t   . endpoint-%d:\n",k);
                            logg(m_buffStatus);
                            print_endp_desc(m_buffStatus,&(interface->endpoint[k]));
                            logg(m_buffStatus);
                        }
                    }
                }
            }
        }
    }
    return m_devh ? 0 : -EIO;
}

int DVS128USB::init(void)
{
    logg("> Starting DVS128_Driver ... \n");
    m_status = libusb_init(NULL);
    if (m_status < 0) {
        logg("\t Failed to initialize libusb\n");
        return m_status;
    }
    logg("\t Initialized libusb \n");
    m_init = true;

    logg("> Searching DVS128 device ... \n");
    m_status = this->findDevice();
    if (m_status < 0) {
        logg("\t Could not find/open DVS128 device\n");
        return this->initFailed();
    }
    logg("\t Found DVS128 device \n");

    logg("> Interfacing with DVS128 device ... \n");
    if (libusb_get_configuration(m_devh, &m_activeConfig) != LIBUSB_SUCCESS) {
        return this->initFailed();
    }
    if (m_activeConfig != 1) {
        if (libusb_set_configuration(m_devh, 1) != LIBUSB_SUCCESS) {
            return this->initFailed();
        }
    }
    m_status = libusb_claim_interface(m_devh, 0);
    if (m_status < 0) {
        logg("\t Could not interface with the device");
        sprintf(m_buffStatus, "\t - usb_claim_interface error %d\n", m_status);
        logg(m_buffStatus);
        return this->initFailed();
    }
    logg("\t Interface connections: \n");
    sprintf(m_buffStatus,"\t   device: %d, on bus: %d\n", m_devBusAddr, m_devBusNum);
    logg(m_buffStatus);
    sprintf(m_buffStatus,"\t   negotiated speed: %d\n", m_devSpeed);
    logg(m_buffStatus);
    if(m_devDesc)
    {
        sprintf(m_buffStatus,"\t   active config id/#configs: %d/%d\n",m_activeConfig,m_devDesc->bNumConfigurations);
        logg(m_buffStatus);
    }
    logg("\t Interfaced with DVS128 device \n");
    m_interfaced = true;

    logg("> Sending Config Biases \n");
    this->sendBiases(m_biases);
}

int DVS128USB::start(void)
{
    if(m_init)
    {
        if(m_interfaced)
        {
            logg("> Starting DVS128_Driver ... \n");
            m_status =
                this->vendorRequest(m_devh, VENDOR_REQUEST_START_TRANSFER, 0, 0, NULL, 0);
            m_started = true;
            if(m_status < 0)
            {
                logg("\t Could not start DVS128_Driver\n");
                this->initFailedWithInterface();
            }
        }
        else
        {
            logg("> Must interface DVS128_Driver first ... \n");
        }
    }
    else
    {
        logg("> Must init DVS128_Driver first ... \n");
    }
}

int DVS128USB::stop(void)
{
    if(m_init)
    {
        if(m_interfaced)
        {
            if(m_started)
            {
                logg("> Stopping DVS128_Driver ... \n");
                m_status =
                    this->vendorRequest(m_devh, VENDOR_REQUEST_STOP_TRANSFER, 0, 0, NULL, 0);
                m_started = false;
                if(m_status < 0)
                {
                    logg("\t Could not stop DVS128_Driver\n");
                    this->initFailedWithInterface();
                }
            }
            else
            {
                logg("> Must start DVS128_Driver first ... \n");
            }
        }
        else
        {
            logg("> Must interface DVS128_Driver first ... \n");
        }
    }
    else
    {
        logg("> Must init DVS128_Driver first ... \n");
    }
}

int DVS128USB::reset(void)
{
    if(m_init)
        m_status = libusb_reset_device(m_devh);
    return m_status;
}


#define HIGHER_BIT_MASK 0x80
#define LOWER_BITS_MASK 0x7F
uint32_t wrapAdd = 0;
uint16_t timestampUSB = 0;
uint32_t timestamp = 0;
uint32_t lastTimestamp = 0;
inline void decodeDVSEvent(  const unsigned char *data,
                             const int bytesRead,
                             std::vector<DVS128USBEvent>* events,
                             std::vector<unsigned int>* specEvents)
{
    unsigned char B1,B2,B3,B4;

    for(size_t i=0;i<bytesRead;i+=4)
    {
        // ======== Assumed format ==========
        // yyyyyyyp 0xxxxxxx tttttttttttttttt
        // |__B1__| |__B2__| |____B3 & B4____|
        if(data[i+1] & HIGHER_BIT_MASK)
        {
            //if we are not synchronized just continue
            //fprintf(stdout,"===== Out of sync ======\n");
            //fflush(stdout);

            // Insert an event in the special channel
            timestampUSB = le16toh(*((uint16_t * ) (&data[i + 2])));
            timestamp = timestampUSB + wrapAdd;
            lastTimestamp = timestamp;
            specEvents->push_back(timestamp);

            // Also insert in the non-special channel
            DVS128USBEvent e;
            e.m_pol = 0;
            e.m_x = 0;
            e.m_y = 0;
            e.m_timestamp = timestamp;//((data[i+2] << 8) | (data[i+3]));
            e.m_rawdata[0] = 's';
            events->push_back(e);
            //i ++;
            continue;
        }

        if ((data[i+3] & 0x80) == 0x80)
        {
            // timestamp bit 15 is one -> wrap: now we need to increment
            // the wrapAdd, uses only 14 bit timestamps
            wrapAdd += 0x4000;

            // Detect big timestamp wrap-around.
            if (wrapAdd == 0)
            {
                // Reset lastTimestamp to zero at this point, so we can again
                // start detecting overruns of the 32bit value.
                lastTimestamp = 0;
            }
        }
        else if ((data[i+3] & 0x40) == 0x40)
        {
            // timestamp bit 14 is one -> wrapAdd reset: this firmware
            // version uses reset events to reset timestamps
            wrapAdd = 0;
            lastTimestamp = 0;
        }
        else
        {
            // For timestamp, LSB MSB (USB is LE)
            // 15 bit value of timestamp in 1 us tick
            timestampUSB = le16toh(*((uint16_t * ) (&data[i + 2])));

            // Expand to 32 bits. (Tick is 1µs already.)
            timestamp = timestampUSB + wrapAdd;

            // Check monotonicity of timestamps.
            if (timestamp < lastTimestamp) {
                // std::cout << "DVS128: non-monotonic time-stamp detected: lastTimestamp="
                //s << lastTimestamp << ", timestamp=" << timestamp << "." << std::endl;
            }
            lastTimestamp = timestamp;
        }

        DVS128USBEvent e;
        e.m_pol = 1 - int(data[i] & 0x1);
        e.m_x = 127-(unsigned int)(data[i+1] & LOWER_BITS_MASK);
        e.m_y = 127-(unsigned int)(data[i] >> 1);
        e.m_timestamp = timestamp;//((data[i+2] << 8) | (data[i+3]));

        events->push_back(e);
    }
}

void* readThreadDVS(void* arg)
{
    struct DVS128USBArgs* args = (struct DVS128USBArgs*)(arg);

    #define TO_READ    512 // bytes
    #define EVENT_SIZE 4 // bytes

    // To receive the blocking bulk transfer
    unsigned char               data[TO_READ];
    int                         bytesRead;
    std::vector<DVS128USBEvent> events;
    std::vector<unsigned int>   specEvents;

    while(true)
    {
        libusb_bulk_transfer(args->devh,
                             USB_ENDPOINT_ADDRESS_EVENTS_IN,
                             data,
                             TO_READ,
                             &bytesRead,
                             0);

        //#define VERBOSE_DVS128USB
        #ifdef VERBOSE_DVS128USB
            fprintf(stdout,"\t Read %d events\n",bytesRead/EVENT_SIZE);

            for(int j=0;j<bytesRead/EVENT_SIZE;j++)
            {
               fprintf(stdout,"\t\t . 0x ");
               for(int i=0;i<EVENT_SIZE;i++)
                   fprintf(stdout,"%02x ", data[EVENT_SIZE*j+i]);
               fprintf(stdout,"\n");
               fflush(stdout);
            }
        #endif

        //Fill in x,y,p,ts when decoding the event.
        events.clear();
        decodeDVSEvent(data,bytesRead,
                       &events,&specEvents);
        args->device->warnEvent(events);
        args->device->warnSpecEvent(specEvents);

        if(*(args->threadStatus)==DVS128USB::TERMINATE_THREAD)
            break;
    }

    pthread_exit(NULL);
}

/*
void* readThreadDVS(void* arg)
{
    struct DVS128USBArgs* args = (struct DVS128USBArgs*)(arg);

    #define TO_READ    512 // bytes
    #define EVENT_SIZE 4 // bytes

    // To receive the blocking bulk transfer
    unsigned char      data[TO_READ];
    int                bytesRead;

    while(true)
    {
        libusb_bulk_transfer(args->devh,
                             USB_ENDPOINT_ADDRESS_EVENTS_IN,
                             data,
                             TO_READ,
                             &bytesRead,
                             10);

        for(int i=0;i<bytesRead/EVENT_SIZE;i++)
        {
            args->events->push_back();
        }

        if(*(args->threadStatus)==DVS128USB::TERMINATE_THREAD)
            break;
    }

    pthread_exit(NULL);
}

void* warnThreadDVS(void* arg)
{
    unsigned char*     data = args->sharedMemory;
    int                bytesRead;

    while(true)
    {
        struct DVS128USBArgs* args = (struct DVS128USBArgs*)(arg);

        //Fill in x,y,p,ts when decoding the event.
        events.clear();
        decodeDVSEvent(data,bytesRead,
                       &events);
        args->device->warnEvent(events);

        if(*(args->threadStatus)==DVS128USB::TERMINATE_THREAD)
            break;
    }
    pthread_exit(NULL);
}
*/

int DVS128USB::listen(void)
{
    if(m_init)
    {
        if(m_interfaced)
        {
            if(m_started)
            {
                if(!m_toggleListening)
                {
                    int sharedMemorySize = 4096;
                    unsigned char* sharedMemory = new unsigned char[sharedMemorySize];

                    m_readThreadArgs.devh = m_devh;
                    m_readThreadArgs.device = this;
                    m_readThreadArgs.threadStatus = &m_readThreadStatus;
                    m_readThreadArgs.events = &m_events;

                    m_warnThreadArgs.devh = m_devh;
                    m_warnThreadArgs.device = this;
                    m_warnThreadArgs.threadStatus = &m_warnThreadStatus;
                    m_warnThreadArgs.events = &m_events;

                    logg("> Activate listening\n");
                    m_readThreadStatus = DVS128USB::RUN_THREAD;
                    m_toggleListening = true;
                    pthread_create(&m_readThread,NULL,readThreadDVS,(void*)&m_readThreadArgs);
                    //pthread_create(&m_warnThread,NULL,,(void*)&m_warnThreadArgs);
                    return 0;
                }
                else
                {
                    return 0;
                }
            }
        }
    }
}

int DVS128USB::stopListening(void)
{
    if(!m_toggleListening)
    {
       return 0;
    }
    else
    {
        //if(pthread_cancel(m_thread)<0)
        //{
           // return -1;
        //}
        //else
        //{
        //    m_toggleListening = false;
        //    return 0;
        //}
        m_readThreadStatus = DVS128USB::TERMINATE_THREAD;
        m_toggleListening = false;
        return 0;
    }
}

DVS128USBBiases DVS128USB::getBiases()
{
    return m_biases;
}

int DVS128USB::sendBiases(DVS128USBBiases b)
{
    m_biases = b;

    // 12 biases, a 24 bits = 3 bytes each.
    uint8_t biases[12 * 3];

    biases[0] = U8T(U32T(b.cas) >> 16);
    biases[1] = U8T(U32T(b.cas) >> 8);
    biases[2] = U8T(U32T(b.cas) >> 0);

    biases[3] = U8T(U32T(b.injGnd) >> 16);
    biases[4] = U8T(U32T(b.injGnd) >> 8);
    biases[5] = U8T(U32T(b.injGnd) >> 0);

    biases[6] = U8T(U32T(b.reqPd) >> 16);
    biases[7] = U8T(U32T(b.reqPd) >> 8);
    biases[8] = U8T(U32T(b.reqPd) >> 0);

    biases[9] = U8T(U32T(b.puX) >> 16);
    biases[10] = U8T(U32T(b.puX) >> 8);
    biases[11] = U8T(U32T(b.puX) >> 0);

    biases[12] = U8T(U32T(b.diffOff) >> 16);
    biases[13] = U8T(U32T(b.diffOff) >> 8);
    biases[14] = U8T(U32T(b.diffOff) >> 0);

    biases[15] = U8T(U32T(b.req) >> 16);
    biases[16] = U8T(U32T(b.req) >> 8);
    biases[17] = U8T(U32T(b.req) >> 0);

    biases[18] = U8T(U32T(b.refr) >> 16);
    biases[19] = U8T(U32T(b.refr) >> 8);
    biases[20] = U8T(U32T(b.refr) >> 0);

    biases[21] = U8T(U32T(b.puY) >> 16);
    biases[22] = U8T(U32T(b.puY) >> 8);
    biases[23] = U8T(U32T(b.puY) >> 0);

    biases[24] = U8T(U32T(b.diffOn) >> 16);
    biases[25] = U8T(U32T(b.diffOn) >> 8);
    biases[26] = U8T(U32T(b.diffOn) >> 0);

    biases[27] = U8T(U32T(b.diff) >> 16);
    biases[28] = U8T(U32T(b.diff) >> 8);
    biases[29] = U8T(U32T(b.diff) >> 0);

    biases[30] = U8T(U32T(b.foll) >> 16);
    biases[31] = U8T(U32T(b.foll) >> 8);
    biases[32] = U8T(U32T(b.foll) >> 0);

    biases[33] = U8T(U32T(b.pr) >> 16);
    biases[34] = U8T(U32T(b.pr) >> 8);
    biases[35] = U8T(U32T(b.pr) >> 0);

    this->vendorRequest(m_devh,
                        VENDOR_REQUEST_SEND_BIASES,
                        0,
                        0,
                        biases,
                        sizeof(biases)
                        );
}

int DVS128USB::initFailed(void)
{
    sprintf(m_buffStatus, "> Initialization failed ... \n\t Exiting DVS128_Driver \n\t !! Must reinit !!\n");
    logg(m_buffStatus);
    libusb_close(m_devh);
    libusb_exit(NULL);
    m_init = false;

    return m_status;
}

int DVS128USB::initFailedWithInterface(void)
{
    libusb_release_interface(m_devh, 0);
    m_interfaced = false;

    return this->initFailed();
}

int DVS128USB::close(void)
{
    logg("> Exiting DVS128_Driver ... \n");
    if(m_init)
    {
        if(m_interfaced)
        {
            if(m_started)
            {
                this->stop();
                logg("\t Stopped DVS128_Driver \n");
            }
            libusb_release_interface(m_devh, 0);
            m_interfaced = false;
            logg("\t Released interface DVS128_Driver \n");
        }
        libusb_close(m_devh);
        libusb_exit(NULL);
        m_init = false;
        logg("\t Closed DVS128_Driver \n");
    }
}

int DVS128USB::discover(void)
{
    if(!m_init)
    {
        logg("> Starting USB initialization ... \n");
        int r = libusb_init(NULL);
        if (r < 0) {
            logg("\t Failed to initialize libusb\n");
            return r;
        }
    }
    libusb_device **devs;

    logg("> Listing current devices ... \n");
    int numDevices = libusb_get_device_list(NULL, &devs);
    if (numDevices < 0) {
        logg("\t Could not find any device connected to USB\n");
        this->initFailed();
    }
    print_devs(m_buffStatus, devs);

    libusb_free_device_list(devs, 1);
}

int DVS128USB::vendorRequest(libusb_device_handle* devh,
                              uint8_t bRequest,
                              uint16_t wValue,
                              uint16_t wIndex,
                              unsigned char *data,
                              uint16_t wLength)
{
    if (devh)
    {
        return libusb_control_transfer(devh,
                                         LIBUSB_ENDPOINT_OUT
                                       | LIBUSB_REQUEST_TYPE_VENDOR
                                       | LIBUSB_RECIPIENT_DEVICE,
                                       bRequest,
                                       wValue,
                                       wIndex,
                                       data,
                                       wLength,
                                       m_ctrlTransTimeout);
    }
}

void DVS128USB::registerListener(DVS128USBListener* listener)
{
    m_listeners.push_back(listener);
}

void DVS128USB::deregisterListener(DVS128USBListener* listener)
{
    m_listeners.remove(listener);
}

void DVS128USB::warnEvent(std::vector<DVS128USBEvent>& events)
{
    std::list<DVS128USBListener*>::iterator it;
    for(it = m_listeners.begin(); it!=m_listeners.end(); it++)
    {
        for(int i=0;i<events.size();i++)
            (*it)->receivedNewDVS128USBEvent(events[i]);
    }
}

void DVS128USB::registerSpecListener(DVS128USBSpecListener* listener)
{
    m_specListeners.push_back(listener);
}

void DVS128USB::deregisterSpecListener(DVS128USBSpecListener* listener)
{
    m_specListeners.remove(listener);
}

void DVS128USB::warnSpecEvent(std::vector<unsigned int>& specEvent)
{
    std::list<DVS128USBSpecListener*>::iterator it;
    for(it = m_specListeners.begin(); it!=m_specListeners.end(); it++)
    {
        for(int i=0;i<specEvent.size();i++)
            (*it)->receivedNewDVS128USBSpecEvent(specEvent[i]);
    }
}

void DVS128USB::registerStatusListener(DVS128USBStatusListener* listener)
{
    m_statusListeners.push_back(listener);
}

void DVS128USB::deregisterStatusListener(DVS128USBStatusListener* listener)
{
    m_statusListeners.remove(listener);
}

void DVS128USB::warnStatus(std::string status)
{
    std::list<DVS128USBStatusListener*>::iterator it;
    for(it = m_statusListeners.begin(); it!=m_statusListeners.end(); it++)
    {
        (*it)->receivedNewDVS128USBStatus(status);
    }
}

void DVS128USB::logg(std::string toLogg)
{
    fprintf(m_verboseOutput, toLogg.c_str());
    fflush(m_verboseOutput);
    this->warnStatus(toLogg);
}
