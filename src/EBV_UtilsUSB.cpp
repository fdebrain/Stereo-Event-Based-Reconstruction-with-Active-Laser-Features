#include <EBV_UtilsUSB.h>
#include <string>

void print_devs(char* output, libusb_device** devs)
{
    libusb_device *dev;
    int i = 0;

    while ((dev = devs[i++]) != NULL) {
        struct libusb_device_descriptor desc;
        int r = libusb_get_device_descriptor(dev, &desc);
        if (r < 0) {
            sprintf(output, "failed to get device descriptor");
            return;
        }

        sprintf(output,"\t %04x:%04x (bus %d, device %d)\n",
                desc.idVendor, desc.idProduct,
                libusb_get_bus_number(dev), libusb_get_device_address(dev));
    }
}

std::string classCode2String(int code)
{
    std::string deviceClass;
    switch (code) {
        case LIBUSB_CLASS_PER_INTERFACE:
            deviceClass = "CLASS_PER_INTERFACE";
            break;
        case LIBUSB_CLASS_AUDIO:
            deviceClass = "CLASS_AUDIO";
            break;
        case LIBUSB_CLASS_COMM:
            deviceClass = "CLASS_COMM";
            break;
        case LIBUSB_CLASS_HID:
            deviceClass = "CLASS_HID";
            break;
        case LIBUSB_CLASS_PHYSICAL:
            deviceClass = "CLASS_PHYSICAL";
            break;
        case LIBUSB_CLASS_PTP:
            deviceClass = "CLASS_PTP";
            break;
        case LIBUSB_CLASS_PRINTER:
            deviceClass = "CLASS_PRINTER";
            break;
        case LIBUSB_CLASS_MASS_STORAGE:
            deviceClass = "CLASS_MASS_STORAGE";
            break;
        case LIBUSB_CLASS_HUB:
            deviceClass = "CLASS_HUB";
            break;
        case LIBUSB_CLASS_DATA:
            deviceClass = "CLASS_DATA";
            break;
        case LIBUSB_CLASS_SMART_CARD:
            deviceClass = "CLASS_SMART_CARD";
            break;
        case LIBUSB_CLASS_CONTENT_SECURITY:
            deviceClass = "CLASS_CONTENT_SECURITY";
            break;
        case LIBUSB_CLASS_VIDEO:
            deviceClass = "CLASS_VIDEO";
            break;
        case LIBUSB_CLASS_PERSONAL_HEALTHCARE:
            deviceClass = "CLASS_PERSONAL_HEALTHCARE";
            break;
        case LIBUSB_CLASS_DIAGNOSTIC_DEVICE:
            deviceClass = "CLASS_DIAGNOSTIC_DEVICE";
            break;
        case LIBUSB_CLASS_WIRELESS:
            deviceClass = "CLASS_WIRELESS";
            break;
        case LIBUSB_CLASS_APPLICATION:
            deviceClass = "CLASS_APPLICATION";
            break;
        case LIBUSB_CLASS_VENDOR_SPEC:
            deviceClass = "CLASS_VENDOR_SPEC";
            break;
        default:
            break;
    }
    return deviceClass;
}

unsigned int decBCD(unsigned char const* nybbles, size_t length)
{
    unsigned int result(0);
    while (length--) {
        result = result * 100 + (*nybbles >> 4) * 10 + (*nybbles & 15);
        ++nybbles;
    }
    return result;
}

void print_dev_desc(char* output, libusb_device_handle* devh, libusb_device_descriptor* desc)
{
    if(desc->bDescriptorType == LIBUSB_DT_DEVICE)
    {
        unsigned char manufacturerName[100];
        unsigned char productName[100];
        unsigned char serialNum[100];

        libusb_get_string_descriptor_ascii(devh,desc->iManufacturer,manufacturerName,100);
        libusb_get_string_descriptor_ascii(devh,desc->iProduct,productName,100);
        libusb_get_string_descriptor_ascii(devh,desc->iSerialNumber,serialNum,100);

        sprintf(output,
                "\t    manufacturer: %s\n"
                "\t    product Name: %s\n"
                "\t    serial Number: %s\n"
                "\t    #configurations: %d\n"
                "\t    usb spec. Release #: %d.0\n"
                "\t    device class: %s\n"
                "\t    device subclass: %s\n"
                "\t    device protocol: %s\n"
                "\t    max Packet Size: %d\n"
                "\t    vendor id: 0x%04x\n"
                "\t    product id: 0x%04x\n"
                "\t    device release #: %d\n",
                manufacturerName,
                productName,
                serialNum,
                desc->bNumConfigurations,
                decBCD((unsigned char*)(&desc->bcdUSB),sizeof(desc->bcdUSB)),
                classCode2String(desc->bDeviceClass).c_str(),
                classCode2String(desc->bDeviceSubClass).c_str(),
                classCode2String(desc->bDeviceProtocol).c_str(),
                desc->bMaxPacketSize0,
                desc->idVendor,
                desc->idProduct,
                decBCD((unsigned char*)(&desc->bcdDevice),sizeof(desc->bcdDevice))
                );
    }
}

std::string transferTypeCode2String(int code)
{
    std::string transferType;
    switch (code) {
        case LIBUSB_TRANSFER_TYPE_CONTROL:
            transferType = "TRANSFER_TYPE_CONTROL";
            break;
        case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
            transferType = "TRANSFER_TYPE_ISOCHRONOUS";
            break;
        case LIBUSB_TRANSFER_TYPE_BULK:
            transferType = "TRANSFER_TYPE_BULK";
            break;
        case LIBUSB_TRANSFER_TYPE_INTERRUPT:
            transferType = "TRANSFER_TYPE_INTERRUPT";
            break;
        default:
            break;
    }
    return transferType;
}

std::string transferDirCode2String(int code)
{
    std::string transferDir;
    switch (code) {
        case LIBUSB_ENDPOINT_IN:
            transferDir = "device -> host";
            break;
        case LIBUSB_ENDPOINT_OUT:
            transferDir = "host -> device";
            break;
        default:
            break;
    }
    return transferDir;
}

void print_endp_desc(char* output, const libusb_endpoint_descriptor* desc)
{
    if(desc->bDescriptorType == LIBUSB_DT_ENDPOINT)
    {
        sprintf(output,
                "\t      address: 0x%02x\n"
                "\t      transfer direction: %s\n"
                "\t      transfer type: %s\n"
                "\t      max packet size: %d\n",
                desc->bEndpointAddress,
                transferDirCode2String(desc->bEndpointAddress & 0x80).c_str(), // 7th bit
                transferTypeCode2String(desc->bmAttributes & 0x03).c_str(), // 1-2nd bits
                desc->wMaxPacketSize
                );
    }
}

void print_interface_desc(char* output, libusb_device_handle* devh, const libusb_interface_descriptor* desc)
{
    if(desc->bDescriptorType == LIBUSB_DT_INTERFACE)
    {
        unsigned char interfaceDescription[100];
        libusb_get_string_descriptor_ascii(devh,desc->iInterface,interfaceDescription,100);

        sprintf(output,
                "\t     interface id: %d\n"
                "\t     description: %s\n"
                "\t     #endpoints: %d\n"
                "\t     class: %s\n"
                "\t     subclass: %s\n"
                "\t     protocol: %s\n",
                desc->bInterfaceNumber,
                interfaceDescription,
                desc->bNumEndpoints,
                classCode2String(desc->bInterfaceClass).c_str(),
                classCode2String(desc->bInterfaceSubClass).c_str(),
                classCode2String(desc->bInterfaceProtocol).c_str()
                );
    }
}

void print_config_desc(char* output, libusb_device_handle* devh, const libusb_config_descriptor* desc)
{
    if(desc->bDescriptorType == LIBUSB_DT_CONFIG)
    {
        unsigned char configDescription[100];
        libusb_get_string_descriptor_ascii(devh,desc->iConfiguration,configDescription,100);

        sprintf(output,
                "\t    config id: %d\n"
                "\t    description: %s\n"
                "\t    #interfaces: %d\n"
                "\t    characteristics: 0x%02x\n"
                "\t    power consumption: %d\n",
                desc->bConfigurationValue,
                configDescription,
                desc->bNumInterfaces,
                desc->bmAttributes,
                desc->MaxPower
                );
    }
}
