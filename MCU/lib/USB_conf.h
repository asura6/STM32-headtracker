#ifndef STM_USB_conf
#define STM_USB_conf

#include "USB_def.h"

/******************************************/
/*********   USER CONFIGURATION   *********/
/******************************************/ 

#define USB_VID 0x0666
#define USB_PID 0x0666

#define USB_MANUFACTURER_NAME 'R','a','i','n','F','o','o'
#define USB_MANUFACTURER_LEN 7U

#define USB_PRODUCT_NAME 'A','s','u','r','a',' ',\
    'H','e','a','d','t','r','a','c','k','e','r'
#define USB_PRODUCT_LEN 17U

#define USB_SERIAL '0','0','0','1'
#define USB_SERIAL_LEN 4U

/***********************************/
/*********   DESCRIPTORS   *********/
/***********************************/

static const USB_device_descriptor_t USB_device_descriptor = {
    .bLength            = 18U,
    .bDescriptorType    = 0x01U,
    .bcdUSB             = 0x0200U,
    .bDeviceClass       = 0x00U,
    .bDeviceSubClass    = 0x00U,
    .bDeviceProtocol    = 0x00U,
    .bMaxPacketSize     = 64U,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01U,
    .iProduct           = 0x02U,
    .iSerialNumber      = 0x03U,
    .bNumConfigurations = 0x01U
};

static const USB_combined_descriptor_t USB_combined_descriptor = {
    .conf_desc = {
        .bLength            = 0x09U,
        .bDescriptorType    = 0x02U,
        .wTotalLength0      = 34U,      .wTotalLength1  = 0x00,
        .bNumInterfaces     = 0x01U,
        .bConfigurationValue = 0x01U,
        .iConfiguration     = 0x00U,
        .bmAttributes       = 0x80U,
        .bMaxPower          = 50U,
    },
    .interf_desc = {
        .bLength            = 0x09U,
        .bDescriptorType    = 0x04U,
        .bInterfaceNumber   = 0x00U,
        .bAlternateSetting  = 0x00U,
        .bNumEndpoints      = 0x01U, //Endpoint zero is excluded
        .bInterfaceClass    = 0x03U,
        .bInterfaceSubClass = 0x00U,
        .bInterfaceProtocol = 0x00U,
        .iInterface         = 0x00U
    },
    .HID_desc = {
        .bLength            = 0x09U,
        .bDescriptorType_HID = 0x21U,
        .bcdHID0            = 0x01U,    .bcdHID1        = 0x01U,
        .bCountryCode       = 0x00U,
        .bNumDescriptors    = 0x01U,
        .bDescriptorType_class  = 0x22U,
        .wDescriptorLength0 = 60U,      .wDescriptorLength1  = 0x00U
    },
    .endp_desc = {
        .bLength            = 7U,
        .bDescriptorType    = 0x05U,
        .bEndpointAddress   = 0x01U | 0x80U,   //Address 1, direction IN
        .bmAttributes       = 0x03U,           //Interrupt endpoint
        .wMaxPacketSize0    = 64U,    .wMaxPacketSize1      = 0x00U,
        .bInterval          = 10U              //ms polling interval
    }
};

/*************************************************/
/*********   HID REPORT AND DESCRIPTOR   *********/
/*************************************************/

static const uint8_t HID_report_desc[] = {
        0x05, 0x01,                     // Usage Page (Generic Desktop)
        0x09, 0x04,                     // Usage (Joystick)
        0xA1, 0x01,                     // Collection (Application)
        0x15, 0x00,                     // Logical Minimum (0)
        0x25, 0x01,                     // Logical Maximum (1)
        0x75, 0x01,                     // Report Size (1)
        0x95, 0x10,                     // Report Count (16)
        0x05, 0x09,                     // Usage Page (Button)
        0x19, 0x01,                     // Usage Minimum (Button #1)
        0x29, 0x10,                     // Usage Maximum (Button #16)
        0x81, 0x02,                     // Input (variable,absolute)
        0x05, 0x01,                     // Usage Page (Generic Desktop)
        0x09, 0x01,                     // Usage (Pointer)
        0xA1, 0x00,                     // Collection ()
	0x16, 0x00, 0x80,               //   Logical Minimum (-32768)
	0x26, 0xFF, 0x7F,               //   Logical Maximum (32767)
        0x75, 0x10,                     //   Report Size (16)
        0x95, 0x06,                     //   Report Count (6)
        0x19, 0x30,                     //   Usage Minimum (X axis)
        0x29, 0x35,                     //   Usage Maximum (Rz axis)
        0x81, 0x02,                     //   Input (variable,absolute)
        0xC0,                           // End Collection
        0x16, 0x00, 0x80,               // Logical Minimum (-32768)
	0x26, 0xFF, 0x7F,               // Logical Maximum (32768)
        0x75, 0x10,                     // Report Size (16)
        0x95, 0x03,                     // Report Count (3)
        0x09, 0x36,                     // Usage (Slider)
        0x81, 0x02,                     // Input (variable,absolute)
        0xC0                            // End Collection
}; 

typedef struct { 
    uint16_t buttons;
    int16_t t_x; //Translate x
    int16_t t_y; //Translate y
    int16_t t_z; //Translate z
    int16_t r_x; //Rotate x (Roll)
    int16_t r_y; //Rotate y (Pitch)
    int16_t r_z; //Rotate z (Yaw)
    int16_t m_x; //Mag x
    int16_t m_y; //Mag y
    int16_t m_z; //Mag z 
} HID_report_t;

#define HID_REPORT_SIZE 20U 

/******************************************/
/*********   STRING DESCRIPTORS   *********/
/******************************************/

static uint16_t USB_string_language[] = {0x0409U}; //English - United States
static uint16_t USB_string_manufacturer[] = {USB_MANUFACTURER_NAME};
static uint16_t USB_string_product[] = {USB_PRODUCT_NAME};
static uint16_t USB_string_serial[] = {USB_SERIAL};

static const USB_string_descriptor_t USB_string_descriptor_zero = {
    .bLength            = 4U,
    .bDescriptorType    = 0x03,
    .bString            = USB_string_language
};

static const USB_string_descriptor_t USB_string_descriptor_manufacturer = {
    .bLength            = 2U + (USB_MANUFACTURER_LEN << 1),
    .bDescriptorType    = 0x03,
    .bString            = USB_string_manufacturer
};

static const USB_string_descriptor_t USB_string_descriptor_product = {
    .bLength            = 2U + (USB_PRODUCT_LEN << 1),
    .bDescriptorType    = 0x03,
    .bString            = USB_string_product
};

static const USB_string_descriptor_t USB_string_descriptor_serial = {
    .bLength            = 2U + (USB_SERIAL_LEN << 1),
    .bDescriptorType    = 0x03,
    .bString            = USB_string_serial
};

#endif
