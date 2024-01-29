#include "usbd_core.h"
#include "usbd_printer.h"

/*!< endpoint address */
#define PRINTER_IN_EP  0x81
#define PRINTER_OUT_EP 0x02

#define USBD_VID           0xFFFF
#define USBD_PID           0xFFFF
#define USBD_MAX_POWER     2
#define USBD_LANGID_STRING 1033

/*!< config descriptor size */
#define USB_CONFIG_SIZE 32

/*!< global descriptor */
static const uint8_t printer_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0001, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    /************** Descriptor of interface *****************/
    0x09,                          /* bLength: Interface Descriptor size */
    USB_DESCRIPTOR_TYPE_INTERFACE, /* bDescriptorType: Interface descriptor type */
    0x00,                          /* bInterfaceNumber: Number of Interface */
    0x00,                          /* bAlternateSetting: Alternate setting */
    0x02,                          /* bNumEndpoints */
    0x07,                          /* bInterfaceClass: HID */
    0x01,                          /* bInterfaceSubClass : 1=BOOT, 0=no boot */
    0x02,                          /* nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse */
    0,                             /* iInterface: Index of string descriptor */
	  /******************** Descriptor of Custom in endpoint ********************/
    0x07,                          /* bLength: Endpoint Descriptor size */
    USB_DESCRIPTOR_TYPE_ENDPOINT,  /* bDescriptorType: */
    PRINTER_IN_EP,                 /* bEndpointAddress: Endpoint Address (IN) */
    0x02,                          /* bmAttributes: Bulk endpoint */
    0x40,0x00,                     /* wMaxPacketSize: */
    0x00,                          /* bInterval: Polling Interval */
	  /******************** Descriptor of Custom out endpoint ********************/
    0x07,                          /* bLength: Endpoint Descriptor size */
    USB_DESCRIPTOR_TYPE_ENDPOINT,  /* bDescriptorType: */
    PRINTER_OUT_EP,                /* bEndpointAddress: */
    0x02,                          /* bmAttributes: Bulk endpoint */
    0x40,0x00,                     /* wMaxPacketSize */
    0x00,                          /* bInterval: Polling Interval */
    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x0A,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'P', 0x00,                  /* wcChar0 */
    'U', 0x00,                  /* wcChar1 */
    'Y', 0x00,                  /* wcChar2 */
    'A', 0x00,                  /* wcChar3 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x24,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'P', 0x00,                  /* wcChar0 */
    'U', 0x00,                  /* wcChar1 */
    'Y', 0x00,                  /* wcChar2 */
    'A', 0x00,                  /* wcChar3 */
    ' ', 0x00,                  /* wcChar4 */
    'P', 0x00,                  /* wcChar5 */
    'R', 0x00,                  /* wcChar6 */
    'I', 0x00,                  /* wcChar7 */
    'N', 0x00,                  /* wcChar8 */
    'T', 0x00,                  /* wcChar9 */
    'E', 0x00,                  /* wcChar10 */
    'R', 0x00,                  /* wcChar11 */
    ' ', 0x00,                  /* wcChar12 */
    'D', 0x00,                  /* wcChar13 */
    'E', 0x00,                  /* wcChar14 */
    'M', 0x00,                  /* wcChar15 */
    'O', 0x00,                  /* wcChar16 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x16,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    '2', 0x00,                  /* wcChar0 */
    '0', 0x00,                  /* wcChar1 */
    '2', 0x00,                  /* wcChar2 */
    '2', 0x00,                  /* wcChar3 */
    '1', 0x00,                  /* wcChar4 */
    '2', 0x00,                  /* wcChar5 */
    '3', 0x00,                  /* wcChar6 */
    '4', 0x00,                  /* wcChar7 */
    '5', 0x00,                  /* wcChar8 */
    '6', 0x00,                  /* wcChar9 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x02,
    0x02,
    0x01,
    0x40,
    0x01,
    0x00,
#endif
    0x00
};

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer[128];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t write_buffer[128];

#ifdef CONFIG_USB_HS
#define PRINTER_MAX_MPS 512
#else
#define PRINTER_MAX_MPS 64
#endif

void usbd_configure_done_callback(void)
{
    /* setup first out ep read transfer */
    usbd_ep_start_read(PRINTER_OUT_EP, read_buffer, 128);
}

void usbd_printer_bulk_out(uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("actual out len:%d\r\n", nbytes);
    /* setup next out ep read transfer */
    usbd_ep_start_read(PRINTER_OUT_EP, read_buffer, 64);
}

void usbd_printer_bulk_in(uint8_t ep, uint32_t nbytes)
{
    USB_LOG_RAW("actual in len:%d\r\n", nbytes);

    if ((nbytes % PRINTER_MAX_MPS) == 0 && nbytes) {
        /* send zlp */
        usbd_ep_start_write(PRINTER_IN_EP, NULL, 0);
    } else {
        ;
    }
}

/*!< endpoint call back */
struct usbd_endpoint printer_out_ep = {
    .ep_addr = PRINTER_OUT_EP,
    .ep_cb = usbd_printer_bulk_out
};

struct usbd_endpoint printer_in_ep = {
    .ep_addr = PRINTER_IN_EP,
    .ep_cb = usbd_printer_bulk_in
};

void printer_init(void)
{
    usbd_desc_register(printer_descriptor);
    usbd_add_interface(usbd_printer_alloc_intf());
    usbd_add_endpoint(&printer_out_ep);
    usbd_add_endpoint(&printer_in_ep);
    usbd_initialize();
}

