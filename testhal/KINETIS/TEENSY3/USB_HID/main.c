/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include "hal.h"
#include "chprintf.h"
/*
 * USB Device Descriptor.
 */
static uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
                         0x00,          /* bDeviceClass.                    */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         0x40,          /* bMaxPacketSize.                  */
                         0x16c0,        /* idVendor.                        */
                         0x0479,        /* idProduct.                       */
                         0x0200,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

#define RAWHID_USAGE_PAGE 0xFFAB
#define RAWHID_USAGE      0x0200
#define RAWHID_TX_SIZE    64
#define RAWHID_RX_SIZE    64

static uint8_t rawhid_desc_data[] = {
  USB_DESC_BYTE(0x06), USB_DESC_WORD(RAWHID_USAGE_PAGE),
  USB_DESC_BYTE(0x0A), USB_DESC_WORD(RAWHID_USAGE),
  USB_DESC_BYTE(0xA1), USB_DESC_BYTE(0x01),    // Collection 0x01
    USB_DESC_BYTE(0x75), USB_DESC_BYTE(0x08),    // report size = 8 bits
    USB_DESC_BYTE(0x15), USB_DESC_BYTE(0x00),    // logical minimum = 0
    USB_DESC_BYTE(0x26), USB_DESC_WORD(0x00FF),  // logical maximum = 255
    USB_DESC_BYTE(0x95), USB_DESC_BYTE(RAWHID_TX_SIZE), // report count
      USB_DESC_BYTE(0x09), USB_DESC_BYTE(0x01),           // usage
      USB_DESC_BYTE(0x81), USB_DESC_BYTE(0x02),           // Input (array)
    USB_DESC_BYTE(0x95), USB_DESC_BYTE(RAWHID_RX_SIZE), // report count
      USB_DESC_BYTE(0x09), USB_DESC_BYTE(0x02),           // usage
      USB_DESC_BYTE(0x91), USB_DESC_BYTE(0x02),           // Output (array)
  USB_DESC_BYTE(0xC0)                              // end collection
}; /* // 3+3+2 + 2+2+3 + 2+2+2+ 2+2+2 +1 */

#define DEBUGHID_USAGE_PAGE 0xFF31
#define DEBUGHID_USAGE      0x0074
#define DEBUGHID_TX_SIZE    64

static uint8_t debughid_desc_data[] = {
  USB_DESC_BYTE(0x06), USB_DESC_WORD(DEBUGHID_USAGE_PAGE),
  USB_DESC_BYTE(0x09), USB_DESC_WORD(DEBUGHID_USAGE),
  USB_DESC_BYTE(0xA1), USB_DESC_BYTE(0x01),    // Collection 0x01
    USB_DESC_BYTE(0x75), USB_DESC_BYTE(0x08),    // report size = 8 bits
    USB_DESC_BYTE(0x15), USB_DESC_BYTE(0x00),    // logical minimum = 0
    USB_DESC_BYTE(0x26), USB_DESC_WORD(0x00FF),  // logical maximum = 255
    USB_DESC_BYTE(0x95), USB_DESC_BYTE(DEBUGHID_TX_SIZE), // report count
      USB_DESC_BYTE(0x09), USB_DESC_BYTE(0x01),           // usage
      USB_DESC_BYTE(0x81), USB_DESC_BYTE(0x02),           // Input (array)
  USB_DESC_BYTE(0xC0)                              // end collection
}; /* // 3+3+2 + 2+2+3 +2+2+2 +1 */

static USBDescriptor hid_descriptors[] =
{
  {sizeof rawhid_desc_data, rawhid_desc_data},
  {sizeof debughid_desc_data, debughid_desc_data},

};

#define RAWHID_TX_ENDPOINT 1
#define RAWHID_RX_ENDPOINT 2
#define DEBUGHID_ENDPOINT  3


/* Configuration Descriptor tree for a HID device. */
#define CONFIG_DESC_SIZE (9 + 9+9+7+7 + 9+9+7)
static uint8_t vcom_configuration_descriptor_data[CONFIG_DESC_SIZE] = {
  /* Configuration Descriptor, //2+7 */
  USB_DESC_CONFIGURATION(CONFIG_DESC_SIZE, /* wTotalLength                  */
                         0x02,          /* bNumInterfaces.                  */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0x80,          /* bmAttributes (self powered).     */
                         250),          /* bMaxPower                        */
  /* Interface Descriptor - RawHID //2+7 */
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x02,          /* bNumEndpoints.                   */
                         0x03,          /* bInterfaceClass (HID)            */
                         0x00,          /* bInterfaceSubClass               */
                         0x00,          /* bInterfaceProtocol               */
                         0),            /* iInterface.                      */
  /* HID Interface Descriptor, HID 1.11 spec, section 6.2.1, //9 */
  USB_DESC_BYTE         (0x09),         /* bLength                          */
  USB_DESC_BYTE         (0x21),         /* bDescriptorType                  */
  USB_DESC_BCD          (0x0111),       /* bcdHID                           */
  USB_DESC_BYTE         (0x00),         /* bCountryCode                     */
  USB_DESC_BYTE         (0x01),         /* bNumDescriptors                  */
  USB_DESC_BYTE         (0x22),         /* bDescriptorType                  */
  USB_DESC_WORD         (sizeof rawhid_desc_data), /* wDescriptorLength     */
  /* Endpoint Descriptor, USB Spec 9.6.6, p269-271,Table 9-13 //7 */
  USB_DESC_ENDPOINT     (RAWHID_TX_ENDPOINT|
                          0x80  ,       /* bEndpointAddress                 */
                         0x03,          /* bmAttributes (0x03 => intr)      */
                         0x0040,        /* wMaxPacketSize                   */
                         0x20           /* bInterval                        */
                        ),
                                                           /* //7 */
  USB_DESC_ENDPOINT     (RAWHID_RX_ENDPOINT, /* bEndpointAddress            */
                         0x03,          /* bmAttributes (0x03 => intr)      */
                         0x0040,        /* wMaxPacketSize                   */
                         0x20           /* bInterval                        */
                        ),
  /* Interface Descriptor - DebugHID // 2+7 */
  USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x01,          /* bNumEndpoints.                   */
                         0x03,          /* bInterfaceClass (HID)            */
                         0x00,          /* bInterfaceSubClass               */
                         0x00,          /* bInterfaceProtocol               */
                         0),            /* iInterface.                      */
  /* HID Interface Descriptor, HID 1.11 spec, section 6.2.1 //9 */
  USB_DESC_BYTE         (0x09),         /* bLength                          */
  USB_DESC_BYTE         (0x21),         /* bDescriptorType                  */
  USB_DESC_BCD          (0x0111),       /* bcdHID                           */
  USB_DESC_BYTE         (0x00),         /* bCountryCode                     */
  USB_DESC_BYTE         (0x01),         /* bNumDescriptors                  */
  USB_DESC_BYTE         (0x22),         /* bDescriptorType                  */
  USB_DESC_WORD         (sizeof debughid_desc_data), /* wDescriptorLength   */
  /* Endpoint Descriptor, USB Spec 9.6.6, p269-271,Table 9-13 //7*/
  USB_DESC_ENDPOINT     (DEBUGHID_ENDPOINT|
                          0x80,         /* bEndpointAddress                 */
                         0x03,          /* bmAttributes (0x03 => intr =)    */
                         0x0040,        /* wMaxPacketSize                   */
                         0x20           /* bInterval                        */
                        ),
}; /* Total size: // 9 + 9+9+7+7 + 9+9+7 */

/*
 * Configuration Descriptor wrapper.
 */
static USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static uint8_t vcom_string0[] = {
  USB_DESC_BYTE(2+2*1),                 /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static uint8_t vcom_string1[] = {
  USB_DESC_BYTE(2+2*7),                 /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'N', 0, 'o', 0, 'p', 0, 'e', 0, 'L', 0, 'a', 0, 'b', 0,
};

/*
 * Device Description string.
 */
static uint8_t vcom_string2[] = {
  USB_DESC_BYTE(2+5*2),                 /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'C', 0, 'h', 0, 'T', 0, 's', 0, 'y', 0,
};

/*
 * Serial Number string.
 */
static uint8_t vcom_string3[] = {
  USB_DESC_BYTE(2+2*3),                 /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  '0' + CH_KERNEL_MAJOR, 0,
  '0' + CH_KERNEL_MINOR, 0,
  '0' + CH_KERNEL_PATCH, 0
};

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t lang) {
  (void)usbp;
  (void)lang;
  switch (dtype) {
    case USB_DESCRIPTOR_DEVICE:
//      sdPut(&SD1,'A');
      return &vcom_device_descriptor;
    case USB_DESCRIPTOR_CONFIGURATION:
//      sdPut(&SD1,'B');
      return &vcom_configuration_descriptor;
    case USB_DESCRIPTOR_STRING:
//      sdPut(&SD1,'C');
      if (dindex < 4)
        return &vcom_strings[dindex];
    case USB_DESCRIPTOR_INTERFACE:
      sdPut(&SD1,'D');
      break;
    case USB_DESCRIPTOR_ENDPOINT:
      sdPut(&SD1,'E');
      break;
    case USB_DESCRIPTOR_DEVICE_QUALIFIER:
      sdPut(&SD1,'F');
      break;
    case USB_DESCRIPTOR_OTHER_SPEED_CFG:
      sdPut(&SD1,'G');
      break;
    case USB_DESCRIPTOR_INTERFACE_POWER:
      sdPut(&SD1,'H');
      break;
    case USB_DESCRIPTOR_INTERFACE_ASSOCIATION:
      sdPut(&SD1,'I');
      break;
    case USB_REQ_GET_INTERFACE:
      sdPut(&SD1,'R');

      break;
    default:
      sdPut(&SD1,'J');
      chprintf((BaseSequentialStream *)&SD1,"?t%Xi%d",dtype,dindex);
      break;
  }
  return NULL;
}

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_hid_descriptor(USBDriver *usbp,
                                           uint8_t dtype,
                                           uint8_t dindex,
                                           uint16_t diface) {
  (void)usbp;
  (void)dindex;
  switch (dtype) {
    case 0x22:      // REPORT
//      sdPut(&SD1,'K');
//      sdPut(&SD1,'0'+diface);
      if (diface < 3)
        return &hid_descriptors[diface];
      break;
    case 0x21:      // HID
    case 0x23:      // Physical descriptor

    default:
      sdPut(&SD1,'L');
      break;
  }
  return NULL;
}


static void hidOUT(USBDriver *usbp, usbep_t ep)
{
  (void)usbp;
  (void)ep;
  sdPut(&SD1,'>');
}

static void hidIN(USBDriver *usbp, usbep_t ep)
{
  (void)usbp;
  (void)ep;
  sdPut(&SD1,'<');
}

/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1instate;

/**
 * @brief   EP1 initialization structure IN.
 */
static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  hidIN,  /* TX */
  hidOUT,
  0x0040,
  0x0000,
  &ep1instate,
  NULL,
  1,
  NULL
};

/**
 * @brief   IN EP2 state.
 */
static USBOutEndpointState ep2outstate;

/**
 * @brief   EP2 initialization structure OUT.
 */
static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  hidIN,
  hidOUT,
  0x0000,
  0x0040,
  NULL,
  &ep2outstate,
  1,
  NULL
};

/**
 * @brief   IN EP3 state.
 */
static USBInEndpointState ep3instate;

/**
 * @brief   EP1 initialization structure IN.
 */
static const USBEndpointConfig ep3config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  NULL,  /* TX */
  NULL,
  0x0040,
  0x0000,
  &ep3instate,
  NULL,
  1,
  NULL
};


/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {
//  (void)usbp;
//  sdPut(&SD1,' ');
  switch (event) {
  case USB_EVENT_RESET:
//    sdPut(&SD1,'\n');
//    sdPut(&SD1,'\r');
    return;
  case USB_EVENT_ADDRESS:
//    sdPut(&SD1,'V');
    return;
  case USB_EVENT_CONFIGURED:
//    sdPut(&SD1,'W');
    chSysLockFromISR();

    /* Enables the endpoints specified into the configuration.
       Note, this callback is invoked from an ISR so I-Class functions
       must be used.*/
    usbInitEndpointI(usbp, RAWHID_TX_ENDPOINT, &ep1config);
    usbInitEndpointI(usbp, RAWHID_RX_ENDPOINT, &ep2config);
    usbInitEndpointI(usbp, DEBUGHID_ENDPOINT, &ep3config);

    chSysUnlockFromISR();
    return;
  case USB_EVENT_SUSPEND:
    sdPut(&SD1,'X');
    return;
  case USB_EVENT_WAKEUP:
    sdPut(&SD1,'Y');
    return;
  case USB_EVENT_STALLED:
//    sdPut(&SD1,'Z');
    return;
  }
  return;
}

/**
 * @brief   Type of a requests handler callback.
 * @details The request is encoded in the @p usb_setup buffer.
 *
 * @param[in] usbp      pointer to the @p USBDriver object triggering the
 *                      callback
 * @return              The request handling exit code.
 * @retval FALSE        Request not recognized by the handler.
 * @retval TRUE         Request handled.
 */
bool hidHandlerHookCB(USBDriver *usbp)
{
//  chprintf((BaseSequentialStream *)&SD1,"=%X",*(uint16_t*)usbp->setup);
  switch(usbFetchWord(usbp->setup))
  {
    case 0x0A21: /* HID SET_IDLE - HID1_11.pdf p52, 7.2.4 */
//      sdPut(&SD1,'M');
      usbSetupTransfer(usbp, NULL, 0, NULL); /* FIXME Nothing to do yet */
      return true;
    case 0x0681: /* HID GET_DESCRIPTOR - HID1_11.pdf p49, 7.1.1 */
    {
      sdPut(&SD1,'N');
      const USBDescriptor *dp = get_hid_descriptor(usbp,
                                    usbp->setup[3], // Type
                                    usbp->setup[2], // Index
                                    usbFetchWord(&usbp->setup[4])); // Interface
      if (dp == NULL)
        return false;
      usbSetupTransfer(usbp, (uint8_t *)dp->ud_string, dp->ud_size, NULL);
      return true;
    } break;
    case 0x01A1:  /* HID GET_REPORT - HID1_11.pdf p51, 7.2.1 */
    {
//      sdPut(&SD1,'P');
      /* FIXME this gets rid of the timeouts,
       *       but it's probably not the right way of doing things
       */
//      uint8_t zero = 0;
//      usbSetupTransfer(usbp, &zero, 1, NULL);
      return true;
    } break;
    case 0x0921:   /* HID SET_REPORT - HID1_11.pdf p52, 7.2.2 */
      sdPut(&SD1,'Q');
//      usbSetupTransfer(usbp, NULL, 0, NULL); /* FIXME Nothing to do yet */
      return true;
    default:
//      chprintf((BaseSequentialStream *)&SD1,"Unk%X",*(uint16_t*)usbp->setup);
      // Let the default handler try now...
      break;
  }
  return false;
}


/*
 * USB driver configuration.
 */
static const USBConfig usbcfg = {
  usb_event,
  get_descriptor,
  hidHandlerHookCB,
  NULL
};

SerialConfig s0cfg = {
  115200
};

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  sdStart(&SD1, &s0cfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */

  usbDisconnectBus(&USBD1);
  chThdSleepMilliseconds(1000);
  usbStart(&USBD1, &usbcfg);
  usbConnectBus(&USBD1);

  const uint8_t buff[] = { 0x42 };
  while (!chThdShouldTerminateX()) {
    chThdSleepMilliseconds(1000);
    palTogglePad(IOPORT3, PORTC_TEENSY_PIN13);
    usbPrepareTransmit(&USBD1,DEBUGHID_ENDPOINT,&buff,1);
  }

  return 0;
}
