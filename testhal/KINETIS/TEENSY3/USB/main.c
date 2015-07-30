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

/*
 * USB Device Descriptor.
 */
static uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0110,        /* bcdUSB (1.1).                    */
                         0x03,          /* bDeviceClass (CDC).              */
                         0x00,          /* bDeviceSubClass.                 */
                         0x00,          /* bDeviceProtocol.                 */
                         0x40,          /* bMaxPacketSize.                  */
                         0x0179,        /* idVendor.                        */
                         0x0001,        /* idProduct.                       */
                         0x0200,        /* bcdDevice.                       */
                         0,             /* iManufacturer.                   */
                         0,             /* iProduct.                        */
                         0,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

/* Configuration Descriptor tree for a HID device. */
static uint8_t vcom_configuration_descriptor_data[18] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(18,            /* wTotalLength.                    */
                         0x01,          /* bNumInterfaces.                  */
                         0x01,          /* bConfigurationValue.             */
                         0,             /* iConfiguration.                  */
                         0x80,          /* bmAttributes (self powered).     */
                         250),           /* bMaxPower                       */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                         0x00,          /* bAlternateSetting.               */
                         0x00,          /* bNumEndpoints.                   */
                         0xFF,          /* bInterfaceClass                  */
                         0x00,          /* bInterfaceSubClass               */
                         0x00,          /* bInterfaceProtocol               */
                         0),            /* iInterface.                      */
};

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
  USB_DESC_BYTE(4),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static uint8_t vcom_string1[] = {
  USB_DESC_BYTE(7*2+2),                 /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'N', 0, 'o', 0, 'p', 0, 'e', 0, 'L', 0, 'a', 0, 'b', 0,
};

/*
 * Device Description string.
 */
static uint8_t vcom_string2[] = {
  USB_DESC_BYTE(2+2*5),                     /* bLength.                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
  'C', 0, 'h', 0, 'T', 0, 's', 0, 'y', 0,
};

/*
 * Serial Number string.
 */
static uint8_t vcom_string3[] = {
  USB_DESC_BYTE(8),                     /* bLength.                         */
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
//   sdPut(&SD1,' ');
  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    sdPut(&SD1,'A');
    return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    sdPut(&SD1,'B');
    return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    sdPut(&SD1,'C');
    if (dindex < 4)
      return &vcom_strings[dindex];
  }
  return NULL;
}

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {
  (void)usbp;
  sdPut(&SD1,' ');
  switch (event) {
  case USB_EVENT_RESET:
    sdPut(&SD1,'\n');
    sdPut(&SD1,'\r');
    return;
  case USB_EVENT_ADDRESS:
    sdPut(&SD1,'E');
    return;
  case USB_EVENT_CONFIGURED:
    sdPut(&SD1,'F');
    return;
  case USB_EVENT_SUSPEND:
    sdPut(&SD1,'G');
    return;
  case USB_EVENT_WAKEUP:
    sdPut(&SD1,'H');
    return;
  case USB_EVENT_STALLED:
    sdPut(&SD1,'I');
    return;
  }
  return;
}

/*
 * USB driver configuration.
 */
static const USBConfig usbcfg = {
  usb_event,
  get_descriptor,
  NULL,
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

  while (!chThdShouldTerminateX()) {
    chThdSleepMilliseconds(1000);
    palTogglePad(IOPORT3, PORTC_TEENSY_PIN13);
  }

  return 0;
}
