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

/**
 * @file    KINETIS/K20x/usb_lld.c
 * @brief   KINETIS USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include <string.h>

#include "hal.h"
#include "chprintf.h"

#if HAL_USE_USB || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USB0 driver identifier.*/
#if KINETIS_USB_USE_USB0 || defined(__DOXYGEN__)
USBDriver USBD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   EP0 state.
 * @note    It is an union because IN and OUT endpoints are never used at the
 *          same time for EP0.
 */
static union {
  /**
   * @brief   IN EP0 state.
   */
  USBInEndpointState in;
  /**
   * @brief   OUT EP0 state.
   */
  USBOutEndpointState out;
} ep0_state;

/**
 * @brief   Buffer for the EP0 setup packets.
 */
static uint8_t ep0setup_buffer[8];

/**
 * @brief   EP0 initialization structure.
 */
static const USBEndpointConfig ep0config = {
  USB_EP_MODE_TYPE_CTRL,
  _usb_ep0setup,
  _usb_ep0in,
  _usb_ep0out,
  0x40,
  0x40,
  &ep0_state.in,
  &ep0_state.out,
  1,
  ep0setup_buffer
};

/* Buffer Descriptor Table (BDT) */
typedef struct {
	uint32_t desc;
	uint8_t* addr;
} bdt_t;
/* BDT Description entry (p.889) */
#define BDT_OWN		0x80
#define BDT_DATA  0x40
#define BDT_KEEP  0x20
#define BDT_NINC  0x10
#define BDT_DTS		0x08
#define BDT_STALL	0x04

#define BDT_DESC(bc, data)	(BDT_OWN | BDT_DTS | ((data&0x1)<<6) | ((bc) << 16))
#define BDT_DESC_DATA0  0
#define BDT_DESC_DATA1  1

/* see p.891 */
#define BDT_TOK_PID(n)	(((n) >> 2) & 15)
#define TOK_PID_SETUP   0x0D
#define TOK_PID_IN      0x09
#define TOK_PID_OUT     0x01


#define TX   1
#define RX   0
#define ODD  1
#define EVEN 0
#define BDT_INDEX(endpoint, tx, odd) (((endpoint) << 2) | ((tx) << 1) | (odd))

/* The USB-FS needs 2 BDT entry per endpoint direction
 *    that adds to: 2*2*16 BDT entries for 16 bi-directional EP
 */
static volatile bdt_t _bdt[(USB_MAX_ENDPOINTS+1)*2*2] __attribute__((aligned(512)));

/* FIXME later with dyn alloc */
static uint8_t _usbb[USB_MAX_ENDPOINTS][64] __attribute__((aligned(4)));
static volatile uint8_t _usbbn=0;
uint8_t* usb_alloc(uint8_t size)
{
  (void)size;
  if(_usbbn < USB_MAX_ENDPOINTS)
    return _usbb[_usbbn++];
  sdPut(&SD1,'z');
  while(1); /* Should not happen, ever */
}
/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/


/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*============================================================================*/

typedef struct {
  union {
     struct {
      uint8_t bmRequestType;
      uint8_t bRequest;
    };
    uint16_t wRequestAndType;
  };
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} usb_setup_t;

typedef union {
  usb_setup_t setup;
  struct {
    uint32_t word1;
    uint32_t word2;
  } raw;
} usb_setup_raw_t;

// static usb_setup_raw_t usb_setup_;


#define USB_SET_ADDRESS       0x0500
#define USB_SET_CONFIGURATION 0x0900
#define USB_GET_CONFIGURATION 0x0880
#define USB_GET_STATUS_DEVICE 0x0080
#define USB_GET_STATUS_ENDPT  0x0082
#define USB_CLEAR_FEATURE     0x0102
#define USB_SET_FEATURE       0x0302
#define USB_GET_DESCRIPTOR    0x0680
#define USB_GET_DESCRIPTOR2   0x0681


#if KINETIS_USB_USE_USB0 || defined(__DOXYGEN__)
/**
 * @brief   USB low priority interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(KINETIS_USB_IRQ_VECTOR) {
  USBDriver *usbp = &USBD1;
  uint8_t istat = USBOTG->ISTAT;

  OSAL_IRQ_PROLOGUE();
  /* 04 - Bit2 - Start Of Frame token received */
  if(istat & USBx_INTEN_SOFTOKEN) {
//     sdPut(&SD1,'a');
    _usb_isr_invoke_sof_cb(usbp);
    // do stuff, eventually
    USBOTG->ISTAT = USBx_INTEN_SOFTOKEN;
  }
  /* 08 - Bit3 - Token processing completed */
  if(istat & USBx_ISTAT_TOKDNE) {
//     sdPut(&SD1,'b');
    uint8_t stat = USBOTG->STAT;
    uint8_t ep = stat >>4;
    if(ep > USB_MAX_ENDPOINTS)
      return;
//     const USBEndpointConfig *epc = usbp->epc[ep];
//     bdt_t *bdte;

//     sdPut(&SD1,'0'+ep);
    switch(ep)
    {
      // USB Control
      case 0:
        _usb_isr_invoke_setup_cb(usbp, ep);
        USBOTG->ISTAT = USBx_ISTAT_TOKDNE;
        break;
//     USBOTG->ISTAT = USBx_ISTAT_TOKDNE;
    }
  }
  /* 01 - Bit0 - Valid USB Reset received */
  if(istat & USBx_ISTAT_USBRST) {
    _usb_reset(usbp);
    _usb_isr_invoke_event_cb(usbp, USB_EVENT_RESET);
    //~ USBOTG->ISTAT = USBx_ISTAT_USBRST;
    return;
  }
  /* 80 - Bit7 - STALL handshake received */
  if(istat & USBx_ISTAT_STALL) {
    sdPut(&SD1,'d');
    USBOTG->ENDPT[0].V = USBx_ENDPTn_EPRXEN | USBx_ENDPTn_EPTXEN | USBx_ENDPTn_EPHSHK;
    USBOTG->ISTAT = USBx_ISTAT_STALL;
  }
  /* 02 - Bit1 - ERRSTAT condition triggered */
  if(istat & USBx_ISTAT_ERROR) {
    sdPut(&SD1,'e');
    uint8_t err = USBOTG->ERRSTAT;
    USBOTG->ERRSTAT = err;
    USBOTG->ISTAT = USBx_ISTAT_ERROR;
  }
  /* 10 - Bit4 - Constant IDLE on USB bus detected */
  if(istat & USBx_ISTAT_SLEEP) {
    sdPut(&SD1,'f');
    USBOTG->ISTAT = USBx_ISTAT_SLEEP;
  }
  /* 20 - Bit5 and 40 - 6 are not used */
  //~ sdPut(&SD1,'');
  OSAL_IRQ_EPILOGUE();
}
#endif /* KINETIS_USB_USE_USB0 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {
  /* Driver initialization.*/
  usbObjectInit(&USBD1);
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp) {
  if (usbp->state == USB_STOP) {
    /* Clock activation.*/
#if KINETIS_USB_USE_USB0
    if (&USBD1 == usbp) {
      chprintf((BaseSequentialStream *)&SD1,"uStart\r\n");

      /* Clear BDT */
      uint8_t i;
      for(i=0;i<USB_MAX_ENDPOINTS+1;i++) {
        _bdt[i].desc=0;
        _bdt[i].addr=0;
      }

      SIM->SCGC4 |= SIM_SCGC4_USBOTG;  /* Enable Clock */
      /* Reset USB module */
      USBOTG->USBTRC0 = USBx_USBTRC0_USBRESET;
      while ((USBOTG->USBTRC0 & USBx_USBTRC0_USBRESET) != 0) ; // wait for reset to end

      // set desc table base addr
      USBOTG->BDTPAGE1 = ((uint32_t)_bdt) >> 8;
      USBOTG->BDTPAGE2 = ((uint32_t)_bdt) >> 16;
      USBOTG->BDTPAGE3 = ((uint32_t)_bdt) >> 24;

      // clear all ISR flags
      USBOTG->ISTAT = 0xFF;
      USBOTG->ERRSTAT = 0xFF;
      USBOTG->OTGISTAT = 0xFF;

      USBOTG->USBTRC0 |= 0x40; // undocumented bit

      // enable USB
      USBOTG->CTL = USBx_CTL_USBENSOFEN;
      USBOTG->USBCTRL = 0;

      // enable reset interrupt
      USBOTG->INTEN = USBx_INTEN_USBRSTEN;

      // enable interrupt in NVIC...
      nvicEnableVector(USB_OTG_IRQn, KINETIS_USB_USB0_IRQ_PRIORITY);

      // enable d+ pullup
      USBOTG->CONTROL = USBx_CONTROL_DPPULLUPNONOTG;
    }
#endif
  }
  /* Configuration.*/
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {
  /* If in ready state then disables the USB clock.*/
  if (usbp->state == USB_STOP) {
#if KINETIS_USB_USE_USB0
    if (&USBD1 == usbp) {
      chprintf((BaseSequentialStream *)&SD1,"uStop\r\n");
      nvicDisableVector(USB_OTG_IRQn);
    }
#endif
  }
}

/**
 * @brief   USB low level reset routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_reset(USBDriver *usbp) {
  // FIXME
  _usbbn = 0;
  sdPut(&SD1,'#');

  // initialize BDT toggle bits
  USBOTG->CTL = USBx_CTL_ODDRST;

  /* EP0 initialization.*/
  usbp->epc[0] = &ep0config;
  usb_lld_init_endpoint(usbp, 0);

  // clear all ending interrupts
  USBOTG->ERRSTAT = 0xFF;
  USBOTG->ISTAT = 0xFF;

  // set the address to zero during enumeration
  usbp->address = 0;
  usb_lld_set_address(usbp);

  // enable other interrupts
  USBOTG->ERREN = 0xFF;
  USBOTG->INTEN = USBx_INTEN_TOKDNEEN |
    USBx_INTEN_SOFTOKEN |
    USBx_INTEN_STALLEN |
    USBx_INTEN_ERROREN |
    USBx_INTEN_USBRSTEN |
    USBx_INTEN_SLEEPEN;

  // is this necessary?
  USBOTG->CTL = USBx_CTL_USBENSOFEN;
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {
  (void)usbp;
  USBOTG->ADDR = usbp->address;
}

/**
 * @brief   Enables an endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_init_endpoint(USBDriver *usbp, usbep_t ep) {
  (void)usbp;

  /* FIXME: Only works for EP0 */

  /* RXe */
  _bdt[BDT_INDEX(ep, RX, EVEN)].desc = BDT_DESC(usbp->epc[ep]->in_maxsize, BDT_DESC_DATA0);
  _bdt[BDT_INDEX(ep, RX, EVEN)].addr = usb_alloc(usbp->epc[ep]->in_maxsize);
  /* RXo */
  _bdt[BDT_INDEX(ep, RX,  ODD)].desc = BDT_DESC(usbp->epc[ep]->in_maxsize, BDT_DESC_DATA0);
  _bdt[BDT_INDEX(ep, RX,  ODD)].addr = usb_alloc(usbp->epc[ep]->in_maxsize);
  /* TXe, not used yet */
  _bdt[BDT_INDEX(ep, TX, EVEN)].desc = 0;
  /* TXo, not used yet */
  _bdt[BDT_INDEX(ep, TX,  ODD)].desc = 0;

  /* Activate endpoint */
  USBOTG->ENDPT[ep].V = USBx_ENDPTn_EPRXEN | USBx_ENDPTn_EPTXEN | USBx_ENDPTn_EPHSHK;
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {
  (void)usbp;
  sdPut(&SD1,'p');
}

/**
 * @brief   Returns the status of an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;
    sdPut(&SD1,'q');

  return EP_STATUS_DISABLED;
}

/**
 * @brief   Returns the status of an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_in(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;
  sdPut(&SD1,'r');

  return EP_STATUS_DISABLED;
}

/**
 * @brief   Reads a setup packet from the dedicated packet buffer.
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to read the received setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @post    The endpoint is ready to accept another packet.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 *
 * @notapi
 */
void usb_lld_read_setup(USBDriver *usbp, usbep_t ep, uint8_t *buf) {
  (void)usbp;
  (void)ep;
  (void)buf;
  sdPut(&SD1,'s');

//   pmap = USB_ADDR2PTR(udp->RXADDR0);
/*  for (n = 0; n < 4; n++) {
    *(uint16_t *)buf = (uint16_t)*pmap++;
    buf += 2;
  }
  */
}

/**
 * @brief   Prepares for a receive operation.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_prepare_receive(USBDriver *usbp, usbep_t ep) {
  USBOutEndpointState *osp = usbp->epc[ep]->out_state;
  sdPut(&SD1,'t');
  /* Transfer initialization.*/
  if (osp->rxsize == 0)         /* Special case for zero sized packets.*/
    osp->rxpkts = 1;
  else
    osp->rxpkts = (uint16_t)((osp->rxsize + usbp->epc[ep]->out_maxsize - 1) /
                             usbp->epc[ep]->out_maxsize);
}

/**
 * @brief   Prepares for a transmit operation.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_prepare_transmit(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;
//   size_t n;
//   USBInEndpointState *isp = usbp->epc[ep]->in_state;
  sdPut(&SD1,'u');
  /* Transfer initialization.*/
  /*n = isp->txsize;
  if (n > (size_t)usbp->epc[ep]->in_maxsize)
    n = (size_t)usbp->epc[ep]->in_maxsize;
  */
  /*
  if (isp->txqueued)
    usb_packet_write_from_queue(USB_GET_DESCRIPTOR(ep),
                                isp->mode.queue.txqueue, n);
  else
    usb_packet_write_from_buffer(USB_GET_DESCRIPTOR(ep),
                                 isp->mode.linear.txbuf, n);
  */
}

/**
 * @brief   Starts a receive operation on an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_out(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;
  sdPut(&SD1,'v');
}

/**
 * @brief   Starts a transmit operation on an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_in(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;
  sdPut(&SD1,'w');
}

/**
 * @brief   Brings an OUT endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_out(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;
  sdPut(&SD1,'x');
}

/**
 * @brief   Brings an IN endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_in(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;
  sdPut(&SD1,'y');
}

/**
 * @brief   Brings an OUT endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_out(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;

}

/**
 * @brief   Brings an IN endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_in(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;

}

#endif /* HAL_USE_USB */

/** @} */
