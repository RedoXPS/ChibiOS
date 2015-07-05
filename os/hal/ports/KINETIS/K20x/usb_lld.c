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
	void * addr;
} bdt_t;

static bdt_t _bdt[(USB_MAX_ENDPOINTS+1)*4];


/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/


/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if KINETIS_USB_USE_USB0 || defined(__DOXYGEN__)
/**
 * @brief   USB low priority interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(KINETIS_USB_IRQ_VECTOR) {
  //~ USBDriver *usbp = &USBD1;

  OSAL_IRQ_PROLOGUE();
  chprintf((BaseSequentialStream *)&SD1,"uISR %X\r\n",USBOTG->ISTAT);
  USBOTG->ISTAT |= 0xFF;

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

      /* Configure USB for 48 MHz clock */
      SIM->CLKDIV2 = SIM_CLKDIV2_USBDIV(1); // USB = 96 MHz PLL / 2
      /* USB uses PLL clock, trace is CPU clock, CLKOUT=OSCERCLK0 */
      SIM->SOPT2 = SIM_SOPT2_USBSRC | SIM_SOPT2_PLLFLLSEL | SIM_SOPT2_TRACECLKSEL | SIM_SOPT2_CLKOUTSEL(6);

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
    /* Reset procedure enforced on driver start.*/
    _usb_reset(usbp);
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
  chprintf((BaseSequentialStream *)&SD1,"uReset\r\n");

  /* EP0 initialization.*/
  usbp->epc[0] = &ep0config;
  usb_lld_init_endpoint(usbp, 0);
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
  (void)ep;
  chprintf((BaseSequentialStream *)&SD1,"uInitEP%d\r\n",ep);

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
  size_t n;
  USBInEndpointState *isp = usbp->epc[ep]->in_state;

  /* Transfer initialization.*/
  n = isp->txsize;
  if (n > (size_t)usbp->epc[ep]->in_maxsize)
    n = (size_t)usbp->epc[ep]->in_maxsize;

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
