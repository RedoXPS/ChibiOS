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
static struct {
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
  64,
  64,
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

/* see p.891 */
#define BDT_PID_OUT   0x01
#define BDT_PID_IN    0x09
#define BDT_PID_SETUP 0x0D
#define BDT_TOK_PID(n)	(((n)>>2)&0xF)

#define DATA0 0
#define DATA1 1

#define RX   0
#define TX   1

#define EVEN 0
#define ODD  1

#define BDT_INDEX(endpoint, tx, odd) (((endpoint)<<2) | ((tx)<<1) | (odd))

#define BDT_BC(n) (((n)>>16)&0x3FF)

/* The USB-FS needs 2 BDT entry per endpoint direction
 *    that adds to: 2*2*16 BDT entries for 16 bi-directional EP
 */
static volatile bdt_t _bdt[(USB_MAX_ENDPOINTS+1)*2*2] __attribute__((aligned(512)));

/* FIXME later with dyn alloc
 * 16 EP
 *  2 directions per EP
 *  2 buffer per direction
 * => 64 buffers
 */
static uint8_t _usbb[(USB_MAX_ENDPOINTS+1)*4][64] __attribute__((aligned(4)));
static volatile uint8_t _usbbn=0;
uint8_t* usb_alloc(uint8_t size)
{
  (void)size;
  if(_usbbn < (USB_MAX_ENDPOINTS+1)*4)
    return _usbb[_usbbn++];
  sdPut(&SD1,'z');
  while(1); /* Should not happen, ever */
}
/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

void usb_packet_transmit(USBDriver *usbp, usbep_t ep, size_t n)
{
  const USBEndpointConfig *epc = usbp->epc[ep];

  bdt_t *bd = (bdt_t *)&_bdt[BDT_INDEX(ep, TX, epc->in_state->odd_even)];

  USBInEndpointState *isp = epc->in_state;

  if (n > (size_t)epc->in_maxsize)
    n = (size_t)epc->in_maxsize;

//  sdPut(&SD1,'0'+ep);
//  chprintf((BaseSequentialStream *)&SD1,"tx%d/%d in%d out%d",n,epc->in_maxsize, epc->in_state->data_bank, epc->out_state->data_bank);
  if (isp->txqueued)
  {
//    sdPut(&SD1,'>');
    output_queue_t *oq = epc->in_state->mode.queue.txqueue;
    /* Copy from queue to _usbb[] */
    size_t i;
    for(i=0;i<n;i++)
    {
      bd->addr[i] = *oq->q_rdptr++;
      if (oq->q_rdptr >= oq->q_top)
        oq->q_rdptr = oq->q_buffer;
    }
    /* Updating queue.*/
    syssts_t sts = osalSysGetStatusAndLockX();
    oq->q_counter += n;
    osalThreadDequeueAllI(&oq->q_waiting, Q_OK);
    osalSysRestoreStatusX(sts);
  }
  else
  {
//     sdPut(&SD1,'y');
    /* Copy from buf to _usbb[] */
    size_t i=0;
    for(i=0;i<n;i++)
      bd->addr[i] = isp->mode.linear.txbuf[i];
  }

  /* Update the Buffer status */
  bd->desc = BDT_DESC(n, epc->in_state->data_bank);
  /* Toggle the odd and data bits for next TX */
  epc->in_state->data_bank ^= DATA1;
  epc->in_state->odd_even ^= ODD;

  osalSysLockFromISR();
  usb_lld_start_in(usbp, ep);
  osalSysUnlockFromISR();
}

void usb_packet_receive(USBDriver *usbp, usbep_t ep, size_t n)
{
  const USBEndpointConfig *epc = usbp->epc[ep];
  USBOutEndpointState *osp = epc->out_state;

  bdt_t *bd = (bdt_t *)&_bdt[BDT_INDEX(ep, RX, epc->out_state->odd_even)];

  if (n > (size_t)epc->out_maxsize)
    n = (size_t)epc->out_maxsize;

//    chprintf((BaseSequentialStream *)&SD1," rx%d/%d in%d out%d", n,epc->out_maxsize, epc->in_state->data_bank, epc->out_state->data_bank);
  if (osp->rxqueued)
  {
//    sdPut(&SD1,'<');
    input_queue_t *iq = osp->mode.queue.rxqueue;
    /* Copy from _usbb[] to queue */
    size_t i;
    for(i=0;i<n;i++)
    {
      *iq->q_wrptr++ = bd->addr[i];
      if (iq->q_wrptr >= iq->q_top)
        iq->q_wrptr = iq->q_buffer;
    }
    /* Updating queue.*/
    osalSysLockFromISR();
    iq->q_counter += n;
    osalThreadDequeueAllI(&iq->q_waiting, Q_OK);
    osalSysUnlockFromISR();
  }
  else
  {
//     sdPut(&SD1,'y');
    /* Copy from _usbb[] to buf  */
    size_t i=0;
    for(i=0;i<n;i++)
      osp->mode.linear.rxbuf[i] = bd->addr[i];
  }

  /* Update the Buffer status */
  epc->out_state->data_bank ^= DATA1;//(bd->desc & 8) ? DATA1 : DATA0;
//  sdPut(&SD1,'.');
//  sdPut(&SD1,'0'+epc->out_state->data_bank);
  bd->desc = BDT_DESC(epc->out_maxsize, epc->out_state->data_bank);

  osalSysLockFromISR();
  usb_lld_start_out(usbp, ep);
  osalSysUnlockFromISR();
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*============================================================================*/

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
    USBOTG->ISTAT = USBx_INTEN_SOFTOKEN;
  }
  /* 08 - Bit3 - Token processing completed */
  while(istat & USBx_ISTAT_TOKDNE) {
//     sdPut(&SD1,'b');
    uint8_t stat = USBOTG->STAT;
    uint8_t ep = stat >> 4;
    if(ep > USB_MAX_ENDPOINTS) {
      sdPut(&SD1,'=');
      return;
    }
    const USBEndpointConfig *epc = usbp->epc[ep];

    /* Get the correct BDT entry */
    uint8_t odd_even = (stat & USBx_STAT_ODD_MASK) >> USBx_STAT_ODD_SHIFT;
    uint8_t tx_rx    = (stat & USBx_STAT_TX_MASK) >> USBx_STAT_TX_SHIFT;
    bd_t *bd = (bd_t*)&_bdt[BDT_INDEX(ep,tx_rx,odd_even)];

    /* Update the ODD/EVEN state for RX */
    if(tx_rx == RX && epc->out_state != NULL)
      epc->out_state->odd_even = odd_even;

//    sdPut(&SD1,' ');
//    sdPut(&SD1,'0'+ep);
    switch(BDT_TOK_PID(bd->desc))
    {
      case BDT_PID_SETUP:                                              // SETUP
      {
//        sdPut(&SD1,'s');
        /* Clear any pending IN stuff */
        _bdt[BDT_INDEX(ep, TX, EVEN)].desc = 0;
        _bdt[BDT_INDEX(ep, TX,  ODD)].desc = 0;
        /* After a SETUP, IN is always DATA1 */
        usbp->epc[ep]->in_state->data_bank = DATA1;

        /* Call SETUP function (ChibiOS core), which sends back stuff */
        _usb_isr_invoke_setup_cb(usbp, ep);
        /* Release Buffer */
        epc->out_state->data_bank ^= DATA1;
        bd->desc = BDT_DESC(epc->out_maxsize,DATA1);
      } break;
      case BDT_PID_IN:                                                 // IN
      {
        if(epc->in_state == NULL)
          break;
        /* Special case for SetAddress for EP0 */
        if(ep == 0 && usbFetchWord(usbp->setup) == 0x0500)
        {
//          sdPut(&SD1,'_');
          usbp->address = usbp->setup[2];
          usb_lld_set_address(usbp);
          _usb_isr_invoke_event_cb(usbp, USB_EVENT_ADDRESS);
          usbp->state = USB_SELECTED;
        }
//        sdPut(&SD1,';');
        uint16_t txed = BDT_BC(bd->desc);
        epc->in_state->txcnt += txed;
        if(epc->in_state->txcnt < epc->in_state->txsize)
        {
//          sdPut(&SD1,'+');
          if (!epc->in_state->txqueued)
            epc->in_state->mode.linear.txbuf += txed;
          usb_packet_transmit(usbp,ep,epc->in_state->txsize - epc->in_state->txcnt);
        }
        else
          if(epc->in_cb != NULL)
          _usb_isr_invoke_in_cb(usbp,ep);
      } break;
      case BDT_PID_OUT:                                                // OUT
      {
        if(epc->out_state == NULL)
          break;
//        sdPut(&SD1,':');
        uint16_t rxed = BDT_BC(bd->desc);

        usb_packet_receive(usbp,ep,rxed);
        if(rxed)
        {
          if(!epc->out_state->rxqueued)
            epc->out_state->mode.linear.rxbuf += rxed;

          /* Update transaction data */
          epc->out_state->rxcnt              += rxed;
          epc->out_state->rxsize             -= rxed;
          epc->out_state->rxpkts             -= 1;

          /* The transaction is completed if the specified number of packets
             has been received or the current packet is a short packet.*/
          if ((rxed < epc->out_maxsize) || (epc->out_state->rxpkts == 0))
            if(epc->out_cb != NULL)
              _usb_isr_invoke_out_cb(usbp, ep);
        }
      } break;
      default:
        sdPut(&SD1,'$');
        break;
    }
    USBOTG->ISTAT = USBx_ISTAT_TOKDNE;
    USBOTG->CTL = USBx_CTL_USBENSOFEN;
    istat = USBOTG->ISTAT;
  }
  /* 01 - Bit0 - Valid USB Reset received */
  if(istat & USBx_ISTAT_USBRST) {
//     sdPut(&SD1,'c');
    _usb_reset(usbp);
    _usb_isr_invoke_event_cb(usbp, USB_EVENT_RESET);
    USBOTG->ISTAT = USBx_ISTAT_USBRST;
    return;
  }
  /* 80 - Bit7 - STALL handshake received */
  if(istat & USBx_ISTAT_STALL) {
//    sdPut(&SD1,'d');
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
//    sdPut(&SD1,'f');
    USBOTG->ISTAT = USBx_ISTAT_SLEEP;
  }
  /* 20 - Bit5 and 40 - 6 are not used */

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

  #if KINETIS_USB_USE_USB0
  SIM->SOPT2 |= SIM_SOPT2_USBSRC;
  #define KINETIS_USBCLK_FREQUENCY 48000000UL
  /* Default assume a 96MHz PLL */
  SIM->CLKDIV2 = SIM_CLKDIV2_USBDIV(1);
  uint32_t i,j;
  for(i = 0; i < 2; i++)
  {
    for(j=0; j < 8; j++)
    {
      if((KINETIS_PLLCLK_FREQUENCY * (i+1)) == (KINETIS_USBCLK_FREQUENCY*(j+1)))
      {
        SIM->CLKDIV2 = i | SIM_CLKDIV2_USBDIV(j);
        break;
      }
    }
  }
  chDbgAssert(i<2 && j <8,"USB Init error");
  #endif
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
//      chprintf((BaseSequentialStream *)&SD1,"uStart\r\n");

      /* Clear BDT */
      uint8_t i;
      for(i=0;i<USB_MAX_ENDPOINTS+1;i++) {
        _bdt[i].desc=0;
        _bdt[i].addr=0;
      }

      SIM->SCGC4 |= SIM_SCGC4_USBOTG;  /* Enable Clock */

      /* Reset USB module, wait for completion */
      USBOTG->USBTRC0 = USBx_USBTRC0_USBRESET;
      while ((USBOTG->USBTRC0 & USBx_USBTRC0_USBRESET));

      /* Set BDT Address */
      USBOTG->BDTPAGE1 = ((uint32_t)_bdt) >> 8;
      USBOTG->BDTPAGE2 = ((uint32_t)_bdt) >> 16;
      USBOTG->BDTPAGE3 = ((uint32_t)_bdt) >> 24;

      /* Clear all ISR flags */
      USBOTG->ISTAT = 0xFF;
      USBOTG->ERRSTAT = 0xFF;
      USBOTG->OTGISTAT = 0xFF;

      /* Enable USB */
      USBOTG->CTL = USBx_CTL_ODDRST | USBx_CTL_USBENSOFEN;
      USBOTG->USBCTRL = 0;

      /* Enable reset interrupt */
      USBOTG->INTEN = USBx_INTEN_USBRSTEN;

      /* Enable interrupt in NVIC */
      nvicEnableVector(USB_OTG_IRQn, KINETIS_USB_USB0_IRQ_PRIORITY);

      /* Enable D+ pullup */
      USBOTG->CONTROL = USBx_CONTROL_DPPULLUPNONOTG;
    }
#endif
  }
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
  // FIXME, dyn alloc
  _usbbn = 0;
//   sdPut(&SD1,'#');

  /* Reset BDT ODD/EVEN bits */
  USBOTG->CTL = USBx_CTL_ODDRST;

  /* EP0 initialization.*/
  usbp->epc[0] = &ep0config;
  usb_lld_init_endpoint(usbp, 0);

  /* Clear all pending interrupts */
  USBOTG->ERRSTAT = 0xFF;
  USBOTG->ISTAT = 0xFF;

  /* Set the address to zero during enumeration */
  usbp->address = 0;
  USBOTG->ADDR = 0;

  /* Enable other interrupts */
  USBOTG->ERREN = 0xFF;
  USBOTG->INTEN = USBx_INTEN_TOKDNEEN |
    USBx_INTEN_SOFTOKEN |
    USBx_INTEN_STALLEN |
    USBx_INTEN_ERROREN |
    USBx_INTEN_USBRSTEN |
    USBx_INTEN_SLEEPEN;

  /* "is this necessary?", Paul from PJRC */
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
//   sdPut(&SD1,'g');
//  chprintf((BaseSequentialStream *)&SD1,"%d",usbp->address);
  USBOTG->ADDR = usbp->address&0x7F;
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
//   sdPut(&SD1,'h');

  const USBEndpointConfig *epc = usbp->epc[ep];
  uint8_t mask=0;

  if(epc->out_state != NULL)
  {
    /* OUT Endpoint */
    epc->out_state->odd_even = EVEN;
    epc->out_state->data_bank = DATA0;
    /* RXe */
    _bdt[BDT_INDEX(ep, RX, EVEN)].desc = BDT_DESC(epc->out_maxsize, DATA0);
    _bdt[BDT_INDEX(ep, RX, EVEN)].addr = usb_alloc(epc->out_maxsize);
    /* RXo */
    _bdt[BDT_INDEX(ep, RX,  ODD)].desc = BDT_DESC(epc->out_maxsize, DATA0);
    _bdt[BDT_INDEX(ep, RX,  ODD)].addr = usb_alloc(epc->out_maxsize);
    /* Enable OUT direction */
    mask |= USBx_ENDPTn_EPRXEN;
  }
  if(epc->in_state != NULL)
  {
    /* IN Endpoint */
    epc->in_state->odd_even = EVEN;
    epc->in_state->data_bank = DATA0;
    /* TXe, not used yet */
    _bdt[BDT_INDEX(ep, TX, EVEN)].desc = 0;
    _bdt[BDT_INDEX(ep, TX, EVEN)].addr = usb_alloc(epc->in_maxsize);
    /* TXo, not used yet */
    _bdt[BDT_INDEX(ep, TX,  ODD)].desc = 0;
    _bdt[BDT_INDEX(ep, TX,  ODD)].addr = usb_alloc(epc->in_maxsize);
    /* Enable IN direction */
    mask |= USBx_ENDPTn_EPTXEN;
  }

  /* EPHSHK should be set for CTRL, BULK, INTR not for ISOC*/
  if((epc->ep_mode & USB_EP_MODE_TYPE) != USB_EP_MODE_TYPE_ISOC)
    mask |= USBx_ENDPTn_EPHSHK;
  /* Endpoint is not a CTRL endpoint, disable SETUP transfers */
  if((epc->ep_mode & USB_EP_MODE_TYPE) != USB_EP_MODE_TYPE_CTRL)
    mask |= USBx_ENDPTn_EPCTLDIS;

  USBOTG->ENDPT[ep].V = mask;
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
  sdPut(&SD1,'i');
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
  sdPut(&SD1,'j');

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
  sdPut(&SD1,'k');

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
//    sdPut(&SD1,'l');
  /* Get the BDT entry */
  USBOutEndpointState *os = usbp->epc[ep]->out_state;
  bdt_t *bd = (bdt_t*)&_bdt[BDT_INDEX(ep, RX, os->odd_even)];
  /* Copy the 8 bytes of data */
  uint8_t n;
  for (n = 0; n < 8; n++) {
    buf[n] = bd->addr[n];
  }
  /* Release the buffer */
  os->data_bank ^= DATA1;
  bd->desc = BDT_DESC(usbp->epc[ep]->out_maxsize,os->data_bank);
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
//  sdPut(&SD1,'m');
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
//   sdPut(&SD1,'n');
  /* Transfer initialization.*/
  usb_packet_transmit(usbp,ep,usbp->epc[ep]->in_state->txsize);
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
//  sdPut(&SD1,'o');
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
//   sdPut(&SD1,'p');
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
//  sdPut(&SD1,'q');
  USBOTG->ENDPT[ep].V |= USBx_ENDPTn_EPSTALL;
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
//  sdPut(&SD1,'r');
  USBOTG->ENDPT[ep].V |= USBx_ENDPTn_EPSTALL;
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
  sdPut(&SD1,'s');
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
  sdPut(&SD1,'t');
}

#endif /* HAL_USE_USB */

/** @} */
