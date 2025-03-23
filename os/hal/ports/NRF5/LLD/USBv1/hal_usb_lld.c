/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
 * @file    hal_usb_lld.c
 * @brief   PLATFORM USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include "hal.h"

#if (HAL_USE_USB == TRUE) || defined(__DOXYGEN__)

#include <string.h>

#include <nrf_mem.h>

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   USB1 driver identifier.
 */
#if (NRF5_USB_USE_USB1 == TRUE) || defined(__DOXYGEN__)
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
  &ep0_state.out
};

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static inline void handle_in_packet(USBDriver *usbp, usbep_t ep) {
  const USBEndpointConfig *epcp = usbp->epc[ep];
  USBInEndpointState *isp = epcp->in_state;

  size_t transferred = usbp->usb->EPIN[ep].AMOUNT;
  isp->txcnt += transferred;
  isp->txbuf += transferred;
  isp->txsize = (transferred < isp->txsize) ? (isp->txsize - transferred) : 0u;

  if(isp->txsize > 0u) {
    usb_lld_start_in(usbp, ep);
  }
  else {
    _usb_isr_invoke_in_cb(usbp, ep);
  }
}

static inline void handle_out_packet(USBDriver *usbp, usbep_t ep) {

  const USBEndpointConfig *epcp = usbp->epc[ep];
  USBOutEndpointState *osp = epcp->out_state;

  size_t transferred = usbp->usb->EPOUT[ep].AMOUNT;

  uint32_t addr = (uint32_t) osp->rxbuf;
  if((addr < NRF_MEMORY_RAM_BASE) || (addr >= NRF_MEMORY_RAM_BASE + NRF_MEMORY_RAM_SIZE)) {
    memcpy(osp->rxbuf, usbp->dma_buffer, transferred);
  }

  osp->rxcnt += transferred;
  osp->rxbuf += transferred;
  osp->rxsize = (transferred < osp->rxsize) ? (osp->rxsize - transferred) : 0u;

  if(osp->rxsize > 0u) {
    usb_lld_start_out(usbp, ep);
  }
  else {
    usbp->usb->SHORTS &= ~USBD_SHORTS_EP0DATADONE_STARTEPOUT0_Msk;
    _usb_isr_invoke_out_cb(usbp, ep);
  }
}

static inline void handle_interrupts(USBDriver *usbp) {
  if(usbp->usb->EVENTS_USBRESET) {
    usbp->usb->EVENTS_USBRESET = 0u;
    _usb_reset(usbp);
  }

  if(usbp->usb->EVENTS_USBEVENT) {
    usbp->usb->EVENTS_USBEVENT = 0u;
    if(usbp->usb->EVENTCAUSE & USBD_EVENTCAUSE_SUSPEND_Pos) {
      usbp->usb->EVENTCAUSE &= ~USBD_EVENTCAUSE_SUSPEND_Pos;
      _usb_suspend(usbp);
    }

    if(usbp->usb->EVENTCAUSE & USBD_EVENTCAUSE_RESUME_Pos) {
      usbp->usb->EVENTCAUSE &= ~USBD_EVENTCAUSE_RESUME_Pos;
      _usb_wakeup(usbp);
    }
  }

  if(usbp->usb->EVENTS_SOF) {
    usbp->usb->EVENTS_SOF = 0u;
    _usb_isr_invoke_sof_cb(usbp);
  }

  if(usbp->usb->EVENTS_EP0SETUP) {
    usbp->usb->EVENTS_EP0SETUP = 0u;
    _usb_isr_invoke_setup_cb(usbp, 0u);
  }

  /* EP0 OUT transfer finished */
  if(usbp->usb->EVENTS_ENDEPOUT[0u]) {
    usbp->usb->EVENTS_ENDEPOUT[0u] = 0u;
    handle_out_packet(usbp, 0u);
  }

  /* EP0 in transfer finished */
  if(usbp->usb->EVENTS_EP0DATADONE) {
    usbp->usb->EVENTS_EP0DATADONE = 0u;
    if(usbp->ep0state == USB_EP0_IN_TX) {
      handle_in_packet(usbp, 0u);
    }
  }

}

static void enable_usb_with_errata(void)
{

  if(*(volatile uint32_t *)0x4006EC00 == 0x00000000)
  {
    *(volatile uint32_t *)0x4006EC00 = 0x00009375;
  }
  *(volatile uint32_t *)0x4006EC14 = 0x000000C0;
  *(volatile uint32_t *)0x4006EC00 = 0x00009375;

  *(volatile uint32_t *)0x4006EC00 = 0x00009375;
  *(volatile uint32_t *)0x4006ED14 = 0x00000003;
  *(volatile uint32_t *)0x4006EC00 = 0x00009375;

  /* Enable the peripheral */
  NRF_USBD->ENABLE = USBD_ENABLE_ENABLE_Enabled << USBD_ENABLE_ENABLE_Pos;

  /* Waiting for peripheral to enable, this should take a few µs */
  while (0 == (NRF_USBD->EVENTCAUSE & USBD_EVENTCAUSE_READY_Msk))
  {
    /* Empty loop */
  }
  NRF_USBD->EVENTCAUSE &= ~USBD_EVENTCAUSE_READY_Msk;

  *(volatile uint32_t *)0x4006EC00 = 0x00009375;
  *(volatile uint32_t *)0x4006ED14 = 0x00000000;
  *(volatile uint32_t *)0x4006EC00 = 0x00009375;

}

/*===========================================================================*/
/* Driver interrupt handlers and threads.                                    */
/*===========================================================================*/

#if NRF5_USB_USE_USB1 || defined(__DOXYGEN__)

OSAL_IRQ_HANDLER(VectorDC) {

  OSAL_IRQ_PROLOGUE();

#if NRF5_USB_USE_USB1 == TRUE
  handle_interrupts(&USBD1);
#endif

  OSAL_IRQ_EPILOGUE();

}

#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {

#if NRF5_USB_USE_USB1 == TRUE
  /* Driver initialization.*/
  usbObjectInit(&USBD1);
  USBD1.usb = NRF_USBD;
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
    /* Enables the peripheral.*/
#if NRF5_USB_USE_USB1 == TRUE
    if (&USBD1 == usbp) {
      enable_usb_with_errata();

      usbp->usb->INTENSET =
          (USBD_INTENSET_USBRESET_Enabled << USBD_INTENSET_USBRESET_Pos)
        | (USBD_INTENSET_USBEVENT_Enabled << USBD_INTENSET_USBEVENT_Pos)
        | (USBD_INTENSET_EP0SETUP_Enabled << USBD_INTENSET_EP0SETUP_Pos)
        | (USBD_INTENSET_EP0DATADONE_Enabled << USBD_INTENSET_EP0DATADONE_Pos)
        | (USBD_INTENSET_ENDEPOUT0_Enabled << USBD_INTENSET_ENDEPOUT0_Pos);

      if(usbp->config->sof_cb != NULL)
        usbp->usb->INTENSET |= (USBD_INTENSET_SOF_Enabled << USBD_INTENSET_SOF_Pos);

      nvicEnableVector(USBD_IRQn, NRF5_USB_IRQ_PRIORITY);
    }
#endif
  }

  usb_lld_reset(usbp);

}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {

  if (usbp->state != USB_STOP) {
    /* Disables the peripheral.*/
#if NRF5_USB_USE_USB1 == TRUE
    if (&USBD1 == usbp) {
      usb_lld_disconnect_bus(usbp);
      nvicDisableVector(USBD_IRQn);
      usbp->usb->INTEN = 0u;
      usbp->usb->ENABLE = 0u;
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
  usbp->epc[0u] = &ep0config;
  usb_lld_init_endpoint(usbp, 0u);
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {
  (void) usbp;
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

  osalDbgAssert(ep == 0u, "nrf8240 supports currently supports EP0 only");

  const USBEndpointConfig *epcp = usbp->epc[ep];
  osalDbgAssert((epcp->ep_mode & USB_EP_MODE_TYPE) == USB_EP_MODE_TYPE_CTRL, "EP0 must be configured as control");

  /* IN endpoint handling.*/
  if (epcp->in_state != NULL) {
    usbp->usb->EPIN[ep].PTR = 0u;
    usbp->usb->EPIN[ep].MAXCNT = 0u;
    usbp->usb->EPINEN |= 1u << (USBD_EPINEN_IN0_Pos + ep);
  }

  /* OUT endpoint handling.*/
  if (epcp->out_state != NULL) {
    usbp->usb->EPOUT[ep].PTR = 0u;
    usbp->usb->EPOUT[ep].MAXCNT = epcp->out_maxsize;
    usbp->usb->EPOUTEN |= 1u << (USBD_EPOUTEN_OUT0_Pos + ep);
  }

}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {

  usbp->usb->EPOUTEN = 0u;
  usbp->usb->EPINEN = 0u;

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

  if(!(usbp->usb->EPOUTEN & (USBD_EPOUTEN_OUT0_Pos + (ep & 0x07u)))) {
    return EP_STATUS_DISABLED;
  }

  return usbp->usb->HALTED.EPOUT[ep];

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

  if(!(usbp->usb->EPINEN & (USBD_EPINEN_IN0_Pos + (ep & 0x07u)))) {
    return EP_STATUS_DISABLED;
  }

  return usbp->usb->HALTED.EPOUT[ep];

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

    (void) ep;

    buf[0u] = usbp->usb->BMREQUESTTYPE;
    buf[1u] = usbp->usb->BREQUEST;
    buf[2u] = usbp->usb->WVALUEL;
    buf[3u] = usbp->usb->WVALUEH;
    buf[4u] = usbp->usb->WINDEXL;
    buf[5u] = usbp->usb->WINDEXH;
    buf[6u] = usbp->usb->WLENGTHL;
    buf[7u] = usbp->usb->WLENGTHH;

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

  const USBEndpointConfig *epcp = usbp->epc[ep];
  USBOutEndpointState *osp = epcp->out_state;

  uint32_t addr = (uint32_t) osp->rxbuf;
  if((addr >= NRF_MEMORY_RAM_BASE) && (addr < NRF_MEMORY_RAM_BASE + NRF_MEMORY_RAM_SIZE))
    usbp->usb->EPOUT[ep].PTR = addr;
  else
    usbp->usb->EPOUT[ep].PTR = (uint32_t) usbp->dma_buffer;

  if(ep == 0u) {
    usbp->usb->SHORTS |= USBD_SHORTS_EP0DATADONE_STARTEPOUT0_Msk;
    usbp->usb->TASKS_EP0RCVOUT = 1u;
  }
  else
    usbp->usb->SIZE.EPOUT[ep] = 0u;

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

  const USBEndpointConfig *epcp = usbp->epc[ep];
  USBInEndpointState *isp = epcp->in_state;

  size_t transfer_size = (isp->txsize < epcp->in_maxsize) ? isp->txsize : epcp->in_maxsize;
  osalDbgCheck(transfer_size <= 64u);

  uint32_t addr = (uint32_t) isp->txbuf;

  if((addr >= NRF_MEMORY_RAM_BASE) && (addr < NRF_MEMORY_RAM_BASE + NRF_MEMORY_RAM_SIZE)) {
    usbp->usb->EPIN[ep].PTR = addr;
  }
  else {
    memcpy(usbp->dma_buffer, isp->txbuf, transfer_size);
    usbp->usb->EPIN[ep].PTR = (uint32_t) usbp->dma_buffer;
  }

  usbp->usb->EPIN[ep].MAXCNT = transfer_size;
  usbp->usb->TASKS_STARTEPIN[ep] = 1u;

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
  usbp->usb->EPSTALL = (ep & 0x07u) << USBD_EPSTALL_EP_Pos
                       | USBD_EPSTALL_IO_Out << USBD_EPSTALL_IO_Pos
                       | USBD_EPSTALL_STALL_Stall << USBD_EPSTALL_STALL_Pos;
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
  usbp->usb->EPSTALL = (ep & 0x07u) << USBD_EPSTALL_EP_Pos
                       | USBD_EPSTALL_IO_Out << USBD_EPSTALL_IO_Pos
                       | USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos;
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
  usbp->usb->EPSTALL = (ep & 0x07u) << USBD_EPSTALL_EP_Pos
                       | USBD_EPSTALL_IO_In << USBD_EPSTALL_IO_Pos
                       | USBD_EPSTALL_STALL_Stall << USBD_EPSTALL_STALL_Pos;
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
  usbp->usb->EPSTALL = (ep & 0x07u) << USBD_EPSTALL_EP_Pos
                       | USBD_EPSTALL_IO_In << USBD_EPSTALL_IO_Pos
                       | USBD_EPSTALL_STALL_UnStall << USBD_EPSTALL_STALL_Pos;
}

void usb_lld_end_setup(USBDriver *usbp, usbep_t ep) {
  osalDbgCheck(ep == 0u);

  usbp->usb->TASKS_EP0STATUS = 1u;
}

#endif /* HAL_USE_USB == TRUE */

/** @} */
