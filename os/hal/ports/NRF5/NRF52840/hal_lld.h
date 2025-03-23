/*
    Copyright (C) 2016 Stephane D'Alu

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
 * @file    NRF5/NRF52840/hal_lld.h
 * @brief   NRF52840 HAL subsystem low level driver header.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification
 * @{
 */
#define PLATFORM_NAME           "Nordic Semiconductor nRF52840"

/**
 * @name    Chip series
 */
#define NRF_SERIES              52

/**
 * @}
 */


/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @brief   Select source of High Frequency Clock (HFCLK)
 * @details Possible values for source are:
 *            0 : 64 MHz internal oscillator (HFINT)
 *            1 : 32 MHz external crystal oscillator (HFXO)
 */
#if !defined(NRF5_HFCLK_SOURCE) || defined(__DOXYGEN__)
#define NRF5_HFCLK_SOURCE             CLOCK_HFCLKSTAT_SRC_RC
#endif

/**
 * @brief   Select source of Low Frequency Clock (LFCLK)
 * @details Possible values for source are:
 *            0 : RC oscillator
 *            1 : External crystal
 *            2 : Synthesized clock from High Frequency Clock (HFCLK)
 *          When crystal is not available it's preferable to use the
 *          internal RC oscillator that synthesizing the clock.
 */
#if !defined(NRF5_LFCLK_SOURCE) || defined(__DOXYGEN__)
#define NRF5_LFCLK_SOURCE             CLOCK_LFCLKSTAT_SRC_RC
#endif

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if (NRF5_HFCLK_SOURCE != CLOCK_HFCLKSTAT_SRC_RC) && (NRF5_HFCLK_SOURCE != CLOCK_HFCLKSTAT_SRC_Xtal)
#error "Possible value for NRF5_HFCLK_SOURCE are CLOCK_HFCLKSTAT_SRC_RC or CLOCK_HFCLKSTAT_SRC_Xtal"
#endif

#if (NRF5_LFCLK_SOURCE != CLOCK_LFCLKSTAT_SRC_RC) && (NRF5_LFCLK_SOURCE != CLOCK_LFCLKSTAT_SRC_Xtal) && (NRF5_LFCLK_SOURCE != CLOCK_LFCLKSTAT_SRC_Synth)
#error "Possible value for NRF5_LFCLK_SOURCE are CLOCK_LFCLKSTAT_SRC_RC, CLOCK_LFCLKSTAT_SRC_Xtal, or CLOCK_LFCLKSTAT_SRC_Synth"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#include "nvic.h"
#include "nrf52_isr.h"


#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H */

/**
 * @}
 */
