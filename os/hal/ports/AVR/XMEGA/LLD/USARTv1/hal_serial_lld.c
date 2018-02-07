/*
    ChibiOS - Copyright (C) 2016..2018 Giovanni Theodore Ateba

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
 * @file    hal_serial_lld.c
 * @brief   AVR serial subsystem low level driver source.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if (HAL_USE_SERIAL == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USART1 serial driver identifier.*/
#if (AVR_SERIAL_USE_USART1 == TRUE) || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {
  38400
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void usart_init(SerialDriver *sdp, const SerialConfig *config) {

  USART_t *u = sdp->usart;

  /* Disable the USART receiver.    */
  /* Disable the USART transmitter. */
  u->CTRLB &= ~(USART_RXEN_bm);
  u->CTRLB &= ~(USART_TXEN_bm);

  /* Disable the Multi Processor Communixation Mode.  */
  /* Use the normal speed.                            */
  /* Use 8 bits transmission mode.                    */
  /* Configure one stop bit.                          */
  u->CTRLB &= ~(USART_MPCM_bm);
  u->CTRLB &= ~(USART_CLK2X_bm);
  u->CTRLB &= ~(USART_TXB8_bm);
  u->CTRLC &= ~USART_SBMODE_bm;

  /* Set asynchronous mode.         */
  /* Set the parity bit.            */
  /* Set 8 bits communication size. */
  u->CTRLC = (u->CTRLC & ~USART_CMODE_gm)   | (USART_CMODE_ASYNCHRONOUS_gc);
  u->CTRLC = (u->CTRLC & ~USART_PMODE_gm)   | (USART_PMODE_DISABLED_gc);
  u->CTRLC = (u->CTRLC & ~USART_CHSIZE_gm)  | (USART_CHSIZE_8BIT_gc);

  /* Set the baud rate for BSCALE = 0.  */
	#define BSCALE 0
	uint16_t br = get_bsel(config->speed);
	u->BAUDCTRLA =(uint8_t)br;
	u->BAUDCTRLB =(BSCALE << USART_BSCALE0_bp) | (br >> 8);

  /* Set the USART RX interruption level.             */
  /* Set the USART TX interruption level.             */
  /* Set the USART register empty interruption level. */
  u->CTRLA = (u->CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_LO_gc;
  u->CTRLA = (u->CTRLA & ~USART_TXCINTLVL_gm) | USART_TXCINTLVL_LO_gc;
  u->CTRLA = (u->CTRLA & ~USART_DREINTLVL_gm) | USART_DREINTLVL_LO_gc;

  /* Enable PMIC interrupt level low. */
  /* Enable global interrupts.        */
  PMIC.CTRL |= PMIC_LOLVLEX_bm;
  sei();

  /* Enable the USART receiver.     */
  /* Enable the USART transmitter.  */
  u->CTRLB |= (USART_RXEN_bm);
  u->CTRLB |= (USART_TXEN_bm);
}

/**
 * @brief   USART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] u         pointer to an USART I/O block
 */
static void usart_deinit(USART_t *u) {

  u->CTRLB &= ~(USART_RXEN_bm); /* Disable the USART receiver.              */
  u->CTRLB &= ~(USART_TXEN_bm); /* Disable the USART transmitter.           */
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] sr        USART SR register value
 */
static void set_error(SerialDriver *sdp, uint8_t sr) {

  eventflags_t sts = 0;

  if (sr & USART_BUFOVF_bm)
    sts |= SD_OVERRUN_ERROR;
  if (sr & USART_PERR_bm)
    sts |= SD_PARITY_ERROR;
  if (sr & USART_FERR_bm)
    sts |= SD_FRAMING_ERROR;

  chnAddFlagsI(sdp, sts);
}

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the USART
 */
static void serve_interrupt(SerialDriver *sdp) {

  (void)sdp;
  /* TODO: To be implemented. */
}

#if AVR_SERIAL_USE_USART1 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  USARTC0.CTRLA &= ~USART_DREINTLVL_gm;
  USARTC0.CTRLA |= USART_DREINTLVL_MED_gc;
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if AVR_SERIAL_USE_USART1 || defined(__DOXYGEN__)
/**
 * @brief   USART1 TX IRQ handler, transmission complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTC0_DRE_vect) {

  msg_t msg;
 
  OSAL_IRQ_PROLOGUE();
  
  serve_interrupt(&SD1);
  osalSysLockFromISR();
  msg = oqGetI(&SD1.oqueue);
  osalSysUnlockFromISR();

  if (msg < MSG_OK) {
    //USARTC0.CTRLB &= ~USART_DREINTLVL_gm;
    //USARTC0.CTRLB &= ~USART_DREINTLVL_gm; /* TODO: Implement this block.  */
  }
  else {
    USARTC0.DATA = msg;
  }

  OSAL_IRQ_EPILOGUE();
}

/**
 * @brief   USART1 RX IRQ handler, reception complete interruption.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(USARTC0_RXC_vect) {

  uint8_t status;

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD1);
  status = USARTC0.STATUS;
  if (status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm));
    set_error(&SD1, status);
  osalSysLockFromISR();
  sdIncomingDataI(&SD1, USARTC0.DATA);
  osalSysUnlockFromISR();
  OSAL_IRQ_EPILOGUE();
}

#endif /* AVR_UART_USE_USART1 */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {

#if AVR_SERIAL_USE_USART1 == TRUE
  sdObjectInit(&SD1, NULL, notify1);
  SD1.usart = &USARTC0;
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL) {
    config = &default_config;
  }

  if (sdp->state == SD_STOP) {
#if AVR_SERIAL_USE_USART1 == TRUE
    if (&SD1 == sdp) {
    }
#endif
  }
  /* Configures the peripheral.*/
  usart_init(sdp, config);
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the USART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  usart_deinit(sdp->usart);
  if (sdp->state == SD_READY) {
#if AVR_SERIAL_USE_USART1 == TRUE
    if (&SD1 == sdp) {

    }
#endif
  }
}

#endif /* HAL_USE_SERIAL == TRUE */

/** @} */
