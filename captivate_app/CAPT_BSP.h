/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
// CAPT_BSP.h
//
//! CAPT_BSP is the board support package for the Capacitive Touch MCU 
//! Development Kit.  It defines the BSP_configureMCU() function, which 
//! configures the MCU clock system (CS) and port muxing (Digital IO).
//
//! \version 1.83.00.05
//! Released on May 15, 2020
//
//*****************************************************************************

#ifndef CAPT_BSP_H_
#define CAPT_BSP_H_

#include <msp430.h>
#include "driverlib.h"

//*****************************************************************************
//
//! \def XT1_OSC_FREQ defines the frequency of the crystal oscillator on
//! XIN/XOUT.
//
//*****************************************************************************
#define XT1_OSC_FREQ                                                    (32768)
#define XT1_OSC_TIMEOUT                                                 (65000)

//*****************************************************************************
//
//! \def MCLK_FREQ defines the main clock frequency in Hz.
//! \def SMCLK_FREQ defines the sub-main clock frequency in Hz.
//! \def ACLK_FREQ defines the auxiliary clock frequency in Hz.
//! \def FLLREF_FREQ defines the FLL reference clock frequency in Hz.
//
//*****************************************************************************
#define MCLK_FREQ                                                    (16000000)
#define SMCLK_FREQ                                                    (2000000)
#define ACLK_FREQ                                                       (32768)
#define FLLREF_FREQ                                                     (32768)

//*****************************************************************************
//
//! \def FLL_RATIO defines ratio of MCLK to the FLL reference clock.
//
//*****************************************************************************
#define FLL_RATIO                                     (MCLK_FREQ / FLLREF_FREQ)

//*****************************************************************************
//
//! \def OSC_TIMEOUT defines the failure timeout for all oscillators.
//
//*****************************************************************************
#define OSC_TIMEOUT                                                      (1000)

//*****************************************************************************
//
//! \def LED1_POUT defines the port-out register LED1 is attached to.
//! \def LED1_PDIR defines the port-direction register LED1 is attached to.
//! \def LED1_PIN defines the port pin that LED1 is attached to.
//! \def LED1_ON defines macro to turn on LED1.
//! \def LED1_OFF defines a macro to turn off LED1.
//! \def LED1_TOGGLE defines a macro to toggle LED1.
//
// Mis macros para control de LEDs y salidas del puerto 2:
#define CANCEL_POUT                                                      (P2OUT)
#define CANCEL_PDIR                                                      (P2DIR)
#define CANCEL_PIN                                                        (BIT7)
#define CANCEL_ON                                    (CANCEL_POUT |= CANCEL_PIN)
#define CANCEL_OFF                                 (CANCEL_POUT &= ~ CANCEL_PIN)
#define CANCEL_TOGGLE                                (CANCEL_POUT ^= CANCEL_PIN)

#define LED_VERDE_ZONA_1_POUT                                            (P2OUT)
#define LED_VERDE_ZONA_1_PDIR                                            (P2DIR)
#define LED_VERDE_ZONA_1_PIN                                              (BIT6)
#define LED_VERDE_ZONA_1_ON      (LED_VERDE_ZONA_1_POUT |= LED_VERDE_ZONA_1_PIN)
#define LED_VERDE_ZONA_1_OFF   (LED_VERDE_ZONA_1_POUT &= ~ LED_VERDE_ZONA_1_PIN)
#define LED_VERDE_ZONA_1_TOGGLE  (LED_VERDE_ZONA_1_POUT ^= LED_VERDE_ZONA_1_PIN)

#define LED_ROJO_ZONA_1_POUT                                             (P2OUT)
#define LED_ROJO_ZONA_1_PDIR                                             (P2DIR)
#define LED_ROJO_ZONA_1_PIN                                               (BIT5)
#define LED_ROJO_ZONA_1_ON         (LED_ROJO_ZONA_1_POUT |= LED_ROJO_ZONA_1_PIN)
#define LED_ROJO_ZONA_1_OFF      (LED_ROJO_ZONA_1_POUT &= ~ LED_ROJO_ZONA_1_PIN)
#define LED_ROJO_ZONA_1_TOGGLE     (LED_ROJO_ZONA_1_POUT ^= LED_ROJO_ZONA_1_PIN)

#define LED_VERDE_ZONA_2_POUT                                            (P2OUT)
#define LED_VERDE_ZONA_2_PDIR                                            (P2DIR)
#define LED_VERDE_ZONA_2_PIN                                              (BIT0)
#define LED_VERDE_ZONA_2_ON      (LED_VERDE_ZONA_2_POUT |= LED_VERDE_ZONA_2_PIN)
#define LED_VERDE_ZONA_2_OFF   (LED_VERDE_ZONA_2_POUT &= ~ LED_VERDE_ZONA_2_PIN)
#define LED_VERDE_ZONA_2_TOGGLE  (LED_VERDE_ZONA_2_POUT ^= LED_VERDE_ZONA_2_PIN)

#define LED_ROJO_ZONA_2_POUT                                             (P2OUT)
#define LED_ROJO_ZONA_2_PDIR                                             (P2DIR)
#define LED_ROJO_ZONA_2_PIN                                               (BIT1)
#define LED_ROJO_ZONA_2_ON         (LED_ROJO_ZONA_2_POUT |= LED_ROJO_ZONA_2_PIN)
#define LED_ROJO_ZONA_2_OFF      (LED_ROJO_ZONA_2_POUT &= ~ LED_ROJO_ZONA_2_PIN)
#define LED_ROJO_ZONA_2_TOGGLE     (LED_ROJO_ZONA_2_POUT ^= LED_ROJO_ZONA_2_PIN)

#define LED_VERDE_ZONA_3_POUT                                            (P2OUT)
#define LED_VERDE_ZONA_3_PDIR                                            (P2DIR)
#define LED_VERDE_ZONA_3_PIN                                              (BIT3)
#define LED_VERDE_ZONA_3_ON      (LED_VERDE_ZONA_3_POUT |= LED_VERDE_ZONA_3_PIN)
#define LED_VERDE_ZONA_3_OFF   (LED_VERDE_ZONA_3_POUT &= ~ LED_VERDE_ZONA_3_PIN)
#define LED_VERDE_ZONA_3_TOGGLE  (LED_VERDE_ZONA_3_POUT ^= LED_VERDE_ZONA_3_PIN)

#define LED_ROJO_ZONA_3_POUT                                             (P2OUT)
#define LED_ROJO_ZONA_3_PDIR                                             (P2DIR)
#define LED_ROJO_ZONA_3_PIN                                               (BIT2)
#define LED_ROJO_ZONA_3_ON         (LED_ROJO_ZONA_3_POUT |= LED_ROJO_ZONA_3_PIN)
#define LED_ROJO_ZONA_3_OFF      (LED_ROJO_ZONA_3_POUT &= ~ LED_ROJO_ZONA_3_PIN)
#define LED_ROJO_ZONA_3_TOGGLE     (LED_ROJO_ZONA_3_POUT ^= LED_ROJO_ZONA_3_PIN)


// Mis macros para control de LEDs y salidas del puerto 1:
#define LED_VERDE_ZONA_4_POUT                                            (P1OUT)
#define LED_VERDE_ZONA_4_PDIR                                            (P1DIR)
#define LED_VERDE_ZONA_4_PIN                                              (BIT7)
#define LED_VERDE_ZONA_4_ON      (LED_VERDE_ZONA_4_POUT |= LED_VERDE_ZONA_4_PIN)
#define LED_VERDE_ZONA_4_OFF   (LED_VERDE_ZONA_4_POUT &= ~ LED_VERDE_ZONA_4_PIN)
#define LED_VERDE_ZONA_4_TOGGLE  (LED_VERDE_ZONA_4_POUT ^= LED_VERDE_ZONA_4_PIN)

#define LED_ROJO_ZONA_4_POUT                                             (P1OUT)
#define LED_ROJO_ZONA_4_PDIR                                             (P1DIR)
#define LED_ROJO_ZONA_4_PIN                                               (BIT6)
#define LED_ROJO_ZONA_4_ON         (LED_ROJO_ZONA_4_POUT |= LED_ROJO_ZONA_4_PIN)
#define LED_ROJO_ZONA_4_OFF      (LED_ROJO_ZONA_4_POUT &= ~ LED_ROJO_ZONA_4_PIN)
#define LED_ROJO_ZONA_4_TOGGLE     (LED_ROJO_ZONA_4_POUT ^= LED_ROJO_ZONA_4_PIN)

// Otras Constantes Usadas

#define LED_VERDE_ZONA_1 BIT6
#define LED_ROJO_ZONA_1 BIT5
#define LED_VERDE_ZONA_2 BIT0
#define LED_ROJO_ZONA_2 BIT1
#define LED_VERDE_ZONA_3 BIT3
#define LED_ROJO_ZONA_3 BIT2
#define LED_VERDE_ZONA_4 BIT7
#define LED_ROJO_ZONA_4 BIT6

#define BUZZER_POS_POUT        (P1OUT)
#define BUZZER_POS_PDIR        (P1DIR)
#define BUZZER_POS_PIN         (BIT5)
#define BUZZER_POS_SEL0        (P1SEL0)
#define BUZZER_POS_SEL1        (P1SEL1)
#define BUZZER_POS_ON          (BUZZER_POS_POUT |= BUZZER_POS_PIN)
#define BUZZER_POS_OFF         (BUZZER_POS_POUT &= ~BUZZER_POS_PIN)
#define BUZZER_POS_TOGGLE      (BUZZER_POS_POUT ^= BUZZER_POS_PIN)

#define BUZZER_NEG_POUT        (P1OUT)
#define BUZZER_NEG_PDIR        (P1DIR)
#define BUZZER_NEG_PIN         (BIT4)
#define BUZZER_NEG_SEL0        (P1SEL0)
#define BUZZER_NEG_SEL1        (P1SEL1)
#define BUZZER_NEG_ON          (BUZZER_NEG_POUT |= BUZZER_NEG_PIN)
#define BUZZER_NEG_OFF         (BUZZER_NEG_POUT &= ~BUZZER_NEG_PIN)
#define BUZZER_NEG_TOGGLE      (BUZZER_NEG_POUT ^= BUZZER_NEG_PIN)


// Frecuencias configurables para el buzzer
#define BUZZER_FREQ_1 1000    // 1000Hz
#define BUZZER_FREQ_2 1500  // 1500Hz
#define BUZZER_FREQ_PROTECTION 500 // 500Hz



#define LED_VERDE_ON(zona) LED_VERDE_ZONA_##zona##_ON
#define LED_VERDE_OFF(zona) LED_VERDE_ZONA_##zona##_OFF
#define LED_ROJO_ON(zona) LED_ROJO_ZONA_##zona##_ON
#define LED_ROJO_OFF(zona) LED_ROJO_ZONA_##zona##_OFF

#define NUM_LEDS 4
#define TIEMPO_VERDE 1000 // Ejemplo de tiempo en milisegundos
#define TIEMPO_ROJO 1000   // Ejemplo de tiempo en milisegundos
#define TIEMPO_OFF 250    // Ejemplo de tiempo en milisegundos
#define TIEMPO_TRANSICION 250  // Ejemplo de tiempo en milisegundos
#define VECES_FORWARD 1
#define VECES_BACKWARD 1

#define BUZZER_NEG BIT4
#define BUZZER_POS BIT5

void delay_ms(unsigned int ms);
void setup();
void blink_all_leds(unsigned char leds, unsigned int times);
void blink_leds_sequence(unsigned int num_leds, unsigned int tiempo_verde, unsigned int tiempo_rojo, unsigned int tiempo_off, unsigned int tiempo_transicion, unsigned int veces_forward, unsigned int veces_backward);
void all_leds_verdes_on();
void all_leds_verdes_off();
void all_leds_rojos_on();
void all_leds_rojos_off();
void buzz_alternate_tones(unsigned int tone1, unsigned int tone2, unsigned int duration);



//*****************************************************************************
//
//! This function is configures the MCU for operation.
//! This involves setting up the digital IO muxes and configuring the clock
//! system (CS).
//
//! \param none
//! \return none
//
//*****************************************************************************
extern void BSP_configureMCU(void);

#endif /* CAPT_BSP_H_ */
