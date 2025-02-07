#include <msp430.h>                      // Generic MSP430 Device Include
#include "driverlib.h"                   // MSPWare Driver Library
#include "captivate.h"                   // CapTIvate Touch Software Library
#include "CAPT_App.h"                    // CapTIvate Application Code
#include "CAPT_BSP.h"                    // CapTIvate EVM Board Support Package
#include <stdint.h>

const unsigned int TIMEOUT_ATTENDDED = 2000;

// Definición de estados
typedef enum {NORMAL, ALARM, ALARM_ATTENDED} State;

// Variables para almacenar valores ADC
volatile unsigned int adcValue1 = 0;
volatile unsigned int adcValue2 = 0;
volatile unsigned int adcValue3 = 0;
volatile unsigned int adcValue4 = 0;



// Variables de estado
volatile State state1 = NORMAL;
volatile State state2 = NORMAL;
volatile State state3 = NORMAL;
volatile State state4 = NORMAL;

//volatile bool attended = 0;

uint8_t ui8LFOsc = CS_XT1CLK_SELECT;

// Constantes de referencia de voltaje (ajustadas para un ADC de 10 bits y Vref=3.3V)
const unsigned int ALARM_THRESHOLD = (unsigned int)((2.23 / 2.97) * 1024);
const unsigned int EXIT_THRESHOLD = (unsigned int)((1.93 / 2.97) * 1024);

// Variables para el parpadeo de los LEDs
volatile unsigned int timerCounter = 0;
volatile unsigned int timerCounterAttendded1 = 0;
volatile unsigned int timerCounterAttendded2 = 0;
volatile unsigned int timerCounterAttendded3 = 0;
volatile unsigned int timerCounterAttendded4 = 0;
volatile unsigned int blinkState = 0;
volatile unsigned int timerBuzzer = 0;


void setupClock();
void setupADC();
void setupTimer();
void setupGPIO();
void checkADCValues();
void leer_ADCs(void);
unsigned int readADC(unsigned int channel);
void checkLEDs(State state, unsigned int threshold, unsigned int led_verde, unsigned int led_rojo, volatile unsigned char *port_out);

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog timer
    PM5CTL0 &= ~LOCKLPM5; // Desbloquea los puertos
    setupClock();
    setupGPIO();
    setupADC();
    setupTimer();

    extern tElement keypad_E00;
    extern tElement keypad_E01;
    extern tButtonSensorParams keypadSensor_Params;

    BUZZER_POS_ON; // Ensure the buzzer is off at begining
    BUZZER_NEG_OFF;

    volatile unsigned int tone1_period;
    volatile unsigned int tone2_period;
    volatile unsigned int tone_toggle;
    volatile unsigned int tone_counter;

    //
    // Start the CapTIvate application
    //
    CAPT_appStart();
    //MAP_CAPT_registerCallback(&keypad, &my_button_callback);
    //
    // Background Loop

    __enable_interrupt();     // Enable global interrupts

    while (1) {

        if(state1 == ALARM || state2 == ALARM || state3 == ALARM || state4 == ALARM)  {

            buzz_alternate_tones(200, 500, 800); // Buzz alternating tones for 1 second
        }


        // Leer secuencialmente los valores de las entradas analógicas
             adcValue1 = readADC(ADCINCH_0);  // Leer P1.0 (ADC0)
             adcValue2 = readADC(ADCINCH_1);  // Leer P1.1 (ADC1)
             adcValue3 = readADC(ADCINCH_2);  // Leer P1.2 (ADC2)
             adcValue4 = readADC(ADCINCH_3);  // Leer P1.3 (ADC3)
       // updateLEDs();
             checkADCValues();

             // Comprobar los valores y encender los LEDs correspondientes
             checkLEDs(state1, ALARM_THRESHOLD, BIT6, BIT5, &P2OUT);  // LED verde y rojo 1 en P2.6 y P2.5
             checkLEDs(state2, ALARM_THRESHOLD, BIT0, BIT1, &P2OUT);  // LED verde y rojo 2 en P2.0 y P2.1
             checkLEDs(state3, ALARM_THRESHOLD, BIT3, BIT2, &P2OUT);  // LED verde y rojo 3 en P2.3 y P2.2
             checkLEDs(state4, ALARM_THRESHOLD, BIT7, BIT6, &P1OUT);  // LED verde y rojo 4 en P1.7 y P1.6
           if (g_bConvTimerFlag == true) //Comprobar si ha habido un touch en CANCEL
                   {
                    g_bConvTimerFlag = false;
                    CAPT_updateUI(&g_uiApp);

                        if(keypad_E00.bTouch == true)
                         {
                            if (state1 == ALARM) {
                                state1 = ALARM_ATTENDED;
                                timerCounterAttendded1 = 0;
                            }
                            if (state2 == ALARM) {
                                state2 = ALARM_ATTENDED;
                                timerCounterAttendded2 = 0;
                            }
                            if (state3 == ALARM) {
                                state3 = ALARM_ATTENDED;
                                timerCounterAttendded3 = 0;
                            }
                            if (state4 == ALARM) {
                                state4 = ALARM_ATTENDED;
                                timerCounterAttendded4 = 0;
                             }
                      //   CANCEL_ON;
                         __delay_cycles(50000);
                         CANCEL_OFF;

                         }

                 }

                CAPT_appSleep();
    }

}

void setupADC() {
    // Configuración del ADC
        ADCCTL0 = ADCSHT_2 | ADCON;        // Ciclos de retención de 16 ADC clock, encender ADC
        ADCCTL1 = ADCSHP;                  // ADC sample-and-hold control, usar temporizador
        ADCCTL2 = ADCRES_1;                // Resolución de 10 bits (1024 niveles)
}

unsigned int readADC(unsigned int channel)
{
    // Deshabilitar el ADC para cambiar el canal
    ADCCTL0 &= ~ADCENC;

    // Configurar el canal de entrada correspondiente
    ADCMCTL0 &= ~0x000F;               // Limpiar el canal de entrada
    ADCMCTL0 |= channel;               // Seleccionar el canal

    // Volver a habilitar el ADC
    ADCCTL0 |= ADCENC;

    // Iniciar conversión
    ADCCTL0 |= ADCSC;

    // Esperar a que la conversión termine
    while (ADCCTL1 & ADCBUSY);

    // Devolver el valor convertido
    return ADCMEM0;
}




void setupTimer() {
    // Configuración del Timer 1
    TA1CCTL0 = CCIE;            // Enable Timer A0 interrupt
    TA1CCR0 = 500 - 1;          // Set Timer A0 period to 1 second (32768 ticks)
    TA1CTL = TASSEL_1 + MC_1;   // ACLK, up mode, clear TAR
}

void setupGPIO() {
    // Configuración de los pines como salidas para los LEDs
       P2DIR |= BIT6 | BIT5 | BIT0 | BIT1 | BIT3 | BIT2;  // P2.6, P2.5, P2.0, P2.1, P2.3, P2.2 como salidas
       P1DIR |= BIT7 | BIT6;                              // P1.7, P1.6 como salidas

       // Limpiar la configuración de selección de funciones para evitar conflictos en P2
       P2SEL0 &= ~(BIT6 | BIT5 | BIT0 | BIT1 | BIT3 | BIT2);  // Selección de función estándar GPIO
       P2SEL1 &= ~(BIT6 | BIT5 | BIT0 | BIT1 | BIT3 | BIT2);

       // Apagar todos los LEDs al inicio
       P2OUT &= ~(BIT6 | BIT5 | BIT0 | BIT1 | BIT3 | BIT2);
       P1OUT &= ~(BIT7 | BIT6);

    // Configuración de BUZZER
    BUZZER_POS_PDIR |= BUZZER_POS_PIN;
    BUZZER_NEG_PDIR |= BUZZER_NEG_PIN;
    BUZZER_POS_ON;
    BUZZER_NEG_OFF;

    // Configuración de la señal CANCEL
    CANCEL_PDIR |= CANCEL_PIN;
    CANCEL_OFF;  // Deshabilitada al inicio

}

void checkADCValues() {
    // Check adcValue1
    if ((adcValue1 > ALARM_THRESHOLD) && (state1 == ALARM_ATTENDED)) {
        state1 = ALARM_ATTENDED;

        }
    else if ((adcValue1 > ALARM_THRESHOLD) && (state1 != ALARM_ATTENDED)){
        state1 = ALARM;
    }
        else if (adcValue1 < EXIT_THRESHOLD) {
        state1 = NORMAL;
        CANCEL_OFF;
    }

    // Check adcValue2
    if ((adcValue2 > ALARM_THRESHOLD) && (state2 == ALARM_ATTENDED)) {
           state2 = ALARM_ATTENDED;
           }
       else if ((adcValue2 > ALARM_THRESHOLD) && (state2 != ALARM_ATTENDED)){
           state2 = ALARM;
       }
           else if (adcValue2 < EXIT_THRESHOLD) {
           state2 = NORMAL;
           CANCEL_OFF;
       }

    // Check adcValue3
        if ((adcValue3 > ALARM_THRESHOLD) && (state3 == ALARM_ATTENDED)) {
               state3 = ALARM_ATTENDED;
               }
           else if ((adcValue3 > ALARM_THRESHOLD) && (state3 != ALARM_ATTENDED)){
               state3 = ALARM;
           }
               else if (adcValue3 < EXIT_THRESHOLD) {
               state3 = NORMAL;
               CANCEL_OFF;
           }

     // Check adcValue2
        if ((adcValue4 > ALARM_THRESHOLD) && (state4 == ALARM_ATTENDED)) {
               state4 = ALARM_ATTENDED;
               }
           else if ((adcValue4 > ALARM_THRESHOLD) && (state4 != ALARM_ATTENDED)){
               state4 = ALARM;
           }
               else if (adcValue4 < EXIT_THRESHOLD) {
               state4 = NORMAL;
               CANCEL_OFF;
           }
}


void checkLEDs(State state, unsigned int threshold, unsigned int led_verde, unsigned int led_rojo, volatile unsigned char *port_out)
{
    // Comparar el valor ADC con el umbral directamente (2.23V corresponde aproximadamente a un valor ADC de 691)
    if (state == NORMAL)
    {

        *port_out |= led_verde;    // Encender LED verde
        *port_out &= ~led_rojo;    // Apagar LED rojo
    }
    else if (state == ALARM)
    {
        if (blinkState) {
            *port_out |= led_rojo;     // Encender LED rojo
                } else {
                    *port_out &= ~led_rojo;   // Apagar LED verde
                }


        *port_out &= ~led_verde;   // Apagar LED verde
    }
    else if (state == ALARM_ATTENDED)
       {
               *port_out |= led_rojo;     // Encender LED rojo
               *port_out |= led_verde;    // Encender tambien el verde para que parpadee en amarillo


       }
}


void setupClock() {
    // Clear port lock
    PM5CTL0 &= ~LOCKLPM5;

    // Configure FRAM wait state (set to 1 to support 16MHz MCLK)
    MAP_FRAMCtl_configureWaitStateControl(FRAMCTL_ACCESS_TIME_CYCLES_1);

    // Attempt to start the low frequency crystal oscillator
    MAP_CS_setExternalClockSource(XT1_OSC_FREQ);
    if (CS_turnOnXT1LFWithTimeout(CS_XT1_DRIVE_0, XT1_OSC_TIMEOUT) == STATUS_FAIL) {
        // If a crystal is not present or is failing, switch the LF
        // clock definition to the internal 32kHz reference oscillator.
        ui8LFOsc = CS_REFOCLK_SELECT;
    }
    // Initialize Clock Signals
    MAP_CS_initClockSignal(CS_FLLREF, ui8LFOsc, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_ACLK, ui8LFOsc, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_8);

    // Tune the DCO parameters
    MAP_CS_initFLL((MCLK_FREQ/1000), FLL_RATIO);
    MAP_CS_clearAllOscFlagsWithTimeout(1000);
}

// Interrupción del Timer 1
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer_A(void) {
    if ((timerCounterAttendded1 >= TIMEOUT_ATTENDDED) && (state1 == ALARM_ATTENDED)) {
        state1 = ALARM;
    }
    if ((timerCounterAttendded2 >= TIMEOUT_ATTENDDED) && (state2 == ALARM_ATTENDED)) {
            state2 = ALARM;
        }
    if ((timerCounterAttendded3 >= TIMEOUT_ATTENDDED) && (state3 == ALARM_ATTENDED)) {
            state3 = ALARM;
        }
    if ((timerCounterAttendded4 >= TIMEOUT_ATTENDDED) && (state4 == ALARM_ATTENDED)) {
            state4 = ALARM;
        }
    if (timerCounter >= 5) {
        blinkState = !blinkState;
        timerCounter = 0;
    }
    timerCounter++;
    timerCounterAttendded1++;
    timerCounterAttendded2++;
    timerCounterAttendded3++;
    timerCounterAttendded4++;
}

void buzz_alternate_tones(unsigned int tone1, unsigned int tone2, unsigned int duration) {
    unsigned int period1 = 1000000 / tone1; // Calculate period for tone1 in microseconds
    unsigned int period2 = 1000000 / tone2; // Calculate period for tone2 in microseconds
    unsigned int half_period1 = period1 / 2; // Calculate half period for tone1
    unsigned int half_period2 = period2 / 2; // Calculate half period for tone2

    unsigned int time_elapsed = 0; // Track the elapsed time

    while (time_elapsed < duration) {
        // Generate tone1
        unsigned int i;
        for (i = 0; i < (duration / 2); i++) {
            BUZZER_POS_TOGGLE; // Toggle the buzzer pin
            BUZZER_NEG_TOGGLE;
            __delay_cycles(5200); // Delay for half the period
            time_elapsed += half_period1 / 1000; // Update elapsed time in milliseconds
        }

        // Generate tone2
        for (i = 0; i < (duration / 2); i++) {
            BUZZER_POS_TOGGLE; // Toggle the buzzer pin
            BUZZER_NEG_TOGGLE;
            __delay_cycles(4300); // Delay for half the period
            time_elapsed += half_period2 / 1000; // Update elapsed time in milliseconds
        }

    }

    BUZZER_POS_ON; // Ensure the buzzer is off at the end
    BUZZER_NEG_OFF;

}


//****************************************************************

