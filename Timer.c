/**
 * @file Timer.c
 * @author Nathaniel Suddarth
 * @brief Provides timing for mandatory wait periods and timed interrupts.
 * @version 0.1
 * @date 2025-06-12
 *
 *
 */
#include "msp430.h"
#include <stdint.h>
#include <stdio.h>
#include "Timer.h"
#include "init_var.h"
//#include "driverlib.h"
#include "I2C.h"
#include "uart.h"
//#include <stdint.h>
#include "main.h"



// Holds Flag Values for half_mil:
volatile uint8_t TIMING_FLAG_BYTE = 0;
// Half_mil Counters Used for Timing:
volatile uint8_t HALF_MIL_COUNTER = 0;
// Iterator for number of milliseconds left during a wait.
volatile uint16_t MS_COUNTER = 0;
volatile uint16_t FIFTY_MS_COUNTER = 0;
volatile uint16_t SECOND_COUNTER = 0;
volatile uint16_t TEN_SECOND_COUNTER = 0;
volatile uint16_t MINUTE_COUNTER = 0;
volatile uint16_t HOUR_COUNTER = 0;
uint16_t DPB_SECOND_COUNTER = 0;

/********************************************************************************************
 *                          SENSIRION SCD30 TIMER FUNCTIONS: *
 ********************************************************************************************/

void timer_A_init()
{
    /*
     * 1 MHz SMCLK clock frequency on the MS430FR2476 development kit
     * Want to count to 500 microseconds = 500 clock ticks.
     * Each clock tick is 1 microsecond.
     */

    // Clear timer
    TA0CTL |= TACLR;
    // Enable interrupt, output mode 7, set to compare mode
    TA0CCTL0 = (TA0CCTL0 | CCIE | OUTMOD_7) & (~CAP);
    // Output mode 7, set to compare mode
    TA0CCTL1 = (TA0CCTL1 | OUTMOD_7) & (~CAP);
    // HALF_MIL_PERIOD==500, 0-499 is 500 microseconds
    TA0CCR0 = HALF_MIL_PERIOD - 1;
    // Interrupt enable, Upmode, SMCLK,
    TA0CTL = TA0CTL | TAIE | MC_1 | TASSEL_2;
    __enable_interrupt();
}


void wait_ms(uint16_t ms)
{
    // Iterating by the half-mil
    MS_COUNTER = 2 * ms;
    // Start timer.
  //  TBxCTL |= MC__UP;  didn't know timer TB0 was used for, dpb
    // General interrupt enable
 //   __enable_interrupt();
    while (MS_COUNTER)
    {
        // WAIT
    }
}

/********************************************************************************************
 *                              SENSIRION SCD30 TIMER ISR:                                  *
 ********************************************************************************************/

/**
 * @brief ISR that performs half_mil timing for main().
 *
 * @author Revised from and combined with the work of Emmanuel Akpan
 */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void half_mil(void)
{
    // Clear TA0 interrupt.
    TA0CTL &= (~TAIFG);
    // Increment half-mil count.
    HALF_MIL_COUNTER++;

    // Get 100 * 0.5 ms to earn a 50 ms.
    if (HALF_MIL_COUNTER > 99)
    {
        Set_New_50_ms;
        FIFTY_MS_COUNTER++;
        HALF_MIL_COUNTER = 0;
    }

    if (FIFTY_MS_COUNTER > 20)
    {
        Set_New_Second;
        SECOND_COUNTER++;
        DPB_SECOND_COUNTER++;
        FIFTY_MS_COUNTER = 0;
        SCD_SECOND++; // USED TO TIME SCD READING SEQUENCE IN MAIN, RESET AFTER 10 SECONDS
        if(SCD_SECOND > 9){
            SCD_SECOND = 0;
        }
    }
    if (SECOND_COUNTER > 9)
    {
        Set_New_Ten_Second;
        TEN_SECOND_COUNTER++;
        SECOND_COUNTER = 0;
    }
    if (TEN_SECOND_COUNTER > 5)
    {
        Set_New_Minute;
        MINUTE_COUNTER++;
        TEN_SECOND_COUNTER = 0;
    }
    if (MINUTE_COUNTER > 59)
    {
        DPB_SECOND_COUNTER = 0;  // USED FOR TESTING TO GIVE A SECOND COUNT UP TO 3600
        Set_New_Hour;
        HOUR_COUNTER++;
        MINUTE_COUNTER = 0;
    }
    if (HOUR_COUNTER > 23)
    {
        // New day, reset hour counter
        HOUR_COUNTER = 0;
    }
    if (TRACK_I2C_TIME)
    {
        I2C_TIME++;
    }
    if (TRACK_UART_TIME)
    {
        UART_TIME++;
    }
    //Counter for wait_ms()
    if (MS_COUNTER)
    {
        MS_COUNTER--;
    }
}
