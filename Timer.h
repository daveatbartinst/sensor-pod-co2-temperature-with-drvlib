/**
 * @file Timer.h
 * @author Nathaniel Suddarth
 * @brief Provides timing for mandatory wait periods and timed interrupts.
 * @version 0.1
 * @date 2025-06-12
 *
 *
 */

#ifndef TIMER_H_
#define TIMER_H_


//#include "UART.h"
////#include "driverlib.h"
////#include "init_var.h"
//#include "msp430.h"
#include <stdint.h>
#include <stdio.h>
//

//Timing flags and counters.
extern volatile uint8_t TIMING_FLAG_BYTE;
extern volatile uint8_t HALF_MIL_COUNTER;
extern volatile uint16_t MS_COUNTER;
extern volatile uint16_t FIFTY_MS_COUNTER;
extern volatile uint16_t FIVE_SECOND_COUNTER;
extern volatile uint16_t MINUTE_COUNTER;
extern volatile uint16_t HOUR_COUNTER;

/**
 * @brief Initialize all timer-related settings. Uses Timer A and SMCLOCK.
 */
void timer_A_init();

/**
 * @brief Wait input number of milliseconds up to a maximum of 32,767 milliseconds.
 *
 * @param ms Number of milliseconds to wait.
 */
void wait_ms(uint16_t ms);


#endif /* TIMER_H_ */
