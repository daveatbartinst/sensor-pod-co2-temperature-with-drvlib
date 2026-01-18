/**
 * @file UART.h
 * @author Nathaniel Suddarth
 * @brief Controls all UART communication with the Pulse related to the Sensirion SCD30 CO2 sensor.
 * @version 0.1
 * @date 2025-07-1
 *
 * @note Revised from and combined with the work of Emmanuel Akpan
 */

#ifndef UART_H_
#define UART_H_

//#include "I2C.h"
////#include "LED.h"
//#include "Timer.h"
////#include "init_var.h"
//
////#include "driverlib.h"
//#include "msp430.h"
//#include <intrinsics.h>
////
#include <stdbool.h>
#include <stdint.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>


// Flag initiates incrementing of UART_TIME.
extern volatile bool TRACK_UART_TIME;

// Tracks the UART time in Timer.c in half-mils.
extern volatile uint16_t UART_TIME;

// Tracks the maximum TX time for UART in half-mils.
extern volatile uint16_t UART_PEAK_TX_TIME;
// Tracks the maximum RX time for UART in half-mils.
extern volatile uint16_t UART_PEAK_RX_TIME;

// TRUE Indicates that the READ buffer is to be sent to the Pulse. FALSE indicates that the COMMAND buffer is to be sent to the Pulse.
extern volatile bool TX_READ;

// tx_buffer for READ:
extern char UART_TX_READ_READ_BUFFER[];
// tx_buffer for COMMAND:
extern char UART_TX_COMMAND_BUFFER[];
extern uint8_t UART_TX_BUFFER_PTR;

extern char UART_RX_BUFFER[];
extern volatile uint8_t UART_RX_BUFFER_PTR;

// Flag for main() to indicate a message needs parsed
extern volatile bool UART_PARSE_RX_FLG;

// Holds Converted Sensirion Read Values:
extern volatile uint8_t CO2_MSB, CO2_LSB, TEMPERATURE, HUMIDITY;
// Holds CoMmanD from Pulse:
extern volatile uint8_t CMD, SIZE, ARG_MSB, ARG_LSB;
// Holds ARGument from Pulse:
extern volatile uint16_t ARG;
// Holds number of bad transmissions from Pulse:
extern volatile uint16_t REDUX_MEASUREMENT;
// Quick way to modify small timing issues on UART.
// TODO: modify later.
extern volatile uint32_t lazy_timer;

// Flip flops between TX/~RX of UART CMDs.
extern bool UART_TX_RX_FLIP_FLOP;
// Indicates that the CMD buffer is ready for filling.
extern bool UART_FILL_CMD_BUFFER_FLG;


/**
 * @brief Initializes all UART-relevant parameters.
 */
void UART_init();

/**
 * @brief Converts hex values to a string.
 *
 * @param v Value in hex.
 * @return char* string.
 */
char *hex_to_string(uint8_t v);

/**
 * @brief Populates the TX buffer with READ data to transmit.
 */
void UART_populate_TX_READ_buffer();

/**
 * @brief Populates the TX buffer with COMMAND data to transmit.
 */
void UART_populate_TX_COMMAND_buffer();

/**
 * @brief Performs the task of receiving a byte from UART. Ensures all formatting should be correct for parsing.
 *
 */
void UART_RX_byte();

/**
 * @brief Parses the RX buffer into its various components. Checks CRCs.
 */
void UART_parse_RX_buffer();

/**
 * @brief Resets communication in case of UART timeout state.
 *
 */
void handle_UART_TIMEOUT();


#endif /* UART_H_ */
