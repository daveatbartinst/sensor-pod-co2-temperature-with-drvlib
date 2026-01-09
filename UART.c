/**
 * @file UART.c
 * @author Nathaniel Suddarth
 * @brief Controls all UART communication with the Pulse related to the Sensirion SCD30 CO2 sensor.
 * @version 0.1
 * @date 2025-07-1
 *
 * @note Revised from and combined with the work of Emmanuel Akpan
 */
#include "UART.h"
#include "init_var.h"
//#include "driverlib.h"
//#include <stdint.h>
// Sensirion Constants:
static const uint8_t UART_crc_polynomial = crc_polynomial; // 0x31 (x^8 + x^5 + x^4 + 1).
static const uint8_t UART_initial_remainder = initial_remainder;


// Holds Converted Sensirion Read Values:
volatile uint8_t CO2_MSB = 0x00, CO2_LSB = 0x00, TEMPERATURE = 0x00, HUMIDITY = 0x00;
// Holds CoMmanD from Pulse:
volatile uint8_t CMD = CMD_RESET_STATE, SIZE = 0x00, ARG_MSB = 0x00, ARG_LSB = 0x00;
// Holds ARGument from Pulse:
volatile uint16_t ARG = 0x00;
// Holds number of bad transmissions from Pulse:
volatile uint16_t REDUX_MEASUREMENT = 0;

volatile uint32_t lazy_timer = 0;

// tx_buffer for READ data:
char UART_TX_READ_BUFFER[UART_TX_READ_BUFFER_SIZE];
// tx_buffer for COMMAND:
char UART_TX_COMMAND_BUFFER[UART_TX_COMMAND_BUFFER_SIZE];
uint8_t UART_TX_BUFFER_PTR = 0;

// rx_buffer Declaration:
char UART_RX_BUFFER[UART_RX_BUFFER_SIZE];
volatile uint8_t UART_RX_BUFFER_PTR = 0;

// String for Byte Conversions:
static char UART_BYTE_CONVERSION_STR[3];

// Flag initiates incrementing of UART_TIME.
volatile bool TRACK_UART_TIME = false;

// Tracks the UART time in Timer.c in half-mils.
volatile uint16_t UART_TIME = 0;

// Tracks the maximum TX time for UART in half-mils.
volatile uint16_t UART_PEAK_TX_TIME = 0;
// Tracks the maximum RX time for UART in half-mils.
volatile uint16_t UART_PEAK_RX_TIME = 0;

// TRUE Indicates that the READ buffer is to be sent to the Pulse. FALSE indicates that the COMMAND buffer is to be sent to the Pulse.
volatile bool TX_READ = false;

// Flag for main() to indicate a message needs parsed
volatile bool UART_PARSE_RX_FLG = false;

// Tracks the state of the current RX, ensures buffer is only filled with
// allowed chars in the correct format.
static uint8_t RX_BYTE_STATE = 0;

// Flip flops between TX/~RX of UART CMDs.
bool UART_TX_RX_FLIP_FLOP = true;
// Indicates that the CMD buffer is ready for filling.
bool UART_FILL_CMD_BUFFER_FLG = false;


/********************************************************************************************
 *                          UART STATIC HELPER FUNCTION DECLARATIONS:                       *
 ********************************************************************************************/

/**
 * @brief Calculates the CRC for the SCD30 sensor.
 *
 * @param input uint16_t to generate a CRC from.
 * @return uint8_t calculated CRC.
 */
static uint8_t UART_checksum_calculation(uint16_t input);

/**
 * @brief Checks whether a CRC received matches one calculated from the input value.
 *
 * @param input uint16_t for input data to generate a test CRC from.
 * @param CRC CRC received in transmission.
 * @return true if the test CRC is equal to the CRC received.
 * @return false if the test CRC is not equal to the CRC received.
 */
static bool UART_check_CRC(uint16_t input, uint8_t CRC);


/********************************************************************************************
 *                            UART STATIC HELPER FUNCTIONS:                                 *
 ********************************************************************************************/

static uint8_t UART_checksum_calculation(uint16_t input)
{
    uint8_t i;
    bool MSB;

    // Load initial remainder into top byte of "shift register" (top byte of
    // input)
    input ^= ((uint16_t)UART_initial_remainder) << One_Byte;
    for (i = 0; i < 16; i++)
    {
        // Check if most significant bit is set or cleared
        MSB = input & 0x8000;
        // Shift to line up with x^8 (bit9 position)
        input <<= 1;
        if (MSB) // If MSB is a 1
        {
            // E.g., 0x[1]31 = 0b[1]_0011_0001
            input ^= ((uint16_t)UART_crc_polynomial) << One_Byte;
        }
    }

    return (uint8_t)(input >> One_Byte);
}

static bool UART_check_CRC(uint16_t input, uint8_t CRC)
{
    uint8_t calculated_CRC = UART_checksum_calculation(input);
    return (calculated_CRC == CRC);
}


/********************************************************************************************
 *                                      UART FUNCTIONS:                                     *
 ********************************************************************************************/


void UART_init()
{
    // Set reset state
    UART_CTLW0 |= UCSWRST;

    // View pages 950 and 1037 of
    // https://www.tij.co.jp/jp/lit/ug/slau208q/slau208q.pdf, the MSP430x5xx and
    // MSP430x6xx Family User's Guide, for a more full explaination.

    // UART settings:
    // Baud rate: 9600
    // Data bits: 8
    // Parity: None
    // Stop bits: 1
    // Over samping: Yes

    // Configure UART @9600 baud
    // Calculations for baud rate are from
    // https://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html

    EUSCI_A_UART_initParam param = {0};
    param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    param.clockPrescalar = 6; // 104;
    param.firstModReg = 8;    // 2;
    param.secondModReg = 17;  // 182;
    param.parity = EUSCI_A_UART_NO_PARITY;
    param.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    param.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    param.uartMode = EUSCI_A_UART_MODE;
    param.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;

#ifdef USE_UART_0
    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }
    EUSCI_A_UART_enable(EUSCI_A0_BASE);
#else
    if (STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A1_BASE, &param))
    {
        return;
    }
    EUSCI_A_UART_enable(EUSCI_A1_BASE);

#endif
    __enable_interrupt();
}

char *hex_to_string(uint8_t v)
{
    if (v < 16)
    {
        snprintf(UART_BYTE_CONVERSION_STR, 3, "0%x", v);
        return UART_BYTE_CONVERSION_STR;
    }
    else
    {
        snprintf(UART_BYTE_CONVERSION_STR, 3, "%x", v);
        return UART_BYTE_CONVERSION_STR;
    }
}

// TODO: Fix Pulse parsing and move TX to main() so wait is half-mil based.
void UART_populate_TX_READ_buffer()
{
    uint8_t CRC0 = 0, CRC1 = 0, CRC2 = 0; // 0 is size + cmd. 1 is CO2 msb + lsb. 2 is temp + hum.
    char Size_str[2];
    char CMD_str[2];
    char CRC0_str[2];
    char CO2_MSB_str[2];
    char CO2_LSB_str[2];
    char CRC1_str[2];
    char TEMP_str[2];
    char RH_str[2];
    char CRC2_str[2];
    uint8_t i = 0;

    CRC0 = UART_checksum_calculation((((uint16_t)sizeof(UART_TX_READ_BUFFER)) << One_Byte) + CMD);
    CRC1 = UART_checksum_calculation((((uint16_t)CO2_MSB) << One_Byte) + CO2_LSB);
    CRC2 = UART_checksum_calculation((((uint16_t)TEMPERATURE) << One_Byte) + HUMIDITY);

    strcpy(Size_str, hex_to_string(sizeof(UART_TX_READ_BUFFER)));
    strcpy(CMD_str, hex_to_string(CMD));
    strcpy(CRC0_str, hex_to_string(CRC0));

    strcpy(CO2_MSB_str, hex_to_string(CO2_MSB));
    strcpy(CO2_LSB_str, hex_to_string(CO2_LSB));
    strcpy(CRC1_str, hex_to_string(CRC1));

    strcpy(TEMP_str, hex_to_string(TEMPERATURE));
    strcpy(RH_str, hex_to_string(HUMIDITY));
    strcpy(CRC2_str, hex_to_string(CRC2));

    snprintf(UART_TX_READ_BUFFER,
             sizeof(UART_TX_READ_BUFFER),
             "CO2:%s%s%s%s%s%s%s%s%s",
             Size_str,
             CMD_str,
             CRC0_str,
             CO2_MSB_str,
             CO2_LSB_str,
             CRC1_str,
             TEMP_str,
             RH_str,
             CRC2_str);

    // Null char to carriage return
    for (i = 0; i < sizeof(UART_TX_READ_BUFFER); i++)
    {
        if (UART_TX_READ_BUFFER[i] == '\0')
        {
            UART_TX_READ_BUFFER[i] = '\r';
        }
    }
}

void UART_populate_TX_COMMAND_buffer()
{
    volatile uint8_t CRC0 = 0, CRC1 = 0;
    char Size_str[2];
    char CMD_str[2];
    char CRC0_str[2];
    char ARG_MSB_str[2];
    char ARG_LSB_str[2];
    char CRC1_str[2];
    uint8_t i = 0;


    CRC0 = UART_checksum_calculation((((uint16_t)sizeof(UART_TX_COMMAND_BUFFER)) << One_Byte) + CMD);
    CRC1 = UART_checksum_calculation((((uint16_t)ARG_MSB) << One_Byte) + ARG_LSB);

    strcpy(Size_str, hex_to_string(sizeof(UART_TX_COMMAND_BUFFER)));
    strcpy(CMD_str, hex_to_string(CMD));
    strcpy(CRC0_str, hex_to_string(CRC0));

    strcpy(ARG_MSB_str, hex_to_string(ARG_MSB));
    strcpy(ARG_LSB_str, hex_to_string(ARG_LSB));
    strcpy(CRC1_str, hex_to_string(CRC1));

    snprintf(UART_TX_COMMAND_BUFFER,
             sizeof(UART_TX_COMMAND_BUFFER),
             "CO2:%s%s%s%s%s%s",
             Size_str,
             CMD_str,
             CRC0_str,
             ARG_MSB_str,
             ARG_LSB_str,
             CRC1_str);

    // Null char to carriage return
    for (i = 0; i < sizeof(UART_TX_COMMAND_BUFFER); i++)
    {
        if (UART_TX_COMMAND_BUFFER[i] == '\0')
        {
            UART_TX_COMMAND_BUFFER[i] = '\r';
        }
    }
}


void UART_RX_byte()
{

    if (UART_RX_BUFFER_PTR == 0)
    {
        // Indicate that interrupt is being handled.
     //   UART_TRANSMITTING_LED_on();
        //Setup for timeout.
      //  CLR_UART_STOPWATCH;
      //  START_UART_STOPWATCH;
    }

    //Ensure that hardware interrupt flag gets cleared as it may be untrustworthy.
    CLR_UART_RX_FULL;
    UART_RX_BUFFER[UART_RX_BUFFER_PTR] = UART_RX_BUF;

    switch (RX_BYTE_STATE)
    {
    case 0: // Waiting for '2'
        if (UART_RX_BUFFER[UART_RX_BUFFER_PTR] == '2')
        {
            UART_RX_BUFFER_PTR++;
            RX_BYTE_STATE++;
        }
        break;
    case 1: // Waiting for 'O'
        if (UART_RX_BUFFER[UART_RX_BUFFER_PTR] == 'O' || UART_RX_BUFFER[UART_RX_BUFFER_PTR] == 'o')
        {
            UART_RX_BUFFER_PTR++;
            RX_BYTE_STATE++;
        }
        else
        {
            UART_RX_BUFFER_PTR = 0;
            RX_BYTE_STATE = 0;
        }
        break;
    case 2: // Waiting for 'C'
        if (UART_RX_BUFFER[UART_RX_BUFFER_PTR] == 'C' || UART_RX_BUFFER[UART_RX_BUFFER_PTR] == 'c')
        {
            UART_RX_BUFFER_PTR++;
            RX_BYTE_STATE++;
        }
        else
        {
            UART_RX_BUFFER_PTR = 0;
            RX_BYTE_STATE = 0;
        }
        break;
    case 3: // Waiting for ':'
        if (UART_RX_BUFFER[UART_RX_BUFFER_PTR] == ':')
        {
            UART_RX_BUFFER_PTR++;
            RX_BYTE_STATE++;
        }
        else
        {
            UART_RX_BUFFER_PTR = 0;
            RX_BYTE_STATE = 0;
        }
        break;
    case 4: // Fill buffer with ASCII-encoded hex characters and '\r' as an end of line character
        // If (buffer not full)
        if (UART_RX_BUFFER_PTR < sizeof(UART_RX_BUFFER) - 1)
        {
            // If (an ASCII-encoded hex character)
            if (((UART_RX_BUFFER[UART_RX_BUFFER_PTR] >= '0' && UART_RX_BUFFER[UART_RX_BUFFER_PTR] <= '9') ||
                 (UART_RX_BUFFER[UART_RX_BUFFER_PTR] >= 'A' && UART_RX_BUFFER[UART_RX_BUFFER_PTR] <= 'F') ||
                 (UART_RX_BUFFER[UART_RX_BUFFER_PTR] >= 'a' && UART_RX_BUFFER[UART_RX_BUFFER_PTR] <= 'f')))
            {
                UART_RX_BUFFER_PTR++; // Get next char
            } // End if (an ASCII-encoded hex character)
            else
            {
                // Invalid character, go back to starting state & disregard read.
                UART_RX_BUFFER_PTR = 0;
                RX_BYTE_STATE = 0;
            }
        } // End if (buffer not full)
        // Buffer is full of correctly formatted characters
        else if (UART_RX_BUFFER_PTR >= sizeof(UART_RX_BUFFER) - 1)
        {
            if (UART_RX_BUFFER[UART_RX_BUFFER_PTR] == '\r')
            {
                UART_RX_BUFFER_PTR = 0;
                RX_BYTE_STATE = 0;
                SET_UART_PARSE_RX_FLG;
                DISABLE_UART_RX;
            }
            else
            {
                // Invalid EOL character, go back to starting state & disregard read.
                UART_RX_BUFFER_PTR = 0;
                RX_BYTE_STATE = 0;
            }
        } // End if (buffer full)
        break;

    default:
        // Should not occur
        UART_RX_BUFFER_PTR = 0;
        RX_BYTE_STATE = 0;
        break;
    }

} // end uart_rx_byte

void UART_parse_RX_buffer()
{
    // Clear flag.
    CLR_UART_PARSE_RX_FLG;

    volatile uint8_t command_ptr = 0;
    char info_buf[4];
    volatile uint16_t arg = 0;
    volatile uint8_t CRC0, CRC1, arg_msb, arg_lsb, cmd, size;
    volatile uint8_t command_ptr_offset = 0;
    volatile bool correct_CRC = true;

    info_buf[0] = '0';
    info_buf[1] = 'x';
    // snprintf(str, sizeof(str),"O and carriage ret");

    command_ptr_offset = 4;

    info_buf[2] = UART_RX_BUFFER[command_ptr + (command_ptr_offset++)];
    info_buf[3] = UART_RX_BUFFER[command_ptr + (command_ptr_offset++)];
    size = strtoul(info_buf, NULL, 16);

    info_buf[2] = UART_RX_BUFFER[command_ptr + (command_ptr_offset++)];
    info_buf[3] = UART_RX_BUFFER[command_ptr + (command_ptr_offset++)];
    cmd = strtoul(info_buf, NULL, 16);

    info_buf[2] = UART_RX_BUFFER[command_ptr + (command_ptr_offset++)];
    info_buf[3] = UART_RX_BUFFER[command_ptr + (command_ptr_offset++)];
    CRC0 = strtoul(info_buf, NULL, 16);

    info_buf[2] = UART_RX_BUFFER[command_ptr + (command_ptr_offset++)];
    info_buf[3] = UART_RX_BUFFER[command_ptr + (command_ptr_offset++)];
    arg_msb = strtoul(info_buf, NULL, 16);

    info_buf[2] = UART_RX_BUFFER[command_ptr + (command_ptr_offset++)];
    info_buf[3] = UART_RX_BUFFER[command_ptr + (command_ptr_offset++)];
    arg_lsb = strtoul(info_buf, NULL, 16);

    arg = (((uint16_t)arg_msb) << 8) + arg_lsb;

    info_buf[2] = UART_RX_BUFFER[command_ptr + (command_ptr_offset++)];
    info_buf[3] = UART_RX_BUFFER[command_ptr + (command_ptr_offset++)];
    CRC1 = strtoul(info_buf, NULL, 16);

    correct_CRC &= UART_check_CRC((((uint16_t)size) << One_Byte) + cmd, CRC0);
    correct_CRC &= UART_check_CRC(arg, CRC1);


    if (correct_CRC)
    {
        // Valid command received.
        CMD = cmd;
        ARG = arg;
        SIZE = size;

        // Once CMD set, command received will be acted upon within main().
    }
    else
    {
        // TODO: Handle bad reads.
        REDUX_MEASUREMENT++;
        // Move back to receiving to prevent program stall.
        UART_RECEIVE_SETUP;
        // Wait for receive_setup. Necessary wait to ensure reliable stop bit behavior.
        Lazy_Wait(250);
    }
}


void handle_UART_TIMEOUT()
{
    // Set back to receive.
  //  UART_TRANSMITTING_LED_off();
    UART_RX_BUFFER_PTR = 0;
    UART_TX_BUFFER_PTR = 0;
    STOP_UART_STOPWATCH;
    CLR_UART_STOPWATCH;
    //TODO: Throw timeout error?
    UART_RECEIVE_SETUP;
    // Wait for receive_setup. Necessary wait to ensure reliable stop bit behavior.
    Lazy_Wait(250);
}


#pragma vector = EUSCI_A1_VECTOR
__interrupt void USCI_A1_Handler(void)
{
    switch (__even_in_range(UCAxIV, 18))
    {
    case 0x00: // Vector 0: No interrupts
        break;
    case 0x02: // Vector 2: Receive Interrupt FlaG (UCRXIFG)
        UART_RX_byte();
        if (UART_TIME > UART_PEAK_RX_TIME)
        {
            UART_PEAK_RX_TIME = UART_TIME;
        }
        break;
    case 0x04: // Vector 4: Transmit Interrupt FlaG (UCTXIFG)
        if (UART_TX_BUFFER_PTR == 0)
        {
            //Setup for timeout.
            CLR_UART_STOPWATCH;
            START_UART_STOPWATCH;
            Lazy_Wait(250);
        }
        if (TX_READ)
        {
            if (UART_TX_BUFFER_PTR < sizeof(UART_TX_READ_BUFFER))
            {
                //Ensure that hardware interrupt flag gets cleared as it may be untrustworthy.
                CLR_UART_TX_EMPTY;
                // Transmit byte by byte.
                UART_TX_BUF = UART_TX_READ_BUFFER[UART_TX_BUFFER_PTR++];
                Lazy_Wait(2000); // Wait between TX chars. Not necessary, but increases Pulse RX success.
            }
            if (UART_TX_BUFFER_PTR >= sizeof(UART_TX_READ_BUFFER))
            {
                // Successful transmission.
                // TODO: Mark successful transmission?
                if (UART_TIME > UART_PEAK_TX_TIME)
                {
                    UART_PEAK_TX_TIME = UART_TIME;
                }

                // Remain always receiving unless told to transmit. Sub-module to Pulse.
                UART_RECEIVE_SETUP;
                STOP_UART_STOPWATCH;
           //     UART_TRANSMITTING_LED_off();
                // Wait for send_setup. Necessary wait to ensure reliable stop bit behavior.
                Lazy_Wait(250);
            }
        }
        // TX_COMMAND, use COMMAND_BUFFER and corresponding format.
        else
        {
            if (UART_TX_BUFFER_PTR < sizeof(UART_TX_COMMAND_BUFFER))
            {

                //Ensure that hardware interrupt flag gets cleared as it may be untrustworthy.
                CLR_UART_TX_EMPTY;
                // Transmit byte by byte.
                UART_TX_BUF = UART_TX_COMMAND_BUFFER[UART_TX_BUFFER_PTR++];
                Lazy_Wait(2000); // Wait between TX chars. Not necessary, but increases Pulse RX success
            }
            if (UART_TX_BUFFER_PTR >= sizeof(UART_TX_COMMAND_BUFFER))
            {
                // Successful transmission.
                // TODO: Mark successful transmission?
                if (UART_TIME > UART_PEAK_TX_TIME)
                {
                    UART_PEAK_TX_TIME = UART_TIME;
                }

                // Remain always receiving unless told to transmit. Sub-module to Pulse.
                UART_RECEIVE_SETUP;
                STOP_UART_STOPWATCH;
           //     UART_TRANSMITTING_LED_off();
                // Wait for send_setup. Necessary wait to ensure reliable stop bit behavior.
                Lazy_Wait(250);
            }
            break;
        }
    case 0x06: // Vector 6: StarT bit Interrupt FlaG (UCSTTIFG)
        break;
    case 0x08: // Vector 8: Transmit ready Interrupt FlaG (UCTXCPTIFG)
        break;
    default:
        // Should not occur
        break;
    }
}
