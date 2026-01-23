/**
 * @file I2C.c
 * @author Nathaniel Suddarth
 * @brief Controls all I2C communication, setting modification, and data parsing related to the Sensirion SCD30 CO2 sensor.
 * @version 0.1
 * @date 2025-06-12
 *
 *
 */
#include "msp430.h"
#include "I2C.h"
#include "init_var.h"
#include "main.h"   // would  not build with main.h  I2C_Status was not found
#include "timer.h"
#include <math.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>






// Sensirion Constants:



 const uint8_t Sensirion_crc_polynomial = crc_polynomial; // 0x31 (x^8 + x^5 + x^4 + 1).
 const uint8_t Sensirion_initial_remainder = initial_remainder;

// TX and RX Buffers:
uint8_t I2C_TX_DATA[I2C_Transmit_Buffer_Size]; // Data transmitted
uint8_t I2C_RX_DATA[I2C_Receive_Buffer_Size];  // Data received
uint16_t I2C_TRANSACTION_HOLDER [24]; //time Data received
// Read Buffer:
 volatile struct I2C_RX_Meas_t I2C_RX_READINGS[I2C_Readings_Buffer_Size]; // Data received after cleaning,
                                                                                // held in a struct.

// Buffer Indices:
 volatile uint8_t I2C_TX_INDEX = 0;          // Index for TX data arr.
 volatile uint8_t I2C_RX_INDEX = 0;          // Index for RX data arr.
 volatile uint8_t I2C_RX_READINGS_INDEX = 0; // Index for RX_READINGS arr.

// Command Sizes:
 uint8_t I2C_TX_CMD_SIZE = 0; // Size of command to be transmitted, used to throw STOP.
 uint8_t I2C_RX_CMD_SIZE = 0; // Size of command to be received, used to throw STOP.

// Boolean Condition Flags:
// Used to determine successful transmission status.
// volatile bool I2C_SUCCESS_FLG = false;
// Used to select which peak time to update.
volatile uint8_t I2C_MODE_BYTE = 0;
// 0-1 if set() function, 2-4 if get() function. Used by I2C_update_LED().
// enum LED_STATE I2C_LED_FLG_BYTE = NOT_IN_TRANSMISSION;
// Holds the return LED state once TX complete. Used by I2C_update_LED().
// enum LED_STATE RETURN_I2C_LED_FLG_BYTE = NOT_IN_TRANSMISSION;
// Remains true if CRC is correct.
 bool CRC_CHECK = true;


//READ  Variables:
 uint32_t CO2 = 0, T = 0, RH = 0;
 uint8_t READ_iterator = 0;
 struct I2C_RX_Meas_t *measurement_ptr;

// Pointer containing location in which to store Sensirion command's returned value.
 volatile uint16_t *I2C_RX_argument_ptr = 0;

// Peak I2C Times:
// The time spent between the Start and Stop of a TX only.
volatile uint16_t I2C_PEAK_TX_TIME = 0;
// The time spent between the Start and Stop of a RX only.
volatile uint16_t I2C_PEAK_RX_TIME = 0;
/* The time spent between the Start and Stop of a RX only for reading sensor
 * data. */
volatile uint16_t I2C_PEAK_READ_TIME = 0;
// Tracks the TX/RX/READ time in Timer.c in half-mils.
volatile uint16_t I2C_TIME = 0;
// Flag initiates incrementing of I2C_TIME.
volatile bool TRACK_I2C_TIME = false;

//Maximum and Minimum Temp, CO2, and Relative Humidity:
volatile uint16_t max_CO2 = 0, min_CO2 = 0xFFFF;
volatile uint8_t max_T = 0, min_T = 0xFF;
volatile uint8_t max_RH = 0, min_RH = 0xFF;

// Counts the number of bad reads:
volatile uint16_t I2C_REDUX_COUNT = 1;


/********************************************************************************************
 *                 SENSIRION SCD30 I2C  HELPER FUNCTION DECLARATIONS:                 *
 ********************************************************************************************/

/**
 * @brief Helper function to set master to transmitter mode, ready for START.
 */
 void I2C_master_TX();

/**
 * @brief Helper function to set master to receiver mode. The master holds the bus until UCBxRXBUF is read.
 */
 void I2C_master_RX();

/**
 * @brief Helper function to start communication by setting the START bit.
 */
 void I2C_START();

/**
 * @brief Helper function to set the STOP bit to end transmission.
 */
 void I2C_STOP();

/**
 * @brief Helper function to disable all I2C interrupts.
 */
 void I2C_disable_interrupts();

/**
 * @brief Helper function to generate the checksum (CRC) byte.
 *
 * Name: CRC-8. Protected Data: read data.
 * Polynomial: 0x31 (x^8 + x^5 + x^4 + 1). Initialization: 0xFF. Reflect Input:
 * false. Reflect Output: false. Final XOR: 0x00. Example: CRC(0xBEEF) = 0x92.
 *
 * Only use the previous two bytes sent when CRC byte is called for.
 */
 uint8_t Sensirion_checksum_calculation(uint16_t input);

/**
 * @brief Helper function to check the checksum (CRC) byte. Ensures calculated value is
 * equal to the received value.
 *
 * Only use the previous two bytes sent before CRC byte as input.
 */
 bool Sensirion_check_CRC(uint16_t input, uint8_t CRC);

/**
 * @brief Helper function to automate the process of loading data into the TX_DATA buffer.
 * I2C_init_master() must be called before using this function.
 *
 * Sets all necessary settings for TX including START.
 * For sending only the command, set byte_num to 2 bytes transmitted.
 */
 void I2C_load_TX_DATA(uint16_t command, uint16_t argument, uint8_t byte_num);

/**
 * @brief Helper function to automate the process of loading data into the RX_DATA buffer.
 *
 * For receiving only the argument, set byte_num to 3 bytes received,
 * 2 bytes of argument + 1 byte CRC.*Required 3ms wait time is included.
 */
 void I2C_load_RX_DATA(uint8_t byte_num);


/**
 * @brief Helper function to parse the RX_Data buffer into individual arguments.
 * CRC bytes are confirmed.
 *
 * @param argument
 */
 void I2C_parse_READ_DATA_element(uint32_t *argument);

/**
 * @brief Helper function to type pun from uint32_t to float without data conversion.
 *
 * @param argument
 * @return float
 */
 float I2C_bytes_to_float(uint32_t *argument);

/**
 * @brief Helper function to add newest reading "meas" to the reading buffer.
 *
 * The element at index 0 is the oldest, and is the first to be overwritten by an element shift.
 * The element at the last index is the newest reading "meas."
 */
 void add_to_RX_READINGS();

/**
 * @brief Helper function to update the maximum TX/RX/READ time of I2C communication with the sensor.
 */
 void I2C_update_peak_time();

/********************************************************************************************
 *                  SENSIRION SCD30  HELPER FUNCTIONS:                                *
 ********************************************************************************************/

 void I2C_master_TX()
{
    // Put eUSCI_B in reset state, clears flags and enabled interrupts
    UCBxCTLW0 |= UCSWRST;
    // Transmitter mode
    UCBxCTLW0 |= UCTR;
    // eUSCI_B in operational state
    UCBxCTLW0 &= ~UCSWRST;
    // enable TX-interrupt, NACK-interrupt, Clock low timeout interrupt (+
    // UCCLTOIE;)
    UCBxIE = UCTXIE + UCNACKIE;

    // general interrupt enable
    __enable_interrupt();
}

 void I2C_master_RX()
{
    // put eUSCI_B in reset state, clears flags and enabled interrupts
    UCBxCTLW0 |= UCSWRST;
    // Receiver mode
    UCBxCTLW0 &= ~UCTR;
    // eUSCI_B in operational state
    UCBxCTLW0 &= ~UCSWRST;
    // enable TX-interrupt, RX-interrupt
    UCBxIE = UCTXIE + UCRXIE;
    // general interrupt enable
    __enable_interrupt();
}

 void I2C_disable_interrupts()
{
    // Disable I2C interrupts
    UCBxIE = 0;
}

 void I2C_START()
{
    // Clear TX index for new transmission...old will be overwritten
    I2C_TX_INDEX = 0;
    // Clear RX index for new transmission...old will be overwritten
    I2C_RX_INDEX = 0;
    // Indicate transmission in progress
 //   I2C_LED_FLG_BYTE = IN_TRANSMISSION;
    // Ensure no STOP
    UCBxCTLW0 &= ~(UCTXSTP);
    // Clear half-mil counter for I2C time
    CLR_I2C_STOPWATCH;
    START_I2C_STOPWATCH;
    // Transmit START
    UCBxCTLW0 |= UCTXSTT;
}

 void I2C_STOP()
{
    // Ensure no START
    UCBxCTLW0 &= ~(UCTXSTT);
    // Transmit STOP
    UCBxCTLW0 |= UCTXSTP;
    // Transmission completed, toggle interrupts
    I2C_disable_interrupts();
  //  I2C_LED_FLG_BYTE = NOT_IN_TRANSMISSION;
}

 uint8_t Sensirion_checksum_calculation(uint16_t input)
{
    uint8_t i;
    bool MSB;

    // Load initial remainder into top byte of "shift register" (top byte of
    // input)
    input ^= (uint16_t)Sensirion_initial_remainder << One_Byte;
    for (i = 0; i < 16; i++)
    {
        // Check if most significant bit is set or cleared
        MSB = input & 0x8000;
        // Shift to line up with x^8 (bit9 position)
        input <<= 1;
        if (MSB) // If MSB is a 1
        {
            // E.g., 0x[1]31 = 0b[1]_0011_0001
            input ^= (uint16_t)Sensirion_crc_polynomial << One_Byte;
        }
    }

    return (uint8_t)(input >> One_Byte);
}

 bool Sensirion_check_CRC(uint16_t input, uint8_t CRC)
{
    uint8_t calculated_CRC = Sensirion_checksum_calculation(input);

    __no_operation(); //For debugging
    return (calculated_CRC == CRC);
}

 void I2C_load_TX_DATA(uint16_t command, uint16_t argument, uint8_t byte_num)
{
    I2C_TX_DATA[0] = (command & 0xFF00) >> One_Byte;           // Load MSB
    I2C_TX_DATA[1] = command & 0x00FF;                         // Load LSB
    I2C_TX_DATA[2] = (argument & 0xFF00) >> One_Byte;          // Load MSB
    I2C_TX_DATA[3] = argument & 0x00FF;                        // Load LSB
    I2C_TX_DATA[4] = Sensirion_checksum_calculation(argument); // Load CRC for Arg
    I2C_TX_CMD_SIZE = byte_num;


    // Reset success and update flags
    Clr_I2C_Success_Flag ;
 //   I2C_SUCCESS_FLG = false;
    SET_TX;
    I2C_master_TX();
    I2C_START();
}

 void I2C_load_RX_DATA(uint8_t byte_num)
{
    Clr_I2C_Success_Flag ;
   // I2C_SUCCESS_FLG = false;

    if (!IS_READ)
    {
        SET_RX;
    }

    I2C_RX_CMD_SIZE = byte_num;
    // Ready to receive
    I2C_master_RX();
    I2C_START();
}

// TODO: Implement a circular buffer to RX_Readings?
 void add_to_RX_READINGS()
{
    uint8_t i = 0;






    if (I2C_RX_READINGS_INDEX < I2C_Readings_Buffer_Size)
    {
        // Add to log of readings
      //  I2C_TRANSACTION_HOLDER[I2C_RX_READINGS_INDEX] = DPB_SECOND_COUNTER;
        I2C_RX_READINGS[I2C_RX_READINGS_INDEX++] = *measurement_ptr;

    }
    else
    {
        for (i = 0; i < I2C_Readings_Buffer_Size - 1; i++)
        {
            // Shift readings over, remove oldest
            I2C_RX_READINGS[i] = I2C_RX_READINGS[i + 1];
          //  I2C_TRANSACTION_HOLDER[i] = I2C_TRANSACTION_HOLDER[i+1];
        }
        // Add newest to "end" of log
       // I2C_TRANSACTION_HOLDER[I2C_Readings_Buffer_Size - 1] = DPB_SECOND_COUNTER;
        I2C_RX_READINGS[I2C_Readings_Buffer_Size - 1] = *measurement_ptr;

        if ((I2C_RX_READINGS[I2C_Readings_Buffer_Size - 1].CO2 > max_CO2) && (I2C_RX_READINGS[I2C_Readings_Buffer_Size - 1].CO2 < 5000))
         {
             max_CO2 = I2C_RX_READINGS[I2C_Readings_Buffer_Size - 1].CO2;
         }
         else if ((I2C_RX_READINGS[I2C_Readings_Buffer_Size - 1].CO2 < min_CO2) && (I2C_RX_READINGS[I2C_Readings_Buffer_Size - 1].CO2 > 0))
         {
             min_CO2 = I2C_RX_READINGS[I2C_Readings_Buffer_Size - 1].CO2;
         }


    }
}

 void I2C_parse_READ_DATA_element(uint32_t *element)
{
    *element = (uint32_t)I2C_RX_DATA[(READ_iterator)++] << Three_Bytes; // Load MMSB
    *element += (uint32_t)I2C_RX_DATA[(READ_iterator)++] << Two_Bytes;  // Load MLSB
    (READ_iterator)++;                                                  // Skip first CRC

    *element += (uint32_t)I2C_RX_DATA[(READ_iterator)++] << One_Byte; // Load LMSB
    *element += (uint32_t)I2C_RX_DATA[(READ_iterator)++];             // Load LLSB
    // Pause iterator at second CRC

    // Check first CRC as uint16_t, truncate accordingly.
    bool crc0 = Sensirion_check_CRC((uint16_t)((*element) >> Two_Bytes), I2C_RX_DATA[(READ_iterator)-3]);
    // Check second CRC as uint16_t, truncate accordingly. Increment iterator
    // after second CRC processed.
    bool crc1 = Sensirion_check_CRC((uint16_t)(*element), I2C_RX_DATA[(READ_iterator)++]);
    // CRC_CHECK holds true until a false crc occurrs. Initialized to true.
    CRC_CHECK = true;
    CRC_CHECK = CRC_CHECK && (crc0 && crc1);
}

 float I2C_bytes_to_float(uint32_t *element)
{
    // Using a union is considered correct practice over type punning with a pointer.
    union
    {
        uint32_t bytes;
        float float_bytes;
    } conversion;

    // The data is preserved through type punning, which would be altered through simple casting to type. Data is rounded.
    conversion.bytes = *element;
    return roundf(conversion.float_bytes);
}


 void I2C_update_peak_time()
{
    if ((I2C_TIME > I2C_PEAK_TX_TIME) && IS_TX)
    {
        I2C_PEAK_TX_TIME = I2C_TIME;
    }

    else if ((I2C_TIME > I2C_PEAK_RX_TIME) && IS_RX)
    {
        I2C_PEAK_RX_TIME = I2C_TIME;
    }

    else if ((I2C_TIME > I2C_PEAK_READ_TIME) && IS_READ)
    {
        I2C_PEAK_READ_TIME = I2C_TIME;
    }
    CLR_I2C_MODE_BYTE;
}

/********************************************************************************************
 *                  SENSIRION SCD30 I2C FUNCTIONS:                                          *
 ********************************************************************************************/

void UCB1_I2C_init_master(uint16_t baud_rate, uint16_t slave_address)
{
    // put eUSCI_B in reset state
    UCB1CTLW0 |= UCSWRST;
    // I2C master mode
    UCB1CTLW0 |= UCMODE_3 + UCMST;
    // baud rate = SMCLK / 0x140; 16MHz / 320 = 50 kHz, recommended for Sensiron
    // sensor
    UCB1BRW = baud_rate;
    // Manually add stop bits after transmission complete, enable clock low
    // timeout (+ UCCLTO_3;)
    UCB1CTLW1 = UCASTP_0;
    // address slave is 61hex (Sensirion)
    UCB1I2CSA = slave_address;
}

//void I2C_init_slave(uint16_t slave_address)
//{
//    // put eUSCI_B in reset state
//    UCBxCTLW0 |= UCSWRST;
//    // I2C slave mode
//    UCBxCTLW0 |= UCMODE_3;
//    // own address is 0x61 (Sensirion)
//    UCBxI2COA0 = slave_address + UCOAEN;
//    // eUSCI_B in operational state
//    UCBxCTLW0 &= ~UCSWRST;
//    // enable TX-interrupt, RX-interrupt
//    UCBxIE = UCTXIE + UCRXIE;
//    // Clear TX index for new transmission...old will be overwritten
//    I2C_TX_INDEX = 0;
//    // Clear RX index for new transmission...old will be overwritten
//    I2C_RX_INDEX = 0;
//    // general interrupt enable
//    __enable_interrupt();
//}

void I2C_avg_readings(uint32_t *CO2_avg, uint16_t *T_avg, uint16_t *RH_avg)
{
    uint32_t co2_sum = 0;
    uint16_t t_sum = 0;
    uint16_t rh_sum = 0;
    *CO2_avg = *T_avg = *RH_avg = 0;
    uint8_t i;

    // Sum the data.
    for (i = 0; i < I2C_Readings_Buffer_Size; i++)
    {
        co2_sum += I2C_RX_READINGS[i].CO2;
        t_sum += I2C_RX_READINGS[i].T;
        rh_sum += I2C_RX_READINGS[i].RH;
    }

    // Cast sums to float, divide, round those values (as floats still), cast back to uintXX_t
    *CO2_avg = (uint32_t)roundf(((float)co2_sum) / I2C_Readings_Buffer_Size);
    *T_avg = (uint16_t)roundf(((float)t_sum) / I2C_Readings_Buffer_Size);
    *RH_avg = (uint16_t)roundf(((float)rh_sum) / I2C_Readings_Buffer_Size);

    // Set maximum and minimum values if conditions met. Max/min are set for averaged buffer values,
    // not for individual readings.
//    if (*CO2_avg > max_CO2)
//    {
//        max_CO2 = *CO2_avg;
//    }
//    else if ((*CO2_avg < min_CO2) && (*CO2_avg > 0))
//    {
//        min_CO2 = *CO2_avg;
//    }

    if (*T_avg > max_T)
    {
        max_T = *T_avg;
    }
    else if ((*T_avg < min_T) && (*T_avg > 0))
    {
        min_T = *T_avg;
    }

    if (*RH_avg > max_RH)
    {
        max_RH = *RH_avg;
    }
    else if ((*RH_avg < min_RH) && (*RH_avg > 0))
    {
        min_RH = *RH_avg;
    }
}


void I2C_parse_RX_DATA()
{
    if (IS_READ)
    {
        // Parse as sensor read.
        READ_iterator = 0;
        CRC_CHECK = true;
        I2C_parse_READ_DATA_element(&CO2);
        I2C_parse_READ_DATA_element(&T);
        I2C_parse_READ_DATA_element(&RH);
        measurement_ptr->CO2 = 0;
        measurement_ptr->T = 0;
        measurement_ptr->RH = 0;

        if (CRC_CHECK &&  I2C_Success_Flag )
        {
            // Type punning the (rounded) byte data to float. Float is rounded and then
            // cast to size for later UART transmission. Pointer is used to ensure byte
            // data is not cast incorrectly. Temperature stored as an int8_t for negative
            // values.
            measurement_ptr->CO2 = (uint16_t)I2C_bytes_to_float(&CO2);
            measurement_ptr->T = (int8_t)I2C_bytes_to_float(&T);
            measurement_ptr->RH = (uint8_t)I2C_bytes_to_float(&RH);

            add_to_RX_READINGS();

            // Transmission success, indicate to update as a successful READ.
          //  I2C_LED_FLG_BYTE = VALID_READ;
            // struct meas fully initialized.
        }
        else
        {
            // Transmission failure, indicate to update as a failed READ.
         //   I2C_LED_FLG_BYTE = INVALID_READ;
            // struct meas initialized to 0s.
        }
    } // End parse as sensor read.
    else
    {
        // Parse as traditional RX.
      //  I2C_LED_FLG_BYTE = RETURN_I2C_LED_FLG_BYTE;
        *I2C_RX_argument_ptr = I2C_RX_DATA[0] << One_Byte;
        *I2C_RX_argument_ptr += I2C_RX_DATA[1];
    } // End parse as traditional RX.
}

//void I2C_update_LED()
//{
//    switch (I2C_LED_FLG_BYTE)
//    {
//    case VALID_SET_FUNCT:
//        // If 0, valid set() argument value. LED depends on success.
//        if (I2C_SUCCESS_FLG)
//        {
//            I2C_COMMAND_SUCCESS_LED_on();
//        }
//        else
//        {
//            I2C_COMMAND_FAILURE_LED_on();
//            // TODO: Error message?
//        }
//        break;
//    case INVALID_SET_FUNCT:
//        // If 1, invalid set() argument value. Yellow LED.
//        I2C_INVALID_ARGUMENT_LED_on();
//        // TODO: Error message?
//        break;
//    case GET_FUNCT:
//        // If 2, regular get() function. LED depends on success & CRC.
//        if (Sensirion_check_CRC(*I2C_RX_argument_ptr, I2C_RX_DATA[2]) && I2C_SUCCESS_FLG)
//        {
//            I2C_COMMAND_SUCCESS_LED_on();
//        }
//        else
//        {
//            I2C_COMMAND_FAILURE_LED_on();
//            I2C_REDUX_COUNT++;
//        }
//        break;
//    case VALID_READ:
//        // If 3, successful READ.
//        I2C_COMMAND_SUCCESS_LED_on();
//        break;
//    case INVALID_READ:
//        // If 4, failed READ.
//        I2C_COMMAND_FAILURE_LED_on();
//        break;
//    case IN_TRANSMISSION:
//        //If 5, currently transmitting.
//        I2C_TRANSMITTING_LED_on();
//        break;
//    case NOT_IN_TRANSMISSION:
//        //If 6, currently out of transmission.
//        I2C_TRANSMITTING_LED_off();
//        break;
//    case IN_SETUP:
//        //If 7, currently in main() setup.
//        I2C_SETUP_LED_on();
//        break;
//    default:
//        break;
//    }
//}

void clear_min_max()
{
    max_CO2 = 0, min_CO2 = 0;
    max_T = 0, min_T = 0;
    max_RH = 0, min_RH = 0;
}

void handle_I2C_TIMEOUT()
{
    //DEBUG Hasn't been thoroughly tested.
    I2C_RX_INDEX = 0;
    I2C_TX_INDEX = 0;
    STOP_I2C_STOPWATCH;
    CLR_I2C_STOPWATCH;

    I2C_REDUX_COUNT++;

    I2C_STOP(); // Transmit STOP.
   // I2C_LED_FLG_BYTE = NOT_IN_TRANSMISSION;
    // Transmission failure, don't update times/LEDs/Parse.
    Clr_I2C_Success_Flag ;
   // I2C_SUCCESS_FLG = false;
    CLR_I2C_MODE_BYTE;
}

/***************************************************************************************
 *                             SENSIRION SCD30 COMMANDS:                               *
 ***************************************************************************************/

void Sensirion_trigger_continuous_measurement(volatile uint16_t argument)
{
//    // Update peak time of last TX/RX/READ
//   // I2C_update_peak_time();
//    // If argument not within Sensirion's defined parameters
//    if (!((argument >= 700 && argument <= 1400) || argument == 0))
//    {
//        // Indicate to update as a set() function with incorrect parameters.
//   //     I2C_LED_FLG_BYTE = INVALID_SET_FUNCT;
//    }
//    else
//    {
        // Indicate to update as a set() function.
      //  RETURN_I2C_LED_FLG_BYTE = VALID_SET_FUNCT;
        // Populate buffer, sending 5 bytes

    // &&&&&&&&  sensor only, pressure is alway 0 so you can use altitude for adjustment dpb
    argument = 0;  // left code in case later we want to use milibars
        I2C_load_TX_DATA(Start_Continuous_Measurement_CMD, argument, 5);
//    }
}

void Sensirion_stop_continuous_measurement()
{
    // Update peak time of last TX/RX/READ
  //  I2C_update_peak_time();
    // Indicate to update as a set() function
  //  RETURN_I2C_LED_FLG_BYTE = VALID_SET_FUNCT;
    // Populate buffer, sending cmd only
    I2C_load_TX_DATA(Stop_Continuous_Measurement_CMD, 0, 2);
}

void Sensirion_set_measurement_interval(volatile uint16_t argument)
{
    // Update peak time of last TX/RX/READ
 //   I2C_update_peak_time();
    // If argument not within Sensirion's defined parameters
    if (!(argument >= 2 && argument <= 1800))
    {
        // Indicate to update as a set() function with incorrect parameters
     //   I2C_LED_FLG_BYTE = INVALID_SET_FUNCT;
    }
    else
    {
        // Indicate to update as a set() function
    //    RETURN_I2C_LED_FLG_BYTE = VALID_SET_FUNCT;
        // Populate buffer, sending 5 bytes
        I2C_load_TX_DATA(Set_Measurement_Interval_CMD, argument, 5);
    }
}

void Sensirion_get_measurement_interval_TX()
{
    // Update peak time of last TX/RX/READ
 //   I2C_update_peak_time();
    // Clear transmission state after TX.
  //  RETURN_I2C_LED_FLG_BYTE = NOT_IN_TRANSMISSION;
    // Populate buffer, sending cmd only
    I2C_load_TX_DATA(Set_Measurement_Interval_CMD, 0, 2);
}

void Sensirion_get_measurement_interval_RX(volatile  uint16_t *interval)
{
    // Update peak time of last TX/RX/READ
 //   I2C_update_peak_time();
    // Indicate to update as a get() function.
  //  RETURN_I2C_LED_FLG_BYTE = GET_FUNCT;
    I2C_RX_argument_ptr = interval;

    // SET_RX;
    // Should be 2 bytes interval plus CRC
    I2C_load_RX_DATA(3);  // get measurement interval
}

void Sensirion_get_data_ready_status_TX()
{
    // Update peak time of last TX/RX/READ
    I2C_update_peak_time();
    // Clear transmission state after TX.
 //   RETURN_I2C_LED_FLG_BYTE = NOT_IN_TRANSMISSION;
    // Populate buffer, sending cmd only
    I2C_load_TX_DATA(Get_Data_Ready_Status_CMD, 0, 2);
}

void Sensirion_get_data_ready_status_RX(volatile uint16_t *ready)
{
    // Update peak time of last TX/RX/READ
  //  I2C_update_peak_time();
    // Indicate to update as a get() function
 //   RETURN_I2C_LED_FLG_BYTE = GET_FUNCT;
    I2C_RX_argument_ptr = ready;
    // SET_RX;
    // Should be 2 bytes ready plus CRC
    I2C_load_RX_DATA(3);  // get data ready status
}

void Sensirion_read_measurement_TX()
{
    // Update peak time of last TX/RX/READ
  //  I2C_update_peak_time();
    // Clear transmission state after TX.
  //  RETURN_I2C_LED_FLG_BYTE = NOT_IN_TRANSMISSION;
    // Populate buffer, sending cmd only
    I2C_load_TX_DATA(Read_Measurement_CMD, 0, 2);
}

void Sensirion_read_measurement_RX(struct I2C_RX_Meas_t *meas)
{
    // Update peak time of last TX/RX/READ
  //  I2C_update_peak_time();
    // Clear transmission state. Will be updated in parsing.
 //   RETURN_I2C_LED_FLG_BYTE = NOT_IN_TRANSMISSION;
    SET_READ;
    measurement_ptr = meas;
    // Populate buffer, sending 15 bytes CO2/T/RH plus 3 bytes CRCs
    I2C_load_RX_DATA(18);  // read scd co2,t,rh
}


void Sensirion_toggle_automatic_self_calibration(volatile uint16_t argument)
{
    // Update peak time of last TX/RX/READ
  //  I2C_update_peak_time();
    if (!(argument == 0 || argument == 1))
    {
        // Indicate to update as a set() function with incorrect parameters
 //       I2C_LED_FLG_BYTE = INVALID_SET_FUNCT;
    }
    else
    {
        // Indicate to update as a set() function
 //       RETURN_I2C_LED_FLG_BYTE = VALID_SET_FUNCT;
        // Populate buffer, sending cmd, arg, CRC
        I2C_load_TX_DATA(Toggle_Automatic_Self_Calibration_CMD, argument, 5);
    }
}

void Sensirion_get_automatic_self_calibration_status_TX()
{
    // Update peak time of last TX/RX/READ
   // I2C_update_peak_time();
    // Clear transmission state after TX.
 //   RETURN_I2C_LED_FLG_BYTE = NOT_IN_TRANSMISSION;
    // Populate buffer, sending cmd only
    I2C_load_TX_DATA(Toggle_Automatic_Self_Calibration_CMD, 0, 2);
}

void Sensirion_get_automatic_self_calibration_status_RX(volatile uint16_t *ASC)
{
    // Update peak time of last TX/RX/READ
 //   I2C_update_peak_time();
    // Indicate to update as a get() function
  //  RETURN_I2C_LED_FLG_BYTE = GET_FUNCT;
    I2C_RX_argument_ptr = ASC;

    // SET_RX;
    // Should be 2 bytes ASC status plus CRC
    I2C_load_RX_DATA(3);  // check self calibration status
}


void Sensirion_set_forced_recalibration_value(volatile uint16_t argument)
{
    // Update peak time of last TX/RX/READ
  //  I2C_update_peak_time();

    if (!(argument >= 400 && argument <= 2000))
    {
        // Indicate to update as a set() function with incorrect parameters
 //       I2C_LED_FLG_BYTE = INVALID_SET_FUNCT;
    }
    else
    {
        // Indicate to update as a set() function
  //      RETURN_I2C_LED_FLG_BYTE = VALID_SET_FUNCT;
        // Populate buffer, sending cmd, arg, CRC
        I2C_load_TX_DATA(Set_Forced_Recalibration_Value_CMD, argument, 5);
    }
}

void Sensirion_get_forced_recalibration_value_TX()
{
    // Update peak time of last TX/RX/READ
  //  I2C_update_peak_time();
    // Clear transmission state after TX.
 //   RETURN_I2C_LED_FLG_BYTE = NOT_IN_TRANSMISSION;
    // Populate buffer, sending cmd only
    I2C_load_TX_DATA(Set_Forced_Recalibration_Value_CMD, 0, 2);
}

void Sensirion_get_forced_recalibration_value_RX(volatile uint16_t *FRC)
{
    // Update peak time of last TX/RX/READ
 //   I2C_update_peak_time();
    // Indicate to update as a get() function
 //   RETURN_I2C_LED_FLG_BYTE = GET_FUNCT;
    I2C_RX_argument_ptr = FRC;

    // SET_RX;
    // Should be 2 bytes FRC plus CRC
    I2C_load_RX_DATA(3);  // get force recal value
}

void Sensirion_set_temperature_offset(volatile uint16_t argument)
{
    // Update peak time of last TX/RX/READ
    I2C_update_peak_time();
    // Indicate to update as a set() function
  //  RETURN_I2C_LED_FLG_BYTE = VALID_SET_FUNCT;
    // Populate buffer, sending cmd, arg, CRC
    I2C_load_TX_DATA(Set_Temperature_Offset_CMD, argument, 5);
}

void Sensirion_get_temperature_offset_TX()
{
    // Update peak time of last TX/RX/READ
  //  I2C_update_peak_time();
    // Clear transmission state after TX.
  //  RETURN_I2C_LED_FLG_BYTE = NOT_IN_TRANSMISSION;
    // Populate buffer, sending cmd only
    I2C_load_TX_DATA(Set_Temperature_Offset_CMD, 0, 2);
}

void Sensirion_get_temperature_offset_RX(volatile uint16_t *offset)
{
    // Update peak time of last TX/RX/READ
 //   I2C_update_peak_time();
    // Indicate to update as a get() function
  //  RETURN_I2C_LED_FLG_BYTE = GET_FUNCT;

    // SET_RX;
    // Should be 2 bytes offset plus CRC
    I2C_load_RX_DATA(3);  // ger temperature offset value
}

void Sensirion_set_altitude_compensation(volatile uint16_t argument)
{
    // Update peak time of last TX/RX/READ
 //   I2C_update_peak_time();
    // Indicate to update as a set() function
 //   RETURN_I2C_LED_FLG_BYTE = VALID_SET_FUNCT;
    // Populate buffer, sending cmd, arg, CRC
    I2C_load_TX_DATA(Set_Altitude_Compensation_CMD, argument, 5);
}


void Sensirion_get_altitude_compensation_TX()
{
    // Update peak time of last TX/RX/READ
    I2C_update_peak_time();
    // Clear transmission state after TX.
 //   RETURN_I2C_LED_FLG_BYTE = NOT_IN_TRANSMISSION;
    // Populate buffer, sending cmd only
    I2C_load_TX_DATA(Set_Altitude_Compensation_CMD, 0, 2);
}

void Sensirion_get_altitude_compensation_RX(volatile uint16_t *compensation)
{
    // Update peak time of last TX/RX/READ
  //  I2C_update_peak_time();
    // Indicate to update as a get() function
 //   RETURN_I2C_LED_FLG_BYTE = GET_FUNCT;
    I2C_RX_argument_ptr = compensation;

    // SET_RX;
    // Should be 2 bytes compensation plus CRC
    I2C_load_RX_DATA(3);  // get altitude comp value
}

void Sensirion_read_firmware_version_TX()
{
    // Update peak time of last TX/RX/READ
    I2C_update_peak_time();
    // Clear transmission state after TX.
  //  RETURN_I2C_LED_FLG_BYTE = NOT_IN_TRANSMISSION;
    // Populate buffer, sending cmd only
    I2C_load_TX_DATA(Read_Firmware_Version_CMD, 0, 2);
}

void Sensirion_read_firmware_version_RX(volatile uint16_t *version)
{
    // Update peak time of last TX/RX/READ
  //  I2C_update_peak_time();
    // Indicate to update as a get() function
  //  RETURN_I2C_LED_FLG_BYTE = GET_FUNCT;
    I2C_RX_argument_ptr = version;

    // SET_RX;
    // Should be 2 bytes version plus CRC
    I2C_load_RX_DATA(3);  // get firmware version
}


void Sensirion_soft_reset()
{
    // Update peak time of last TX/RX/READ
   // I2C_update_peak_time();
    // Indicate to update as a set() function
  //  RETURN_I2C_LED_FLG_BYTE = VALID_SET_FUNCT;
    // Populate buffer, sending cmd only
    I2C_load_TX_DATA(Soft_Reset_CMD, 0, 2);
}

/********************************************************************************************
 *                                SENSIRION SCD30 I2C ISR: *
 ********************************************************************************************/

/**
 * I2C communication Interrupt Service Routine.
 */
#pragma vector = USCI_Bx_VECTOR
__interrupt void USCI_B1_ISR(void)
{
    switch (__even_in_range(UCBxIV, 0x1e))
    {
    case 0x00: // Vector 0: No interrupts
        break;
    case 0x02: // Vector 2: Arbitration Lost Interrupt FlaG (ALIFG)
        break;
    case 0x04: // Vector 4: No ACKnowledgement Interrupt FlaG (NACKIFG)
        I2C_STOP(); // Transmit STOP.
        // Transmission failure, don't update times/LEDs/Parse.
        Clr_I2C_Success_Flag ;
    //    I2C_SUCCESS_FLG = false;
        CLR_I2C_MODE_BYTE;
        break;
    case 0x06: // Vector 6: STarT condition Interrupt FlaG (STTIFG)
        break;
    case 0x08: // Vector 8: StoP condition Interrupt FlaG (STPIFG)
        break;
    case 0x0a: // Vector 10: Receive Interrupt FlaG 3 (RXIFG3)
        break;
    case 0x0c: // Vector 12: Transmit Interrupt FlaG 3 (TXIFG3)
        break;
    case 0x0e: // Vector 14: Receive Interrupt FlaG 2 (RXIFG2)
        break;
    case 0x10: // Vector 16: Transmit Interrupt FlaG 2 (TXIFG2)
        break;
    case 0x12: // Vector 18: Receive Interrupt FlaG 1 (RXIFG1)
        break;
    case 0x14: // Vector 20: Transmit Interrupt FlaG 1 (TXIFG1)
        break;
    case 0x16:                                  // Vector 22: Receive Interrupt FlaG 0 (RXIFG0)
        if (I2C_RX_INDEX < I2C_RX_CMD_SIZE - 1) // Ensure previous transmission completed
        {
            I2C_RX_DATA[I2C_RX_INDEX++] = UCBxRXBUF; // Set global RX data variable.
            SCD30_ALTITUDE = 0;  // just here for setting break pt dpb
        }
        else
        {
            // Transmission successful
            Set_I2C_Success_Flag ;
           // I2C_SUCCESS_FLG = true;
            I2C_STOP();                              // Transmit STOP after last byte read.
            I2C_RX_DATA[I2C_RX_INDEX++] = UCBxRXBUF; // Read last byte data.
            // Stop tracking I2C time
            STOP_I2C_STOPWATCH;
        }
        break;
    case 0x18:                              // Vector 24: Transmit Interrupt FlaG 0 (TXIFG0)
        if (I2C_TX_INDEX < I2C_TX_CMD_SIZE) // Ensure previous transmission completed
        {
            // Use this path with the Sensirion
            UCBxTXBUF = I2C_TX_DATA[I2C_TX_INDEX++]; // Load buffer
        }
        else
        {
            // Transmission successful
            Set_I2C_Success_Flag ;
          //  I2C_SUCCESS_FLG = true;
            I2C_STOP(); // Transmit STOP after next ACKnowledge.
            // Stop tracking I2C time
            STOP_I2C_STOPWATCH;
      //      I2C_LED_FLG_BYTE = RETURN_I2C_LED_FLG_BYTE;
        }
        break;
    case 0x1a:
        // Vector 26: Byte CouNTer Interrupt FlaG (BCNTIFG)
        break;
    case 0x1c:
        // Vector 28: Clock Low Time-Out Interrupt FlaG (CLTOIFG)
        break;
    case 0x1e:
        // Vector 30: 9th bit Interrupt FlaG (BIT9IFG)
        break;
    default:
        break;
    }
}
