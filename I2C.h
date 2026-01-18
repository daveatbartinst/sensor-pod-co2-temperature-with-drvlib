/**
 * @file I2C.h
 * @author Nathaniel Suddarth
 * @brief Controls all I2C communication, setting modification, and data parsing related to the Sensirion SCD30 CO2 sensor.
 * @version 0.1
 * @date 2025-06-12
 *
 *
 */

#ifndef I2C_H_
#define I2C_H_




//#include "driverlib.h"

//#include "hw_config.h"
//#include <math.h>
//
#include <stdbool.h>
#include <stdint.h>
//#include <stdio.h>
//#include <string.h>


/**
 * @brief Contains CO2, Temp (T), Relative Humidity (RH) values.
 * These values are the uint converted data values of hex-encoded float values
 * (returned by Sensirion).
 * CO2 in ppm, T in degrees C, RH in percent.
 *
 */
struct I2C_RX_Meas_t
{
    uint16_t CO2; // 0 - 10,000 ppm
    int8_t T;    // -40 - 125 degrees Celsius
    uint8_t RH;   // 0 - 100 percent relative humidity
};



// Timer Variables:

extern volatile uint16_t I2C_BREAK_TIMER;
extern volatile bool I2C_END_BREAK_FLG;

// Peak I2C Times:
extern volatile uint16_t I2C_PEAK_TX_TIME;
extern volatile uint16_t I2C_PEAK_RX_TIME;
extern volatile uint16_t I2C_PEAK_READ_TIME;
extern volatile uint16_t I2C_TIME;
extern volatile bool TRACK_I2C_TIME;
extern volatile uint8_t I2C_MODE_BYTE;
extern uint8_t SCD_STATUS ;
// Maximum and Minimum Temp, CO2, and Relative Humidity:
extern volatile uint16_t max_CO2, min_CO2;
extern volatile uint8_t max_T, min_T;
extern volatile uint8_t max_RH, min_RH;

// Counts the number of bad reads:
extern volatile uint16_t I2C_REDUX_COUNT;


extern uint8_t SCD_STATUS ;

/********************************************************************************************
 *                          SENSIRION SCD30 I2C FUNCTIONS:                                  *
 ********************************************************************************************/


/**
 * @brief Initialize I2C Master with 7-Bit Address. Initializes all necessary
 * GPIO and relevant functions.
 * Must run I2C_master_TX() or I2C_master_RX() to finish initialization.
 *
 * @param baud_rate Baud rate of the master device.
 * @param slave_address Expected address of the slave device.
 */
void UCB1_I2C_init_master(uint16_t baud_rate, uint16_t slave_address);

/**
 * @brief Initialize I2C slave with a 7-Bit Address.
 * Slave is automatically set as RX or TX by master R/~W Bit.
 * Used in debugging I2C communication by designating one dev.
 * board as a master and the other as a slave.
 *
 * @param slave_address Chosen address for the slave device.
 */
void I2C_init_slave(uint16_t slave_address);

/**
 * @brief Loads the average of the reading values of each type
 * into the giver pointers.
 * Ensure that there are at minimum I2C_Readings_Buffer_Size
 * readings before calling this function.
 *
 * @param CO2_avg Pointer through which the CO2 average will be stored.
 * @param T_avg Pointer throughin which the CO2 average will be stored.
 * @param RH_avg Pointer through which the CO2 average will be stored.
 */
void I2C_avg_readings(uint32_t *CO2_avg, uint16_t *T_avg, uint16_t *RH_avg);

/**
 * @brief Parses the RX buffer into a READ or traditional response. Extracts data for handling.
 */
void I2C_parse_RX_DATA();

/**
 * @brief Updates the LEDs depending on the previously set state.
 */
//void I2C_update_LED();

/**
 * @brief Clears the minimum and maximum data for fresh readings.
 */
void clear_min_max();

/**
 * @brief Resets communication in case of I2C timeout state.
 *
 */
void handle_I2C_TIMEOUT();


/*****************************************************************************
 *                          Sensirion Commands:                              *
 *****************************************************************************/

    /**
 * @brief Starts continuous measurement of the SCD30 to measure CO2 concentration,
 * humidity and temperature. Setting the ambient pressure will overwrite previous
 * settings of altitude compensation.
 *
 * @param argument Ambient pressure in mbar from 700 - 1400. Set to 0 to deactivate ambient pressure compensation.
 * (default ambient pressure = 1013.25 mBar).
 */
void Sensirion_trigger_continuous_measurement(volatile uint16_t argument);

/**
 * @brief Stops the continuous measurement of the SCD30.
 *
 */
void Sensirion_stop_continuous_measurement();

/**
 * @brief Sets the measurement interval used by the SCD30 sensor to measure in continuous
 * measurement mode. Initial value is 2 s.
 *
 * @param argument Interval in seconds from 2 - 1800.
 */
void Sensirion_set_measurement_interval(volatile uint16_t argument);

/**
 * @brief Sends command to the SCD30. The paired RX command must be run to receive the measurement interval.
 */
void Sensirion_get_measurement_interval_TX();

/**
 * @brief Receives the measurement interval.
 *
 * @param interval A pointer through which the received interval is stored.
 */
void Sensirion_get_measurement_interval_RX(volatile uint16_t *interval);

/**
 * @brief Sends command to the SCD30. The paired RX command must be run to receive the data ready status.
 */
void Sensirion_get_data_ready_status_TX();

/**
 * @brief Determines if a measurement can be read from the sensor's buffer.
 *
 * @param ready 1 if ready, 0 if not ready.
 */
void Sensirion_get_data_ready_status_RX(volatile uint16_t *ready);

/**
 * @brief Sends command to the SCD30. The paired RX command must be run to receive the measurement.
 */
void Sensirion_read_measurement_TX();

/**
 * @brief Read new measurement data once ready.
 *
 * @param meas RX_Meas type struct of the reading upon successful
 * transmission. Unsuccessful transmission returns a struct
 * of values initialized to 0.
 */
void Sensirion_read_measurement_RX(struct I2C_RX_Meas_t *meas);

/**
 * @brief Activates or deactivates the automatic self-calibration of the Sensirion
 * sensor.
 * See the Sensirion SCD30 Interface Description datasheet for further information.
 *
 * @param argument 0 deactivates continuous ASC, 1 activates continuous ASC.
 */
void Sensirion_toggle_automatic_self_calibration(volatile uint16_t argument);

/**
 * @brief Sends command to the SCD30. The paired RX command must be run to receive the ASC status.
 *
 */
void Sensirion_get_automatic_self_calibration_status_TX();

/**
 * @brief Returns the ASC status without modifying the value.
 *
 * @param ASC If transmission successful, ASC status returned. If unsuccessful, 0 returned.
 */
void Sensirion_get_automatic_self_calibration_status_RX(volatile uint16_t *ASC);

/**
 * @brief Forced recalibration (FRC) compensates for sensor drifts by matching sensor reading
 * to a reference value of the CO2 concentration in close proximity to the SCD30.
 * See the Sensirion SCD30 Interface Description datasheet for further information.
 *
 * @param argument Reference value to match, 400 - 2000 ppm.
 */
void Sensirion_set_forced_recalibration_value(volatile uint16_t argument);

/**
 * @brief Sends command to the SCD30. The paired RX command must be run to receive the FRC value.
 */
void Sensirion_get_forced_recalibration_value_TX();

/**
 * @brief Returns the FRC value without modifying the value.
 *
 * @param FRC If transmission successful, FRC value returned. If unsuccessful, 0 returned.
 */
void Sensirion_get_forced_recalibration_value_RX(volatile uint16_t *FRC);

/**
 * @brief Temperature offset compensates for heat produced by the SCD30 and other electrical components.
 * Used to restore accuracy to temperature and relative humidity readings.
 *
 * @param argument Offset value = (argument * 0.01) degrees C.
 */
void Sensirion_set_temperature_offset(volatile uint16_t argument);

/**
 * @brief Sends command to the SCD30. The paired RX command must be run to receive the offset value.
 *
 */
void Sensirion_get_temperature_offset_TX();

/**
 * @brief Returns the temperature offset value without modifying the value.
 *
 *
 * @param offset If transmission successful, temperature offset value returned. If unsuccessful, 0 returned.
 */
void Sensirion_get_temperature_offset_RX(volatile uint16_t *offset);

/**
 * @brief Compensate CO2 measurement deviations due to altitude. Setting altitude is disregarded when an
 * ambient pressure is given to the sensor.
 *
 * @param argument Height above sea level in meters.
 */
void Sensirion_set_altitude_compensation(volatile uint16_t argument);

/**
 * @brief Sends command to the SCD30. The paired RX command must be run to receive the compensation value.
 */
void Sensirion_get_altitude_compensation_TX();

/**
 * @brief Returns the altitude compensation value without modifying the value.
 *
 * @param compensation Altitude compensation value if transmission successful. 0 if unsuccessful.
 */
void Sensirion_get_altitude_compensation_RX(volatile uint16_t *compensation);

/**
 * @brief Sends command to the SCD30. The paired RX command must be run to receive the firmware version.
 *
 */
void Sensirion_read_firmware_version_TX();

/**
 * @brief Returns the firmware version without modifying the value.
 *
 * @param version Firmware version if transmission successful. 0 if unsuccessful.
 */
void Sensirion_read_firmware_version_RX(volatile uint16_t *version);

/**
 * @brief Forces the sensor into the same state as after powering up without the
 * need for removing the power-supply. The sensor is able to receive the command
 * at any time, regardless of its internal state.
 */
void Sensirion_soft_reset();

#endif /* I2C_H_ */
