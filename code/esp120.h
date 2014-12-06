/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file ESP120.h
 *
 * @brief Header for ESP120.c, accessing I2C features of the ESP120 PSU (HP3kW)
 *
 * - Compiler:          gcc-avr
 * - Project:           AVR-lib
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author              Jasper Aorangi
 * @Date: July 2014
 *****************************************************************************/
#pragma once
#include <avr/io.h> 
#include "AVR-lib/i2c_safe.h"

struct ESP120AnalogData {
    uint32_t Current;          // Current through the device in mA
    uint32_t MaxCurrent;       // Maximum Current in mA
    uint32_t MinCurrent;       // Minimum Current in mA
    uint16_t LineVoltage;      // Line Voltage in V*100
    uint8_t  Temperature1;     // Temperature 1
    uint8_t  Temperature1Trip; // Temperature 1 Fan trip
    uint8_t  Temperature1Fail; // Temperature 1 Supply Fail
    uint8_t  Temperature2;     // Temperature 2
    uint8_t  Temperature2Trip; // Temperature 2 Fan trip
    uint8_t  Temperature2Fail; // Temperature 2 Supply Fail
    uint8_t  TemperatureUpdateStatus; // Temperature Update Status 1-new data 0-old data
};

struct ESP120FirmwareDebug {
    uint8_t ControlStatusRegister;
    uint8_t GeneralFlag1;
    uint8_t GeneralFlag2;
    uint8_t GeneralFlag3;
    uint8_t Port0;
    uint8_t Port1;
    uint8_t Port2;
};

struct ESP120FirmwareRevision {
    uint8_t MajorRevisionNumber;
    uint8_t MinorRevisionNumber;
};

/**
 * @brief Access the writable Control Status Bits: Fan Hi and Output OutDisable
 * 
 * @param Address: The I2C address of the PSU
 * @param FanHi: Boolean, 1 for fan Hi, 0 for normal
 * @param OutDisable: Boolean, 1 for output disable 0 (else) output enabled
 * @retval 1: Fail
 * @retval 0: Success
 **/
uint8_t esp120_set_status_register(uint8_t Address,uint8_t FanHi, uint8_t OutDisable);


/**
 * @brief Read the Control Status Register
 * @return The control status register
 * BIT 7     BIT 6   BIT 5  BIT 4         BIT 3        BIT 2   BIT 1   BIT 0
 * PSON_STAT BAD_CAL FAN_HI SELFTEST_FAIL ROUT_DISABLE OC_TRIP OV_TRIP OT_TRIP
 * @retval 0xFF: Fail (either way 8)
 **/
uint8_t esp120_get_status_register(uint8_t Address);

/**
 * @brief Read Analog Data
 * @param Address: The I2C address of the PSU
 * @param ESP120AnalogData: Struct to store the data in
 * @retval 1: Fail
 * @retval 0: Success
 **/
uint8_t esp120_analog_data(uint8_t Address, struct ESP120AnalogData *AnalogData);

/**
 * @brief Test mode
 * 
 * Test Mode will disables the Write Protection to the upper half of the EEPROM allowing the Calibration Test
 * program to write the CAL Table Data into the EEPROM. In this Mode both the Power and the Fail LED will be
 * ON simultaneously. The power supply must be in Standby position for the command to be effective. The
 * power supply will ignore this command if it is turned ON.
 * 
 * @param Address: The I2C address of the PSU
 * @retval 1: Fail
 * @retval 0: Success
 **/
uint8_t esp120_test_mode(uint8_t Address);

/**
 * @brief Firmware debug mode.
 * 
 * Firmware debug command returns the Status, Internal Flags and Port Pin Status of the microcontroller. This
 * command will be used to debug the Firmware only.
 * 
 * @param Address: The I2C address of the PSU
 * @param ESP120FirmwareDebug: Struct to store the data in
 * @retval 1: Fail
 * @retval 0: Success
 **/
uint8_t esp120_firmware_debug(uint8_t Address, struct ESP120FirmwareDebug *FirmwareDebugData);


/**
 * @brief Get the Firmware revision number
 * 
 * Firmware Revision Number will be hard coded into the Firmware itself. This command will return the revision
 * number of the Firmware.
 * 
 * @param Address: The I2C address of the PSU
 * @param ESP120FirmwareRevision: Struct to store the data in
 * @retval 1: Fail
 * @retval 0: Success
 **/
uint8_t esp120_firmware_revision(uint8_t Address, struct ESP120FirmwareRevision *FirmwareData);

/**
 * @brief System Test mode: Set test mode and data output format
 * 
 * @param Address: The I2C address of the PSU
 * @param Mode: Data type mode:
 * 0 Default, Display processed ADC Data
 * 1 Raw ADC Data
 * 2 All Zero’s (0)
 * 3 All One’s (1)
 * 4 Alternating Zero’s (0) and One’s (1)
 * @retval 1: Fail
 * @retval 0: Success
 **/
uint8_t esp120_system_test_mode(uint8_t Address, uint8_t Mode);

