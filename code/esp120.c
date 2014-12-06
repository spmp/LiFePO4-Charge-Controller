/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file ESP120.c
 *
 * @brief Functions for accessing I2C features of the ESP120 PSU (HP3kW)
 *
 * - Compiler:          gcc-avr
 * - Project:           AVR-lib
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author              Jasper Aorangi
 * @Date: July 2014
 *****************************************************************************/
#include "esp120.h"
 

/**
 * @brief Access the writable Control Status Bits: Fan Hi and Output OutDisable
 * 
 * @param Address: The I2C address of the PSU
 * @param FanHi: Boolean, 1 for fan Hi, 0 for normal
 * @param OutDisable: Boolean, 1 for output disable 0 (else) output enabled
 * @retval 1: Fail
 * @retval 0: Success
 **/
uint8_t esp120_set_status_register(uint8_t Address,uint8_t FanHi, uint8_t OutDisable){
    // Set Staus Register (0x1), Confirmed (eg 32 for Fan hi, 8 for PSU diable)
    uint8_t esp120ControlRegisterData;
    esp120ControlRegisterData = 0x00 | (FanHi << 5) | (OutDisable <<3);
    // Start communication with PSU:
    i2c_safe_start((Address << 1)| I2C_WRITE);
    // Set the pointer register:
    i2c_safe_write(0x1);
    // Send Data: Date|Data|lsb of sum previous Data
    i2c_safe_write(esp120ControlRegisterData);
    i2c_safe_write(esp120ControlRegisterData);
    i2c_safe_write(esp120ControlRegisterData + esp120ControlRegisterData);
    i2c_safe_stop();
    return 0;
}

/**
 * @brief Read the Control Status Register
 * @return The control status register
 * BIT 7     BIT 6   BIT 5  BIT 4         BIT 3        BIT 2   BIT 1   BIT 0
 * PSON_STAT BAD_CAL FAN_HI SELFTEST_FAIL ROUT_DISABLE OC_TRIP OV_TRIP OT_TRIP
 * @retval 0xFF: Fail (either way 8)
 **/
uint8_t esp120_get_status_register(uint8_t Address){
    return 0;
}

/**
 * @brief Read Analog Data
 * @param Address: The I2C address of the PSU
 * @param ESP120AnalogData: Struct to store the data in
 * @retval 1: Fail
 * @retval 0: Success
 **/
uint8_t esp120_analog_data(uint8_t Address, struct ESP120AnalogData *AnalogData){
    
    uint8_t esp120AnalogData[18];
    uint8_t esp120AnalogDataCount;
    // Start communication with PSU:
    i2c_safe_start((Address << 1)| I2C_WRITE);
    // Set the pointer register:
    i2c_safe_write(0x3);
    // Set read mode:
    i2c_safe_rep_start((Address << 1)| I2C_READ);
    // Get 17 bytes of data...
        for (esp120AnalogDataCount=0; esp120AnalogDataCount < 17; esp120AnalogDataCount++)
    {
        esp120AnalogData[esp120AnalogDataCount] = i2c_safe_readAck();
    }
    esp120AnalogData[17] = i2c_safe_readNak();
    i2c_safe_stop();
    
    AnalogData->Current = (esp120AnalogData[0]+(esp120AnalogData[1]<<8)+(esp120AnalogData[2]<<16));
    AnalogData->MaxCurrent =  (esp120AnalogData[3]+(esp120AnalogData[4]<<8)+(esp120AnalogData[5]<<16));
    AnalogData->MinCurrent =  (esp120AnalogData[6]+(esp120AnalogData[7]<<8)+(esp120AnalogData[8]<<16));
    AnalogData->LineVoltage =  (esp120AnalogData[9]+(esp120AnalogData[10]<<8));
    AnalogData->Temperature1 = esp120AnalogData[11];
    AnalogData->Temperature1Trip = esp120AnalogData[12];
    AnalogData->Temperature1Fail = esp120AnalogData[13];
    AnalogData->Temperature2 = esp120AnalogData[14];
    AnalogData->Temperature2Trip = esp120AnalogData[15];
    AnalogData->Temperature2Fail = esp120AnalogData[16];
    AnalogData->TemperatureUpdateStatus = esp120AnalogData[17];
    
    return 0;
}

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
uint8_t esp120_test_mode(uint8_t Address){
    // Start communication with PSU:
    i2c_safe_start((Address << 1)| I2C_WRITE);
    // Set the pointer register:
    i2c_safe_write(0x4);
    // Send Data: 0x04|0x04|lsb of sum 0x04+0x04
    i2c_safe_write(0x4);
    i2c_safe_write(0x4);
    i2c_safe_write(0x8);
    i2c_safe_stop();
    
    return 0;
}

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
uint8_t esp120_firmware_debug(uint8_t Address, struct ESP120FirmwareDebug *FirmwareDebugData){
    // Firmware debug, 0x05 (Confirmed)
    uint8_t FirmwareData[8];
    uint8_t FirmwareByteCount;
    // Start communication with PSU:
    i2c_safe_start((Address << 1)| I2C_WRITE);
    // Set the pointer register:
    i2c_safe_write(0x5);
    // Set read mode:
    i2c_safe_rep_start((Address << 1)| I2C_READ);
    // Get 7 bytes of data:
    FirmwareDebugData->ControlStatusRegister = i2c_safe_readAck();
    FirmwareDebugData->GeneralFlag1 = i2c_safe_readAck();
    FirmwareDebugData->GeneralFlag2 = i2c_safe_readAck();
    FirmwareDebugData->GeneralFlag3 =i2c_safe_readAck();
    FirmwareDebugData->Port0 = i2c_safe_readAck();
    FirmwareDebugData->Port1 = i2c_safe_readAck();
    FirmwareDebugData->Port2 = i2c_safe_readNak();
    i2c_safe_stop();
    
    return 0;
}


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
uint8_t esp120_firmware_revision(uint8_t Address, struct ESP120FirmwareRevision *FirmwareData){
    // Firmware Revision (0x6) (Confirmed. Example is Major: 1, Minor 7.)
    // Start communication with PSU:
    i2c_safe_start((Address << 1)| I2C_WRITE);
    // Set the pointer register:
    i2c_safe_write(0x6);
    // Set read mode:
    i2c_safe_rep_start((Address << 1)| I2C_READ);
    // Get 2 bytes of data:
    FirmwareData->MajorRevisionNumber = i2c_safe_readAck();
    FirmwareData->MinorRevisionNumber = i2c_safe_readNak();
    i2c_safe_stop();
        
    return 0;
}

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
uint8_t esp120_system_test_mode(uint8_t Address, uint8_t Mode){
    // System Test Mode (0x7) and data format (0x8)
    // Start communication with PSU:
    i2c_safe_start((Address << 1)| I2C_WRITE);
    // Set the pointer register:
    i2c_safe_write(0x7);
    // Send Data: 0x7|0x7|lsb of sum 0x07...
    i2c_safe_write(0x7);
    i2c_safe_write(0x7);
    i2c_safe_write(0x14);
    i2c_safe_stop();
    //Set Data format:
    // Start communication with PSU:
    i2c_safe_start((Address << 1)| I2C_WRITE);
    // Set the pointer register:
    i2c_safe_write(0x8);
    // Send Data: 0x7|0x7|lsb of sum 0x04
    i2c_safe_write(Mode);
    i2c_safe_write(Mode);
    i2c_safe_write(Mode+Mode);
    i2c_safe_stop();
    
    return 0;
}