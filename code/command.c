/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file command.h
 *
 * @brief Header for command.c, application specific commands to handle USART input strings
 *
 * - Compiler:          gcc-avr
 * - Project:           LiFePO4 Charge Controller
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author              Jasper Aorangi
 * @Date: July 2014
 *****************************************************************************/
#include "command.h"

uint8_t eloaded;
uint8_t wloaded;
uint8_t Wloaded;

struct ESP120AnalogData ESP120AnalogData;
struct ESP120FirmwareDebug ESP120FirmwareDebug;
struct ESP120FirmwareRevision ESP120FirmwareRevision;

/* Fixed text strings */
const char test_string_V[] PROGMEM = "Vx100\r\n";
const char test_string_A[] PROGMEM = "Ax100\r\n";
const char test_string_Ah[] PROGMEM = "Ah\r\n";
const char test_string_percent[] PROGMEM = "%\r\n";
// const char test_string_kWh[] PROGMEM = "kWh\r\n";

/* Parse null terminated string with expected format:
 * <single letter command><command value>
 * ie: t130400 to set time to 13:04:00
 *  Strip and check command
 *  Check that command value is a number. (strtol() )
 */
void handle_line(const char* line) {
    char *endptr = 0;

    //Check whether rest of line is a number and if so proceed to logic
    uint32_t argument_value = strtoul(line+1, &endptr, 10);
    if (*endptr == '\0') {
        command_from_serial(line[0], argument_value, &process);
//         command_from_serial(line[0], argument_value, &process);
    }
}

/* The Giant mess that is the commands from serial */
void command_from_serial(char commandname, uint32_t commandvalue, struct Process *process) {
    
    
    struct Inputs *inputs = &process->inputs;
    struct Outputs *outputs = &process->outputs;
    struct Settings *settings = &process->settings;
    
    
    uint8_t esp120reg;
    uint8_t PSUaddress;
    PSUaddress = 24;
    
    
    switch(commandname) {
        //Help!
        case 'h': //Disable logging
            send_string_p(PSTR("Help! Available commands.\r\n \
            The following returns the current state \r\n \
            \t c: Current\r\n \
            \t v: Voltage\r\n \
            \t A: Ah\r\n \
            \t s: State of charge\r\n \
            \t d: duty cycle \r\n \
            \t t: time \r\n \
            \t X: Stop all \r\n \
            Commands are case sensetive letters followed by a number (commandvalue) with no space\r\n \
            Default commandvalue is 0 if not given\r\n \
            \t S: Set charge state\r\n \
            \t C: Set charge current\r\n \
            \t V: Set charge voltage\r\n \
            \t F: Set float voltage\r\n \
            \t M: Set voltage max\r\n \
            \t N: Set current max\r\n \
            \t p: set program number\r\n \
            \t T: Set Time (s)\r\n \
            1 Enable/0 Disable:\r\n \
            \t L: Logging\r\n \
            \t P: Process control enable \r\n"));
            break;
                
            case 'c':  // Current
                send_string_p(PSTR("The current is "));
                    send_uint16(inputs->current);
                    send_string_p(test_string_A);
                break;
                    
            case 'v':  // Voltage
                send_string_p(PSTR("The voltage is "));
                send_uint16(inputs->voltage);
                send_string_p(test_string_V);
                break;
                
            case 'A':  // Amp hour
                send_string_p(PSTR("Ah is "));
                    send_uint16((outputs->Ah_count)/(1800));
                    send_string_p(test_string_Ah);
                break;
                    
            case 's':  // State of charge
                send_string_p(PSTR("State of charge is "));
                send_uint16(outputs->charge_progress);
                send_string_p(test_string_percent);
                break;
                
            case 'd':  // Duty cyle
//                 send_string_p(PSTR("Toggling D9"));
//                 PORTB ^= (1 << PORTB5);
                if ( commandvalue == 0 ){
                    send_string_p(PSTR("The duty is "));
                    send_uint16(get_pwm(1));
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting the duty to "));
                    send_uint16(commandvalue);
                    set_pwm(1,commandvalue);
                    send_newline();
                }
                break;
            
            case 't':  // Time
                send_string_p(PSTR("The time is: "));
                send_uint32_half(timestamp);
                send_newline();
                break;
                
            case 'X':  // Stop all
                send_string_p(PSTR("Ceasing to charge, Wright ANCHOR, STOP!! "));
                    outputs->charge_state = 4;
                break;
                    
            case 'S':  // Set charge state
                send_string_p(PSTR("Setting the charge state to "));
                outputs->charge_state = commandvalue;
                send_uint16(outputs->charge_state);
                send_newline();
                break;
                    
            case 'C':  // Set charge current
                send_string_p(PSTR("Setting the charge current to "));
                settings->current_charge = commandvalue;
                send_uint16(settings->current_charge);
                send_newline();
                break;
                    
            case 'V':  // Set charge Voltage
                send_string_p(PSTR("Setting the charge voltage to "));
                settings->voltage_charged = commandvalue;
                send_uint16(settings->voltage_charged);
                send_newline();
                break;
                    
            case 'F':  // Set float voltage
                send_string_p(PSTR("Setting the float voltage to "));
                settings->voltage_float = commandvalue;
                send_uint16(settings->voltage_float);
                send_newline();
                break;
                    
            case 'M':  // Max voltage
                send_string_p(PSTR("Setting the maximum voltage to "));
                settings->voltage_max = commandvalue;
                send_uint16(settings->voltage_max);
                send_newline();
                break;
                    
            case 'N':  // Max current
                send_string_p(PSTR("Setting the maximum current to "));
                settings->current_max = commandvalue;
                send_uint16(settings->current_max);
                send_newline();
                break;
                    
            case 'p':  // Set the program number
                send_string_p(PSTR("Setting the program number to "));
//                 outputs->charge_state = commandvalue;
//                 send_uint16(outputs->charge_state);
                send_newline();
                break;
                    
            case 'T':  // Set the time
                send_string_p(PSTR("Setting the time to "));
                timestamp = commandvalue;
                send_uint32_half(timestamp);
                send_newline();
                break;
            
        case 'L': //logging
            if (commandvalue != 1){
                disable_logging();
                send_string_p(PSTR("Logging disabled, enable with 'L1'.\r\n"));
            }
            else {
                enable_logging();
                send_string_p(PSTR("Logging enabled, disable with 'L'.\r\n"));
            }
            break;
            
        case 'P': // Process COntrol
            if (commandvalue != 1){
                process_control_disable();
                send_string_p(PSTR("Process control disabled, enable with 'P1'.\r\n"));
            }
            else {
                process_control_enable();
                send_string_p(PSTR("Process control enabled, disable with 'P'.\r\n"));
            }
            break;
        
        case '<': // Wait time
            if (commandvalue == 0){
                send_string_p(PSTR("The float time is "));
                send_uint32(outputs->float_timer);
                send_newline();
            }
            else {
                send_string_p(PSTR("Setting the float time to "));
                outputs->float_timer = commandvalue;
                send_uint32(outputs->float_timer);
                send_newline();
            }
            break;
        
        case '>': // Float time
            if (commandvalue == 0){
                send_string_p(PSTR("The rest time is "));
                send_uint32(outputs->rest_timer);
                send_newline();
            }
            else {
                send_string_p(PSTR("Setting the rest time to "));
                outputs->rest_timer = commandvalue;
                send_uint32(outputs->rest_timer);
                send_newline();
            }
            break;
//BEGIN Dummy Input
        
        //Voltage
        case 'j':
            send_string_p(PSTR("Forcing Inputs->Voltage to: "));
            inputs->voltage = commandvalue;
            send_uint16(inputs->voltage);
            send_newline();
            break;
            
        //Current
        case 'k':
            send_string_p(PSTR("Forcing Inputs->Current to: "));
            inputs->current = commandvalue;
            send_uint16(inputs->current);
            send_newline();
            break;                 
            
//BEGIN: I2C Code
            
//         ////////////////////////////////////////////////    
//         //I2C related and debugging
        case 'I':
            send_string_p(PSTR("Scanning I2C bus in write mode: \r\n"));
            i2c_safe_write_scan_bus(0x00, 0x7F);
            send_string_p(PSTR("Scanning I2C bus in read mode: \r\n"));
            i2c_safe_read_scan_bus(0x00, 0x7F);
            send_newline();
            send_string_p(PSTR("completed\r\n"));
            break;
        
        //PID Type
        case 'Q':
                if ( commandvalue == 0 ){
                    send_string_p(PSTR("The PID type is "));
                    send_uint8(PIDtype);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting the PID type to "));
                    PIDtype = commandvalue;
                    send_uint8(PIDtype);
                    send_newline();
                }
                break;        
        //PID Type
        case 'a':
                if ( commandvalue == 0 ){
                    send_string_p(PSTR("cPID max read is "));
                    send_uint16(cPIDmaxread);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting cPIDmaxread type to "));
                    cPIDmaxread = commandvalue;
                    send_uint16(cPIDmaxread);
                    send_newline();
                }
                break;
//             send_string_p(PSTR("Reading firt two bytes of analog data from 31: \r\n"));
// //             send_uint8(commandvalue);
// //             send_newline();
//             send_uint16(i2c_safe_read_sixteen(31,commandvalue));
//             send_newline();
// //             i2c_start(commandvalue);
// //             send_string("Ack, sending 0x03 \r\n");
// //             i2c_write(0x03);
// //             send_string("Ack, sending repeated start + read \r\n");
// //             i2c_rep_start((commandvalue << 1)| I2C_READ); //Address of the device in read mode
// //             send_string("Ack, recieving word: \r\n");
// //             i2c_safe_readNak();
// // //             send_uint8(i2c_safe_readNak()); // Get the byte, sending NACK
// //             send_string("Stopping...\r\n");
// //             i2c_stop();
            break; 
            
        case 'W':
            uint8_t blee;
            PSUaddress = 31;
            
            send_string("Multi part data:\n\r");
            i2c_safe_start((PSUaddress << 1)| I2C_WRITE); //Address of the device in write mode
            i2c_safe_write(commandvalue); //Set the pointer register if there is one. reg address of 0xFF indicates there is none
            i2c_safe_rep_start((PSUaddress << 1)| I2C_READ); //Address of the device in read mode
            
            for (blee=0;blee<=20;blee++)
            {
                send_uint8(blee);
                send_newline();
                send_uint8(i2c_safe_readAck());
                send_newline();
            }
            send_uint8(blee);
            send_uint8(i2c_safe_readNak());
            send_newline();
            i2c_safe_stop();
            break;    
            
        case 'E':
            uint8_t blah;
            
            for (blah=0;blah<=11;blah++)
            {
                send_uint8(blah);
                send_newline();
                send_uint16(i2c_safe_read_word(commandvalue, blah));
                send_newline();
            }
                /*
            send_string("0x01:\n\r");
            send_uint16(i2c_safe_read_sixteen(commandvalue, 0x03));
            send_string("boll\r\n");*/
            break;
            
        /* Beginning to construct functions for the ESP-120 */
        
        // Test mode, 0x04 (confirmed)
        case 'q':
//             uint8_t PSUaddress;
            // PSUaddress = 31;
//             uint8_t esp120TestMode;
//             esp120reg = 0x4;
            send_string_p(PSTR("Setting Testmode (0x04)...\r\n"));
//             // Start communication with PSU:
//             i2c_start((PSUaddress << 1)| I2C_WRITE);
//             // Set the pointer register:
//             i2c_write(esp120reg);
//             // Send Data: 0x04|0x04|lsb of sum 0x04
//             i2c_write(0x4);
//             i2c_write(0x4);
//             i2c_write(0x8);
//             i2c_stop();
//             // Acknowledge we have sent:
            if (!esp120_test_mode(PSUaddress));
                send_string_p(PSTR("Done.\r\n"));
            
            break;
            
        // Firmware debug, 0x05 (Confirmed)
        case 'w':
//             uint8_t PSUaddress;
//             uint8_t esp120reg;
//             uint8_t FirmwareData[8];
//             uint8_t FirmwareByteCount;
            // PSUaddress = 31;
//             esp120reg = 0x5;
            send_string_p(PSTR("Setting Firmware debug mode (0x05)...\r\n"));
//             // Start communication with PSU:
//             i2c_start((PSUaddress << 1)| I2C_WRITE);
//             // Set the pointer register:
//             i2c_write(esp120reg);
//             // Set read mode:
//             i2c_rep_start((PSUaddress << 1)| I2C_READ);
//             // Get 8 bytes of data:
//             for (FirmwareByteCount=0; FirmwareByteCount < 7; FirmwareByteCount++)
//             {
//                 FirmwareData[FirmwareByteCount] = i2c_readAck();
//             }
//             FirmwareData[7] = i2c_readNak();
//             i2c_stop();
//             // Acknowledge we have sent:
//             send_string_p(PSTR("Done. And the data is:\r\n"));
//             for (FirmwareByteCount=0; FirmwareByteCount <= 7; FirmwareByteCount++)
//             {
//                 send_uint8(FirmwareData[FirmwareByteCount]);
//                 send_newline();
//             }
//             
            if (!esp120_firmware_debug(PSUaddress, &ESP120FirmwareDebug)){
                send_uint16(ESP120FirmwareDebug.ControlStatusRegister);
                send_newline();
            }
            
            break;
            
        // Firmware Revision (0x6) (Confirmed. Example is Major: 1, Minor 7.)
        case 'e':
//             uint8_t PSUaddress;
//             uint8_t esp120reg;
//             uint8_t FirmwareRevisionData[2];
//             
//             // PSUaddress = 31;
//             esp120reg = 0x6;
//             
            send_string_p(PSTR("Getting Firmware revision numbers (0x06)...\r\n"));
//             // Start communication with PSU:
//             i2c_start((PSUaddress << 1)| I2C_WRITE);
//             // Set the pointer register:
//             i2c_write(esp120reg);
//             // Set read mode:
//             i2c_rep_start((PSUaddress << 1)| I2C_READ);
//             // Get 2 bytes of data:
//             FirmwareRevisionData[0] = i2c_readAck();
//             FirmwareRevisionData[1] = i2c_readNak();
//             i2c_stop();
//             // Acknowledge we have sent:
//             send_string_p(PSTR("Done. And the Firmware Revision is:\r\n"));
//             send_string_p(PSTR("Major number: "));
//             send_uint8(FirmwareRevisionData[0]);
//             send_newline();
//             send_string_p(PSTR("Minor number: "));
//             send_uint8(FirmwareRevisionData[1]);
//             send_newline();
//             esp120_firmware_revision(PSUaddress, &ESP120FirmwareRevision);
            if(!esp120_firmware_revision(PSUaddress, &ESP120FirmwareRevision)){
                send_string_p(PSTR("Major number: "));
                send_uint16(ESP120FirmwareRevision.MajorRevisionNumber);
                send_newline();
                send_string_p(PSTR("Minor number: "));
                send_uint16(ESP120FirmwareRevision.MinorRevisionNumber);
                send_newline();
            }
            
            break;
            
        // System Test Mode (0x7) and data format (0x8)
        case 'r':
//             uint8_t PSUaddress;
//             uint8_t esp120TestMode;
            
            uint8_t esp120testModeDataFormat;
            esp120testModeDataFormat = commandvalue;
            /* Data Formats
                0 Default, Display processed ADC Data
                1 Raw ADC Data
                2 All Zero’s (0)
                3 All One’s (1)
                4 Alternating Zero’s (0) and One’s (1) 
            */
            
            // PSUaddress = 31;
            esp120reg = 0x7;
            
            send_string_p(PSTR("Setting System Testmode (0x07), and data format (0x08) ...\r\n"));
//             // Start communication with PSU:
//             i2c_start((PSUaddress << 1)| I2C_WRITE);
//             // Set the pointer register:
//             i2c_write(esp120reg);
//             // Send Data: 0x7|0x7|lsb of sum 0x04
//             i2c_write(0x7);
//             i2c_write(0x7);
//             i2c_write(0x14);
//             i2c_stop();
//             // Acknowledge we have sent:
//             send_string_p(PSTR("Done. Setting System Testmode\r\n"));
//             
//             esp120reg = 0x8;
//             // Start communication with PSU:
//             i2c_start((PSUaddress << 1)| I2C_WRITE);
//             // Set the pointer register:
//             i2c_write(esp120reg);
//             // Send Data: 0x7|0x7|lsb of sum 0x04
// //             i2c_write(esp120testModeDataFormat);
// //             i2c_write(esp120testModeDataFormat);
// //             i2c_write(esp120testModeDataFormat+esp120testModeDataFormat);
//             i2c_write(3);
//             i2c_write(3);
//             i2c_write(6);
//             i2c_stop();
//             // Acknowledge we have sent:
            if(!esp120_system_test_mode(PSUaddress,4)){
                send_string_p(PSTR("Done. Setting data format.\r\n"));
            }
            
            break;
            
        // Set Staus Register (0x1), Confirmed (eg 32 for Fan hi, 8 for PSU diable)
        case 'Y':
            //             uint8_t PSUaddress;
            //             uint8_t esp120reg;
//             uint8_t esp120ControlRegisterData;
//             
//             // PSUaddress = 31;
//             esp120reg = 0x1;
//             esp120ControlRegisterData = commandvalue;
//             
            send_string_p(PSTR("Setting Control Register (0x1) to: "));
//             send_uint8(esp120ControlRegisterData);
//             send_newline();
//             // Start communication with PSU:
//             i2c_start((PSUaddress << 1)| I2C_WRITE);
//             // Set the pointer register:
//             i2c_write(esp120reg);
//             // Send Data: 0x7|0x7|lsb of sum 0x04
//             i2c_write(esp120ControlRegisterData);
//             i2c_write(esp120ControlRegisterData);
//             i2c_write(esp120ControlRegisterData + esp120ControlRegisterData);
//             i2c_stop();
            // Acknowledge we have sent:
            if (!esp120_set_status_register(PSUaddress,1,1)){
                send_string_p(PSTR("Done.\r\n"));
            }
            
            break;
            
        // Read all Analog Data (0x3)
        case 'u':
            //             uint8_t PSUaddress;
            //             uint8_t esp120reg;
//             uint8_t esp120AnalogData[18];
//             uint8_t esp120AnalogDataCount;
//             
//             // PSUaddress = 31;
//             esp120reg = 0x3;
//             
            send_string_p(PSTR("Reading analog data:\r\n"));
//             // Start communication with PSU:
//             i2c_start((PSUaddress << 1)| I2C_WRITE);
//             // Set the pointer register:
//             i2c_write(esp120reg);
//             // Set read mode:
//             i2c_rep_start((PSUaddress << 1)| I2C_READ);
//             // Get 17 bytes of data...
//             for (esp120AnalogDataCount=0; esp120AnalogDataCount < 17; esp120AnalogDataCount++)
//             {
//                 esp120AnalogData[esp120AnalogDataCount] = i2c_readAck();
//             }
//             esp120AnalogData[17] = i2c_readNak();
//             i2c_stop();
//             // Acknowledge we have sent:
//             send_string_p(PSTR("Done. And the data is:\r\n"));
//             for (esp120AnalogDataCount=0; esp120AnalogDataCount < 18; esp120AnalogDataCount++)
//             {
//                 send_uint8(esp120AnalogData[esp120AnalogDataCount]);
//                 send_newline();
//             }
            if (!esp120_analog_data(PSUaddress, &ESP120AnalogData)){
                send_string_p(PSTR("Current: "));
                send_uint16(ESP120AnalogData.Current);
                send_string_p(PSTR("mA\r\nLine Voltage: "));
                send_uint16(ESP120AnalogData.LineVoltage);
                send_string_p(PSTR("Vx100\r\nTemperature 1: "));
                send_uint16(ESP120AnalogData.Temperature1);
                send_newline();
            }
            break;
            
        //Read Current
        case 'x':
//             if (!commandvalue){
//                 esp120_analog_data(PSU1_ADDRESS, &ESP120AnalogData);
//                 send_uint16(ESP120AnalogData.Current);
//                 send_string(" ");
//                 esp120_analog_data(PSU2_ADDRESS, &ESP120AnalogData);
//                 send_uint16(ESP120AnalogData.Current);
//                 send_newline();
//             }
//             else {
                esp120_analog_data(PSUaddress, &ESP120AnalogData);
                send_uint16(ESP120AnalogData.Current);
                send_newline();
//             }
                
            break;
                
        // Go crazy on current reading.
        case 'O':
            //             uint8_t PSUaddress;
            //             uint8_t esp120reg;
            uint32_t esp120AnalogData2[3];
            uint8_t esp120AnalogDataCount2;
            uint32_t Current;
            uint32_t Current2;
            uint8_t UseCurrent2;
            uint16_t cRazy;
            
            UseCurrent2 = 0;
            
            // PSUaddress = 31;
            esp120reg = 0x3;
            for (cRazy=0; cRazy<=1000; cRazy++){
//             send_string_p(PSTR("Reading analog data:"));
            // Start communication with PSU:
            i2c_start((PSUaddress << 1)| I2C_WRITE);
            // Set the pointer register:
            i2c_write(esp120reg);
            // Set read mode:
            i2c_rep_start((PSUaddress << 1)| I2C_READ);
            // Get 17 bytes of data...
            esp120AnalogData2[0]=i2c_readAck();
            esp120AnalogData2[1]=i2c_readAck();
            esp120AnalogData2[2] = i2c_readNak();
            i2c_stop();
            
            Current = (esp120AnalogData2[1]<<8)+esp120AnalogData2[0];
            
            if (UseCurrent2){
                PSUaddress = 30;
                // Start communication with PSU:
                i2c_start((PSUaddress << 1)| I2C_WRITE);
                // Set the pointer register:
                i2c_write(esp120reg);
                // Set read mode:
                i2c_rep_start((PSUaddress << 1)| I2C_READ);
                // Get 17 bytes of data...
                esp120AnalogData2[0]=i2c_readAck();
                esp120AnalogData2[1]=i2c_readAck();
                esp120AnalogData2[2] = i2c_readNak();
                i2c_stop();
                
                Current2 = (esp120AnalogData2[1]<<8)+esp120AnalogData2[0];
                PSUaddress = 31;
                
                send_uint32(Current);
                send_string_p(PSTR("    "));
                send_uint32(Current2);
                send_newline();
            }
            else {
                // Acknowledge we have sent:
                send_uint32(Current);
                send_newline();
            }
                
            
            _delay_ms(100);
            }           
            
            break;
            
        //Toggle the PSU1_ON pin's
        case 'o':
            send_string_p(PSTR("Toggling power pin's\r\n"));
            PORTD ^= (1 << PSU1_ON);
            PORTD ^= (1 << PSU2_ON);
            
            break;
            
        //Read ADC from function
        case '[':
            send_string_p(PSTR("ADC reading: "));
            send_uint16(get_voltage());
            send_newline();
            
            break;
            
        //Go CRAZY on Voltage
        case ']':
//             uint16_t cRazy;
            for (cRazy=0; cRazy<=1000; cRazy++){
                send_uint16(get_voltage());
                send_newline();
                _delay_ms(10);
            }
            
            break;
            
        //Set PID voltage
        case '-':
            send_string_p(PSTR("Setting PID setpoint to: "));
            settings->voltage_charged = commandvalue;
            send_uint16(settings->voltage_charged);
            send_newline();
            
            break;
            
            
//         case 'R': //testing i2c_safe_read_sixteen
//             send_string_p(PSTR("I2C safe: Reading 16 bits from register 0x0, from address:"));
//             send_uint16(commandvalue);
//             send_newline();
//             send_uint16(i2c_safe_read_sixteen(commandvalue, 0x0) );
//             send_newline();
//             break;
//         case 'Z': //testing read_AT30TSE758
//             send_string_p(PSTR("read_AT30TSE758 , from address:"));
//             send_uint16(commandvalue);
//             send_newline();
//             send_uint16(read_AT30TSE758(commandvalue));
//             send_newline();
//             break;
        case 'Z': //testing read_MCP3221
            send_string_p(PSTR("read_MCP3221 from pressure sensor. code is:"));
               send_uint16(i2c_safe_read_sixteen(0x4D,0xFF)); //read address 0x4D, with no register (0xFFF)
//             send_uint16(read_MCP3221());
            send_newline();
            break;
        
//         case 'C': //Go Crazy!
//             send_string_p(PSTR("Going Crazy on level!!!"));
//             uint16_t poo;
//             uint16_t fart;
//             poo =10000;
//             fart = read_MCP3221(); //give us 8x origina reading
//             while (poo > 0){
//                 //Use averaging over 8 numbers
//                 fart = ((fart*7 + read_MCP3221() )>>3);
//                 send_uint16(fart);
//                 send_newline();
//                 _delay_ms(250);
//                 poo--;
//             }
//             break;
//         case 'B': //Go Crazy!
//             send_string_p(PSTR("Going Crazy on temperature!!!"));
//             uint16_t wee;
//             wee =10000;
//             while (wee > 0){
//                 send_uint16(temperature());
//                 send_newline();
//                 poo--;
//             }
//             break;
//         case 'V':
//             send_string_p(PSTR("reading i2c_safe_sixteen with adress"));
//             send_uint16(wloaded);
//             send_string_p(PSTR(", and register "));
//             send_uint16(commandvalue);
//             send_newline();
//             send_uint16(i2c_safe_read_sixteen(wloaded, commandvalue));
//             send_newline();
//             break;
//             // data to write to register
//         case 'w':
//             wloaded = commandvalue ;
//             send_string_p(PSTR("Loading data: "));
//             send_uint16(wloaded);
//             send_string_p(PSTR(" ready to write.\r\n"));
//             break;
//             // Toggle an output
//         case 'c':
//             send_string_p(PSTR("Toggling PortD pin"));
//             send_uint16(commandvalue);
// //             DDRD |= (1<<commandvalue); 
//             send_newline();
//             PORTD ^= (1<<commandvalue);
// //             pump_set(commandvalue);
//             break;

    }
}
