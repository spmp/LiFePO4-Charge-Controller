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
#include "process-control.h"

uint8_t eloaded;
uint8_t wloaded;
uint8_t Wloaded;

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
                send_string_p(PSTR("The duty is "));
                    send_uint16(outputs->pwm_duty);
                    send_newline();
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
//                 disable_logging();
                send_string_p(PSTR("Process control disabled, enable with 'P1'.\r\n"));
            }
            else {
//                 enable_logging();
                send_string_p(PSTR("Process control enabled, disable with 'P'.\r\n"));
            }
            break;
//         ////////////////////////////////////////////////    
//         //I2C related and debugging
//         case 'I':
//             send_string_p(PSTR("Scanning I2C bus in write mode: \r\n"));
//             i2c_safe_write_scan_bus(0x00, 0x7F);
//             send_string_p(PSTR("Scanning I2C bus in read mode: \r\n"));
//             i2c_safe_read_scan_bus(0x00, 0x7F);
//             send_newline();
//             send_string_p(PSTR("completed\r\n"));
//             break;
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
//         case 'X': //testing read_MCP3221
//             send_string_p(PSTR("read_MCP3221 from pressure sensor. code is:"));
//             send_uint16(read_MCP3221());
//             send_newline();
//             break;
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
