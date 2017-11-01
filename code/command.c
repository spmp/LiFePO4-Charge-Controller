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

/* Fixed text strings */
const char test_string_V[]  PROGMEM = "Vx100\r\n";
const char test_string_A[]  PROGMEM = "Ax100\r\n";
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
    }
}

/* The Giant mess that is the commands from serial */
void command_from_serial(char commandname, uint32_t commandvalue, struct Process *process) {

    struct Inputs   *inputs   = &process->inputs;
    struct Outputs  *outputs  = &process->outputs;
    struct Settings *settings = &process->settings;
    
    uint8_t esp120reg;
    uint8_t PSUaddress;
    PSUaddress = 24;
    
    switch(commandname) {
        //Help!
        case 'h':
            send_string_p(PSTR("Help! Available commands.\r\n \
            The following returns the current state \r\n \
            \t C: Current\r\n \
            \t V: Voltage\r\n \
            \t B: Power\r\n \
            \t A: Ah\r\n \
            \t X: Stop all \r\n \
            Commands are case sensitive letters followed by a number with no space\r\n \
            No command value will return the current setting\r\n \
            \t Q: Program \r\n \
            \t t: time \r\n \
            \t m: Charge mode (1 stop, 2 CC, 9, Interim, 3 Bal, 4 CV, 5 CP, 6 Abs, 7 Retry)\r\n \
            \t q: CC current\r\n \
            \t a: CC Voltage\r\n \
            \t r: CC Voltage BMS\r\n \
            \t z: CC PID proportion\r\n \
            \t w: CV current\r\n \
            \t e: CV current done\r\n \
            \t s: CV Voltage\r\n \
            \t v: CV Voltage BMS\r\n \
            \t x: CV PID proportion\r\n \
            \t c: CV PID proportion BMS\r\n \
            \t i: Balancing current\r\n \
            \t l: Balancing Voltage\r\n \
            \t f: Balancing Voltage BMS\r\n \
            \t .: Balancing PID proportion\r\n \
            \t /: Balancing PID proportion BMS\r\n \
            \t N: Set current max\r\n \
            \t M: Set voltage max\r\n \
            \t p: duty cycle \r\n \
            \t {: voltage slope \r\n \
            \t }: voltage offset \r\n \
            \t [: current slope \r\n \
            \t ]: current offset \r\n \
            1 Enable/0 Disable:\r\n \
            \t L: Logging\r\n \
            \t P: Process control enable \r\n"));
            break;
                
            case 'C':  // Current
                send_string_p(PSTR("The current is "));
                send_uint16(inputs->current);
                send_string_p(test_string_A);
                break;
                    
            case 'V':  // Voltage
                send_string_p(PSTR("The voltage is "));
                send_uint16(inputs->voltage);
                send_string_p(test_string_V);
                break;
                
            case 'X':  // Stop all
                send_string_p(PSTR("Ceasing to charge, Wright ANCHOR, STOP!! "));
                    outputs->charge_mode = CHARGE_MODE_OFF;
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
                
            case 'p':  // Duty cyle
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
                
            case '[':  // Voltage slope
                if ( commandvalue == 0 ){
                    send_string_p(PSTR("The voltage slope (x1000) is "));
                    send_uint32(uint32_t(settings->analog_voltage_slope_code*1000));
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting the voltage slope (x1000) to "));
                    settings->analog_voltage_slope_code = float(commandvalue) / 1000;
                    send_uint32(uint32_t(settings->analog_voltage_slope_code*1000));
                    send_newline();
                }
                break;
                
            case ']':  // Voltage offset
                if ( commandvalue == 0 ){
                    send_string_p(PSTR("The voltage offset (x1000) is "));
                    send_uint32(uint32_t(settings->analog_voltage_offset_code*1000));
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting the voltage offset (x1000) to "));
                    settings->analog_voltage_offset_code = float(commandvalue) / 1000;
                    send_uint32(uint32_t(settings->analog_voltage_offset_code*1000));
                    send_newline();
                }
                break;  
            
            case '{':  // Current slope
                if ( commandvalue == 0 ){
                    send_string_p(PSTR("The current slope (x1000) is "));
                    send_uint32(uint32_t(settings->analog_current_slope_code*1000));
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting the current slope (x1000) to "));
                    settings->analog_current_slope_code = float(commandvalue) / 1000;
                    send_uint32(uint32_t(settings->analog_current_slope_code*1000));
                    send_newline();
                }
                break;
                
            case '}':  // Current offset
                if ( commandvalue == 0 ){
                    send_string_p(PSTR("The current offset (x1000) is "));
                    send_uint32(uint32_t(settings->analog_current_offset_code*1000));
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting the current offset (x1000) to "));
                    settings->analog_current_offset_code = float(commandvalue) / 1000;
                    send_uint32(uint32_t(settings->analog_current_offset_code*1000));
                    send_newline();
                }
                break;
                
                
            /** Show/Set parameters **/
            case 'Q':  // Program 
                if (commandvalue == 0){
                    send_string_p(PSTR("The Program  is "));
                    send_uint16(settings->program_number);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting Program  to "));
                    settings->program_number = commandvalue;
                    send_uint16(settings->program_number);
                    send_newline();
                }
                break;
            case 't':  // time 
                if (commandvalue == 0){
                    send_string_p(PSTR("The time  is "));
                    send_uint32_half(timestamp);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting time  to "));
                    timestamp = commandvalue;
                    send_uint32_half(timestamp);
                    send_newline();
                }
                break;
            case 'm':  // Charge mode 
                if (commandvalue == 0){
                    send_string_p(PSTR("The Charge mode  is "));
                    send_uint16(outputs->charge_mode);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting Charge mode  to "));
                    switch(commandvalue)
                    {
                      case 'CHARGE_MODE_CONSTANT_CURRENT':
                        settings->analog_voltage_slope_code = ANALOG_VOLTAGE_HIGH_A_SLOPE_CODE;
                        settings->analog_voltage_offset_code = ANALOG_VOLTAGE_HIGH_A_OFFSET_CODE;
                        break;
                      default :
                        settings->analog_voltage_slope_code = ANALOG_VOLTAGE_LOW_A_SLOPE_CODE;
                        settings->analog_voltage_offset_code = ANALOG_VOLTAGE_LOW_A_OFFSET_CODE;
                    }
                    outputs->charge_mode = commandvalue;
                    send_uint16(outputs->charge_mode);
                    send_newline();
                }
                break;
            case 'q':  // CC current
                if (commandvalue == 0){
                    send_string_p(PSTR("The CC current is "));
                    send_uint16(settings->current_cc);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting CC current to "));
                    settings->current_cc = commandvalue;
                    send_uint16(settings->current_cc);
                    send_newline();
                }
                break;
            case 'a':  // CC Voltage
                if (commandvalue == 0){
                    send_string_p(PSTR("The CC Voltage is "));
                    send_uint16(settings->voltage_cc);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting CC Voltage to "));
                    settings->voltage_cc = commandvalue;
                    send_uint16(settings->voltage_cc);
                    send_newline();
                }
                break;
            case 'r':  // CC Voltage BMS
                if (commandvalue == 0){
                    send_string_p(PSTR("The CC Voltage BMS is "));
                    send_uint16(settings->BMS_max_voltage_cv);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting CC Voltage BMS to "));
                    settings->BMS_max_voltage_cv = commandvalue;
                    send_uint16(settings->BMS_max_voltage_cv);
                    send_newline();
                }
                break;
            case 'z':  // CC PID proportion
                if (commandvalue == 0){
                    send_string_p(PSTR("The CC PID proportion is "));
                    send_uint16(settings->pid_proportion_cc);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting CC PID proportion to "));
                    settings->pid_proportion_cc = commandvalue;
                    send_uint16(settings->pid_proportion_cc);
                    send_newline();
                }
                break;
            // Balancing
            case 'i':  // Balancing current
                if (commandvalue == 0){
                    send_string_p(PSTR("The balancing current is "));
                    send_uint16(settings->current_balancing);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting balancing current to "));
                    settings->current_balancing = commandvalue;
                    send_uint16(settings->current_balancing);
                    send_newline();
                }
                break;
            case 'l':  // Balancing Voltage
                if (commandvalue == 0){
                    send_string_p(PSTR("The balancing Voltage is "));
                    send_uint16(settings->voltage_balancing);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting balancing Voltage to "));
                    settings->voltage_balancing = commandvalue;
                    send_uint16(settings->voltage_balancing);
                    send_newline();
                }
                break;
            case 'f':  // Balancing Voltage BMS
                if (commandvalue == 0){
                    send_string_p(PSTR("The balancing Voltage BMS is "));
                    send_uint16(settings->BMS_max_voltage_balancing);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting balancing Voltage BMS to "));
                    settings->BMS_max_voltage_balancing = commandvalue;
                    send_uint16(settings->BMS_max_voltage_balancing);
                    send_newline();
                }
                break;
            case '.':  // Balancing PID proportion
                if (commandvalue == 0){
                    send_string_p(PSTR("The balancing PID proportion is "));
                    send_uint16(settings->pid_proportion_balancing);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting valancing PID proportion to "));
                    settings->pid_proportion_balancing = commandvalue;
                    send_uint16(settings->pid_proportion_balancing);
                    send_newline();
                }
                break;
            case '/':  // Balancing PID proportion
                if (commandvalue == 0){
                    send_string_p(PSTR("The balancing PID BMS proportion is "));
                    send_uint16(settings->pid_proportion_balancing_bms);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting valancing PID BMS proportion to "));
                    settings->pid_proportion_balancing_bms = commandvalue;
                    send_uint16(settings->pid_proportion_balancing_bms);
                    send_newline();
                }
                break;
            case 'w':  // CV current
                if (commandvalue == 0){
                    send_string_p(PSTR("The CV current is "));
                    send_uint16(settings->current_cv);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting CV current to "));
                    settings->current_cv = commandvalue;
                    send_uint16(settings->current_cv);
                    send_newline();
                }
                break;
            case 'e':  // CV current done
                if (commandvalue == 0){
                    send_string_p(PSTR("The CV current done is "));
                    send_uint16(settings->current_cv_done);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting CV current done to "));
                    settings->current_cv_done = commandvalue;
                    send_uint16(settings->current_cv_done);
                    send_newline();
                }
                break;
            case 's':  // CV Voltage
                if (commandvalue == 0){
                    send_string_p(PSTR("The CV Voltage is "));
                    send_uint16(settings->voltage_cv);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting CV Voltage to "));
                    settings->voltage_cv = commandvalue;
                    send_uint16(settings->voltage_cv);
                    send_newline();
                }
                break;
            case 'v':  // CV Voltage
                if (commandvalue == 0){
                    send_string_p(PSTR("The CV Voltage BMS is "));
                    send_uint16(settings->BMS_max_voltage_cv);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting CV Voltage BMS to "));
                    settings->BMS_max_voltage_cv = commandvalue;
                    send_uint16(settings->BMS_max_voltage_cv);
                    send_newline();
                }
                break;
            case 'x':  // CV PID proportion
                if (commandvalue == 0){
                    send_string_p(PSTR("The CV PID proportion is "));
                    send_uint16(settings->pid_proportion_cv);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting CV PID proportion to "));
                    settings->pid_proportion_cv = commandvalue;
                    send_uint16(settings->pid_proportion_cv);
                    send_newline();
                }
                break;
            case 'c':  // CV PID proportion BMS
                if (commandvalue == 0){
                    send_string_p(PSTR("The CV BMS PID proportion is "));
                    send_uint16(settings->pid_proportion_cv_bms);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting CV BMS PID proportion to "));
                    settings->pid_proportion_cv_bms = commandvalue;
                    send_uint16(settings->pid_proportion_cv_bms);
                    send_newline();
                }
                break;
            case 'N':  // Set current max
                if (commandvalue == 0){
                    send_string_p(PSTR("The Set current max is "));
                    send_uint16(settings->current_max);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting Set current max to "));
                    settings->current_max = commandvalue;
                    send_uint16(settings->current_max);
                    send_newline();
                }
                break;
            case 'M':  // Set voltage max
                if (commandvalue == 0){
                    send_string_p(PSTR("The Set voltage max is "));
                    send_uint16(settings->voltage_max);
                    send_newline();
                }
                else {
                    send_string_p(PSTR("Setting Set voltage max to "));
                    settings->voltage_max = commandvalue;
                    send_uint16(settings->voltage_max);
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
            
        //Toggle the PSU1_ON pin's
        case 'o':
            send_string_p(PSTR("Toggling power pin's\r\n"));
            PORTD ^= (1 << PSU1_ON);
            PORTD ^= (1 << PSU2_ON);
            
            break;

        //Set PID voltage
        case '-':
            send_string_p(PSTR("Setting PID setpoint to: "));
            settings->voltage_cc = commandvalue;
            send_uint16(settings->voltage_cc);
            send_newline();
            
            break;

    }
}
