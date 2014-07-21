#include "command.h"
# include <avr/pgmspace.h>
#include <util/delay.h>
#include "temperature.h"
#include "i2c_safe.h"
#include "log.h"
#include "clock.h"
#include "level.h"
#include "state-machine.h"
#include "hardware.h"


uint8_t eloaded;
uint8_t wloaded;
uint8_t Wloaded;

/* Fixed text strings */
const char test_string_mm[] PROGMEM = "mm\r\n";
const char test_string_L[] PROGMEM = "L\r\n";
const char test_string_s[] PROGMEM = "s\r\n";
const char test_string_degc[] PROGMEM = "°Cx100\r\n";
const char test_string_kWh[] PROGMEM = "kWh\r\n";

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
        command_from_serial(line[0], argument_value, &program[state_machine_config_program]);
//         command_from_serial(line[0], argument_value, &program);
    }
}

/* The Giant mess that is the commands from serial */
void command_from_serial(char commandname, uint32_t commandvalue, struct Program *program) {
    
    
    struct Inputs *inputs = &program->inputs;
    struct Outputs *outputs = &program->outputs;
    struct Settings *settings = &program->settings;
    
    switch(commandname) {
        //Help!
        case 'h': //Disable logging
            send_string_p(PSTR("Help! Available commands.\r\n \
            Commands are case sensetive letters followed by a number (commandvalue) with no space\r\n \
            Default commandvalue is 0 if not given\r\n \
            \t l: Level (mm)\r\n \
            \t v: Volume (L)\r\n \
            1 Enable/0 Disable:\r\n \
            \t L: Logging\r\n \
            \t S: State machine\r\n \
            \t F: Fill\r\n \
            \t H: Heat\r\n \
            \t P: Pump\r\n \
            0 get value/Other Set value:\r\n \
            \t t: Time (s)\r\n \
            \t T: Temperature (°C)\r\n \
            \t f: Fill now (or to level)\r\n \
            \t b: Boost now (or to temp)\r\n \
            \t Y: TimeToHot1\r\n \
            \t y: temp TTH1\r\n \
            \t U: TTH2\r\n \
            \t u: temp TTH2\r\n \
            \t M: Max level\r\n \
            \t m: Min level\r\n \
            \t J: Heater min level\r\n \
            \t N: Fill level\r\n \
            //\t n: Fill level liters\r\n \
            \t G: Max temp\r\n \
            \t g: Min temp\r\n \
            \t s: Midsun\r\n \
            \t d: DHP (kWh)\r\n \
            \t z: Zero the level\r\n \
            \t O: Progam to configure\r\n \
            \t o: Running program\r\n \
            \t C: Config dump\r\n"));
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
            
        case 'S': //State machine
            if (commandvalue != 1){
                disable_state_machine();
                send_string_p(PSTR("State machine disabled, enable with 'S1'.\r\n"));
            }
            else {
                enable_state_machine();
                send_string_p(PSTR("State machine enabled, disable with 'S'.\r\n"));
            }
            break;
            
        case 'F': //Filler
            if (commandvalue != 1){
                settings->fill_enable = 0;
                send_string_p(PSTR("Filler disabled, enable with 'F1'.\r\n"));
            }
            else {
                settings->fill_enable = 1;
                send_string_p(PSTR("Filler enabled, disable with 'F0'.\r\n"));
            }
            break;
            
        case 'H': //Heater
            if (commandvalue != 1){
                settings->heater_enable = 0;
                send_string_p(PSTR("Heater disabled, enable with 'H1'.\r\n"));
            }
            else {
                settings->heater_enable = 1;
                send_string_p(PSTR("Heater enabled, disable with 'H0'.\r\n"));
            }
            break;
            
        case 'P': //Pump
            if (commandvalue != 1){
                settings->pump_enable = 0;
                send_string_p(PSTR("Pump disabled, enable with 'P1'.\r\n"));
            }
            else {
                settings->pump_enable = 1;
                send_string_p(PSTR("Pump enabled, disable with 'P0'.\r\n"));
            }
            break;
            
        case 'f': //Fill the tank
            if (commandvalue == 0){
                inputs->fill_now = 1;
                send_string_p(PSTR("Filling the tank to "));
                send_uint16(settings->level_fill);
            }
            else {
                settings->level_fill = commandvalue;
                inputs->fill_now = 1;
                send_string_p(PSTR("Filler the tank to "));
                send_uint16(commandvalue);
                //TODO: Figure out how to temporarily set fill to the value.
            }
            break;
        case 'k':
            send_string_p(PSTR("Setting output fill (hw)\r\n"));
//             outputs->fill = 1;
            fill_set(commandvalue);
            break;
        case 'K':
            send_string_p(PSTR("Clearing output fill (hw)\r\n"));
//             outputs->fill = 0;
            OUTPUT_PORT &=  ~(1<<FILL_PIN);
            break;
        case 'j':
            send_string_p(PSTR("Heating the tank to "));
            heater_set(commandvalue);
            break;
            
        case 'b': //Heat the tank
            if (commandvalue == 0){
                inputs->boost_now = 1;
                send_string_p(PSTR("Heating the tank to "));
                send_uint16(settings->temperature_settemp);
            }
            else {
                settings->temperature_settemp = commandvalue;
                inputs->boost_now = 1;
                send_string_p(PSTR("Heating the tank to "));
                send_uint16(settings->temperature_settemp);
            }
            break;
            
        case 't': //Time
            if ( commandvalue == 0 ){
                send_string_p(PSTR("The time is: "));
                send_uint32_half(timestamp);
                send_newline();
            }
            else {
                send_string_p(PSTR("Setting the time to "));
                send_uint32_half(commandvalue);
                send_newline();
                timestamp = commandvalue;
            }
            break;
           
        case 'T':  //Temperature
            if ( commandvalue == 0 ){
                send_string_p(PSTR("The temperature is: "));
                send_uint16(inputs->temperature);
                send_string_p(test_string_degc);
            }
            else {
                settings->temperature_settemp = commandvalue;
                send_string_p(PSTR("Setting temperature setpoint to "));
                send_uint16(settings->temperature_settemp);
                send_string_p(test_string_degc);
            }
            break;
            
        case 'l':  //Level
            send_string_p(PSTR("The level is "));
                send_uint16(inputs->level);
                send_string_p(test_string_mm);
            break;
                
        case 'v':  //Volume
            send_string_p(PSTR("The volume is "));
            send_uint16(inputs->volume);
            send_string_p(test_string_L);
            break;
            
        //Time and temp to hot
        case 'Y': //Time to hot 1
            if ( commandvalue == 0 ){
                send_string_p(PSTR("The time_to_hot_1 is: "));
                send_uint16(settings->time_to_hot_1);
                send_string_p(test_string_s);
            }
            else {
                settings->time_to_hot_1 = commandvalue;
                send_string_p(PSTR("Setting time_to_hot_1 to "));
                send_uint16(settings->time_to_hot_1);
                send_string_p(test_string_s);
            }
            break;
            
        case 'y': //time_to_hot_1 settemp
            if ( commandvalue == 0 ){
                send_string_p(PSTR("Setpoint temperature 1 is: "));
                send_uint16(settings->temperature_set_1);
                send_string_p(test_string_degc);
            }
            else {
                settings->temperature_set_1 = commandvalue;
                send_string_p(PSTR("Setting setpoint temperature 1 to "));
                send_uint16(settings->temperature_set_1);
                send_string_p(test_string_degc);
            }
            break;
            
        case 'U': //Time to hot 2
            if ( commandvalue == 0 ){
                send_string_p(PSTR("The time_to_hot_2 is: "));
                send_uint16(settings->time_to_hot_2);
                send_string_p(test_string_s);
            }
            else {
                settings->time_to_hot_2 = commandvalue;
                send_string_p(PSTR("Setting time_to_hot_2 to "));
                send_uint16(settings->time_to_hot_2);
                send_string_p(test_string_s);
            }
            break;
            
        case 'u': //time_to_hot_2 settemp
            if ( commandvalue == 0 ){
                send_string_p(PSTR("Setpoint temperature 2 is: "));
                send_uint16(settings->temperature_set_2);
                send_string_p(test_string_degc);
            }
            else {
                settings->temperature_set_2 = commandvalue;
                send_string_p(PSTR("Setting setpoint temperature 2 to "));
                send_uint16(settings->temperature_set_2);
                send_string_p(test_string_degc);
            }
            break;
            
        case 'M': //Maximum level
            if ( commandvalue == 0 ){
                send_string_p(PSTR("Current maximum level is: "));
                send_uint16(settings->level_full);
                send_string_p(test_string_mm);
            }
            else {
                settings->level_full = commandvalue;
                send_string_p(PSTR("Setting maximum level to "));
                send_uint16(settings->level_full);
                send_string_p(test_string_mm);
            }
            break;
            
        case 'm': //Minimum level
            if ( commandvalue == 0 ){
                send_string_p(PSTR("Current minimum level is: "));
                send_uint16(settings->level_min);
                send_string_p(test_string_mm);
            }
            else {
                settings->level_min = commandvalue;
                send_string_p(PSTR("Setting minimum level to "));
                send_uint16(settings->level_min);
                send_string_p(test_string_mm);
            }
            break;
            
        case 'J': //Heater minimum level
            if ( commandvalue == 0 ){
                send_string_p(PSTR("Current heater minimum level is: "));
                send_uint16(settings->level_heater_min);
                send_string_p(test_string_mm);
            }
            else {
                settings->level_heater_min = commandvalue;
                send_string_p(PSTR("Setting heater minimum level to "));
                send_uint16(settings->level_heater_min);
                send_string_p(test_string_mm);
            }
            break;
            
        case 'N': //Current fill level
            if ( commandvalue == 0 ){
                send_string_p(PSTR("Current fill level is: "));
                send_uint16(settings->level_fill);
                send_string_p(test_string_mm);
            }
            else {
                settings->level_fill = commandvalue;
                send_string_p(PSTR("Setting fill level to "));
                send_uint16(settings->level_fill);
                send_string_p(test_string_mm);
            }
            break;
            
        case 'n': //Current fill level in liters
            if ( commandvalue == 0 ){
                send_string_p(PSTR("Current fill level is: "));
                send_uint16(settings->level_fill);
                send_string_p(test_string_L);
            }
//             else { //Not doing as no need for this inverse transform to be in the micro
//                 //                 settings->level_fill = commandvalue;
//                 send_string_p(PSTR("Setting fill level to "));
//                 //                 send_uint16(settings->level_fill);
//                 send_string_p(PSTR("liters."));
//                 send_newline();
//             }
            break;
            
        case 'G': //Maximum temperature 
            if ( commandvalue == 0 ){
                send_string_p(PSTR("Maximum temperature is: "));
                send_uint16(settings->temperature_max);
                send_string_p(test_string_degc);
            }
            else {
                settings->temperature_max = commandvalue;
                send_string_p(PSTR("Setting maximum temperature to "));
                send_uint16(settings->temperature_max);
                send_string_p(test_string_degc);
            }
            break;
            
        case 'g': //Minimum temperature 
            if ( commandvalue == 0 ){
                send_string_p(PSTR("Minimum temperature is: "));
                send_uint16(settings->temperature_min);
                send_string_p(test_string_degc);
            }
            else {
                settings->temperature_min = commandvalue;
                send_string_p(PSTR("Setting minimum temperature to "));
                send_uint16(settings->temperature_min);
                send_string_p(test_string_degc);
            }
            break;
            
        case 's': //Midday sun
            if ( commandvalue == 0 ){
                send_string_p(PSTR("Midday by the sun is at "));
                send_uint16(settings->midsun);
                send_string_p(test_string_s);
            }
            else {
                settings->midsun = commandvalue;
                send_string_p(PSTR("Setting midday by the sun to "));
                send_uint16(settings->midsun);
                send_string_p(test_string_s);
            }
            break;
            
        case 'd': //Daily heating potential
            if ( commandvalue == 0 ){
                send_string_p(PSTR("Daily heating potential is "));
                send_uint16(settings->daily_heat_potential);
                send_string_p(test_string_kWh);
            }
            else {
                settings->midsun = commandvalue;
                send_string_p(PSTR("Setting daily heating potential to "));
                send_uint16(settings->daily_heat_potential);
                send_string_p(test_string_kWh);
            }
            break;
            
        case 'z': //Zero level
            if ( commandvalue == 0 ){
                send_string_p(PSTR("Level zero is: "));
                send_uint16(level_sensor_zero);
                send_newline();
            }
            else {
                level_sensor_zero = commandvalue;
                send_string_p(PSTR("Level zero is: "));
                send_uint16(level_sensor_zero);
                send_newline();
            }
            break;
//                 send_string_p(PSTR("WTF!!\r\n"));
// //                 level();
// //                 level_zero();
//                 send_string_p(PSTR("Zeroing the level.\r\n"));
//             break;
            
        case 'o': //Program to run
            if ( commandvalue == 0 || commandvalue > NUM_PROGRAM){
                send_string_p(PSTR("The running program is number "));
                send_char('1'+state_machine_program);
                send_newline();
            }
            else {
                state_machine_program = commandvalue-1;
                send_string_p(PSTR("Running program switched to "));
                send_char('1'+state_machine_program);
                send_newline();
            }
            break;
            
        case 'O': //Program to configure
            if ( commandvalue == 0 || commandvalue > NUM_PROGRAM){
                send_string_p(PSTR("The program being configured is "));
                send_char('1'+state_machine_config_program);
                send_newline();
            }
            else {
                state_machine_config_program = commandvalue-1;
                send_string_p(PSTR("Now configuring program "));
                send_char('1'+state_machine_config_program);
                send_newline();
            }
            break;
            
        //Temperature sensors
        case 'A': //Intitialise AT30TSE758
            send_string_p(PSTR("Initilaising AT30TSE758 sensor 1...\r\n"));
            if ( !init_AT30TSE758(TEMP_SENSOR1_ADDRESS) ){
                send_string_p(PSTR("Success."));
            }
            else {
                send_string_p(PSTR("Failed!"));
            }
            send_newline();
            send_string_p(PSTR("Initilaising AT30TSE758 sensor 2...\r\n"));
            if ( !init_AT30TSE758(TEMP_SENSOR2_ADDRESS) ){
                send_string_p(PSTR("Success."));
            }
            else {
                send_string_p(PSTR("Failed!"));
            }
            send_newline();
            send_string_p(PSTR("Initilaising AT30TSE758 sensor 3...\r\n"));
            if ( !init_AT30TSE758(TEMP_SENSOR3_ADDRESS) ){
                send_string_p(PSTR("Success."));
            }
            else {
                send_string_p(PSTR("Failed!"));
            }
            send_newline();
            break;
            
        ////////////////////////////////////////////////    
        //I2C related and debugging
        case 'I':
            send_string_p(PSTR("Scanning I2C bus in write mode: \r\n"));
            i2c_safe_write_scan_bus(0x00, 0x7F);
            send_string_p(PSTR("Scanning I2C bus in read mode: \r\n"));
            i2c_safe_read_scan_bus(0x00, 0x7F);
            send_newline();
            send_string_p(PSTR("completed\r\n"));
            break;
        case 'R': //testing i2c_safe_read_sixteen
            send_string_p(PSTR("I2C safe: Reading 16 bits from register 0x0, from address:"));
            send_uint16(commandvalue);
            send_newline();
            send_uint16(i2c_safe_read_sixteen(commandvalue, 0x0) );
            send_newline();
            break;
        case 'Z': //testing read_AT30TSE758
            send_string_p(PSTR("read_AT30TSE758 , from address:"));
            send_uint16(commandvalue);
            send_newline();
            send_uint16(read_AT30TSE758(commandvalue));
            send_newline();
            break;
        case 'X': //testing read_MCP3221
            send_string_p(PSTR("read_MCP3221 from pressure sensor. code is:"));
            send_uint16(read_MCP3221());
            send_newline();
            break;
        case 'C': //Go Crazy!
            send_string_p(PSTR("Going Crazy on level!!!"));
            uint16_t poo;
            uint16_t fart;
            poo =10000;
            fart = read_MCP3221(); //give us 8x origina reading
            while (poo > 0){
                //Use averaging over 8 numbers
                fart = ((fart*7 + read_MCP3221() )>>3);
                send_uint16(fart);
                send_newline();
                _delay_ms(250);
                poo--;
            }
            break;
        case 'B': //Go Crazy!
            send_string_p(PSTR("Going Crazy on temperature!!!"));
            uint16_t wee;
            wee =10000;
            while (wee > 0){
                send_uint16(temperature());
                send_newline();
                poo--;
            }
            break;
        case 'V':
            send_string_p(PSTR("reading i2c_safe_sixteen with adress"));
            send_uint16(wloaded);
            send_string_p(PSTR(", and register "));
            send_uint16(commandvalue);
            send_newline();
            send_uint16(i2c_safe_read_sixteen(wloaded, commandvalue));
            send_newline();
            break;
            // data to write to register
        case 'w':
            wloaded = commandvalue ;
            send_string_p(PSTR("Loading data: "));
            send_uint16(wloaded);
            send_string_p(PSTR(" ready to write.\r\n"));
            break;
            // Toggle an output
        case 'c':
            send_string_p(PSTR("Toggling PortD pin"));
            send_uint16(commandvalue);
//             DDRD |= (1<<commandvalue); 
            send_newline();
            PORTD ^= (1<<commandvalue);
//             pump_set(commandvalue);
            break;

    }
}
