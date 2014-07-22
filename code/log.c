/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file log.c
 *
 * @brief Application specific logging to the hardware USART
 *
 * - Compiler:          gcc-avr
 * - Project:           LiFePO4 Charge Controller
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author              Jasper Aorangi
 * @Date: July 2014
 *****************************************************************************/
#include "log.h"

uint8_t logenable = LOG_ENABLE; //logging enabled/disabled at startup

// Send logging data over serial
void log_to_serial(struct Process *process) {
    struct Inputs *inputs = &process->inputs;
    struct Outputs *outputs = &process->outputs;
    
    //Temporarily disabled logging
    if ( logenable ){
        send_string_p(PSTR("l ")); // sending a log message
        send_uint32_half(timestamp & 0xFFFFFFFF);
        send_string_p(PSTR(" c "));
        send_uint16(inputs->current);
        send_string_p(PSTR(" v "));
        send_uint16(inputs->voltage);
        send_string_p(PSTR(" SoC "));
        send_uint16(outputs-> charge_progress);
        send_string_p(PSTR(" pwm "));
        send_char(outputs->pwm_duty);
//         send_string_p(PSTR(" P "));
//         send_char('0'+pump_state());
//         send_string_p(PSTR(" f "));
//         send_char('0'+outputs->fill);
//         send_string_p(PSTR(" F "));
//         send_char('0'+fill_state());
//         send_string_p(PSTR(" h "));
//         send_char('0'+outputs->heating);
//         send_string_p(PSTR(" H "));
//         send_char('0'+heater_state());
//         send_string_p(PSTR(" OP "));
//         send_uint16(OUTPUT_PORT);
        send_newline();
    }
}

// enable logging
void enable_logging(void){
    logenable = 1;
}

// disable logging
void disable_logging(void){
    logenable = 0;
}