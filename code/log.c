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
    struct Settings *settings = &process->settings;
    
    //Temporarily disabled logging
    if ( logenable ){
        send_string_p(PSTR("l ")); // sending a log message
        send_uint32_half(timestamp & 0xFFFFFFFF);
        send_string_p(PSTR(" c "));
        send_uint16(inputs->current);
//         send_uint16(inputs->PSU1AnalogData.Current);
//         send_string_p(PSTR(" c1 "));
//         send_uint16(inputs->PSU2AnalogData.Current);
        send_string_p(PSTR(" v "));
        send_uint16(inputs->voltage);
        send_string_p(PSTR(" Progress "));
        send_uint8(PIDtype);
        send_string_p(PSTR(" Ahx100 "));
        send_uint16(outputs->Ah_count);
        send_string_p(PSTR(" Charge state "));
        send_uint16(outputs->charge_state);
        send_string_p(PSTR(" pwm "));
        send_uint16(outputs->pwm_duty);
        send_string_p(PSTR(" Vsp "));
        send_uint16(settings->voltage_charged);
        send_string_p(PSTR(" Csp "));
        send_uint16(settings->current_charge);
        send_string_p(PSTR(" PiD "));
        if (settings->PIDoutput < 0){
            send_string_p(PSTR("-"));
            send_uint16(-(settings->PIDoutput));
        }
        else {
            send_uint16(settings->PIDoutput);
        }
            
            
//         send_string_p(PSTR(" P "));
//         send_uint16(pidData_cv.P_Factor);
//         send_string_p(PSTR(" I "));
//         send_uint16(pidData_cv.I_Factor);
//         send_string_p(PSTR(" D "));
//         send_uint16(pidData_cv.D_Factor);
//         send_string_p(PSTR(" LastVal "));
//         send_uint16(pidData_cv.lastProcessValue);
//         send_string_p(PSTR(" SumEr "));
//         send_uint32(pidData_cv.sumError);
//         send_string_p(PSTR(" MaxEr "));
//         send_uint32(pidData_cv.maxError);
//         send_string_p(PSTR(" MaxSumEr "));
//         send_uint32(pidData_cv.maxSumError);
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