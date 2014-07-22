/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file process-control.c
 *
 * @brief Variables and functions related to the state and controll of a process
 *
 * - Compiler:          gcc-avr
 * - Project:           LiFePO4 Charge Controller
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author              Jasper Aorangi
 * @Date: July 2014
 *****************************************************************************/
#include "process-control.h"

// struct Process process = {
//     {0},
//     {0},
//     {4000,3000,10500,10220,9660,7200,70,40}
// };

/**
 * @brief Get the state of the system, input values and output states
 **/
void get_state(struct Process *process) {
    
    struct Inputs *inputs = &process->inputs;
    struct Outputs *outputs = &process->outputs;
    
    // Get the external states
    inputs->voltage = get_voltage();
    inputs->current = get_current();
    
    // Get the internal states
    outputs->pwm_duty = get_pwm_duty(PWM_CHAN_A, ABSOLUTE);
    outputs->charge_progress = get_SoC(ABSOLUTE);
}