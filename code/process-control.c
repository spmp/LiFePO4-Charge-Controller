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


/**
 * @brief Controll the process; Charging LiFePO4 batteries
 * 
 * This is the master process to manage the inputs and outputs for managing
 * two HP 3kW PSU's in order to charge and protect a 28*3.2V@100Ah LiFePO4 
 * battery bank
 * 
 * The task is generally
 *  * Read inputs and systems state 
 *  * Make descisions on what to do based on that state
 *  * Check for out of bounds and emergency conditions
 *  * Set the outputs
 * 
 * We are aiming for the ability to manage different charging models.
 * This can be achieved through selection by different 
 * 'processes' via the 'process_number' variable of our struct.
 *  This 'process' only effects the descision stage of the process-control
 * 
 *  Process 1
 *  ---------
 *  brief Constant current charging up to a set voltage 
 * 
 *  we will check whether a battery is connected, and its voltage is below the cut off
 *  If it is then we will
 *      Set the PWM to a minimum value
 *      Turn on the PSU's 
 *      Begin constant current charging via PID on current sense to PWM duty cycle
 *  Once the set point is reached we
 *      Turn off the PSU's and start a 'rest' counter
 *      If the constant current time was very small then the rest counter is very small.w 
 *      In the future we could record the Ah and time etc
 *  Once the 'rest' counter has timed out we can go to constant voltage float
 *      Set the PWM to a minimum value
 *      Turn on the PSU's 
 *      Begin constant voltage charging via PID on voltage sense to PWM duty cycle
 *  At any stage the process can be killed or started with a push button LED.
 * 
 *  More details of this particular process are in the c. file
 * @param *process, a struct of type Process in which to save the system state
 **/
void process_control(struct Process *process)
{
    