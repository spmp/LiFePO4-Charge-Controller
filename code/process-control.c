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

uint8_t begin_process_control_flag = 0;
uint8_t process_control_running_flag = 0;
uint8_t process_control_enable = 1;               //enabled by default

/** @brief struct(s) for PID **/
struct u_PID_DATA pidData_cc;    // PID data for constant current
struct u_PID_DATA pidData_cv;    // PID data for constant voltage
// = {   // PID data for constant voltage
//     .lastProcessValue = 0,//! Last process value, used to find derivative of process value.
//     .sumError = 0,//! Summation of errors, used for integrate calculations
//     .P_Factor = 1,//! The Proportional tuning constant, multiplied with SCALING_FACTOR
//     .I_Factor = 0,//! The Integral tuning constant, multiplied with SCALING_FACTOR
//     .D_Factor = 0,//! The Derivative tuning constant, multiplied with SCALING_FACTOR
//     .maxError = MAX_UINT,//! Maximum allowed error, avoid overflow
//     .maxSumError = MAX_LONG//! Maximum allowed sumerror, avoid overflow
// };

/** @brief struct Process process **/
// struct Process process = {
// {0},
// {0},
// {1,4000,3000,10500,10220,9660,7200,70,40}
// };
struct Process process = {
    {0},
    {
    .pwm_duty = 10,          // Duty cycle of pwm_duty
    .PSU_state = 20,         // State of the PSU's
    .Ah_count = 30,          // How many Ah's have passed through the charger in this cycle
    .charge_timer = 40,      // How long have we been in the 'bulk' charging phase
    .cur_rest_time = 50,     // How long have we been resting?
    .rest_timer = 60,        // How long do we need to rest for?
    .charge_state = 70,      // State of charging, Bulk, rest, float, Done
    .charge_progress = 80   // Percentage of charging done.
    },
    {
    .process_number = 1,    // An index for the process, such that process_control knows with process to run.
    .current_max = 4000,       // Absolute maximum current. Shutdown if over
    .charge_current = 3000,    // Current to charge the batteries at
    .voltage_max = 10500,       // Absolute maximum battery voltage. Shutdown if over
    .charged_voltage = 3000,   // Voltage at which to stop bulk charging
    .float_voltage = 9660,     // Voltage at which to float the batteries
    .rest_time = 36,         // Time between charged voltage and driving or float/done. This is in seconds per Ah
    .max_PSU_temp = 70,      // Maximum temperature for any PSU before shutdown
    .max_battery_temp = 50,  // Maximum temperature of any battery before shutdown.
    /** PID **/
    .cc_P_Factor = 1,        //! The cc Proportional tuning constant, multiplied with SCALING_FACTOR
    .cc_I_Factor = 0,        //! The cc Integral tuning constant, multiplied with SCALING_FACTOR
    .cc_D_Factor = 0,        //! The cc Derivative tuning constant, multiplied with SCALING_FACTOR
    .cv_P_Factor = 4,        //! The cv Proportional tuning constant, multiplied with SCALING_FACTOR
    .cv_I_Factor = 0,        //! The cv Integral tuning constant, multiplied with SCALING_FACTOR
    .cv_D_Factor = 3        //! The cv Derivative tuning constant, multiplied with SCALING_FACTOR
    }
};
/** 
 * @brief Initialise the PID from Process struct
 * @param *process, a struct of type Process in which get the pid initialisation variables
 **/
void init_PID(struct Process *process)
{
    struct Settings *settings = &process->settings;
    u_pid_Init(settings->cc_P_Factor * SCALING_FACTOR, settings->cc_I_Factor * SCALING_FACTOR, settings->cc_D_Factor * SCALING_FACTOR, &pidData_cc);
    u_pid_Init(settings->cv_P_Factor * SCALING_FACTOR, settings->cv_I_Factor * SCALING_FACTOR, settings->cv_D_Factor * SCALING_FACTOR, &pidData_cv);
}

/**
 * @brief Get the state of the system, input values and output states
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void get_state(struct Process *process) {
    
    struct Inputs *inputs = &process->inputs;
    struct Outputs *outputs = &process->outputs;
    
    // Get the external states
    inputs->voltage = get_voltage();
    inputs->current = get_current();
    
    // Get the internal states
    outputs->pwm_duty = get_pwm_duty(PWM_CHAN_A, ABSOLUTE);
//     outputs->charge_progress = get_SoC(ABSOLUTE);
}

/**
 * @brief Calculate the outputs in order to controll the process
 * 
 * Details are elsewhere and in code.
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void calculate_outputs(struct Process* process)
{
    
    struct Inputs *inputs = &process->inputs;
    struct Outputs *outputs = &process->outputs;
    struct Settings *settings = &process->settings;
    
    outputs->Ah_count = u_pid_Controller(settings->charged_voltage, inputs->voltage, &pidData_cv);
    outputs->pwm_duty += (int32_t)(outputs->Ah_count/8);
    
}

/**
 * @brief Check that no inputs or propoesd outputs will break anything
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void check_limits(struct Process* process)
{
    //
}

/**
 * @brief Set the calculated outputs to hardware
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void set_outputs(struct Process* process)
{
    struct Outputs *outputs = &process->outputs;
    set_pwm(PWM_CHAN_A,outputs->pwm_duty);
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
 *  -# we will check whether a battery is connected, and its voltage is below the cut off
 *  -# If it is then we will
 *      + Set the PWM to a minimum value
 *      + Turn on the PSU's 
 *      + Begin constant current charging via PID on current sense to PWM duty cycle
 *  -# Once the set point is reached we
 *      + Turn off the PSU's and start a 'rest' counter
 *      + If the constant current time was very small then the rest counter is very small.w 
 *      + In the future we could record the Ah and time etc
 *  -# Once the 'rest' counter has timed out we can go to constant voltage float
 *      + Set the PWM to a minimum value
 *      + Turn on the PSU's 
 *      + Begin constant voltage charging via PID on voltage sense to PWM duty cycle
 *  -# At any stage the process can be killed or started with a push button.
 *  Visual aids/outputs have not been added or thought about yet. On car LED strips look pretty cool!
 * 
 * We do all this by splitting the process into the following functions:
 * @function get_state(process), Get the state of the system
 * @function calculate_outputs(process), Calculate the updates to the outputs
 * @function check_limits(process), Check that no inputs or propoesd outputs will break anything
 * @function set_outputs(process), Set the updated output states to hardware
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void process_control(struct Process *process)
{
//     /**make sure only one instance of process_control is running, and is enabled **/
//     if (process_control_running_flag || !process_control_enable) {
//         return;
//     }
//     // Process_control is running
//     process_control_running_flag = 1;
    
    /**First things first, lets get the system state **/
    get_state(process);         // That was easy!
    
    /** Now lets calculate how to change things to maintain the process **/
    calculate_outputs(process);
    
    /** Lets check that nothing has gone wrong or will go wrong if we use these settings **/
    check_limits(process);
    
    /** Now we set the proposed updates to hardware **/
    set_outputs(process);
    
    //Process_control has finished
//     process_control_running_flag = 1;
}
