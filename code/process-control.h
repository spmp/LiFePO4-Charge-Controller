/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file process-control.h
 *
 * @brief Header for process-control.c, variables and functions related to the state and controll of a process
 *
 * - Compiler:          gcc-avr
 * - Project:           LiFePO4 Charge Controller
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author              Jasper Aorangi
 * @Date: July 2014
 *****************************************************************************/
#pragma once
#include <avr/io.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "hardware.h"
#include "AVR-lib/lib/pid.h"
#include "AVR-lib/usart.h"
#include "AVR-lib/clock.h"
#include "esp120.h"

extern uint8_t PIDtype;
extern uint16_t cPIDmaxread;

extern uint8_t begin_process_control_flag;
extern uint8_t process_control_running_flag;
extern struct u_PID_DATA pidData_cv;     // PID data for constant voltage

struct Inputs {
    uint16_t voltage;           // Voltage across the terminals in v*100
    uint16_t current;           // Current flowing through the controller in A*100
    uint8_t  BMS_overvolt;      // Signal from the BMS that a battery is overvoltage
    uint8_t  BMS_overtemp;      // Signal from the BMS that a battery is over temperature threshold
    uint16_t battery_temp;      // Temperature of the hottest battery
    struct ESP120AnalogData PSU1AnalogData;
    uint8_t PSU1StatusReg;      // PSU1 Status Register
    struct ESP120AnalogData PSU2AnalogData;
    uint8_t PSU2StatusReg;      // PSU2 Status Register
};

struct Outputs {
    int16_t pwm_duty;          // Duty cycle of pwm_duty
    uint8_t  PSU_state;         // State of the PSU's
    uint16_t Ah_count;          // How many Ah's have passed through the charger in this cycle. (Ahx100)
    uint16_t charge_timer;      // How long have we been in the 'bulk' charging phase
    uint16_t cur_rest_time;     // How long have we been resting?
    uint16_t rest_timer;        // How long do we need to rest for?
    uint16_t float_timer;        // How long do we need to float charge?
    uint8_t  charge_state;      // State of charging; 0 not charging, 1 Bulk charging, 2 resting, 3 float charging,4 Done (examples), 7 stop - error
    uint8_t  error_code;        // Code of error that is encounted, will be set and cleared regularily - needs to be writtne to EEPROM somewhere
    uint8_t  charge_progress;   // Percentage of charging done.
};

struct Settings {
    uint8_t  program_number;    // An index for the process, such that process_control knows with process to run.
    uint16_t current_max;       // Absolute maximum current. Shutdown if over
    uint16_t current_threhold;  // Current at which to try to recover over current condition, needs to be lower than current_max and above current_charge
    uint16_t current_charge;    // Current to charge the batteries at
    uint16_t voltage_max;       // Absolute maximum battery voltage. Shutdown if over
    uint16_t voltage_min;       // Minimum battery voltage. Error or batteries disconnected if under.
    uint16_t voltage_threshold; // Voltage at which to try to recover over voltage condition, needs to be lower than voltage_max and above voltage_charged
    uint16_t voltage_charged;   // Voltage at which to stop bulk charging
    uint16_t voltage_float;     // Voltage at which to float the batteries
    uint16_t rest_time;         // Time between charged voltage and driving or float/done. This is in seconds per Ah
    uint16_t max_PSU_temp;      // Maximum temperature for any PSU before shutdown
    uint16_t max_PSU_volt;      // Maximum PSU line voltage
    uint16_t min_PSU_volt;      // Minimum PSU voltage
    uint16_t max_battery_temp;  // Maximum temperature of any battery before shutdown.
    /** PID **/
    int16_t PIDoutput;          // Output from the PID algorythm
    int16_t cc_P_Factor;        //! The cc Proportional tuning constant, multiplied with SCALING_FACTOR
    int16_t cc_I_Factor;        //! The cc Integral tuning constant, multiplied with SCALING_FACTOR
    int16_t cc_D_Factor;        //! The cc Derivative tuning constant, multiplied with SCALING_FACTOR
    int16_t cv_P_Factor;        //! The cv Proportional tuning constant, multiplied with SCALING_FACTOR
    int16_t cv_I_Factor;        //! The cv Integral tuning constant, multiplied with SCALING_FACTOR
    int16_t cv_D_Factor;        //! The cv Derivative tuning constant, multiplied with SCALING_FACTOR
};

struct Process {
    struct Inputs inputs;
    struct Outputs outputs;
    struct Settings settings;
};

/**
 * @struct process, The actual struct containing all the process data
 **/
extern struct Process process;

/** 
 * @brief Initialise the PID from Process struct
 * @param *process, a struct of type Process in which get the pid initialisation variables
 **/
void init_PID(struct Process *process);

/**
 * @brief Get the state of the system, input values and output states
 * @param *process, a struct of type Process in which to save the system state
 **/
void get_state(struct Process *process);

/**
 * @brief Calculate the outputs in order to controll the process
 * @param *process, a struct of type Process in which to save the system state
 **/
void calculate_outputs(struct Process *process);

/**
 * @brief Check that no inputs or propoesd outputs will break anything
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void check_limits(struct Process *process);

/**
 * @brief Set the calculated outputs to hardware
 * 
 * @param *process, a struct of type Process in which to save the system state
 **/
void set_outputs(struct Process *process);


/**
 * @brief Enable process control
 **/
void process_control_enable( void );

/**
 * @brief Disable process control
 **/
void process_control_disable(void);

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
 *  More details of this particular process are in the c. file
 * @param *process, a struct of type Process in which to save the system state
 **/
void process_control(struct Process *process);


/**
 * @brief Gravefully turn on the Power Supplies
 * 
 * @retval 0, Everything went well
 * @retval 1, OH dear, there was some error!
 **/
// uint8_t turn_on_PSUs(struct Process *process )