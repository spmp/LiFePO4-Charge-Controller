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
#include "AVR-lib/lib/pid.h"
#include "AVR-lib/usart.h"
#include "AVR-lib/clock.h"
#include "BMScomms/BMScomms.h"
#include "settings.h"
#include "hardware.h"

/**
 * @brief Defines for `calculate_outputs`
 * 
 * @def CHARGE_MODE_OFF                 Charging off/finished
 * @def CHARGE_MODE_CONSTSANT_CURRENT   Constant current charging
 * @def CHARGE_MODE_ABSORBTION          Absorbtion charge
 * @def CHARGE_MODE_CONSTANT_VOLTAGE    Constand voltage charging
 * 
 * @def PID_MAX_OUTPUT                  Maximum PID output - essential max PWM value
 * @def PID_CONSTANT_CURRENT_PROPORTION The proportion (P) for constant current PID
 * 
 * @def SETPOINT_REACHED_COUNT          The number of cycles with measurement above the setpoint
 *                                      before it is considered `reached`
 * @def PWM_THRESHOLD_REDUCTION         How much to reduce the PWM by if over threshold detected
 **/

// Charge modes
#define CHARGE_MODE_OFF                 1
#define CHARGE_MODE_CONSTANT_CURRENT    2
#define CHARGE_MODE_INTERIM             9
#define CHARGE_MODE_BALANCE             3
#define CHARGE_MODE_CONSTANT_VOLTAGE    4
#define CHARGE_MODE_CONSTANT_POWER      5
#define CHARGE_MODE_ABSORBTION          6
#define CHARGE_MODE_RETRY_WAIT          7
#define CHARGE_MODE_OFF_ERROR           8

#define PWM_THRESHOLD_REDUCTION         1000

#define SETPOINT_REACHED_COUNT          4

// LED Blinking
#define LED_BLINK_PERIOD                10

extern uint8_t ChargeMode;
extern uint16_t cPIDmaxread;

extern uint8_t begin_process_control_flag;
extern uint8_t process_control_running_flag;
extern struct u_PID_DATA pidData_cv;     // PID data for constant voltage

struct Inputs {
    uint16_t voltage;           // Voltage across the terminals in v*100
    uint16_t current;           // Current flowing through the controller in A*100
    uint16_t BMS_max_voltage;   // Signal from the BMS that a battery is overvoltage
    uint8_t  BMS_balancing;     // Whether the BMS is balancing
    uint8_t  BMS_status;      // Signal from the BMS that a battery is overvoltage
    uint8_t  PSU1StatusReg;     // PSU1 Status Register
    uint8_t  PSU2StatusReg;     // PSU2 Status Register
};

struct Outputs {
    int16_t  pwm_duty;          // Duty cycle of pwm_duty
    uint8_t  PSU_state;         // State of the PSU's
    uint8_t  error_code;        // Code of error that is encounted, will be set and cleared regularily - needs to be writtne to EEPROM somewhere
    uint8_t  charge_mode;
    uint8_t  charge_retries;    // Numer of times we have retried charging
    uint8_t  last_charge_mode;  // The charge mode that was in progress when error occured
    uint32_t time_wait_retry;   // Time when retry wait is dones
    uint8_t  led_green;         // Green LED
    uint8_t  led_red;           // Red LED
};

struct Settings {
    uint8_t  program_number;    // An index for the process, such that process_control knows with process to run.
    uint8_t  charge_retries_max;// Max number of times to try charging given stop conditions
    // CC
    uint16_t current_cc;        // Current to charge the batteries at
    uint16_t voltage_cc;        // Voltage at which to switch to CV
    uint16_t BMS_max_voltage_cc;// Maximum BMS voltage over which switch to CV
    uint16_t pid_proportion_cc; // Proportional parameter for CC PID
    // CV
    uint16_t BMS_max_voltage_cv;// Maximum BMS voltage to track for CV charging
    uint16_t voltage_cv;        // Voltage at which to kill CV charging
    uint16_t current_cv;        // Max current for CV charge
    uint16_t current_cv_done;   // Current below which CV stops and balancing charge starts
    uint16_t pid_proportion_cv; // Proportional parameter for CV PID
    uint16_t pid_proportion_cv_bms; // Proportion parameter for CV when reference is BMX_max_voltage
    // Balancing
    uint16_t BMS_max_voltage_balancing ;// Maximum BMS voltage to track for balance charging
    uint16_t current_balancing; // Current to balance batteries at
    uint16_t voltage_balancing; // Voltage at which to stop bulk charging
    uint16_t pid_proportion_balancing; // Proportional parameter for CC PID
    uint16_t pid_proportion_balancing_bms; // Proportion parameter for Balancing when reference is BMX_max_voltage
    // Limits
    uint16_t power_max;          // Power to charge the batteries at, whilst current below current_cp_max
    uint16_t current_max;       // Absolute maximum current. Shutdown if over current_max and above current_charge
    uint16_t voltage_max;       // Absolute maximum battery voltage. Shutdown if over
    uint16_t voltage_min;       // Minimum battery voltage. Error or batteries disconnected if under. voltage_max and above voltage_charged
    uint16_t max_battery_temp;  // Maximum temperature of any battery before shutdown.
    /** PID **/
    int16_t PIDoutput;
    /** Voltage calibration **/
    float analog_voltage_offset_code;
    float analog_voltage_slope_code;
    /** Current calibration **/
    float analog_current_offset_code;
    float analog_current_slope_code;
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
 * @brief Find the current setpoint given a current, a voltage, and a maximum current
 * 
 * @param power The desired power in Watts
 * @param current_max The maximum current in A*100
 * @param measured_voltage The measured voltage in V*100
 * @return The currrent required to meet CP conditions
 **/
uint16_t cp_current_calc(uint16_t power_set, uint16_t current_max, uint16_t measured_voltage);
