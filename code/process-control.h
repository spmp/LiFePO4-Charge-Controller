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
#include "hardware.h"

struct Inputs {
    uint16_t voltage;           // Voltage across the terminals in v*100
    uint16_t current;           // Current flowing through the controller in A*100
    uint8_t  BMS_overvolt;      // Signal from the BMS that a battery is overvoltage
    uint8_t  BMS_overtemp;      // Signal from the BMS that a battery is over temperature threshold
    uint16_t battery_temp;      // Temperature of the hottest battery
//     uint16_t PSU1_temperature;  // PSU1 Temperature
//     uint16_t PSU1_line_voltage; // PSU1 Line voltage
//     uint16_t PSU1_temperature;  // PSU1 Temperature
//     uint16_t PSU1_line_voltage; // PSU1 Line voltage
//     etc.    
};

struct Outputs {
    uint16_t pwm_duty;          // Duty cycle of pwm_duty
    uint8_t  PSU_state;         // State of the PSU's
    uint8_t  charge_state;      // State of charging, Bulk, rest, float, Done
    uint8_t  charge_progress;   // Percentage of charging done.
};

struct Settings {
    uint16_t current_max;       // Absolute maximum current. Shutdown if over
    uint16_t charge_current;    // Current to charge the batteries at
    uint16_t voltage_max;       // Absolute maximum battery voltage. Shutdown if over
    uint16_t charged_voltage;   // Voltage at which to stop bulk charging
    uint16_t float_voltage;     // Voltage at which to float the batteries
    uint16_t rest_time;         // Time between charged voltage and driving or float/done
    uint16_t max_PSU_temp;      // Maximum temperature for any PSU before shutdown
    uint16_t max_battery_temp;  // Maximum temperature of any battery before shutdown.
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
 * @brief Get the state of the system, input values and output states
 * @param *process, a struct of type Process in which to save the system state
 **/
void get_state(struct Process *process);
