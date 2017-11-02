/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file settings.h
 *
 * @brief Important application specific settings
 *
 * - Compiler:          gcc-avr
 * - Project:           LiFePO4 Charge Controller
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author              Jasper Aorangi
 * @Date: May 2017
 *****************************************************************************/
#pragma once
#include "AVR-lib/clock.h"

// Cell specific defines
#define NUM_CELLS                         28
#define V_MULTIPLIER                      10
#define CHARGED_VPC                       3.55
#define BALANCING_VPC                     3.6
#define OVER_VPC                          3.65
#define UNDER_VPC                         2.8
#define SHUNT_CURRENT                     900

// We want a fudge factor to in reality ensure that total voltage is not a factor in
// process control
#define V_FUDGE                           90

// Time in seconds from power on of circuit to enabing of process control
// to give the PSU's enough time to stabilie
#define PSU_WAIT_ON                       10

// Maximum number of times to retry charging if limits are exceeded
#define SETTINGS_CHARGE_RETRIES_MAX       5
#define SETTINGS_CHARGE_RETRY_WAIT_TIME   MEDIUM_TIME_FRACTION*PSU_WAIT_ON


/** BMS voltage settings **/
#define SETTINGS_BMS_MAX_VOLTAGE_CC       3500
#define SETTINGS_BMS_MAX_VOLTAGE_CV       3500
#define SETTINGS_PID_PROPORTION_CV_BMS    10
#define SETTINGS_PID_MAX_CV_BMS           10
#define SETTINGS_BMS_MAX_VOLTAGE_BALANCING  3560
#define SETTINGS_PID_PROPORTION_BALANCING_BMS 300
#define SETTINGS_PID_MAX_BALANCING_BMS    1
#define SETTINGS_BMSCOMMS_ERROR_COUNT     12

/** Constant Current (CC) settings **/
// CC current
#define SETTINGS_CURRENT_CC               25000
// Voltage at which CC is complete
#define SETTINGS_VOLTAGE_CC_CHARGED       (NUM_CELLS * CHARGED_VPC * V_MULTIPLIER) + V_FUDGE
// PID proportion for CC
#define SETTINGS_PID_PROPORTION_CC        500
// PID maximum, effectivly clamps maximum voltage
#define SETTINGS_PID_MAX_CC               25
/** Constant Voltage (CV) settings **/
// Constant voltage setpoint
#define SETTINGS_VOLTAGE_CV               (NUM_CELLS * BALANCING_VPC * V_MULTIPLIER) + V_FUDGE
// Max current for CV charge
#define SETTINGS_CURRENT_CV               25000
#define SETTINGS_CURRENT_CV_DONE          SHUNT_CURRENT
// PID proportion for CV
#define SETTINGS_PID_PROPORTION_CV        10
// PID maximum, effectivly clamps maximum voltage
#define SETTINGS_PID_MAX_CV               30
/** Balancing ssettings **/
#define SETTINGS_VOLTAGE_BALANCING        (NUM_CELLS * BALANCING_VPC * V_MULTIPLIER) + V_FUDGE
#define SETTINGS_CURRENT_BALANCING        SHUNT_CURRENT
#define SETTINGS_PID_PROPORTION_BALANCING 30
#define SETTINGS_PID_MAX_BALANCING        SETTINGS_PID_MAX_CC



// Process PID settings
#define PWM_THRESHOLD_REDUCTION           1000


// Absolute maximum current. Shutdown if over
#define SETTINGS_CURRENT_MAX              35000
// Absolute maximum battery voltage. Shutdown if over
#define SETTINGS_VOLTAGE_MAX              (NUM_CELLS * OVER_VPC * V_MULTIPLIER) + V_FUDGE
// Minimum battery voltage. Error or batteries disconnected if under.
#define SETTINGS_VOLTAGE_MIN              NUM_CELLS * UNDER_VPC * V_MULTIPLIER
// Maximum temperature of any battery before shutdown.
#define SETTINGS_MAX_BATTERY_TEMP         50
#define SETTINGS_POWER_MAX                2400


// LED indicator timestep
#define SETTINGS_LED_INDICATOR_FREQ       1
