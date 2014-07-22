/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file hardware.h
 *
 * @brief Header for Hardware.c, setting up project specific Hardware
 *
 * - Compiler:          gcc-avr
 * - Project:           LiFePO4 Charge Controller
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author               Jasper Aorangi
 * @Date: July 2014
 *****************************************************************************/

#pragma once
#include <avr/io.h>
#include "AVR-lib/adc.h"
#include "AVR-lib/i2c_safe.h"
#include "AVR-lib/usart.h"
#include "AVR-lib/timers.h"

/** 
 * This header/c file together setup the project specific hardware and resources
 * 
 * Hardware Description:
 *  The basic unit is an Arduino Nano, an AT Mega 328p
 *  
 *  Inputs
 *      Current sensing, either by I2C or ADC from shunt+IN-AMP
 *          @resource ADC, internal 10bit
 *              OR
 *          @resource I2C, to powersupply 1 and/or two
 *          @resource PC5, I2C SCL
 *          @resource PC4, I2C SDA
 *              @I2Caddress 0xA0
 *                  ... to ...
 *              @I2Caddress 0xAE
 *          @var current, uint16, mA
 *      Voltage sensing via a 100:1 divider with zener protection to int. ADC
 *          @resource ADC0/PC0, internal 10bit, 1.1Vref, giving resolution of 0.11V (Arduino Nano A0)
 *          @var voltage, uint16, mV
 *              OR
 *          @resource I2C, to 16bit ADC
 *          @resource PC5, I2C SCL
 *          @resource PC4, I2C SDA
 *              @I2Caddress 0xF4628, or (0xF4629,32,33)
 *      Switches
 *          @resource ???
 *  Outputs:
 *      Heartbeat
 *          @resource Timer/Counter2, Clock heartbeat
 *          @resource PB5, on board LED
 *      Voltage to the PSU voltage set pin. PWM to RC-network
 *          @resource Timer/Counter1, in PWM mode 8
 *          @resource OC1A/PB1 (Arduino Nano D9)
 *          @var duty, uint16, PWM duty cyle
 *      Serial input outuput
 *          @resource USART
 *          @resource PD0, Rx
 *          @resource PD1, Tx
 *      Serial, software serial Rx for Cycle analyst input.
 *          @resource ???
 **/

/** Outupt type defines: **/
#define ABSOLUTE        0       // Absolute value of variable under inspection
#define PERCENT         1       // Percentage*100

/** PWM Channel names **/
#define PWM_CHAN_A      0
#define PWM_CHAN_B      1

/* Project specific defines */
#define USART_BAUDE     38400   //Highest stable Baude at 16MhZ FCPU
#define I2C_FREQ        100000  //Limit for the HP PSU
#define PWM_TOP         0xFFFF  // PWM TOP value, 16bit.
#define DESIGN_SOC      8960    // System design State of Charge in Ah

/* Hardware specific defines: */
#define ONBOARD_LEDPort PORTB
#define ONBOARD_LED_DriectionReg    DDRB
#define ONBOARD_LED     PB5

#define VsensePort      PORTC
#define VsensePin       PC0
#define VsenseADC       ADC0
#define VenseADC_REF    VREF_INTERNAL

/**
 * @brief Initialise hardware 
 * 
 * Initialise all hardware relevant to the project, usart, i2c, clock, external
 * sensors, Everyting!
 * One call to rule them all
 */
void init_hardware(void);

/**
 * The followng are functions abstacting measurements into internal values.
 **/

/**
 * @brief Get voltage measurement, taking into consideration protection diode drop
 * 
 * @return uint16, the corrected voltage in mV
 **/
uint16_t get_voltage( void );

/**
 * @brief Get the current flowing through the system
 * 
 * @return uint16, the current in A*100
 **/
uint16_t get_current( void );

/**
 * @brief Get the state of charge of the batteries
 * 
 * Gets the state of charge in Ah or percentage*100
 * @var type, see defines
 * @return uint16, the state of charge
 **/
uint16_t get_SoC( uint8_t op_type );

/**
 * @brief Get the PWM duty cycle
 * 
 * @var channel, PWM channel, see defines 
 * @var type, see defines
 * @return uint16, the pwm duty cycle
 **/
uint16_t get_pwm_duty( uint8_t pwm_chan, uint8_t op_type );