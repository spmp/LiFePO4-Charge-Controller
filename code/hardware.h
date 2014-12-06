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
#include "process-control.h"
#include "esp120.h"
#include "AVR-lib/wd.h"

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
#define I2C_FREQ        50000  //Limit for the HP PSU
#define PWM_TOP         0xFFF  // PWM TOP value, 12bit. Such that PWM Frequency is ClockSpeed(16Mhz)/(2*Prescaler(1)*TOP) = 1.953kHz
#define PWM_START       0xFFF-1 // Which value to start the PWM at on powerup from cold 
#define DESIGN_SOC      8960    // System design State of Charge in Ah

#define PSU_WAIT_ON     10      //Time in seconds from PSU power on to output ready

/* Hardware specific defines: */
#define ONBOARD_LEDPort PORTB
#define ONBOARD_LED_DriectionReg    DDRB
#define ONBOARD_LED     PB5

#define GREEN_LED       PB4
#define RED_LED         PB3

#define PSU_ON_PORT     PORTD
#define PSU1_ON         PD3
#define PSU2_ON         PD5

#define VsensePort      PORTC
#define VsensePin       PC0
#define VsenseADC       ADC0
#define VenseADC_REF    VREF_INTERNAL

#define PSU1_ADDRESS    29
#define PSU2_ADDRESS    30

/**
 * @struct PSU_state, The state PSU's 1 and 2 from I2C data
 **/
struct PSU_state {
    uint16_t PSU1_current;      // PSU1 current
    uint16_t PSU1_temperature;  // PSU1 Temperature
    uint16_t PSU1_line_voltage; // PSU1 Line voltage
    uint8_t  PSU1_PSON_state;   // PSU1 power status
    uint16_t PSU2_current;      // PSU2 current
    uint16_t PSU2_temperature;  // PSU2 Temperature
    uint16_t PSU2_line_voltage; // PSU2 Line voltage
    uint8_t  PSU2_PSON_state;   // PSU2 power status
};

/**
 * @brief Initialise hardware 
 * 
 * Initialise all hardware relevant to the project, usart, i2c, clock, external
 * sensors, Everyting!
 * One call to rule them all
 */
void init_hardware(void);

/**
 * @brief Initialise I/O ports
 * 
 * Initialise PIN I/O states that do not fall into the PWM, I2C/TWi etc categories
 */
void init_io_ports(void);

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
 * @param type, see defines
 * @return uint16, the state of charge
 **/
uint16_t get_SoC( uint8_t op_type );

/**
 * @brief Get the PWM duty cycle
 * 
 * @param channel, PWM channel, see defines 
 * @param type, see defines
 * @return uint16, the pwm duty cycle
 **/
uint16_t get_pwm_duty( uint8_t pwm_chan, uint8_t op_type );

/**
 * @brief Get the state of the PSUs via I2C
 * 
 * @param psu_struct, struct PSU_state, struct in which to store the PSU's states
 **/
void get_psu_state(struct Inputs *ProcessControlInputs);

/**
 * @brief Get PSU's current
 * 
 * @param psu_number, the number 1 or 2 of the PSU
 * @param psu_struct, struct PSU_state, struct with the stored the PSU's states
 * @return uint16 the current flowing through of a PSU
 **/
uint16_t get_psu_current( uint8_t psu_number, struct PSU_state *psu_struct);

/**
 * @brief Get PSU's voltage
 * 
 * @param psu_number, the number 1 or 2 of the PSU
 * @param psu_struct, struct PSU_state, struct with the stored the PSU's states
 * @return uint16 the line voltage of a PSU
 **/
uint16_t get_psu_line_voltage( uint8_t psu_number, struct PSU_state *psu_struct);

/**
 * @brief Get PSU's temperature
 * 
 * @param psu_number, the number 1 or 2 of the PSU
 * @param psu_struct, struct PSU_state, struct with the stored the PSU's states
 * @return uint16 the temperature of a PSU
 **/
uint16_t get_psu_temperature( uint8_t psu_number, struct PSU_state *psu_struct);

/**
 * @brief Power on or off a PSU
 * 
 * @param psu_number, the number 1 or 2 of the PSU
 * @param state, 0 turn off, 1 turn on, other turn off
 **/
void psu_power(uint8_t psu_number, uint8_t state);

/**
 * @brief Check the power state of a PSU in realtime.
 * 
 * @param psu_number, the number 1 or 2 of the PSU
 * @param state, 0 turn off, 1 turn on, other turn off
 **/
uint8_t psu_power_check(uint8_t psu_number);