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
#include "AVR-lib/usart.h"
#include "AVR-lib/timers.h"
#include "AVR-lib/wd.h"
#include "BMScomms/BMScomms.h"
#include "process-control.h"
#include "softuart/softuart.h"

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
 *          @resource I2C, to powersupply 1 and/or 2
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

/* Hardware specific defines: */
#define ONBOARD_LEDPort           PORTB
#define ONBOARD_LED_DirectionReg  DDRB
#define ONBOARD_LED               PB5

#define LED_PORT        PORTB
#define LED_GREEN       PB4
#define LED_RED         PB3
#define LED_ON          1
#define LED_OFF         0

#define PSU_ON_DDR      DDRD
#define PSU_ON_PORT     PORTD
#define PSU1_ON         PD3
#define PSU2_ON         PD5

#define VsenseDDR       DDRC
#define VsensePort      PORTC
#define VsensePin       PC0

#define V_SENSE_ADC_PIN 0
#define V_SENSE_ADC_REF VREF_VCC

#define A_SENSE_ADC_PIN PC2
#define A_SENSE_ADC_REF VREF_VCC

#define PSU1_ADDRESS    24
#define PSU2_ADDRESS    24

/** Analog temperature sensor coefficients **/
#define ANALOG_CURRENT_OFFSET_CODE -10690
#define ANALOG_CURRENT_SLOPE_CODE  86.067

// ADC Voltage linear calibration points for high and low current
#define ANALOG_VOLTAGE_HIGH_A_OFFSET_CODE 96.1
#define ANALOG_VOLTAGE_HIGH_A_SLOPE_CODE  0.9497
#define ANALOG_VOLTAGE_LOW_A_OFFSET_CODE  96.1
#define ANALOG_VOLTAGE_LOW_A_SLOPE_CODE   0.9497

// #define ANALOG_VOLTAGE_OFFSET_CODE 154
// #define ANALOG_VOLTAGE_SLOPE_CODE  0.9

// Measuring at battery at 30A with old diode
// #define ANALOG_VOLTAGE_OFFSET_CODE 200
// #define ANALOG_VOLTAGE_SLOPE_CODE  0.823

// #define ANALOG_VOLTAGE_OFFSET_CODE 457
// #define ANALOG_VOLTAGE_SLOPE_CODE  0.53773585
// #define ANALOG_VOLTAGE_OFFSET_CODE 0
// #define ANALOG_VOLTAGE_SLOPE_CODE  1.06788865

/** Bit banging macros thanks to AVRFreaks **/
#define BIT(x) (0x01 << (x))
#define bit_set(p,m) ((p) |= (m))
#define bit_clear(p,m) ((p) &= ~(m))
#define bit_flip(p,m) ((p) ^= (m))

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
uint16_t get_voltage(float slope, float offset);

/**
 * @brief Get the current flowing through the system
 * 
 * @return uint16, the current in A*100
 **/
uint16_t get_current(float slope, float offset);

/**
 * @brief Get the current via analog sensor connected to pin A2/ADC2
 * 
 * @param psu_struct, struct PSU_state, struct in which to store the PSU's states
 **/
uint16_t get_analog_current(float slope, float offset);

/**
 * @brief Get the PWM duty cycle
 * 
 * @param channel, PWM channel, see defines 
 * @param type, see defines
 * @return uint16, the pwm duty cycle
 **/
uint16_t get_pwm_duty( uint8_t pwm_chan, uint8_t op_type );

/** 
 * @brief Set the indicator LED's to the given state, being on (LED_ON) or
 * off (LED_OFF)
 **/
void led_indicators_set(uint8_t led_green, uint8_t led_red);

/**
 * @brief Power on or off a PSU
 * 
 * @param psu_number, the number 1 or 2 of the PSU
 * @param state, 0 turn off, 1 turn on, other turn off
 **/
void psu_power(uint8_t psu_number, uint8_t state);

/**
 * @brief toggle the onboard LED
 **/
void led_onboard_toggle();
