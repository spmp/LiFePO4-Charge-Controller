/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file hardware.c
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
#include "hardware.h"

/**
 * @brief Initialise hardware 
 * 
 * Initialise all hardware relevant to the project, usart, i2c, clock, external
 * sensors, Everyting!
 * One call to rule them all
 */
void init_hardware(void ) {
    init_clock();
    init_usart(USART_BAUDE, F_CPU);
    i2c_init(I2C_FREQ,F_CPU);
    init_pwm(PWM_TOP, 1);
}

/**
 * The followng are functions abstacting measurements into internal values.
 **/

/**
 * @brief Get voltage measurement, taking into consideration protection diode drop
 * 
 * @return uint16, the corrected voltage in mV
 **/
uint16_t get_voltage( void )
{
//     return read_ADC_pin_millivolts(0, VREF_INTERNAL);
    return read_ADC_pin_millivolts(0, VREF_VCC);
}

/**
 * @brief Get the current flowing through the system
 * 
 * @return uint16, the current in A*100
 **/
uint16_t get_current( void )
{
    return 20000;
}


/**
 * @brief Get the state of charge of the batteries
 * 
 * Gets the state of charge in Ah or percentage*100
 * @var type, see defines
 * @return uint16, the state of charge
 **/
uint16_t get_SoC( uint8_t op_type )
{
    uint16_t soc = 531;
    
    if (!op_type){
        return soc;
    }
    else {
        return ((uint32_t)soc*100)/DESIGN_SOC;
    }
}

/**
 * @brief Get the PWM duty cycle
 * 
 * @var type, see defines
 * @return uint16, the pwm duty cycle
 **/
uint16_t get_pwm_duty( uint8_t pwm_chan, uint8_t op_type )
{
    if (!pwm_chan){
        if (!op_type){
            return OCR1A;
        }
        else {
            return ((uint32_t)OCR1A*10000)/PWM_TOP;
        }
    }
    else {
        if (!op_type){
            return OCR1B;
        }
        else {
            return ((uint32_t)OCR1B*10000)/PWM_TOP;
        }
    }
}