/*Tiis   file has been prepared for Doxygen automatic documentation generation.*/
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
    wd_reset();
    WD_SET(WD_OFF);
    init_clock();
    init_io_ports();
    init_usart(USART_BAUDE, F_CPU);
    bmscomms_init_suart();
    init_pwm(PWM_TOP, 1);
    set_pwm(1,PWM_START);
    WD_SET(WD_RST,WDTO_4S);
}

/**
 * @brief Initialise I/O ports
 * 
 * Initialise PIN I/O states that do not fall into the PWM, I2C/TWi etc categories
 */
void init_io_ports(void ) {

    //Set Internal LED pin to output
    DDRB |= (1 << ONBOARD_LED);
    //Set Green LED pin to output
    DDRB |= (1 << GREEN_LED);
    //Set Red LED pin to output
    DDRB |= (1 << RED_LED);
    
    //Set PSU1_ON pin to output
    DDRD |= (1 << PSU1_ON);
    //Set PSU2_ON pin to output
    DDRD |= (1 << PSU2_ON);
    
    //Set outputs OFF
    PORTD &= ~((1 << PSU1_ON) | (1 << PSU2_ON));
    //Set Green LED ON
    PORTB |= (1 << GREEN_LED);

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
    return read_ADC_pin_millivolts(0, VenseADC_REF);
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

/**
 * @brief Power on or off a PSU
 * 
 * @param psu_number, the number 1 or 2 of the PSU. Anything else (0,3,4..) does both
 * @param state, 0 turn off, 1 turn on, other turn off
 **/
void psu_power(uint8_t psu_number, uint8_t state)
{
    //Send the I2R command to turn the PSU on
    if (psu_number == 1 ) {
        if (state == 1) {
            //turn on
            PSU_ON_PORT |= (1 << PSU1_ON);
        }
        else {
            //turn off
            PSU_ON_PORT &= ~(1 << PSU1_ON);
        }
    }
    //Send the I2R command to turn the PSU on
    else if (psu_number == 2 ) {
        if (state == 1) {
            //turn on
            PSU_ON_PORT |= (1 << PSU2_ON);
        }
        else {
            //turn off
            PSU_ON_PORT &= ~(1 << PSU2_ON);
        }
    }
    else {
        if (state == 1 ){
            //turn on both PSU's
            PSU_ON_PORT |= ((1 << PSU1_ON) | (1 << PSU2_ON));
        }
        else {
            //turn off both PSU's
            PSU_ON_PORT &= ~((1 << PSU1_ON) | (1 << PSU2_ON));
        }
    }
}

