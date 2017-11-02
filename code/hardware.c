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
    wd_reset();
    WD_SET(WD_OFF);
    init_clock();
    init_io_ports();
    init_usart(USART_BAUDE, F_CPU);
    bmscomms_init_suart();
    //softuart_init(4800);
    init_pwm(PWM_TOP, 1);
    set_pwm(1,PWM_START);
    WD_SET(WD_RST,WDTO_4S);
    // Turn indicaor LED's off
    led_indicators_set(LED_OFF, LED_OFF);
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
    DDRB |= (1 << LED_GREEN);
    //Set Red LED pin to output
    DDRB |= (1 << LED_RED);
    
    //Set PSU1_ON pin to output
    PSU_ON_DDR |= (1 << PSU1_ON);
    //Set PSU2_ON pin to output
    PSU_ON_DDR |= (1 << PSU2_ON);
    
    //Set outputs OFF
    PSU_ON_PORT &= ~((1 << PSU1_ON) | (1 << PSU2_ON));
    //Set Green LED ON
    LED_PORT |= (1 << LED_GREEN);
}

/**
 * The followng are functions abstacting measurements into internal values.
 **/

/**
 * @brief Get voltage measurement, taking into consideration protection diode drop
 * 
 * @return uint16, the corrected voltage in mV
 **/
uint16_t get_voltage(float slope, float offset){
  float currentTmp = read_ADc_pin_linearFunc(V_SENSE_ADC_PIN, V_SENSE_ADC_REF, slope, offset);
  if (currentTmp >= 0){
    return currentTmp;
  } else {
    return 0;
  }
}

/**
 * @brief Get the current via analog sensor connected to pin A2/ADC2/PC2
 * 
 * @return uint16, the current in A*1000
 **/
uint16_t get_analog_current(float slope, float offset){
  float currentTmp = read_ADc_pin_linearFunc(A_SENSE_ADC_PIN, A_SENSE_ADC_REF, slope, offset);
  if (currentTmp >= CURRENT_MEASURE_MAX){
    return 0;
  } else if (currentTmp >= 0){
    return currentTmp;
  }
  return 0;
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

/** 
 * @brief Set the indicator LED's to the given state, being on (LED_ON) or
 * off (LED_OFF)
 **/
void led_indicators_set(uint8_t led_green, uint8_t led_red) {
  if (led_green == LED_ON) {
    bit_set(LED_PORT, BIT(LED_GREEN)); 
  }
  else
  {
    bit_clear(LED_PORT, BIT(LED_GREEN));
  }
  if (led_red == LED_ON) {
    bit_set(LED_PORT, BIT(LED_RED)); 
  }
  else
  {
    bit_clear(LED_PORT, BIT(LED_RED));
  }
}

/**
 * @brief toggle the onboard LED
 **/
void led_onboard_toggle() {
  bit_flip(ONBOARD_LEDPort, BIT(ONBOARD_LED));
}
