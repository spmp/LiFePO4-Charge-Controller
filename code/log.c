/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file log.c
 *
 * @brief Application specific logging to the hardware USART
 *
 * - Compiler:          gcc-avr
 * - Project:           LiFePO4 Charge Controller
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author              Jasper Aorangi
 * @Date: July 2014
 *****************************************************************************/
#include "log.h"
#include "process-control.h"

uint8_t logenable = LOG_ENABLE; //logging enabled/disabled at startup

// Send logging data over serial
void log_to_serial(struct Process *process) {
  struct Inputs   *inputs   = &process->inputs;
  struct Outputs  *outputs  = &process->outputs;
  struct Settings *settings = &process->settings;
  
  //Temporarily disabled logging
  if ( logenable ){
    send_string_p(PSTR("l ")); // sending a log message
    send_uint32_half(timestamp & 0xFFFFFFFF);
    send_string_p(PSTR(" c "));
    send_uint16(inputs->current);
    send_string_p(PSTR(" v "));
    send_uint16(inputs->voltage);
    send_string_p(PSTR(" vBMS "));
    send_uint16(inputs->BMS_max_voltage);
    send_string_p(PSTR(" B "));
    send_uint8(inputs->BMS_balancing);
    send_string_p(PSTR(" S "));
    send_uint8(inputs->BMS_status);
    send_string_p(PSTR(" p "));
    send_uint16((uint32_t)inputs->current * inputs->voltage / 10000);
    send_string_p(PSTR(" cAc "));
    send_uint16(read_ADC_pin(A_SENSE_ADC_PIN, A_SENSE_ADC_REF));
    send_string_p(PSTR(" cP "));
    send_uint16(get_analog_current(ANALOG_CURRENT_SLOPE_CODE, ANALOG_CURRENT_OFFSET_CODE))  ;
    send_string_p(PSTR(" pwm "));
    send_uint16(outputs->pwm_duty);
    send_string_p(PSTR(" PiD "));
    if (settings->PIDoutput < 0)
    {
      send_string_p(PSTR("-"));
      send_uint16(-(settings->PIDoutput));
    }
    else
    {
      send_string_p(PSTR(" "));
      send_uint16(settings->PIDoutput);
    }
    switch(outputs->charge_mode) 
    {
      case CHARGE_MODE_CONSTANT_CURRENT:
        send_string_p(PSTR(" Csp "));
        send_uint16(settings->current_cc);
        send_string_p(PSTR(" Vsp "));
        send_uint16(settings->voltage_cc);
        send_string_p(PSTR(" Psp "));
        send_uint16(0);
        send_string_p(PSTR(" M CC"));
        break;
      case CHARGE_MODE_CONSTANT_VOLTAGE:
        send_string_p(PSTR(" Csp "));
        send_uint16(settings->current_cv);
        send_string_p(PSTR(" Vsp "));
        send_uint16(settings->voltage_cv);
        send_string_p(PSTR(" Psp "));
        send_uint16(0);
        send_string_p(PSTR(" M CV"));
        break;
      case CHARGE_MODE_BALANCE  :
        send_string_p(PSTR(" Csp "));
        send_uint16(settings->current_balancing);
        send_string_p(PSTR(" Vsp "));
        send_uint16(settings->voltage_balancing);
        send_string_p(PSTR(" Psp "));
        send_uint16(0);
        send_string_p(PSTR(" M Bal"));
        break;
      case CHARGE_MODE_ABSORBTION:
        send_string_p(PSTR(" Csp "));
        send_uint16(settings->current_balancing);
        send_string_p(PSTR(" Vsp "));
        send_uint16(settings->voltage_balancing);
        send_string_p(PSTR(" Psp "));
        send_uint16(0);
        send_string_p(PSTR(" M Bal"));
        break;
      case CHARGE_MODE_RETRY_WAIT:
        send_string_p(PSTR(" Csp "));
        send_uint16(0);
        send_string_p(PSTR(" Vsp "));
        send_uint16(0);
        send_string_p(PSTR(" Psp "));
        send_uint16(0);
        send_string_p(PSTR(" W "));
        send_uint32_half((outputs->time_wait_retry - timestamp) & 0xFFFFFFFF);
        send_string_p(PSTR(" M Retrying num. "));
        send_uint8(outputs->charge_retries);
        break;
      case CHARGE_MODE_OFF:
        send_string_p(PSTR(" Csp "));
        send_uint16(0);
        send_string_p(PSTR(" Vsp "));
        send_uint16(0);
        send_string_p(PSTR(" Psp "));
        send_uint16(0);
        send_string_p(PSTR(" M Off"));
        break;
      case CHARGE_MODE_OFF_ERROR:
        send_string_p(PSTR(" Csp "));
        send_uint16(0);
        send_string_p(PSTR(" Vsp "));
        send_uint16(0);
        send_string_p(PSTR(" Psp "));
        send_uint16(0);
        send_string_p(PSTR(" M Off Error"));
        break;
      default:
        send_string_p(PSTR(" Csp "));
        send_uint16(0);
        send_string_p(PSTR(" Vsp "));
        send_uint16(0);
        send_string_p(PSTR(" Psp "));
        send_uint16(0);
        send_string_p(PSTR(" M ??"));
        break;        
    }    
    send_newline();
  }
}

// enable logging
void enable_logging(void){
  logenable = 1;
}

// disable logging
void disable_logging(void){
  logenable = 0;
}
