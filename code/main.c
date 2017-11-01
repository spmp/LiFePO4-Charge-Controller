#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "hardware.h"
#include "AVR-lib/wd.h"
#include "process-control.h"
#include "command.h"
#include "log.h"
#include "AVR-lib/usart.h"

#define VERSION "0.5.2"

// Watchdog and reset state.
// "You're not suppose to call get_mcusr() in main().
// "the attribute section(".init3") puts the code in before main() so it runs automatically before entering main().
// NOTE: This code will work with a recent version of 'optiboot' as the bootloader:
uint8_t resetFlags __attribute__ ((section(".noinit")));
void resetFlagsInit(void) __attribute__ ((naked)) __attribute__ ((section (".init0")));
void resetFlagsInit(void)
{
  // save the reset flags passed from the bootloader
  __asm__ __volatile__ ("mov %0, r2\n" : "=r" (resetFlags) :);
}


/**
 * @var send_log, whether to log to USART or not
 **/
uint8_t send_log = 0;

/**
 * @brief function to be called from the once per second clock ISR
 * 
 * Activates the log send
 * Toggles the LED
 **/
void once_per_second() {
  begin_process_control_flag = 1;
  send_log = 1;
  led_onboard_toggle(); //Toggle LED
}

// #undef MEDIUM_TIME_INTERVAL
// #define MEDIUM_TIME_INTERVAL 15
void medium_timestep() {
  //begin_process_control_flag = 1;
}

// void fast_timestep() {
//   send_log = 1;
//   begin_process_control_flag = 1;
// }

int main() {
  cli();
  set_sleep_mode(SLEEP_MODE_IDLE);
  init_hardware();
  //     init_PID(&process);
  sei();
  
  //Send initialisation message:
  // TODO: This will show how many resets... after the R
  send_string_p(PSTR("Rx LiFePO4 Charge Controller for ESP-120 PSU's. Version "VERSION"\r\nJasper Aorangi 2017. Have a nice day 8): "));
  send_uint16(resetFlags);
  send_string_p(PSTR(" x\r\n"));
  
  //Set timer callbacks
  clock_set_seconds_callback(&once_per_second);
  clock_set_medium_time_callback(&medium_timestep);
  //     clock_set_fast_time_callback(&fast_timestep);
  
  //USART line handler
  usart_set_handle_char_string_from_serial(&handle_line);
  
  
  //Pre Process Control Wait and checks -- Cannot wait more than Watchdog reset
  // Wait `PSU_WAIT_ON` seconds before starting the process to give the PSU's
  // time to stabilise
  uint8_t i;
  for (i=0; i <= PSU_WAIT_ON; i++)
  {
    // Delay 1s 
    _delay_ms(1000);             
    wd_reset();
  }
  process_control_enable();           //Enable Process Control
  
  for (;;) {
    sleep_mode(); // blocked until after an interrupt has fired
    
    //USART
    while (num_in_serial_buffer())
    { //check whether there is anything on the serial buffer, if there is, look at it
      handle_single_char_from_serial();
    }
    
    //Process Controll
    if ( begin_process_control_flag )
    {
      begin_process_control_flag = 0;
      process_control(&process);
    }
    
    //Logging
    if (send_log)
    {
      send_log = 0;
      log_to_serial(&process);
    }
    wd_reset();
  }
  
  return 0;
}

