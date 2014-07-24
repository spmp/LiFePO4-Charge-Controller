#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "hardware.h"
#include "process-control.h"
#include "command.h"
#include "log.h"

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
    send_log = 1;
    PORTB ^= (1 << PORTB5); //Toggle LED
}

#undef MEDIUM_TIME_INTERVAL
#define MEDIUM_TIME_INTERVAL 15
void medium_timestep() {
//     send_log = 1;
    begin_process_control_flag = 1;
}

void fast_timestep() {
    send_log = 1;
    begin_process_control_flag = 1;
}

int main() {
    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    init_hardware();
    init_PID(&process);
    
    //Set timer callbacks
    clock_set_seconds_callback(&once_per_second);
    clock_set_medium_time_callback(&medium_timestep);
//     clock_set_fast_time_callback(&fast_timestep);
    
    //USART line handler
    usart_set_handle_char_string_from_serial(&handle_line);
    
    sei();
    
    for (;;) {
        sleep_mode(); // blocked until after an interrupt has fired
        
        //USART
        while (num_in_serial_buffer()) { //check whether there is anything on the serial buffer, if there is, look at it
            handle_single_char_from_serial();
        }
        
        //Process Controll
        if ( begin_process_control_flag ) {
            begin_process_control_flag = 0;
            process_control(&process);
        }
        
        //Logging
        if (send_log) {
            send_log = 0;
            log_to_serial(&process);
        }
    }

    return 0;
}

