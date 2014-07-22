#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "hardware.h"

// #include "state-machine.h"
#include "command.h"
#include "log.h"

uint8_t send_log = 0;

void once_per_second() {
    send_log = 1;
    PORTB ^= (1 << PORTB5); //Toggle LED
}

// void medium_timestep() {
//     begin_state_machine_flag = 1;
// }


int main() {
    cli();
    set_sleep_mode(SLEEP_MODE_IDLE);
    init_hardware();
    
    //Set timer callbacks
    clock_set_seconds_callback(&once_per_second);
//     clock_set_medium_time_callback(&medium_timestep);
    
    //USART line handler
    usart_set_handle_char_string_from_serial(&handle_line);
    
    sei();

//     struct Program program;

    for (;;) {
        sleep_mode(); // blocked until after an interrupt has fired
        
        //USART
        while (num_in_serial_buffer()) { //check whether there is anything on the serial buffer, if there is, look at it
            handle_single_char_from_serial();
        }
        
//         //State machine 
//         if ( begin_state_machine_flag ) {
//             begin_state_machine_flag = 0;
//             state_machine(&program[state_machine_program]);
//             state_machine(&program);
        }
        
        //Logging
        if (send_log) {
            send_log = 0;
            log_to_serial(&program[state_machine_program]);
//             log_to_serial(&program);
        }
    }

    return 0;
}

