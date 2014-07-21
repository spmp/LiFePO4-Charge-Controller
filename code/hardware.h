#pragma once
#include <avr/io.h>

/* Header to bring together all hardware related functions and routines, set
 * things and stuff like that.
 */

/* Hardware specific defines: */
#define ONBOARD_LED     PB5


#define OUTPUT_PORT    PORTD
#define INPUT_PORT      PIND
#define DIRECTION_REG   DDRD
#define PUMP_PIN        PD3
#define FILL_PIN        PD7
#define HEAT1_PIN       PD5
#define HEAT2_PIN       PD6

// define electric connections according to your circuit, DATA line 
// From: http://www.avrfreaks.net/index.php?name=PNphpBB2&file=viewtopic&t=73829
// Credit to: Tomasz Ostrowski
#define DATA_PORT      PORTD
#define DATA_DDR      DDRD
#define DATA_PINPORT   PINB 
#define DATA_PIN      0 
#define SWITCH_DATA_IN  DATA_DDR &= ~_BV(DATA_PIN) 
#define SWITCH_DATA_OUT DATA_DDR |= _BV(DATA_PIN); NOP   ///TODO is NOP needed? 
#define CLEAR_DATA      DATA_PORT &= ~_BV(DATA_PIN) 
#define SET_DATA        DATA_PORT |= _BV(DATA_PIN) 
#define DATA            (DATA_PINPORT & _BV(DATA_PIN))
#define TOGGLE_DATA     DATA_PORT ^= _BV(DATA_PIN)

/*
#define OUTPUT_PORT     PORTB
#define INPUT_PORT      PINB
#define DIRECTION_REG   DDRB
#define PUMP_PIN        PB2
#define FILL_PIN        PB1
#define HEAT1_PIN       PB3
#define HEAT2_PIN       PB4
*/

/* Heater size */
#define HEATER_SIZE     3000L
#define SPECIFIC_HEAT_WATER 4186L

/* AVR specifics */
#include "i2c_safe.h"
#include "usart.h"
#include "clock.h"

/* Inputs */
#include "temperature.h"        //Temperature reading
#include "level.h"              //Level and volume reading

/* Initialise all hardware relevant to the project, usart, i2c, clock, external
 * sensors, Everyting!
 * One call to rule them all */
void init_hardware(void);

/* Initialise I/O ports on the AVR */
void init_io_ports(void);

/* Hardware states */
uint8_t pump_state(void);
uint8_t fill_state(void);
uint8_t heater_state(void);

/* Set hardware states */
void pump_set(uint8_t state);
void fill_set(uint8_t state);
void heater_set(uint8_t state);
