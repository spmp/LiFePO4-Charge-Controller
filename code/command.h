/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file command.h
 *
 * @brief Header for command.c, application specific commands to handle USART input strings
 *
 * - Compiler:          gcc-avr
 * - Project:           LiFePO4 Charge Controller
 * - uC                 AVR Mega328p on Arduino Nano board
 * 
 * @author              Jasper Aorangi
 * @Date: July 2014
 *****************************************************************************/
#pragma once
#include <avr/io.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "AVR-lib/usart.h"
#include "AVR-lib/timers.h"
#include "hardware.h"
#include "log.h"
#include "process-control.h"

/* Key presses for serial interface */
#define USART_KEY_LEVEL l
#define USART_KEY_VOLUME v
#define USART_KEY_LOGGING_EN L
#define USART_KEY_STATEMACHINE_EN S
#define USART_KEY_FILL_EN F
#define USART_KEY_HEAT_EN H
#define USART_KEY_PUMP_EN P
#define USART_KEY_TIME t
#define USART_KEY_TEMPERATURE T
#define USART_KEY_FILL f
#define USART_KEY_BOOST b
#define USART_KEY_TIMETOHOT1 Y
#define USART_KEY_TEMPTOHOT y
#define USART_KEY_TIMETOHOT2 U
#define USART_KEY_TEMPTOHOT2 u
#define USART_KEY_MAXLEVEL M
#define USART_KEY_MINLEVEL m
#define USART_KEY_HEATERLEVEL J
#define USART_KEY_FILLLEVEL N
#define USART_KEY_FILLLEVELLITRE n
#define USART_KEY_MAXTEMP G
#define USART_KEY_MINTEMP g
#define USART_KEY_MIDSUN s
#define USART_KEY_DHP d
#define USART_KEY_ZERO z
#define USART_KEY_PROGRAM_CONFIG O
#define USART_KEY_PROGrAM_RUN o
#define USART_KEY_CONFIG_DUMP C
/* Parse the recieved line into command name and value */
void handle_line(const char* line);

/* Take action on command name and command value */
void command_from_serial(char commandname, uint32_t commandvalue, struct Process *process);


