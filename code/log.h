/*This file has been prepared for Doxygen automatic documentation generation.*/
/** 
 * @file log.h
 *
 * @brief Header for log.c, application specific logging to the hardware USART
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
#include "AVR-lib/usart.h"
#include "AVR-lib/clock.h"
#include "process-control.h"

#ifndef LOG_ENABLE
#define LOG_ENABLE 1 //1 log enabled, 0 log disabled
#endif

/* Logging bit */

// Send logging data over serial
void log_to_serial(struct Process *process);

// enable logging
void enable_logging(void);

// disable logging
void disable_logging(void);

