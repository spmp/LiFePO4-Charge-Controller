#pragma once
#include <avr/io.h>
#include "usart.h"
#include "clock.h"
#include "state-machine.h"

#ifndef LOG_ENABLE
#define LOG_ENABLE 1 //1 log enabled, 0 log disabled
#endif

/* Logging bit */

// Send logging data over serial
void log_to_serial(struct Program *program);

// enable logging
void enable_logging(void);

// disable logging
void disable_logging(void);

