#include "hardware.h"

/**
 * @brief Initialise hardware 
 * 
 * Initialise all hardware relevant to the project, usart, i2c, clock, external
 * sensors, Everyting!
 * One call to rule them all
 */
void init_hardware(void ) {
    init_clock();
    init_usart(USART_BAUDE, F_CPU);
    i2c_init(I2C_FREQ,F_CPU);
}
