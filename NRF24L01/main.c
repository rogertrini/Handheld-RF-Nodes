// This program is to test the RX and TX using NRF24L01 Modules using AVR C
// We will create the learning stack and register level controls for spi modules

#include <avr/io.h> // gives register definitions for AVR microcontrollers
#include <util/delay.h> 
#include <avr/interrupt.h>
#include <stdint.h> // this is for professional fixed-size types
#include <stdbool.h> // allows us to use bool type for True/False

// User defined libraries
#include "spi.h"
#include "nrf24.h"


