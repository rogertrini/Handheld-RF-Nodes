/*
 * main.c
 * Purpose: AVR C firmware for the ROGER handheld RF node.
 * Target: ATmega328P at 16 MHz.
 * Project: CPE 301 Handheld RF Communication System.
 */

#define F_CPU 16000000UL
#define DEVICE_NAME "ROGER NRF"
#define DEVICE_ID 2
#define THIS_ADDR "ROGER"
#define PEER_ADDR "JOSH1"

#define BUTTON1_PIN PC0
#define BUTTON2_PIN PC1
#define BUTTON3_PIN PC2
#define BUTTON4_PIN PC3
#define BUTTON_DDR DDRC
#define BUTTON_PORT PORTC
#define BUTTON_PINREG PINC

#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_PIN PB0

#define BUZZER_DDR DDRD
#define BUZZER_PORT PORTD
#define BUZZER_PIN PD3
#define BUZZER_ACTIVE_HIGH 1

#include "rf_node_common.c"
