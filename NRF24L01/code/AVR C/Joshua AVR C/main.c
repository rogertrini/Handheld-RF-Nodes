/*
 * main.c
 * Purpose: AVR C firmware for the JOSH1 handheld RF node.
 * Target: ATmega328P at 16 MHz.
 * Project: CPE 301 Handheld RF Communication System.
 */

#define F_CPU 16000000UL
#define DEVICE_NAME "JOSH NRF"
#define DEVICE_ID 1
#define THIS_ADDR "JOSH1"
#define PEER_ADDR "ROGER"

#define BUTTON1_PIN PD2
#define BUTTON2_PIN PD3
#define BUTTON3_PIN PD4
#define BUTTON4_PIN PD5
#define BUTTON_DDR DDRD
#define BUTTON_PORT PORTD
#define BUTTON_PINREG PIND

#define LED_DDR DDRB
#define LED_PORT PORTB
#define LED_PIN PB0

#define BUZZER_DDR DDRC
#define BUZZER_PORT PORTC
#define BUZZER_PIN PC0
#define BUZZER_ACTIVE_HIGH 1

#include "rf_node_common.c"
