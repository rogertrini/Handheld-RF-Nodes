/*
 * rf_node_common.c
 * Purpose: Shared AVR C code for the handheld nRF24L01 RF nodes.
 * Notes: This file is included by each node's main.c after pin/address macros.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define CE_DDR DDRB
#define CE_PORT PORTB
#define CE_PIN PB1          /* Arduino D9 */
#define CSN_DDR DDRB
#define CSN_PORT PORTB
#define CSN_PIN PB2         /* Arduino D10 */
#define SPI_DDR DDRB
#define SPI_PORT PORTB
#define MOSI_PIN PB3        /* Arduino D11 */
#define MISO_PIN PB4        /* Arduino D12 */
#define SCK_PIN PB5         /* Arduino D13 */

#define OLED_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define NRF_CONFIG 0x00
#define NRF_EN_AA 0x01
#define NRF_EN_RXADDR 0x02
#define NRF_SETUP_AW 0x03
#define NRF_SETUP_RETR 0x04
#define NRF_RF_CH 0x05
#define NRF_RF_SETUP 0x06
#define NRF_STATUS 0x07
#define NRF_RX_ADDR_P1 0x0B
#define NRF_TX_ADDR 0x10
#define NRF_RX_PW_P1 0x12
#define NRF_FIFO_STATUS 0x17
#define NRF_FLUSH_TX 0xE1
#define NRF_FLUSH_RX 0xE2
#define NRF_R_RX_PAYLOAD 0x61
#define NRF_W_TX_PAYLOAD 0xA0
#define NRF_W_REGISTER 0x20
#define NRF_NOP 0xFF

#define NRF_MASK_RX_DR 6
#define NRF_MASK_TX_DS 5
#define NRF_MASK_MAX_RT 4
#define NRF_PRIM_RX 0
#define NRF_PWR_UP 1
#define NRF_EN_CRC 3

#define PACKET_SIZE 6
#define MESSAGE_COUNT 6
#define SOS_INDEX 5
#define DEBOUNCE_MS 160UL
#define RX_INDICATOR_MS 350UL
#define SOS_FLASH_MS 3000UL
#define SOS_FLASH_STEP_MS 120UL
#define BUZZ_MS 180UL

static const char *messages[MESSAGE_COUNT] = {"HI", "BYE", "YES", "NO", "CLEAR", "SOS"};

#pragma pack(push, 1)
typedef struct
{
    uint8_t messageIndex;
    uint8_t senderId;
    uint32_t counter;
} Packet;
#pragma pack(pop)

static volatile uint32_t g_ms = 0;
static uint8_t currentIndex = 0;
static int8_t receivedIndex = -1;
static int8_t lastSentIndex = -1;
static uint32_t txCounter = 0;
static uint32_t lastButtonMs = 0;
static uint32_t rxIndicatorUntil = 0;
static uint32_t buzzerUntil = 0;
static uint32_t sosFlashUntil = 0;
static uint32_t lastSosFlashMs = 0;
static bool sosLedState = false;
static bool lastB1 = true;
static bool lastB2 = true;
static bool lastB3 = true;
static bool lastB4 = true;

/*
 * Function: timer0_init
 * Purpose: Creates a 1 ms system tick using Timer0.
 * Parameters: None.
 * Return: None.
 */
static void timer0_init(void)
{
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 249;
    TIMSK0 = (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect)
{
    g_ms++;
}

/*
 * Function: millis
 * Purpose: Returns elapsed milliseconds since startup.
 * Parameters: None.
 * Return: Millisecond counter.
 */
static uint32_t millis(void)
{
    uint32_t value;
    uint8_t sreg = SREG;
    cli();
    value = g_ms;
    SREG = sreg;
    return value;
}

/*
 * Function: uart_init
 * Purpose: Initializes UART at 9600 baud.
 * Parameters: None.
 * Return: None.
 */
static void uart_init(void)
{
    UBRR0H = 0;
    UBRR0L = 103;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

/*
 * Function: uart_tx
 * Purpose: Sends one character over UART.
 * Parameters: c - character to send.
 * Return: None.
 */
static void uart_tx(char c)
{
    while (!(UCSR0A & (1 << UDRE0))) {}
    UDR0 = c;
}

/*
 * Function: uart_print
 * Purpose: Sends a null-terminated string over UART.
 * Parameters: s - string pointer.
 * Return: None.
 */
static void uart_print(const char *s)
{
    while (*s != '\0')
    {
        uart_tx(*s++);
    }
}

/*
 * Function: uart_print_u32
 * Purpose: Sends an unsigned integer over UART.
 * Parameters: value - integer to print.
 * Return: None.
 */
static void uart_print_u32(uint32_t value)
{
    char buf[11];
    uint8_t i = 0;

    if (value == 0)
    {
        uart_tx('0');
        return;
    }

    while (value > 0 && i < sizeof(buf))
    {
        buf[i++] = (char)('0' + (value % 10));
        value /= 10;
    }

    while (i > 0)
    {
        uart_tx(buf[--i]);
    }
}

/*
 * Function: gpio_init
 * Purpose: Configures buttons, LED, and buzzer pins.
 * Parameters: None.
 * Return: None.
 */
static void gpio_init(void)
{
    BUTTON_DDR &= ~((1 << BUTTON1_PIN) | (1 << BUTTON2_PIN) | (1 << BUTTON3_PIN) | (1 << BUTTON4_PIN));
    BUTTON_PORT |= (1 << BUTTON1_PIN) | (1 << BUTTON2_PIN) | (1 << BUTTON3_PIN) | (1 << BUTTON4_PIN);

    LED_DDR |= (1 << LED_PIN);
    LED_PORT &= ~(1 << LED_PIN);

    BUZZER_DDR |= (1 << BUZZER_PIN);
    BUZZER_PORT &= ~(1 << BUZZER_PIN);
}

/*
 * Function: led_write
 * Purpose: Sets the red LED state.
 * Parameters: on - true turns LED on.
 * Return: None.
 */
static void led_write(bool on)
{
    if (on)
    {
        LED_PORT |= (1 << LED_PIN);
    }
    else
    {
        LED_PORT &= ~(1 << LED_PIN);
    }
}

/*
 * Function: buzzer_write
 * Purpose: Sets the buzzer pin state.
 * Parameters: on - true turns buzzer on.
 * Return: None.
 */
static void buzzer_write(bool on)
{
    if (on)
    {
        BUZZER_PORT |= (1 << BUZZER_PIN);
    }
    else
    {
        BUZZER_PORT &= ~(1 << BUZZER_PIN);
    }
}

/*
 * Function: spi_init
 * Purpose: Initializes AVR hardware SPI as master.
 * Parameters: None.
 * Return: None.
 */
static void spi_init(void)
{
    SPI_DDR |= (1 << MOSI_PIN) | (1 << SCK_PIN) | (1 << CSN_PIN) | (1 << CE_PIN);
    SPI_DDR &= ~(1 << MISO_PIN);
    CSN_PORT |= (1 << CSN_PIN);
    CE_PORT &= ~(1 << CE_PIN);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
}

/*
 * Function: spi_transfer
 * Purpose: Transfers one byte over SPI.
 * Parameters: data - byte to send.
 * Return: Byte received.
 */
static uint8_t spi_transfer(uint8_t data)
{
    SPDR = data;
    while (!(SPSR & (1 << SPIF))) {}
    return SPDR;
}

static void nrf_csn_low(void) { CSN_PORT &= ~(1 << CSN_PIN); }
static void nrf_csn_high(void) { CSN_PORT |= (1 << CSN_PIN); }
static void nrf_ce_low(void) { CE_PORT &= ~(1 << CE_PIN); }
static void nrf_ce_high(void) { CE_PORT |= (1 << CE_PIN); }

/*
 * Function: nrf_write_reg
 * Purpose: Writes one nRF24L01 register.
 * Parameters: reg - register address, value - data byte.
 * Return: None.
 */
static void nrf_write_reg(uint8_t reg, uint8_t value)
{
    nrf_csn_low();
    spi_transfer(NRF_W_REGISTER | reg);
    spi_transfer(value);
    nrf_csn_high();
}

/*
 * Function: nrf_read_reg
 * Purpose: Reads one nRF24L01 register.
 * Parameters: reg - register address.
 * Return: Register value.
 */
static uint8_t nrf_read_reg(uint8_t reg)
{
    uint8_t value;
    nrf_csn_low();
    spi_transfer(reg);
    value = spi_transfer(NRF_NOP);
    nrf_csn_high();
    return value;
}

/*
 * Function: nrf_write_buf
 * Purpose: Writes multiple bytes to an nRF command/register.
 * Parameters: command - SPI command, data - buffer, len - byte count.
 * Return: None.
 */
static void nrf_write_buf(uint8_t command, const uint8_t *data, uint8_t len)
{
    uint8_t i;
    nrf_csn_low();
    spi_transfer(command);
    for (i = 0; i < len; i++)
    {
        spi_transfer(data[i]);
    }
    nrf_csn_high();
}

/*
 * Function: nrf_read_buf
 * Purpose: Reads multiple bytes from an nRF command/register.
 * Parameters: command - SPI command, data - buffer, len - byte count.
 * Return: None.
 */
static void nrf_read_buf(uint8_t command, uint8_t *data, uint8_t len)
{
    uint8_t i;
    nrf_csn_low();
    spi_transfer(command);
    for (i = 0; i < len; i++)
    {
        data[i] = spi_transfer(NRF_NOP);
    }
    nrf_csn_high();
}

/*
 * Function: nrf_command
 * Purpose: Sends a standalone nRF command.
 * Parameters: command - command byte.
 * Return: None.
 */
static void nrf_command(uint8_t command)
{
    nrf_csn_low();
    spi_transfer(command);
    nrf_csn_high();
}

/*
 * Function: nrf_rx_mode
 * Purpose: Places nRF24L01 into receive mode.
 * Parameters: None.
 * Return: None.
 */
static void nrf_rx_mode(void)
{
    nrf_ce_low();
    nrf_write_reg(NRF_CONFIG, (1 << NRF_EN_CRC) | (1 << NRF_PWR_UP) | (1 << NRF_PRIM_RX));
    _delay_us(150);
    nrf_ce_high();
}

/*
 * Function: nrf_init
 * Purpose: Initializes nRF24L01 addresses, channel, payload, and RX mode.
 * Parameters: None.
 * Return: None.
 */
static void nrf_init(void)
{
    const uint8_t thisAddress[5] = THIS_ADDR;
    const uint8_t peerAddress[5] = PEER_ADDR;

    nrf_ce_low();
    _delay_ms(100);
    nrf_write_reg(NRF_CONFIG, (1 << NRF_EN_CRC) | (1 << NRF_PWR_UP));
    nrf_write_reg(NRF_EN_AA, 0x02);
    nrf_write_reg(NRF_EN_RXADDR, 0x02);
    nrf_write_reg(NRF_SETUP_AW, 0x03);
    nrf_write_reg(NRF_SETUP_RETR, 0x5F);
    nrf_write_reg(NRF_RF_CH, 108);
    nrf_write_reg(NRF_RF_SETUP, 0x06);
    nrf_write_reg(NRF_RX_PW_P1, PACKET_SIZE);
    nrf_write_buf(NRF_W_REGISTER | NRF_RX_ADDR_P1, thisAddress, 5);
    nrf_write_buf(NRF_W_REGISTER | NRF_TX_ADDR, peerAddress, 5);
    nrf_write_reg(NRF_STATUS, 0x70);
    nrf_command(NRF_FLUSH_RX);
    nrf_command(NRF_FLUSH_TX);
    nrf_rx_mode();
}

/*
 * Function: nrf_available
 * Purpose: Checks for received RF payloads.
 * Parameters: None.
 * Return: true if RX data is ready.
 */
static bool nrf_available(void)
{
    return (nrf_read_reg(NRF_STATUS) & (1 << NRF_MASK_RX_DR)) != 0;
}

/*
 * Function: nrf_read_packet
 * Purpose: Reads one received RF packet.
 * Parameters: packet - output packet pointer.
 * Return: None.
 */
static void nrf_read_packet(Packet *packet)
{
    nrf_read_buf(NRF_R_RX_PAYLOAD, (uint8_t *)packet, PACKET_SIZE);
    nrf_write_reg(NRF_STATUS, (1 << NRF_MASK_RX_DR));
}

/*
 * Function: nrf_send_packet
 * Purpose: Sends one RF packet and waits for success/failure.
 * Parameters: packet - input packet pointer.
 * Return: true if packet was acknowledged.
 */
static bool nrf_send_packet(const Packet *packet)
{
    uint8_t status;
    uint32_t start;

    nrf_ce_low();
    nrf_write_reg(NRF_CONFIG, (1 << NRF_EN_CRC) | (1 << NRF_PWR_UP));
    _delay_us(150);
    nrf_write_reg(NRF_STATUS, 0x70);
    nrf_command(NRF_FLUSH_TX);
    nrf_write_buf(NRF_W_TX_PAYLOAD, (const uint8_t *)packet, PACKET_SIZE);

    nrf_ce_high();
    _delay_us(15);
    nrf_ce_low();

    start = millis();
    do
    {
        status = nrf_read_reg(NRF_STATUS);
        if (status & (1 << NRF_MASK_TX_DS))
        {
            nrf_write_reg(NRF_STATUS, (1 << NRF_MASK_TX_DS));
            nrf_rx_mode();
            return true;
        }
        if (status & (1 << NRF_MASK_MAX_RT))
        {
            nrf_write_reg(NRF_STATUS, (1 << NRF_MASK_MAX_RT));
            nrf_command(NRF_FLUSH_TX);
            nrf_rx_mode();
            return false;
        }
    } while ((millis() - start) < 100);

    nrf_rx_mode();
    return false;
}

/*
 * Function: twi_init
 * Purpose: Initializes AVR TWI/I2C at about 100 kHz.
 * Parameters: None.
 * Return: None.
 */
static void twi_init(void)
{
    TWSR = 0x00;
    TWBR = 72;
    TWCR = (1 << TWEN);
}

/*
 * Function: twi_start
 * Purpose: Sends an I2C START and address.
 * Parameters: addr - 7-bit address plus R/W bit.
 * Return: None.
 */
static void twi_start(uint8_t addr)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {}
    TWDR = addr;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {}
}

/*
 * Function: twi_write
 * Purpose: Writes one byte on I2C.
 * Parameters: data - byte to write.
 * Return: None.
 */
static void twi_write(uint8_t data)
{
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT))) {}
}

/*
 * Function: twi_stop
 * Purpose: Sends an I2C STOP condition.
 * Parameters: None.
 * Return: None.
 */
static void twi_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

/*
 * Function: oled_command
 * Purpose: Sends one SSD1306 command byte.
 * Parameters: cmd - command byte.
 * Return: None.
 */
static void oled_command(uint8_t cmd)
{
    twi_start(OLED_ADDR << 1);
    twi_write(0x00);
    twi_write(cmd);
    twi_stop();
}

/*
 * Function: oled_data
 * Purpose: Sends one SSD1306 data byte.
 * Parameters: data - pixel data byte.
 * Return: None.
 */
static void oled_data(uint8_t data)
{
    twi_start(OLED_ADDR << 1);
    twi_write(0x40);
    twi_write(data);
    twi_stop();
}

/*
 * Function: oled_init
 * Purpose: Initializes the 128x64 SSD1306 OLED.
 * Parameters: None.
 * Return: None.
 */
static void oled_init(void)
{
    _delay_ms(50);
    oled_command(0xAE);
    oled_command(0xD5); oled_command(0x80);
    oled_command(0xA8); oled_command(0x3F);
    oled_command(0xD3); oled_command(0x00);
    oled_command(0x40);
    oled_command(0x8D); oled_command(0x14);
    oled_command(0x20); oled_command(0x00);
    oled_command(0xA1);
    oled_command(0xC8);
    oled_command(0xDA); oled_command(0x12);
    oled_command(0x81); oled_command(0xCF);
    oled_command(0xD9); oled_command(0xF1);
    oled_command(0xDB); oled_command(0x40);
    oled_command(0xA4);
    oled_command(0xA6);
    oled_command(0xAF);
}

/*
 * Function: oled_clear
 * Purpose: Clears the OLED display.
 * Parameters: None.
 * Return: None.
 */
static void oled_clear(void)
{
    uint16_t i;
    oled_command(0x21); oled_command(0); oled_command(127);
    oled_command(0x22); oled_command(0); oled_command(7);
    for (i = 0; i < 1024; i++)
    {
        oled_data(0x00);
    }
}

/*
 * Function: oled_set_cursor
 * Purpose: Sets text cursor page and column.
 * Parameters: col - column, page - display page.
 * Return: None.
 */
static void oled_set_cursor(uint8_t col, uint8_t page)
{
    oled_command(0x21); oled_command(col); oled_command(127);
    oled_command(0x22); oled_command(page); oled_command(7);
}

/*
 * Function: font_get
 * Purpose: Returns 5x7 font columns for needed characters.
 * Parameters: c - ASCII character, out - 5 byte output array.
 * Return: None.
 */
static void font_get(char c, uint8_t out[5])
{
    uint8_t blank[5] = {0, 0, 0, 0, 0};
    uint8_t data[5];
    memcpy(data, blank, 5);

    switch (c)
    {
        case 'A': { uint8_t t[5]={0x7E,0x11,0x11,0x11,0x7E}; memcpy(data,t,5); break; }
        case 'B': { uint8_t t[5]={0x7F,0x49,0x49,0x49,0x36}; memcpy(data,t,5); break; }
        case 'C': { uint8_t t[5]={0x3E,0x41,0x41,0x41,0x22}; memcpy(data,t,5); break; }
        case 'D': { uint8_t t[5]={0x7F,0x41,0x41,0x22,0x1C}; memcpy(data,t,5); break; }
        case 'E': { uint8_t t[5]={0x7F,0x49,0x49,0x49,0x41}; memcpy(data,t,5); break; }
        case 'F': { uint8_t t[5]={0x7F,0x09,0x09,0x09,0x01}; memcpy(data,t,5); break; }
        case 'G': { uint8_t t[5]={0x3E,0x41,0x49,0x49,0x7A}; memcpy(data,t,5); break; }
        case 'H': { uint8_t t[5]={0x7F,0x08,0x08,0x08,0x7F}; memcpy(data,t,5); break; }
        case 'I': { uint8_t t[5]={0x00,0x41,0x7F,0x41,0x00}; memcpy(data,t,5); break; }
        case 'J': { uint8_t t[5]={0x20,0x40,0x41,0x3F,0x01}; memcpy(data,t,5); break; }
        case 'L': { uint8_t t[5]={0x7F,0x40,0x40,0x40,0x40}; memcpy(data,t,5); break; }
        case 'N': { uint8_t t[5]={0x7F,0x02,0x04,0x08,0x7F}; memcpy(data,t,5); break; }
        case 'O': { uint8_t t[5]={0x3E,0x41,0x41,0x41,0x3E}; memcpy(data,t,5); break; }
        case 'P': { uint8_t t[5]={0x7F,0x09,0x09,0x09,0x06}; memcpy(data,t,5); break; }
        case 'R': { uint8_t t[5]={0x7F,0x09,0x19,0x29,0x46}; memcpy(data,t,5); break; }
        case 'S': { uint8_t t[5]={0x46,0x49,0x49,0x49,0x31}; memcpy(data,t,5); break; }
        case 'T': { uint8_t t[5]={0x01,0x01,0x7F,0x01,0x01}; memcpy(data,t,5); break; }
        case 'V': { uint8_t t[5]={0x1F,0x20,0x40,0x20,0x1F}; memcpy(data,t,5); break; }
        case 'X': { uint8_t t[5]={0x63,0x14,0x08,0x14,0x63}; memcpy(data,t,5); break; }
        case 'Y': { uint8_t t[5]={0x07,0x08,0x70,0x08,0x07}; memcpy(data,t,5); break; }
        case ':': { uint8_t t[5]={0x00,0x36,0x36,0x00,0x00}; memcpy(data,t,5); break; }
        case '1': { uint8_t t[5]={0x00,0x42,0x7F,0x40,0x00}; memcpy(data,t,5); break; }
        default: break;
    }

    memcpy(out, data, 5);
}

/*
 * Function: oled_char
 * Purpose: Draws one 5x7 character.
 * Parameters: c - character, scale - vertical scale of 1 or 2.
 * Return: None.
 */
static void oled_char(char c, uint8_t scale)
{
    uint8_t cols[5];
    uint8_t i;
    font_get(c, cols);

    for (i = 0; i < 5; i++)
    {
        oled_data(cols[i]);
        if (scale > 1)
        {
            oled_data(cols[i]);
        }
    }
    oled_data(0x00);
}

/*
 * Function: oled_print
 * Purpose: Prints a string on the OLED.
 * Parameters: s - string, scale - simple text scale.
 * Return: None.
 */
static void oled_print(const char *s, uint8_t scale)
{
    while (*s != '\0')
    {
        oled_char(*s++, scale);
    }
}

/*
 * Function: oled_line
 * Purpose: Draws a horizontal separator line.
 * Parameters: page - OLED page to draw on.
 * Return: None.
 */
static void oled_line(uint8_t page)
{
    uint8_t i;
    oled_set_cursor(0, page);
    for (i = 0; i < 128; i++)
    {
        oled_data(0x01);
    }
}

/*
 * Function: draw_screen
 * Purpose: Updates OLED with TX, sent, and received message status.
 * Parameters: None.
 * Return: None.
 */
static void draw_screen(void)
{
    const char *rxText = (receivedIndex < 0) ? "NONE" : messages[(uint8_t)receivedIndex];
    uint8_t rxLen = (uint8_t)strlen(rxText);
    uint8_t x = (uint8_t)((128 - (rxLen * 12)) / 2);

    oled_clear();
    oled_set_cursor(0, 0);
    oled_print(DEVICE_NAME, 1);

    oled_set_cursor(0, 1);
    oled_print("TX:", 1);
    oled_print(messages[currentIndex], 1);

    if (lastSentIndex >= 0)
    {
        oled_set_cursor(58, 1);
        oled_print("S:", 1);
        oled_print(messages[(uint8_t)lastSentIndex], 1);
    }

    oled_line(2);
    oled_set_cursor(0, 3);
    oled_print("RECEIVED", 1);
    oled_set_cursor(x, 5);
    oled_print(rxText, 2);
}

/*
 * Function: draw_status
 * Purpose: Displays a single status message.
 * Parameters: status - message to display.
 * Return: None.
 */
static void draw_status(const char *status)
{
    oled_clear();
    oled_set_cursor(0, 0);
    oled_print(status, 1);
}

/*
 * Function: previous_message
 * Purpose: Selects the previous TX message.
 * Parameters: None.
 * Return: None.
 */
static void previous_message(void)
{
    currentIndex = (currentIndex == 0) ? (MESSAGE_COUNT - 1) : (currentIndex - 1);
    draw_screen();
    lastButtonMs = millis();
}

/*
 * Function: next_message
 * Purpose: Selects the next TX message.
 * Parameters: None.
 * Return: None.
 */
static void next_message(void)
{
    currentIndex++;
    if (currentIndex >= MESSAGE_COUNT)
    {
        currentIndex = 0;
    }
    draw_screen();
    lastButtonMs = millis();
}

/*
 * Function: start_buzzer
 * Purpose: Starts a short buzzer pulse.
 * Parameters: durationMs - pulse duration.
 * Return: None.
 */
static void start_buzzer(uint32_t durationMs)
{
    buzzer_write(true);
    buzzerUntil = millis() + durationMs;
}

/*
 * Function: handle_buzzer
 * Purpose: Stops buzzer after timeout.
 * Parameters: None.
 * Return: None.
 */
static void handle_buzzer(void)
{
    if (buzzerUntil != 0 && millis() >= buzzerUntil)
    {
        buzzer_write(false);
        buzzerUntil = 0;
    }
}

/*
 * Function: start_sos_alert
 * Purpose: Starts SOS LED and buzzer alert.
 * Parameters: None.
 * Return: None.
 */
static void start_sos_alert(void)
{
    sosFlashUntil = millis() + SOS_FLASH_MS;
    lastSosFlashMs = 0;
    sosLedState = false;
    led_write(false);
    buzzer_write(false);
}

/*
 * Function: handle_sos_flash
 * Purpose: Flashes LED and buzzer during SOS.
 * Parameters: None.
 * Return: None.
 */
static void handle_sos_flash(void)
{
    if (sosFlashUntil == 0)
    {
        return;
    }

    if (millis() >= sosFlashUntil)
    {
        sosFlashUntil = 0;
        led_write(false);
        buzzer_write(false);
        return;
    }

    if ((millis() - lastSosFlashMs) >= SOS_FLASH_STEP_MS)
    {
        lastSosFlashMs = millis();
        sosLedState = !sosLedState;
        led_write(sosLedState);
        buzzer_write(sosLedState);
    }
}

/*
 * Function: send_message
 * Purpose: Sends the selected message packet.
 * Parameters: messageIndex - message array index.
 * Return: None.
 */
static void send_message(uint8_t messageIndex)
{
    Packet packet;
    bool ok;

    if (messageIndex >= MESSAGE_COUNT)
    {
        return;
    }

    packet.messageIndex = messageIndex;
    packet.senderId = DEVICE_ID;
    packet.counter = ++txCounter;
    ok = nrf_send_packet(&packet);

    lastSentIndex = (int8_t)messageIndex;
    draw_screen();

    uart_print("sent=");
    uart_tx(ok ? '1' : '0');
    uart_print(" msg=");
    uart_print(messages[messageIndex]);
    uart_print(" count=");
    uart_print_u32(packet.counter);
    uart_print("\r\n");

    led_write(ok);
    rxIndicatorUntil = millis() + 120;
    lastButtonMs = millis();
}

/*
 * Function: handle_radio
 * Purpose: Processes incoming RF packets.
 * Parameters: None.
 * Return: None.
 */
static void handle_radio(void)
{
    while (nrf_available())
    {
        Packet packet;
        nrf_read_packet(&packet);

        if (packet.messageIndex < MESSAGE_COUNT)
        {
            receivedIndex = (int8_t)packet.messageIndex;
            draw_screen();

            if (packet.messageIndex == SOS_INDEX)
            {
                start_sos_alert();
            }
            else
            {
                led_write(true);
                rxIndicatorUntil = millis() + RX_INDICATOR_MS;
                start_buzzer(BUZZ_MS);
            }

            uart_print("received=1 msg=");
            uart_print(messages[packet.messageIndex]);
            uart_print(" count=");
            uart_print_u32(packet.counter);
            uart_print("\r\n");
        }
    }
}

/*
 * Function: handle_buttons
 * Purpose: Reads buttons and runs message actions.
 * Parameters: None.
 * Return: None.
 */
static void handle_buttons(void)
{
    bool b1 = (BUTTON_PINREG & (1 << BUTTON1_PIN)) != 0;
    bool b2 = (BUTTON_PINREG & (1 << BUTTON2_PIN)) != 0;
    bool b3 = (BUTTON_PINREG & (1 << BUTTON3_PIN)) != 0;
    bool b4 = (BUTTON_PINREG & (1 << BUTTON4_PIN)) != 0;

    if ((millis() - lastButtonMs) > DEBOUNCE_MS)
    {
        if (lastB1 && !b1)
        {
            previous_message();
        }
        if (lastB2 && !b2)
        {
            next_message();
        }
        if (lastB3 && !b3)
        {
            send_message(currentIndex);
        }
        if (lastB4 && !b4)
        {
            send_message(SOS_INDEX);
        }
    }

    lastB1 = b1;
    lastB2 = b2;
    lastB3 = b3;
    lastB4 = b4;
}

/*
 * Function: main
 * Purpose: Initializes hardware and runs the RF node loop.
 * Parameters: None.
 * Return: Program return code.
 */
int main(void)
{
    gpio_init();
    uart_init();
    timer0_init();
    spi_init();
    twi_init();
    sei();

    uart_print("=== ");
    uart_print(DEVICE_NAME);
    uart_print(" ===\r\n");

    oled_init();
    draw_status("START");
    nrf_init();
    draw_screen();
    uart_print("Ready\r\n");

    while (1)
    {
        handle_buttons();
        handle_radio();
        handle_buzzer();
        handle_sos_flash();

        if (sosFlashUntil == 0 && rxIndicatorUntil != 0 && millis() > rxIndicatorUntil)
        {
            led_write(false);
            rxIndicatorUntil = 0;
        }
    }

    return 0;
}
