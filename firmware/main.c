/*
 * main.c
 *
 *  Created on: Feb 24, 2013
 *      Author: Jason Milldrum
 *     Company: Etherkit
 *
 *     Copyright (c) 2013, Jason Milldrum
 *     All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice, this list
 *  of conditions and the following disclaimer.
 *
 *  - Redistributions in binary form must reproduce the above copyright notice, this list
 *  of conditions and the following disclaimer in the documentation and/or other
 *  materials provided with the distribution.
 *
 *  - Neither the name of Etherkit nor the names of its contributors may be
 *  used to endorse or promote products derived from this software without specific
 *  prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 *  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 *  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>

#include "morsechar.h"
#include "sinewave.h"

// Input and output pin definitions
#define	SIDETONE				PD6			// Sidetone output
#define SIDETONE_PORT			PORTD		// Sidetone output port
#define SIDETONE_DDR			DDRD		// Sidetone output DDR

#define PADDLE_DIT				PD3			// Dit paddle
#define PADDLE_DIT_PORT			PORTD		// Dit paddle port
#define	PADDLE_DIT_DDR			DDRD		// Dit paddle DDR
#define PADDLE_DIT_PIN			PIND		// Dit paddle pin
#define PADDLE_DIT_PC			PCINT19

#define PADDLE_DAH				PD4			// Dah paddle
#define PADDLE_DAH_PORT			PORTD		// Dah paddle port
#define	PADDLE_DAH_DDR			DDRD		// Dah paddle DDR
#define PADDLE_DAH_PIN			PIND		// Dah paddle pin
#define PADDLE_DAH_PC			PCINT20

#define CMD_BUTTON				PD2			// Command button
#define CMD_BUTTON_PORT			PORTD
#define CMD_BUTTON_DDR			DDRD
#define CMD_BUTTON_PIN			PIND
#define CMD_BUTTON_PC			PCINT18

#define MSG_BUTTON				PD5			// Message button
#define MSG_BUTTON_PORT			PORTD
#define MSG_BUTTON_DDR			DDRD
#define MSG_BUTTON_PIN			PIND
#define MSG_BUTTON_PC			PCINT21

#define ENC_A					PB0			// Rotary encoder A output
#define ENC_A_PORT				PORTB
#define ENC_A_DDR				DDRB
#define ENC_A_PIN				PINB
#define ENC_A_PC				PCINT0

#define ENC_B					PD7			// Rotary encoder B output
#define ENC_B_PORT				PORTD
#define ENC_B_DDR				DDRD
#define ENC_B_PIN				PIND
#define ENC_B_PC				PCINT23

#define ENC_BUTTON				PC3			// Rotary encoder button
#define ENC_BUTTON_PORT			PORTC
#define ENC_BUTTON_DDR			DDRC
#define ENC_BUTTON_PIN			PINC
#define ENC_BUTTON_PC			PCINT11

#define MUTE					PB6			// Mute output
#define MUTE_PORT				PORTB
#define MUTE_DDR				DDRB

#define TX						PB7			// Transmit output
#define TX_PORT					PORTB
#define TX_DDR					DDRB

#define SPI_DDR					DDRB		// SPI
#define SPI_PORT				PORTB
#define SPI_MOSI				PB3
#define SPI_SCK					PB5
#define SPI_FSYNC				PB1
#define SPI_SS					PB2

#define RIT_LED					PC1			// RIT enable LED
#define RIT_LED_PORT			PORTC
#define RIT_LED_DDR				DDRC

#define XIT_LED					PC0			// RIT enable LED
#define XIT_LED_PORT			PORTC
#define XIT_LED_DDR				DDRC

#define I2C_SDA					PC4
#define I2C_SDA_PORT			PORTC
#define I2C_SDA_DDR				DDRC

#define I2C_SCL					PC5
#define I2C_SCL_PORT			PORTC
#define I2C_SCL_DDR				DDRC

// Constant defines
#define TIMER2_COUNT			249			// 4 us clk period * 250 ticks (0-249)= 1 ms Timer2 CTC A overflow
#define DEBOUNCE_PRESS_TIME		5			// Amount of captures for press keybounce (in 1 ms increments)
#define DEBOUNCE_HOLD_TIME		500			// Amount of captures for hold keybounce (in 1 ms increments)
#define DEFAULT_WPM				13			// Default keyer WPM
#define	MIN_WPM					5			// Minimum WPM setting
#define MAX_WPM					40			// Maximum WPM setting
#define TX_ON_DELAY				2			// TX sequence delay time (in 1 ms increments)
#define TX_OFF_DELAY			2
#define MUTE_OFF_DELAY			20			// Mute off delay time (in 1 ms increments)
#define ANNOUNCE_BUFFER_SIZE	41			// Buffer size for announce string
#define MENU_EXPIRATION			4000		// Menu expiration time (in 1 ms increments)
#define REC_EXPIRATION			1000		// Keyer memory character record expiration
#define REC_SPACE_EXPIRATION	2000		// Keyer memory space character expiration
#define MSG_BUFFER_SIZE			41			// Keyer message size in characters
#define SLEEP_DELAY				300			// Time (in ms) to delay before going to sleep because of inactivity
#define ST_REFCLK				429497		// Sidetone DDS ref clock - 2^32 / 10 kHz sample rate
#define ST_DEFAULT				600			// Default sidetone frequency
#define ST_HIGH					900			// High sidetone frequency
#define ST_LOW					400			// Low sidetone frequency

// DDS tuning steps (50 MHz master clock)
#define DDS_20HZ				0x1B
#define DDS_100HZ				0x86

// Band selection
// Use this section to choose the proper band for your CC1
// Uncomment the desired band and make sure all other bands are commented out

//#define BAND_40M
#define BAND_20M

// Band constants
// 40 Meters
#ifdef BAND_40M
#define DDS_INIT				0x3D2806B	// AD9834 w/ 50 MHz Fc, 11.945 MHz VFO - 600 Hz shift
#define DDS_TX_INIT				0x23FE5C9
#define FREQ_INIT				7030000
#define LOWER_FREQ_LIMIT		7000000
#define UPPER_FREQ_LIMIT		7300000
#endif

// 20 Meters
#ifdef BAND_20M
#define DDS_INIT				0x6126634 	// AD9834 w/ 50 MHz Fc, 18.9752 MHz VFO + 600 Hz shift
#define DDS_TX_INIT				0x47FCB92
#define FREQ_INIT				14060000
#define LOWER_FREQ_LIMIT		14000000
#define UPPER_FREQ_LIMIT		14350000
#endif

// Macro for any button press
#define ANYBUTTON				(dit_active == TRUE) || (dah_active == TRUE) || (cmd_btn == BTN_PRESS) || (msg_btn == BTN_PRESS)

// State machine and other enumerations
enum BOOL {FALSE, TRUE};
enum STATE {STATE_INIT, STATE_IDLE, STATE_DIT, STATE_DAH, STATE_DITDELAY, STATE_DAHDELAY, STATE_WORDDELAY, STATE_KEYDOWN, STATE_ENDKEYDOWN, STATE_MENUANNOUNCE, STATE_MENUINPUT, STATE_VALIDATECHAR, STATE_EXIT};
enum MODE {MODE_KEYER, MODE_SK, MODE_ANNOUNCE, MODE_TUNE, MODE_MENU, MODE_SETWPM, MODE_PLAYBACK, MODE_RECORD, MODE_CAL};
enum BTN {BTN_OFF, BTN_PRESS, BTN_HOLD};
enum TUNERATE {TUNE_SLOW, TUNE_FAST};
enum FREQREG {REG_0, REG_1}; // During incremental tuning, REG_0 is the RX freq, REG_1 is the TX freq
enum INCTUNE {INC_NONE, INC_RIT, INC_XIT};
enum VFO {VFO_RX, VFO_TX};
enum MSGMEM {MSGMEM_1, MSGMEM_2};

// Global variable defs
uint16_t dit_length;
uint8_t wpm, prev_wpm, pwm_delay;
uint32_t cur_state_end, prev_state_end, sleep_timer;
enum STATE prev_state, cur_state, next_state;
enum MODE prev_mode, cur_mode, default_mode;
char * announce_buffer;
char * text_buffer;
static char msg_buffer[MSG_BUFFER_SIZE];
char menu[] = {'S', 'W', '1', '2', 'V', 'K', '\0'};
enum TUNERATE tune_rate = TUNE_FAST;
uint16_t tune_step = DDS_100HZ;
uint8_t tune_freq_step = 25;
uint16_t st_freq, prev_st_freq;
enum VFO rx_vfo = VFO_RX;

// Global variables used in ISRs
volatile uint32_t timer, cur_timer;
volatile uint8_t ind;
volatile uint8_t port_b_latch, port_d_latch;
volatile unsigned long freq;
volatile enum BOOL sidetone_on = FALSE;
volatile enum BOOL mute_on = FALSE;
volatile enum BOOL key_down = FALSE;
volatile enum BOOL fc_done = FALSE;
volatile enum BOOL dit_active, dah_active;
volatile enum BOOL allow_sleep = TRUE;
volatile enum BTN cmd_btn, msg_btn, both_btn, enc_btn;
volatile enum BOOL enc_a, enc_b;
volatile enum INCTUNE inc_tune_state;
volatile enum FREQREG tune_reg;
volatile uint32_t tx_start, tx_end, mute_start, mute_end, led_toggle;
volatile uint32_t st_phase_acc, st_tune_word;
volatile uint8_t st_sine_lookup;
volatile uint32_t dds_freq_word, dds_rit_freq_word, dds_xit_freq_word, dds_tx_freq_word, dds_old_tx_freq_word;
volatile uint32_t tune_freq;
volatile int16_t it_delta;
volatile uint16_t st_period;

// EEPROM variables
uint8_t EEMEM ee_wpm = DEFAULT_WPM;
enum BOOL EEMEM ee_keyer = TRUE;
char EEMEM ee_msg_mem_1[MSG_BUFFER_SIZE] = "";
char EEMEM ee_msg_mem_2[MSG_BUFFER_SIZE] = "";
uint32_t EEMEM ee_dds_init = DDS_INIT;

// Function prototypes
void set_wpm(uint8_t);
void init(void);
void debounce(enum BOOL);
void announce(char * msg, uint16_t freq, uint8_t speed);
void read_voltage(void);
void count_frequency(void);
void poll_buttons(void);
void tune_dds(uint32_t dds_word, enum FREQREG reg, enum BOOL init);
void tx_dds(enum BOOL tx);
void send_dds_word(uint16_t);
void set_dds_freq_reg(enum FREQREG reg);
void set_st_freq(uint32_t);
void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(uint8_t);
uint8_t i2c_read_ack(void);
uint8_t i2c_read_nack(void);
uint8_t i2c_status(void);
//uint8_t dac_write(uint16_t);


// Timer1 ISR
//
// Timer1 is the sinewave generator.
ISR(TIMER1_COMPA_vect)
{
	cli();
	if(sidetone_on == TRUE)
	{
		SIDETONE_DDR |= _BV(SIDETONE);
		SIDETONE_PORT ^= _BV(SIDETONE);

		// PWM generator
		st_phase_acc = st_phase_acc + st_tune_word;
		st_sine_lookup = (uint8_t)(st_phase_acc >> 24);
		OCR0A = pgm_read_byte_near(&sinewave[st_sine_lookup]); // Just use the upper 8 bits for sine lookup
	}
	else
	{
		// Hi-Z the port when not using
		SIDETONE_DDR &= ~(_BV(SIDETONE));
		SIDETONE_PORT &= ~(_BV(SIDETONE));
	}
	sei();
}

// Timer2 ISR
//
// Fires every 1 ms. Used as a main system clock, and handles the
// mute and transmit ports.
ISR(TIMER2_COMPA_vect)
{
	// Handle mute
	if(((timer > mute_start) && (timer < mute_end)) || (mute_on == TRUE))
	//if(mute_on == TRUE)
		MUTE_PORT |= _BV(MUTE);
	else
		MUTE_PORT &= ~(_BV(MUTE));

	// Handle transmit
	if((timer < tx_end) && (timer > tx_start))
	{
		// Keep DDS on TX freq for a few ms after TX to allow proper envelope shaping
		set_dds_freq_reg(REG_1);
		if(key_down == TRUE)
			TX_PORT |= _BV(TX);
		else
			TX_PORT &= ~(_BV(TX));
	}
	else
	{
		set_dds_freq_reg(REG_0);
		TX_PORT &= ~(_BV(TX));
	}

	// Handle the RIT/XIT LED
	if(inc_tune_state != INC_NONE)
	{
		if(inc_tune_state == INC_RIT)
		{
			RIT_LED_PORT |= _BV(RIT_LED);
			XIT_LED_PORT &= ~(_BV(XIT_LED));
		}
		else if(inc_tune_state == INC_XIT)
		{
			XIT_LED_PORT |= _BV(XIT_LED);
			RIT_LED_PORT &= ~(_BV(RIT_LED));
		}

	}

	debounce(FALSE);

	// Need to consider timer overflow?
	timer++;
}

/*
// Just needed to wake up on pin change
ISR(PCINT0_vect)
{
	// Stop any more pin change interrupts
	PCICR = 0;

	cur_mode = default_mode;
	cur_state = STATE_IDLE;

	// Needs some idle time to get up to speed
	cur_state_end = cur_timer + 100;
	sleep_timer = cur_timer + SLEEP_DELAY;
}


// Just needed to wake up on pin change
ISR(PCINT1_vect)
{
	// Stop any more pin change interrupts
	PCICR = 0;

	cur_mode = default_mode;
	cur_state = STATE_IDLE;

	// Needs some idle time to get up to speed
	cur_state_end = cur_timer + 100;
	sleep_timer = cur_timer + SLEEP_DELAY;
}


// Just needed to wake up on pin change
ISR(PCINT2_vect)
{
	// Stop any more pin change interrupts
	PCICR = 0;

	cur_mode = default_mode;
	cur_state = STATE_IDLE;

	// Needs some idle time to get up to speed
	cur_state_end = cur_timer + 100;
	sleep_timer = cur_timer + SLEEP_DELAY;
}
*/

void init(void)
{
	// Disable interrupts
	cli();

	// Setup Timer0 as phase correct PWM
	TCCR0A = _BV(COM0A1) | _BV(WGM00); // Set for Phase Correct PWM mode, output on OC0A
	TCCR0B = _BV(CS00); // Prescaler /1

	// Setup Timer1 as sample rate generator for sidetone
	TCCR1B = _BV(WGM12) | _BV(CS11); // Set for CTC mode, Prescaler /8
	TCCR1A = 0;
	//OCR1A = 49; // 1 MHz clock / 16 kHz sample rate = 50
	OCR1A = 99; // (1 MHz clock / 10 kHz sample rate) = 100
	TIMSK1 = _BV(OCIE1A);

	// Setup Timer2 as main event timer, 8 us tick
	TCCR2A = _BV(WGM21); // Set for CTC mode
	TCCR2B = _BV(CS21) | _BV(CS20); // Prescaler /32 for 8 MHz clock
	//TCCR2B = _BV(CS22); // Prescaler /64 for 16 MHz clock
	//TCCR2B = _BV(CS21); // Prescaler /8 for 1 MHz clock
	TIMSK2 |= _BV(OCIE2A); // Enable Timer2 CTC interrupt
	OCR2A = TIMER2_COUNT; // Timer2 CTC A value

	// Setup ADC
	//ADCSRA |= _BV(ADPS2) | _BV(ADEN); // Prescaler /16, enable ADC
	//ADMUX = _BV(REFS0) | _BV(ADLAR) | _BV(MUX1);  // AREF ref voltage, left adjust result, read channel 0

	// Setup pin change interrupts on paddle inputs and buttons
	//PCMSK0 = _BV(ENC_A_PC);
	//PCMSK2 = _BV(PADDLE_DIT_PC) | _BV(PADDLE_DAH_PC) | _BV(CMD_BUTTON_PC) | _BV(MSG_BUTTON_PC) |_BV(ENC_B_PC) | _BV(ENC_BUTTON_PC);
	//PCICR = _BV(PCIE0) | _BV(PCIE1) | _BV(PCIE2);

	// Configure output ports
	SIDETONE_DDR |= _BV(SIDETONE);
	MUTE_DDR |= _BV(MUTE);
	TX_DDR |= _BV(TX);
	RIT_LED_DDR &= ~(_BV(RIT_LED));
	RIT_LED_PORT &= ~(_BV(RIT_LED));
	XIT_LED_DDR &= ~(_BV(XIT_LED));
	XIT_LED_PORT &= ~(_BV(XIT_LED));

	I2C_SDA_DDR |= _BV(I2C_SDA);
	I2C_SDA_PORT |= _BV(I2C_SDA);

	I2C_SCL_DDR |= _BV(I2C_SCL);
	I2C_SCL_PORT |= _BV(I2C_SCL);

	// Configure input ports
	PADDLE_DIT_DDR &= ~(_BV(PADDLE_DIT));
	PADDLE_DIT_PORT |= _BV(PADDLE_DIT); // Enable pull-up

	PADDLE_DAH_DDR &= ~(_BV(PADDLE_DAH));
	PADDLE_DAH_PORT |= _BV(PADDLE_DAH); // Enable pull-up

	CMD_BUTTON_DDR &= ~(_BV(CMD_BUTTON));
	CMD_BUTTON_PORT |= _BV(CMD_BUTTON); // Enable pull-up

	MSG_BUTTON_DDR &= ~(_BV(MSG_BUTTON));
	MSG_BUTTON_PORT |= _BV(MSG_BUTTON); // Enable pull-up

	ENC_A_DDR &= ~(_BV(ENC_A));
	ENC_A_PORT |= _BV(ENC_A); // Enable pull-up

	ENC_B_DDR &= ~(_BV(ENC_B));
	ENC_B_PORT |= _BV(ENC_B); // Enable pull-up

	ENC_BUTTON_DDR &= ~(_BV(ENC_BUTTON));
	ENC_BUTTON_PORT |= _BV(ENC_BUTTON); // Enable pull-up

	// Configure SPI
	SPI_DDR |= _BV(SPI_MOSI) | _BV(SPI_SCK) | _BV(SPI_SS) | _BV(SPI_FSYNC);
	//SPCR = _BV(SPE) | _BV(MSTR) |_BV(CPOL) | _BV(SPR0);
	SPCR = _BV(SPE) | _BV(MSTR) | _BV(CPOL) | _BV(SPI2X);
	//SPI_PORT |= _BV(SPI_SS);

	uint8_t spi_data = SPSR; // Dummy read to clear interrupt flag
	spi_data = SPDR;

	//set_sleep_mode(SLEEP_MODE_STANDBY);

	// Initialize global variables
	prev_state = STATE_IDLE;
	cur_state = STATE_IDLE;
	next_state = STATE_IDLE;

	timer = 0;

	dds_freq_word = eeprom_read_dword(&ee_dds_init);
	dds_tx_freq_word = DDS_TX_INIT;
	tune_freq = FREQ_INIT;
	tune_dds(dds_freq_word, REG_0, TRUE);
	tune_dds(dds_freq_word, REG_0, FALSE);
	tune_dds(dds_tx_freq_word, REG_1, FALSE);

	st_freq = ST_DEFAULT;
	set_st_freq(st_freq);

	inc_tune_state = INC_NONE;
	tune_reg = REG_0;

	MUTE_PORT |= _BV(MUTE);

	wpm = eeprom_read_byte(&ee_wpm);
	set_wpm(wpm);

	if(eeprom_read_byte(&ee_keyer) == FALSE)
		cur_mode = MODE_SK;
	else
		cur_mode = MODE_KEYER;

	// Check to see if we should startup in straight key mode
	if((dah_active == TRUE) && (dit_active == FALSE))
		cur_mode = MODE_SK;

	if(bit_is_clear(ENC_BUTTON_PIN, ENC_BUTTON))
	{
		cur_mode = MODE_CAL;
		cur_state = STATE_IDLE;
	}

	TX_PORT &= ~(_BV(TX));

	i2c_init();

	// Delay for just a bit to let AF amp get ready before starting
	for (uint16_t i = 0; i < 300; i++)
		_delay_ms(1);

	MUTE_PORT &= ~(_BV(MUTE));

	// Enable interrupts
	sei();
}

void set_wpm(uint8_t new_wpm)
{
	// Dit length in milliseconds is 1200 ms / WPM
	// then divide that by the 1 ms per timer tick
	dit_length = (1200 / new_wpm);
}

void debounce(enum BOOL flush)
{

	static uint16_t dit_on_count, dah_on_count, dit_off_count, dah_off_count, cmd_on_count, msg_on_count, both_on_count;
	static uint16_t enca_on_count, enca_off_count, encb_on_count, encb_off_count, enc_on_count;

	if(flush == TRUE)
	{
		dit_on_count = 0;
		dah_on_count = 0;
		dit_off_count = 0;
		dah_off_count = 0;
		cmd_on_count = 0;
		msg_on_count = 0;
		both_on_count = 0;
		enc_on_count = 0;
		enca_on_count = 0;
		encb_on_count = 0;
	}

	// Debounce DIT
	if(bit_is_clear(PADDLE_DIT_PIN, PADDLE_DIT))
	{
		if(dit_on_count < DEBOUNCE_PRESS_TIME)
			dit_on_count++;
		dit_off_count = 0;
	}
	else
	{
		if(dit_off_count < DEBOUNCE_PRESS_TIME)
			dit_off_count++;
		dit_on_count = 0;
	}

	// Debounce DAH
	if(bit_is_clear(PADDLE_DAH_PIN, PADDLE_DAH))
	{
		if(dah_on_count < DEBOUNCE_PRESS_TIME)
			dah_on_count++;
		dah_off_count = 0;
	}
	else
	{
		if(dah_off_count < DEBOUNCE_PRESS_TIME)
			dah_off_count++;
		dah_on_count = 0;
	}

	// Set button flags according to final debounce count
	if(dit_on_count >= DEBOUNCE_PRESS_TIME)
		dit_active = TRUE;
	if(dit_off_count >= DEBOUNCE_PRESS_TIME)
		dit_active = FALSE;

	if(dah_on_count >= DEBOUNCE_PRESS_TIME)
		dah_active = TRUE;
	if(dah_off_count >= DEBOUNCE_PRESS_TIME)
		dah_active = FALSE;


	// Debounce both control buttons
	if((bit_is_clear(CMD_BUTTON_PIN, CMD_BUTTON)) && bit_is_clear(MSG_BUTTON_PIN, MSG_BUTTON))
		both_on_count++;
	else
	{
		if((both_on_count >= DEBOUNCE_PRESS_TIME) && (both_on_count < DEBOUNCE_HOLD_TIME))
			both_btn = BTN_PRESS;
		else if(both_on_count >= DEBOUNCE_HOLD_TIME)
			both_btn = BTN_HOLD;
		else
			both_btn = BTN_OFF;

		both_on_count = 0;
	}


	// Debounce CMD button
	if(bit_is_clear(CMD_BUTTON_PIN, CMD_BUTTON))
	{
		cmd_on_count++;
		//mute_on = TRUE;
	}
	else
	{
		if((cmd_on_count >= DEBOUNCE_PRESS_TIME) && (cmd_on_count < DEBOUNCE_HOLD_TIME))
			cmd_btn = BTN_PRESS;
		else if(cmd_on_count >= DEBOUNCE_HOLD_TIME)
			cmd_btn = BTN_HOLD;
		else
			cmd_btn = BTN_OFF;

		cmd_on_count = 0;
	}

	// Debounce MSG button
	if(bit_is_clear(MSG_BUTTON_PIN, MSG_BUTTON))
		msg_on_count++;
	else
	{
		if((msg_on_count >= DEBOUNCE_PRESS_TIME) && (msg_on_count < DEBOUNCE_HOLD_TIME))
			msg_btn = BTN_PRESS;
		else if(msg_on_count >= DEBOUNCE_HOLD_TIME)
			msg_btn = BTN_HOLD;
		else
			msg_btn = BTN_OFF;

		msg_on_count = 0;
	}

	// Debounce encoder button
	if(bit_is_clear(ENC_BUTTON_PIN, ENC_BUTTON))
		enc_on_count++;
	else
	{
		if((enc_on_count >= DEBOUNCE_PRESS_TIME) && (enc_on_count < DEBOUNCE_HOLD_TIME))
			enc_btn = BTN_PRESS;
		else if(enc_on_count >= DEBOUNCE_HOLD_TIME)
			enc_btn = BTN_HOLD;
		else
			enc_btn = BTN_OFF;

		enc_on_count = 0;
	}


	// Debounce Encoder A
	if(bit_is_clear(ENC_A_PIN, ENC_A))
	{
		if(enca_on_count < DEBOUNCE_PRESS_TIME)
			enca_on_count++;
		enca_off_count = 0;
	}
	else
	{
		if(enca_off_count < DEBOUNCE_PRESS_TIME)
			enca_off_count++;
		enca_on_count = 0;
	}


	// Debounce Encoder B
	if(bit_is_clear(ENC_B_PIN, ENC_B))
	{
		if(encb_on_count < DEBOUNCE_PRESS_TIME)
			encb_on_count++;
		encb_off_count = 0;
	}
	else
	{
		if(encb_off_count < DEBOUNCE_PRESS_TIME)
			encb_off_count++;
		encb_on_count = 0;
	}

	// Set encoder flags
	if(enca_on_count >= DEBOUNCE_PRESS_TIME)
		enc_a = TRUE;
	if(enca_off_count >= DEBOUNCE_PRESS_TIME)
		enc_a = FALSE;

	if(encb_on_count >= DEBOUNCE_PRESS_TIME)
		enc_b = TRUE;
	if(encb_off_count >= DEBOUNCE_PRESS_TIME)
		enc_b = FALSE;

	/*
	// Don't go to sleep if there are any paddle or button presses
	if((dit_on_count > 0) || (dah_on_count > 0) || (cmd_on_count > 0) || (msg_on_count > 0) || (both_on_count > 0))
		allow_sleep = FALSE;
	else
		allow_sleep = TRUE;
		*/
}

void announce(char * msg, uint16_t freq, uint8_t speed)
{
	// Convert to uppercase
	strupr(msg);

	// Need buffer overflow checking here
	strcpy(announce_buffer, msg);

	// Retain the current state and mode
	prev_state = cur_state;
	prev_state_end = cur_state_end;
	prev_mode = cur_mode;
	prev_st_freq = st_freq;
	st_freq = freq;
	prev_wpm = wpm;
	wpm = speed;

	set_st_freq(st_freq);
	set_wpm(wpm);

	// Set into announce mode
	cur_state = STATE_IDLE;
	cur_mode = MODE_ANNOUNCE;
}

void read_voltage(void)
{
	uint16_t vcc, vcc_mon;
	char *vcc_out;

	// Setup ADC
	ADCSRA |= _BV(ADPS2) | _BV(ADEN); // Prescaler /16, enable ADC
	ADMUX = _BV(REFS0) | _BV(ADLAR) | _BV(MUX1);  // AREF ref voltage, left adjust result, read channel 0

	vcc_out = malloc(10);

	// Start ADC conversion
	ADCSRA |= _BV(ADSC);

	// Wait for ADC conversion to finish
	loop_until_bit_is_clear(ADCSRA, ADSC);

	// Get ADC value
	vcc_mon = ADCH;

	// Full scale reading at uC is 16.1 V
	// We'll use fixed point numbers, so full scale is 161 * 0.1 V
	vcc = (vcc_mon * 161) / 256;

	// Format for output
	sprintf(vcc_out, "%dR%d", vcc / 10, vcc % 10);

	announce(vcc_out, st_freq, wpm);

	free(vcc_out);
}

void count_frequency(void)
{
	char *freq_out;

	freq_out = malloc(15);

	// Format and output frequency
	if(inc_tune_state == INC_XIT || inc_tune_state == INC_RIT)
	{
		if(it_delta < 0)
			sprintf(freq_out, "-%1iR%2.2i", (int)abs(((it_delta / 1000) % 100)), (int)abs(((it_delta % 1000) / 10)));
		else
			sprintf(freq_out, "%1iR%2.2i", (int)((it_delta / 1000) % 100), (int)abs(((it_delta % 1000) / 10)));
	}
	else
		sprintf(freq_out, "%02uR%02u", (unsigned int)((tune_freq / 1000) % 100), (unsigned int)((tune_freq % 1000) / 10));
	announce(freq_out, st_freq, wpm);

	free(freq_out);
}

void poll_buttons(void)
{
	static uint8_t prev_enc_state;

	// Handle buttons if not in CAL mode
	if(cur_mode != MODE_CAL)
	{
		if(both_btn == BTN_HOLD)
		{
			cur_state = STATE_IDLE;
			prev_mode = cur_mode;
			cur_mode = MODE_TUNE;
			sleep_timer = cur_timer + SLEEP_DELAY;
		}
		else if(cmd_btn == BTN_PRESS)
		{
			prev_mode = cur_mode;
			cur_mode = MODE_MENU;
			cur_state = STATE_IDLE;
			sleep_timer = cur_timer + SLEEP_DELAY;
		}
		else if(cmd_btn == BTN_HOLD)
		{
			// Playback message memory 1
			eeprom_read_block((void*)&msg_buffer, (const void*)&ee_msg_mem_1, 40);
			strcpy(announce_buffer, msg_buffer);
			cur_state = STATE_IDLE;
			prev_mode = cur_mode;
			cur_mode = MODE_PLAYBACK;
			sleep_timer = cur_timer + SLEEP_DELAY;
		}
		else if(msg_btn == BTN_PRESS)
		{
			count_frequency();
			sleep_timer = cur_timer + SLEEP_DELAY;
		}
		else if(msg_btn == BTN_HOLD)
		{
			// Playback message memory 1
			eeprom_read_block((void*)&msg_buffer, (const void*)&ee_msg_mem_2, 40);
			strcpy(announce_buffer, msg_buffer);
			cur_state = STATE_IDLE;
			prev_mode = cur_mode;
			cur_mode = MODE_PLAYBACK;
			sleep_timer = cur_timer + SLEEP_DELAY;
		}

		if(enc_btn == BTN_PRESS)
		{
			// If we are in normal tuning mode, pressing the tune knob toggles tuning rates
			if(inc_tune_state == INC_NONE)
			{
				if(tune_rate == TUNE_FAST)
				{
					tune_rate = TUNE_SLOW;
					tune_step = DDS_20HZ;
					tune_freq_step = 5;
					sleep_timer = cur_timer + SLEEP_DELAY;
					debounce(TRUE);
					announce("S", ST_LOW, 25);
				}
				else
				{
					tune_rate = TUNE_FAST;
					tune_step = DDS_100HZ;
					tune_freq_step = 25;
					sleep_timer = cur_timer + SLEEP_DELAY;
					debounce(TRUE);
					announce("F", ST_HIGH, 25);
				}
			}
			// Otherwise if we are in RIT or XIT, pressing the tune knob toggles between the two VFOs
			else
			{
				if(rx_vfo == VFO_RX)
				{
					rx_vfo = VFO_TX;
					tune_dds(dds_xit_freq_word, REG_0, FALSE);
					announce("T", ST_LOW, 25);
				}
				else
				{
					rx_vfo = VFO_RX;
					tune_dds(dds_rit_freq_word, REG_0, FALSE);
					announce("R", ST_LOW, 25);
				}
			}
		}
		else if(enc_btn == BTN_HOLD)
		{
			// Rotate through the 3 states
			inc_tune_state++;
			if(inc_tune_state > 2)
				inc_tune_state = INC_NONE;

			switch(inc_tune_state)
			{
				case INC_RIT:
					RIT_LED_DDR |= _BV(RIT_LED);
					tune_rate = TUNE_SLOW;
					tune_step = DDS_20HZ;
					tune_freq_step = 5;
					dds_rit_freq_word = dds_freq_word;
					dds_xit_freq_word = dds_freq_word;
					dds_old_tx_freq_word = dds_tx_freq_word;
					tune_dds(dds_rit_freq_word, REG_0, FALSE);
					tune_dds(dds_tx_freq_word, REG_1, FALSE);
					rx_vfo = VFO_RX;
					//tune_reg = REG_0;
					//set_dds_freq_reg(tune_reg);
					it_delta = 0;
					debounce(TRUE);
					sleep_timer = cur_timer + SLEEP_DELAY;
					announce("R", ST_HIGH, 25);
					break;

				case INC_XIT:
					XIT_LED_DDR |= _BV(XIT_LED);
					tune_rate = TUNE_FAST;
					tune_step = DDS_100HZ;
					tune_freq_step = 25;
					dds_rit_freq_word = dds_freq_word;
					dds_xit_freq_word = dds_freq_word;
					dds_old_tx_freq_word = dds_tx_freq_word;
					tune_dds(dds_xit_freq_word, REG_0, FALSE);
					tune_dds(dds_tx_freq_word, REG_1, FALSE);
					rx_vfo = VFO_TX;
					//tune_reg = REG_1;
					//set_dds_freq_reg(tune_reg);
					it_delta = 0;
					debounce(TRUE);
					sleep_timer = cur_timer + SLEEP_DELAY;
					announce("X", ST_HIGH, 25);
					break;

				case INC_NONE:
				default:
					RIT_LED_DDR &= ~(_BV(RIT_LED));
					RIT_LED_PORT &= ~(_BV(RIT_LED));
					XIT_LED_DDR &= ~(_BV(XIT_LED));
					XIT_LED_PORT &= ~(_BV(XIT_LED));
					tune_rate = TUNE_FAST;
					tune_step = DDS_100HZ;
					tune_freq_step = 25;
					dds_tx_freq_word = dds_old_tx_freq_word;
					tune_dds(dds_freq_word, REG_0, FALSE);
					tune_dds(dds_tx_freq_word, REG_1, FALSE);
					//tune_reg = REG_0;
					//set_dds_freq_reg(tune_reg);
					it_delta = 0;
					debounce(TRUE);
					sleep_timer = cur_timer + SLEEP_DELAY;
					announce("O", ST_HIGH, 25);
					break;
			}
		}
	}

	// Handle encoder
	uint8_t cur_enc_state = 0;

	// Set bits representing current encoder state
	if(enc_a)
		cur_enc_state += 0x02;
	if(enc_b)
		cur_enc_state += 0x01;

	// If the current state is different from previous state, the encoder has moved
	if(cur_enc_state != prev_enc_state)
	{
		prev_enc_state = (prev_enc_state >> 1) & 0x01;
		//sleep_timer = cur_timer + SLEEP_DELAY;

		// Compare current B state to previous A state
		if((prev_enc_state ^ (cur_enc_state & 0x01)) == 1)
		{
			// Don't allow tuning if we are on the locked VFO
			if((inc_tune_state == INC_RIT && rx_vfo == VFO_RX) || (inc_tune_state == INC_XIT && rx_vfo == VFO_TX) || (inc_tune_state == INC_NONE))
			{

				if(tune_freq > LOWER_FREQ_LIMIT)
				{

					if(inc_tune_state == INC_RIT)
					{
						dds_rit_freq_word -= tune_step;
						it_delta -= tune_freq_step;
						tune_dds(dds_rit_freq_word, REG_0, FALSE);
					}
					else if(inc_tune_state == INC_XIT)
					{
						dds_tx_freq_word -= tune_step;
						dds_xit_freq_word -= tune_step;
						it_delta -= tune_freq_step;
						tune_dds(dds_xit_freq_word, REG_0, FALSE);
					}
					else if(cur_mode == MODE_CAL)
					{
						dds_freq_word -= DDS_20HZ;
						tune_dds(dds_freq_word, REG_0, FALSE);
						eeprom_write_dword(&ee_dds_init, dds_freq_word);
						eeprom_busy_wait();
					}
					else
					{
						dds_tx_freq_word -= tune_step;
						dds_freq_word -= tune_step;
						tune_freq -= tune_freq_step;
						tune_dds(dds_freq_word, REG_0, FALSE);
					}

					tune_dds(dds_tx_freq_word, REG_1, FALSE);
					//set_dds_freq_reg(tune_reg);
				}
				else
				{
					debounce(TRUE);
					announce("L", ST_HIGH, 25);
				}
			}
		}
		else
		{
			// Don't allow tuning if we are on the locked VFO
			if((inc_tune_state == INC_RIT && rx_vfo == VFO_RX) || (inc_tune_state == INC_XIT && rx_vfo == VFO_TX) || (inc_tune_state == INC_NONE))
			{

				// Tune up as long as we are not at upper limit
				if(tune_freq < UPPER_FREQ_LIMIT)
				{
					if(inc_tune_state == INC_RIT)
					{
						dds_rit_freq_word += tune_step;
						it_delta += tune_freq_step;
						tune_dds(dds_rit_freq_word, REG_0, FALSE);
					}
					else if(inc_tune_state == INC_XIT)
					{
						dds_tx_freq_word += tune_step;
						dds_xit_freq_word += tune_step;
						it_delta += tune_freq_step;
						tune_dds(dds_xit_freq_word, REG_0, FALSE);
					}
					else if(cur_mode == MODE_CAL)
					{
						dds_freq_word += DDS_20HZ;
						tune_dds(dds_freq_word, REG_0, FALSE);
						eeprom_write_dword(&ee_dds_init, dds_freq_word);
						eeprom_busy_wait();
					}
					else
					{
						dds_tx_freq_word += tune_step;
						dds_freq_word += tune_step;
						tune_freq += tune_freq_step;
						tune_dds(dds_freq_word, REG_0, FALSE);
					}

					tune_dds(dds_tx_freq_word, REG_1, FALSE);
					//set_dds_freq_reg(tune_reg);
				}
				else
				{
					debounce(TRUE);
					announce("U", ST_HIGH, 25);
				}
			}
		}
	}

	prev_enc_state = cur_enc_state;
}

void tune_dds(uint32_t dds_word, enum FREQREG reg, enum BOOL init)
{
	uint16_t dds_word_high, dds_word_low, freq_reg;

	if(reg == REG_1)
		freq_reg = 0x8000;
	else
		freq_reg = 0x4000;

	dds_word_low = (uint16_t)((dds_word & 0x3FFF) + freq_reg);
	dds_word_high = (uint16_t)(((dds_word >> 14) & 0x3FFF) + freq_reg);

	if(init == TRUE)
		send_dds_word(0x2118);
	//else
		//send_dds_word(0x2018);

	// Send frequency word LSB
	send_dds_word(dds_word_low);

	// Send frequency word MSB
	send_dds_word(dds_word_high);

	if(init == TRUE)
	{
		// Send phase
		send_dds_word(0xC000);

		// Exit reset
		send_dds_word(0x2018);
	}
}

void tx_dds(enum BOOL tx)
{
	if(tx == TRUE)
		send_dds_word(0x2838);
	else
		send_dds_word(0x2018);
}

void send_dds_word(uint16_t dds_word)
{
	SPI_PORT |= _BV(SPI_SCK);
	SPI_PORT &= ~(_BV(SPI_FSYNC));
	SPDR = (uint8_t)((dds_word >> 8) & 0xFF);
	while(!(SPSR & (1<<SPIF)));
	SPDR = (uint8_t)(dds_word & 0xFF);
	while(!(SPSR & (1<<SPIF)));
	SPI_PORT |= _BV(SPI_FSYNC);
}

void set_dds_freq_reg(enum FREQREG reg)
{
	// Control register
	if(reg == REG_1)
		send_dds_word(0x2818);
	else
		send_dds_word(0x2018);
}

void set_st_freq(uint32_t st_freq)
{
	// PWM sidetone
	st_tune_word = st_freq  * ST_REFCLK; // A way to avoid 64-bit math, ST_REFCLK is 1/(2^32/REFCLK)
}

void i2c_init(void)
{
	//set SCL to ~55 kHz
	TWSR = 0x00;
	TWBR = 0x0A;

	//enable I2C
	TWCR = _BV(TWEN);
}

void i2c_start(void)
{
	TWCR = _BV(TWINT) | _BV(TWSTA) | _BV(TWEN);
	while((TWCR & _BV(TWINT)) == 0);
}

void i2c_stop(void)
{
	TWCR = _BV(TWINT) | _BV(TWSTO) | _BV(TWEN);
}

void i2c_write(uint8_t data)
{
	TWDR = data;
	TWCR = _BV(TWINT) | _BV(TWEN);
	while((TWCR & _BV(TWINT)) == 0);
}

uint8_t i2c_read_ack(void)
{
	TWCR = _BV(TWINT) | _BV(TWEN) | _BV(TWEA);
	while((TWCR & _BV(TWINT)) == 0);
	return TWDR;
}

uint8_t i2c_read_nack(void)
{
	TWCR = _BV(TWINT) | _BV(TWEN);
	while((TWCR & _BV(TWINT)) == 0);
	return TWDR;
}

uint8_t i2c_status(void)
{
	uint8_t status;
	//mask status
	status = TWSR & 0xF8;
	return status;
}

int main(void)
{
	//static uint32_t cur_timer = 0;
	static uint32_t rec_timeout, rec_space_timeout;
	static uint8_t cur_character = '\0';
	static uint8_t rec_input, rec_count, rec_char_count;
	static char * cur_char_p;
	static char * cur_menu_p;
	static char * cur_menu;
	static char val_index;
	static enum BOOL rec_space_last;
	static enum MSGMEM rec_msg;

	announce_buffer = malloc(ANNOUNCE_BUFFER_SIZE);
	memset(announce_buffer, '\0', ANNOUNCE_BUFFER_SIZE);
	cur_char_p = announce_buffer;

	text_buffer = malloc(MSG_BUFFER_SIZE);
	memset(text_buffer, '\0', MSG_BUFFER_SIZE);

	init();

	if(cur_mode == MODE_CAL)
		announce("CAL", st_freq, 22);
	else
		announce("CC1", st_freq, 22);

	// Main event loop
	while(1)
	{
		// Latch the current time
		// MUST disable interrupts during this read or there will be an occasional corruption of cur_timer
		cli();
		cur_timer = timer;
		sei();

		// Handle the current mode
		switch(cur_mode)
		{
		case MODE_SK:
			default_mode = MODE_SK;
			poll_buttons();

			switch(cur_state)
			{
			case STATE_IDLE:
				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = FALSE;

				if(dit_active == TRUE)
				{
					tx_start = cur_timer + TX_ON_DELAY;
					tx_end = UINT32_MAX;
					cur_state_end = UINT32_MAX;
					cur_state = STATE_KEYDOWN;
				}
				else
				{
					cur_state = STATE_IDLE;
				}
				break;

			case STATE_KEYDOWN:
				if(tune_freq > UPPER_FREQ_LIMIT || tune_freq < LOWER_FREQ_LIMIT)
				{
					key_down = FALSE;
					sidetone_on = FALSE;
					mute_on = FALSE;
				}
				else
				{
					key_down = TRUE;
					sidetone_on = TRUE;
					mute_on = TRUE;
				}

				if(dit_active == FALSE)
				{
					cur_state = STATE_EXIT;
					cur_state_end = cur_timer;
					mute_end = cur_timer + MUTE_OFF_DELAY;
					tx_end = cur_timer + TX_OFF_DELAY;
				}
				else
					tx_end = UINT32_MAX;
				break;

			case STATE_EXIT:
				key_down = FALSE;
				sidetone_on = FALSE;

				if(tune_freq > UPPER_FREQ_LIMIT || tune_freq < LOWER_FREQ_LIMIT)
					mute_on = FALSE;
				else
					mute_on = TRUE;

				if(cur_timer >= cur_state_end)
					cur_state = STATE_IDLE;
				break;

			default:
				break;
			}

			/*
			// Go to sleep
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			cli();
			if((cur_mode == MODE_KEYER) && (cur_state == STATE_IDLE) && (cur_timer > sleep_timer))
			{
				MUTE_PORT &= ~(_BV(MUTE));
				PCICR = _BV(PCIE2);
				sleep_enable();
				sei();
				sleep_cpu();
				sleep_disable();
			}
			sei();
			*/

			break;

		case MODE_KEYER:
			default_mode = MODE_KEYER;
			poll_buttons();

			// Handle MODE_KEYER state conditions
			switch(cur_state)
			{
			case STATE_IDLE:
				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = FALSE;
				mute_end = cur_timer;

				// Dit paddle only
				if((dit_active == TRUE) && (dah_active == FALSE))
				{
					prev_state = STATE_IDLE;
					cur_state = STATE_DIT;
					next_state = STATE_IDLE;
					cur_state_end = cur_timer + dit_length;
					tx_start = cur_timer + TX_ON_DELAY;
					tx_end = cur_state_end + TX_OFF_DELAY;
					mute_start = cur_timer;
					mute_end = cur_state_end + MUTE_OFF_DELAY;
				}
				// Dah paddle only
				else if((dah_active == TRUE) && (dit_active == FALSE))
				{
					prev_state = STATE_IDLE;
					cur_state = STATE_DAH;
					next_state = STATE_IDLE;
					cur_state_end = cur_timer + (dit_length * 3);
					tx_start = cur_timer + TX_ON_DELAY;
					tx_end = cur_state_end + TX_OFF_DELAY;
					mute_start = cur_timer;
					mute_end = cur_state_end + MUTE_OFF_DELAY;
				}
				// Dit and dah paddle at same time (rare case)
				else if((dit_active == TRUE) && (dah_active == TRUE) && (next_state == STATE_IDLE))
				{
					prev_state = STATE_IDLE;
					cur_state = STATE_DIT;
					next_state = STATE_DAH;
					cur_state_end = cur_timer + dit_length;
					tx_start = cur_timer + TX_ON_DELAY;
					tx_end = cur_state_end + TX_OFF_DELAY;
					mute_start = cur_timer;
					mute_end = cur_state_end + MUTE_OFF_DELAY;
				}
				else
				{
					cur_state = STATE_IDLE;
				}

				break;

			case STATE_DIT:
				if(tune_freq > UPPER_FREQ_LIMIT || tune_freq < LOWER_FREQ_LIMIT)
				{
					key_down = FALSE;
					sidetone_on = FALSE;
					mute_on = FALSE;
				}
				else
				{
					key_down = TRUE;
					sidetone_on = TRUE;
					mute_on = TRUE;
				}

				if(cur_timer > cur_state_end)
				{
					prev_state = STATE_DIT;
					cur_state = STATE_DITDELAY;
					cur_state_end = cur_timer + dit_length;
					mute_start = cur_timer;
					mute_end = cur_state_end + MUTE_OFF_DELAY;
				}

				if((dah_active == TRUE) && (next_state == STATE_IDLE))
					next_state = STATE_DAH;

				break;

			case STATE_DAH:
				if(tune_freq > UPPER_FREQ_LIMIT || tune_freq < LOWER_FREQ_LIMIT)
				{
					key_down = FALSE;
					sidetone_on = FALSE;
					mute_on = FALSE;
				}
				else
				{
					key_down = TRUE;
					sidetone_on = TRUE;
					mute_on = TRUE;
				}

				if(cur_timer > cur_state_end)
				{
					prev_state = STATE_DAH;
					cur_state = STATE_DITDELAY;
					cur_state_end = cur_timer + dit_length;
					mute_start = cur_timer;
					mute_end = cur_state_end + MUTE_OFF_DELAY;
				}

				if((dit_active == TRUE) && (next_state == STATE_IDLE))
					next_state = STATE_DIT;

				break;

			case STATE_DITDELAY:
				if(cur_timer > cur_state_end)
				{
					if(next_state == STATE_DIT)
					{
						cur_state = STATE_DIT;
						cur_state_end = cur_timer + dit_length;
						tx_start = cur_timer + TX_ON_DELAY;
						tx_end = cur_state_end + TX_OFF_DELAY;
						mute_start = cur_timer;
						mute_end = cur_state_end + MUTE_OFF_DELAY;
					}
					else if(next_state == STATE_DAH)
					{
						cur_state = STATE_DAH;
						cur_state_end = cur_timer + (dit_length * 3);
						tx_start = cur_timer + TX_ON_DELAY;
						tx_end = cur_state_end + TX_OFF_DELAY;
						mute_start = cur_timer;
						mute_end = cur_state_end + MUTE_OFF_DELAY;
					}
					else
					{
						cur_state = STATE_IDLE;
						cur_state_end = cur_timer;
					}

					prev_state = STATE_DITDELAY;
					next_state = STATE_IDLE;
				}

				if((dit_active == TRUE) && (prev_state == STATE_DAH) && (next_state == STATE_IDLE))
					next_state = STATE_DIT;
				else if((dah_active == TRUE) && (prev_state == STATE_DIT) && (next_state == STATE_IDLE))
					next_state = STATE_DAH;

				key_down = FALSE;
				sidetone_on = FALSE;
				if(tune_freq > UPPER_FREQ_LIMIT || tune_freq < LOWER_FREQ_LIMIT)
					mute_on = FALSE;
				else
					mute_on = TRUE;
				break;

			case STATE_EXIT:
				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = TRUE;
				//mute_on = FALSE;

				if(cur_timer > cur_state_end)
				{
					cur_state = STATE_IDLE;
				}

				sleep_timer = cur_timer + SLEEP_DELAY;
				break;

			default:
				break;
			}

			/*
			// Go to sleep
			cli();
			if((cur_mode == MODE_KEYER) && (cur_state == STATE_IDLE) && (cur_timer > sleep_timer))
			{
				MUTE_PORT &= ~(_BV(MUTE));
				//SIDETONE_DDR &= ~(_BV(SIDETONE));
				PCICR = _BV(PCIE2);
				sleep_enable();
				sei();
				sleep_cpu();
				sleep_disable();
			}
			sei();
			*/

			break;

		case MODE_ANNOUNCE:
			switch(cur_state)
			{
			case STATE_IDLE:
				// If this is the first time thru the MODE_ANNOUNCE loop, get the first character
				if((cur_char_p == announce_buffer) && (cur_character == '\0'))
				{
					cur_character = pgm_read_byte(&morsechar[(*cur_char_p) - MORSE_CHAR_START]);
				}

				// Get the current element in the current character
				if(cur_character != '\0')
				{
					if(cur_character == 0b10000000 || cur_character == 0b11111111)	// End of character marker or SPACE
					{
						// Set next state based on whether EOC or SPACE
						if(cur_character == 0b10000000)
						{
							cur_state_end = cur_timer + (dit_length * 3);
							cur_state = STATE_DAHDELAY;
						}
						else
						{
							cur_state_end = cur_timer + (dit_length * 7);
							cur_state = STATE_DAHDELAY;
						}

						// Grab next character, set state to inter-character delay
						cur_char_p++;

						// If we read a NULL from the announce buffer, set cur_character to NULL,
						// otherwise set to correct morse character
						if((*cur_char_p) == '\0')
							cur_character = '\0';
						else
							cur_character = pgm_read_byte(&morsechar[(*cur_char_p) - MORSE_CHAR_START]);
					}
					else
					{
						// Mask off MSb, set cur_element
						if((cur_character & 0b10000000) == 0b10000000)
						{
							cur_state_end = cur_timer + (dit_length * 3);
							cur_state = STATE_DAH;
						}
						else
						{
							cur_state_end = cur_timer + dit_length;
							cur_state = STATE_DIT;
						}

						// Shift left to get next element
						cur_character = cur_character << 1;
					}
				}
				else
				{
					// Clear the announcement buffer and set buffer pointer back to beginning
					strcpy(announce_buffer, "");
					cur_char_p = announce_buffer;
					cur_character = '\0';

					// Set back into previous mode
					st_freq = prev_st_freq;
					set_st_freq(st_freq);

					wpm = prev_wpm;
					set_wpm(wpm);

					cur_mode = prev_mode;
					cur_state = prev_state;
					cur_state_end = prev_state_end;
				}
				break;

			case STATE_DIT:
			case STATE_DAH:
				if(cur_timer > cur_state_end)
				{
					cur_state_end = cur_timer + dit_length;
					cur_state = STATE_DITDELAY;
				}

				key_down = FALSE;
				sidetone_on = TRUE;
				mute_on = TRUE;
				break;

			case STATE_DITDELAY:
			case STATE_DAHDELAY:
			case STATE_WORDDELAY:
				if(cur_timer > cur_state_end)
					cur_state = STATE_IDLE;

				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = TRUE;
				break;

			default:
				break;
			}
			break;

		case MODE_TUNE:
			switch(cur_state)
			{
			case STATE_IDLE:
				tx_start = cur_timer + TX_ON_DELAY;
				tx_end = UINT32_MAX;
				cur_state_end = UINT32_MAX;
				mute_start = cur_timer;
				mute_end = UINT32_MAX;

				cur_state = STATE_KEYDOWN;
				break;

			case STATE_KEYDOWN:
				key_down = TRUE;
				sidetone_on = TRUE;
				mute_on = TRUE;

				if(ANYBUTTON)
				{
					cur_state = STATE_ENDKEYDOWN;
					cur_state_end = cur_timer + MUTE_OFF_DELAY;
					tx_end = cur_timer;
				}
				break;

			case STATE_ENDKEYDOWN:
				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = TRUE;

				if(cur_timer >= cur_state_end)
				{
					mute_on = FALSE;
					cur_state = STATE_IDLE;
					cur_mode = default_mode;
				}
				break;

			default:
				break;
			}
			break;

		case MODE_MENU:
			switch(cur_state)
			{
			case STATE_IDLE:
				// Point to the beginning of the menu
				cur_menu_p = menu;
				cur_state = STATE_MENUANNOUNCE;
				break;

			case STATE_MENUANNOUNCE:
				cur_menu = malloc(2);
				memset(cur_menu, '\0', 2);

				// Get the menu char (just 1 from the array)
				memcpy(cur_menu, cur_menu_p, 1);

				// Set menu input expiration
				cur_state_end = cur_timer + MENU_EXPIRATION;

				// Set next state
				cur_state = STATE_MENUINPUT;

				// Announce the menu item
				announce(cur_menu, st_freq, wpm);

				free(cur_menu);
				break;

			case STATE_MENUINPUT:
				text_buffer = malloc(20);
				memset(text_buffer, '\0', 20);

				// Wait for input
				if(cur_timer < cur_state_end)
				{
					// If CMD/FREQ pressed, advance to next menu item
					if(cmd_btn == BTN_PRESS)
					{
						debounce(TRUE);
						cur_menu_p++;
						// If at end of menu, return to previous mode
						if((*cur_menu_p) == '\0')
						{
							cur_state = STATE_IDLE;
							cur_mode = default_mode;

							announce("X", ST_LOW, wpm);
						}
						else
							cur_state = STATE_MENUANNOUNCE;
					}
					// If MSG/OK pressed, select this menu item
					else if(msg_btn == BTN_PRESS)
					{
						// need to clear the button buffer
						debounce(TRUE);

						switch(*cur_menu_p)
						{
						// Change keyer speed
						case 'S':
							cur_state_end = cur_timer + MENU_EXPIRATION;
							cur_mode = MODE_SETWPM;

							announce("R", ST_HIGH, 22);
							break;

						// Read WPM
						case 'W':
							cur_state = STATE_IDLE;
							cur_mode = default_mode;

							sprintf(text_buffer, "%d", wpm);
							announce(text_buffer, st_freq, wpm);
							break;

						// Record keyer memory 1
						case '1':
							cur_state = STATE_INIT;
							cur_mode = MODE_RECORD;
							rec_msg = MSGMEM_1;

							announce("R", ST_HIGH, 22);
							break;

						// Record keyer memory 2
						case '2':
							cur_state = STATE_INIT;
							cur_mode = MODE_RECORD;
							rec_msg = MSGMEM_2;

							announce("R", ST_HIGH, 22);
							break;

						// Read voltage
						case 'V':
							cur_state = STATE_IDLE;
							cur_mode = default_mode;

							read_voltage();
							break;

						// Toggle keyer/straight key mode
						case 'K':
							if(default_mode == MODE_KEYER)
							{
								default_mode = MODE_SK;
								cur_state = STATE_IDLE;
								cur_mode = default_mode;
								eeprom_busy_wait();
								eeprom_write_byte(&ee_keyer, FALSE);

								announce("S", ST_HIGH, wpm);
							}
							else
							{
								default_mode = MODE_KEYER;
								cur_state = STATE_IDLE;
								cur_mode = default_mode;
								eeprom_busy_wait();
								eeprom_write_byte(&ee_keyer, TRUE);

								announce("K", ST_HIGH, wpm);
							}
							break;

						default:
							break;
						}
					}
				}
				else // Bail out of menu if past menu expiration
				{
					cur_state = STATE_IDLE;
					cur_mode = default_mode;

					// Send "X" to indicate expiration
					announce("X", ST_LOW, wpm);
				}

				free(text_buffer);
				break;

			default:
				cur_state = STATE_IDLE;
				cur_mode = default_mode;
				break;
			}
			break;

		case MODE_SETWPM:
			if(cur_timer < cur_state_end)
			{
				if(cmd_btn == BTN_PRESS)
				{
					if(wpm < MAX_WPM)
						wpm++;
					set_wpm(wpm);
					cur_state_end = cur_timer + MENU_EXPIRATION;
					announce("I", st_freq, wpm);
				}
				else if(msg_btn == BTN_PRESS)
				{
					if(wpm > MIN_WPM)
						wpm--;
					set_wpm(wpm);
					cur_state_end = cur_timer + MENU_EXPIRATION;
					announce("I", st_freq, wpm);
				}
			}
			else // done setting WPM, announce current setting
			{
				// Save WPM in EEPROM
				eeprom_busy_wait();
				eeprom_write_byte(&ee_wpm, wpm);

				cur_state = STATE_IDLE;
				cur_mode = default_mode;

				sprintf(text_buffer, "%d", wpm);
				announce(text_buffer, st_freq, wpm);
			}
			break;

		// Consolidate with MODE_ANNOUNCE code
		case MODE_PLAYBACK:
			// Cancel playback if any button pressed
			if(ANYBUTTON)
			{
				// Clear the announcement buffer and set buffer pointer back to beginning
				strcpy(announce_buffer, "");
				cur_char_p = announce_buffer;
				cur_character = '\0';

				// Set back into previous mode
				mute_end = cur_timer;
				cur_mode = prev_mode;
				cur_state = prev_state;
				cur_state_end = prev_state_end;
			}

			switch(cur_state)
			{
			case STATE_IDLE:
				// If this is the first time thru the MODE_PLAYBACK loop, get the first character
				if((cur_char_p == announce_buffer) && (cur_character == '\0'))
				{
					cur_character = pgm_read_byte(&morsechar[(*cur_char_p) - MORSE_CHAR_START]);
				}

				// Get the current element in the current character
				if(cur_character != '\0')
				{
					if(cur_character == 0b10000000 || cur_character == 0b11111111)	// End of character marker or SPACE
					{
						// Set next state based on whether EOC or SPACE
						if(cur_character == 0b10000000)
						{
							cur_state_end = cur_timer + (dit_length * 3);
							cur_state = STATE_DAHDELAY;
						}
						else
						{
							cur_state_end = cur_timer + (dit_length * 7);
							cur_state = STATE_DAHDELAY;
						}

						// Grab next character, set state to inter-character delay
						cur_char_p++;

						// If we read a NULL from the announce buffer, set cur_character to NULL,
						// otherwise set to correct morse character
						if((*cur_char_p) == '\0')
							cur_character = '\0';
						else
							cur_character = pgm_read_byte(&morsechar[(*cur_char_p) - MORSE_CHAR_START]);
					}
					else
					{
						// Mask off MSb, set cur_element
						if((cur_character & 0b10000000) == 0b10000000)
						{
							cur_state_end = cur_timer + (dit_length * 3);
							cur_state = STATE_DAH;
							tx_start = cur_timer + TX_ON_DELAY;
							tx_end = cur_state_end;
							mute_start = cur_timer;
							mute_end = UINT32_MAX;
						}
						else
						{
							cur_state_end = cur_timer + dit_length;
							cur_state = STATE_DIT;
							tx_start = cur_timer + TX_ON_DELAY;
							tx_end = cur_state_end;
							mute_start = cur_timer;
							mute_end = UINT32_MAX;
						}

						// Shift left to get next element
						cur_character = cur_character << 1;
					}
				}
				else
				{
					// Clear the announcement buffer and set buffer pointer back to beginning
					strcpy(announce_buffer, "");
					cur_char_p = announce_buffer;
					cur_character = '\0';

					// Set back into previous mode
					mute_end = cur_timer;
					cur_mode = prev_mode;
					cur_state = prev_state;
					cur_state_end = prev_state_end;
				}
				break;

			case STATE_DIT:
			case STATE_DAH:
				if(cur_timer > cur_state_end)
				{
					cur_state_end = cur_timer + dit_length;
					cur_state = STATE_DITDELAY;
				}

				key_down = TRUE;
				sidetone_on = TRUE;
				mute_on = TRUE;
				break;

			case STATE_DITDELAY:
			case STATE_DAHDELAY:
			case STATE_WORDDELAY:
				if(cur_timer > cur_state_end)
					cur_state = STATE_IDLE;

				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = TRUE;
				break;

			default:
				break;
			}
			break;

			// TODO: Review current recording
			// TODO: Cancel recording
		case MODE_RECORD:
			switch(cur_state)
			{
			case STATE_INIT:
				mute_start = cur_timer;
				mute_end = UINT32_MAX;

				// Initialize the current recorded character
				rec_input = 0;
				rec_count = 0;
				rec_char_count = 0;
				rec_timeout = UINT32_MAX;
				rec_space_timeout = UINT32_MAX;
				rec_space_last = TRUE;

				strcpy(msg_buffer, "");

				cur_state = STATE_IDLE;
				break;

			case STATE_IDLE:
				// Dit paddle only
				if((dit_active == TRUE) && (dah_active == FALSE))
				{
					prev_state = STATE_IDLE;
					cur_state = STATE_DIT;
					next_state = STATE_IDLE;
					cur_state_end = cur_timer + dit_length;
					rec_timeout = cur_timer + REC_EXPIRATION;

					// Add this element to the recorded character
					rec_count++;
					if(rec_count > 8)
						next_state = STATE_VALIDATECHAR;
				}
				// Dah paddle only
				else if((dah_active == TRUE) && (dit_active == FALSE))
				{
					prev_state = STATE_IDLE;
					cur_state = STATE_DAH;
					next_state = STATE_IDLE;
					cur_state_end = cur_timer + (dit_length * 3);
					rec_timeout = cur_timer + REC_EXPIRATION;

					// Add this element to the recorded character
					rec_input = rec_input + (0b10000000 >> rec_count);
					rec_count++;
					if(rec_count > 8)
						next_state = STATE_VALIDATECHAR;
				}
				// Dit and dah paddle at same time (rare case)
				else if((dit_active == TRUE) && (dah_active == TRUE) && (next_state == STATE_IDLE))
				{
					prev_state = STATE_IDLE;
					cur_state = STATE_DIT;
					next_state = STATE_DAH;
					cur_state_end = cur_timer + dit_length;
					rec_timeout = cur_timer + REC_EXPIRATION;

					// Add this element to the recorded character
					rec_input = rec_input + (0b10000000 >> (rec_count + 1));
					rec_count = rec_count + 2;
					if(rec_count > 8)
						next_state = STATE_VALIDATECHAR;
				}
				else
				{
					//rec_space_timeout = cur_timer + REC_SPACE_EXPIRATION;
				}

				// Handle character record timeout
				// Need to handle SPACE
				if((cur_timer > rec_timeout) && (rec_count > 0))
					cur_state = STATE_VALIDATECHAR;
				else if((cur_timer > rec_space_timeout) && (rec_count == 0) && (rec_space_last == FALSE))
				{
					cur_state = STATE_VALIDATECHAR;
				}

				// If CMD is pressed, we are done recording
				if(cmd_btn == BTN_PRESS)
					cur_state = STATE_EXIT;

				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = TRUE;
				break;

			case STATE_DIT:
				if(cur_timer > cur_state_end)
				{
					prev_state = STATE_DIT;
					cur_state = STATE_DITDELAY;
					cur_state_end = cur_timer + dit_length;
				}

				if((dah_active == TRUE) && (next_state == STATE_IDLE))
				{
					next_state = STATE_DAH;
					rec_timeout = cur_timer + REC_EXPIRATION;
					rec_space_timeout = cur_timer + REC_SPACE_EXPIRATION;

					// Add this element to the recorded character
					rec_input = rec_input + (0b10000000 >> rec_count);
					rec_count++;
					if(rec_count > 8)
						next_state = STATE_VALIDATECHAR;
				}

				key_down = FALSE;
				sidetone_on = TRUE;
				mute_on = TRUE;
				break;

			case STATE_DAH:
				if(cur_timer > cur_state_end)
				{
					prev_state = STATE_DAH;
					cur_state = STATE_DITDELAY;
					cur_state_end = cur_timer + dit_length;
				}

				if((dit_active == TRUE) && (next_state == STATE_IDLE))
				{
					next_state = STATE_DIT;
					rec_timeout = cur_timer + REC_EXPIRATION;
					rec_space_timeout = cur_timer + REC_SPACE_EXPIRATION;

					// Add this element to the recorded character
					rec_count++;
					if(rec_count > 8)
						next_state = STATE_VALIDATECHAR;
				}

				key_down = FALSE;
				sidetone_on = TRUE;
				mute_on = TRUE;
				break;

			case STATE_DITDELAY:
				if(cur_timer > cur_state_end)
				{
					if(next_state == STATE_DIT)
					{
						cur_state = STATE_DIT;
						cur_state_end = cur_timer + dit_length;
					}
					else if(next_state == STATE_DAH)
					{
						cur_state = STATE_DAH;
						cur_state_end = cur_timer + (dit_length * 3);
					}
					else if(next_state == STATE_VALIDATECHAR)
						cur_state = STATE_VALIDATECHAR;
					else
					{
						cur_state = STATE_IDLE;
						rec_space_timeout = cur_timer + REC_SPACE_EXPIRATION;
					}

					prev_state = STATE_DITDELAY;
					next_state = STATE_IDLE;
				}

				if((dit_active == TRUE) && (prev_state == STATE_DAH) && (next_state == STATE_IDLE))
				{
					next_state = STATE_DIT;
					rec_timeout = cur_timer + REC_EXPIRATION;
					rec_space_timeout = cur_timer + REC_SPACE_EXPIRATION;

					// Add this element to the recorded character
					rec_count++;
					if(rec_count > 8)
						next_state = STATE_VALIDATECHAR;
				}
				else if((dah_active == TRUE) && (prev_state == STATE_DIT) && (next_state == STATE_IDLE))
				{
					next_state = STATE_DAH;
					rec_timeout = cur_timer + REC_EXPIRATION;
					rec_space_timeout = cur_timer + REC_SPACE_EXPIRATION;

					// Add this element to the recorded character
					rec_input = rec_input + (0b10000000 >> rec_count);
					rec_count++;
					if(rec_count > 8)
						next_state = STATE_VALIDATECHAR;
				}

				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = TRUE;
				break;

			case STATE_VALIDATECHAR:
				// Compare recorded character with the Morse Code table

				// If rec_input is 0, dump to invalid

				// Tack a trailing "1" onto rec_input to indicate end of character
				rec_input = rec_input + (0b10000000 >> rec_count);

				if(rec_count == 0)
				{
					// Insert a space into the message buffer
					strncat(msg_buffer, " ", 1);
					rec_space_last = TRUE;
					rec_input = 0;
					rec_count = 0;
					cur_state = STATE_IDLE;
					rec_space_timeout = cur_timer + REC_SPACE_EXPIRATION;

					if(++rec_char_count >= (MSG_BUFFER_SIZE - 1))
						cur_state = STATE_EXIT;

					announce("E", ST_HIGH, 22);

				}
				else if(rec_input == 0b00000000 && rec_count >= 8) // erase last character
				{
					if(rec_char_count > 0)
					{
						strncpy(text_buffer, msg_buffer, --rec_char_count);
						strcpy(msg_buffer, text_buffer);

						// Reinitialize the current recorded character
						rec_input = 0;
						rec_count = 0;
						rec_space_last = TRUE;
						cur_state = STATE_IDLE;
						rec_space_timeout = cur_timer + REC_SPACE_EXPIRATION;

						announce("T", ST_LOW, 22);
					}
					else
					{
						// Reinitialize the current recorded character
						rec_input = 0;
						rec_count = 0;
						rec_space_last = TRUE;
						cur_state = STATE_IDLE;
						rec_space_timeout = cur_timer + REC_SPACE_EXPIRATION;
						announce("X", ST_LOW, 22);
					}
				}
				else
				{
					for(val_index = MORSE_CHAR_START; val_index <= 'Z'; val_index++)
					{
						if(rec_input == pgm_read_byte(&morsechar[val_index - MORSE_CHAR_START]))
						{
							// Add recorded character to text buffer
							strncat(msg_buffer, &val_index, 1);

							// Reinitialize the current recorded character
							rec_input = 0;
							rec_count = 0;
							rec_space_last = FALSE;
							cur_state = STATE_IDLE;
							rec_space_timeout = cur_timer + REC_SPACE_EXPIRATION;


							if(++rec_char_count >= (MSG_BUFFER_SIZE - 1))
								cur_state = STATE_EXIT;

							// Indicate successful entry
							announce("E", ST_HIGH, 22);

							break;
						}
					}

					// If no match, the character isn't valid. Toss it out and announce error
					// No match if rec_input is not reset to 0
					if(val_index > 'Z')
					{
						// Reinitialize the current recorded character
						rec_input = 0;
						rec_count = 0;
						cur_state = STATE_IDLE;
						rec_space_timeout = cur_timer + REC_SPACE_EXPIRATION;

						// Indicate an error
						announce("X", ST_LOW, 22);
					}
				}

				//cur_state = STATE_IDLE;
				//rec_space_timeout = cur_timer + REC_SPACE_EXPIRATION;

				break;

			case STATE_EXIT:
				// Get rid of trailing space
				if(rec_space_last == TRUE)
				{
					strncpy(text_buffer, msg_buffer, --rec_char_count);
					strcpy(msg_buffer, text_buffer);
				}

				// Write the memory to EEPROM
				if(rec_msg == MSGMEM_1)
					eeprom_write_block((const void*)&msg_buffer, (void*)&ee_msg_mem_1, MSG_BUFFER_SIZE);
				else if(rec_msg == MSGMEM_2)
					eeprom_write_block((const void*)&msg_buffer, (void*)&ee_msg_mem_2, MSG_BUFFER_SIZE);

				// Unmute and reset back to default mode
				mute_end = cur_timer;
				mute_on = FALSE;
				cur_state = STATE_IDLE;
				cur_mode = default_mode;

				// Repeat the message buffer
				announce(msg_buffer, st_freq, wpm);

				break;

			default:
				break;
			}
			break;


		// Don't really need to do anything here except let the RX tune
		case MODE_CAL:
			default_mode = MODE_CAL;
			poll_buttons();

			switch(cur_state)
			{
			case STATE_IDLE:
				cur_state = STATE_IDLE;
				cur_state_end = UINT32_MAX;
				key_down = FALSE;
				sidetone_on = FALSE;
				mute_on = FALSE;
				mute_end = cur_timer;

				inc_tune_state = INC_NONE;
				break;

			default:
				cur_state = STATE_IDLE;
				break;
			}
			break;

		default:
			break;
		} // END switch(cur_mode)

	}
}

