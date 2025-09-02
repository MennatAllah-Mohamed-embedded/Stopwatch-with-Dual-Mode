/*
 * stopwatch_with_dual_mode.c
 * Cleaned version - same functionality
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// stopwatch modes
#define COUNT_UP   1
#define COUNT_DOWN 0

// stopwatch digits
short sec_ones=0, sec_tens=0;
short min_ones=0, min_tens=0;
short hour_ones=0, hour_tens=0;

// mode flag
unsigned char mode = COUNT_UP;

// ---------------- Initialization ----------------
void timer1_ctc(void) {
	TCNT1 = 0;
	OCR1A = 15625;                           // 1 second at 16MHz / 1024
	TIMSK |= (1<<OCIE1A);                    // enable compare A interrupt
	TCCR1A = (1<<FOC1A);                     // CTC non-PWM
	TCCR1B = (1<<WGM12) | (1<<CS12) | (1<<CS10); // CTC, prescaler=1024
}

void INT0_Init (void) {                     // Reset
	DDRD  &= ~(1<<PD2);
	PORTD |= (1<<PD2);
	MCUCR |= (1<<ISC01);                     // falling edge
	GICR  |= (1<<INT0);
}

void INT1_Init (void) {                     // Pause
	DDRD  &= ~(1<<PD3);
	MCUCR |= (1<<ISC11)|(1<<ISC10);          // rising edge
	GICR  |= (1<<INT1);
}

void INT2_Init (void) {                     // Resume
	DDRB  &= ~(1<<PB2);
	PORTB |= (1<<PB2);
	MCUCSR &= ~(1<<ISC2);                    // falling edge
	GICR  |= (1<<INT2);
}

// ---------------- ISRs ----------------
ISR(TIMER1_COMPA_vect) {
	if (mode == COUNT_UP) {
		sec_ones++;
		if (sec_ones == 10) { sec_ones = 0; sec_tens++; }
		if (sec_tens == 6)  { sec_tens = 0; min_ones++; }
		if (min_ones == 10) { min_ones = 0; min_tens++; }
		if (min_tens == 6)  { min_tens = 0; hour_ones++; }
		if (hour_ones == 10){ hour_ones = 0; hour_tens++; }
	}
	else { // COUNT_DOWN
		if (!(sec_ones==0 && sec_tens==0 && min_ones==0 && min_tens==0 && hour_ones==0 && hour_tens==0)) {
			sec_ones--;
			if (sec_ones < 0) { sec_ones=9; sec_tens--; }
			if (sec_tens < 0) { sec_tens=5; min_ones--; }
			if (min_ones < 0) { min_ones=9; min_tens--; }
			if (min_tens < 0) { min_tens=5; hour_ones--; }
			if (hour_ones < 0){ hour_ones=9; hour_tens--; }
		}
		else {
			PORTD |= (1<<PD0);                 // buzzer on
			TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10)); // stop timer
		}
	}
}

ISR(INT0_vect) {                            // Reset
	sec_ones=sec_tens=0;
	min_ones=min_tens=0;
	hour_ones=hour_tens=0;
	PORTD &= ~(1<<PD0);                      // buzzer off
}

ISR(INT1_vect) {                            // Pause
	TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10));
}

ISR(INT2_vect) {                            // Resume
	TCCR1B |= (1<<CS12) | (1<<CS10);
}

// ---------------- Main ----------------
int main(void) {
	// outputs
	DDRC |= (1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3);  // BCD to 7-seg
	DDRA |= (1<<PA0)|(1<<PA1)|(1<<PA2)|(1<<PA3)|(1<<PA4)|(1<<PA5); // enables
	DDRD |= (1<<PD4)|(1<<PD5)|(1<<PD0);           // LEDs + buzzer

	// inputs
	DDRB = 0x00;
	PORTB = 0xFF;                                 // pull-ups
	PORTD |= (1<<PD2);                            // INT0 pull-up

	// default LED (count-up mode)
	PORTD |= (1<<PD4);
	PORTD &= ~(1<<PD5);

	// flags
	unsigned char mode_toggle=0;
	unsigned char dec_sec=0, inc_sec=0;
	unsigned char dec_min=0, inc_min=0;
	unsigned char dec_hour=0, inc_hour=0;

	// init
	INT0_Init();
	INT1_Init();
	INT2_Init();
	timer1_ctc();
	sei();

	while(1) {
		// toggle mode
		if (!(PINB & (1<<PB7))) {
			if (mode_toggle==0) {
				mode = (mode==COUNT_UP) ? COUNT_DOWN : COUNT_UP;
				PORTD &= ~(1<<PD0); // buzzer off
				mode_toggle=1;
			}
		} else { mode_toggle=0; }

		// LEDs
		if (mode==COUNT_UP) { PORTD|=(1<<PD4); PORTD&=~(1<<PD5); }
		else                { PORTD|=(1<<PD5); PORTD&=~(1<<PD4); }

		// seconds
		if (!(PINB & (1<<PB5))) { if (!dec_sec) { sec_ones--; if(sec_ones<0){ sec_ones=9; if(sec_tens>0) sec_tens--; } dec_sec=1; } }
		else dec_sec=0;

		if (!(PINB & (1<<PB6))) { if (!inc_sec) { sec_ones++; if(sec_ones==10){ sec_ones=0; sec_tens++; if(sec_tens==6) sec_tens=0; } inc_sec=1; } }
		else inc_sec=0;

		// minutes
		if (!(PINB & (1<<PB3))) { if (!dec_min) { min_ones--; if(min_ones<0){ min_ones=9; if(min_tens>0) min_tens--; } dec_min=1; } }
		else dec_min=0;

		if (!(PINB & (1<<PB4))) { if (!inc_min) { min_ones++; if(min_ones==10){ min_ones=0; min_tens++; if(min_tens==6) min_tens=0; } inc_min=1; } }
		else inc_min=0;

		// hours
		if (!(PINB & (1<<PB0))) { if (!dec_hour) { hour_ones--; if(hour_ones<0){ hour_ones=9; if(hour_tens>0) hour_tens--; } dec_hour=1; } }
		else dec_hour=0;

		if (!(PINB & (1<<PB1))) { if (!inc_hour) { hour_ones++; if(hour_ones==10){ hour_ones=0; hour_tens++; } inc_hour=1; } }
		else inc_hour=0;

		// display
		PORTA=0;
		PORTC = (PORTC&0xF0)|(hour_tens&0x0F); PORTA|=(1<<PA0); _delay_ms(2); PORTA&=~(1<<PA0);
		PORTC = (PORTC&0xF0)|(hour_ones&0x0F); PORTA|=(1<<PA1); _delay_ms(2); PORTA&=~(1<<PA1);
		PORTC = (PORTC&0xF0)|(min_tens&0x0F);  PORTA|=(1<<PA2); _delay_ms(2); PORTA&=~(1<<PA2);
		PORTC = (PORTC&0xF0)|(min_ones&0x0F);  PORTA|=(1<<PA3); _delay_ms(2); PORTA&=~(1<<PA3);
		PORTC = (PORTC&0xF0)|(sec_tens&0x0F);  PORTA|=(1<<PA4); _delay_ms(2); PORTA&=~(1<<PA4);
		PORTC = (PORTC&0xF0)|(sec_ones&0x0F);  PORTA|=(1<<PA5); _delay_ms(2); PORTA&=~(1<<PA5);
	}
}
