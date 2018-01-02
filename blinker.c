#include "blinker.h"

uint8_t blink_led(uint8_t led, uint8_t on_led, uint8_t start) // blink and store status of 8 leds 
{
	if (led > 7) return LEDS.out_byte;

	if (on_led) {
		V.blink_out &= ~(0x0001 << led); // reset the bit 
	} else {
		V.blink_out |= (0x0001 << led); // set the bit 
	}

	if (start) {
		V.blink |= (0x0001 << led); // set the blink bit for the LED
	} else {
		V.blink &= ~(0x0001 << led); // reset the blink bit for the LED
		LEDS.out_byte &= ~(0x01 << led); // reset the LEDS bit for the LED
		LEDS.out_byte |= (((V.blink_out >> led)&0x01) << led); // restore the LEDS bit status to the on_led state
	}
	return LEDS.out_byte;
}

uint8_t is_led_blinking(uint8_t led)
{
	return(V.blink >> led)&0b00000001;
}

uint8_t blink_led_alt(uint8_t alt) // set alt and returns the previous value
{
	uint8_t old_alt = V.blink_alt;
	V.blink_alt = alt;
	return old_alt;
}

uint8_t is_led_on(uint8_t led)
{
	return(V.blink_out >> led)&0b00000001;
}

void Blink_Init(void)
{
	LEDS.out_byte = 0xff;
	V.blink = 0;
	V.blink_out = 0;
	V.blink_alt = 0;
	// High priority interrupt in ISR for Blink timer
}

