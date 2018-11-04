// HD POV Version for XC8
// PIC18F1320 Configuration Bit Settings 

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits 40MHz fosc with PLL and 10MHz clock input
#pragma config FSCM = ON        // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal External Switchover mode enabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bit (Brown-out Reset enabled)
// BORV = No Setting

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit 
#pragma config WDTPS = 4096    // Watchdog Timer Postscale Select bits 

// CONFIG3H
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled, RA5 input pin disabled)

// CONFIG4L
#pragma config STVR = ON        // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Low-Voltage ICSP Enable bit (Low-Voltage ICSP disabled)

// CONFIG5L
#pragma config CP0 = ON        // Code Protection bit (Block 0 (00200-000FFFh) not code-protected)
#pragma config CP1 = ON        // Code Protection bit (Block 1 (001000-001FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (00200-000FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (001000-001FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (00200-000FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (001000-001FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from table reads executed in other blocks)


/*
 * Driver for hard-drive strobe for POV demo
 * Versions
 * 1.0 RGB support
 * 1.1 multi sequence support
 * 1.2 cleanup state machine and data logic
 * 1.3 add routines for remote configuration of strobes
 * 1.4 add buffering for rs232
 * 1.5 cleanup remote data handling
 * 1.6 Beta version
 * 1.7 release cleanup
 * 2.0 convert to xc8 v2.00 c99
 */

#include  <xc.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "pov_mon.h"
#include <string.h>
#include "ringbufs.h"

int16_t sw_work(void);
void init_povmon(void);
uint8_t init_hov_params(void);

struct V_data V = {0};
struct L_data L[strobe_max] = {0}, *L_ptr;

/* RS232 command buffer */
struct ringBufS_t ring_buf1;

const char build_date[] = __DATE__, build_time[] = __TIME__, versions[] = "2.00";
const uint16_t TIMEROFFSET = 18000, TIMERDEF = 60000;

void __interrupt() tm_handler(void) // timer/serial functions are handled here
{
	LED1 = 1;
	// line rotation sequencer
	if (INTCONbits.INT0IF) { // Hall effect index signal, start of rotation
		INTCONbits.INT0IF = false;
		RPMLED = (uint8_t)!RPMLED;
		if (V.l_state == ISR_STATE_LINE) { // off state too long for full rotation, hall signal while in state
			V.l_full += strobe_adjust; // off state lower limit adjustments for smooth strobe rotation
		}
		V.l_state = ISR_STATE_FLAG; // restart lamp flashing sequence, off time

		L_ptr = &L[V.line_num]; // select line strobe data
		V.rotations++;

		/* limit rotational timer values during offsets */
		switch (L_ptr->sequence.down) {
		case false:
			L_ptr->strobe += L_ptr->sequence.offset;
			if (L_ptr->strobe < V.l_full)
				L_ptr->strobe = V.l_full; // set to sliding lower limit
			break;
		case true:
			L_ptr->strobe -= L_ptr->sequence.offset;
			if (L_ptr->strobe < V.l_full)
				L_ptr->strobe = strobe_limit_h;
			break;

		default:
			L_ptr->strobe -= L_ptr->sequence.offset;
			if (L_ptr->strobe < V.l_full)
				L_ptr->strobe = strobe_limit_h;
			break;
		}
		V.line_num++;
		if (L_ptr->sequence.end || (V.line_num >= strobe_max)) { // rollover for sequence patterns
			V.line_num = 0;
			V.sequences++;
		}
	}

	// line RGB pulsing state machine
	if (PIR1bits.TMR1IF || (V.l_state == ISR_STATE_FLAG)) { // Timer1 int handler, for strobe rotation timing
		PIR1bits.TMR1IF = false;

		switch (V.l_state) {
		case ISR_STATE_FLAG:
			WRITETIMER1(L_ptr->strobe); // strobe positioning during rotation
			T1CONbits.TMR1ON = 1;
			G_OUT = 0;
			R_OUT = 0;
			B_OUT = 0;
			V.l_state = ISR_STATE_LINE; // off time after index to start time
			break;
		case ISR_STATE_LINE:
			WRITETIMER1(V.l_width);
			if (!L_ptr->sequence.skip) {
				if (L_ptr->sequence.R)
					R_OUT = 1;
				if (L_ptr->sequence.G)
					G_OUT = 1;
				if (L_ptr->sequence.B)
					B_OUT = 1;
			}

			V.l_state = ISR_STATE_WAIT; // on start time duration for strobe pulse
			break;
		case ISR_STATE_WAIT: // waiting for next HALL sensor pulse
		default:
			T1CONbits.TMR1ON = 0; // idle timer
			G_OUT = 0; // blank RGB
			R_OUT = 0;
			B_OUT = 0;
			break;
		}
	}

	// remote command data buffer
	if (PIR1bits.RCIF) { // is data from RS-232 port
		V.rx_data = RCREG; // save in state machine register
		if (RCSTAbits.OERR) {
			RCSTAbits.CREN = 0; // clear overrun
			RCSTAbits.CREN = 1; // re-enable
		}
		ringBufS_put(&ring_buf1, V.rx_data); // buffer RS232 data
	}

	// check timer0 for blinker led
	if (INTCONbits.TMR0IF) {
		INTCONbits.TMR0IF = false;
		WRITETIMER0(TIMEROFFSET);
		LED5 = (uint8_t)!LED5; // active LED blinker
	}
	LED1 = 0;
}

void USART_putc(uint8_t c)
{
	while (!TXSTAbits.TRMT);
	TXREG = c;
}

void USART_puts(uint8_t *s)
{
	while (*s) {
		USART_putc(*s);
		s++;
	}
}

void USART_putsr(const char *s)
{
	while (*s) {
		USART_putc(*s);
		s++;
	}
}

void puts_ok(uint16_t size)
{
//	itoa(V.str, size, 10);
	USART_putsr("\r\n OK");
	USART_puts(V.str); // send size of data array
}

/* main loop work routine */
int16_t sw_work(void)
{
	static uint8_t position = 0, offset = 0, rx_data;
	static uint8_t *L_tmp_ptr;

	static union L_union_type { // so we can access each byte of the command structure
		uint8_t L_bytes[sizeof(L[0]) + 1];
		L_data L_tmp;
	} L_union;
	int16_t ret = 0;

	if (V.l_state != ISR_STATE_WAIT)
		ret = -1;

	if (!SW1) {
		USART_putsr("\r\n Timer limit,");
//		itoa(V.str, V.l_full, 10);
		USART_puts(V.str);
		USART_putsr(" Timer value,");
//		itoa(V.str, L_ptr->strobe, 10);
		USART_puts(V.str);
	}

	/* command state machine 
	 * u/U update the current display buffer with remote RS232 data
	 * d/D display the current display buffer on RS232 port
	 * e/E clear/set end of lines flag on display buffer
	 * i/I timer info command
	 * z/Z null command
	 */
	if (!ringBufS_empty(&ring_buf1)) {
		rx_data = ringBufS_get(&ring_buf1);
		switch (V.comm_state) {
		case APP_STATE_INIT:
			switch (rx_data) {
			case 'u':
			case 'U':
				V.comm_state = APP_STATE_WAIT_FOR_UDATA;
				break;
			case 'd':
			case 'D':
				V.comm_state = APP_STATE_WAIT_FOR_DDATA;
				break;
			case 'e':
				V.comm_state = APP_STATE_WAIT_FOR_eDATA;
				puts_ok(V.l_size);
				break;
			case 'E':
				V.comm_state = APP_STATE_WAIT_FOR_EDATA;
				puts_ok(V.l_size);
				break;
			case 'i':
			case 'I': // info command
				USART_putsr(" Timer limit,");
//				itoa(V.str, V.l_full, 10);
				USART_puts(V.str);
				USART_putsr(" OK");
				break;
			case 'z':
			case 'Z': // null command for fillers, silent
				break;
			default:
				USART_putsr("\r\n NAK_I");
				ret = -1;
				break;
			}
			break;
		case APP_STATE_WAIT_FOR_eDATA:
		case APP_STATE_WAIT_FOR_EDATA:
		case APP_STATE_WAIT_FOR_DDATA:
		case APP_STATE_WAIT_FOR_UDATA:
			position = rx_data;
			if (position >= strobe_max) {
				USART_putsr(" NAK_P");
				V.comm_state = APP_STATE_INIT;
				ret = -1;
				break;
			}
			offset = 0;
			switch (V.comm_state) {
			case APP_STATE_WAIT_FOR_UDATA:
				V.comm_state = APP_STATE_WAIT_FOR_RDATA;
				break;
			case APP_STATE_WAIT_FOR_DDATA:
				V.comm_state = APP_STATE_WAIT_FOR_SDATA;
				break;
			case APP_STATE_WAIT_FOR_eDATA:
				INTCONbits.GIEH = 0;
				L[position].sequence.end = 0; // clear end flag
				INTCONbits.GIEH = 1;
				V.comm_state = APP_STATE_WAIT_FOR_SDATA;
				break;
			case APP_STATE_WAIT_FOR_EDATA:
				INTCONbits.GIEH = 0;
				L[position].sequence.end = 1; // set end flag
				INTCONbits.GIEH = 1;
				V.comm_state = APP_STATE_WAIT_FOR_SDATA;
				break;
			default:
				break;
			}
			USART_putsr(" OK");
			break;
		case APP_STATE_WAIT_FOR_RDATA: // receive
			L_union.L_bytes[offset] = rx_data;
			offset++;
			if (offset >= sizeof(L_union.L_tmp)) {
				INTCONbits.GIEH = 0;
				L[position] = L_union.L_tmp;
				INTCONbits.INT0IF = false;
				INTCONbits.GIEH = 1;
				USART_putsr(" OK,");
//				utoa(V.str, (uint16_t) L_union.L_tmp.strobe, 10);
				USART_puts(V.str);
				V.comm_state = APP_STATE_INIT;
			}
			break;
		case APP_STATE_WAIT_FOR_SDATA: // send
			L_tmp_ptr = (void*) &L[position]; // set array start position
			do { // send ascii data to the rs232 port
				USART_putsr(" ,");
				if (offset) {
//					itoa(V.str, *L_tmp_ptr, 16); // show hex
				} else {
//					itoa(V.str, *L_tmp_ptr, 2); // show bits
				}
				USART_puts(V.str);
				L_tmp_ptr++;
				offset++;
			} while (offset < V.l_size);
			V.comm_state = APP_STATE_INIT;
			USART_putsr(" OK");
			break;
		default:
			USART_putsr(" NAK_C");
			V.comm_state = APP_STATE_INIT;
			if (ringBufS_full(&ring_buf1))
				ringBufS_flush(&ring_buf1, 0);
			ret = -1;
			break;
		}
	}

	return ret;
}

/* controller hardware setup */
void init_povmon(void)
{
	/*
	 * check for a clean POR
	 */
	V.boot_code = false;
	if (RCON != 0b0011100)
		V.boot_code = true;

	if (STKPTRbits.STKFUL || STKPTRbits.STKUNF) {
		V.boot_code = true;
		STKPTRbits.STKFUL = 0;
		STKPTRbits.STKUNF = 0;
	}

	ADCON1 = 0x7F; // all digital, no ADC
	/* interrupt priority ON */
	RCONbits.IPEN = 1;
	/* define I/O ports */
	RMSPORTA = RMSPORT_IOA;
	RMSPORTB = RMSPORT_IOB;

	G_OUT = OFF; // preset all LEDS
	LED1 = OFF;
	LED2 = OFF;
	LED3 = OFF;
	LED4 = OFF;
	LED5 = OFF;
	LED6 = OFF;
	RPMLED = OFF;
	//	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256); // led blinker
	T0CON = 0b10000111;
	WRITETIMER0(TIMEROFFSET); //	start timer0 at ~1/2 second ticks
	//	OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_2 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF); // strobe position clock
	T1CON = 0b10010101;
	WRITETIMER1(TIMERDEF);
	/* data link */
	COMM_ENABLE = true; // for PICDEM4 onboard RS-232, not used on custom boards
	TXSTAbits.TXEN = 1;
	RCSTAbits.CREN = 1;
	RCSTAbits.SPEN = 1;
	TXSTAbits.SYNC = 0;
	TXSTAbits.SYNC = 0;
	TXSTAbits.BRGH = 0;
	BAUDCTLbits.BRG16 = 1;
	SPBRG = 129; /* 19200 baud */

	/*      work int thread setup */
	INTCONbits.TMR0IE = 1; // enable int
	INTCON2bits.TMR0IP = 1; // make it high P

	/* rotation timer */
	PIE1bits.TMR1IE = 1;
	IPR1bits.TMR1IP = 1;

	INTCONbits.INT0IE = 1; // enable RPM sensor input
	INTCON2bits.INTEDG0 = 0; // falling edge trigger
	INTCON2bits.RBPU = 0; // enable weak pull-ups

	PIE1bits.RCIE = 1; // enable rs232 serial receive interrupts
	IPR1bits.RCIP = 1;

	init_hov_params();
	ringBufS_init(&ring_buf1);

	/* Enable all high priority interrupts */
	INTCONbits.GIEH = 1;
}

/* program data setup */
uint8_t init_hov_params(void)
{
	V.line_num = 0;
	V.comm_state = APP_STATE_INIT;
	V.l_size = sizeof(L[0]);
	V.l_state = ISR_STATE_WAIT;
	V.l_full = strobe_limit_l;
	V.l_width = strobe_line;

	USART_putsr("\r\nVersion ");
	USART_putsr(versions);
	USART_putsr(", ");
//	itoa(V.str, sizeof(L[0]), 10);
	USART_puts(V.str);
	USART_putsr(", ");
	USART_putsr(build_date);
	USART_putsr(", ");
	USART_putsr(build_time);
	if (V.boot_code)
		USART_putsr(", dirty boot");

	L_ptr = &L[0];
	/* three line strobes in 3 16-bit timer values for spacing */
	/* for an interrupt driven state machine */
	L[0].strobe = 60000;
	L[0].sequence.R = 1;
	L[0].sequence.offset = strobe_up;

	L[1].strobe = 50000; // 62000
	L[1].sequence.G = 1;
	L[1].sequence.offset = strobe_down;

	L[2].strobe = 40000;
	L[2].sequence.B = 1;
	L[2].sequence.offset = strobe_around;

	L[3].strobe = 30000;
	L[3].sequence.R = 1;
	L[3].sequence.G = 1;
	L[3].sequence.B = 1;
	L[3].sequence.offset = 0;
	L[3].sequence.end = 1;

	L[strobe_max - 1].sequence.end = 1;
	return 0;
}

void main(void)
{
	/* configure system */
	init_povmon();

	/* Loop forever */
	while (true) { // busy work
		sw_work(); // run housekeeping for non-ISR tasks
	}
}
