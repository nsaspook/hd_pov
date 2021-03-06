
// PIC18F1320 Configuration Bit Settings 

#include <p18f1320.h>

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits 40MHz fosc with PLL and 10MHz clock input
#pragma config FSCM = ON        // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = ON        // Internal External Switchover bit (Internal External Switchover mode enabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bit (Brown-out Reset enabled)
// BORV = No Setting

// CONFIG2H
#pragma config WDT = ON        // Watchdog Timer Enable bit 
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
 */

#include <timers.h>
#include <stdlib.h>
#include <usart.h>
#include <stdio.h>
#include <EEP.h>
#include "pat.h"
#include <string.h>

void tm_handler(void);
int16_t sw_work(void);
void init_rmsmon(void);
uint8_t init_rms_params(void);

#pragma udata
int8_t str[24];
near struct L_data *L_ptr;
#pragma udata access ACCESSBANK
near volatile struct V_data V;
volatile uint16_t timer0_off = TIMEROFFSET;
near volatile struct L_data L[2];
volatile uint8_t l_state = 2;
volatile uint16_t l_full = strobe_limit_l;

const far rom int8_t build_date[] = __DATE__, build_time[] = __TIME__;

#pragma code tm_interrupt = 0x8

void tm_int(void)
{
	_asm goto tm_handler _endasm
}
#pragma code

#pragma interrupt tm_handler

void tm_handler(void) // timer/serial functions are handled here
{

	if (INTCONbits.INT0IF) { // Hall effect index signal, start of rotation
		INTCONbits.INT0IF = FALSE;
		RPMLED = !RPMLED;
		if (l_state == 1) { // off state too long for full rotation, hall signal while in state 1
			l_full += strobe_adjust; // off state lower limit adjustments for smooth strobe rotation
		}
		l_state = 0; // restart lamp flashing sequence, off time

		L_ptr = &L[V.line_num]; // select line strobe data
		
		/* limit rotational timer values */
		switch (V.line_num) {
		case 0:
			L_ptr->strobe[0] -= strobe_down; // start sliding the positions
			if (L_ptr->strobe[0] < l_full)
				L_ptr->strobe[0] = strobe_limit_h; // set to upper limit rollover
			break;
		case 1:
			L_ptr->strobe[0] += strobe_up;
			if (L_ptr->strobe[0] < l_full)
				L_ptr->strobe[0] = l_full; // set to sliding lower limit
			break;
		default:
			L_ptr->strobe[0] -= strobe_down;
			if (L_ptr->strobe[0] < l_full)
				L_ptr->strobe[0] = strobe_limit_h;
			break;
		}

		V.line_num++;
	}

	if (PIR1bits.TMR1IF || l_state == 0) { //      Timer1 int handler, for strobe timing
		PIR1bits.TMR1IF = FALSE;
		WriteTimer1(L_ptr->strobe[l_state]); // strobe positioning during rotation

		switch (l_state) {
		case 0:
			G_OUT = 0;
			l_state = 1; // off time after index to start time
			break;
		case 1:
			G_OUT = 1;
			l_state = 2; // on start time duration for strobe pulse
			break;
		case 2:
			G_OUT = 0; // wait to next rotation
			break;
		default:
			G_OUT = 0;
			break;
		}
	}

	if (PIR1bits.RCIF) { // is data from RS-232 port
		V.rx_data = RCREG;
		if (RCSTAbits.OERR) {
			RCSTAbits.CREN = 0; // clear overrun
			RCSTAbits.CREN = 1; // re-enable
		}
		V.comm = TRUE;
	}


	if (INTCONbits.TMR0IF) { //      check timer0 
		INTCONbits.TMR0IF = FALSE; //      clear interrupt flag
		WriteTimer0(timer0_off);
		LED5 = !LED5; // active LED blinker
	}

}

/* main loop routine */
int16_t sw_work(void)
{
	ClrWdt(); // reset watchdog

	if (!SW1) {
		putrsUSART("Timer limit ");
		itoa(l_full, str);
		putsUSART(str);
		putrsUSART("Timer value ");
		itoa(L_ptr->strobe[0], str);
		putsUSART(str);
		LED1 = 1;
	} else {
		LED1 = 0;
	}

	return 0;
}

void init_rmsmon(void)
{
	/*
	 * check for a clean POR
	 */
	V.boot_code = FALSE;
	if (RCON != 0b0011100)
		V.boot_code = TRUE;

	if (STKPTRbits.STKFUL || STKPTRbits.STKUNF) {
		V.boot_code = TRUE;
		STKPTRbits.STKFUL = 0;
		STKPTRbits.STKUNF = 0;
	}

	ADCON1 = 0x7F; // all digital, no ADC
	/* interrupt priority ON */
	RCONbits.IPEN = 1;
	/* define I/O ports */
	RMSPORTA = RMSPORT_IOA;
	RMSPORTB = RMSPORT_IOB;

	G_OUT = LEDON; // preset all LEDS
	LED1 = LEDON;
	LED2 = LEDON;
	LED3 = LEDON;
	LED4 = LEDON;
	LED5 = LEDON;
	LED6 = LEDON;
	RPMLED = LEDON;
	timer0_off = TIMEROFFSET; // blink fast
	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256); // led blinker
	WriteTimer0(timer0_off); //	start timer0 at ~1/2 second ticks
	OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_2 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF); // strobe position clock
	WriteTimer1(SAMPLEFREQ);
	/* data link */
	COMM_ENABLE = TRUE; // for PICDEM4 onboard RS-232, not used on custom board
	OpenUSART(USART_TX_INT_OFF &
		USART_RX_INT_ON &
		USART_ASYNCH_MODE &
		USART_EIGHT_BIT &
		USART_CONT_RX, 64); // 40MHz fosc 9600
	TXSTAbits.SYNC = 0;
	TXSTAbits.BRGH = 0;
	BAUDCTLbits.BRG16 = 0;
	SPBRG = 64;

	/*      work int thread setup */
	INTCONbits.TMR0IE = 1; // enable int
	INTCON2bits.TMR0IP = 1; // make it high level

	PIE1bits.TMR1IE = 1; // enable int
	IPR1bits.TMR1IP = 1; // make it high level

	INTCONbits.INT0IE = 1; // enable RPM sensor input
	INTCON2bits.RBPU = 0; // enable weak pull-ups

	init_rms_params();

	/* Enable all high priority interrupts */
	INTCONbits.GIEH = 1;
}

uint8_t init_rms_params(void)
{
	V.spinning = FALSE;
	V.valid = TRUE;
	V.comm = FALSE;
	V.comm_state = 0;
	V.line_num = 0;

	L_ptr = &L[0];
	/* two line strobes in 3 16-bit timer values for spacing */
	/* for an interrupt driven state machine */
	L[0].strobe[0] = 60000; 
	L[0].strobe[1] = 64900;
	L[0].strobe[2] = 10000;
	L[1].strobe[0] = 50000; // 62000
	L[1].strobe[1] = 64900;
	L[1].strobe[2] = 10000;
	return 0;
}

void main(void)
{
	init_rmsmon();

	/* Loop forever */
	while (TRUE) { // busy work
		sw_work(); // run housekeeping
	}
}
