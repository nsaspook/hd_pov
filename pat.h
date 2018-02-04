#ifndef PAT_H_INCLUDED
#define PAT_H_INCLUDED
//	hardware defines 

#ifdef INTTYPES
#include <stdint.h>
#else
#define INTTYPES
/*unsigned types*/
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned long uint32_t;
typedef unsigned long long uint64_t;
/*signed types*/
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed long int32_t;
typedef signed long long int64_t;
#endif

typedef enum {
	/* rs232 Application's state machine's initial state. */
	APP_STATE_INIT = 0,
	APP_STATE_WAIT_FOR_UDATA,
	APP_STATE_WAIT_FOR_RDATA,
	APP_STATE_WAIT_FOR_DDATA,
	APP_STATE_WAIT_FOR_SDATA,
	APP_STATE_WAIT_FOR_eDATA,
	APP_STATE_WAIT_FOR_EDATA,
	/* Application Error state*/
	APP_STATE_ERROR

} APP_STATES;

typedef struct V_data { // ISR data structure
	uint8_t valid : 1;
	APP_STATES comm_state;
	uint8_t spinning : 1;
	uint8_t boot_code : 1;
	uint8_t line_num : 2;
	uint8_t c_line_num : 2;
	uint8_t at_end : 1;
	uint8_t rx_data, tx_data;
	uint16_t rotations, sequences, patterns, l_size;
} V_data;

typedef struct L_seq {
	uint8_t down : 1; // rotation direction
	uint8_t R : 1;
	uint8_t G : 1;
	uint8_t B : 1;
	uint8_t end : 1; // last line in sequence
	uint8_t skip : 1; // don't light led
	uint16_t offset; // line movement 
};

/* data for one complete rotation*/
typedef struct L_data {
	struct L_seq sequence;
	uint16_t strobe;
} L_data;

#define TRUE	1
#define FALSE	0
#define	ON      1
#define	OFF     0
#define	LEDON	0   // logic low lights led
#define	LEDOFF	1

#define	TIMEROFFSET	18000		// timer0 16bit counter value for ~1 second to overflow 44268
#define	SAMPLEFREQ	60000		// timer1 default value

#define RMSPORTA	TRISA
#define RMSPORTB	TRISB
#define RMSPORT_IOA	0b00010000		// SW1 input RA4
#define RMSPORT_IOB	0b00010001		// Rs-232 transmit on B1, receive on B4, hall gear sensor on B0

#define LED1		LATAbits.LATA3
#define LED2		LATAbits.LATA3
#define LED3		LATAbits.LATA3
#define LED4		LATBbits.LATB6
#define LED5		LATBbits.LATB7
#define LED6		LATAbits.LATA6		
#define COMM_ENABLE	LATBbits.LATB3

#define G_OUT		LATAbits.LATA0
#define R_OUT		LATAbits.LATA1
#define B_OUT		LATAbits.LATA2
#define TACHIN		LATBbits.LATB0
#define RPMLED		LATBbits.LATB5
#define SW1         PORTAbits.RA4

#define PAT2		// display patterns


#ifdef PAT1
#define strobe_up	67
#define strobe_down	31
#define strobe_around	109
#endif

#ifdef PAT2
#define strobe_up	60
#define strobe_down	360
#define strobe_around	1080
#endif

#define strobe_adjust	11
#define strobe_limit_l	24250 // this limit is calc'd from the rs-232 port
#define strobe_limit_h	65534
#define strobe_line	64900 // line width timer count
#define strobe_complete	1000 // end of rotation timer count
#define strobe_max	16
#endif 