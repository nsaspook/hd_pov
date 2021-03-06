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

typedef struct V_data { // ISR data structure
    uint8_t valid : 1;
    uint8_t comm : 1;
    uint8_t comm_state;
    uint8_t spinning : 1;
    uint8_t boot_code : 1;
    uint8_t line_num :1;
    uint8_t rx_data, tx_data;
} V_data;

typedef struct L_data {
	uint16_t strobe[3];	
} L_data;

#define TRUE	1
#define FALSE	0
#define	ON	1
#define	OFF	0
#define	LEDON	0   // logic low lights led
#define	LEDOFF	1

#define	TIMEROFFSET	18000		// timer0 16bit counter value for ~1 second to overflow 
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
#define SW1		PORTAbits.RA4

#define strobe_up	67
#define strobe_down	31
#define strobe_adjust	11
#define strobe_limit_l	24250 // this limit +500 is from the rs-232 port
#define strobe_limit_h	65530
#endif 