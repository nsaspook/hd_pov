#ifndef PAT_H_INCLUDED
#define PAT_H_INCLUDED 

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

typedef enum {
	/* rotation state machine */
	ISR_STATE_FLAG = 0,
	ISR_STATE_LINE,
	ISR_STATE_WAIT,
	ISR_STATE_ERROR

} ISR_STATES;

typedef struct V_data { // control data structure with possible volatile issues
	APP_STATES comm_state;
	volatile ISR_STATES l_state;
	uint8_t boot_code : 1;
	volatile uint8_t line_num : 2;
	volatile uint8_t rx_data;
	uint16_t l_size;
	volatile uint16_t rotations, sequences;
	volatile uint16_t l_full, l_width;
	uint8_t str[24];
} V_data;

typedef volatile struct L_seq {
	uint8_t down : 1; // rotation direction
	uint8_t R : 1;
	uint8_t G : 1;
	uint8_t B : 1;
	uint8_t end : 1; // last line in sequence
	uint8_t skip : 1; // don't light led
	uint8_t rot :1;  // rotation and sequence flags
	uint8_t seq :1;
	uint16_t offset; // line movement 
};

/* data for one complete rotation*/
typedef volatile struct L_data {
	struct L_seq sequence;
	uint16_t strobe;
} L_data;

#define	ON      true
#define	OFF     false

//	hardware defines
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

/* rotation params for 40mHZ PIC18f1320 */
#define strobe_adjust	11
#define strobe_limit_l	24250 // this limit is calc'd from the rs-232 port
#define strobe_limit_h	65534
#define strobe_line	65100 // line width timer count
#define strobe_max	16
#endif 