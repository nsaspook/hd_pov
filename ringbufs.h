/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

#include <xc.h> // include processor files - each processor file is guarded.  

#ifndef RINGBUFS_H
#define	RINGBUFS_H

#ifdef	__cplusplus
extern "C" {
#endif

	/*unsigned types*/
	typedef unsigned char uint8_t;
	typedef unsigned int uint16_t;
	typedef unsigned long uint32_t;
	typedef unsigned long long uint64_t;
	/*signed types*/
	typedef signed char int8_t;
	typedef signed int int16_t;
	typedef signed long int32_t;
	typedef signed long long int64_t;


#define RBUF_SIZE    16

	typedef struct ringBufS_t {
		uint8_t buf[RBUF_SIZE];
		int8_t head;
		int8_t tail;
		int8_t count;
	} ringBufS_t;

	void ringBufS_init(ringBufS_t *_this);
	int8_t ringBufS_empty(ringBufS_t *_this);
	int8_t ringBufS_full(ringBufS_t *_this);
	uint8_t ringBufS_get(ringBufS_t *_this);
	void ringBufS_put(ringBufS_t *_this, const uint8_t c);
	void ringBufS_flush(ringBufS_t *_this, const int8_t clearBuffer);


#ifdef	__cplusplus
	extern "C" {
#endif /* __cplusplus */

		// TODO If C++ is being used, regular C code needs function names to have C 
		// linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
	}
#endif /* __cplusplus */

#endif	/* XC_HEADER_TEMPLATE_H */

