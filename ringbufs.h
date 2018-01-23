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


#define RBUF_SIZE    32u

	typedef struct ringBufS_t {
		uint8_t buf[RBUF_SIZE];
		uint8_t head;
		uint8_t tail;
		uint8_t count;
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

#endif	/* RINGBUFS_H */

