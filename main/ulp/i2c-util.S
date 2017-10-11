/*
 * I2C ULP utility routines
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "stack.S"

.text

write_intro:
	psr
	jump i2c_start_cond

	ld r2,r3,20 // Address
	lsh r2,r2,1
	psr
	jump i2c_write_byte
	jumpr popfail,1,ge

	ld r2,r3,16 // Register
	psr
	jump i2c_write_byte
	jumpr popfail,1,ge
	ret


.global write8
write8:
	psr
	jump write_intro

write_b:
	ld r2,r3,8 // data byte
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	psr
	jump i2c_stop_cond

	move r2,0 // Ok
	ret


.global write16
write16:
	psr
	jump write_intro

	ld r2,r3,8 // data byte 1
	rsh r2,r2,8
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	jump write_b


read_intro:
	psr
	jump i2c_start_cond

	ld r2,r3,16 // Address
	lsh r2,r2,1
	psr
	jump i2c_write_byte
	jumpr popfail,1,ge

	ld r2,r3,12 // Register
	psr
	jump i2c_write_byte
	jumpr popfail,1,ge

	psr
	jump i2c_start_cond

	ld r2,r3,16
	lsh r2,r2,1
	or r2,r2,1 // Address Read
	psr
	jump i2c_write_byte
	jumpr popfail,1,ge

	ret
popfail:
	pop r1 // pop caller return address
	move r2,1
	ret

.global read8
read8:
	psr
	jump read_intro

	move r2,1 // last byte
	psr
	jump i2c_read_byte
	push r0

	psr
	jump i2c_stop_cond

	pop r0

	move r2,0 // OK
	ret
fail:
	move r2,1
	ret

.global read16
read16:
	psr
	jump read_intro

	move r2,0
	psr
	jump i2c_read_byte
	push r0

	move r2,1 // last byte
	psr
	jump i2c_read_byte
	push r0

	psr
	jump i2c_stop_cond

	pop r0
	pop r2 // first byte
	lsh r2,r2,8
	or r2,r2,r0
	move r0,r2

	move r2,0 // OK
	ret
