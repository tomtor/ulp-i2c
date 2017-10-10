/*
 * I2C ULP utility routines
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "stack.S"

.text

.global write8
write8:
	psr
	jump i2c_start_cond

	ld r2,r3,16 // Address
	lsh r2,r2,1
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	ld r2,r3,12 // Register
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	ld r2,r3,8 // data byte
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	psr
	jump i2c_stop_cond

	move r2,0 // Ok
	ret


.global read8
read8:
	psr
	jump i2c_start_cond

	ld r2,r3,12 // Address
	lsh r2,r2,1
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	ld r2,r3,8 // Register
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	psr
	jump i2c_start_cond

	ld r2,r3,12
	lsh r2,r2,1
	or r2,r2,1 // Address Read
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

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
	jump i2c_start_cond

	ld r2,r3,12 // Address
	lsh r2,r2,1
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	ld r2,r3,8 // Register
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	psr
	jump i2c_start_cond

	ld r2,r3,12
	lsh r2,r2,1
	or r2,r2,1 // Address Read
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

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
