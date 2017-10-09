/*
 * Demo of I2C ULP routines
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "stack.S"

.set BMP180_ADDR,0x77 // 7-bit address
.set WRITE,((BMP180_ADDR << 1) | 0)
.set READ,((BMP180_ADDR << 1) | 1)

.set BMP180_REG_CONTROL,0xF4
.set BMP180_REG_RESULT,0xF6

.set BMP180_COMMAND_TEMPERATURE,0x2E
.set BMP180_COMMAND_PRESSURE0,0x34
.set BMP180_COMMAND_PRESSURE1,0x74
.set BMP180_COMMAND_PRESSURE2,0xB4
.set BMP180_COMMAND_PRESSURE3,0xF4

.macro status s
	move r1,status
	move r0,\s
	st r0,r1,0
.endm

/* Define variables, which go into .bss section (zero-initialized data) */
	.bss
	.global status
status:	.long 0

	.global stack
stack:
	.skip 100
	.global stackEnd
stackEnd:
	.long 0

	/* Code goes into .text section */
	.text
	.global entry
entry:
	move r3,stackEnd

	status 1

	wait 100
	psr
	jump i2c_start_cond

	move r2,WRITE
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	status 2

	move r2,BMP180_REG_CONTROL
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	move r2,BMP180_COMMAND_TEMPERATURE
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	psr
	jump i2c_stop_cond

	status 3

	move r2,5
	psr
	jump waitMs

	psr
	jump i2c_start_cond

	move r2,WRITE
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	move r2,BMP180_REG_RESULT
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	psr
	jump i2c_start_cond

	status 4

	move r2,READ
	psr
	jump i2c_write_byte
	jumpr fail,1,ge

	status 5

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
	move r1,status
	st r2,r1,0

	wait 100


fail:
wake_up:
	/* Wake up the SoC, end program */
	wake
	halt

waitMs:
	wait 8000
	sub r2,r2,1
	jump doneWaitMs,eq
	jump waitMs
doneWaitMs:
	ret
