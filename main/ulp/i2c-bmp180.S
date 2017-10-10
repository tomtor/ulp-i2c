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

/* Define variables, which go into .bss section (zero-initialized data) */
	.bss
	.global status
status:	.long 0

	.global	counter
counter: .long 0

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

	// Read the BMP-180 every 4 timer cycles:
	move r1,counter
	ld r0,r1,0
	add r0,r0,1
	st r0,r1,0 // increment counter
	and r0,r0,0x3
	jumpr waitNext,1,ge

	move r1,BMP180_ADDR
	push r1
	move r1,BMP180_REG_CONTROL
	push r1
	move r1,BMP180_COMMAND_TEMPERATURE
	push r1
	psr
	jump write8
	add r3,r3,3 // remove 3 arguments from stack
	move r0,r2 // test for error in r2
	jumpr fail,1,ge

	// Wait 5ms for sensor computation
	move r2,5
	psr
	jump waitMs

	// Read 16 bit result
	move r1,BMP180_ADDR
	push r1
	move r1,BMP180_REG_RESULT
	push r1
	psr
	jump read16
	add r3,r3,2 // remove call parameters from stack
	move r1,r0 // save result
	move r0,r2 // test for error
	jumpr fail,1,ge
	move r2,status // store result
	st r1,r2,0

	jump wake_up

fail:
	move r1,status
	move r0,0 // 0 signals error
	st r0,r1,0

wake_up:
	/* Wake up the SoC, end program */
	wake

waitNext:
	halt

// Wait for r2 milliseconds
waitMs:
	wait 8000
	sub r2,r2,1
	jump doneWaitMs,eq
	jump waitMs
doneWaitMs:
	ret
