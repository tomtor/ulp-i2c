/*
 * Demo of I2C ULP routines
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "stack.S"

.set BMP180_ADDR,0x77 // 7-bit address

.set BMP180_REG_CONTROL,0xF4
.set BMP180_REG_RESULT,0xF6

.set BMP180_COMMAND_TEMPERATURE,0x2E
.set BMP180_COMMAND_PRESSURE0,0x34
.set BMP180_COMMAND_PRESSURE1,0x74
.set BMP180_COMMAND_PRESSURE2,0xB4
.set BMP180_COMMAND_PRESSURE3,0xF4

/* Define variables, which go into .bss section (zero-initialized data) */
	.bss

	.global temp
temp:	.long 0
	.global	pressure, pressure2
pressure: .long 0
pressure2: .long 0

	.global ac1, ac2, ac3, ac4, ac5, ac6, b1, b2, mb, mc, md

ac1: .long 0
ac2: .long 0
ac3: .long 0
ac4: .long 0
ac5: .long 0
ac6: .long 0

b1: .long 0
b2: .long 0

mb: .long 0
mc: .long 0
md: .long 0

	/* Code goes into .text section */
	.text

.global readBMP
readBMP:
	move r1,ac1
	ld r0,r1,0
	jumpr initBMP,1,lt
didInit:
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
	move r2,temp // store result
	st r1,r2,0

	// Read raw pressure
	move r1,BMP180_ADDR
	push r1
	move r1,BMP180_REG_CONTROL
	push r1
	move r1,BMP180_COMMAND_PRESSURE1
	push r1
	psr
	jump write8
	add r3,r3,3 // remove 3 arguments from stack
	move r0,r2 // test for error in r2
	jumpr fail,1,ge

	// Wait 8 ms for sensor computation
	move r2,8
	psr
	jump waitMs

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
	move r2,pressure // store result
	st r1,r2,0

	move r1,BMP180_ADDR
	push r1
	move r1,(BMP180_REG_RESULT+2)
	push r1
	psr
	jump read8
	add r3,r3,2 // remove call parameters from stack
	move r1,r0 // save result
	move r0,r2 // test for error
	jumpr fail,1,ge
	move r2,pressure2 // store result
	st r1,r2,0

	ret

fail:
	move r1,temp
	move r0,0 // 0 signals error
	st r0,r1,0
	ret


#define BMP085_CAL_AC1           0xAA
#define BMP085_CAL_AC2           0xAC
#define BMP085_CAL_AC3           0xAE
#define BMP085_CAL_AC4           0xB0
#define BMP085_CAL_AC5           0xB2
#define BMP085_CAL_AC6           0xB4
#define BMP085_CAL_B1            0xB6
#define BMP085_CAL_B2            0xB8
#define BMP085_CAL_MB            0xBA
#define BMP085_CAL_MC            0xBC
#define BMP085_CAL_MD            0xBE

// Read calibration data
initBMP:
	move r1,ac1
	push r1
	move r1,BMP180_ADDR
	push r1
	move r1,BMP085_CAL_AC1
	push r1
read_cal:
	psr
	jump read16
	or r2,r2,0 // test error
	jump readok,eq
	jump fail
readok:
	ld r1,r3,12
	st r0,r1,0
	add r1,r1,1
	st r1,r3,12 // next cal parameter address
	ld r0,r3,4
	add r0,r0,2 // next register
	st r0,r3,4
	jumpr read_cal,BMP085_CAL_MD+2,lt
	add r3,r3,3
	jump didInit

// Wait for r2 milliseconds
waitMs:
	wait 8000
	sub r2,r2,1
	jump doneWaitMs,eq
	jump waitMs
doneWaitMs:
	ret
