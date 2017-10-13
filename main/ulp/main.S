/*
 * Demo of I2C ULP routines
 */

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc_ulp.h"

#include "stack.S"


/* Define variables, which go into .bss section (zero-initialized data) */
	.bss

prev_temp:	.long 0
prev_pressure: .long 0
prev_pressure2: .long 0

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

	// GPIO2 LED ON
	WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 12, 1, 1)

	psr
	jump readBMP

	// GPIO2 LED OFF
	WRITE_RTC_REG(RTC_GPIO_OUT_REG, RTC_GPIO_OUT_DATA_S + 12, 1, 0)

	/* wake up after significant change */
	move r1,pressure
	ld r0,r1,0
	move r1,prev_pressure
	ld r2,r1,0
	sub r0,r0,r2
	psr
	jump abs
	jumpr testTemp,6,lt
	jump wakeUp

testTemp:
	move r1,temp
	ld r0,r1,0
	move r1,prev_temp
	ld r2,r1,0
	sub r0,r0,r2
	psr
	jump abs
	jumpr waitNext,10,lt

wakeUp:
	/* safe new pressure and temp */
	move r1,pressure
	ld r0,r1,0
	move r1,prev_pressure
	st r0,r1,0
	move r1,temp
	ld r0,r1,0
	move r1,prev_temp
	st r0,r1,0
	/* Wake up the SoC, end program */
	wake
	/* Stop the wakeup timer so it does not restart ULP */
	WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)

waitNext:
	halt

// Compute abs value of R0
abs:
	and r1,r0,0x8000
	jump noNegate,eq
	move r1,0
	sub r0,r1,r0
noNegate:
	ret
