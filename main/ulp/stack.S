/*
 * ULP stack and subroutine macros
 */

.macro push rx
	st \rx,r3,0
	sub r3,r3,1
.endm

.macro pop rx
	add r3,r3,1
	ld \rx,r3,0
.endm

// Prepare subroutine jump, uses scratch register sr
.macro psr sr=r1 pos=.
	.set _next2,(\pos+16)
	move \sr,_next2
	push \sr
.endm

// Return from subroutine
.macro ret sr=r1
	pop \sr
	jump \sr
.endm
