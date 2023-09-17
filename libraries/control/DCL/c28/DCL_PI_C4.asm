; DCL_PI_C4.asm - Parallel form PI controller
;
; Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
; ALL RIGHTS RESERVED 

   	  .if $defined(__TI_EABI__)
		.if __TI_EABI__
		.asg	DCL_runPI_C4, _DCL_runPI_C4
		.endif
      .endif

		.global _DCL_runPI_C4

		.sect	"dclfuncs"

; C prototype: float DCL_runPI_C4(DCL_PI *p, float32_t rk, float32_t yk)
; argument 1 = *p : 32-bit PI structure address [XAR4]
; argument 2 = rk : 32-bit floating-point reference [R0H]
; argument 3 = yk : 32-bit floating-point feedback [R1H]
; return = uk : 32-bit floating-point [R0H]

		.align	2

_DCL_runPI_C4:
		.asmfunc
		SUBF32		R2H, R0H, R1H		; R2H = v1
||		MOV32   	R1H, *XAR4			; R1H = Kpa
		MOV			AR0, #0xA			; AR0 = 10
		MPYF32		R0H, R1H, R2H		; R0H = v2
||		MOV32   	R1H, *+XAR4[2]		; R1H = Kia
		MOV32   	R3H, *+XAR4[AR0]	; R3H = i6
		MPYF32		R2H, R1H, R2H		; R2H = v3
		MOV32   	R1H, *+XAR4[4]		; R1H = i10
		MPYF32		R2H, R2H, R3H		; R2H = v8
		NOP								; delay slot
		ADDF32		R3H, R1H, R2H		; R3H = v4
		MOV			AR0, #8				; AR0 = 8
		ADDF32		R0H, R0H, R3H		; R3H = v5
||		MOV32   	*+XAR4[4], R3H		; save i10
		ZERO		R1H					; R1H = 0.0f
		ADDF32		R2H, R1H, #1.0		; R2H = 1.0f
		MOV32		R3H, *+XAR4[6]		; R3H = Umaxa
		MINF32		R0H, R3H			; R0H = sat+
||		MOV32		R2H, R1H			; R2H = v6
		MOV32		R3H, *+XAR4[AR0]	; R3H = Umina
		MAXF32		R0H, R3H			; R0H = uk
||		MOV32		R2H, R1H			; R2H = v6
		MOV			AR0, #0xA			; AR0 = 10
		MOV32		*+XAR4[AR0], R2H	; save i6
		LRETR							; return call
		.endasmfunc

		.end

; end of file
