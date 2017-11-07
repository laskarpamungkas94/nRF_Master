	.cdecls C,LIST,"PeripheralHeaderIncludes.h"
	.include "PWMDRV_2ch_UpDwnCntFreq.asm"
	.include "PWMDRV_4ch_HiResUpDwnCntFreq.asm"
	.include "ADCDRV_1ch.asm"
	;.include "DLOG_1ch.asm"
	.include "CNTL_2P2Z.asm"

		; dummy variable for pointer initialisation
ZeroNet	 .usect "ZeroNet_Section",2,1,1	; output terminal 1


	.def _Ctrl_Init
	.text
;=============================
_Ctrl_Init:
;=============================
	ZAPA
	MOVL	XAR0, #ZeroNet
	MOVL	*XAR0, ACC

	;ADCDRV_1ch_INIT 0
	CNTL_2P2Z_INIT 1
	;PWMDRV_1ch_UpDwnCntFreq_INIT 1
	;PWMDRV_2ch_UpDwnCntFreq_INIT 1,2
	PWMDRV_4ch_HiResUpDwnCntFreq_INIT 1,2,3,4
	;DLOG_1ch_INIT 1

	LRETR


	.sect "ramfuncs"
	.def _Ctrl_ISR
;=============================
_Ctrl_ISR:
;=============================
	; full context save - push any unprotected registers onto stack
	;PUSH  	AR1H:AR0H
	;PUSH  	XAR2
	;PUSH  	XAR3
	;PUSH  	XAR4
	;PUSH  	XAR5
	;PUSH  	XAR6
	;PUSH  	XAR7
	;PUSH  	XT
	;SPM   	0          				; set C28 mode
	;CLRC  	AMODE     				; set addressing mode
	;CLRC  	PAGE0,OVM

	;ADCDRV_1ch 0
	CNTL_2P2Z 1
	;PWMDRV_2ch_UpDwnCntFreq 1,2
	PWMDRV_4ch_HiResUpDwnCntFreq 1,2,3,4
	;DLOG_1ch 1
	;CPT_GainTrack 1

	;MOVW	DP, #_EPwm1Regs.ETCLR
	;MOV		@_EPwm1Regs.ETCLR, #0x01

	;MOVW 	DP,#_PieCtrlRegs.PIEACK
	;MOV 	@_PieCtrlRegs.PIEACK,#0x4

	; Clear ADC Interrupt flag
	;MOVW    DP,#_AdcRegs.ADCINTFLGCLR
	;MOV     @_AdcRegs.ADCINTFLGCLR,#0x1

	; Acknowledge PIE interrupt Group 1
	;MOVW 	DP,#_PieCtrlRegs.PIEACK
	;MOV 	@_PieCtrlRegs.PIEACK,#0x1

; full context restore
	;POP   	XT
	;POP   	XAR7
	;POP   	XAR6
	;POP   	XAR5
	;POP   	XAR4
	;POP   	XAR3
	;POP   	XAR2
	;POP   	AR1H:AR0H
	;IRET

.end
