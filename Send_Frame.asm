#include <xc.inc>
	
#define NUM_PIXELS  8
	
GLOBAL _send_byte	    ; make _send_frame globally accessible
GLOBAL _send_frame
GLOBAL _COLOR
GLOBAL _count
	
	; everything following will be placed into the mytext psect
PSECT mytext, local, class=CODE, delta=2, optim=inline
	
        OUTPUT_0 MACRO
            bsf LATA, 0x2   // 125 ns
            nop             // 250 ns
            bcf LATA, 0x2   // 375 ns
            goto $ + 1      // 625 ns
            nop             // 750 ns
        ENDM

        OUTPUT_1 MACRO
            bsf LATA, 0x2   // 125 ns
            goto $ + 1      // 375 ns
            goto $ + 1      // 625 ns
            bcf LATA, 0x2   // 750 ns
	    nop		    // 875 ns
        ENDM

_send_frame:
	bcf GIE
	; save FSR1H/L
	
	movlw 0x40	; address of led output array
	movwf FSR1L
	clrf FSR1H
	movlw (NUM_PIXELS*3)
	movwf _count

_send_f:
	moviw FSR1++
	movwf _COLOR
    
_send_byte:

SEND_BIT23:
        btfsc _COLOR, 7
        goto SEND_BIT23_1

SEND_BIT23_0:
        OUTPUT_0
        goto SEND_BIT22

SEND_BIT23_1:
        OUTPUT_1

SEND_BIT22:		    // 0 ns
        btfsc _COLOR, 6
        goto SEND_BIT22_1   // 375 ns

SEND_BIT22_0:		    // 250 ns
        OUTPUT_0            // 1000 ns
        goto SEND_BIT21     // 1250 ns

SEND_BIT22_1:		    // 375 ns
        OUTPUT_1            // 1250 ns

SEND_BIT21:		    // 1250 ns
        btfsc _COLOR, 5
        goto SEND_BIT21_1

SEND_BIT21_0:
        OUTPUT_0
        goto SEND_BIT20

SEND_BIT21_1:
        OUTPUT_1

SEND_BIT20:
        btfsc _COLOR, 4
        goto SEND_BIT20_1

SEND_BIT20_0:
        OUTPUT_0
        goto SEND_BIT19

SEND_BIT20_1:
        OUTPUT_1

SEND_BIT19:
        btfsc _COLOR, 3
        goto SEND_BIT19_1

SEND_BIT19_0:
        OUTPUT_0
        goto SEND_BIT18

SEND_BIT19_1:
        OUTPUT_1

SEND_BIT18:
        btfsc _COLOR, 2
        goto SEND_BIT18_1

SEND_BIT18_0:
        OUTPUT_0
        goto SEND_BIT17

SEND_BIT18_1:
        OUTPUT_1

SEND_BIT17:
        btfsc _COLOR, 1
        goto SEND_BIT17_1

SEND_BIT17_0:
        OUTPUT_0
        goto SEND_BIT16

SEND_BIT17_1:
        OUTPUT_1

SEND_BIT16:
        btfsc _COLOR, 0
        goto SEND_BIT16_1

SEND_BIT16_0:
        bsf LATA, 0x2   // 125 ns
        nop             // 250 ns
        bcf LATA, 0x2   // 375 ns
        moviw FSR1++
	movwf _COLOR
	decfsz _count,f
	goto _send_byte
	; restore FSR1H/L
	bsf GIE
	return

SEND_BIT16_1:
        bsf LATA, 0x2   // 125 ns
        goto $ + 1      // 375 ns
        moviw FSR1++
	movwf _COLOR
        bcf LATA, 0x2   // 750 ns

SEND_END:
	decfsz _count,f
	goto _send_byte
	; restore FSR1H/L
	bsf GIE
        return
	