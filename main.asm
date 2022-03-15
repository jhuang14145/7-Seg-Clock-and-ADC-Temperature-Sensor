;***************************************************************************
;*
;* Title: temp_meas
;* Author: jhuang14145
;* Version: 1.0
;* Last updated: 11/18/2021
;* Target: atmega4809
;*
;* DESCRIPTION
;* Conditionally takes in a BCD value and displays it on an LED Display
;* Shifts BCD values in an array and stores 4 separte values corresponding
;* to each individual seven segment display
;* Also conditionally converts and displays BCD values in hex on LED DIsplay
;*
;*
;* VERSION HISTORY
;* 1.3 Original version
;***************************************************************************

.nolist
.include "m4809def.inc"
.list

.dseg
bcd_entries: .byte 4
led_display: .byte 4
hex_result: .byte 4
flag_POST: .byte 1
binary_num: .byte 2

.cseg


ldi r16, TCA_SINGLE_CLKSEL_DIV256_gc | TCA_SINGLE_ENABLE_bm
sts TCA0_SINGLE_CTRLA, r16

.equ PERIOD_EXAMPLE_VALUE = 162 ;=20 bef
start:
	rjmp begin

.org TCA0_OVF_vect		;counter interrrupt
	jmp toggle_pin_ISR

.org ADC0_RESRDY_vect	;enable resrdy interrupt vector
	jmp poll_adc_isr

.org PORTE_PORT_vect
	jmp porte_isr		;vector for all PORTE pin change IRQs




begin:
	ldi XH, HIGH(flag_POST)	;initialize pointer to the POST flag
	ldi XL, LOW(flag_POST)
	ldi r16, 0x01
	st X, r16

	ldi XH, HIGH(bcd_entries)	;initialize Xpointer to bcd entries
	ldi XL, LOW(bcd_entries)
	ldi r16, 0x01
	st X+, r16			;stores 1,2,3,4 in bcd display
	inc r16
	st X+, r16
	inc r16
	st X+, r16
	inc r16
	st X+, r16

	ldi r16, 0xFF				;outputs
	out VPORTD_DIR, r16

	ldi r16, 0b11110000			;as outputs
	out VPORTC_DIR, r16

	ldi r16, 0x00				;inputs
	out VPORTA_DIR, r16

	ldi r16, 0b11110000			;input and output respectively
	out VPORTE_DIR, r16			;PE1, PE2, PE3 all as inputs

	ldi XH, HIGH(PORTA_PIN0CTRL);pointer X pointing to pin ctrl
	ldi XL, LOW(PORTA_PIN0CTRL)
	ldi r18, 0x08				;loop control register
	ldi r17, 0b10001000			;sets INVEN to 1 and PINnCTRL to 1

	;TCA0;
	sbi VPORTE_DIR, 1

	ldi r16, TCA_SINGLE_WGMODE_NORMAL_gc	;WGMODE normal
	sts TCA0_SINGLE_CTRLB, r16

	ldi r16, TCA_SINGLE_OVF_bm			;enable overflow interrupt
	sts TCA0_SINGLE_INTCTRL, r16

	;load period low byte then high byte
	ldi r16, LOW(PERIOD_EXAMPLE_VALUE)	;set the period
	sts TCA0_SINGLE_PER, r16
	ldi r16, HIGH(PERIOD_EXAMPLE_VALUE)	;set the period
	sts TCA0_SINGLE_PER + 1, r16
	;set clock and start timer
	ldi r16, TCA_SINGLE_CLKSEL_DIV256_gc | TCA_SINGLE_ENABLE_bm
	sts TCA0_SINGLE_CTRLA, r16

	sei


loop1: 
	st X+, r17					;apply r17 and increment pointer
	dec r18						;decrement loop cntrl var
	brne loop1					;loop

set_irqs:
	;Configure interrupts
	lds r16, PORTE_PIN0CTRL	;set ISC for PE0 to pos. edge
	ori r16, 0x02			;set ISC for rising edge
	sts PORTE_PIN0CTRL, r16

	lds r16, PORTE_PIN2CTRL	;set ISC for PE2 to pos. edge
	ori r16, 0x02			;set ISC for rising edge
	sts PORTE_PIN2CTRL, r16

	sei

	ldi r17, 0x0F			;loop contorl variable for post_display
	//rcall post_display		;call for POST operation


	//task 2//

	lds r16, VREF_CTRLA		;configure vref as 2.5V
	ori r16, 0b00100000		;choose 2.5V option
	sts VREF_CTRLA, r16		;store it


	lds r16, PORTE_PIN3CTRL ;disable theinterrupt and digital input buffer
	ori r16, 0x4			;set as analog input
	sts PORTE_PIN3CTRL, r16

	lds r16, ADC0_CTRLC		;select CTRLC
	ori r16, 0b01000101		;enable integernal reference value, select high ref voltage
	sts ADC0_CTRLC, r16		; also use DIV64 then output to CTRLC

	lds r16, ADC0_MUXPOS	;select mux
	ori r16, 0x0B			;choose PE3
	sts ADC0_MUXPOS, r16	;output

	lds r16, ADC0_CTRLA		;select CTRLA
	ori r16, 0b00000001		;choose enable ADC
	sts ADC0_CTRLA, r16		;begin conversion

	ldi r16, 1
	sts ADC0_INTCTRL, r16

	lds r16, ADC0_COMMAND	;choose COMMAND 
	ori r16, 0x01			;put a 1 - to have conversion enable
	sts ADC0_COMMAND, r16	;output a 1 to ADC0_COMMAND to begin conversion



main_loop:
	rjmp main_loop



;***************************************************************************
;* 
;* "poll_ADC_isr" - Polls the ADC and stores it in led_Display
;*
;* Description:
;* Polls ADC and then converts the value into a 7 segment version.
;* Then it stores the polling into le_display
;*
;* Author: jhuang14145
;* Version: 1.0
;* Last updated: 11/19/2021
;* Target:	ATmega4809
;* Number of words: 23
;* Number of cycles: 27
;* Low registers modified:	r16, r17, r18, 
;* High registers modified: none
;*
;* Parameters: ADC0_RESL, ADC0_RESH
;* Returns:	none
;*
;* Notes: 
;*
;***************************************************************************
poll_ADC_isr:
	cli
	push r16
	push r17
	push r18
	in r16, CPU_SREG
	push r16

	; 2500 in hex = 0x09CF
	lds r16, ADC0_RESL	;low of res
	lds r17, ADC0_RESH	;high of res
	ldi r18, 0xCF		;low byte of 2500 in hex
	ldi r19, 0x09		;high byte of 2500 in hex
	rcall mpy16u		;result is in r18,r19,r20,r21; we want r19 and r20

	lsr r20	;this finishes the shift back two more times in order to get
	ror r19	;a final of a 10 bit shift
	lsr r20
	ror r19	; now we will have a final multiplied and divided v_res

	;500 in hex = 0x01F4
	; we have to subtract 500 from r19:r20
	ldi r16, 0x01	;high byte of 500
	ldi r17, 0xF4	;low byte of 500
	sub r20, r17	;this does orur 16 bit subtraction with carry
	sbc r19, r16
	;r19:r20 is out result 
	sts LOW(binary_num), r19	;store our 16 bit num in binary_num
	sts HIGH(binary_num), r20	;sotre high and lwo bit resdpectively

	ldi r17, HIGH(binary_num)	;load the binary 16-bit value to convert
	ldi r16, LOW(binary_num)
	call bin16_to_BCD			;call the conversion routine
	;r24:r23:r22 is our result

	mov r16, r22
	andi r16, 0x0F
	sts hex_result+0, r16

	mov r16, r22
	swap r16
	andi r16, 0x0F
	sts hex_result+1, r16

	mov r16, r23
	andi r16, 0x0F
	sts hex_result+2, r16

	mov r16, r23
	swap r16
	andi r16, 0x0F
	sts hex_result+3, r16


	ldi XH, HIGH(hex_result)	;initialize X pointer to hex_result
	ldi XL, LOW(hex_result)
	ldi YH, HIGH(led_display)	;pointer to led_display array
	ldi YL, LOW(led_display)

	ld r18, X+					;takes array input
	rcall hex_to_7seg			;converts to 7 seg
	st Y+, r18					;stores it in led_display
								;repeats 4 times for all 4 elemetns in the array
	ld r18, X+
	rcall hex_to_7seg
	st Y+, r18

	ld r18, X+
	rcall hex_to_7seg
	cbr r18, 0				;clear out the bit 7 to remove decimal light
	st Y+, r18

	ld r18, X+
	rcall hex_to_7seg
	st Y+, r18

	lds r16, ADC0_COMMAND	;choose COMMAND 
	ori r16, 0x01			;put a 1 - to have conversion enable
	sts ADC0_COMMAND, r16	;output a 1 to ADC0_COMMAND to begin conversion

	pop r16
	out CPU_SREG, r16
	pop r18
	pop r17
	pop r16 
	sei
	reti

	; V_out = ((RES*2500)/1024) - 500


poll_bcd_hex:
	ldi XH, HIGH(bcd_entries)	;pointer to bcd_entries
	ldi XL, LOW(bcd_entries)
	ld r16, X+					;load dig0 to first 4 bits
	ld r19, X+					;load dig1 to first 4 bits
	swap r19					;shift dig1 to left 4 bits
	add r16, r19				;OR the two shifted together

	ld r17, X+					;load dig2 to first 4 bits
	ld r19, X+					;load dig3 to first 4 bits
	swap r19					;shift dig3 to left 4 bits
	add r17, r19				;OR the dig2 and dig3 together

	ldi r18, 0x00				;clear r18 to make sure it's zeroes
	rcall BCD2bin16				;convert -> result stored in r15:r14

	ldi YH, HIGH(hex_result)	;pointer to hex_result
	ldi YL, LOW(hex_result)

	mov r16, r14				;copy bottom nibble to r16
	andi r16, 0x0F				;clear out left 4 bigits
	st Y+, r16					;store in x[0] as hex0

	mov r16, r14				;copy bottom nibble to r16
	andi r16, 0xF0				;clear out right 4 bits						
	swap r16					;shift 4 bits to right 
	st Y+, r16					;store in x[1] as hex[1]

	mov r16, r15				;copy top nibble
	andi r16, 0x0F				;clear left 4 digs
	st Y+, r16					;set to x[2] as hex[2]

	mov r16, r15				;copy top nibble 
	andi r16, 0xF0				;clear right 4 digs
	swap r16					;shift to right side

	st Y+, r16					;store in x[3] as hex[3]

	ldi XH, HIGH(hex_result)	;initialize X pointer to hex_result
	ldi XL, LOW(hex_result)
	ldi YH, HIGH(led_display)	;pointer to led_display array
	ldi YL, LOW(led_display)

	ld r18, X+					;takes array input
	rcall hex_to_7seg			;converts to 7 seg
	st Y+, r18					;stores it in led_display
								;repeats 4 times for all 4 elemetns in the array
	ld r18, X+
	rcall hex_to_7seg
	st Y+, r18

	ld r18, X+
	rcall hex_to_7seg
	st Y+, r18

	ld r18, X+
	rcall hex_to_7seg
	st Y+, r18

	ret					;exit subroutine

	

;***************************************************************************
;* 
;* "poll_digit_entry" - Polls Pushbutton 1 for Conditional Digit Entry
;*
;* Description:
;* Polls the flag associated with pushbutton 1. This flag is connected to 
;* PE0. IF the flag is set, the contents of the array bdc_entries is shifted
;* left and the BCD digit set on the least significant 4 bits of PORTA_IN are
;* stored in the least significant byte of the bcd_entries array. THen the
;* corresponding segment values for each digit in the bcd_entried display are 
;* writted into the led_display. Note: entry of a non-BCD value is ignored
;* Author: jhuang14145
;* Version: 1.0
;* Last updated: 10/29/2021
;* Target: a
;* Number of words:
;* Number of cycles:
;* Low registers modified:
;* High registers modified:
;*
;* Parameters:
;* Returns:
;*
;* Notes: 
;*
;***************************************************************************

poll_digit_entry:
	in r18, VPORTA_IN
	swap r18
	com r18
	mov r17, r18
	rcall reverse_bits
	swap r17
	mov r18, r17
	andi r18, 0x0F
	cpi r18, 0x0A				;compares to see if eq ot higher than 10
	brsh clear					;back to main_loop if not valid

	mov r17, r18
	ldi XH, HIGH(bcd_entries)	;initialize Xpointer to bcd entries
	ldi XL, LOW(bcd_entries)
	ld r19, X+					;copy first 3 register values
	ld r20, X+
	ld r21, X+

	st X, r21					;store pos 2 to pos 3
	st -X, r20					;store pos 1 to pos 2
	st -X, r19					;store pos 0 to pos 1
	st -X, r17					;store new val to pos 0

	ldi XH, HIGH(bcd_entries)	;initialize X pointer to bcd_entries
	ldi XL, LOW(bcd_entries)
	ldi YH, HIGH(led_display)	;pointer to led_display array
	ldi YL, LOW(led_display)

	lds r18, bcd_entries+0		;takes array input
	rcall hex_to_7seg			;converts to 7 seg
	sts led_display, r18		;stores it in led_display
								;repeats 4 times for all 4 elemetns in the array
	lds r18, bcd_entries+1
	rcall hex_to_7seg
	sts led_display+1, r18

	lds r18, bcd_entries+2
	rcall hex_to_7seg
	sts led_display+2, r18

	lds r18, bcd_entries+3
	rcall hex_to_7seg
	sts led_display+3, r18

	ret

clear:
	ret

;***************************************************************************
;* 
;* "store" - displays from led_display
;*
;* Description: Takes input from led_display, then converts to 7 seg, then
;*				outputs the function
;* Author:						jhuang14145
;* Version:						1.0						
;* Last updated:				10/29/2021
;* Target:						ATmega4809
;* Number of words:				
;* Number of cycles:			
;* Low registers modified:		none		
;* High registers modified:		none 
;*
;* Parameters: 
;* Returns: 
;*
;* Notes: 
;*
;***************************************************************************
mux_display:
	ldi YH, HIGH(led_display);pointer to led_display array
	ldi YL, LOW(led_display)

	rcall dig_1
	rcall dig_2
	rcall dig_3
	rcall dig_4

	ret						;exit subroutine

dig_1:
	ldi r16, 0b11111111		;enable pin
	out VPORTC_OUT, r16		;turn on transistor for pin
	lds r18, led_display+3	;store led_display value
	out VPORTD_OUT, r18		;output led_display 
	ldi r16, 0b01111111		;disable pin
	out VPORTC_OUT, r16		;turn off transistor for pin
	rcall var_delay			;delay
	ret
dig_2:
	ldi r16, 0b11111111		;enable pin
	out VPORTC_OUT, r16		;turn on transistor for pin
	lds r18, led_display+2	;store led_display value
	out VPORTD_OUT, r18		;output led_display 
	ldi r16, 0b10111111		;disable pin
	out VPORTC_OUT, r16		;turn off transistor for pin
	rcall var_delay			;delay
	ret
dig_3:
	ldi r16, 0b11111111		;enable pin
	out VPORTC_OUT, r16		;turn on transistor for pin
	lds r18, led_display+1	;store led_display value
	out VPORTD_OUT, r18		;output led_display 
	ldi r16, 0b11011111		;disable pin
	out VPORTC_OUT, r16		;turn off transistor for pin
	rcall var_delay			;delay
	ret
dig_4:
	ldi r16, 0b11111111		;enable pin
	out VPORTC_OUT, r16		;turn on transistor for pin
	lds r18, led_display+0	;store led_display value
	out VPORTD_OUT, r18		;output led_display 
	ldi r16, 0b11101111		;disable pin
	out VPORTC_OUT, r16		;turn off transistor for pin
	rcall var_delay			;delay
	ret


;***************************************************************************
;* 
;* "hex_to_7seg" - Hexadecimal to Seven Segment Conversion
;*
;* Description: Converts a right justified hexadecimal digit to the seven
;* segment pattern required to display it. Pattern is right justified a
;* through g. Pattern uses 0s to turn segments on ON.
;*
;* Author:						Ken Short
;* Version:						1.0						
;* Last updated:				101620
;* Target:						ATmega4809
;* Number of words:				8
;* Number of cycles:			13
;* Low registers modified:		none		
;* High registers modified:		r16, r18, ZL, ZH
;*
;* Parameters: r18: right justified hex digit, high nibble 0
;* Returns: r18: segment values a through g right justified
;*
;* Notes: 
;*
;***************************************************************************
hex_to_7seg:
	andi r18, 0x0F				;clear ms nibble
    ldi ZH, HIGH(hextable * 2)  ;set Z to point to start of table
    ldi ZL, LOW(hextable * 2)
    ldi r16, $00                ;add offset to Z pointer
    add ZL, r18
    adc ZH, r16
    lpm r18, Z                  ;load byte from table pointed to by Z
	ret

    ;Table of segment values to display digits 0 - F
    ;!!! seven values must be added - verify all values
hextable: .db $01, $4F, $12, $06, $4C, $24, $20, $0F, $00, $84, $88, $E0, $B0, $C2, $B0, $B8

	var_delay:		;delay function
	ldi r22, 0x2F
outer_loop:
	ldi r21, 110
inner_loop:
	dec r21
	brne inner_loop
	dec r22
	brne outer_loop
	ret

;***************************************************************************
;* 
;* "reverse_bits" - Reverse Bits
;*
;* Description: Reverses the bit positions in a byte passed in.Bit 0
;* is bit 7, and bit 7 will be bit 0, ... , etc. 
;*
;* Author:					jhuang14145
;* Version:					1.0
;* Last updated:			10/30/2020
;* Target:					ATmega4809
;* Number of words:			8
;* Number of cycles:		7
;* Low registers modified:	r17, r19
;* High registers modified:	none
;*
;* Parameters: r17
;* Returns: r17
;*
;* Notes: 
;*
;***************************************************************************

reverse_bits:
	clc                ; clear
	ldi r19, 0x80      ; load 0x80 into r19
	rloop:
		rol r17        ; rotate left with carry r17
		ror r19        ; rotate right with carry r19
		brcc rloop     ; branch if carry cleared
		mov r17, r19   ; copy r19 into r17
		ret            ; return



	
.def	copyL	=r12		;temporary register
.def	copyH	=r13		;temporary register
.def	mp10L	=r14		;Low byte of number to be multiplied by 10
.def	mp10H	=r15		;High byte of number to be multiplied by 10
.def	adder	=r19		;value to add after multiplication	

;***** Code

mul10a:	;***** multiplies "mp10H:mp10L" with 10 and adds "adder" high nibble
	swap	adder
mul10b:	;***** multiplies "mp10H:mp10L" with 10 and adds "adder" low nibble
	mov	copyL,mp10L	;make copy
	mov	copyH,mp10H
	lsl	mp10L		;multiply original by 2
	rol	mp10H
	lsl	copyL		;multiply copy by 2
	rol	copyH		
	lsl	copyL		;multiply copy by 2 (4)
	rol	copyH		
	lsl	copyL		;multiply copy by 2 (8)
	rol	copyH		
	add	mp10L,copyL	;add copy to original
	adc	mp10H,copyH	
	andi	adder,0x0f	;mask away upper nibble of adder
	add	mp10L,adder	;add lower nibble of adder
	brcc	m10_1		;if carry not cleared
	inc	mp10H		;	inc high byte
m10_1:	ret	

;***** Main Routine Register Variables

.def	tbinL	=r14		;Low byte of binary result (same as mp10L)
.def	tbinH	=r15		;High byte of binary result (same as mp10H)
.def	fBCD0	=r16		;BCD value digits 1 and 0
.def	fBCD1	=r17		;BCD value digits 2 and 3
.def	fBCD2	=r18		;BCD value digit 5

;***** Code

BCD2bin16:
	andi	fBCD2,0x0f	;mask away upper nibble of fBCD2
	clr	mp10H		
	mov	mp10L,fBCD2	;mp10H:mp10L = a
	mov	adder,fBCD1
	rcall	mul10a		;mp10H:mp10L = 10a+b
	mov	adder,fBCD1
	rcall	mul10b		;mp10H:mp10L = 10(10a+b)+c
	mov	adder,fBCD0		
	rcall	mul10a		;mp10H:mp10L = 10(10(10a+b)+c)+d
	mov	adder,fBCD0
	rcall	mul10b		;mp10H:mp10L = 10(10(10(10a+b)+c)+d)+e
	ret
	


porte_ISR:
	push r16				;save r16 then SREG
	in r16, CPU_SREG
	push r16
	cli						;clear global interrupt enable

							;Determine which pins of PORTE have IRQs
	lds r16, PORTE_INTFLAGS	;check for PE0 IRQ flag set
	sbrc r16, 0
	rcall PB1_sub			;execute subroutine for PE0

	lds r16, PORTE_INTFLAGS	;check for PE2 IRQ flag set
	sbrc r16, 2
	rcall PB2_sub			;execute subroutine for PE2

	pop r16					;restore SREG then r16
	out CPU_SREG, r16
	pop r16
	reti					;return from PORTE pin change ISR


;Subroutines called by porte_ISR
PB1_sub:					;PE0's task to be done
	call poll_digit_entry
	nop
	ldi r16, PORT_INT0_bm	;clear IRQ flag for PE0
	sts PORTE_INTFLAGS, r16
	ret

PB2_sub:					;PE2's task to be done
	call poll_bcd_hex
	nop
	ldi r16, PORT_INT2_bm	;clear IRQ flag for PE2
	sts PORTE_INTFLAGS, r16
	ret
reset:
	jmp start

toggle_pin_ISR:
	push r16
	in r16, CPU_SREG
	push r16
	push r17

	ldi r17, 0b00000010
	in r16, VPORTE_OUT
	eor r16, r17
	out VPORTE_OUT, r16

	ldi r16, TCA_SINGLE_OVF_bm	;clear OVF flag
	sts TCA0_SINGLE_INTFLAGS, r16

	rcall mux_display				;mux display

	pop r17
	pop r16
	out CPU_SREG, r16
	pop r16

	reti

;***************************************************************************
;* 
;* "post_display" - POST Operation
;*
;* Description: Turns all all segments and decimal for approx 1s. THen blanks
;*
;* Author:					jhuang14145
;* Version:					1.0
;* Last updated:			11/18/2020
;* Target:					ATmega4809
;* Number of words:			
;* Number of cycles:		
;* Low registers modified:	
;* High registers modified:	
;*
;* Parameters: 
;* Returns: 
;*
;* Notes: 
;*
;***************************************************************************

post_display:
	//alter for 1s
	ldi r16, $00			;7seg for 8
	sts led_display+0, r16	;load led display with all 8's
	sts led_display+1, r16
	sts led_display+2, r16
	sts led_display+3, r16
	dec r17
	breq clearPost			;repeat until 0	
	rcall mux_display

	rjmp post_display		;loop back to post_display

clearPost:
	ret

;***************************************************************************
;*
;* "mpy16u" - 16x16 Bit Unsigned Multiplication
;*
;* This subroutine multiplies the two 16-bit register variables 
;* mp16uH:mp16uL and mc16uH:mc16uL.
;* The result is placed in m16u3:m16u2:m16u1:m16u0.
;*  
;* Number of words	:14 + return
;* Number of cycles	:153 + return
;* Low registers used	:None
;* High registers used  :7 (mp16uL,mp16uH,mc16uL/m16u0,mc16uH/m16u1,m16u2,
;*                          m16u3,mcnt16u)	
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	mc16uL	=r16		;multiplicand low byte
.def	mc16uH	=r17		;multiplicand high byte
.def	mp16uL	=r18		;multiplier low byte
.def	mp16uH	=r19		;multiplier high byte
.def	m16u0	=r18		;result byte 0 (LSB)
.def	m16u1	=r19		;result byte 1
.def	m16u2	=r20		;result byte 2
.def	m16u3	=r21		;result byte 3 (MSB)
.def	mcnt16u	=r22		;loop counter

;***** Code

mpy16u:	clr	m16u3		;clear 2 highest bytes of result
	clr	m16u2
	ldi	mcnt16u,16	;init loop counter
	lsr	mp16uH
	ror	mp16uL

m16u_1:	brcc	noad8		;if bit 0 of multiplier set
	add	m16u2,mc16uL	;add multiplicand Low to byte 2 of res
	adc	m16u3,mc16uH	;add multiplicand high to byte 3 of res
noad8:	ror	m16u3		;shift right result byte 3
	ror	m16u2		;rotate right result byte 2
	ror	m16u1		;rotate result byte 1 and multiplier High
	ror	m16u0		;rotate result byte 0 and multiplier Low
	dec	mcnt16u		;decrement loop counter
	brne	m16u_1		;if not done, loop more
	ret

;***************************************************************************
;* 
;* "bin16_to_BCD" - 16-bit Binary to BCD Conversion
;*
;* Description: Converts a 16-bit unsigned binary number to a five digit
;* packed BCD number. Uses subroutine div16u from Atmel application note AVR200
;*
;* Author:					Ken Short
;* Version:					0.0
;* Last updated:			111320
;* Target:					ATmega4809
;* Number of words:
;* Number of cycles:
;* Low registers modified:	r14, r15
;* High registers modified: r16, r17, r18, r19, r20, r22, r23, r24
;*
;* Parameters: r17:r16 16-bit unsigned right justified number to be converted.
;* Returns:		r24:r23:r22 five digit packed BCD result.
;*
;* Notes: 
;* Subroutine uses repeated division by 10 to perform conversion.
;***************************************************************************
bin16_to_BCD:
	ldi r19, 0			;high byte of divisor for div16u
	ldi r18, 10			;low byte of the divisor for div16u

	rcall div16u		;divide original binary number by 10
	mov r22, r14		;result is BCD digit 0 (least significant digit)
	rcall div16u		;divide result from first division by 10, gives digit 1 
	swap r14			;swap digit 1 for packing
	or r22, r14			;pack

	rcall div16u		;divide result from second division by 10, gives digit 2
	mov r23, r14		;place in r23
	rcall div16u		;divide result from third division by 10, gives digit 3 
	swap r14			;swap digit 3 for packing
	or r23, r14			;pack

	rcall div16u		;divide result from fourth division by 10, gives digit 4
	mov r24, r14		;place in r24

	ret


;Subroutine div16u is from Atmel application note AVR200

;***************************************************************************
;*
;* "div16u" - 16/16 Bit Unsigned Division
;*
;* This subroutine divides the two 16-bit numbers 
;*# "dd16uH:dd16uL" (dividend) and "dv16uH:dv16uL" (divisor). 
;* The result is placed in "dres16uH:dres16uL" and the remainder in
;* "drem16uH:drem16uL".
;*  
;* Number of words	:19
;* Number of cycles	:235/251 (Min/Max)
;* Low registers used	:2 (drem16uL,drem16uH)
;* High registers used  :5 (dres16uL/dd16uL,dres16uH/dd16uH,dv16uL,dv16uH,
;*			    dcnt16u)
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	drem16uL=r14
.def	drem16uH=r15
.def	dres16uL=r16
.def	dres16uH=r17
.def	dd16uL	=r16
.def	dd16uH	=r17
.def	dv16uL	=r18
.def	dv16uH	=r19
.def	dcnt16u	=r20

;***** Code

div16u:	clr	drem16uL	;clear remainder Low byte
	sub	drem16uH,drem16uH;clear remainder High byte and carry
	ldi	dcnt16u,17	;init loop counter
d16u_1:	rol	dd16uL		;shift left dividend
	rol	dd16uH
	dec	dcnt16u		;decrement counter
	brne	d16u_2		;if done
	ret			;    return
d16u_2:	rol	drem16uL	;shift dividend into remainder
	rol	drem16uH
	sub	drem16uL,dv16uL	;remainder = remainder - divisor
	sbc	drem16uH,dv16uH	;
	brcc	d16u_3		;if result negative
	add	drem16uL,dv16uL	;    restore remainder
	adc	drem16uH,dv16uH
	clc			;    clear carry to be shifted into result
	rjmp	d16u_1		;else
d16u_3:	sec			;    set carry to be shifted into result
	rjmp	d16u_1
	
