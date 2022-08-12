;############################################################################
; SWINSID
;
; This file is a reconstruction of the SwinSID Nano source code. It is based
; on the last "Lazy Jones fix" firmware
;
; Original SwinSID firmware was developed by Swinkels
; Lazy Jones fix developed by Máté "CodeKiller" Sebök
;
; Source code reconstruction by Daniël Mantione
;
; Set your editor tab size to 4.
;
;############################################################################


#include <avr/io.h>

.text

; Convenient symbols for port numbers when using in/out

p_PINB	= PINB - __SFR_OFFSET
p_DDRB	= DDRB - __SFR_OFFSET
p_PORTB	= PORTB - __SFR_OFFSET
p_PINC	= PINC - __SFR_OFFSET
p_DDRC	= DDRC - __SFR_OFFSET
p_PORTC	= PORTC - __SFR_OFFSET
p_PIND	= PIND - __SFR_OFFSET
p_DDRD	= DDRD - __SFR_OFFSET
p_PORTD	= PORTD - __SFR_OFFSET
p_EIMSK	= EIMSK - __SFR_OFFSET
p_TCCR0A	= TCCR0A  - __SFR_OFFSET
p_TCCR0B	= TCCR0B  - __SFR_OFFSET
p_OCR0A	= OCR0A - __SFR_OFFSET
p_SPL	= SPL - __SFR_OFFSET
p_SPH	= SPH - __SFR_OFFSET
p_SREG	= SREG - __SFR_OFFSET

;
; bit numbers:
;
b0	= 0x00
b1	= 0x01
b2	= 0x02
b3	= 0x03
b4	= 0x04
b5	= 0x05
b6	= 0x06
b7	= 0x07
;---------------------------------------
;
; GLOBAL REGISTERS
;
; R11    : Used by CS irq handler to temporary store data bus
; R12    : Used by CS irq handler as temp register
; R26/XL : Used by CS irq handler to create pointer to SID register being written.
; R27/XH : Fixed to SRAM area where SID registers are stored.
; R28/YL : CS irq handler stores R26 into this if D0=1. Purpose unknown. 

.org 0x000,0xff

irq_reset:
	rjmp	reset

; Explicitely set the current address to make the assembler complain if above code
; is modified and increases in length.
.org 0x002,0xff

irq_chipselect:
    ; CPU generates INT0 interrupt and starts execution here when the
    ; SID chipselect line drops low
    ;
    ; The SID registers are stored in SRAM. The high byte, XH or r27 is global and
    ; never modified. The following code writes the address bus to XL or r26, making
    ; X a pointer to the right register in SRAM and then stores the data bus D0..D7
    ; to that memory location.
    ;
    ; AVR finishes current instruction on irq then needs 3 cycles before execution.
    ; In practice this means 4-6 cycles latency. Assume CS happens on rise of PHI2 + 80ns
    ; and we are running at 32MHz, so a cycle takes 31.25 ns. This means that execution starts
    ; here on rise PHI2 + 205-268ns. The number at the start of the comment below is the lower
    ; bound (rounded to whole ns) when the instruction starts add 62.5ns to get the upper bound.
    ; The second number is the number of clock cycles the instruction will take.

#ifdef LAZY_JONES_FIX
	in	r26,p_PINC	; 205 1 Read port C (PC0..PC4 = A0..A4, PC5 = D2, PC6 = reset)
	in	r11,p_PIND	; 236 1 Read port D (PD0..PD7 = D0..D7, except PD2 which is CS)
	sbic p_PINB,b5	; 268 1/2 If RW=0 (thus a write to sid) skip the RETI.
	reti			; 299 1 Ignore reads from the SID.
	in	r12,p_SREG	; 330 1 Save AVR flags register SREG
	bst	r26,b5		; 361 1 Load D2 into T flag
	bld	r11,b2		; 393 1 Store T into bit 2, so r11 contains full D0..D7
	andi r26,0x1f	; 424 1 Keep bits 0..4, so r26 contains A0..A4.
	st	X,r11		; 455 2 Store read value from databus in correct SID register
	out	p_SREG,r12	; 518 1 Restore AVR flags register
	sbrs r11,b0		; 549 1/2 Skip the mov if D0=1
	mov	r28,r26		; 580 1 Copy A0..A4 to r28/yl. This is for the Lazy Jones fix
					;       Further checks performed outside IRQ handler.
	reti			; 611 1
#else
	in r26,p_PINC	; 205 1 Read port C (PC0..PC4 = A0..A4, PC5 = D2, PC6 = reset)
	in r11,p_PIND	; 236 1 Read port D (PD0..PD7 = D0..D7, except PD2 which is CS)
	sbrc r11,b2;	; 268 1/2 Skip next jmp if CS low (redundant, irq means it is low)
	rjmp no_cs		; 299 1 Get out of here if CS high
	in r12,p_SREG	; 330 1 Save AVR flags register SREG
	bst r26,b5		; 361 1 Load D2 into T flag
	bld r11,b2		; 393 1 Store T into bit 2, so r11 contains full D0..D7
	andi r26,0x1f	; 424 1 Keep bits 0..4, so r26 contains A0..A4.
	st X,r11		; 455 2 Store read value from databus in correct SID register
	out p_SREG,r12	; 518 1 Restore AVR flags register
no_cs:
	reti			;  611 1
#endif
	
	; Note that in a computer with a 6502 styled bus, you are expected to read the bus when PHI2
	; falls. Due to interrupt latency this is next to impossible in a microcontroller, so the
	; triggering on PHI2 and depending on the correct latency is a necessary alternatice.
	; The MOS 6510 datasheet specifies the moment the data bus is stable in the value T_MDS.
	; It specifies a typical 150ns and max 200ns. This means that the above code is compliant
	; with 6510 timing.

; Explicitely set the current address to make the assembler complain if above code
; is modified and increases in length.
.org 0x01c,0x00

irq_timer0_compa:
    ; The timer 0 compare match A interrupt for every sample. It writes the computed
    ; sample in sample_l/sample_h to the PWN registers of timer 1, which means the audio
    ; output pins PB1/PB2 are updated.
    ;
    ; NOTE: This interrupt service routine is problematic: Because it disables interrupts
    ; while it runs, if a chipselect occurs while it is running, the SwinSID will handle the
    ; chipselect interrupt way too late and will read garbage from the bus.
	sei
	rjmp nexti1			; Looks like a NOP?
nexti1:
	push r23
	lds r23,sample_h	; Get high byte of sample
	sts OCR1AL,r23		; Write to output compare register A
	lds r23,sample_l	; Get low byte of sample
	sts OCR1BL,r23		; Write to output compare register B
	ldi r23,0xff
	sts sample_written,r23 ; Tell main loop to generate new sample
	pop r23
	reti

;****************************************************************************
; Macro for 8-bit signed saturated addition
; This macro adds the number b to a
;****************************************************************************


.macro satadds a b
	add	\a ,\b
	brvc no_overflow\@
	brpl underflow\@
	; Overflow
	ldi \a ,127
	rjmp no_overflow\@
underflow\@:
	ldi \a ,-128
no_overflow\@:
.endm

.macro satadds_crazy a b
	add	\a ,\b
	mov r23,\a				; ??? Why just why move to r23 and back later?
	brvc no_overflow\@
	brpl underflow\@
	; Overflow
	ldi	r23,127
	rjmp no_overflow\@
underflow\@:
	ldi	r23,-128
no_overflow\@:
	mov \a , r23
.endm

;****************************************************************************
; Macro for 16-bit signed saturated addition
; This macro adds the number bh:bl to ah:al
; 
; The implementation of this is broken, because in case of overflow, the low
; byte is left unmodified. It might be on purpose, because the 6581 SID
; filter is known to behave weird in case of saturation, but I seriously
; doubt it because the SID filter is an analog circuit and does not
; behave like an ignored low byte. Most likely it is an unintentional bug.
;****************************************************************************
.macro satadds16 ah al bh bl
	add	\al ,\bl
	adc	\ah ,\bh
	brvc no_overflow\@
	brpl underflow\@
	ldi	\ah ,127
	rjmp no_overflow\@
underflow\@:
	ldi	\ah ,-128
no_overflow\@:
.endm

; Crazy version with same functionality that uses r23 as temp register
.macro satadds16_crazy ah al bh bl
	add	\al ,\bl
	adc	\ah ,\bh
	mov	r23,\ah				; ??? Why just why move to r23 and back later?
	brvc no_overflow\@
	brpl underflow\@
	ldi	r23 ,127
	rjmp no_overflow\@
underflow\@:
	ldi	r23 ,-128
no_overflow\@:
	mov \ah ,r23
.endm

;****************************************************************************
; Macro for 8-bit signed saturated subtractionm
; This macro subtracts the number b from a
;****************************************************************************
.macro satsubs a b
	sub	\a ,\b
	brvc no_overflow\@
	brpl underflow\@
	; Overflow
	ldi	\a ,127
	rjmp no_overflow\@
underflow\@:
	ldi \a ,-128
no_overflow\@:
.endm

;****************************************************************************
; Macro for 16-bit signed saturated subtraction
; This macro subtracts the number bh:bl from ah:al
; 
; The implementation of this is broken, because in case of overflow, the low
; byte is left unmodified. It might be on purpose, because the 6581 SID
; filter is known to behave weird in case of saturation, but I seriously
; doubt it because the SID filter is an analog circuit and does not
; behave like an ignored low byte. Most likely it is an unintentional bug.
;****************************************************************************
.macro satsubs16 ah al bh bl
	sub	\al , \bl
	sbc	\ah , \bh
	mov r23,\ah				; ??? Why just why move to r23 and back later?
	brvc no_overflow\@
	brpl underflow\@
	ldi r23,127
	rjmp no_overflow\@
underflow\@:
	ldi r23,-128
no_overflow\@:
	mov \ah ,r23			; Move back to r5.
.endm


;****************************************************************************
; The following macro generates sample data for a single voice.
; The two parameters are the voice number and its buddy (for ring modulation
; and hard sync). The macro accesses variables through textual expansion.
; After execution of the macro, rhe sample value is returned in r1:r0
;
; Example: genvoice 3 2
; ... generates a sample for voice 3 its buddy is voice 2.

.macro gen_voice v b
	;************************************************************************
	; Oscillator
	;************************************************************************

	lds r23,ctrl\v
	sbrs r23,1		; If voice 3 syncronized with voice 2 (bit 1), skip the rjmp
	rjmp osc_nosync\v
	tst r20				; Need to synchronize?
	breq osc_nosync\v
	; Synchronize!
	clr r23
	sts accu\v\()m,r23
	sts accu\v\()h,r23
osc_nosync\v :
	; Compute accu3:=accu3+24*freq
	; r20 will become non-zero when accu3 overflows
	;
	; Freq is added 24 times, because SID runs at 1MHz, while our sample rate
	; is 1/24 MHz (41116 Hz), so we have to add 24 times to replicate the SID
	; oscillator behaviour.
	;
	; First step: r22:r21 = freqh3 * 12 + hi8(freql3 * 12)
	;             r0 = lo8(freql3 * 12)
	lds r14,freqh\v
	lds r13,freql\v
	clr r19
	ldi r23,12
	mul r14,r23
	mov r22,r1
	mov r21,r0
	mul r13,r23
	add r21,r1
	adc r22,r19
	; Second step: Add r22:r21:r0 twice to accumulator
	lds r19,accu\v\()l
	lds r13,accu\v\()m
	lds r14,accu\v\()h
	clr r20
	add r19,r0
	adc r13,r21
	adc r14,r22
	rol r20
	mov r23,r14   ; ??? Save high byte before first add
	add r19,r0
	adc r13,r21
	adc r14,r22
	rol r20
	sts accu\v\()l,r19
	sts accu\v\()m,r13
	sts accu\v\()h,r14

	mov r13,r23  ; ??? Saved high byte before first add
	
	
	;************************************************************************
	; ADSR
	;************************************************************************
	;
	lds r18,envelope_val\v
	lds r15,ctrl\v
	lds r21,ad\v
	lds r22,sr\v
	lds r25,previous_ctrl\v
	sts previous_ctrl\v ,r15
	mov r23,r25
	andi r23,0xf0		; Was there a waveform enabled in previous sample?
    brne wave_on\v		; Yes then skip.
	mov r23,r15			; ??? why temp move to r23?
	andi r23,0x0f		; If no, then waveform bits in current sample don't matter
	mov r15,r23			; move back to r15
wave_on\v :
	eor	r25,r15			; Compare previous ctrl with current ctrl
	bst	r25,b0			; Has the gate bit been changed?
#ifdef LAZY_JONES_FIX
	brts gate_changed\v	; Yes, then skip
	; Gate bit not changed
	cpi r28,(7*(\v - 1) + 4)	; Was ctrl\v the last register written?
	brne gate_unchanged\v ; No, then skip gate handling
	clr r28				; Clear r28/yl
	bst r15,b0			; Was the gate turned on?
#endif
	brtc gate_unchanged\v ; No, then skip gate handling
gate_changed\v :
	; Gate bit changed
	clr r23
	sts rampcounter\v ,r23
	sts ramppos\v\()l,r23
	sts ramppos\v\()h,r23
	bst r15,b0			; Was gate bit switched on?
	brts gate_enabled\v	; Yes, then skip
	; Gate bit was disabled
	sts	envelope_updown\v ,r23	; Envelope is ramping down
	rjmp setup_release\v
gate_enabled\v :
	ldi r19,1
	sts	envelope_updown\v ,r19	; Envelope is ramping up
	rjmp setup_attack\v
setup_release\v :
	mov r30,r22			; Sustain/release
	andi r30,0x0f		; Get release duration in r30/zl
	ldi r31,hi8(decrel_rates) ; High byte of decay/release table in r31/zh
	lsl r30				; Compute offset
	lpm r0,Z+			; Get low byte of duration
	lpm r1,Z			; Get high byte of duration
	sts ramplength\v\()l,r0
	sts ramplength\v\()h,r1
	rjmp decrel_nextcounter\v
setup_decay\v :
	clr r23
	sts rampcounter\v ,r23
	sts ramppos\v\()l,r23
	sts ramppos\v\()h,r23
	mov r30,r21			; Attack/decay
	andi r30,0x0f		; Get decay duration in r30/zl
	ldi r31,hi8(decrel_rates)  ; High byte of decay/release table in r31/zh
	lsl r30				; A rate entry is 2 bytes
	lpm r0,Z+			; Get low byte of duration
	lpm r1,Z			; Get high byte of duration
	sts ramplength\v\()l,r0
	sts ramplength\v\()h,r1
	rjmp decrel_nextcounter\v
setup_attack\v :
	mov r30,r21			; Attack decay
	andi r30,0xf0		; Mask attach duration		
	swap r30			; Attack duration in r30/zl
	lsl r30				; A rate entry is 2 bytes
	ldi r31,hi8(decrel_rates)
	lpm r0,Z+
	lpm r1,Z
	sts ramplength\v\()l,r0
	sts ramplength\v\()h,r1
	rjmp store_progress_end_adsr\v
gate_unchanged\v :
	; Gate bit unchanged
	;
	; Increase ramppos by 1
	lds r17,ramppos\v\()l
	lds r19,ramppos\v\()h
	ldi r23,1
	add r17,r23
	clr r23
	adc r19,r23
	
	; Did we reach end of ramp?
	lds r8,ramplength\v\()l
	lds r9,ramplength\v\()h
	cp r19,r9			; Compare high byte
    breq end_check_l\v	; If equal compare low byte as well
	sts ramppos\v\()l,r17
	sts ramppos\v\()h,r19
	rjmp end_adsr\v	; Not end of ramp
end_check_l\v :
	cp r17,r8			; Compare low byte
    breq end_reached\v	; If equal then end of ramp
	sts ramppos\v\()l,r17
	sts ramppos\v\()h,r19
	rjmp end_adsr\v		; Not end of ramp

end_reached\v :
	; End of ramp reached
	clr r23
	sts ramppos\v\()l,r23
	sts ramppos\v\()h,r23

	lds r19,envelope_updown\v
	tst r19				; Is envelop ramping up?
	brne env_rampup\v	; Yes then jump
	; If end of ramp and ramping down, we have reached end of release phase.
	; We have reached ramplength, increase the ramp counter
	lds r17,rampcounter\v
	inc r17
	lds r23,rampcounterend\v
	cp r17,r23			; Max repeats reached?
	breq rdown_ctrend_reached\v ; Yes then branch
	sts rampcounter\v ,r17 ; store increased value
	rjmp decrel_nextcounter\v
env_rampup\v :
	mov r30,r21			; Attack decay
	andi r30,0xf0		; Mask attack duration		
	swap r30			; Attack duration in r30/zl
	ldi r31,hi8(decrel_rates)
	lsl r30
	lpm r0,Z+
	lpm r1,Z
	sts ramplength\v\()l,r0
	sts ramplength\v\()h,r1
	ldi r23,255
	cp r18,r23			; End of attack phase reached?
	breq endofattack\v
	inc r18				; Envelope level up
	rjmp store_progress_end_adsr\v
endofattack\v :
	; End of attack phase
	clr r19
	sts envelope_updown\v ,r19
	rjmp setup_decay\v
stepup_release_leveldown\v :
	mov	r30,r22			; Sustain/release
	andi r30,0x0f		; Mask release duration
	ldi r31,hi8(decrel_rates) ; High byte of decay/release table in r31/zh
	lsl r30				; A rate entry is 2 bytes
	lpm r0,Z+			; Get low byte of duration
	lpm r1,Z			; Get high byte of duration
	sts ramplength\v\()l,r0
	sts ramplength\v\()h,r1
	rjmp leveldown\v
rdown_ctrend_reached\v :
	; We are ramping down and have repeated the ramp sufficient times
	clr r23
	sts rampcounter\v ,r23
	tst r18
	breq decrel_nextcounter\v
	bst r15,b0			; Check gate to distinguish between decay and release
	brtc stepup_release_leveldown\v	; If release then jump
	; We are in decay
	mov	r23,r22			; Sustain/release
	andi r23,0xf0		; Max sustain level
	mov r0,r23
	swap r0				; Sustain level in r0
	or r23,r0			; Both high & low nibble contain sustain level
	cp r18,r23			; Sustain level recached?
	brcs decrel_nextcounter\v ; Don't decrease any further if sustain level reached
leveldown\v :
	dec r18				; Envelope level down
decrel_nextcounter\v :
	mov r30,r18
	ldi r31,hi8(exptable)
	lpm r0,Z
	sts rampcounterend\v ,r0
store_progress_end_adsr\v :
	sts envelope_val\v ,r18	; Store adr_progress after modification
end_adsr\v :


	;************************************************************************
	; Waveform
	;************************************************************************

	sbrs r15,b7		; Is noise enabled?
	rjmp no_noisewave\v

	; Noise waveform is enabled
	
	; seed_c3:=(seed_c3+freqh3) shr 1 + freqh3;
	; r23 becomes nonzero if any of the two adds did overflow
	; What is likely going on in the following code is that every sample, a
	; randomization interation happens on C3. When there is an overflow,
	; some bigger randomization iteration happens on seed_a3 and seend_b3
	; and the waveform value is updated.
	lds r0,seed_c\v
	lds r1,freqh\v
	clr r23
	add r0,r1
	rol r23
	lsr r1
	add r0,r1
	rol r23
	sts seed_c\v ,r0
	breq noise_no_overflow\v		; Branch if no overflow
	; Some bit manipulation that looks like a random iteration over
	; a random see;
	lds r13,seed_a\v
	lds r14,seed_b\v
	mov r21,r14
	mov r22,r14
	lsl r21
	eor r21,r22
	bst r21,b7
	bld r13,b0
	lsl r13
	rol r14
	sts seed_a\v ,r13
	sts seed_b\v ,r14
	; More bit manipulation
	clr	r23
	bst r14,b7
	bld r23,b7
	bst r14,b5
	bld r23,b6
	bst r14,b1
	bld r23,b5
	subi r23,0x80		; Store new waveform value?
	sts waveform_val\v ,r23
	rjmp waveval_ready\v
noise_no_overflow\v :
	; Load current waveform value
	lds r23,waveform_val\v
	rjmp waveval_ready\v
test_bit_set\v :
	ldi r23,0x00
	sts accu\v\()h,r23
	sts accu\v\()m,r23
	sts accu\v\()l,r23
	rjmp build_wavetable_ptrh\v
no_noisewave\v :
	 sbrc r15,b3			; If test bit cleared, skip next rjmp
	 rjmp test_bit_set\v	; If test bit set, reset oscillator (repeats each sample)
build_wavetable_ptrh\v :
	ldi r31,0x10			; base address of wavetable data
	rjmp build_wavetable_ptrl\v
#ifndef LAZY_JONES_FIX
	; Dead code, purpose unknown
	mov r24,r16				; R16 is only used below, no idea
	sbrc r16, 5
	add r13, r13
	sbrc r16, 6
	com r13
	lds r23,poty + \v - 1	; Very weird, these are the read-only registers in the SID
	add r13, r23
	andi r16,0x0f	; 15
	add r16,r31
	mov r23,r15
	swap r23
	andi r23,0x0f	; 15
	add r31,r23
	rjmp wave_ptr_ready\v
#endif
no_waveform_selected\v :
	lds r23,freqh\v
	sts waveform_val\v ,r23
	subi r23,0x80
	rjmp waveval_ready\v
build_wavetable_ptrl\v :
	mov r23,r15
	swap r23
	andi r23,0x0f			; waveform mask into low nibble
    breq no_waveform_selected\v
	add r31,r23				; high byte now points to correct wave table
	mov r16,r31				; Save r31
wave_ptr_ready\v:
	bst r23,b2				; Check if pulse is selected
	brtc no_pulse_selected\v
	; Get the pulse width. Least significant 4 bits are thrown away.
	lds	r19,pwl\v
	andi r19,0xf0			; Mask middle 4 bits of pulse width
	lds r25,pwh\v
	andi r25,0x0f			; Mask high 4 bits of pulse widt
	add r25,r19				; Add together
	swap r25				; Swap low & high nibble to get correct pulse width
	mov r19,r25
	mov r30,r14				; R14 still contains accu3h (waveform progress value)
							; that was loaded quite a bit up from here.
	lpm r22,Z				; Get waveform data
	cp r30,r19				; Pulse width threshold exceeded?
	brcc pwm_higha\v
	clr r22					; 0 when beyond pulse width threshold
pwm_higha\v :
	mov r31,r16				; ??? r31 is restored, but it wasn't modified!
	mov r30,r13				; Even weirder, this is the saved value from after the first
							; add to the accumulator far above.
	lpm r21,Z				; Get waveform data again
	cp r30,r19				; Pulse width threshold exceeded?
	brcc pwm_highb\v
	clr r21
pwm_highb\v :
	rjmp waveval_loaded\v
no_pulse_selected\v :
	mov r30,r14				; R14 still contains accu3h (waveform progress value)
							; that was loaded quite a bit up from here.
	lpm r22,Z				; Get waveform data
	mov r31,r16				; ??? r31 is restored, but it wasn't modified!
	mov r30,r13				; ??? this is the saved value from after the first
							; add to the accumulator far above.
	lpm r21,Z
	rjmp waveval_loaded\v
#ifndef LAZY_JONES_FIX
	; Dead code, purpose unknown
	subi r21, 128
	subi r22, 128
	satadds r21,r22
	mov r23,r21
	rjmp waveval_ready\v
#endif
ringmodulation\v :
	; Ring modulate voice with buddy voice
	bst r15,b4				; Triangle enabled?
	brtc waveval_postring\v	; If no then ring modulation is ignored
	lds r1,accu\b\()h
	sbrc r1,b7
	com r23
	rjmp waveval_postring\v
waveval_loaded\v :
	lds r23,waveform_val\v		; save last waveform value
	sts waveform_val\v ,r22		; store new waveform value
	; The following code is a mathematically broken way to average, it does:
	;
	; r23:=(r23+r22) div 2 + r21) div 2 - 128
	;
	; ... but mathematically this isn't the same as:
	;
	; r23:=(r23+r22+r21) div 4 - 128 (and you would rather like to divide by 3)
	add r23,r22
	ror r23
	add r23,r21
	ror r23
	subi r23,0x80
waveval_ready\v :
	 sbrc r15,b2				; ring modulation enabled?
	 rjmp ringmodulation\v		; this rjmp is skipped if not enabled
waveval_postring\v :
	mulsu r23,r18				; Multiply with envelope value (still in r18)
	; Divide by 4:
	asr r1
	ror r0
	asr r1
	ror r0

	clr r7						; ??? r7 is not used
.endm


;****************************************************************************
; Main mixing loop
;
; This loop is continuously executed to generate samples for all voices
;****************************************************************************
	
mixing_loop:
	; Output of voices that need to be filtered will be accumulated in r2/r3
	clr r2
	clr r3
	; Output of voices that doesn't need to be filtered will be accumulated in r4/r5
	clr r4
	clr r5

	;************************************************************************
	; Voice 3
	;************************************************************************
	gen_voice 3 2

	lds r23,reson
	sbrs r23,b2					; Is voice 3 filtered?
	rjmp not_filtered3			; This jump is skipped if filtered
	add r2,r0					; Add to voice data to be filtered
	adc r3,r1					; Add to voice data to be filtered
	clr r0
	clr r1
not_filtered3:
	lds r23,vol_fil
	sbrc r23,b7
	rjmp voice_muted3
	add r4,r0					; Add to voice data not to be filtered
	adc r5,r1					; Add to voice data not to be filtered
voice_muted3:

	;************************************************************************
	; Voice 2
	;************************************************************************
	gen_voice 1 3

	lds r23,reson
	sbrs r23,b0					; Is voice 1 filtered?
	rjmp not_filtered1			; This jump is skipped if filtered
	add r2,r0					; Add to voice data to be filtered
	adc r3,r1					; Add to voice data to be filtered
	clr r0
	clr r1
not_filtered1:
	add r4,r0					; Add to voice data not to be filtered
	adc r5,r1					; Add to voice data not to be filtered

	;************************************************************************
	; Voice 2
	;************************************************************************
	gen_voice 2 1

	lds r23,reson
	sbrs r23,b1					; Is voice 2 filtered?
	rjmp not_filtered2			; This jump is skipped if filtered
	add r2,r0					; Add to voice data to be filtered
	adc r3,r1					; Add to voice data to be filtered
	clr r0
	clr r1
not_filtered2:
	add r4,r0					; Add to voice data not to be filtered
	adc r5,r1					; Add to voice data not to be filtered

	;************************************************************************
	; Filter
	;************************************************************************
	
	; The filter computes:
	; - the high frequency component in r9
	; - the band frequency component in r22:r21
	; - the low  frequency component in r18:r17
	;
	; These are then added to the output depending on which components are
	; enabled in the vol_fil register
	
	; Calculate high frequency component:
	mov r18,r3
	lds r19,filter_acc_low_h
	satsubs r18,r19			; Macro for signed saturated subtraction

	lds r23,reson
	ori r23,0x0f			; Bit 0..3 set, filter resonance in bit 4..7
	mov r24,r23
	ldi r25,150				; Multiply by 150
	mul r23,r25	
	mov r23,r1				; Get high byte
	com r23					; Subtract from 255 (inverts)
	lds r19,filter_acc_band_h
	mulsu r19,r23

	satsubs r18,r1			; Macro for signed saturated subtraction
	mov	r9,r18

	; The SwinSID only looks at the high byte of the cutoff frequency and
	; therefore is inaccurate
	lds r23,filterh
	sbis p_PINB,b0		; 6581 mode?
	rjmp do_6581_filter
	
	;8580 filter
	;cutoff:=cutoff+(cutoff-(cutoff*cutoff) div 256)
	mul r23,r23				; Square
	mov r0,r23
	sub r0,r1				; Subtract high byte of square from original value
	add r23,r0				; Add to original value
	brcs overflow3
do_6581_filter:
	tst r23					; filterh zero?
	brne filterh_nonzero
	inc r23					; make it non-zero
	rjmp filterh_nonzero
overflow3:
	ldi r23,255				; If overflow then saturate
filterh_nonzero:
	mulsu r18,r23
	lds r21,filter_acc_band_l
	lds r22,filter_acc_band_h
	satadds16 r22 r21,r1 r0	; Macro for saturated addition r5:r4 := r22:r21 + r1:r0
	sts filter_acc_band_l,r21
	sts filter_acc_band_h,r22

	mulsu r22,r23
	lds r17,filter_acc_low_l
	lds r18,filter_acc_low_h
	satadds16 r18 r17,r1 r0	; Macro for saturated addition r5:r4 := r18:r17 + r1:r0
	sts filter_acc_low_l,r17
	sts filter_acc_low_h,r18
	
	lds	r2,vol_fil			; Low pass enabled?
	sbrs r2,b4				; Skip next jump if yes
	rjmp no_low_pass		; Jump if low pass not enabled
	; Add low frequency component
	satsubs16 r5 r4,r18 r17	; Macro for saturated substraction r5:r4 := r5:r4 - r18:r17
no_low_pass:
	sbrs r2,b5				; Band pass enabled?
	rjmp no_band_pass		; Jump if low pass not enabled
	; Add band frequency component
	satadds16_crazy r5 r4,r22 r21 ; Macro for saturated addition r5:r4 := r5:r4 + r22:r21
no_band_pass:
	 sbrs r2,b6				; High pass enabled?
	 rjmp no_high_pass		; Jump if low pass not enabled
	; Add high frequency component
	satadds_crazy r5,r9		; Macro for signed saturated addition
no_high_pass:

	;************************************************************************
	; Master volume
	;************************************************************************
	lds r23,vol_fil
	andi r23,0x0f			; Mask master volume
	lds r25,previous_volume
	lds r19,volume_change_progress	; Actual linear volume
	cp r23,r25				; Did the volume change?
	breq volume_unchanged
	; Volume has changed. When the volume changed, a bias is added to the output
	; sample for 255 cycles, in order to allow digi playback.
	mov r25,r23
	sts previous_volume,r25
	ldi r19,255				; If the volume changes, for 255 samples, there is a bias
volume_unchanged:
	tst r19
	breq volume_change_complete
	; Add the bias.
	subi r19,0x01
	sts volume_change_progress,r19
	subi r25,8				; Make it signed Range becomes -8 to 7
	lsl r25					; Multiply by 8 to get the sample at desired volume
	lsl r25
	lsl r25
	; Add to the current sample
	satadds r25,r5			; Macro for signed saturated addition
	mov r5,r25
	rjmp sample_not_written
zero_sample:
	clr r5
	clr r4
	rjmp sample_not_written
volume_change_complete:
	tst r25
	breq zero_sample
sample_not_written:
	; Wait until the last sample has been written to PWM
	lds r23,sample_written
	tst r23		; Zero?
	breq sample_not_written	; If zero then loop
	ldi r23,0x80
	eor r5,r23
	sts sample_h,r5
	sts sample_l,r4
	clr r23
	sts sample_written,r23 ; Reset sample written flag
	rjmp mixing_loop

;*****************************************************************************
; Startup code
;*****************************************************************************

reset:
	cli
	; Set stack pointer to top of SRAM
	ldi r23,lo8(RAMEND)
	out p_SPL,r23
	ldi r23,hi8(RAMEND)
	out p_SPH,r23

	; Set the high byte of the output compare registers to 0, because timer1
	; runs in 8-bit mode (counts only to 255).
	; (the low bytes will be used to output samples)
	ldi r23,0
	sts OCR1AH,r23
	sts OCR1BH,r23

	; Set timer 1 in fast PWM 8-bit mode, enable PWM on PB1/PB2 and set the
	; clock divider to 1 (so timer1 counts at 32MHz).
	ldi r23,0xa1
	sts TCCR1A,r23
	ldi r23,0x09
	sts TCCR1B,r23

	; Set port C and D to input
	ldi r23,0x00
	out p_DDRC,r23
	out p_DDRD,r23

	; Enable built-in pull-up resistors on port C&D
	ldi r23,0xff
	out p_PORTC,r23
	out p_PORTD,r23

	; Enable built-in pull-up resistors on PB0 (6581/8580 selection)
	ldi r23,0x01
	out p_PORTB,r23

	; Set PB1/PB2 (PWM audio output) to output
	ldi r23,0x06
	out p_DDRB,r23

	; Initialize timer 0 (timer interrupt)
	; The interrupt is triggered 32000000 / 8 / 96 = 41667 times per second
	; Set "clear on compare match" mode, disable PWM
	; Timer clock source = CLKIO/8
	ldi r23,0x02
	out p_TCCR0A,r23
	ldi r23,0x02
	out p_TCCR0B,r23
	; OCR0A = 95 ; Reset timer on counter value 95
	ldi r23,95
	out p_OCR0A,r23
	; Enable timer 0 output compare match A interrupt
	ldi r23,0x02
	sts TIMSK0,r23

	; Set voice 1 to 1137.3Hz (between C#6 and D6?)
	; 19081 * 1000000 / 16777216 = 1137.3
	ldi r17,lo8(19081)
	ldi r18,hi8(19081)
	sts freql1,r17
	sts freqh1,r18

	; INT0 (chip select) triggers interrupt on falling edge
	ldi r23,0x02
	sts EICRA,r23
	; Enable external interrupt INT0 (chip select)
	ldi r23,0x01
	out p_EIMSK,r23

	; Set r27/xh (global register) to point to SID registers in SRAM
	ldi r27,hi8(freql1)
	
	; Play a triangle wave on voice 1
	ldi r23,0x11
	sts ctrl1,r23
	ldi r23,0x10
	sts previous_ctrl1,r23	; So gate on is detected

	; Attack 2ms, decay 1.5s, sustain 0, release 6ms
	ldi r23,0x0a
	sts ad1,r23
	ldi r23,0x00
	sts sr1,r23

	; Initialize envelope values at zero volume
	clr r23
	sts envelope_val1,r23
	sts envelope_val2,r23
	sts envelope_val3,r23

	; Initialize voice 2 & 3 to triangle
	ldi r23,0x10
	sts ctrl2,r23
	sts ctrl3,r23

	; Purpose unknown
	sts previous_ctrl2,r23
	sts previous_ctrl3,r23
	
	; Initalize envelope ramp direction for each voice
	clr r23
	sts envelope_updown1,r23
	sts envelope_updown2,r23
	sts envelope_updown3,r23

	; Clear volume ramps (part of ADSR functionality)
	sts ramppos1l,r23
	sts ramppos1h,r23
	sts ramplength1h,r23
	sts ramppos2l,r23
	sts ramppos2h,r23
	sts ramplength2h,r23
	sts ramppos3l,r23
	sts ramppos3h,r23
	sts ramplength3h,r23
	
	; Initialize ramp counters (part of ADSR functionality)
	ldi r23,1
	sts rampcounterend1,r23
	sts rampcounterend2,r23
	sts rampcounterend3,r23
	
	; Initialize volume to 15
	ldi r23,0x0f
	sts vol_fil,r23
	
	; Disable the filter
	clr r23
	sts reson,r23

	; Initialize the filter accumulators
	sts filter_acc_low_h,r23
	sts filter_acc_low_l,r23
	sts filter_acc_band_h,r23
	sts filter_acc_band_l,r23
	
	; Initialize unused registers to 0
	clr r23
	sts empty1d,r23
	sts empty1e,r23
	sts empty1f,r23
	
	; Initialize seeds for random generator for noise
	ldi r23,0x29
	sts seed_a1,r23
	sts seed_b1,r23
	ldi r23,0x97
	sts seed_a2,r23
	sts seed_b2,r23
	ldi r23,0xFB
	sts seed_a3,r23
	sts seed_b3,r23
	
	; Enable interrupts and start main loop
	sei
	rjmp mixing_loop

.data

exptable:
	.byte 1,30,30,30,30,30,30,16,16,16,16,16,16,16,16,8
	.byte 8,8,8,8,8,8,8,8,8,8,8,4,4,4,4,4
	.byte 4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4
	.byte 4,4,4,4,4,4,4,2,2,2,2,2,2,2,2,2
	.byte 2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2
	.byte 2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,1
	.byte 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
	.byte 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
	.byte 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
	.byte 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
	.byte 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
	.byte 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
	.byte 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
	.byte 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
	.byte 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1
	.byte 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1


decrel_rates:
	.word 1
	.word 1
	.word 3
	.word 4
	.word 6
	.word 9
	.word 11
	.word 13
	.word 16
	.word 40
	.word 81
	.word 130
	.word 162
	.word 488
	.word 813
	.word 1302


;.org 0x1000,0xff
.section .wavetable,"a"

	;************************************************************************
	; Start of wave tables
	;************************************************************************

	; Waveform 0: No waveform selected.
	; The following is actually a sine wave:
	.byte 128,131,134,137,140,144,147,150,153,156,159,162,165,168,171,174
	.byte 177,179,182,185,188,191,193,196,199,201,204,206,209,211,213,216
	.byte 218,220,222,224,226,228,230,232,234,235,237,239,240,241,243,244
	.byte 245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255
	.byte 255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246
	.byte 245,244,243,241,240,239,237,235,234,232,230,228,226,224,222,220
	.byte 218,216,213,211,209,206,204,201,199,196,193,191,188,185,182,179
	.byte 177,174,171,168,165,162,159,156,153,150,147,144,140,137,134,131
	.byte 128,125,122,119,116,112,109,106,103,100, 97, 94, 91, 88, 85, 82
	.byte  79, 77, 74, 71, 68, 65, 63, 60, 57, 55, 52, 50, 47, 45, 43, 40
	.byte  38, 36, 34, 32, 30, 28, 26, 24, 22, 21, 19, 17, 16, 15, 13, 12
	.byte  11, 10,  8,  7,  6,  6,  5,  4,  3,  3,  2,  2,  2,  1,  1,  1
	.byte   1,  1,  1,  1,  2,  2,  2,  3,  3,  4,  5,  6,  6,  7,  8, 10
	.byte  11, 12, 13, 15, 16, 17, 19, 21, 22, 24, 26, 28, 30, 32, 34, 36
	.byte  38, 40, 43, 45, 47, 50, 52, 55, 57, 60, 63, 65, 68, 71, 74, 77
	.byte  79, 82, 85, 88, 91, 94, 97,100,103,106,109,112,116,119,122,125

	; Waveform 1: Triangle
	.byte   0,  2,  4,  6,  8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30
	.byte  32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 62
	.byte  64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88, 90, 92, 94
	.byte  96, 98,100,102,104,106,108,110,112,114,116,118,120,122,124,126
	.byte 128,130,132,134,136,138,140,142,144,146,148,150,152,154,156,158
	.byte 160,162,164,166,168,170,172,174,176,178,180,182,184,186,188,190
	.byte 192,194,196,198,200,202,204,206,208,210,212,214,216,218,220,222
	.byte 224,226,228,230,232,234,236,238,240,242,244,246,248,250,252,254
	.byte 254,252,250,248,246,244,242,240,238,236,234,232,230,228,226,224
	.byte 222,220,218,216,214,212,210,208,206,204,202,200,198,196,194,192
	.byte 190,188,186,184,182,180,178,176,174,172,170,168,166,164,162,160
	.byte 158,156,154,152,150,148,146,144,142,140,138,136,134,132,130,128
	.byte 126,124,122,120,118,116,114,112,110,108,106,104,102,100, 98, 96
	.byte  94, 92, 90, 88, 86, 84, 82, 80, 78, 76, 74, 72, 70, 68, 66, 64
	.byte  62, 60, 58, 56, 54, 52, 50, 48, 46, 44, 42, 40, 38, 36, 34, 32
	.byte  30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10,  8,  6,  4,  2,  2

	; Waveform 2: Sawtooth
	.byte   0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
	.byte  16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31
	.byte  32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47
	.byte  48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63
	.byte  64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79
	.byte  80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95
	.byte  96, 97, 98, 99,100,101,102,103,104,105,106,107,108,109,110,111
	.byte 112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127
	.byte 128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143
	.byte 144,145,146,147,148,149,150,151,152,153,154,155,156,157,158,159
	.byte 160,161,162,163,164,165,166,167,168,169,170,171,172,173,174,175
	.byte 176,177,178,179,180,181,182,183,184,185,186,187,188,189,190,191
	.byte 192,193,194,195,196,197,198,199,200,201,202,203,204,205,206,207
	.byte 208,209,210,211,212,213,214,215,216,217,218,219,220,221,222,223
	.byte 224,225,226,227,228,229,230,231,232,233,234,235,236,237,238,239
	.byte 240,241,242,243,244,245,246,247,248,249,250,251,252,253,254,255

	; Waveform 3: Sawtooth + triangle
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  7
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 29
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  8
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 56, 93
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  7
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 33
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  9
	.byte   0,  0,  0,  0,  0,  0,  0, 24,128,128,128,128,128,128,128,130
	.byte 204,198,214,224,224,224,224,224,240,240,240,240,248,248,252,254

	; Waveform 4: Pulse
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255

	; Waveform 5: Pulse + triangle
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 13
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 12
	.byte   0,  0,  0,  0,  0,  0,  0, 30,  0,  4,  8, 68, 50, 97,110,124
	.byte   0,  0,  0,  0,  0,  0,  0,  8,  0,  0,  0, 24,  8, 56,104,130
	.byte   0,  8, 32,112, 96,128,128,130,128,128,128,139,130,152,171,187
	.byte 128,128,128,136,128,156,176,192,164,192,192,193,192,195,201,218
	.byte 192,200,206,224,224,224,225,233,227,238,240,242,246,248,251,254
	.byte 254,251,248,245,242,240,238,227,233,225,224,224,224,206,198,192
	.byte 218,201,195,192,193,192,192,168,192,176,156,128,140,128,128,128
	.byte 187,172,154,130,140,128,128,128,130,128,128, 96,112, 24, 16,  0
	.byte 132,104, 48,  8, 32,  0,  0,  0,  8,  0,  0,  0,  0,  0,  0,  0
	.byte 124,111, 97, 54, 68, 12,  4,  0, 31,  0,  0,  0,  0,  0,  0,  0
	.byte  16,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte  13,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0

	; Waveform 6: Pulse + sawtooth
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  2
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1
	.byte   0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  3,  0,  3,  3, 24
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  5, 30
	.byte   0,  0,  0,  0,  0,  0,  0,  4,  0,  0,  0,  4,  0,  6,  6, 46
	.byte   0,  0,  0,  6,  0, 10, 23, 78,  4, 47, 73,104,101,116,122,134
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  8
	.byte   0,  0,  0,  0,  0,  0,  0,  8,  0,  0,  0,  8,  0, 16, 40,112
	.byte   0,  0,  0,  0,  0,  0,  8, 40,  0,  8,  8, 48,  8, 80,128,135
	.byte   8, 48, 80,128,128,128,130,141,128,131,131,150,139,169,181,189
	.byte  32,112,128,128,128,128,128,132,128,128,128,136,132,148,152,195
	.byte 128,132,132,164,148,188,192,193,192,192,193,198,193,200,209,221
	.byte 192,192,192,194,192,202,204,222,202,216,224,224,224,226,228,237
	.byte 224,225,229,237,237,240,240,244,242,246,248,249,251,252,253,255

	; Waveform 7: Pulse + sawtooth + triangle
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  9,112
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  6
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, 74
	.byte   0,  0,  0, 16,  0, 16, 56,128, 88,128,128,128,128,128,136,192
	.byte 192,192,192,192,192,192,194,216,224,224,224,234,240,243,249,253

	; Waveform 8: Noise
	.byte  75,108,203,124,172,175,168,239,203,161, 62, 94,  0,186,166,  0
	.byte  33,243,120,142,137,196,203, 63, 14, 36, 56,204,  8,139,214,108
	.byte  80,242, 61,216, 61,208,134, 71,168, 30, 73,114,232, 84,157,144
	.byte 214,105, 27, 90,202,213,218,  7,154,145,114, 80,161, 21,252,108
	.byte 181, 88, 16, 20, 46,210,197,127,229,124,180,101, 49,206,244,  0
	.byte 235,191, 29,  5,106,199, 73,175,135,223, 13,179,153,127,131, 76
	.byte 121,158, 67, 47,127,180,100,151,130,186,126, 57,218, 39,171, 79
	.byte  96,244,128,145,107,153, 23, 54,212, 13,  8,247,242,200,106, 11
	.byte 158,195,214, 43, 47,118, 99,142,126,216,169,237,227, 97,193,127
	.byte  52, 10, 67,253,204, 75, 70,158,129, 27, 99, 27,171,242,177,171
	.byte  35,201,200,  7, 64, 23,193,102,219,213, 52,129, 75,123, 56,143
	.byte 105,  0,102, 73,141,220,213,230,141,  8, 29, 31,196,252, 87, 43
	.byte   8,175, 27,195,177,153,128, 30,152,179, 31,244, 20,117, 15,127
	.byte 254,214,232,116,173, 78,196, 14,250,214, 56,  2, 60,230, 94,139
	.byte  76,117,206, 94,130,251,159,182,181,113,105, 72, 61, 78, 70, 78
	.byte 243,139,203,128, 46,160, 18, 21,199,132,179,198, 21,175,197,202

	; Waveform 9: Noise + triangle
	; It's actually a trapezoid
	.byte 128,134,141,147,153,159,166,172,178,184,190,196,202,208,214,220
	.byte 226,232,237,243,249,254,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255
	.byte 255,255,255,255,255,255,255,255,255,255,255,254,249,243,237,232
	.byte 226,220,214,208,202,196,190,184,178,172,166,159,153,147,141,134
	.byte 128,122,115,109,103, 97, 90, 84, 78, 72, 66, 60, 54, 48, 42, 36
	.byte  30, 24, 19, 13,  7,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0
	.byte   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  2,  7, 13, 19, 24
	.byte  30, 36, 42, 48, 54, 60, 66, 72, 78, 84, 90, 97,103,109,115,122

	; Waveform 10: Noise + sawtooth
	; It's actually some kind of double concave ramp
	.byte   0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  2,  2,  2,  3,  3,  4
	.byte   4,  5,  5,  6,  6,  7,  8,  8,  9, 10, 11, 11, 12, 13, 14, 15
	.byte  16, 17, 18, 19, 20, 21, 23, 24, 25, 26, 28, 29, 30, 32, 33, 35
	.byte  36, 38, 39, 41, 42, 44, 46, 47, 49, 51, 53, 54, 56, 58, 60, 62
	.byte  64, 66, 68, 70, 72, 74, 77, 79, 81, 83, 86, 88, 90, 93, 95, 98
	.byte 100,103,105,108,110,113,116,118,121,124,127,129,132,135,138,141
	.byte 144,147,150,153,156,159,163,166,169,172,176,179,182,186,189,193
	.byte 196,200,203,207,210,214,218,221,225,229,233,236,240,244,248,252
	.byte   0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  2,  2,  2,  3,  3,  4
	.byte   4,  5,  5,  6,  6,  7,  8,  8,  9, 10, 11, 11, 12, 13, 14, 15
	.byte  16, 17, 18, 19, 20, 21, 23, 24, 25, 26, 28, 29, 30, 32, 33, 35
	.byte  36, 38, 39, 41, 42, 44, 46, 47, 49, 51, 53, 54, 56, 58, 60, 62
	.byte  64, 66, 68, 70, 72, 74, 77, 79, 81, 83, 86, 88, 90, 93, 95, 98
	.byte 100,103,105,108,110,113,116,118,121,124,127,129,132,135,138,141
	.byte 144,147,150,153,156,159,163,166,169,172,176,179,182,186,189,193
	.byte 196,200,203,207,210,214,218,221,225,229,233,236,240,244,248,252

	; Waveform 11: Noise + sawtooth + triangle
	; It's actually a sine wave
	.byte 128,131,134,137,140,144,147,150,153,156,159,162,165,168,171,174
	.byte 177,179,182,185,188,191,193,196,199,201,204,206,209,211,213,216
	.byte 218,220,222,224,226,228,230,232,234,235,237,239,240,241,243,244
	.byte 245,246,248,249,250,250,251,252,253,253,254,254,254,255,255,255
	.byte 255,255,255,255,254,254,254,253,253,252,251,250,250,249,248,246
	.byte 245,244,243,241,240,239,237,235,234,232,230,228,226,224,222,220
	.byte 218,216,213,211,209,206,204,201,199,196,193,191,188,185,182,179
	.byte 177,174,171,168,165,162,159,156,153,150,147,144,140,137,134,131
	.byte 128,125,122,119,116,112,109,106,103,100, 97, 94, 91, 88, 85, 82
	.byte  79, 77, 74, 71, 68, 65, 63, 60, 57, 55, 52, 50, 47, 45, 43, 40
	.byte  38, 36, 34, 32, 30, 28, 26, 24, 22, 21, 19, 17, 16, 15, 13, 12
	.byte  11, 10,  8,  7,  6,  6,  5,  4,  3,  3,  2,  2,  2,  1,  1,  1
	.byte   1,  1,  1,  1,  2,  2,  2,  3,  3,  4,  5,  6,  6,  7,  8, 10
	.byte  11, 12, 13, 15, 16, 17, 19, 21, 22, 24, 26, 28, 30, 32, 34, 36
	.byte  38, 40, 43, 45, 47, 50, 52, 55, 57, 60, 63, 65, 68, 71, 74, 77
	.byte  79, 82, 85, 88, 91, 94, 97,100,103,106,109,112,116,119,122,125

	.bss

;-----------------------------------------------------
; SID registers at bottom of SRAM
;-----------------------------------------------------
;.org 0x0100 - 96
freql1:	.skip	1
freqh1:	.skip	1
pwl1:	.skip	1
pwh1:	.skip	1
ctrl1:	.skip	1
ad1:	.skip	1
sr1:	.skip	1

freql2:	.skip	1
freqh2:	.skip	1
pwl2:	.skip	1
pwh2:	.skip	1
ctrl2:	.skip	1
ad2:	.skip	1
sr2:	.skip	1

freql3:		.skip	1
freqh3:		.skip	1
pwl3:		.skip	1
pwh3:		.skip	1
ctrl3:		.skip	1
ad3:		.skip	1
sr3:		.skip	1

filterl:	.skip	1
filterh:	.skip	1
reson:		.skip	1
vol_fil:	.skip	1
potx:		.skip	1
poty:		.skip	1
osc3:		.skip	1
env3:		.skip	1
empty1d:	.skip	1
empty1e:	.skip	1
empty1f:	.skip	1

;
; Internal variables of the SID simulation
;
; Note that each voice has exactly 16 bytes of variables
; and the order for each voice is the same, so the same
; variable of the next voice can be reached by adding an
; offset of 16.

; Voice 1
ramppos1l:			.skip	1
envelope_val1:		.skip	1
envelope_updown1:	.skip	1
accu1l:				.skip	1
accu1m:				.skip	1
accu1h:				.skip	1
previous_ctrl1:		.skip	1
waveform_val1:		.skip	1
ramplength1l:		.skip	1
seed_c1:			.skip	1
seed_a1:			.skip	1
seed_b1:			.skip	1
rampcounter1:		.skip	1
rampcounterend1:	.skip	1
ramppos1h:			.skip	1
ramplength1h:		.skip	1

; Voice 2
ramppos2l:			.skip	1
envelope_val2:		.skip	1
envelope_updown2:	.skip	1
accu2l:				.skip	1
accu2m:				.skip	1
accu2h:				.skip	1
previous_ctrl2:		.skip	1
waveform_val2:		.skip	1
ramplength2l:		.skip	1
seed_c2:			.skip	1
seed_a2:			.skip	1
seed_b2:			.skip	1
rampcounter2:		.skip	1
rampcounterend2:	.skip	1
ramppos2h:			.skip	1
ramplength2h:		.skip	1

; Voice 3
ramppos3l:			.skip	1
envelope_val3:		.skip	1
envelope_updown3:	.skip	1
accu3l:				.skip	1
accu3m:				.skip	1
accu3h:				.skip	1
previous_ctrl3:		.skip	1
waveform_val3:		.skip	1
ramplength3l:		.skip	1
seed_c3:			.skip	1
seed_a3:			.skip	1
seed_b3:			.skip	1
rampcounter3:		.skip	1
rampcounterend3:	.skip	1
ramppos3h:			.skip	1
ramplength3h:		.skip	1

; Other
filter_acc_low_l:	.skip	1
filter_acc_low_h:	.skip	1
filter_acc_band_l:	.skip	1
filter_acc_band_h:	.skip	4
sample_l:			.skip	1
sample_h:			.skip	3
sample_written:		.skip	1
previous_volume:	.skip	1
volume_change_progress:	.skip   1
