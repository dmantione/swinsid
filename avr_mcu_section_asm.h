#define AVR_MMCU_TAG					 0
#define AVR_MMCU_TAG_NAME				 1
#define AVR_MMCU_TAG_FREQUENCY			 2
#define AVR_MMCU_TAG_VCC				 3
#define AVR_MMCU_TAG_AVCC				 4
#define AVR_MMCU_TAG_AREF				 5
#define AVR_MMCU_TAG_LFUSE				 6
#define AVR_MMCU_TAG_HFUSE				 7
#define AVR_MMCU_TAG_EFUSE				 8
#define AVR_MMCU_TAG_SIGNATURE			 9
#define AVR_MMCU_TAG_SIMAVR_COMMAND		10 
#define AVR_MMCU_TAG_SIMAVR_CONSOLE		11
#define AVR_MMCU_TAG_VCD_FILENAME		12
#define AVR_MMCU_TAG_VCD_PERIOD			13
#define AVR_MMCU_TAG_VCD_TRACE			14
#define AVR_MMCU_TAG_VCD_PORTPIN		15
#define AVR_MMCU_TAG_VCD_IRQ			16
#define AVR_MMCU_TAG_PORT_EXTERNAL_PULL	17


#define SIMAVR_CMD_NONE					 0
#define SIMAVR_CMD_VCD_START_TRACE		 1
#define SIMAVR_CMD_VCD_STOP_TRACE,		 2
#define SIMAVR_CMD_UART_LOOPBACK		 3

.macro AVR_MMCU_BYTE tag byte
	.section .mmcu,""
	.byte tag
	.byte 1 /* length */
__s\@:
	.byte byte
__e\@:
	.previous
.endm

.macro AVR_MMCU_2BYTE tag bytes
	.section .mmcu,""
	.byte tag
	.byte 2 /* length */
__s\@:
	.2byte bytes
__e\@:
	.previous
.endm

.macro AVR_MMCU_4BYTE tag bytes
	.section .mmcu,""
	.byte \tag
	.byte 4 /* length */
	.4byte \bytes
	.previous
.endm


.macro AVR_MMCU_STRING tag str
	.section .mmcu,""
	.byte \tag
	.byte __e\@ - __s\@
__s\@:
	.asciz "\str"
__e\@:
	.previous
.endm

.macro AVR_MCU speed name
	AVR_MMCU_STRING AVR_MMCU_TAG_NAME \name
	AVR_MMCU_4BYTE AVR_MMCU_TAG_FREQUENCY \speed
	.section .mmcu,""
_mmcu:
	.byte AVR_MMCU_TAG
	.byte 0
	.previous
.endm

.macro AVR_MCU_VOLTAGES vcc avcc aref
	AVR_MMCU_4BYTE AVR_MMCU_TAG_VCC \vcc
	AVR_MMCU_4BYTE AVR_MMCU_TAG_AVCC \avcc
	AVR_MMCU_4BYTE AVR_MMCU_TAG_AREF \aref
	
.endm


.macro AVR_MCU_EXTERNAL_PORT_PULL port mask val
	AVR_MCU_4BYTE AVR_MMCU_TAG_PORT_EXTERNAL_PULL ((\port << 16) | (\mask << 8) | val)
.endm

.macro AVR_MCU_VCD_PORT_PIN port pin name
	.section .mmcu,""
	.byte AVR_MMCU_TAG_VCD_PORTPIN
	.byte __e\@ - __s\@
__s\@:
	.byte \port
	.word \pin
	.asciiz "\name"
__e\@:
	.previous

.endm

.macro AVR_MCU_VCD_IRQ_TRACE vect_number what trace_name
	.section .mmcu,""
	.byte AVR_MMCU_TAG_VCD_IRQ
	.byte __e\@ - __s\@
__s\@:
	.byte \vect_number
	.word \what
	.asciiz "\trace_name"
__e\@:
	.previous
.endm

.macro AVR_MCU_VCD_IRQ irq_name
	AVR_MCU_VCD_IRQ_TRACE \irq_name\()_vect_num 1 \irq_name
.endm

.macro AVR_MCU_VCD_IRQ_PENDING
	AVR_MCU_VCD_IRQ_TRACE \irq_name\()_vect_num 0 \irq_name\()_pend
.endm

.macro AVR_MCU_VCD_ALL_IRQ
	AVR_MCU_VCD_IRQ_TRACE 0xff 1 "IRQ"
.endm

.macro AVR_MCU_VCD_ALL_IRQ_PENDING
	AVR_MCU_VCD_IRQ_TRACE 0xff 0 "IRQ_PENDING"
.endm

.macro AVR_MCU_VCD_FILE name period
	AVR_MCU_STRING AVR_MMCU_TAG_VCD_FILENAME \name
	AVR_MCU_4BYTE AVR_MMCU_TAG_VCD_PERIOD \period
.endm

.macro AVR_MCU_SIMAVR_COMMAND register
	AVR_MCU_2BYTE AVR_MMCU_TAG_SIMAVR_COMMAND \register
.endm

.macro AVR_MCU_SIMAVR_CONSOLE register
	AVR_MCU_2BYTE AVR_MMCU_TAG_SIMAVR_CONSOLE \register
.endm
