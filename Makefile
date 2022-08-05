
.PRECIOUS: %.o %.elf

all: SwinSID88_reconstructed.hex

%.o: %.asm
		cpp -traditional-cpp -I/usr/avr/sys-root/include -D__AVR_ATmega88A__ -D__ASSEMBLER__ $< | \
		  avr-as -mmcu=atmega88a -o $@

%.elf: %.o swinsid_atmega88.ld
		avr-ld -mavr4 -T swinsid_atmega88.ld -o $@ $<

%.hex: %.elf
		avr-objcopy -j .text -j .wavetable -j .data  -O ihex $< $@
		[ -f $(basename $<).compare ] && diff -q $(basename $<).compare $@ || ( rm $@ ; exit 1 )

