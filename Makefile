
.PRECIOUS: %.o %.elf %.d

CPPFLAGS=-traditional-cpp -I/usr/avr/sys-root/include -D__AVR_ATmega88A__ -D__ASSEMBLER__
FIRMWARES=SwinSID88_lazy_jones_fix.hex SwinSID88_20120524.hex SwinSID88_20141027.hex

all: $(FIRMWARES)

# Dependency file is needed to make sure firmware is rebuilt when include file changes.
%.d: %.asm
		rm -f $@; \
		 cpp -MM $(CPPFLAGS) $< > $@.$$$$; \
		 sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
		 rm -f $@.$$$$


%.o: %.asm %.d
		cpp $(CPPFLAGS) $< | \
		  avr-as -mmcu=atmega88a -o $@

%.elf: %.o swinsid_atmega88.ld
		avr-ld -mavr4 -T swinsid_atmega88.ld -o $@ $<

%.hex: %.elf
		avr-objcopy -j .text -j .wavetable -j .data  -O ihex $< $@
		if [ -f $(basename $<).compare ]; then diff -q $(basename $<).compare $@ ; fi || ( rm $@ ; exit 1 )

clean:
		rm -f *.o *.elf *.hex

include $(FIRMWARES:.hex=.d)
