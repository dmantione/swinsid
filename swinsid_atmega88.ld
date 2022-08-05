OUTPUT_FORMAT("elf32-avr","elf32-avr","elf32-avr")
OUTPUT_ARCH(avr:4)
__FUSE_REGION_LENGTH__ = DEFINED(__FUSE_REGION_LENGTH__) ? __FUSE_REGION_LENGTH__ : 1K;
__LOCK_REGION_LENGTH__ = DEFINED(__LOCK_REGION_LENGTH__) ? __LOCK_REGION_LENGTH__ : 1K;
__SIGNATURE_REGION_LENGTH__ = DEFINED(__SIGNATURE_REGION_LENGTH__) ? __SIGNATURE_REGION_LENGTH__ : 1K;
__USER_SIGNATURE_REGION_LENGTH__ = DEFINED(__USER_SIGNATURE_REGION_LENGTH__) ? __USER_SIGNATURE_REGION_LENGTH__ : 1K;


MEMORY
{
  rom   (rx)   : ORIGIN = 0, LENGTH = 8192
  sram   (rw!x) : ORIGIN = 0x800100, LENGTH = 1024
  eeprom (rw!x) : ORIGIN = 0x810000, LENGTH = 512
  fuse      (rw!x) : ORIGIN = 0x820000, LENGTH = __FUSE_REGION_LENGTH__
  lock      (rw!x) : ORIGIN = 0x830000, LENGTH = __LOCK_REGION_LENGTH__
  signature (rw!x) : ORIGIN = 0x840000, LENGTH = __SIGNATURE_REGION_LENGTH__
  user_signatures (rw!x) : ORIGIN = 0x850000, LENGTH = __USER_SIGNATURE_REGION_LENGTH__
}


SECTIONS
{
  . = 0x0000;
  .text 0x0000 : { *(.text) } > rom
  .data 0x0e00 : { *(.data) } > rom
  .wavetable 0x1000 : { *(.wavetable) } > rom
  .bss 0x800100 : { *(.bss) } > sram
}
