# SwinSID

This is a reconstruction of the SwinSID Nano source code. The SwinSID was developed between 2005 and 2012 by Swinkels. In 2014 Codekiller release the well known "Lazy Jones fix" firmware, which did fix the audio in the game Lazy Jones.

Years have passed and development of the SwinSID has stalled. The "Lazy Jones fix" firmware is still being used today. Without any source code it is difficult for others to improve the firmware. Swinkels has completely disappeared from the scene and his SwinSID website has now disappeared.

Nevertheless, the SwinSID remains a very popular SID alternative. While there are SID replacements that provide better compatibility, the SwinSID remains an economical solution to bring back sound to a C64. Thus the dead situation of the firmware is bad for the community. In order to end this situation I have reconstructed the SwinSID firmware. If you assemble the source code here, you will end up with an exact copy of the SwinSID "Lazy Jones fix" firmware for the SwinSID Nano.

# How to build

It is assumed that you are building on a Linux system. You need to have cross AVR binutils and avr-libc installed. On (Open)SuSE this can be installed with this command (run as root):

zypper install cross-avr-binutils avr-libc

On CentOS and other Red Hat derived distributions you can use:

yum install avr-binutils avr-libc

On Debian derived distributions you can use:

apt-get install binutils-avr avr-libc

In the Makefile, you may have to adjust the include path (default /usr/avr/sys-root/include ). Point it to the avr-libc include files.

Then you can just type "make" and the SwinSID firmware will be built. It is automatically compared with the original firmware to spot any differences.
