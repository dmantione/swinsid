# SwinSID

This is a reconstruction of the SwinSID Nano source code. The SwinSID was developed between 2005 and 2012 by Swinkels. In 2014 Codekiller did release the well known "Lazy Jones fix" firmware, which did fix the audio in the game Lazy Jones.

Years have passed and development of the SwinSID has stalled. The "Lazy Jones fix" firmware is still being used today. Without any source code it is difficult for others to improve the firmware. Swinkels has completely disappeared from the scene and his [SwinSID website](http://web.archive.org/web/20191212101114/http://www.swinkels.tvtom.pl/swinsid/) has now disappeared as well.

Nevertheless, the SwinSID remains a very popular SID alternative. While there are SID replacements that provide better compatibility, the SwinSID remains an economical solution to bring back sound to a C64. Thus the dead situation of the firmware is bad for the community. In order to end this situation I have reconstructed the SwinSID firmware. If you assemble the source code here, you will end up with an exact copy of the SwinSID firmware, actually three firmwares will be built:
- SwinSID88_20120524.hex - This is the last release firmware by Swinkels
- SwinSID88_lazy_jones_fix.hex - This is the "Lazy Jones Fix firmware by Codekiller. Most SwinSIDs use this firmware at the moment.
- SwinSID88_20141027.hex - An unreleased firmware by Swinkels. It contains a cleaner fix for Lazy Jones than the fix from Codekiller, but removes checks of the RW line and it is reported that it can cause glitches.

# How to build

It is assumed that you are building on a Linux system. You need to have cross AVR binutils and avr-libc installed. On (Open)SuSE this can be installed with this command (run as root):

`zypper install cross-avr-binutils avr-libc`

On CentOS and other Red Hat derived distributions you can use:

`yum install avr-binutils avr-libc`

On Debian derived distributions you can use:

`apt-get install binutils-avr avr-libc`

In the Makefile, you may have to adjust the include path (default `/usr/avr/sys-root/include` ). Point it to the avr-libc include files.

Then you can just type "make" and the SwinSID firmware will be built. It is automatically compared with the original firmware to spot any differences.
