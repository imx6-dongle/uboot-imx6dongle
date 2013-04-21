Das U-Boot for GK802
====================

This is a u-boot for GK802/Hi802 "HDMI dongles" based on the Nitrogen6X production u-boot.

You will want

- a serial console, and probably
- to edit the built-in environment variables to suit yourself

Building
--------

You need an ARM cross compiler. For example, if you have `arm-eabi-gcc`:

    $ make gk802\_config CROSS\_COMPILE=arm-eabi-
    $ make ARCH=arm CROSS\_COMPILE=arm-eabi-

This will result in u-boot.imx, which is the imx6 boot image. You will need pkg-config and libusb-1.0 installed to build the USB booting tool.

Booting from SD
-------

Copy the boot image to your SD card - 1kb from the start. For example if `/dev/sdb` is your SD card reader:

    $ dd if=u-boot.imx bs=1k seek=1 of=/dev/sdb

Make sure you don't have a partition in this space! (1MB is a safe place to start your first partition)

Booting from USB
-------

i.MX6 processors, as strapped in the GK802, will fall back into USB boot mode if they don't find a valid boot image on the SD cards.
Use the imxboot tool in the tools directory:

    $ sudo tools/imxboot u-boot.imx
