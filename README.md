Das U-Boot for GK802
====================

This is a u-boot for GK802/Hi802 "HDMI dongles" based on the Nitrogen6X production u-boot.

You will want

- a serial console, and probably
- to edit the built-in environment variables to suit yourself

Building
--------

You need an ARM cross compiler. For example, if you have `arm-eabi-gcc`:

    $ make gk802_config CROSS_COMPILE=arm-eabi-
    $ make ARCH=arm CROSS_COMPILE=arm-eabi-

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


Booting Kernel/Initrd over USB
-------

imxboot can also load a kernel or initrd (or any data) from a file into device memory before launching uboot. This can be useful for kernel development.

Usage is to add as pairs of file and load address to the command line, ie

    tools/imxboot ./u-boot.imx ../linux-imx/arch/arm/boot/zImage 0x10008000

As shown here, uboot command `bootz 0x10008000` will launch the kernel. You can type this on the console, put it in a ubootcmd file on the MMC, change include/configs/gk802.h, or just hack it into the bottom of include/config.h (where it will stay until you run `make gk802_config` again.)

    #undef CONFIG_BOOTCOMMAND
    #define CONFIG_BOOTCOMMAND "bootz 0x10008000"

(For kernel & initrd, just specify another file & address on the command line, and add the address to the bootz arguments.)
