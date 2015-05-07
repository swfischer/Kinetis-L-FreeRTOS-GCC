# Kinetis-L-FreeRTOS-GCC

A Framework for development on Kinetis-L MCUs.

# Purpose

* The main reason for this project was that I was given a few FRDM-KL25Z boards but I could not find a decent baseline that wasn't tied to a specific IDE such as Code Warrior or the like.  Being an embedded software engineer I desire to understand all aspects of how the code boots and the hardware gets configured, and most of those IDE based environments tend to hide portions of that process.  GCC is my toolchain of choice, so I used [Andrew Payne's bare metal demo](http://github.com/payne92/bare-metal-arm) as a starting point and built my framework from there.
* In the interest of knowing everything that is in the build and conserving ROM/RAM space, I chose not to include any sort of standard C-library.  This code is specifically built with the "-nostdlib" option and any standard C functions that I needed were either built myself or found on-line.  Note that I've found that even though the "-nostdlib" option is specified, the arm-none-eabi-gcc toolchain will pull in "newlib" (the standard C-library it's built to use) if you include any calls into it.  This can be seen either by looking at the "frdm.asm" file, which the makefile creates, or just by observing a big jump in the "frdm.s19" file size.
* Finally, I wanted to play around with an RTOS I that was not familiar with, so I built on top of FreeRTOS.  FreeRTOS has its own heap code, so this helped confirm my decision about not using a standard C-library.

# Functionality

This code, when booted, will produce a command line prompt that can be used to test different portions of the hardware.

As currently configured, the command line is produced on the KL25Z USB port.  The OpenSDA USB port is used for flashing and debugging.  This can be changed such that the OpenSDA USB port is used for the command line by commenting out the "USB_CONSOLE_ENABLED" flag in the "frdm/frdmCfg.h" file.

By default the multi-color LED boots into a "glow" mode where each color (red, green, blue) is slowly and individually changed from off to fully on and back to off.

The commands available at the prompt allow for the following behaviors:

* Set the LED to glow in the default behavior described above ("led glow").
* Set the LED to glow in only one color (for instance "led glow red").
* Set the LED to change the blue color intensity based on the touch pad touch position ("led touch").
* Set the LED to change a specific color intensity based on the touch pad touch position ("led touch green").
* Set the LED color and intensity based on the accelerometer ("led accel").  In this case each color (red, green, blue) is controlled by a specific accelerometer axis, thus different combinations of color can be produced based on rotating the FRDM-KL25Z board on its three axes.
* Display the current accelerometer data ("accel" or "accel timed").
* Display the current touch pad touch position ("touch" or "touch timed").
* Display an ADC channel count value ("adc 1" or "adc 1 timed").
* Display the MCU temperature ("adc temp" or "adc temp timed").
* Display the current clock gate settings ("clock").
* Enable or disable a clock gate ("clock enable UART0" or "clock disable UART0").
* Display a memory location ("mem 0x40049000" or "mem 0x40049000 5").
* Display version and other system information ("version").
* Reboot the KL25Z ("reboot").
* Interact with GPIOs (see "help gpio" and notes 1 & 2)
* Interact with Servo PWM outputs (see "help servo" and notes 1 & 2).  This driver is based on a PIT timer and is expected to control up to 16 separate servo motors (*** functional but not well tested yet).

NOTE 1: For GPIO and servo motor control, the desired output pin must be properly muxed as a GPIO via the "frdm/pinmux.c" file first.  The typical pin ALT setting for GPIO functionality is "1", but you should check the reference manual (Chapter 10: Signal Multiplexing and Signal Descriptions) to be sure.

NOTE 2: GPIO pins are identified by a GPIO ID for the "gpio" and "servo" commands.  The GPIO ID for a given port and pin combination can be determined via the "gpio id" command.  Use the "help gpio" command to learn how the "gpio id" command works.

# Compiling

This code is built and tested via the "arm-none-eabi-gcc" tool chain on an Ubuntu 14.04 virtual machine.

For recent Ubuntu releases, the toolchain can be installed via:

* sudo apt-get install gcc-arm-none-eabi

On older Ubuntu releases, you may need to run these first:

* sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
* sudo apt-get update

The code can be downloaded via:

* git clone https://github.com/swfischer/Kinetis-L-FreeRTOS-GCC.git

Once the toolchain is install and the code has been downloaded, build the code via:

* cd `<install-path>`/FRDM-KL25Z
* type "make"

The output is a "frdm.s19" file which can be downloaded to an FRDM-KL25Z board and executed.

# Framework Content

The framework contains:

In ./frdm/*:

* boot code (startup.c)
* a main function which creates 2 threads and starts the OS (main.c)
* a module which performs MCU-wide pin mux configuration (pinmux.c/h)
* a pseudo CMSIS OS API layer (os.c/h)

In ./rtos/*:

* a copy of the FreeRTOS files needed to handle the features used by this framework, including: tasks, timers, event groups, and mutexes.  The FreeRTOS directory structure is maintained here, but only the needed files are populated.

In ./drv/*:

* a driver for the on-board accelerometer (accel.c/h)
* a driver for accessing the MCU ADC hardware (adc.c/h) - providing bus locking to prevent resource conflicts
* a driver for managing the MCU clock gates (clk.c/h) - providing clock enable/disable counting to prevent conflicts
* a driver for accessing the MCU GPIOs (gpio.c/h) - just a simple wrapper
* a driver for managing the MCU I2C buses (i2c.c/h) - providing bus locking to prevent resource conflicts
* a driver for managing the MCU interrupts (irq.c/h) - providing IRQ register/unregister/enable/disable features, including virtual IRQ support for the individual GPIOs of GPIO banks A & D
* a driver for the on-board multi-color LED (led.c/h)
* a driver for the on-board touch pad (touch.c/h)
* a driver for the console UART via the OpenSDA port (uart.c/h) - this includes a TX path which used DMA to reduce interrupt overhead
* a driver for interacting with GPIOs (gpio.c/h)
* a driver for interacting with Servo PWM signals (servo.c/h) - this includes the ability to independently control up to 16 servo motors (*** functional but not well tested yet)

In ./util/*:

* a small subset of standard C library functions

In ./apps/*:

* a task which supports the use of the LED in a number of different modes (ledTask.c/h)
* a console task and supporting files to implement an interactive console which contains a number of commands to interact with the hardware.  Type "help" at the console prompt, "yes?", for a list of commands.

In ./usb/*:

* ported the Freescale CDC USB driver to the framework
* cleaned up the USB driver to make it more sensible/usable
* the console task can now output the console to either USB port based on the USB_CONSOLE_ENABLED flag in "frdm/frdmCfg.h"

# Contact

[Steve Fischer](mailto:steve2641@gmail.com)
