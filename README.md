# Kinetis-L-FreeRTOS-GCC

A Framework for development on Kinetis-L MCUs.

# Purpose

* The main reason for this project was that I was given a few FRDM-KL25Z boards but I could not find a decent baseline that wasn't tied to a specific IDE like Code Warrior or the like.  Being an embedded software engineer I desire to understand all aspects of how the code boots and the hardware gets configured, and most of those IDE based environments tend to hide portions of that process.  GCC is my toolchain of choice, so I used [Andrew Payne's bare metal demo](http://github.com/payne92/bare-metal-arm) as a starting point and built my framework from there.
* In the interest of knowing everything that is in the build and conserving ROM/RAM space, I chose not to include any sort of standard C-library.  This code is specifically built with the "-nostdlib" option and any standard C functions that I needed were either built myself or found on-line.  Note that I've found that even though the "-nostdlib" option is specified, the arm-none-eabi-gcc toolchain will pull in "newlib" (the standard C-library it's built to use) if you include any calls into it.  This can be seen either by looking at the "frdm.asm" file, which the makefile creates, or just be observing a big jump in the "frdm.s19" file size.
* Finally, I wanted to play around with an RTOS I that was not familiar with, so I built on top of FreeRTOS.  FreeRTOS has its own heap code, so this helped confirm my decision about not using a standard C-library.

# Compiling

This code is built and tested via the "arm-none-eabi-gcc" tool chain on an Ubuntu 14.04 virtual machine.

For Ubuntu, the toolchain can be installed via:

* sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
* sudo apt-get update
* sudo apt-get install gcc-arm-none-eabi

The code can be downloaded via:

* git clone https://github.com/swfischer/Kinetis-L-FreeRTOS-GCC.git

Once the toolchain is install and the code has been downloaded, build the code via:

* cd <install-path>/FRDM-KL25Z
* type "make"

The output is a "frdm.s19" file which can be downloaded to an FRDM-KL25Z board and executed.

# Framework Content

The framework contains:

In ./frdm/*:

* Boot code (startup.c)
* a main function which creates 2 threads and starts the OS (main.c)
* a module which performs MCU-wide pin mux configuration (pinmux.c/h)
* a pseudo CMSIS OS API layer (os.c/h)

In ./rtos/*:

* a copy of the FreeRTOS files needed to handle the features used by this framework, including: tasks, timers, event groups, and mutexes.  The FreeRTOS directory structure is maintained here, but only the needed files are populated.

In ./drv/*:

* a driver for the on-board accelerometer (accel.c/h)
* a driver for managing the MCU clock gates (clk.c/h) - providing clock enable/disable counting to prevent conflicts
* a driver for accessing the MCU GPIOs (gpio.c/h) - just a simple wrapper
* a driver for managing the MCU I2C buses (i2c.c/h) - providing bus locking to prevent resource conflicts
* a driver for managing the MCU interrupts (irq.c/h) - providing IRQ register/unregister/enable/disable features, including virtual IRQ support for the individual GPIOs of GPIO banks A & D
* a driver for the on-board multi-color LED (led.c/h)
* a driver for the on-board touch pad (touch.c/h)
* a driver for the console UART via the OpenSDA port (uart.c/h) - this includes a TX path which used DMA to reduce interrupt overhead

In ./util/*:

* a small subset of standard C library functions

In ./apps/*:

* a task which supports used of the LED in a number of different modes (ledTask.c/h)
* a console task and supporting files to implement an interactive console which contains a number of commands to interact with the hardware.  Type "help" for a list of commands.

# Contact

[Steve Fischer](mailto:steve2641@gmail.com)
