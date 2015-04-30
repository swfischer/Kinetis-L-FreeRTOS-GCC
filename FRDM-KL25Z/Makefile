#******************************************************************************

PROG = frdm

APPS_PATH = ./apps
DRV_PATH  = ./drv
FRDM_PATH = ./frdm
USB_PATH  = ./usb
UTIL_PATH = ./util
RTOS_PATH = ./rtos
HEAP_PATH = $(RTOS_PATH)/portable/MemMang
PORT_PATH = $(RTOS_PATH)/portable/GCC/ARM_CM0

CC = arm-none-eabi-gcc
AR = arm-none-eabi-ar
LD = arm-none-eabi-ld
OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump

DEBUG_OPTS = -g3 -gdwarf-2 -gstrict-dwarf
OPTS = -Os
TARGET = -mcpu=cortex-m0plus -mthumb -mfloat-abi=soft

CFLAGS  = -nostdlib -Wall -Wa,-adhlns="$@.lst" $(TARGET) $(DEBUG_OPTS) $(OPTS)

CFLAGS += -I ./inc \
          -I $(APPS_PATH) \
          -I $(DRV_PATH) \
          -I $(FRDM_PATH) \
          -I $(USB_PATH) \
          -I $(UTIL_PATH) \
          -I $(RTOS_PATH)/include \
          -I $(RTOS_PATH)/portable/GCC/ARM_CM0

LDFLAGS = -T $(FRDM_PATH)/mkl25z4.ld

RTOS_OBJS = $(RTOS_PATH)/list.o \
       $(RTOS_PATH)/queue.o \
       $(RTOS_PATH)/croutine.o \
       $(RTOS_PATH)/tasks.o \
       $(RTOS_PATH)/timers.o \
       $(RTOS_PATH)/event_groups.o \
       $(HEAP_PATH)/heap_1.o \
       $(PORT_PATH)/port.o

OBJS = $(FRDM_PATH)/startup.o \
       $(FRDM_PATH)/main.o \
       $(FRDM_PATH)/pinmux.o \
       $(FRDM_PATH)/os.o \
       $(DRV_PATH)/accel.o \
       $(DRV_PATH)/adc.o \
       $(DRV_PATH)/clk.o \
       $(DRV_PATH)/gpio.o \
       $(DRV_PATH)/i2c.o \
       $(DRV_PATH)/irq.o \
       $(DRV_PATH)/led.o \
       $(DRV_PATH)/servo.o \
       $(DRV_PATH)/touch.o \
       $(DRV_PATH)/uart.o \
       $(USB_PATH)/usbCfg.o \
       $(USB_PATH)/usbCdc.o \
       $(USB_PATH)/usbDev.o \
       $(USB_PATH)/usbTask.o \
       $(UTIL_PATH)/utilsBuf.o \
       $(UTIL_PATH)/utilsHwDelay.o \
       $(UTIL_PATH)/utilsString.o \
       $(UTIL_PATH)/utilsPrintf.o \
       $(APPS_PATH)/consoleTask.o \
       $(APPS_PATH)/consoleCmds.o \
       $(APPS_PATH)/console.o \
       $(APPS_PATH)/ledTask.o \
       $(RTOS_OBJS)

LIBS =

.PHONY: clean

all: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(LIB) -o $(PROG).elf $(OBJS)
	$(OBJDUMP) --disassemble $(PROG).elf > $(PROG).asm
	$(OBJDUMP) -h -t $(PROG).elf > $(PROG).map
	$(OBJCOPY) -O srec $(PROG).elf $(PROG).s19

%.o:%.c
	$(CC) $(CFLAGS) -c -o $@ $<
    
clean:
	rm -f $(APPS_PATH)/*.o $(FRDM_PATH)/*.o $(DRV_PATH)/*.o $(USB_PATH)/*.o $(UTIL_PATH)/*.o
	rm -f $(APPS_PATH)/*.lst $(FRDM_PATH)/*.lst $(DRV_PATH)/*.lst $(USB_PATH)/*.lst $(UTIL_PATH)/*.lst
	rm -f $(RTOS_PATH)/*.o $(HEAP_PATH)/*.o $(PORT_PATH)/*.o
	rm -f $(RTOS_PATH)/*.lst $(HEAP_PATH)/*.lst $(PORT_PATH)/*.lst
	rm -f *.elf *.asm *.s19 *.map
