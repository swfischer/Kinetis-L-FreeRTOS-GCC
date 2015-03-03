********************************************************************************
		Release notes for usb device demo for kinetis L-series
		        Freescale Semiconductor February 2012   
********************************************************************************

Demo code runs on the TWR-KL25Z48M module.

usb_device example emulates a COM port in a Windows HOST using the Comunication Device Class (CDC). The USB is recognized as a standard COM port and creates an echo aplication. This means that everything that goes out from the terminal returns to the terminal 

===========
Basic Setup
===========

TWR-KL25Z48M setup
----------------
J18 P5V_VREGIN
1-2	ON

================
Demo explanation
================
Download the code and run it.
After the micro USB and the PC are connected, the PC will request a driver, point to the Freescale_CDC_Driver_kinetis.inf file located at the utils folder. 

In the Device Manager window a Freescale CDC device will be found after the enumeration process is completed.

After that just start a hyperterminal opening the COM created by the MCU and start typing. All characters comes back immediately


Note:
usb software needs to set the PLL up to 96MHz, therefore a different systinit.c, tower.h, freedom.h are used and are located in the same folder as this project.