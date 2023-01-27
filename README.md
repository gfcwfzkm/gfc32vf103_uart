# gfc32vf103_uart - FIFO USART library

Easy to use FIFO usart for the GD32VF103 RISC-V microcontrollers by GigaDevices. Since the basic firmware provided by GigaDevices lacks  any extras and are on a rather basic level, I decided to port my xmega-uart-fifo project over to the GD32VF103. 

Simply download the library and link it to your project. The example.c file should provide a basic example on how this library can be used and worked with.

Don't forget to configure the interrupts as the library does not take the part of enabling the interrupts and setting the required priority (eclic).

Moved to https://github.com/gfcwfzkm/gd32vf103_libraries
