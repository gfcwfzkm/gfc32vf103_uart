# gfc32vf103_uart - FIFO USART library

Easy to use FIFO usart for the GD32VF103 RISC-V microcontrollers by GigaDevices. Since the basic firmware provided by GigaDevices lacks  any extras and are on a rather basic level, I decided to port my xmega-uart-fifo project over to the GD32VF103. 

Simply download the library and link it to your project. The example.c file should provide a basic example on how this library can be used and worked with.

Don't forget to configure the interrupts as the library does not take the part of enabling the interrupts and setting the required priority (eclic).

# License

```textile
MIT License

Copyright (c) 2021 gfcwfzkm

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```