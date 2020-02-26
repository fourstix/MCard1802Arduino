MCard1802Arduino
================

This repository contains various Arduino programs for supporting the 1802 Membership Card with Arduino based hardware.

My first computer was a Netronics Elf II. It was an RCA 1802 based single board computer that was sold as a kit,
but it had a hexadecimal keypad, an LED, video and an expansion bus. Like many high school kids, I mowed yards
and spent my hard-earned dollars to buy a kit from Netronics [based on their ads.](http://www.cosmacelf.com/gallery/netronics-ads/)

My orignal Elf II was lost in a move long ago, but today Lee Hart's 1802 Membership card duplicates the orignal elf hardware.
However, instead of the orignal Elf front panel, I wanted to simulate the Netronics Elf II interface using hardware I had at hand. 
I wrote Arduino based code to support communication to the 1802 Membership card and to use the Adafruit seven segment display and
a hexadecimal keypad to simulate the Netronics Elf. I also created code to simulate the original Pixie video as well.


Introduction
-------------

A very good source of information on the RCA 1802 chip and Cosmac Elf computer can be found on the 
[CosmacElf web page.](http://www.cosmacelf.com) The 1802 was a fantastic microprocessor that still has quite a 
dedicated following today.

The 1802 Membership card is available from Lee Hart on his [website.](http://www.sunrise-ev.com/1802.htm)  
Additional documentation and other information are availble from Herb Johnson's 
[1802 Membership Card wesite.](http://www.retrotechnology.com/memship/memship.html)

The Sparkfun [Qwiic](https://www.sparkfun.com/qwiic) interface is a 3.3V I2C based interface that makes
it very easy to connect various hardware to the Arduiono.  

Information on the Sparkfun Qwiic interface is available [here.](https://www.sparkfun.com/qwiic)


This code suports the [Adafruit 7 segment LED backpack](https://www.adafruit.com/product/878) for
hexadecimal output Please see the 
[documentation](https://github.com/fourstix/MCard1802Arduino/blob/master/docs/MCard1802SevenSeg.pdf)
for more information.
 
A [Sparkfun Qwiic 3x4 Keypad](https://www.sparkfun.com/products/15290) can be used for key input. The
star key (&ast;) is used as a shift key for hexadeciamal keys A-F (&ast;, 0-6) and control inputs, 
like run, wait (&ast;8), load (&ast;7), etc.  The hash key (#) is used as the input key. See the 
[documentation](https://github.com/fourstix/MCard1802Arduino/blob/master/docs/MCard1802QwiicKeypad.pdf)
for more information.

A Qwiic hexadecimal keypad created from a 4x4 keypad and a 3.3v/8MHz Arduino pro-mini can also be used.
Details about the hexadecimal keypad including firmware are available at the 
[fourstix/Hex_Keypad_Arduino_Library](https://github.com/fourstix/Hex_Keypad_Arduino_Library)
git hub repsository.

This code simulates a cdp1861 Pixi video chip, using an Arduino 5v/16MHz Pro-mini and other hardware.
This code uses a video ram buffer with a 128 x 64 graphics display supported by the
[U8G2 graphics library](https://github.com/olikraus/u8g2) as a video display.  The code will simulate
the interrupts, external flag 1 signal, and DMA Output requests from the original pixie video.  This
allows [programs](https://github.com/fourstix/MCard1802Arduino/blob/master/docs/Cdp1802SampleProgramCode.txt)
written for the original Cosmac Elf hardware to run directly on the simulator.

U8G2 supports many kinds of 128 x 64 displays.  A list of supported displays is available 
[here.](https://github.com/olikraus/u8g2/wiki/u8g2setupcpp)


For example, this [SSD1306 I2C 128 x64 OLED display](https://www.adafruit.com/product/938) available
from Adadruit works fine with the Qwiic interface and is supported by Uthe 8G2 graphics library.

This code uses the MCP23017 Arduino library by Bertrand Lemasle availble on
[github.](https://github.com/blemasle/arduino-mcp23017)
to communicate to the 1802 Membership card via I2C.


 
Sample Configurations
---------------------
Here are some sample configurations running actual [CDP1802 programs](https://github.com/fourstix/QwiicCosmacElfSim/blob/master/docs/Cdp1802SampleProgramCode.txt)
on different combinations of Qwiic compatible hardware.

<table class="table table-hover table-striped table-bordered">
  <tr align="center">
   <td><img src="https://github.com/fourstix/MCard1802Arduino/blob/master/pics/MCard1802IO.JPG"></td>
   <td><img src="https://github.com/fourstix/MCard1802Arduino/blob/master/pics/Spaceship.jpg"></td> 
  </tr>
  <tr align="center">
    <td>1802 Membership card with a Sparkfun Qwiic 3x4 Keypad, two Adafruit 7 Segment backpacks, one for data and another for address,with SH1106 128x64 OLED display</td>
    <td>Close up of SH1106 128x64 OLED display with 1802 Membership card running Cosmac Elf Spaceship program.</td>
  </tr>
  <tr align="center">
   <td><img src="https://github.com/fourstix/MCard1802Arduino/blob/master/pics/MCard1802Mcp23017.jpg"></td>
   <td><img src="https://github.com/fourstix/MCard1802Arduino/blob/master/pics/MCard1802Dual23017.jpg"></td> 
  </tr>
  <tr align="center">
    <td>1802 Membership card with MCP23017, Hexadecimal 4x4 Keypad, Adafruit 7 Segment backpack and an SH1106 128x64 OLED display.</td>
    <td>1802 Membership card with two MCP23017's, Hexadecimal 4x4 Keypad, Adafruit 7 Segment backpack and an SH1106 128x64 OLED display.</td>
  </tr>  
</table>

Repository Contents
-------------------
* **/src/MCard1802AddressDisplay/**
  * MCard1802AddressDisplay.ino -- Arduino based Adafruit 7 Segment backpack used to show the memory address lines for the 1802 Membership card.
* **/src/MCard1802DataDisplay/** 
  * MCard1802DataDisplay.ino -- Arduino based Adafruit 7 Segment backpack used to show digital output for the 1802 Membership card.
* **/src/MCard1802Dual23017/**  
  * MCard1802Dual23017.ino -- Arduino based Hex Keypad digital input and Adafruit 7 Segment backpack used to show digital output for an
1802 Membership card using two MCP23017's for I2C 
 * communication.
* **/src/MCard1802HexKeypad/**  
  * MCard1802HexKeypad.ino -- Arduino based Hex Keypad digital input and Adafruit 7 Segment backpack used to show digital output for an
1802 Membership card.
* **/src/MCard1802Mcp23017/**  
  * MCard1802Mcp23017.ino -- Arduino based Hex Keypad digital input and Adafruit 7 Segment backpack used to show digital output for an
1802 Membership card using an MCP23017 for I2C 
* **/src/MCard1802PixieVideo/**  
  * MCard1802PixieVideo.ino -- Arduino based Pixie Video simulator for 1802 Membership card and a 128 x 64 graphic display. 
* **/src/MCard1802QwiicKeypad/** 
  * MCard1802QwiicKeypad.ino -- Arduino based Qwiic Keypad input for 1802 Membership card.
* **/docs** -- documentation files
  * MCard1802QwiicKeypad.pdf -- documentation of Qwiic Keypad key assignments.
  * MCard1802SevenSeg.pdf -- documentation for Adafruit 7 Segment backpack output.
  * Cdp1802SampleProgramCode.txt -- Sample 1802 code listings
* **/pics** -- pictures of sample configurations


License Information
-------------------

This code is public domain under the MIT License, but please buy me a beer
if you use this and we meet someday (Beerware).

References to any products, programs or services do not imply
that they will be available in all countries in which their respective owner operates.

Sparkfun, the Sparkfun logo, and other Sparkfun products and services are
trademarks of the Sparkfun Electronics Corporation, in the United States,
other countries or both. 

Adafruit, the Adafruit logo, and other Adafruit products and services are
trademarks of the Adafruit Industries, in the United States,other countries or both. 

Other company, product, or services names may be trademarks or services marks of others.

All libraries used in this code are copyright their respective authors.
  
Universal 8bit Graphics Library
Copyright (c) 2016, olikraus
All Rights Reserved
 
Sparkfun Qwiic Keypad Arduino Library
Copyright (c) 2016 SparkFun Electronics
Written by Pete Lewis @ SparkFun Electronics, 3/12/2019

Adadruit LED Backpack Library
Copyright (c) 2012 Adafruit Industries
Written by Limor Fried/Ladyada, 2012 

MCP23017 Arduino Library
Copyright (c) 2017 Bertrand Lemasle

The 1802 Membership Card Microcomputer 
Copyright (c) 2006-2020  by Lee A. Hart.
 
Many thanks to the original authors for making their designs and code avaialble as open source.
 

This code, firmware, and software is released under the [MIT License](http://opensource.org/licenses/MIT).

The MIT License (MIT)

Copyright (c) 2019 by Gaston Williams

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

**THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.**