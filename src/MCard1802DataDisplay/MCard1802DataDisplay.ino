/*
 * Arduino based Adafruit 7 Segment backpack used to show digital
 * output the for 1802 Membership card.
 * 
 * Copyright (c) 2020 by Gaston Williams
 * 
 * Based on the 1802 Membership card hardware by Lee Hart. 
 * The 1802 Membership card is available here: 
 * http://www.sunrise-ev.com/1802.htm 
 * 
 * The 1802 Membership Card Microcomputer 
 * Copyright (c) 2006-2020  by Lee A. Hart.
 * 
 * 
 * An Adafruit 7 Segment I2C backpack is used to show the outputs
 * on the 1802 membership card data out lines.  This code could be used as
 * a replacement for TIL311 displays that are no longer readily available.
 * 
 * This code uses Adafruit 7 segment LED backpack 
 * to show Hexadecimal output. It is available at:
 * https://www.adafruit.com/product/878
 * 
 * All libraries and hardware designs are copyright their respective authors.
 * 
 * Adadruit LED Backpack Library
 * Copyright (c) 2012 Adafruit Industries
 * Written by Limor Fried/Ladyada, 2012 
 * 
 * The 1802 Membership Card Microcomputer hardware design
 * Copyright (c) 2006-2020 by Lee A. Hart
 * 
 * Many thanks to the original authors for making their designs and code avaialble.
 */  
#include "Adafruit_LEDBackpack.h"

//Change debug token from 0 to 1 to include debug code in compile
#define DEBUG 0

//Define mask for single hex digit
#define NIBBLE_MASK 0x0F
/*
 * Port D Data Direction Mask 
 * (0 = Input, 1 = No Change)
 * Arduino Pin        1802 Membership Card 
 *      D7                    DO5   P1 - 7         
 *      D6                    DO4   P1 - 6
 *      D5                    DO3   P1 - 5
 *      D4                    DO2   P1 - 4
 *      D3                    DO1   P1 - 3
 *      D2                    DO0   P1 - 2
 *      TX (Reserved)         --
 *      RX (Reserved)         --
 */      
#define PORTD_INPUT  B00000011 
/*
 * Port B Data Direction Mask 
 * (0 = Input, 1 = No Change)
 * 
 * Arduino Pin        1802 Membership Card
 *      Xtal (Reserved)       --
 *      Xtal (Reserved)       --
 *      D13  (Output Only)    --
 *      D12                   MP     (Memory Protect)
 *      D11                   /Wait  P1 - 29
 *      D10                   /Clear P1 - 28 
 *      D9                    DO7    P1 - 9
 *      D8                    DO6    P1 - 8
 */ 
#define PORTB_INPUT  B11100000

//Single Bit masks

//Bit Masks for for /Clear D10 and /Wait D11
#define CLEAR_BIT B00000100
#define WAIT_BIT  B00001000

//Bit Mask for Memory Protect D12
#define MP_BIT    B00010000

//Mask for bits D9,D8 for DO7,DO6
#define DATA_HI   B00000011

//Only one bit used for Q in Port C so no need for direct manipulation
#define Q_PIN A0

//used to indicate the memory should be protected from writes (read-only)
boolean mem_protect = false;

//cdp1802 Wait line
boolean wait_1802 = false;

//cdp1802 Clear line
boolean clear_1802 = false;

//Q output bit
boolean q_bit = false;

//bytes for digital output bits
byte databus = 0x00;


//bytes for Port input
byte data_d = 0x00;
byte data_b = 0x00;

//Setup 4 digit 7 segment hex display
Adafruit_7segment sevenseg = Adafruit_7segment();

/*
 * Seven Segment Display routines
 */

//Blank the Status display 
void blankBackpack() {
  sevenseg.writeDigitRaw(0, 0x00);  
  sevenseg.writeDigitRaw(1, 0x00);
  sevenseg.drawColon(false);
  sevenseg.writeDigitRaw(3, 0x00);
  sevenseg.writeDigitRaw(4, 0x00);
  //Update display
  sevenseg.writeDisplay();
} //blankBackpack

// Show status on 4 digit Seven segment display
void printBackpackStatus() {
  byte hex_digit = 0x00;


    // keep first two digits blank with wait and clear as decimal points
    sevenseg.writeDigitRaw(0, wait_1802 ? 0x80 : 0x00);  
    sevenseg.writeDigitRaw(1, clear_1802 ? 0x80 : 0x00);    

    //Print first hex digit of databus byte with Q bit as decimal point
    hex_digit = (databus >> 4) & NIBBLE_MASK;
    sevenseg.writeDigitNum(3, hex_digit, q_bit);

    //Print second hex digit of databus byte with Memory Protect as decimal
    hex_digit = databus & NIBBLE_MASK;
    sevenseg.writeDigitNum(4, hex_digit, mem_protect);    
 
  
  //Update the display with changes
  sevenseg.writeDisplay();   
} //printBackpackStatus

/*
 * This code uses port manipulation to efficiently write a data byte out
 * to the data input lines of the membership calls.  While it looks a bit
 * awkward, direct port manipulation is much faster than multiple calls 
 * to digitalWrite() or digitalRead() to write or read a set of bits.
 * For single pins, the regular digital I/O functions are just fine.
 * 
 * More information is available here:
 * https://www.arduino.cc/en/Reference/PortManipulation
 * 
 * Arduino Pin        1802 Membership Card
 *    Port D (Input)
 *      D2                    DO0     P1 - 2
 *      D3                    DO1     P1 - 3
 *      D4                    DO2     P1 - 4
 *      D5                    DO3     P1 - 5
 *      D6                    DO4     P1 - 6
 *      D7                    DO5     P1 - 7
 *      
 *    Port B (Input)  
 *      D8                    DO6     P1 - 8
 *      D9                    DO7     P1 - 9
 *      D10                   /Clear  P1 - 28
 *      D11                   /Wait   P1 - 29
 *      D12                   MP      (Memory Protect)
 *      D13 (Output Only)
 *      
 *    I2C pins for 7 Segment Backpack
 *      A4 (SDA)              
 *      A5 (SCL)              
 *      
 */
 
void setup() {
  //Set up the inputs for D2 - D7 on port D (Bits 0, 1 are reserved)
  //DDRD is the port D digital direction register (0 = Input)
  DDRD &= PORTD_INPUT;

  //Set up the inputs on D8 - D12 on port B (Bits 6, 7 are reserved)
  //DDRB is the port B digital direction register (1 = Output)
  DDRB &= PORTB_INPUT;

  //Set up one input bit on port C for Q input
  //Direct port manipulation is overkill for only one or two bits
  pinMode(Q_PIN, INPUT);

  //Setup the Seven Segment Status Display
  sevenseg.begin(0x70);
  blankBackpack();
  
  //Initialize serial for debugging
  #if DEBUG
    Serial.begin(9600);
  #endif
} //setup

void loop() {
  
  //Set up data for Port D pins D2 - D7
  data_d = PIND;  
  
  //Set up data for PORTB pins D8 - D13 
  data_b = PINB;

  //Set Memory Protect flag from bit
  mem_protect = data_b & MP_BIT;
      
  //Set wait and clear flags Negative logic: low is true
  wait_1802  = !(data_b & WAIT_BIT);    
  clear_1802 = !(data_b & CLEAR_BIT);    
  
  databus = (data_b & DATA_HI) << 6;
  databus |= (data_d >> 2);

  q_bit = digitalRead(Q_PIN);
    
  #if DEBUG
    Serial.print("Port D: ");
    Serial.println(data_d, HEX);
    Serial.print("Port B: ");
    Serial.println(data_b, HEX);
    Serial.print("Data Bus");
    Serial.println(databus, HEX);
  #endif 

  //update display
  printBackpackStatus();
} //loop
