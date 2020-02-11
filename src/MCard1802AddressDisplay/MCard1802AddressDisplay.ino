/*
 * Arduino based Adafruit 7 Segment backpack used to show the memory
 * address lines for 1802 Membership card.
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
 * An Adafruit 7 Segment I2C backpack is used to show the address bytes
 * on the 1802 membership card memory address lines.  These lines are 
 * available at U2, Pins 10 - 3
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
 *      D7                    MA3   U2, Pin 7
 *      D6                    MA2   U2, Pin 8
 *      D5                    MA1   U2, Pin 9
 *      D4                    MA0   U2, Pin 10
 *      TPB                   White          
 *      TPA                   Green
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
 *      D12                   /Clear P1 - 28
 *      D11                   MA7    P1 - 9
 *      D10                   MA6    P1 - 8            
 *      D9                    MA5    P1 - 7 
 *      D8                    MA4    P1 - 6
 */ 
#define PORTB_INPUT  B11110000

//Single input for /Wait on A0
#define WAIT_PIN  A0

//Single Bit masks

//Bit Mask for /Clear bit at D12
#define CLEAR_MASK B00010000

//Bit Masks for TPA and TPB
#define TPA_BIT   B00000100
#define TPB_BIT   B00001000

//Lower 4 bits in Port B, D11-D8, have the high nibble of address byte
#define HI_NIBBLE_MASK   B00001111
//Upper 4 bits in Port D, D7-D4, have the low nibble of address byte
#define LO_NIBBLE_MASK   B11110000 

//flags for 1802 
boolean tpa = false;
boolean tpb = false;

//Program flags are positive logic, opposite of the bit logic
//For example when the bit flag /Clear = 0, program flag cpu_clear is true.
boolean cpu_wait = false;
boolean cpu_clear = false;

//bytes for Port input
byte data_d   = 0x00;
byte data_b   = 0x00;

//data bytes for hex digits
byte hi_data_d = 0x00;
byte hi_data_b = 0x00;
byte lo_data_d = 0x00;
byte lo_data_b = 0x00;

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
void printBackpackAddress() {
  byte hex_digit = hi_data_b & NIBBLE_MASK;

  //Print first hex digit of upper address byte    
  sevenseg.writeDigitNum(0, hex_digit, false);  

  //Print second hex digit of upper address byte
  hex_digit = (hi_data_d >> 4) & NIBBLE_MASK;
  sevenseg.writeDigitNum(1, hex_digit, false);     

  //Print first hex digit of lower address byte
  hex_digit = lo_data_b & NIBBLE_MASK;
  sevenseg.writeDigitNum(3, hex_digit, false);

  //Print second hex digit of lower address byte
  hex_digit = (lo_data_d >> 4) & NIBBLE_MASK;
  sevenseg.writeDigitNum(4, hex_digit, false);    
 
  //Update the display with changes
  sevenseg.writeDisplay();   
} //printBackpackStatus

//CPU state methods (flags are positive logic, opposite of bit logic)
//true when 1802 is in wait mode
boolean isWaitMode() {
  return cpu_wait && !cpu_clear;
}

//true when 1802 is in reset mode
boolean isResetMode() {
  return !cpu_wait && cpu_clear;
}

//true when 1802 is in reset mode
boolean isLoadMode() {
  return cpu_wait && cpu_clear;
}

//true when 1802 is in run mode
boolean isRunMode() {
  return !cpu_wait && !cpu_clear;
}
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
 *      D2                    TPA     U3, pin 11 (Green)
 *      D3                    TPB     U5, pin 3 (White)
 *      D4                    MA0     U2, Pin 10
 *      D5                    MA1     U2, Pin 9
 *      D6                    MA2     U2, Pin 8
 *      D7                    MA3     U2, Pin 7
 *      
 *    Port B (Input)  
 *      D8                    MA4     U2, Pin 6
 *      D9                    MA5     U2, Pin 5
 *      D10                   MA6     U2, Pin 4
 *      D11                   MA7     U2, Pin 3
 *      D12                   /Clear  P1 - 28
 *      D13 (Output Only)
 *      
 *    /Wait Input and I2C for 7 Segment Backpack
 *      A0                    /Wait   P1 - 29    
 *      A1                    
 *      A2                   
 *      A3
 *      A4 (SDA)              
 *      A5 (SCL)              
 *      
 */
//Zero out all display data variables
void clearData() {
  data_d = 0x00;
  data_b = 0x00;
  hi_data_d = 0x00;
  hi_data_b = 0x00;
  lo_data_d = 0x00;
  lo_data_b = 0x00; 
  tpa = false;
  tpb = false;
} //clearData
 
void setup() {
  //Set up the inputs for D2 - D7 on port D (Bits 0, 1 are reserved)
  //DDRD is the port D digital direction register (0 = Input)
  DDRD &= PORTD_INPUT;

  //Set up the inputs on D8 - D12 on port B (Bits 6, 7 are reserved)
  //DDRB is the port B digital direction register (0 = Input)
  DDRB &= PORTB_INPUT;

  //Set up input pin for /Wait line
  pinMode(WAIT_PIN, INPUT);

  //Setup the Seven Segment Status Display
  sevenseg.begin(0x70);
  blankBackpack();
  
  //Initialize serial for debugging
  #if DEBUG
    Serial.begin(9600);
  #endif
} //setup

/*
 * This is a bit of a hack since it's easy to miss TPA during the load mode.
 * In load mode there is only one TPA pulse during the DMA cycle, followed
 * by multiple TPB pulses.
 * 
 * A better way to do this would be to multiplex the latched Address lines 
 * on U2 using an MCP23017 and send the address bytes over I2C.
 */
void loop() {  
  //Get data from port D and check for either TPA or TPB
  data_d = PIND;
  data_b = PINB;      

  //Negative Logic: cpu_clear is true when bit is low
  cpu_clear = !(data_b & CLEAR_MASK);
  //Negative Logic: cpu_wait is true when bit is low
  cpu_wait  = !digitalRead(WAIT_PIN);

  #if DEBUG
    Serial.print("Clear: ");
    Serial.println(cpu_clear);
    Serial.print("Wait: ");
    Serial.println(cpu_wait);
  #endif

  if (data_d & TPA_BIT) {
    hi_data_d = data_d;
    hi_data_b = data_b; 
    tpa = true;   
  } else if (data_d & TPB_BIT) {
    data_b = PINB;      
    lo_data_d = data_d;
    lo_data_b = data_b;          
    tpb = true;
  } //if TPA else if TPB

  //In reset mode clear captured data and blank display
  if (isResetMode()) {
    #if DEBUG
      Serial.println("Reset");
    #endif
     blankBackpack();
     clearData(); 
   } else if (tpa || tpb) {    
    printBackpackAddress();
    #if DEBUG
      Serial.print("TPA: ");
      Serial.println(tpa);
      Serial.print("Upper Port D: ");
      Serial.println(hi_data_d, HEX);
      Serial.print("Upper Port B: ");
      Serial.println(hi_data_b, HEX);
      
      Serial.print("TPB: ");
      Serial.println(tpb);    
      Serial.print("Lower Port D: ");
      Serial.println(lo_data_d, HEX);
      Serial.print("Lower Port B: ");
      Serial.println(lo_data_b, HEX);    
    #endif    
    tpa = false;
    tpb = false;
    } //if - else if tpa || tpb
} //loop
