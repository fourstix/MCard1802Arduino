/*
 * Uses an Arduino based Hex Keypad digital input and an  
 * Adafruit 7 Segment backpack to show digital output for
 * the 1802 Membership card using two MCP23017's for I2C 
 * communication.
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
 * A Sparkfun 4x4 Keypad was used for key input.  The star
 * key * is mapped to E and the hash key # is F.
 * 
 * The Hex Keypad Arduino Library is based upon the 
 * Sparkfun Qwiic Keypad Arduino Library modified for
 * hexadecimal input from a 4x4 Keypad.
 * 
 * See the Hex Keypad Arduino Library Github project for 
 * information, firmware and an Arduino library for this 
 * hexadecimal keypad.
 * 
 * The Hex Keypad Arduino Library is available here:
 * https://github.com/fourstix/Hex_Keypad_Arduino_Library
 * 
 * The 16 button Keypad is available here:
 * https://www.sparkfun.com/products/15290
 * 
 * An Adafruit 7 Segment I2C backpack is used to display the
 * on the 1802 membership card data bus and status information.
 * 
 * It is available at:
 * https://www.adafruit.com/product/878
 * 
 * This code uses the Arduino MCP23017 library 
 * written by Bertrand Lemasle to communicate 
 * via I2C with the 1802 Membership card.
 * 
 * The Arduino MCP23017 library is available here:
 * https://github.com/blemasle/arduino-mcp23017
 * 
 * All libraries and hardware designs are copyright their respective authors.
 * 
 * Sparkfun Qwiic Keypad Arduino Library
 * Copyright (c) 2019 SparkFun Electronics
 * Written by Pete Lewis @ SparkFun Electronics, 3/12/2019
 * 
 * Adadruit LED Backpack Library
 * Copyright (c) 2012 Adafruit Industries
 * Written by Limor Fried/Ladyada, 2012 
 * 
 * The Arduino-MCP23017 library
 * Copyright (c) 2017 Bertrand Lemasle
 * 
 * The Hex_Keypad_Arduino_Library
 * Copyright (c) 2020 by Gaston Williams
 * 
 * The 1802 Membership Card Microcomputer hardware design
 * Copyright (c) 2006-2020 by Lee A. Hart
 * 
 * Many thanks to the original authors for making their designs and code avaialble.
 */
#include <Wire.h>
#include "Adafruit_LEDBackpack.h"
#include "Hex_Keypad_Arduino_Library.h"
#include "MCP23017.h"

//Change debug token from 0 to 1 to include debug code in compile
#define DEBUG 0

//I2C Address for first MCP23017
#define MCP1_ADDR  0x20

//I2C Address for second MCP23017
#define MCP2_ADDR  0x21

//I2C Address to Seven Segment LED Backpack
#define DISPLAY_ADDR  0x70

//Define mask for data byte
#define BYTE_MASK  0x00FF

//Define mask for single hex digit
#define NIBBLE_MASK 0x0F

//Hold ef4 low for minimum of 50 mSec to simulate keypress
#define KEY_PRESS_DURATION 50 

//Minimum time for rapid input
#define RAPID_INPUT_DURATION  10

//Brief wait before next input 
#define COMMAND_DELAY 50

//Pin to poll for Keypad input
#define KEY_READY_PIN A0

//Pin to poll for Input Button
#define INPUT_BTN_PIN A1

//Negative logic: must set bit high when /EF4 on D13 is false (HIGH)
#define EF4_PIN  13

////Define status input pins
//#define  WAIT_PIN 2
//#define CLEAR_PIN 3
//#define     Q_PIN 4
//#define    MP_PIN 5 

//Define Status bits
#define WAIT_BIT  0x01
#define CLEAR_BIT 0x02
#define Q_BIT     0x04
#define MP_BIT    0x08

//Space character used for invalid key input
#define NO_KEY_CHAR ' '

//Invalid Program index for bad input
#define PGM_INVALID -1

//Programs available to load in memory
#define PGM_ZEROS      0
#define PGM_SEQUENCE   1 
#define PGM_SPACESHIP  2
#define PGM_TV_CLOCK   3
#define PGM_VIDEO_TEST 4
#define PGM_MONITOR    5
#define PGM_LIFE       6

//Program Count for input 
#define PGM_COUNT      7

//Page size for loading programs
#define PAGE_SIZE 256

//Size of a Byte written in ASCII characters
#define BYTE_CHAR_SIZE 2

//allows input button to continuously hold input 
boolean hold_input = false; 

//bytes for digital output bits
byte data_bus = 0x00;

//used to indicate the memory should be protected from writes (read-only)
boolean mem_protect = false;

//cdp1802 Load Switch
boolean load_1802 = false;

//cdp1802 Run Switch
boolean run_1802 = false;

//flag for cdp1802 load mode
boolean load_mode = false;

//Q output bit
boolean q_bit = false;

//buffer for digital characters typed from the keypad
uint16_t key_buffer = 0x00;

//bytes for digital output bits
byte data_hi = 0x00;
byte data_lo = 0x00;

//address byte for load display
byte load_address = 0x00;

//key to send to MCP23017 Port A
byte key_data = 0x00;
//previous key sent to MCP23017 Port A
byte old_key_data = 0x00;
//previous data read from MCP23017 Port B
byte old_data_bus = 0x00;

//External Flag 4 staus used for keyboard input key
boolean input_pressed = false;

//time when input key was pressed
unsigned long t_input_down;

//time to wait for input key
int key_duration = KEY_PRESS_DURATION;

//character from keypad
char c_input = NO_KEY_CHAR;

//Allow character input from serial
boolean allow_serial = true;

//program loading variables
boolean loading = false;
byte program_type = 0;
byte nibble_count = 0;
int  byte_index = 0;
int  jump_count = 0;
int  jump_index = 0;
int  pgm_page_size = PAGE_SIZE;

//Define MCP23017's
MCP23017 mcp1 = MCP23017(MCP1_ADDR);
MCP23017 mcp2 = MCP23017(MCP2_ADDR);

//Setup 4 digit 7 segment hex display
Adafruit_7segment sevenseg = Adafruit_7segment();

//Define Sparkfun 4x4 Qwiic Keypad
KEYPAD hexKeypad; 


/*
 * Calculate the time after an event
 *
 * unsigned long eventTimestamp - time event occurred
 * unsigned int duration - duration in milliseconds
 * returns unsigned long - the time passed since event
 *
 * Note: The millis() function takes about 50 days to roll over, but even so
 * since the time values are unsigned long, their difference is always positive and 
 * the correct value, even if the millis() function has rolled over through zero.
 *
 */
unsigned long waitAfterEvent(unsigned long eventTimestamp, unsigned int duration) {
//Current time
unsigned long tick = millis();
  
  return (tick - eventTimestamp) < duration;
}

/*
 * Keypad utilities 
 */
//Check the Keypad int pin and returns true
//if a key is ready to be read from the keypad
boolean buttonPressed() {
  //Ready line goes low when a key is pressed
  return !digitalRead(KEY_READY_PIN);
}

//Check the keypad and get a charcter if button was pressed
char checkKeypad() {
  //Set character to invalid value
  char button = NO_KEY_CHAR;

  // Check ready line and get the last key pressed
  if (buttonPressed()) {
    //Put key press in buffer and read it
    hexKeypad.updateFIFO();
    button = hexKeypad.getButton();

    if (button <= 0) {
      //for anything else return a space
      button = NO_KEY_CHAR;
    } // if-else button > 0
  }  // if buttonPressed()
  return button;
} // checkKeypad

// Pretty print two hex digits for display
void print2Hex(uint8_t v) {  
  //If single Hex digit
  if (v < 0x10) {
   Serial.print(F("0"));
  } // if v < 0x10
  Serial.print(v, HEX);
}

//Get the numeric value of a hexadecimal character
byte getHexValue(char d) {
  byte value = 0x00;
  // check to see what range value is in
  if (isDigit(d)) {
    value = d - '0';   
  } else {
      value = 10 + toupper(d) - 'A';
  } // if else
  return value;
} // getHexValue

//Process a character as a hex digit 
void processHexChar(char h) {
  //Save previous value of key_data
  old_key_data = key_data;
  //shift previous digit into high nibble and clear rest of bits
  key_buffer = (key_buffer << 4) & 0x00F0;
  key_buffer |= getHexValue(h);
  
  //set data bus values for output
  data_lo = (key_buffer << 2) & BYTE_MASK;
  data_hi = (key_buffer >> 6) & BYTE_MASK;  

  //set data value for mcp23017
  key_data = key_buffer & BYTE_MASK;
  
  #if DEBUG
    Serial.print(F("Key Buffer: "));
    print2Hex(key_data);
    Serial.println();
  #endif  
} //processHexChar

//Setup variables to begin loading a program
void beginLoading(int type) {
  loading = true;
  program_type = type;
  nibble_count = 0;
  byte_index = 0;
  //Check for programs larger than one page in size 
  if (type == PGM_LIFE) {
    pgm_page_size = 3 * PAGE_SIZE;
  } else {
    pgm_page_size = PAGE_SIZE;
  }
} // beginLoading


// Check for serial input 
char checkSerial () {
  char cs = NO_KEY_CHAR;  
     if (Serial.available()) {
    // read a character   
    cs = Serial.read();
    } // if Serial.available
    return cs;
} // checkSerial

//Print a not loading message
void printNotLoadingMsg() {
  Serial.println(F("This function is only available in Load Mode at the start of a page, address xx00."));
}

//Check for Load Address before the beginning of a new page boundary
boolean isPageBegin() {
  //if we are at 00 (before page 0) or FF (before page 1 to 7F)
  return (load_address == 0x00 || load_address == 0xFF);
}

//Check for Load Address at next to last byte of a page boundary
boolean isLastProgramByte() {
  //Always 0xFE for every page before the last byte is loaded or skipped
  return (load_address == 0xFE);
} //isLastProgramByte

//Check for Load Address at second to last byte of a page boundary
boolean isLastSkipByte() {
  //Always 0xFD since we want to end at 0xFF
  return (load_address == 0xFD);
} //isLastSkipByte
//Loadable
boolean isLoadable() {
  //Load mode with memory protect off
  return load_mode && !mem_protect;
}

//Examine mode
boolean isExamineMode() {
  //Load mode with memory protect on
  return load_mode && mem_protect;
}

//Process a character from the keypad        
void processChar(char c) {
  //Process a character from either the keypad or serial monitor
  switch(c) {        
    //Input
    case '#':
    case ',':
    case 'I':
    case 'i':
      //Set ef4 true
      input_pressed = true;
      //Set timestamp for holding flag for duration of keypress
      t_input_down = millis();  
      #if DEBUG
        Serial.println(F("Input"));           
      #endif
    break;
    
    //Jump to the end of a page
    case 'J':
    case 'j':
      if (isExamineMode()) {
        Serial.println(F("Jumping to end of the page."));
          jump_count = PAGE_SIZE - load_address - 1;
          //If already at end of page, go to the next page
          if (jump_count <= 0) {
            jump_count = PAGE_SIZE;
          }
          jump_index = 0;
          #if DEBUG
            Serial.print(F("Current Address: "));
            Serial.println(load_address, HEX);
            Serial.print(F("Jump Count: "));
            Serial.println(jump_count, HEX);
          #endif
      } else {
          Serial.println(F("This function is only available in Examine mode, when Load and M/P switches are both on."));
      } // if-else isloadable
    break;

    //load a program from the menu
    case 'P':
    case 'p':
      if (isLoadable() && isPageBegin()) {
        int program_number = PGM_INVALID;
        showProgramMenu();
        program_number = readProgramNumber();
        if (isValidProgram(program_number)) { 
            Serial.print(F("Loading Program "));  
            Serial.println(program_number);
            beginLoading(program_number);
        } else {
          Serial.print(F("Cannot load program. Invalid Program number: "));  
          Serial.println(program_number);
        }
      } else {
        printNotLoadingMsg();
      } // if-else load_mode
    break; 
    
//   //Load a page of with a digital clock program
//    case 'K':
//    case 'k':
//      if (isLoadable() && isPageBegin()) {
//        Serial.println(F("Loading the digital clock program."));
//        beginLoading(PGM_TV_CLOCK);
//      } else {
//        printNotLoadingMsg();
//      } // if-else isloadable
//    break; 
//
//   //Load three pages with a game of life program
//    case 'L':
//    case 'l':  //el not one
//      if (isLoadable() && isPageBegin()) {
//        Serial.println(F("Loading the Game of Life program."));
//        beginLoading(PGM_LIFE);
//      } else {
//        printNotLoadingMsg();
//      } // if-else isloadable
//    break;     
//    
//    case 'M':
//    case 'm':
//      if (isLoadable() && isPageBegin()) {
//        Serial.println(F("Loading the EETOPS Monitor program."));
//        beginLoading(PGM_MONITOR);
//      } else {
//        printNotLoadingMsg();
//      } // if-else isloadable
//    break;         
//    //Load a page of with a sequence of numbers
//    case 'N':
//    case 'n':
//      if (isLoadable() && isPageBegin()) {
//        Serial.println(F("Loading a page with a sequence of numbers."));
//        beginLoading(PGM_SEQUENCE);
//      } else {
//        printNotLoadingMsg();
//      } // if-else isloadable
//    break;
//    //Load a page of with a sequence of numbers
//    case 'S':
//    case 's':
//      if (isLoadable() && isPageBegin()) {
//        Serial.println(F("Loading the Elf Spaceship program."));
//        beginLoading(PGM_SPACESHIP);
//      } else {
//        printNotLoadingMsg();
//      } // if-else load_mode
//    break;
//    
//    //Load a page with the video DMA test program
//    case 'V':
//    case 'v':
//      if (isLoadable() && isPageBegin()) {
//        Serial.println(F("Loading the video DMA test program."));
//        beginLoading(PGM_VIDEO_TEST);
//      } else {
//        printNotLoadingMsg();
//      } // if-else isloadable
//    break;           
//    //Load a page of Zeros
//    case 'Z':
//    case 'z':
//      if (isLoadable() && isPageBegin()) {
//        Serial.println(F("Loading memory page with zeros."));
//        beginLoading(PGM_ZEROS);
//      } else {
//        printNotLoadingMsg();
//      } // if-else load_mode
//    break; 

    //Show menu
    case '?':
      showMenu();
    break;   
      
    //Process Hex characters
    default:         
      if (isHexadecimalDigit(c)) {
        #if DEBUG
          Serial.println(c);                  
        #endif
        processHexChar(c);
      }
       break;
  } // switch    
} //processChar

//Get the duration for an input key stroke
unsigned int getInputKeyDuration() {
  unsigned int duration = KEY_PRESS_DURATION;
  //if we are loading or jumping ahead set to rapid value
  if (loading || (jump_index < jump_count)) {
    duration = RAPID_INPUT_DURATION;
  } // if loading or jumping
  return duration;
} //getInputKeyDuration

//Menu for Serial input

// Show Menu of commands on Serial monitor
void showMenu() {
  Serial.println(F(" 0-F = hex digit for code input"));
  Serial.println(F(" #,I = Input data to memory during load mode"));
  Serial.println(F("     - Multi-byte input load eg: 00,01,02,03#"));  
  Serial.println(F("     - Simulate an Input button press in run mode")); 
  Serial.println();
  Serial.println(F(" J - Jump to end of a page in examine mode."));
  Serial.println(F(" P - autoload a Program in load mode."));
  Serial.println();
  Serial.println(F(" ? = show this menu")); 
}

//Prompt user to enter program number
void showProgramMenu() {
  Serial.println();
  Serial.println(F(" Enter program number below to select the program to load:"));
  Serial.println(F(" 0 - load a page with all Zeros"));
  Serial.println(F(" 1 - load a page with Sequential Numbers"));  
  Serial.println(F(" 2 - load a page with the Spacehip program"));
  Serial.println(F(" 3 - load a page with a Digital Clock program"));
  Serial.println(F(" 4 - load a page with the Video DMA Test program"));
  Serial.println(F(" 5 - load a page with the EETOPS Monitor program"));
  Serial.println(F(" 6 - load 3 pages with a Game of Life program"));
}

//Read program number from serial input until a newline or other character is found
int readProgramNumber() {
    //return an invalid value if no digits in text
    int value = PGM_INVALID;
    boolean found = false;
    while (true) {
    while (! Serial.available());  // wait for characters
    char c = Serial.read();  // read a character    
    if (isdigit(c)) {
      //First time clear invalid value before accumulating
      if (!found) {
        value = 0;
      }
      value *= 10;  //move decimal place for previous digits
      value += c - '0';  // add numeric value
      found = true;
    } else if (found) {
      //if we have found a number, break
      break;
    } 
  } // while reading Serial input
  return value;
}  // readProgramlNumber

//Test whether a program number is valid to load
boolean isValidProgram(int pgm_number) {
  return (PGM_INVALID < pgm_number && pgm_number < PGM_COUNT);
}
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

//Set all the status flags from their repective pin
void set1802Status() {
  //Get the control line status from MCP23017
  byte control_data = mcp2.readPort(MCP23017_PORT::B);
  boolean value = false;

  #if DEBUG
    Serial.print(F("Control lines: "));
    print2Hex(control_data);
    Serial.println();
  #endif
  
  //Negative logic: Load is true when wait bit low
  //Clear all bits but Wait and negate boolean value
  load_1802 = !(control_data & WAIT_BIT);
  
  //Run follows Clear bit
  run_1802 = control_data & CLEAR_BIT;
  

  q_bit = control_data & Q_BIT;

  mem_protect = control_data & MP_BIT;
  
  //Set the flag to indicate load mode
  load_mode = (load_1802 && !run_1802);
} //set1802Status

// Show status on 4 digit Seven segment display
void printBackpackStatus() {
  byte hex_digit = 0x00;
  if (load_mode) {
    //Print first hex digit of address byte with run bit as decimal point
    hex_digit = (load_address >> 4) & NIBBLE_MASK;
    sevenseg.writeDigitNum(0, hex_digit, run_1802);

    //Print second hex digit of data_bus byte with Memory Protect as decimal
    hex_digit = load_address & NIBBLE_MASK;
    sevenseg.writeDigitNum(1, hex_digit, load_1802);    

    //Turn on colon
    sevenseg.drawColon(true);  
  } else {
    // keep first two digits blank with wait and clear as decimal points
    sevenseg.writeDigitRaw(0, run_1802 ? 0x80 : 0x00);  
    sevenseg.writeDigitRaw(1, load_1802 ? 0x80 : 0x00);    
    //turn off colon
    sevenseg.drawColon(false);
  } //if-else load_mode

  //Print first hex digit of data_bus byte with Q bit as decimal point
  hex_digit = (data_bus >> 4) & NIBBLE_MASK;
  sevenseg.writeDigitNum(3, hex_digit, q_bit);

  //Print second hex digit of data_bus byte with Memory Protect as decimal
  hex_digit = data_bus & NIBBLE_MASK;
  sevenseg.writeDigitNum(4, hex_digit, mem_protect);    
 
  
  //Update the display with changes
  sevenseg.writeDisplay();   
} //printBackpackStatus

/*
 *    Arduino Pin           1802 Membership Card                  
 *      D13 (Output)        /EF4    P1 - 27
 *      
 *    Analog pin for Digital Input
 *      A0 (Input)          /RDY from hex Keypad
 *      A1 (Input)          Input Button
 *      
 *    I2C pins for Qwiic Keypad  
 *      A4 (SDA)              
 *      A5 (SCL)              
 *      
 */
 
void setup() {
  //Setup MCP23017
  Wire.begin();
  
  //Set up first MCP23017
  mcp1.init();
  mcp1.portMode(MCP23017_PORT::A, 0);         //Port A as output
  mcp1.portMode(MCP23017_PORT::B, 0b11111111);//Port B as input

  //Initialize GPIO ports
  mcp1.writeRegister(MCP23017_REGISTER::GPIOA, 0x00);
  mcp1.writeRegister(MCP23017_REGISTER::GPIOB, 0x00);

  //Set up second MCP23017
  mcp2.init();
  mcp2.portMode(MCP23017_PORT::A, 0b11111111);//Port A as input
  mcp2.portMode(MCP23017_PORT::B, 0b11111111);//Port B as input

  //Initialize GPIO ports
  mcp2.writeRegister(MCP23017_REGISTER::GPIOA, 0x00);
  mcp2.writeRegister(MCP23017_REGISTER::GPIOB, 0x00);

  // Set up the Qwiic Keypad communication
  hexKeypad.begin();
  
  //Set up digital input
  pinMode(KEY_READY_PIN, INPUT);
    
  //Set up digital input
  pinMode(INPUT_BTN_PIN, INPUT_PULLUP);

  //Set up Input flag /EF4 pin
  pinMode(EF4_PIN, OUTPUT);

//  //Initialize status pins
//  pinMode(WAIT_PIN, INPUT);
//  pinMode(CLEAR_PIN, INPUT);
//  pinMode(Q_PIN, INPUT);
//  pinMode(MP_PIN, INPUT);
  
  //Setup the Seven Segment Status Display
  sevenseg.begin(DISPLAY_ADDR);
  blankBackpack();
  
  
  //Initialize serial for input and debugging
  Serial.begin(115200);
  
  //Show menu of commands
  showMenu();
} //setup

/*
 * MCard1802Dual - Update display during load mode
 * to show the address lower byte.
 * 
 * Data is read from Input port B on the MCP23017 from
 * the Data Output port on the 1802 Membership Card. 
 * 
 * Data is written to the Output port A on MCP23017 into 
 * the Data Input port on the 1802 Membership card. 
 * 
 * Control lines (/CLEAR, /WAIT, Q and MP) are read from 
 * Arduino pins.
 * 
 * First MCP23017 
 * Pin             1802 Membership Card
 * Port A Outputs:    Data In: 
 *  28 - GPA7         P1-15   In7 
 *  27 - GPA6         P1-16   In6
 *  26 - GPA5         P1-17   In5
 *  25 - GPA4         P1-18   In4
 *  24 - GPA3         P1-19   In3
 *  23 - GPA2         P1-20   In2
 *  22 - GPA1         P1-21   In1
 *  21 - GPA0         P1-22   In0
 *  
 *  Port B Inputs:    Data Out:
 *   8 - GPB7         P1-9    Out7
 *   7 - GPB6         P1-8    Out6
 *   6 - GPB5         P1-7    Out5
 *   5 - GPB4         P1-6    Out4
 *   4 - GPB3         P1-5    Out3
 *   3 - GPB2         P1-4    Out2
 *   2 - GPB1         P1-3    Out1
 *   1 - GPB0         P1-2    Out0
 *  
 *  Other Pin connections
 *  
 *  18 - Reset (to +VDD)
 *  17 - A2 (to GND for I2C address 0x20)
 *  16 - A1 (to GND for I2C address 0x20) 
 *  15 - A0 (to GND for I2C address 0x20)
 *  
 *  13 - SDA (to Arduino Pin A4, SDA)
 *  12 - SCL (to Arduino Pin A5, SCL)
 *  
 *   9 VDD to +VDD
 *   8 GND to GND
 *   
 * Second MCP23017 
 * Pin             1802 Membership Card
 * Port A Inputs:  Memory Address Bus:   
 *  28 - GPA7         U2, Pin3  MA7
 *  27 - GPA6         U2, Pin4  MA6
 *  26 - GPA5         U2, Pin5  MA5
 *  25 - GPA4         U2, Pin6  MA4
 *  24 - GPA3         U2, Pin7  MA3
 *  23 - GPA2         U2, Pin8  MA2
 *  22 - GPA1         U2, Pin9  MA1
 *  21 - GPA0         U2, Pin10 MA0
 *  
 *  Port B Inputs:    Control line:
 *   8 - GPB7         N.C.
 *   7 - GPB6         N.C.
 *   6 - GPB5         N.C.
 *   5 - GPB4         N.C.
 *   4 - GPB3         U9, Pin 5 MP
 *   3 - GPB2         P1-12     Q
 *   2 - GPB1         P1-28   /Clear
 *   1 - GPB0         P1-29    Out0
 *  
 *  Other Pin connections
 *  
 *  18 - Reset (to +VDD)
 *  17 - A2 (to GND for I2C address 0x21)
 *  16 - A1 (to GND for I2C address 0x21) 
 *  15 - A0 (to +VDD for I2C address 0x21)
 *  
 *  13 - SDA (to Arduino Pin A4, SDA)
 *  12 - SCL (to Arduino Pin A5, SCL)
 *  
 *   9 VDD to +VDD
 *   8 GND to GND
 */
void loop() {
  //Check the input button
  hold_input = !digitalRead(INPUT_BTN_PIN);
  //Adjust key duration if jumping ahead or loading program
  key_duration = getInputKeyDuration();
  if (hold_input) {
    t_input_down = millis();
    input_pressed = true;
    allow_serial = true;
    #if DEBUG
      //Print a string of dots
      Serial.print(".");
    #endif
  } else if (input_pressed && waitAfterEvent(t_input_down, key_duration)) {
    //Continue to hold input key for duration
    input_pressed = true;
    //block input during input keystroke time to avoid Serial overrun
    allow_serial = false;
  } else {
    input_pressed = false;
    allow_serial = true;
  } // if hold_input else if
  
  //If the key data has changed write to 1802
  if (key_data != old_key_data) {
    #if DEBUG
      Serial.print("Writing ");
      print2Hex(key_data);c
      Serial.println(" to data bus.");
    #endif
    mcp1.writeRegister(MCP23017_REGISTER::GPIOA, key_data);
    old_key_data = key_data;
  } //if key_data != old_key_data


  //Set /EF4 bit 
  //Negative Logic: 0 = pressed, 1 = not pressed
  if (input_pressed) {
    //Negative Logic: /EF4 true when LOW
    digitalWrite(EF4_PIN, LOW);
  } else {
    //Negative Logic: /EF4 false when HIGH
    digitalWrite(EF4_PIN, HIGH);
  } //if-else input_pressed

  //Jumpring and Loading takes precedence over key input and serial input
  if ((jump_index < jump_count) && allow_serial) {
    //Enter input until we reach the end of the page
    c_input = ',';
    jump_index++;
    //if skipping stops before the end of the page, do one more byte
    //This can happen after a reset
    if ((jump_index == jump_count) && !isLastSkipByte()) {
      #if DEBUG
        Serial.print(F("Last Address: "));
        Serial.println(load_address, HEX);
      #endif
      //Adjust jump count for last remaining byte
      jump_count = 1;
      jump_index = 0;
    } // if !isLastByteBeforePageEnd()
  } else if (loading && allow_serial) {
    if (nibble_count >= BYTE_CHAR_SIZE) {
      //Enter input to load program byte
      c_input = ',';
      //Set nibble count for next byte
      nibble_count = 0;
      byte_index++;
      //After loading an entire page, turn off loading
      if (byte_index >= pgm_page_size) {
        //This will be our last byte to load
        loading = false;
        //Check address and print warning message if not at end of page
        if (!isLastProgramByte()) {
          Serial.print(F("Program did not load to end of page.  Current address: "));
          Serial.println(load_address, HEX);
        } //!isLastProgramByte 
      } //if byte_index      
    } else {
      //Get next character from input byte's two hex nibble characters
      c_input = getNextByteChar(program_type, byte_index, nibble_count);
      nibble_count++;
    } //if-else nibble_count >= BYTE_CHAR_SIZE  
  } else {
  //get next character
    c_input = checkKeypad();
    //If we don't have a character check the Serial input
    if (isSpace(c_input) && allow_serial) {
       c_input = checkSerial();
    } // isSpace && allow_serial
  }
  //process any character input
  processChar(c_input);  

  //Save previous data
  old_data_bus = data_bus;
  //Read the input data
  data_bus = mcp1.readPort(MCP23017_PORT::B);

  #if DEBUG
    if (data_bus != old_data_bus) {
      Serial.println();
      Serial.print(F("Data bus: "));
      print2Hex(data_bus);
      Serial.println();
    } // if data_bus != old_data_bus
  #endif
  
  //update status flags
  set1802Status();

  //get the address byte for load display from Port A
  if (load_mode) {
    load_address = mcp2.readPort(MCP23017_PORT::A);
    #if DEBUG
      Serial.print(F("Address byte: "));
      print2Hex(load_address);
      Serial.println();
    #endif 
  } else {
    //Else set it to zero
    load_address = 0;
  } //if load_mode
  
  //update display  
  printBackpackStatus();  
} //loop
