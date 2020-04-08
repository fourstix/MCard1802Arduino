/*
 * Arduino based Hex Keypad digital input and Adafruit 7
 * Segment backpack used to show digital output for an
 * 1802 Membership card using an MCP23017 for I2C communication.
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
#define MCP_ADDR  0x20

//I2C Address to Seven Segment LED Backpack
#define DISPLAY_ADDR  0x70

//Define mask for data byte
#define BYTE_MASK  0x00FF

//Define mask for single hex digit
#define NIBBLE_MASK 0x0F

//Hold ef4 low for minimum of 50 mSec to simulate keypress
#define KEY_PRESS_DURATION 50 

//Brief wait before next input 
#define COMMAND_DELAY 50

//Pin to poll for Keypad input
#define KEY_READY_PIN A0

//Pin to poll for Input Button
#define INPUT_BTN_PIN A1


//Negative logic: must set bit high when /EF4 on D13 is false (HIGH)
#define EF4_PIN  13

//Define status input pins
#define  WAIT_PIN 2
#define CLEAR_PIN 3
#define     Q_PIN 4
#define    MP_PIN 5 

//Space character used for invalid key input
#define NO_KEY_CHAR ' '

//allows input button to continuously hold input 
boolean hold_input = false; 

//bytes for digital output bits
byte data_bus = 0x00;

//used to indicate the memory should be protected from writes (read-only)
boolean mem_protect = false;

//cdp1802 Wait line
boolean load_1802 = false;

//cdp1802 Clear line
boolean run_1802 = false;

//Q output bit
boolean q_bit = false;

//buffer for digital characters typed from the keypad
uint16_t key_buffer = 0x00;

//bytes for digital output bits
byte data_hi = 0x00;
byte data_lo = 0x00;

//bytes for Port output
byte data_d = 0x00;
byte data_b = 0x00;


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

//character from keypad
char c_input = NO_KEY_CHAR;

//Allow character input from serial
boolean allow_serial = true;

//Define MCP23017 
MCP23017 mcp = MCP23017(MCP_ADDR);

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


// Check for serial input 
char checkSerial () {
  char cs = NO_KEY_CHAR;  
     if (Serial.available()) {
    // read a character   
    cs = Serial.read();
    } // if Serial.available
    return cs;
} // checkSerial

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

//Menu for Serial input

// Show Menu of commands on Serial monitor
void showMenu() {
  Serial.println(F(" 0-F = hex digit for code input"));
  Serial.println(F(" #,I = Input data to memory during load mode"));
  Serial.println(F("     - Multi-byte input load eg: 00,01,02,03#"));  
  Serial.println(F("     - Simulate an Input button press in run mode")); 
  Serial.println(F(" ? = show this menu")); 
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
  //Negative logic: Load is true when pin low
  load_1802   = !digitalRead(WAIT_PIN);
  //Run follows Clear pin
  run_1802  = digitalRead(CLEAR_PIN);
  
  q_bit       = digitalRead(Q_PIN);
  mem_protect = digitalRead(MP_PIN);
} //set1802Status

// Show status on 4 digit Seven segment display
void printBackpackStatus() {
  byte hex_digit = 0x00;

  // keep first two digits blank with wait and clear as decimal points
  sevenseg.writeDigitRaw(0, run_1802 ? 0x80 : 0x00);  
  sevenseg.writeDigitRaw(1, load_1802 ? 0x80 : 0x00);    

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
 *      D5  (Input)           MP    (Memory Protect)
 *      D4  (Input)           Q     P1-12
 *      D3  (Input)         /CLEAR  P1-28
 *      D2  (Input)         /WAIT   P1-29
 *      
 *    Analog pin for Digital Input
 *      A0                  /RDY from hex Keypad
 *      A1                  Input Button
 *      
 *    I2C pins for Qwiic Keypad  
 *      A4 (SDA)              
 *      A5 (SCL)              
 *      
 */
 
void setup() {
  //Setup MCP23017
  Wire.begin();
  
  //Set up MCP23017
  mcp.init();
  mcp.portMode(MCP23017_PORT::A, 0);         //Port A as ouput
  mcp.portMode(MCP23017_PORT::B, 0b11111111);//Port B as input

  //Initialize GPIO ports
  mcp.writeRegister(MCP23017_REGISTER::GPIOA, 0x00);
  mcp.writeRegister(MCP23017_REGISTER::GPIOB, 0x00);

  // Set up the Qwiic Keypad communication
  hexKeypad.begin();
  
  //Set up digital input
  pinMode(KEY_READY_PIN, INPUT);
    
  //Set up digital input
  pinMode(INPUT_BTN_PIN, INPUT_PULLUP);

  //Set up Input flag /EF4 pin
  pinMode(EF4_PIN, OUTPUT);

  //Initialize status pins
  pinMode(WAIT_PIN, INPUT);
  pinMode(CLEAR_PIN, INPUT);
  pinMode(Q_PIN, INPUT);
  pinMode(MP_PIN, INPUT);
  
  //Setup the Seven Segment Status Display
  sevenseg.begin(DISPLAY_ADDR);
  blankBackpack();
  
  
  //Initialize serial for input and debugging
  Serial.begin(115200);
  
  //Show menu of commands
  showMenu();
} //setup

/*
 * MCard1802Mcp23017 - Update display to show status of
 * Run and Load input buttons, rather than /CLEAR, /WAIT
 * states. 
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
 * MCP23017 Pin     1802 Membership Card
 * Outputs:           Data In: 
 *  28 - GPA7         P1-15   In7 
 *  27 - GPA6         P1-16   In6
 *  26 - GPA5         P1-17   In5
 *  25 - GPA4         P1-18   In4
 *  24 - GPA3         P1-19   In3
 *  23 - GPA2         P1-20   In2
 *  22 - GPA1         P1-21   In1
 *  21 - GPA0         P1-22   In0
 *  
 *  Inputs:           Data Out:
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
 */
void loop() {
  //Check the input button
  hold_input = !digitalRead(INPUT_BTN_PIN);
  
  if (hold_input) {
    t_input_down = millis();
    input_pressed = true;
    allow_serial = true;
    #if DEBUG
      //Print a string of dots
      Serial.print(".");
    #endif
  } else if (input_pressed && waitAfterEvent(t_input_down, KEY_PRESS_DURATION)) {
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
      print2Hex(key_data);
      Serial.println(" to data bus.");
    #endif
    mcp.writeRegister(MCP23017_REGISTER::GPIOA, key_data);
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
         
  //get next character
  c_input = checkKeypad();
  //If we don't have a character check the Serial input
  if (isSpace(c_input) && allow_serial) {
     c_input = checkSerial();
  }
  
  //process any character input
  processChar(c_input);  

  //Save previous data
  old_data_bus = data_bus;
  //Read the input data
  data_bus = mcp.readPort(MCP23017_PORT::B);

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
  
  //update display  
  printBackpackStatus();  
} //loop
