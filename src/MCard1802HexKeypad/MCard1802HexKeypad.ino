/*
 * Arduino based Hex Keypad digital input and Adafruit 7
 * Segment backpack used to show digital output for an
 * 1802 Membership card.
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
 * All libraries and hardware designs are copyright their respective authors.
 * 
 * Sparkfun Qwiic Keypad Arduino Library
 * Copyright (c) 2019 SparkFun Electronics
 * Written by Pete Lewis @ SparkFun Electronics, 3/12/2019
 * 
 * The Hex_Keypad_Arduino_Library
 * Copyright (c) 2020 by Gaston Williams
 * 
 * The 1802 Membership Card Microcomputer hardware design
 * Copyright (c) 2006-2020 by Lee A. Hart
 * 
 * Many thanks to the original authors for making their designs and code avaialble.
 */
#include "Hex_Keypad_Arduino_Library.h" 

//Change debug token from 0 to 1 to include debug code in compile
#define DEBUG 0

//Define mask for data byte
#define BYTE_MASK  0x00FF
/*
 * Port D Data Direction Mask 
 * (1 = Output, 0 = No Change)
 * Arduino Pin        1802 Membership Card 
 *      D7                    DI5   P1 - 17         
 *      D6                    DI4   P1 - 18
 *      D5                    DI3   P1 - 19
 *      D4                    DI2   P1 - 20
 *      D3                    DI1   P1 - 21
 *      D2                    DI0   P1 - 22
 *      TX (Reserved)         --
 *      RX (Reserved)         --
 */      
#define PORTD_OUTPUT  B11111100 
/*
 * Port B Data Direction Mask 
 * (1 = Output, 0 = No Change)
 * 
 * Arduino Pin        1802 Membership Card
 *      Xtal (Reserved)       --
 *      Xtal (Reserved)       --
 *      D13  (Output Only)  
 *      D12                   /EF4   P1 - 27
 *      D11                   /Wait  P1 - 29
 *      D10                   /Clear P1 - 28 
 *      D9                    DI7    P1 - 16
 *      D8                    DI6    P1 - 15
 */ 
#define PORTB_OUTPUT  B00111111


//Negative logic: must set bit 4 high when /EF4 on D12 is false (HIGH)
#define NO_INPUT      B00100000

//Hold ef4 low for minimum of 50 mSec to simulate keypress
#define KEY_PRESS_DURATION 50 

//Brief wait before next input 
#define COMMAND_DELAY 50

//Pin to poll for Keypad input
#define KEY_READY_PIN A0


//Pin to poll for Input Button
#define INPUT_BTN_PIN A1

//Space character used for invalid key input
#define NO_KEY_CHAR ' '

//allows input button to continuously hold input 
boolean hold_input = false; 

//used to indicate a shift on the keypad
boolean shifted = false;    

//bytes for digital output bits
byte data_hi = 0x00;
byte data_lo = 0x00;

//bytes for Port output
byte data_d = 0x00;
byte data_b = 0x00;

//buffer for digital characters typed from the keypad
uint16_t key_buffer = 0x00;

//External Flag 4 staus used for keyboard input key
boolean input_pressed = false;

//time when input key was pressed
unsigned long t_input_down;

//character from keypad
char c_input = NO_KEY_CHAR;

//Allow character input from serial
boolean allow_serial = true;

//Define Sparkfun Qwiic Keypad
KEYPAD hexKeypad; 


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

    if (button > 0) {
      if (shifted) {
        button = shiftKey(button);
      } else if (button == '*') {
        //First asterisk is shift
        shifted = true;
      } // if-else shifted
    } else {
      //for anything else return a space
      button = NO_KEY_CHAR;
    } // if-else button > 0
  }  // if buttonPressed()
  return button;
} // checkKeypad

//Handle shifted key input
char shiftKey(char key) {
  char shifted_key = NO_KEY_CHAR;
  switch (key) {
    //Shift 1-6 is Hex A-F
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':      
      shifted_key = 'A' + key - '1';
    break;
    
    default:
      //Ignore anything else
    break;    
  } // switch key
  
  //We're done shifting so clear the flag and return the shifted key
  shifted = false;
  return shifted_key;
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
  //shift previous digit into high nibble and clear rest of bits
  key_buffer = (key_buffer << 4) & 0x00F0;
  key_buffer |= getHexValue(h);
  //set data bus values for output
  data_lo = (key_buffer << 2) & BYTE_MASK;
  data_hi = (key_buffer >> 6) & BYTE_MASK;
  #if DEBUG
    Serial.println(F("Key Buffer "));
    Serial.println(key_buffer, HEX);
    Serial.println(F("Data Lo: "));
    Serial.println(data_lo, HEX);
    Serial.println(F("Data Hi: "));
    Serial.println(data_hi, HEX);
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
 *    Port D (Output)
 *      D2                    DI0     P1 - 22
 *      D3                    DI1     P1 - 21
 *      D4                    DI2     P1 - 20
 *      D5                    DI3     P1 - 19
 *      D6                    DI4     P1 - 18
 *      D7                    DI5     P1 - 17
 *      
 *    Port B (Output)  
 *      D8                    DI6     P1 - 16
 *      D9                    DI7     P1 - 15
 *      D10                   
 *      D11                   
 *      D12                   
 *      D13 (Output)          /EF4    P1 - 27
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
  // Set up the Qwiic Keypad communication
  hexKeypad.begin();

  //Set up the outputs for D2 - D7 on port D (Bits 0, 1 are reserved for Serial)
  //DDRD is the port D digital direction register (1 = Output)
  DDRD |= PORTD_OUTPUT;

  //Set up the outputs on D8 - D13 on port B (Bits 6, 7 are reserved)
  //DDRB is the port B digital direction register (1 = Output)
  DDRB |= PORTB_OUTPUT;

  //Set up digital input
  pinMode(KEY_READY_PIN, INPUT);

  
  //Set up digital input
  pinMode(INPUT_BTN_PIN, INPUT_PULLUP);
  //Initialize serial for input and debugging
  Serial.begin(115200);
  
  //Show menu of commands
  showMenu();
} //setup

/*
 * DataInOut 1 - Using Qwiic Keypad for Hex entry,
 * with buttons for Input, Run, Load and M/P
 */
void loop() {
  //Check the input button
  hold_input = !digitalRead(INPUT_BTN_PIN);
  
  if (hold_input) {
    t_input_down = millis();
    input_pressed = true;
    allow_serial = true;
    #if DEBUG
      Serial.println("Input Button down!");
    #endif
  } else if (input_pressed && (millis() - t_input_down < KEY_PRESS_DURATION)) {
    //Continue to hold input key for duration
    input_pressed = true;
    //block input during input keystroke time to avoid Serial overrun
    allow_serial = false;
  } else {
    input_pressed = false;
    allow_serial = true;
  } // if hold_input else if
  
  //Set up data for Port D pins D2 - D7
  data_d = data_lo;  
  
  //Set up data for PORTB pins D8 - D13 
  data_b = data_hi;

  //Set /EF4 bit if needed (Negative Logic: 1 = not pressed)
  if (!input_pressed) {
    data_b |= NO_INPUT;
  } //if !input_pressed
  
  //write out pins D2 - D7 to PORT D
  PORTD = data_d;  
  //write out pins D8 - D13 to PORT B
  PORTB = data_b;
  
  #if DEBUG
    Serial.print(F("Port D: "));
    Serial.println(data_d, HEX);
    Serial.print(F("Port B: "));
    Serial.println(data_b, HEX);
  #endif 

  //get next character
  c_input = checkKeypad();
  
  //If we don't have a character check the Serial input
  if (isSpace(c_input) && allow_serial) {
     c_input = checkSerial();
  } //if isSpace(c_input) && allow_serial
  
  //process any character input
  processChar(c_input);  
} //loop
