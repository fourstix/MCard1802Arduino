/*
 * Arduino based Pixie Video simulator for 1802 Membership card
 * Copyright (c) 2020 by Gaston Williams
 * 
 * Based on the 1802 Membership card hardware by Lee Hart. 
 * The 1802 Membership card is available here: 
 * http://www.sunrise-ev.com/1802.htm 
 * 
 * The 1802 Membership Card Microcomputer 
 * Copyright (c) 2006-2020  by Lee A. Hart.
 * 
 * This code simulates a cdp1861 Pixi video chip to load a video
 * ram buffer then update a 128 x 64 OLED display.  The EF1 flag, 
 * interrupts and dma lines are all manipulated as in a real cdp1861 
 * chip running in 32 x 64 resolution.
 *
 * The U8G2 graphics library is available at:
 * https://github.com/olikraus/u8g2
 * 
 * This code uses a 128 x 64 graphics display supported by the U8G2 library
 * as a video display.  U8G2 supports many kinds of 128 x 64 displays.  
 * 
 * Please see https://github.com/olikraus/u8g2/wiki/u8g2setupcpp for a list.
 * 
 * This SSD1306 I2C 128 x64 OLED display available from 
 * Adadruit works fine: https://www.adafruit.com/product/938
 * 
 * All libraries are copyright their respective authors.
 * 
 * Universal 8bit Graphics Library
 * Copyright (c) 2016, olikraus@gmail.com
 * All Rights Reserved
 * 
 * The 1802 Membership Card Microcomputer hardware design
 * Copyright (c) 2006-2020 by Lee A. Hart
 * 
 * Many thanks to the original authors for making their designs and code avaialble.
 */
#include <U8g2lib.h>
 
//Change debug token from 0 to 1 to include debug code in compile
#define DEBUG 0

//Change debug timing token from 0 to 1 to measure times
//Note the debug logic printouts will affect times, so leave it off 
//When doing timing measurements
#define DEBUG_TIMING 0
/*
 * Port D Data Direction Mask 
 * (0 = Input, 1 = No Change)
 * Arduino Pin        1802 Membership Card 
 *      D7                    DB7   
 *      D6                    DB6   
 *      D5                    DB5   
 *      D4                    DB4 
 *      D3                    SC1   
 *      D2                    SC0  
 *      TX (Reserved)         --
 *      RX (Reserved)         --
 */      
#define PORTD_INPUT     B00000011 

//Mask for DMA data bits
#define D_DATA_MASK     B11110000

//Mask for DMA data bits
#define D_STATUS_MASK   B00001100

//Masks for DMA status bits
#define DMA_STATUS_CODE B00001000

//Masks for DMA status bits
#define INT_STATUS_CODE  B00001100

/*
 * Port B Data Direction Mask 
 * (0 = Input, 1 = No Change)
 * 
 * Arduino Pin        1802 Membership Card
 *      Xtal (Reserved)       --
 *      Xtal (Reserved)       --
 *      D13  (Output Only)    /EF1
 *      D12                   VIDEO
 *      D11                   DB3
 *      D10                   DB2
 *      D9                    DB1
 *      D8                    DB0
 */ 
#define PORTB_INPUT  B11100000
//Mask for DMA data bits
#define B_DATA_MASK  B00001111

//Mask for Video On bit
#define VIDEO_ON_BIT B00010000

//Pin 13 (LED Pin) is the external flag line for video
#define EF1_PIN 13
/*
 * 
 * Arduino Pin        1802 Membership Card
 * 
 *    Analog pins for Digital Output
 *      A0                /INT
 *      A1                /DMA_OUT
 *      A2                /SS (Low for Single-Step mode)
 *      A3                STEP (High for single step)
 *
 *    I2C pins
 *      A4 (SDA)              
 *      A5 (SCL)     
 */
#define INT_REQ_PIN A0
#define DMA_OUT_PIN A1
#define SS_MODE_PIN A2
#define STEP_PIN    A3

//States for Pixie Video
#define VIDEO_OFF   0
#define VIDEO_BEGIN 1
#define VIDEO_DMA   2
#define VIDEO_END   3

//Video state cycles
#define VIDEO_STATE_BEGIN  0
#define VIDEO_INT_BEGIN   29
//cycle 30 Interrupt read
//cycle 31 Interrupt response - 29 cycles to DMA

//cycle 56 End interrupt and EF1 signals - 4 cycles to DMA
#define VIDEO_SIGNALS_END 56
#define VIDEO_DMA_REQ     58
//End the video setup - next cycle is the first DMA cycle
#define VIDEO_SETUP_END   60


/*
 * Old TV's used to blank the screen when returning to the top of
 * the video display.  Some programs rely on this extra time at 
 * the end of each video frame. We need to wait at least this many 
 * cycles before starting the next video display cycle. The original
 * pixie video end of frame had time 941 2-cycle instructions  
 */
//End of Frame buffer cycles
#define END_BUFFER_CYCLES 1882

//Number of DMA Cycles in a single line
#define LINE_DMA_COUNT  8

//Total number of cycles per line
#define MAX_CYCLES_LINE 14

#define LINE_DMA_OFF     6
#define LINE_DMA_ON      12

//Video buffer for 32 x 64 resolution
#define VIDEO_BUFFER_SIZE 256

//end of frame signaled by /EF1 for last 4 lines
#define LINE_SIGNAL_END   123

//Last line in frame 
#define LAST_IN_FRAME 127

//Number of bytes in a display row (64 bits)
#define BYTES_PER_ROW 8

//Number of Rows in a Video display
#define DISPLAY_ROWS 32

//Bytes in a single raster line
#define RASTER_DATA_SIZE  16

//Up to 128 lines in a video
byte dma_line = 0;

//flag to capture video data
boolean capture = false;

byte state = VIDEO_OFF;

//counter for Video state machine
unsigned int cycles = 0;

byte video_data[VIDEO_BUFFER_SIZE];

//Debug variable for previous byte
byte previous = 0x00;

//flag to turn video on or off
boolean pixie_video = false;

//flag to indicate current state of video
boolean video_on = false;

//flag to print information after dma finished
boolean print_buffer = false;

//flag to reset counter rather than increment
boolean reset_counter = false;

//flag to redraw display if data has changed
boolean redraw = false;


/* 
 * U8g2 constructor for the video display
 * 
 * The Complete list of u8g2 constructors is available here:
 * https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
 */ 

//Generic SSD1306 I2C 128x64 OLED display
//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//Generic SH1106 I2C  128x64 OLED  display
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

//Lookup table for XBM format bitmap conversion
// In XBM format LSB is the left most bit, 
// so we expand each nibble to a byte and flip bits
const uint8_t lookup_xbm[16] PROGMEM {
  0x00, 0xC0, 0x30, 0xF0, 0x0C, 0xCC, 0x3C, 0xFC,
  0x03, 0xC3, 0x33, 0xF3, 0x0F, 0xCF, 0x3F, 0xFF};

#if DEBUG_TIMING
  //Time stamps for measuring time
  unsigned long t_start_video = 0L;
  unsigned long t_stop_video = 0L;
#endif 

//Set up video display
//Must be called in setup routine
void setupVideo() {
  //u8g2.setBusClock(400000); // fast I2C mode
  u8g2.begin();
  u8g2.setBitmapMode(false); //non-transparent
  u8g2.setDrawColor(1); // White
} // setupVideo  


// Paint entire display
void paintDisplay() {
  u8g2.firstPage();  
  do {
    drawDisplay();
  } while( u8g2.nextPage() );
} // paintDisplay

// Clear video buffer and display.
void blankDisplay() {
  clearVideoData();  
  paintDisplay();
} // blankVideo

//Draw the entire display
void drawDisplay() {
  for(int row = 0; row < DISPLAY_ROWS; row++) {
    byte raster_data[16];
 
    //Fill line with expanded data
    for (int i = 0; i < BYTES_PER_ROW; i++) {
      //Get a byte from the video data and transform it for display    
      byte b = video_data[row * BYTES_PER_ROW + i];
      byte hi = (b >> 4) & 0x0F;
      byte lo = b & 0x0F;
  
      // For XBM LSB is the left most, so we expand to byte and flip bits
      raster_data[2*i]   = pgm_read_byte_near(lookup_xbm + hi);
      raster_data[2*i+1] = pgm_read_byte_near(lookup_xbm + lo);  
    } // for i
  
    //Draw line twice for 32 x 64 resolution
    u8g2.drawXBM(0, 2*row, 128, 1, raster_data);
    u8g2.drawXBM(0, 2*row+1, 128, 1, raster_data);
  
  } // for row < DISPLAY_ROWS
}  //drawDisplay

boolean isInterrupt() {
  byte data_d = (PIND & D_STATUS_MASK);
  boolean interrupt = ( data_d == INT_STATUS_CODE);
   #if DEBUG
        Serial.print("IRQ Status code: 0x");
        print2hex(data_d);
        Serial.println();
  #endif   
  return interrupt;
}

//check to see if dma request is acknowledged
boolean isDmaAck() {
  byte data_d = (PIND & D_STATUS_MASK);
  boolean dma_status = (data_d == DMA_STATUS_CODE);
   #if DEBUG
        Serial.print("DMA Status code: 0x");
        print2hex(data_d);
        Serial.println();
  #endif    
  return dma_status;
}

//set the /EF1 flag for Pixie Video
void setExternalFlag(boolean state) {
  if (state) {
    //Negative logic: On is low, off is high
    digitalWrite(EF1_PIN, LOW);
  } else {
    //Negative logic: On is low, off is high
    digitalWrite(EF1_PIN, HIGH);
  } //if state
} //setExternalFlag()

/*
 * Video state machine
 * 
 * Note that single step freezes at the very end of each cycle.
 * Data is held valid for reading, but any changes made in this 
 * cycle will be read in the _next_ cycle.  Then acted upon in
 * the following cycle.  
 * 
 * This is why DMA signals are changed two cycles before. 
 * State codes indicate the next cycle.  So for example. a DMA 
 * acknowledge state code, indicates the next cycle is a DMA cycle.
 */
byte doVideoState() {
  //get ready for next step
  singleStep(false);

  switch (state) {        
/*
 * Video Begin State
 * 
 * The timing for cycles in this state are critical.  EF1 must be asserted for
 * at least 56 cycles (4 lines of 14 cycles).  The Interrupt request must be
 * raised and repsoned to with exactly 29 cycles before the first DMA cycle.  
 * 
 * Cycle 0(A)
 * 1      2     3     4     5     6     7   8   9  10  11  12  13  14
 * 15     16    17    18    19   20    21  22  23  24  25  26  27  28
 * 29(B)  30(C) 31(D) 32    33   34    35  36  37  38  39  40  41  42
 * 43     44    45    46    47   48    49  50  51  52  53  54  55  56(E)  
 * 57     58(F) 59(G) 60(H) 0(I)
 * 
 * (A) Cycle 0  - /EF1 is on
 * (B) Cycle 29 - Interrupt Begin (/INT is on)
 * (C) Cycle 30 - Interrupt read by 1802
 * (D) Cycle 31 - Interrupt response by 1802 - 29 cycles before first DMA
 * (E) Cycle 56 - Signals End (/EF1 off, /INT off) - 4 cycles before first DMA
 * (F) Cycle 58 - DMA request - /DMA_OUT is on
 * (G) Cycle 59 - DMA read by 1802 (DMA occurs after instruction completes)
 * (H) Cycle 60 - Setup Ends - DMA Acknowledged, counter reset, state = VIDEO_DMA
 * (I) Cycle 0, State VIDEO_DMA - First DMA cycle occurs
 */    
    //Start of video frame
    case VIDEO_BEGIN:   
      if (cycles == VIDEO_STATE_BEGIN) {
        #if DEBUG
          Serial.print("Asserting /EF1: ");        
          Serial.println(cycles);
        #endif  
        //raise EF1 4 lines before DMA
        setExternalFlag(true);
        #if DEBUG_TIMING
          t_start_video = millis();
        #endif
      } else if (cycles == VIDEO_INT_BEGIN) {
        //raise interrupt two lines before DMA
        #if DEBUG
          Serial.print("Requesting Interrupt: ");
          Serial.println(cycles);
        #endif          
        requestInterrupt(true);
      } else if (cycles == VIDEO_SIGNALS_END) {        
        #if DEBUG
          Serial.print("Ending Signals: ");
          Serial.println(cycles);
        #endif    
        setExternalFlag(false);
        requestInterrupt(false);        
      } else if (cycles == VIDEO_DMA_REQ) {
        //raise DMA request two cycles before
        #if DEBUG
          Serial.print("Requesting DMA: ");
          Serial.println(cycles);
        #endif            
        requestDma(true);
      } else if (cycles >= VIDEO_SETUP_END) {
        //wait for dma request to be acknowledged
        #if DEBUG
            Serial.print("Video Setup End: ");
            Serial.println(cycles);
          #endif  
        if (isDmaAck()) {
          //Next cycle will be a dma cycle so set state
          #if DEBUG
            Serial.print("DMA Acknowledged: ");
            Serial.println(cycles);
          #endif          
          state = VIDEO_DMA;
          reset_counter = true;
          dma_line = 0;
          capture = false;
          } //if isDmaAck
      } else {          
          #if DEBUG
            if (isInterrupt()) {
              Serial.print(F("Interrupt: "));
              Serial.println(cycles);
            } // if isInterrupt
          #endif
      } // if-else cycles
    break;
/*
 * Video DMA State
 * 
 * The timing here is also critical.  The logic reads one byte per cycle
 * via DMA requests to the 1802.  Eight bytes are read via DMA per line, 
 * followed by 6 instructions cycles before the start of the next line. 
 * 
 * Only data from the last line of each group of four is captured as video 
 * data for the OLED display.  But DMA is asserted for eight cycles on every
 * line so that the timing is maintained.  The cycle counter is reset for
 * each line, for 128 lines. The external flag /EF1 is on for the last four
 * lines of the video.
 * 
 * Cycle:
 * 0(A) 1   2   3   4   5   6(B)  7(C)  8(D)  9   10   11   12(E)  13(F)
 * 
 * (A) Cycles 0 to 7 - Eight DMA cycles where one byte is read from the 1802.
 * Video data is captured every fourth line on lines 3, 7, 11,... 119, 123, 127
 * (B) Cycle 6 - DMA request removed
 * (C) Cycle 7 - Last DMA cycle, next 6 cycles are instruction cycles.
 * (D) Cycles 8 to 13 - Six instruction cycles (for three 2-cycle instructions)
 * (E) Cycle 12 - DMA request raised for each line except for line 127.
 * (F) Cycle 13 - End of Line, DMA acknowledged, line counter incremented, cycle
 * counter reset to zero for next line. 
 * 
 * During lines 124 to 127, /EF1 is on.  At the end of line 127, reset cycle 
 * counter and set the state to VIDEO_END 
 * 
 */
    case VIDEO_DMA:   
      //Record first 8 cycles after DMA request
      if (cycles < LINE_DMA_COUNT) {
        //Data bytes for PORT B and PORT D
        byte data_d = PIND;
        byte data_b = PINB;
        if (capture) {          
          byte raster = dma_line / 4;  //integer divide
          byte offset = BYTES_PER_ROW * raster + cycles;           
          byte new_byte = (data_d & D_DATA_MASK) | (data_b & B_DATA_MASK);    
          byte old_byte = video_data[offset];
          //If any byte has changed save it and set flag to update display
          if (new_byte != old_byte) {
            redraw = true;
            video_data[offset] = new_byte;
          } // if new_byte != old_byte                    
        } //if cycles < LINE_DMA_COUNT 
                
        //on the next to last dma cycle raise /DMA_OUT line high
        if (cycles == LINE_DMA_OFF) {
          //remove DMA request
          requestDma(false);        
        } //if LINE_DMA_OFF
      //After 8 DMA cycles, there are 6 instruction cycles before next line
      } else if (cycles == LINE_DMA_ON) {         
        //assert next dma request on all lines except last one
        if (dma_line < LAST_IN_FRAME) {
          requestDma(true);
        } //if dma_line < LAST_IN_FRAME
      //At the end of the Line the DMA should be acknowledged for the next line
      } else if (cycles > LINE_DMA_ON) {
        /* 
         * At end of each line, check that dma acknowledged for next line.
         * Except there is no dma ack at end of last line, because it's 
         * finished reading all the video data required for the display.
         * If DMA isn't acknowledged either this is the end of the last line 
         * in the frame or otherwise just wait until it is acknowledged.
         */        
        if (isDmaAck()) {
          dma_line++;
          /* 
           * Capture data on last line of each block of four lines. For a
           * 32x64 display resolution, video data is usually repeated four times.
           * So only the last line would actually appear on the display.
           */
          capture = (dma_line % 4 == 3);
          reset_counter = true;          
          if (dma_line > LINE_SIGNAL_END) {
            //set /EF1 low during last 4 lines 
            setExternalFlag(true);
          } //if dma_line > LINE_SIGNAL_END                
        } else if (dma_line >= LAST_IN_FRAME) {            
            //After the end of last line, end the video frame              
            state = VIDEO_END;
            reset_counter = true; 
            setExternalFlag(false);       
        } //if isDmaAck else if LAST_IN_FRAME
      }//if-else cycles
    break;
    //Update the display if needed, then wait and restart
    case VIDEO_END:
      //Make sure to keep /EF1 flag off
      setExternalFlag(false); 

      #if DEBUG
        if (redraw) {
          Serial.println(F("DMA data received: "));
          for (int i = 0; i < VIDEO_BUFFER_SIZE; i++) {
            if (i % BYTES_PER_ROW == 0) {
              Serial.println();
            } //if i % BYTES_PER_ROW
            //print hex values so they all line up
            print2hex(video_data[i]);   
            Serial.print(" ");
          } //for 
          Serial.println();  
        } // if redraw
      #endif
      
      //Check to see if we need to redraw the display
      if (redraw) {                      
        u8g2.clear();
        redraw = false;                 
        //If not using tiles, paint the entire display 
        paintDisplay();          
      } //if redraw  
      
    #if DEBUG_TIMING
      //set timestamp
      t_stop_video = millis();
      Serial.print(F("Time for video frame: "));
      Serial.print(t_stop_video - t_start_video);
      Serial.println(F(" mSec"));
    #endif
       
    //Wait awhile before starting a new video cycle, some programs depend on this
    if (cycles >= END_BUFFER_CYCLES) {
      state = VIDEO_BEGIN;
      reset_counter = true;
    } //if
    break;
    
    //VIDEO_OFF and any unknown state 
    default:
      //do nothing
    break;
  } //switch
  //Advance one step
  nextStep();
  return state;
} // doVideoState()

//Clear the dma data buffer
void clearVideoData() {
  for (int i = 0; i < VIDEO_BUFFER_SIZE; i++) {
    video_data[i] = 0x00;
  } //for
} //clearVideoData

//Check the status of the video bit
boolean isVideoActive() {
  byte data_b = PINB;
  //when video bit goes high, pixie video is active
  boolean pixie_video = data_b & VIDEO_ON_BIT;
  return pixie_video; 
} // isVideoActive

//Request interrupt from 1802
void requestInterrupt(boolean intReq) {
  //Negative logic: /INT is true when low, false when high
  if (intReq) {
    digitalWrite(INT_REQ_PIN, LOW);  
  } else {
    digitalWrite(INT_REQ_PIN, HIGH);  
  } //if intReq
}//requestInterrupt

//Request DMA OUtput from 1802
void requestDma(boolean dmaReq) {
  //Negative logic: /DMA_OUT is true when low, false when high
  if (dmaReq) {
    digitalWrite(DMA_OUT_PIN, LOW);  
  } else {
    digitalWrite(DMA_OUT_PIN, HIGH);  
  } //if dmaReq
} //requestDma

//Put cpu into Single Step mode
void setSingleStepMode(boolean ss_mode) {
  //Negative logic: SS Mode is activated when low, off when high
  if (ss_mode) {
    //Set step pin Low to be ready for first step
    digitalWrite(STEP_PIN, LOW);   
    //Set /SS enable low, excution will stop at next cycle 
    digitalWrite(SS_MODE_PIN, LOW);
  } else {
    //Set //SS enable high
    digitalWrite(SS_MODE_PIN, HIGH);
    //Set step pin High to propagate change
    digitalWrite(STEP_PIN, HIGH);
  } //if ss_mode
} //setSingleStepMode

//Single Step cpu one cycle
void singleStep(boolean s_step) {
  //Single step triggers on Low to HIGH transition
  if (s_step) {
    digitalWrite(STEP_PIN, HIGH);
  } else {
    digitalWrite(STEP_PIN, LOW);
  } //if s_step
} // singleStep

#if DEBUG
  //Print video off / on status
  void printVideoState () {
    Serial.print(F("Pixie Video is "));
    Serial.println(video_on ? F("on.") : F("off."));
  } //printVideoState
  
// Pretty print two hex digits for a byte value
void print2hex(byte b) {  
  //If single Hex digit
  if (b < 0x10) {
   Serial.print(F("0"));
  } // if b < 0x10
  Serial.print(b, HEX);
}  
#endif

//Advance cpu one step and update counter
void nextStep() {
  singleStep(true);
  
  if (reset_counter) {
    cycles = 0;
    reset_counter = false;
  } else {
    cycles++;
  } // if-else reset_counter
} //


//set parameters to start video
boolean startVideo() {
  video_on = true;
  //set initial cycle count to zero
  cycles = 0;
  //Always draw display at least once
  redraw = true;  
  //start state machine  
  state = VIDEO_BEGIN;  
  setSingleStepMode(true);
} //startVideo

//set parameters to stop video
void stopVideo() {
  video_on = false;
  //Reset all video control pins
  resetVideoControls();
  //set initial cycle count to zero
  cycles = 0;
  //clear display and data
  u8g2.clear();
  clearVideoData();
  //release 1802
  setSingleStepMode(false);
  //reset redraw flag
  redraw = false;
  //reset video state
  state = VIDEO_OFF;
}

//Set all control output pins back to logical off states
void resetVideoControls() {
  //Turn off int, dma and ef1 control signals
  //Negative Logic: off when bit is high
  digitalWrite(INT_REQ_PIN, HIGH);
  digitalWrite(DMA_OUT_PIN, HIGH);
  digitalWrite(EF1_PIN, HIGH);    
}

//One time start logic goes here  
void setup() {
  //Set up analog pins as digital outputs
  pinMode(SS_MODE_PIN, OUTPUT);
  //Set initial state of digital output high (false)
  digitalWrite(SS_MODE_PIN, HIGH);
    
  pinMode(STEP_PIN, OUTPUT);
  //Set initial state of digital output low
  digitalWrite(STEP_PIN, LOW);

  pinMode(INT_REQ_PIN, OUTPUT);
  //Set initial state of digital output high (false)
  digitalWrite(INT_REQ_PIN, HIGH);
  
  pinMode(DMA_OUT_PIN, OUTPUT);
  //Set initial state of digital output high (false)
  digitalWrite(DMA_OUT_PIN, HIGH);

  
  //Set up the inputs on port D (Bits 0, 1 are reserved)
  //DDRD is the port D digital direction register (0 = Input)
  DDRD &= PORTD_INPUT;

  //Set up the inputs on port B (Bits 6, 7 are reserved, bit 5 is output only)
  //DDRB is the port B digital direction register (0 = Input)
  DDRB &= PORTB_INPUT;  

  //Set pin 13 as output, even though already set as output
  pinMode(EF1_PIN, OUTPUT);
  //Set initial state of digital output high (false)
  digitalWrite(EF1_PIN, HIGH);


  #if DEBUG || DEBUG_TIMING
    Serial.begin(115200);
    Serial.println("Ready");
  #endif
  //Setup video display
  setupVideo();
  //Set up initial state as off
  state = VIDEO_OFF;
} //setup

/*
 * This code simulates a cdp1861 Pixi video chip to load a video
 * ram buffer then update a 128 x 64 OLED display.  The EF1 flag, 
 * interrupts and dma lines are all manipulated as in a real cdp1861 
 * chip running in 32 x 64 resolution.
 */
void loop() {
  if (video_on) {
    singleStep(false);
    //check to see if pixie video is no longer active
    if (!isVideoActive()) {        
      stopVideo();
      #if DEBUG  
        printVideoState();
      #endif
    } //if !isVideoActive    
    //video state machine
    doVideoState();
  } else {   
    //turn video on when N0 goes high
    if (isVideoActive()) {            
      startVideo();
      #if DEBUG
        printVideoState();
      #endif             
    }//if pixie_video
  } //if-else video_on  
} //loop
