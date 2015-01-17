/*
 * "ChronomeFirmware" - Arduino Based RGB Pressure Sensitive Monome Clone by Owen Vallis 09/23/2010
 *
 * --------------------------------------------------------------------------
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * --------------------------------------------------------------------------
 *
 * Parts of this code is based on Matthew T. Pandina's excellent TLC5940 C Library, with pins updated to work with the
 * Arduino MEGA. For those portions, he asked that his copyright be added to the code.
 *
 * Copyright 2010 Matthew T. Pandina. All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, are permitted 
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of 
 *   conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of 
 *   conditions and the following disclaimer in the documentation and/or other materials 
 *   provided with the distribution.
 * 
 *
 * Thanks to Brad Hill, Martijn Zwartjes, Jordan Hochenbaum, Johnny McClymont, Tim Exley, and Jason Edwards 
 * for answering my questions along the way.
 * 
 * Please DO NOT email monome with technical questions and/or help regarding this code or clone.  
 * They are in NO WAY responsible or affiliated with this project other than they were our inspiration 
 * and we used many of their methods and pulled from their code.
 * 
 * Additionally, while we are availble and willing to help as much as possible, we too CANNOT be held
 * responsible for anything you do with this code.  Please feel free to report any bugs, suggestions 
 * or improvements to us as they are all welcome.  Again, we cannot be held responsible for any damages 
 * or harm caused by the use or misuse of this code or our instructions.  Thank you for understanding.
 * --------------------------------------------------------------------------
 *
 * Links:
 * http://www.flipmu.com - Our website - Click "Chronome Project" on the Navigation Menu under Work.
 * www.monome.org - the "original" monome and our inspiration
 *
 */
/////////////////////////////////////////////////////////////////////
//            Set-up for Ultrasonic Rangefinders
/////////////////////////////////////////////////////////////////////

#include <NewPing.h>

/////////////////////////////////////////////////////////////////////
//            Set-up for the 10-dof 
/////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

/////////////////////////////////////////////////////////////////////
//            Normal Chronome Activity
/////////////////////////////////////////////////////////////////////
// supports uint8_t and uint16_t
#include <stdint.h>
// Definition of interrupt names
#include <avr/interrupt.h>
// ISR interrupt service routine
#include <avr/io.h>

/////////////////////////////////////////////////////////////////////
//            Set-up for the 10-dof 
/////////////////////////////////////////////////////////////////////

#define SONAR_NUM     5 // Number or sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

/////////////////////////////////////////////////////////////////////
//            Normal Chronome Activity
/////////////////////////////////////////////////////////////////////

//************************* TLC5940 pin definitions  **********************
// MEGA PWM PIN 11
#define GSCLK 11
#define GSCLK_DDR DDRB
#define GSCLK_PORT PORTB
#define GSCLK_PIN PB5
// MEGA MOSI PIN 51
#define SIN 51
#define SIN_DDR DDRB
#define SIN_PORT PORTB
#define SIN_PIN PB2
// MEGA SCK PIN 52
#define SCLK 52
#define SCLK_DDR DDRB
#define SCLK_PORT PORTB
#define SCLK_PIN PB1
// MEGA PIN 41 
#define BLANK 41
#define BLANK_DDR DDRG
#define BLANK_PORT PORTG
#define BLANK_PIN PG0
// MEGA PIN 40
#define XLAT 40
#define XLAT_DDR DDRG
#define XLAT_PORT PORTG
#define XLAT_PIN PC1
// MEGA PIN 39
#define VPRG 39
#define VPRG_DDR DDRG
#define VPRG_PORT PORTG
#define VPRG_PIN PG2
// MEGA PIN 22
#define REDTR 22
// MEGA PIN 23
#define GREENTR 23
// MEGA PIN 24
#define BLUETR 24

// MEGA PINS 49-42 ROWS are on PORTL 
#define ROWS PORTL 

// Additional SPI PIN defs (Not used but set)
// MEGA MISO PIN 50
#define DATAIN 50 
// MEGA SS PIN 53
#define SLAVESELECT 53  


//************************* Variables *************************************
//************************* Macros  ***************************************
#define TLC5940_N 4
#define numColors (uint8_t)3

#define setLow(port, pin) ((port) &= ~(1 << (pin)))
#define setHigh(port, pin) ((port) |= (1 << (pin)))

#if (16 * TLC5940_N > 255)
#define channel_t uint16_t
#else
#define channel_t uint8_t
#endif
#define numChannels ((channel_t)16 * TLC5940_N)

#if (24 * TLC5940_N > 255)
#define gsData_t uint16_t
#else
#define gsData_t uint8_t
#endif

#define gsDataSize ((gsData_t)24 * TLC5940_N)
#define numChannels ((channel_t)16 * TLC5940_N)

/////////////////////////////////////////////////////////////////////
//            Set-up for the 10-dof 
/////////////////////////////////////////////////////////////////////
/* Assign a unique ID to the sensors */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);

/////////////////////////////////////////////////////////////////////
//            Normal Chronome Activity
/////////////////////////////////////////////////////////////////////

uint8_t gsData[numColors][gsDataSize];
uint8_t gsStateData[numColors][gsDataSize];
uint16_t previousButtonValue[8][8];

boolean led13;
long time = 1000;
long dofPingTime = 2500;  

/////////////////////////////////////////////////////////////////////
//            Set-up for Ultrasonic Rangefinders
/////////////////////////////////////////////////////////////////////

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.

/////////////////////////////////////////////////////////////////////
//            Set-up for the 10-dof 
/////////////////////////////////////////////////////////////////////

//************************* Serial Functions setup *************
uint8_t tolerance = 7;

void sendSerial(uint8_t Data)
{
  while (!(UCSR0A & (1 << UDRE0)));

  UDR0 = Data;
}

//************************* Serail Functions From the Octinct *************

//Debugging definitions: uncomment the relevant line to turn it on
#define REDALERT 100 //Draw colour is forced to red if the serial receive buffer has more than the specified number of characters in it

/* Size of the serial buffer before the Tinct is forced to parse it continually.
 The buffer size is 128 bytes, and if it gets there the Tinct can (and will) crash.
 The largest command size is 9 bytes, so 119 is an absolute maximum value.
 Set it lower than this to be safe.
 
 If the Tinct hits this limit, it will start to flicker, and might miss commands,
 but it won't crash. Probably.
 */
#define TOOFULL 100

// Variables for interpreting the serial commands
uint8_t address, state, x, y, pos;
uint16_t r, g, b;
uint8_t ready = true;

// For interrupt timing; needed only to do intermediate clock speeds
/* Divide interrupt frequency by a factor of FREQ. It is preferable to keep
 FREQ as small as possible, and control the frequency of the interrupts
 using the hardware clock. Setting it to 1 disables this entirely, which,
 if it works, is ideal; this should be the same as commenting out the
 "#define FREQ" statement entirely.
 */
#define FREQ 1 // How many interrupts occur before the serial commands are read
#if FREQ > 1
byte int_counter = 0;
#endif

//The timer interrupt routine, which periodically interprets the serial commands
ISR(TIMER2_OVF_vect) {
  sei(); //Reenable global interrupts, otherwise serial commands will get dropped


#if FREQ > 1
  if(++int_counter == FREQ){ // Only do this once every FREQ-th interrupt
    int_counter = 0;
#endif //FREQ
    do{ // This do ensures that the data is always parsed at least once per cycle
      if(Serial.available()){
#ifdef REDALERT // if REDALERT is defined, draw colour turns red when the buffer is getting dangerously full
        if(Serial.available() > REDALERT){
          for(int x = 0; x < 64; x++)
          {
            TLC5940_SetGS(x, 4095, 0);
            TLC5940_SetGS(x, 0, 1);
            TLC5940_SetGS(x, 0, 2);           
          }
        }
#endif //REDALERT
        if(ready){ // If the last command has finished executing, read in the next command and reset the command flag
          address = Serial.read();
          ready = false;
        }

        // if the MSB doesn't equal 1, then we are missing our address message. Trash byte and read again.
        if((address & 0x80) != 0x80){
          ready=true;
          break;
        }

        switch (address & 0xf) { //Execute the appropriate command, but only if we have received enough bytes to complete it. We might one day add "partial completion" for long command strings.
        case 2: // rgb_led_on
          if( Serial.available()) {
            int byte1 = Serial.read();
            x = byte1 >> 4;
            y = byte1 & 0xf;
            pos = (x)+(y*8);

            TLC5940_SetGSState(pos, true);
            ready=true;
          }
          break;
        case 3: // rgb_led_off
          if( Serial.available()) {
            int byte1 = Serial.read();
            x = byte1 >> 4;
            y = byte1 & 0xf;
            pos = (x)+(y*8);

            TLC5940_SetGSState(pos, false);
            ready=true;
          }
          break;
        case 4: // rgb_led_color
          if( Serial.available() > 3 ) {
            uint8_t pos = Serial.read();
            x = (pos >> 4);
            y = (pos & 0x0F);
            pos = (x)+(y*8);
            r = (uint16_t)(Serial.read() * 32);
            if(r > y * 35) {
              r = r - (y * 35);
            }
            g = (uint16_t)(Serial.read() * 32);
            if(g > y * 35) {
              g = g - (y * 35);
            }
            b = (uint16_t)(Serial.read() * 32);
            if(b > y * 35) {
              b = b - (y * 35);
            }
            TLC5940_SetGS(pos, r, 0);
            TLC5940_SetGS(pos, g, 1);
            TLC5940_SetGS(pos, b, 2);
            ready=true;
          }
          break;
        case 5: // rgb_led_all_on
          {
            boolean state = (address >> 4) & 0x01;
            for (int pos = 0; pos < 64; pos++) {               
              TLC5940_SetGSState(pos, state);
            }
            ready = true;
          }
          break;
        case 6: // rgb_led_row
          {
            if( Serial.available()) {
              uint8_t ledRow = (address >> 4) & 0x07;
              uint8_t rowState = Serial.read();

              for (uint8_t col = 0; col < 8; col++) {   
                uint8_t state = (rowState >> col) & 0x01;            
                TLC5940_SetGSState((ledRow * 8) + col, state);
              }
              ready = true;
            }
          }
          break;
        case 7: // rgb_led_col
          {
            if( Serial.available()) {
              uint8_t ledCol = (address >> 4) & 0x07;
              uint8_t colState = Serial.read();

              for (uint8_t row = 0; row < 8; row++) {   
                uint8_t state = (colState >> row) & 0x01;            
                TLC5940_SetGSState(ledCol + (row * 8), state);
              }
              ready = true;
            }
          }
          break;
        default:
          break;
        }
      }
    }
    // If the serial buffer is getting too close to full, keep executing the parsing until it falls below a given level
    // This might cause flicker, or even dropped messages, but it should prevent a crash.
    while (Serial.available() > TOOFULL);
#if FREQ > 1
  }
#endif //FREQ
}


//************************* RGB Function **********************************
// set all GrayScale Color
void TLC5940_SetAllGS(uint16_t value) {
  uint8_t tmp1 = (value >> 4);
  uint8_t tmp2 = (uint8_t)(value << 4) | (tmp1 >> 4);
  for (uint8_t i = 0; i < numColors; i++){
    gsData_t j = 0;
    do {
      gsData[i][j++] = tmp1; // bits: 11 10 09 08 07 06 05 04
      gsData[i][j++] = tmp2; // bits: 03 02 01 00 11 10 09 08
      gsData[i][j++] = (uint8_t)value; // bits: 07 06 05 04 03 02 01 00
    } 
    while (j < gsDataSize);
  }
}

// set a single GrayScale Color
void TLC5940_SetGS(channel_t channel, uint16_t value, uint8_t color) {
  channel = numChannels - 1 - channel;
  uint16_t i = (uint16_t)channel * 3 / 2;
  switch (channel % 2) {
  case 0:
    gsData[color][i++] = (value >> 4);
    gsData[color][i++] = (gsData[color][i] & 0x0F) | (uint8_t)(value << 4);
    break;
  default: // case 1:
    gsData[color][i++] = (gsData[color][i] & 0xF0) | (value >> 8);
    gsData[color][i++] = (uint8_t)value;
    break;
  }
}

// turn on or off an LED
void TLC5940_SetGSState(channel_t channel, boolean state) {
  channel = numChannels - 1 - channel;
  for (uint8_t n = 0; n < numColors; n++){
    uint16_t i = (uint16_t)channel * 3 / 2;
    switch (channel % 2) {
    case 0:
      gsStateData[n][i++] = gsData[n][i] * state;
      gsStateData[n][i++] = (gsStateData[n][i] & 0x0F) | ((gsData[n][i] & 0xF0) * state);
      break;
    default: // case 1:
      gsStateData[n][i++] = (gsStateData[n][i] & 0xF0) | ((gsData[n][i] & 0x0F) * state);
      gsStateData[n][i++] = gsData[n][i] * state;
      break;
    }
  }
}

// ISR for clocking in the next Color's GSData. 
ISR(TIMER3_COMPA_vect) { 
  static uint8_t color = 0;

  PORTA = 0x07;

  setHigh(BLANK_PORT, BLANK_PIN);
  setHigh(XLAT_PORT, XLAT_PIN);
  setLow(XLAT_PORT, XLAT_PIN);
  setLow(BLANK_PORT, BLANK_PIN);

  PORTA &= ~(1 << color);

  // Below this we have 4096 cycles to shift in the data for the next cycle
  for (gsData_t i = 0; i < gsDataSize; i++) {

    SPDR = gsStateData[color][i];
    while (!(SPSR & (1 << SPIF)));
  }

  color = (color + 1) % numColors;
}
//************************* Ultrasonic Setup ******************************

NewPing sonar[SONAR_NUM] = {     // Sensor object array.
  NewPing(1, 0, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(3, 2, MAX_DISTANCE),
  NewPing(5, 4, MAX_DISTANCE),
  NewPing(9, 8, MAX_DISTANCE),
  NewPing(11, 10, MAX_DISTANCE)
  };

  //************************* Button Functions ******************************
  void readADC() {

    for( uint8_t row = 0; row < 8; row++){

      // incrment and set row high
      ROWS = (1 << row); 

      // let the board settle after we shift a row
      delayMicroseconds(100); 

      // check each column's value  
      for( uint8_t col = 0; col < 8; col++)
      {

        uint16_t currentButtonValue = analogRead(col);

        // if we have changed then send it out
        if(abs(previousButtonValue[row][col] - currentButtonValue) > tolerance
          || (previousButtonValue[row][col] != 0 && currentButtonValue == 0))
        {
          // This is to avoid the noise near zero
          if(currentButtonValue > 10 || currentButtonValue == 0) {   
            sendSerial(0x10 | ((col) & 0x0F));
            sendSerial((row << 4) | (uint8_t)(currentButtonValue >> 8));
            sendSerial((uint8_t)currentButtonValue);  
          }
        }

        // store current value
        previousButtonValue[row][col] = currentButtonValue;   

        delayMicroseconds(10);    
      }
    }
  }

//************************* Arduino Loops *********************************
// Setup Device
void setup(){
  Serial.begin(115200);//this runs at 57600 for normal chronome activity 
  /////////////////////////////////////////////////////////////////////
  //            Set-up for Ultrasonic Rangefinders
  /////////////////////////////////////////////////////////////////////

  pingTimer[0] = millis() + 75;           // First ping starts at 75ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++){ // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;
  }
  /////////////////////////////////////////////////////////////////////
  //                   Test all sensors on 10-dof
  /////////////////////////////////////////////////////////////////////

  Serial.println(F("Adafruit 10DOF Tester")); 
  Serial.println(""); 
  /* Initialise the sensors */
  if(!accel.begin())
  {
    Serial.println(F("No LSM303 detected ... Check your wiring!"));    
  }
  if(!mag.begin())
  {
    Serial.println("No LSM303 detected ... Check your wiring!");
  }
  if(!bmp.begin())
  {
    Serial.print("No BMP085 detected ... Check your wiring or I2C ADDR!");
  }
  if(!gyro.begin())
  {
    Serial.print("No L3GD20 detected ... Check your wiring or I2C ADDR!");
  }
  /////////////////////////////////////////////////////////////////////
  //                  Normal Chronome Activity
  /////////////////////////////////////////////////////////////////////
  //************** SETUP PINS **************
  pinMode(GSCLK, OUTPUT);
  pinMode(SCLK, OUTPUT);
  pinMode(VPRG, OUTPUT);
  pinMode(XLAT, OUTPUT);
  pinMode(BLANK, OUTPUT);
  pinMode(SIN, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SLAVESELECT,OUTPUT);
  pinMode(REDTR,OUTPUT);
  pinMode(GREENTR,OUTPUT);
  pinMode(BLUETR,OUTPUT);
  pinMode(13, OUTPUT);

  for( int i = 0; i < 8; i++){
    pinMode(42+i, OUTPUT);
  }

  digitalWrite(SLAVESELECT,HIGH); //disable device
  setLow(GSCLK_PORT, GSCLK_PIN);
  setLow(SCLK_PORT, SCLK_PIN);
  setHigh(VPRG_PORT, VPRG_PIN);
  setLow(XLAT_PORT, XLAT_PIN);

  //************** SET ADC **************
  ROWS = (1 << 0);       // Set the first Chronome Row High

  //************** SET SPI **************
  // Enable SPI, Master, set clock rate fck/2
  SPCR = (1 << SPE) | (1 << MSTR);
  SPSR = (1 << SPI2X);

  //  Clear SPI data Registers
  byte clr;
  clr=SPSR;
  clr=SPDR;

  //************** SET TIMERS **************
  // Dont need to call sei(); because Arduino already does this
  // Clear TIMER1 Reg back to default
  TCCR1A = 0x00;
  TCCR1B = 0x00;
  // Enable timer 1 Compare Output channel A in toggle mode
  TCCR1A |= (1 << COM1A0);
  // Configure timer 1 for CTC mode
  TCCR1B |= (1 << WGM12);
  // Set up timer to fCPU (no Prescale) = 16Mhz/8 = 2Mhz
  // Set CTC compare value to pulse PIN at 2Mhz
  // (1 / Target Frequency) / (1 / Timer Clock Frequency) - 1
  TCCR1B |= (1 << CS11);
  // Full period of PIN 11 pulse requires 2 ticks (HIGH, LOW) 
  // So PIN 11 @ 2Mhz = (2 ticks (HIGH, LOW)) = 1Mhz
  OCR1A = 0;

  // Clear TIMER3 Reg back to default  
  TCCR3A = 0x00;
  TCCR3B = 0x00;
  // Configure timer 3 for CTC mode
  TCCR3B |= (1 << WGM32);
  // Set up timer to fCPU (no prescale) = 16Mhz/8 = 2Mhz 
  TCCR3B |= (1 << CS31);
  // Set CTC compare value to 4096 @ half TIMER1 frequency 
  // So (4096*2) @ 2Mhz = 4096 @ 1Mhz
  OCR3A = (4096*2) - 1;
  // Enable Timer/Counter3 Compare Match A interrupt
  TIMSK3 |= (1 << OCIE3A);

  // Setup the timer interrupt for Serial
  TCCR2A = 0;
  TCCR2B = 0<<CS22 | 1<<CS21 | 1<<CS20;

  //Timer2 Overflow Interrupt Enable
  TIMSK2 = 1<<TOIE2;

  //************** SET FirstCycle **************
  // Default all channels to all white
  TLC5940_SetAllGS(4095);
  // Default all LED states to off
  for(int pos = 0; pos < 64; pos++) {
    /* //used for setting a single default color other than white
     TLC5940_SetGS(pos, 2000, 0);
     TLC5940_SetGS(pos, 0, 1);
     TLC5940_SetGS(pos, 4095, 2);
     */
    TLC5940_SetGSState(pos, false);
  }

  PORTA = 0x07;

  setHigh(BLANK_PORT, BLANK_PIN);
  setLow(VPRG_PORT, VPRG_PIN);      
  setHigh(XLAT_PORT, XLAT_PIN);
  setLow(XLAT_PORT, XLAT_PIN);    
  setHigh(SCLK_PORT, SCLK_PIN);
  setLow(SCLK_PORT, SCLK_PIN);
  setLow(BLANK_PORT, BLANK_PIN);

  PORTA = 0x03;
}
void readDOF(){
  /* Get a new sensor event */
  sensors_event_t event;
  /* Display the results (acceleration is measured in m/s^2) */
  accel.getEvent(&event);
  /*
  Serial.print(F("ACCEL "));
   Serial.print("X: "); 
   Serial.print(event.acceleration.x); 
   Serial.print("  ");
   Serial.print("Y: "); 
   Serial.print(event.acceleration.y); 
   Serial.print("  ");
   Serial.print("Z: "); 
   Serial.print(event.acceleration.z); 
   Serial.print("  ");
   Serial.println("m/s^2 ");
   */
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  mag.getEvent(&event);
  /*
  Serial.print(F("MAG   "));
   Serial.print("X: "); 
   Serial.print(event.magnetic.x); 
   Serial.print("  ");
   Serial.print("Y: "); 
   Serial.print(event.magnetic.y); 
   Serial.print("  ");
   Serial.print("Z: "); 
   Serial.print(event.magnetic.z); 
   Serial.print("  ");
   Serial.println("uT");
   */
  /* Display the results (gyrocope values in rad/s) */
  gyro.getEvent(&event);
  /*
  Serial.print(F("GYRO  "));
   Serial.print("X: "); 
   Serial.print(event.gyro.x); 
   Serial.print("  ");
   Serial.print("Y: "); 
   Serial.print(event.gyro.y); 
   Serial.print("  ");
   Serial.print("Z: "); 
   Serial.print(event.gyro.z); 
   Serial.print("  ");
   Serial.println("rad/s ");  
   */
  /* Display the pressure sensor results (barometric pressure is measure in hPa) */
  /*
  bmp.getEvent(&event);
   if (event.pressure)
   {
  /* Display atmospheric pressure in hPa 
   
   Serial.print(F("PRESS "));
   Serial.print(event.pressure);
   Serial.print(F(" hPa, "));
  /* Display ambient temperature in C 
   float temperature;
   bmp.getTemperature(&temperature);
   Serial.print(temperature);
   Serial.print(F(" C, "));
  /* Then convert the atmospheric pressure, SLP and temp to altitude    
  /* Update this next line with the current SLP for better results      
   float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
   Serial.print(bmp.pressureToAltitude(seaLevelPressure,
   event.pressure,
   temperature)); 
   Serial.println(F(" m"));
   }
   
   Serial.println(F(""));
   */
}
/////////////////////////////////////////////////////////////////////
//            Functions for Ultrasonic Rangefinders
/////////////////////////////////////////////////////////////////////
void echoCheck() { // If ping received, set the sensor distance to array.
  if (sonar[currentSensor].check_timer())
    cm[currentSensor] = sonar[currentSensor].ping_result / US_ROUNDTRIP_CM;
}

void oneSensorCycle() { // Sensor ping cycle complete, do something with the results.
  for (uint8_t i = 0; i < SONAR_NUM; i++) {
    Serial.print(i);
    Serial.print("=");
    Serial.print(cm[i]);
    Serial.print("cm ");
  }
  Serial.println();
}
/////////////////////////////////////////////////////////////////////
//                            Main Loop
/////////////////////////////////////////////////////////////////////

void loop() {  
  readADC();//this is all this is needed in the loop for normal chronome functionality
  unsigned long currentTime = millis();
  if (currentTime - dofPingTime > time){
    readDOF();
    time = currentTime; 
  }
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through all the sensors.
    if (currentTime >= pingTimer[i]) {         // Is it this sensor's time to ping?
      pingTimer[i] += PING_INTERVAL * SONAR_NUM;  // Set next time this sensor will be pinged.
      if (i == 0 && currentSensor == SONAR_NUM - 1) oneSensorCycle(); // Sensor ping cycle complete, do something with the results.
      sonar[currentSensor].timer_stop();          // Make sure previous timer is canceled before starting a new ping (insurance).
      currentSensor = i;                          // Sensor being accessed.
      cm[currentSensor] = 0;                      // Make distance zero in case there's no ping echo for this sensor.
      sonar[currentSensor].ping_timer(echoCheck); // Do the ping (processing continues, interrupt will call echoCheck to look for echo).
    }
  }
  //read the buttons
}








