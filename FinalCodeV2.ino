//P2527970 ENGD2103 Assessment Submission
//Libs
#include <Arduino.h>
#include <Wire.h>

//Register State Arrays
//bool BRegisterState[8] = {0, 0, 0, 0, 0, 0, 0, 0,}; //Arrays to store state of Registers B, C, D
//bool CRegisterState[8] = {0, 0, 0, 0, 0, 0, 0, 0,}; 
//bool DRegisterState[8] = {0, 0, 0, 0, 0, 0, 0, 0,}; 

//Timing
unsigned long currentMillis; //Variable for time since program start ~50 days

//Heartbeat
bool heartbeatState; //store state of heartbeat for toggle
const int heartbeatInterval = 500; //interval for heartbeat. Swap between 500 and 1000 to confirm difference. 500 for final.
unsigned long heartbeatPreviousTime; //big number for last time of heartbeat state flip
#define P13_ON PORTB = PORTB | B00100000; //macro to internal led high. | writes bit 1
#define P13_OFF PORTB = PORTB & B11011111; //macro to internal led low. & writes bit 0

//MPU
const int MPU = 0x68; //mpu addr
int16_t AccX, AccY, AccZ; //Integers for acceleration. Signed and constant 16bits. 
//int16_t GyrX, GyrY, GyrZ;  //As above, but not sure if need rotation rates? Take readings, delete if not useful or to free memory.
int accelState; //store state of dominant accelerometer axis

//MPU Filtering
int orientation; //current orientation
int oldOrientation; //store last orientation
const int orientationInterval = 100; //interval to find old orientation (debouncing)
int orientationPreviousTime; //last orientation
const int domMult = 1.2; //multiplier for determining dominant axis

//SREG vars
byte sRegStatePre = B00000000; //holds state of sReg before heartbeat
byte sRegState; //sReg state to use after heartbeat
byte oldSRegState; //last state of sReg. use for scheduler if implemented

//7Seg Character macros
#define SEG_OFF B00000000   //value to send to shift reg is B00000000
#define SEG_0 B00111111     //64+16+8+4+2+1 B01011111
#define SEG_1 B00000110     //4+2 B00000110
#define SEG_2 B01011011     //64+16+8+2+1 B01011011
#define SEG_3 B01001111     //64+8+4+2+1 B01001111
#define SEG_4 B01100110     //64+32+4+2 B01100110
#define SEG_5 B01101101     //64+32+8+4+1 B01101101
#define SEG_6 B01111101     //64+32+16+8+4+1 B01111101
#define SEG_7 B00000111     //4+2+1 B00000111
#define SEG_8 B01111111     //64+32+16+8+4+2+1 B01111111
#define SEG_9 B01101111     //64+32+8+4+2+1 B01101111
#define SEG_A B01110111     //64+32+16+4+2+1 B01110111
#define SEG_b B01111100     //64+32+16+8+4 B01111100
#define SEG_C B00111001     //32+16+8+1 B00111001
#define SEG_d B01011110     //64+16+8+4+2 B01011110
#define SEG_E B01111001     //64+32+16+8+1 B01111001
#define SEG_F B01110001     //64+32+16+1 B01110001
#define SEG_DP B10000000    //128 B10000000

//SReg Macros
#define SREG_LATCH_HIGH PORTC = PORTC | B00001000 //bit mask for relevant pin high/low
#define SREG_LATCH_LOW  PORTC = PORTC & B11110111
#define SREG_CLOCK_HIGH PORTB = PORTB | B00010000
#define SREG_CLOCK_LOW  PORTB = PORTB & B11101111
#define SREG_DATA_HIGH  PORTB = PORTB | B00000001
#define SREG_DATA_LOW   PORTB = PORTB & B11111110

//Button variables
bool button1State = 0; //state for buttons and last states (edge detection)
bool oldButton1State = 0;
bool button2State = 0;
bool oldButton2State = 0;
bool button3State = 0;
bool oldButton3State = 0;
const int buttonDebounceInterval = 50; //interval delay for debouncing
unsigned long lastButtonDebounceTime1; //last time of debounces
unsigned long lastButtonDebounceTime2;
unsigned long lastButtonDebounceTime3;

//Game Variables
int currentCredits = 0; //store current credits


//Run at startup
void setup() {
  //Register setup
  DDRB = DDRB | B00111111; //BReg outputs. first 2 not available. rest output
  PORTB = PORTB | B00000000; //BReg states.
  DDRC = DDRC | B00111000;
  PORTC = PORTC | B00000111;
  //DDRD = DDRD | B11111100; 
  //PORTD = PORTD | B00000000;

  //Begin I2C
  Wire.begin(); //start i2c
  Wire.beginTransmission(MPU); //transmit to mpu address
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true); //leave bus open


  SREG_LATCH_LOW; //set latch for sreg low
  
  //Serial
  Serial.begin(9600); //begin Serial port. Disable for final.
  Serial.println("Serial begin");
}

//Main loop
void loop() {
  currentMillis = millis(); //update time since program start

  heartbeatMain(); //run main heartbeat loop (on P13) (and now dp!)

  MPUmeasure(); //run MPU reading

  buttonRead();

  creditCounter(); //before shift reg so we can do this in one loop

  shiftRegisterUpdate(); //run sReg update

  serialPrint(); //run serialPrint. disable for final
}

void heartbeatMain() { //Indicates he live still. now including dp!
   if(currentMillis - heartbeatPreviousTime >= heartbeatInterval){ //check for interval between last toggle and current time
    heartbeatPreviousTime = currentMillis;                          //update last toggle time
    
    if(heartbeatState == 0){ 
     heartbeatState = 1;    //swap state
     P13_ON; //internal led on
   }
   else{
     heartbeatState = 0;    //swap state
     P13_OFF; //internal led off
   }
  }
}

void MPUmeasure() {
  Wire.beginTransmission(MPU); //signal mpu on i2c bus
  Wire.write(0x3b); //start on accel_xout_h register
  Wire.endTransmission(false); //don't release bus after sending
  Wire.requestFrom(MPU,6,true); //request from mpu, 6 registers, release bus on complete
  AccX = Wire.read() << 8 | Wire.read(); //AccX high and low bit 0x3b 0x3c. <<8 shifts first bits 8 higher
  AccY = Wire.read() << 8 | Wire.read(); //AccY high and low bit 0x3d 0x3e
  AccZ = Wire.read() << 8 | Wire.read(); //AccZ high and low bit 0x3f 0x40
/*
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);
  GyrX = Wire.read() << 8 | Wire.read(); //As above, but with Gyro data. May not need.
  GyrY = Wire.read() << 8 | Wire.read();
  GyrZ = Wire.read() << 8 | Wire.read();
*/

  if(abs(AccX) > domMult*abs(AccY) && abs(AccX) > domMult*abs(AccZ)) { //is dominant axis X
    if(AccX > 0) { //X in one dir
      accelState = 2; //Tilt far
    }
    else {
      accelState = 3; //Tilt close, spin
    }
  }
  if(abs(AccY) > domMult*abs(AccX) && abs(AccY) > domMult*abs(AccZ)) { //is dom axis y?
    if(AccY > 0) { //y in one dir
      accelState = 4; //Tilt left, Coin add
    }
    else {
      accelState = 5; //Tilt right, Coin remove
    }
  }
  if(abs(AccZ) > domMult*abs(AccX) && abs(AccZ) > domMult*abs(AccY)) { //is dom axis z?
    if(AccZ > 0) { //z in one dir
      accelState = 0; //Bottom down, resting
    }
    else {
      accelState = 1; //Bottom Up
    }
  }
}

void buttonRead() {


  if(currentMillis - lastButtonDebounceTime1 >= buttonDebounceInterval) {
    
  }
  if(currentMillis - lastButtonDebounceTime2 >= buttonDebounceInterval) {
    
  }
  if(currentMillis - lastButtonDebounceTime3 >= buttonDebounceInterval) {
    
  }
}

void creditCounter() {
  orientation = accelState; //update working variable for this section
  
  if(currentMillis - orientationPreviousTime >= orientationInterval) { //update old orientation for rising edge detection
    orientationPreviousTime = currentMillis;
    oldOrientation = orientation;
  }

  if(orientation == 4 && oldOrientation == 0) { 
    currentCredits++;
    oldOrientation = orientation;
  }

  if(orientation == 5 && oldOrientation == 0) {
    currentCredits--;
    oldOrientation = orientation;
  }

  if(currentCredits > 15) { //credits may not exceed 15
    currentCredits = 15;
  }
  if(currentCredits < 0) { //credits may not be less than 0
    currentCredits = 0;
  }

  switch(currentCredits) { //pop this at the end of creditCounter. Tells the shift register to display current credits.
    case 0:sRegStatePre = SEG_0; break; //for given number, update the sReg value
    case 1:sRegStatePre = SEG_1; break;
    case 2:sRegStatePre = SEG_2; break;
    case 3:sRegStatePre = SEG_3; break;
    case 4:sRegStatePre = SEG_4; break;
    case 5:sRegStatePre = SEG_5; break;
    case 6:sRegStatePre = SEG_6; break;
    case 7:sRegStatePre = SEG_7; break;
    case 8:sRegStatePre = SEG_8; break;
    case 9:sRegStatePre = SEG_9; break;
    case 10:sRegStatePre = SEG_A; break;
    case 11:sRegStatePre = SEG_b; break;
    case 12:sRegStatePre = SEG_C; break;
    case 13:sRegStatePre = SEG_d; break;
    case 14:sRegStatePre = SEG_E; break;
    case 15:sRegStatePre = SEG_F; break;
    default:sRegStatePre = SEG_0; break;
  }
}

void shiftRegisterUpdate() {
  
  if(heartbeatState == 1){  //This short block uses bitmasks to ensure the DP matches the heartbeat without affecting the rest of the segments.
    sRegState = sRegStatePre | SEG_DP;
  }
  else{
    sRegState = sRegStatePre & ~SEG_DP;
  }

  //Serial.println("ShiftStart");
  for(int shiftBit=0; shiftBit<8;  shiftBit++) { //repeat 8 times, incrementing.
    if(bitRead(sRegState, 7-shiftBit) == 1) {
      SREG_DATA_HIGH;
      //Serial.println(bitRead(sRegState, 7-shiftBit));
    }
    else{
      SREG_DATA_LOW;
      //Serial.println(bitRead(sRegState, 7-shiftBit));
    }
    SREG_CLOCK_HIGH; //cycle clock once data stable
    SREG_CLOCK_LOW;
    SREG_DATA_LOW; //drop data
    }
  SREG_LATCH_HIGH; //latch to output data to register
  SREG_LATCH_LOW;
}

void serialPrint() { //Disable for final
    //Serial.println(BRegisterState[2]); //print state of Heartbeat from BReg
    //Serial.print('X');
    //Serial.print(AccX);
    //Serial.print('Y');
    //Serial.print(AccY);
    //Serial.print('Z');
    //Serial.println(AccZ);
    Serial.println(accelState);

}
