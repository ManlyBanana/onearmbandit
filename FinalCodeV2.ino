//P2527970 ENGD2103 Embedded Systems Assessment Submission

//I've got some notes and thoughts at the bottom.

//Libs
#include <Arduino.h>
#include <Wire.h>

//Timing
unsigned long currentMillis = 0; //Variable for time since program start ~50 days

//Heartbeat
bool heartbeatState; //store state of heartbeat for toggle
const int heartbeatInterval = 500; //interval for heartbeat. Swap between 500 and 1000 to confirm difference. 500 for final.
unsigned long heartbeatPreviousTime = 0; //big number for last time of heartbeat state flip
#define P13 B00100000
#define P13_ON PORTB = PORTB | P13; //macro to internal led high. | writes bit 1
#define P13_OFF PORTB = PORTB & ~P13; //macro to internal led low. & writes bit 0

//MPU
const int MPU = 0x68; //mpu address
int16_t AccX, AccY, AccZ; //Integers for acceleration. Signed and constant 16bits. 
//int16_t GyrX, GyrY, GyrZ;  //As above, but not sure if need rotation rates? Take readings, delete if not useful or to free memory.
int accelState; //store state of dominant accelerometer axis

//MPU Filtering (Also used in the wheel roll detection)
int orientation; //current orientation
int oldOrientation; //store last orientation
const int orientationInterval = 200; //interval to find old orientation (debouncing)
unsigned long orientationPreviousTime = 0; //last orientation
const int domMult = 2; //multiplier for determining dominant axis. adjust for best performance

//SREG vars
byte sRegStatePre = B00000000; //holds state of sReg before heartbeat
byte sRegState; //sReg state to use after heartbeat
byte oldSRegState; //last state of sReg. use for scheduler if implemented
bool displayInvert; //Should the display be inverted? 1 is yes
unsigned long lastDisplayInvert = 0; //Non blocing delay var
const int displayInvertTime = 500; //delay time. Check spec.

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
#define SREG_LATCH_HIGH PORTC |= B00001000 //bit mask for relevant pin high/low
#define SREG_LATCH_LOW  PORTC &= B11110111
#define SREG_CLOCK_HIGH PORTB |= B00010000
#define SREG_CLOCK_LOW  PORTB &= B11101111
#define SREG_DATA_HIGH  PORTB |= B00000001
#define SREG_DATA_LOW   PORTB &= B11111110

//Button variables
const int buttonEdgeInterval = 50; //interval delay for debouncing

bool rawButton1State = 0; //state for buttons and last states (debouncing)
bool oldButton1State = 0; //last state of buttons
unsigned long lastButton1ReadTime = 0; //last time of debounces
unsigned long fallingEdgeTime1 = 0; //last time of falling edge
unsigned long risingEdgeTime1 = 0; //last time of rising edge
bool button1Press300 = 0; //button press >300ms. Lasts one loop.

bool rawButton2State = 0; //as above
bool oldButton2State = 0;
unsigned long lastButton2ReadTime = 0;
unsigned long fallingEdgeTime2 = 0;
unsigned long risingEdgeTime2 = 0;
bool button2Press300 = 0;

bool rawButton3State = 0;
bool oldButton3State = 0;
unsigned long lastButton3ReadTime = 0;
unsigned long fallingEdgeTime3 = 0;
unsigned long risingEdgeTime3 = 0;
bool button3Press300 = 0;

//Wheel Macros
//Wheel 1 
#define WHEEL_1_OFF     PORTD &= B11100011 //set the 3 LEDs in the wheel to off
#define WHEEL_1_RED     PORTD |= B00000100; PORTD &= B11100111 //set relevant LEDs
#define WHEEL_1_YLW     PORTD |= B00001000; PORTD &= B11101011
#define WHEEL_1_GRN     PORTD |= B00010000; PORTD &= B11110011
//Wheel 2
#define WHEEL_2_OFF     analogWrite(9, 0); analogWrite(10, 0); analogWrite(11, 0) //We can use analog write for this
#define WHEEL_2_RED     analogWrite(9, 255); analogWrite(10, 0) //might change to digital and just use resistors. maybe not?
#define WHEEL_2_YLW     analogWrite(9, 255); analogWrite(10, 48)
#define WHEEL_2_GRN     analogWrite(9, 0); analogWrite(10, 255)
//Wheel 3
#define WHEEL_3_OFF     PORTD &= B00011111
#define WHEEL_3_RED     PORTD |= B00100000; PORTD &= B00111111
#define WHEEL_3_YLW     PORTD |= B01000000; PORTD &= B01011111
#define WHEEL_3_GRN     PORTD |= B10000000; PORTD &= B10011111

//Wheel Variables
bool rollingAllNorm; //variable states if rolling (overall) is occuring

unsigned int wheel1Time; //total time to spin wheel
int wheel1State; //current state of the wheel
bool wheel1SpinningNorm; //state of the wheel spinning in 'normal' mode (all 3)
unsigned long lastWheel1SpinningNorm = 0; //time of wheel spin state change
unsigned int wheel1ChangeTime; //time to change to next colour. Starts at 50, increments.
unsigned long lastWheel1StateChange = 0; //when did the wheel last change?
unsigned long lastWheel1TimerChange = 0; //when did we last update the wheel timer

unsigned int wheel2Time; //these are as above.
int wheel2State;
bool wheel2SpinningNorm;
unsigned long lastWheel2SpinningNorm = 0;
unsigned int wheel2ChangeTime;
unsigned long lastWheel2StateChange = 0;
unsigned long lastWheel2TimerChange = 0;

unsigned int wheel3Time; //same here
int wheel3State;
bool wheel3SpinningNorm;
unsigned long lastWheel3SpinningNorm = 0;
unsigned int wheel3ChangeTime;
unsigned long lastWheel3StateChange = 0;
unsigned long lastWheel3TimerChange = 0;

bool endRollFlash = 0;
int endRollFlashNumber = 0;
const int endRollFlashInterval = 500;
unsigned long endRollLastFlash = 0;

//Solo Spin Vars
bool soloWheel1Spin = 0; //bool states for individual wheel spins
bool soloWheel2Spin = 0;
bool soloWheel3Spin = 0;
bool soloRandomGen = 0;

//Game Variables
int currentCredits = 0; //store current credits. Could program to eeprom to save progress?
bool matchingWheels = 0; //turns 1 if all 3 wheels match
const int jackpotHoldTime = 500; //wait time when a jackpot is achieved to hold wheels
unsigned long jackpotPreviousTime = 0; //time for non blocking delay
bool randomGen; //random generation 
bool jackpotAllow = 1; //check if the jackpot has already been detected

//Debug
int debugVar = 0; //I used this variable where I needed to quickly check the flow of data through a function. Quick and simple.

  //Run at startup
void setup() {
  //Register setup. Determine data direction and pullups (if applicable)
  DDRB = DDRB | B00111111; //BReg outputs. first 2 not available. rest output
  PORTB = PORTB | B00000000; //BReg states.
  DDRC = DDRC | B00111000;
  PORTC = PORTC | B00000111; //pull ups
  DDRD = DDRD | B11111100; 
  PORTD = PORTD | B00000000;

  //Begin I2C
  Wire.begin(); //start i2c
  Wire.beginTransmission(MPU); //transmit to mpu address
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true); //leave bus open

  //Clear Shift Register
  SREG_LATCH_LOW; //sets latch low before data sending
  SREG_DATA_LOW; //data line low
  for(int shiftBit=0; shiftBit<8;  shiftBit++) { //Pulse clock 8 times to clear data
    SREG_CLOCK_HIGH;  
    SREG_CLOCK_LOW;
    }
  SREG_LATCH_HIGH; //latch to output data to register
  SREG_LATCH_LOW; //finish latch toggle, and leave latch low

  wheel1State = 0; //set Wheel States to 0
  wheel2State = 0;
  wheel3State = 0;

  //Serial
  //Serial.begin(9600); //begin Serial port. Disable for final.
  //Serial.println("Serial begin");
}

  //Main loop
void loop() { //each module contains it's own scheduling, so unnecesary cycles can be avoided. these are contained within the fu
  currentMillis = millis(); //update time since program start, so all functions are on the same time

  heartbeatMain(); //run main heartbeat loop, output to P13 and a global variable

  MPUmeasure(); //run MPU reading, and determine dominant axis for tilt detection

  buttonRead(); //reads buttons from PIN register, performs debounce and (rising) edge detect.

  leverPullDetect(); //conditions for a valid lever pull

  wheelSpinMain(); //Script for a roll of all 3 wheels

  wheelSpinSolo(); //single wheel spins

  wheelLEDOutput(); //Wheel LED Output

  jackpotDetect(); //Detect and respond to jackpot conditions, w/ payout and game reset

  creditCounter(); //before shift reg so we can do this in one loop

  shiftRegisterUpdate(); //run sReg update

  //serialPrint(); //run serialPrint. Not needed in final
}

void heartbeatMain() { 
if(currentMillis - heartbeatPreviousTime >= heartbeatInterval){ //check for interval between last toggle and current time
  heartbeatPreviousTime = currentMillis;                          //update last toggle time
    if(heartbeatState == 0){ 
      heartbeatState = 1;    //State off
      //P13_ON; //internal led on
    }
    else{
      heartbeatState = 0;    //State on
      //P13_OFF; //internal led off
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
  /* Enable if i need gyro data. Probably won't
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,6,true);
  GyrX = Wire.read() << 8 | Wire.read(); 
  GyrY = Wire.read() << 8 | Wire.read();
  GyrZ = Wire.read() << 8 | Wire.read();
  */

  if(abs(AccX) > domMult*abs(AccY) && abs(AccX) > domMult*abs(AccZ)) { //is dominant axis X
    if(AccX > 0) { //X in one dir
      accelState = 2; //Tilt away
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
  rawButton1State = PINC & B00000100; //Set variables from reading pins on Register C. A3, 2, 1, respectively
  rawButton2State = PINC & B00000010;
  rawButton3State = PINC & B00000001;

  if(currentMillis - lastButton1ReadTime >= buttonEdgeInterval) { //only run every 50ms
    lastButton1ReadTime = currentMillis; //update last time
    if(rawButton1State != oldButton1State) { //check for differing button states
      if(rawButton1State == LOW) { //check for falling edge
        fallingEdgeTime1 = currentMillis; //update time of falling edge
      }
      if(rawButton1State == HIGH) { //check for rising edge
        risingEdgeTime1 = currentMillis; //update rising edge time
        if(risingEdgeTime1 - fallingEdgeTime1 >= 300) { //find difference in times
          button1Press300 = 1; //set the variable to 0
        }
        fallingEdgeTime1 = 0; //reset variables after finding difference
        risingEdgeTime1 = 0; //
      }
    }
    else {
      button1Press300 = 0; //after a loop, or otherwise, reset to 0. This means it should only send one pulse per loop
    }
    oldButton1State = rawButton1State; //update old button state
  }

  if(currentMillis - lastButton2ReadTime >= buttonEdgeInterval) { //As above, but with button 2
    lastButton2ReadTime = currentMillis;
    if(rawButton2State != oldButton2State) {
      if(rawButton2State == LOW) {
        fallingEdgeTime2 = currentMillis;
      }
      if(rawButton2State == HIGH) {
        risingEdgeTime2 = currentMillis;
        if(risingEdgeTime2 - fallingEdgeTime2 >= 300) {
          button2Press300 = 1;
        }
        fallingEdgeTime2 = 0;
        risingEdgeTime2 = 0;
      }
    }
    else {
      button2Press300 = 0;
    }
    oldButton2State = rawButton2State;
  }

  if(currentMillis - lastButton3ReadTime >= buttonEdgeInterval) { //as above, but now with button 3
    lastButton3ReadTime = currentMillis;
    if(rawButton3State != oldButton3State) {
      if(rawButton3State == LOW) {
        fallingEdgeTime3 = currentMillis;
      }
      if(rawButton3State == HIGH) {
        risingEdgeTime3 = currentMillis;
        if(risingEdgeTime3 - fallingEdgeTime3 >= 300) {
          button3Press300 = 1;
        }
        fallingEdgeTime3 = 0;
        risingEdgeTime3 = 0;
      }
    }
    else {
      button3Press300 = 0;
    }
    oldButton3State = rawButton3State;
  }
}

void leverPullDetect() {
  orientation = accelState; //update orientation variable for this section

  if(currentMillis - orientationPreviousTime >= orientationInterval && rollingAllNorm == 0 && soloWheel1Spin == 0 && soloWheel2Spin == 0 && soloWheel3Spin == 0) { //Check interval 200ms, and only when not rolling
    if(orientation == 0 && oldOrientation == 3 && currentCredits >= 1) { //Check for spinning movement (return to flat), and credits >0
      oldOrientation = orientation; //update last orientation
      currentCredits--; //take 1 credit
      rollingAllNorm = 1; //Start rolling process
      randomGen = 1; //start random number generation
      jackpotAllow = 1; //allows jackpots
      randomSeed(currentMillis * AccZ); //I was not happy with the current random numbers, which were repeating exactly on each reset. This is an attempt to seed it.
    }
  }
}

void wheelSpinMain() {
  //When the rolling is first called
  if(rollingAllNorm == 1) { //Are we currently rolling?
    if(randomGen == 1) { //Are we supposed to be generating the random numbers & resetting the game?
      wheel1ChangeTime = 50; //reset all delays for wheel led switches to 50ms
      wheel2ChangeTime = 50;
      wheel3ChangeTime = 50;
      
      wheel1Time = random(1200,1600); //generate random times for each wheel
      wheel2Time = random(2000,2600);
      wheel3Time = random(2000,2600);

      wheel1State = random(1,4); //random start states for wheels
      wheel2State = random(1,4); //SHOULD I KEEP THIS??? PROBABLY NOT. HECK.
      wheel3State = random(1,4);

      wheel1SpinningNorm = 1; //Set all wheels spinning AFTER delays and states are ready.
      wheel2SpinningNorm = 1; //literally just states whether we spin or not.
      wheel3SpinningNorm = 1;

      lastWheel1SpinningNorm = currentMillis; //update time of change (and wheel starting)
      lastWheel2SpinningNorm = currentMillis;
      lastWheel3SpinningNorm = currentMillis;

      lastWheel1StateChange = currentMillis; //update these time variables too, to hold time of state change
      lastWheel2StateChange = currentMillis;
      lastWheel3StateChange = currentMillis;

      lastWheel1TimerChange = currentMillis; //and this one for when we increment the timer
      lastWheel2TimerChange = currentMillis;
      lastWheel3TimerChange = currentMillis; 

      randomGen = 0; //don't generate more than one per roll
    }
    if(wheel1SpinningNorm == 1) {  //detects this wheel should be spinning
      if(currentMillis - lastWheel1SpinningNorm <= wheel1Time) { //Has the time to spin elapsed?
        if(currentMillis - lastWheel1StateChange >= wheel1ChangeTime) { //should the colour have changed?
          lastWheel1StateChange = currentMillis; //update timer
          wheel1State++; //increment wheel state
          if(wheel1State > 3) { //don't let colour overflow
            wheel1State = 1;
          }
          if(currentMillis - lastWheel1SpinningNorm >= 500 && currentMillis - lastWheel1TimerChange >= 20) { //has it been 500ms and the increment for delay change?
            lastWheel1TimerChange = currentMillis; //update timer
            wheel1ChangeTime = wheel1ChangeTime + 15; //increment colour delay
          }
        }
      }
      else {
        wheel1SpinningNorm = 0; //if timer has elapsed, turn this module off
        lastWheel1SpinningNorm = currentMillis; //update the last change time
      }
    }
    
    if(wheel2SpinningNorm == 1) { //should this be spinning
      if(currentMillis - lastWheel2SpinningNorm <= wheel2Time + wheel1Time) { //as above, but with cumulative time
        if(currentMillis - lastWheel2StateChange >= wheel2ChangeTime) {
          lastWheel2StateChange = currentMillis;
          wheel2State++;
          if(wheel2State > 3) {
            wheel2State = 1;
          }
        }
        if(currentMillis - lastWheel2TimerChange >= 20 && wheel1SpinningNorm == 0) { //if wheel 1 has stopped, begin incrememnting timer
          lastWheel2TimerChange = currentMillis;
          wheel2ChangeTime = wheel2ChangeTime + 10;
        }
      }
      else {
        wheel2SpinningNorm = 0; //as above
        lastWheel2SpinningNorm = currentMillis;
      }
    }

    if(wheel3SpinningNorm == 1) { //as above
      if(currentMillis - lastWheel3SpinningNorm <= wheel3Time + wheel2Time + wheel1Time) { //cumulative time
        if(currentMillis - lastWheel3StateChange >= wheel3ChangeTime) {
          lastWheel3StateChange = currentMillis;
          wheel3State++;
          if(wheel3State > 3) {
            wheel3State = 1;
          }
        }
        if(currentMillis - lastWheel3TimerChange >= 20 && wheel2SpinningNorm == 0) { //if wheel 2 has stopped, begin slowing down
          lastWheel3TimerChange = currentMillis;
          wheel3ChangeTime = wheel3ChangeTime + 10;
        }
      }
      else {
        wheel3SpinningNorm = 0;
        lastWheel3SpinningNorm = currentMillis;
        endRollFlash = 1; //start the flashing at end of the roll
      }
    }
    
    if(endRollFlash == 1 && endRollFlashNumber < 4) { //end roll flashing. For loop, but non blocking. Checks for i <4
      if(currentMillis - endRollLastFlash >= endRollFlashInterval) { //need to change SReg Invert state?
        endRollLastFlash = currentMillis; //update timer
        endRollFlashNumber++; //increment 'i' counter
        if(endRollFlashNumber % 2 == 1) { //even numbers don't invert, odd do. Should end on displayInvert = 0
          displayInvert = 1; //as explained above
        }
        else {
          displayInvert = 0; //turn off display invert in all other cases. 
        }
      }
    }
    else if(endRollFlash == 1 && endRollFlashNumber >= 4) { //check if we've hit the right number of flashes
      endRollFlash = 0; //turn off this stuff
      endRollFlashNumber = 0; //and reset the 'i' variable
    }
    if(wheel1SpinningNorm == 0 && wheel2SpinningNorm == 0 && wheel3SpinningNorm == 0 && endRollFlash == 0) { //detect all 3 wheels have stopped and flash is stopped
      rollingAllNorm = 0; //turn of the rolling var
    }
  }
}

void wheelSpinSolo() {
  if(rollingAllNorm == 0 && currentCredits >= 5 && jackpotAllow == 1 && soloWheel1Spin == 0 && soloWheel2Spin == 0 && soloWheel3Spin == 0) { //checks if there's just been a win, it's not currently rolling, and we have enough credits.
    if(button1Press300) { //next 3 are the same. 
      button1Press300 = 0; //did we have a button press?
      currentCredits -= 5; //take 5 credits
      soloWheel1Spin = 1; //tun on wheel 1 spin
      soloRandomGen = 1; //randomly gen numbers
      lastWheel1SpinningNorm = currentMillis; //update timers
      lastWheel1StateChange = currentMillis;
      lastWheel1TimerChange = currentMillis;
      wheel1ChangeTime = 50; //reset this time
    }
    if(button2Press300) { //as above
      button2Press300 = 0;
      currentCredits -= 5;
      soloWheel2Spin = 1;
      soloRandomGen = 1;
      lastWheel2SpinningNorm = currentMillis;
      lastWheel2StateChange = currentMillis;
      lastWheel2TimerChange = currentMillis;
      wheel2ChangeTime = 50;
    }
    if(button3Press300) { //as above
      button3Press300 = 0;
      currentCredits -= 5;
      soloWheel3Spin = 1;
      soloRandomGen = 1;
      lastWheel3SpinningNorm = currentMillis;
      lastWheel3StateChange = currentMillis;
      lastWheel3TimerChange = currentMillis;
      wheel3ChangeTime = 50;
    }
  }

  if(soloRandomGen) { //generate new timings for the individual wheel spins, and only once
    wheel1Time = random(2000, 2600); //I didn't like original timings
    wheel2Time = random(2000, 2600);
    wheel3Time = random(2000, 2600);
    soloRandomGen = 0;
  }

  //Most of this code is the same as main spins
  if(soloWheel1Spin) {  //detects this wheel should be spinning 
    if(currentMillis - lastWheel1SpinningNorm <= wheel1Time) { //Has the time to spin elapsed?
      if(currentMillis - lastWheel1StateChange >= wheel1ChangeTime) { //should the colour have changed?
        lastWheel1StateChange = currentMillis; //update timer
        wheel1State++; //increment wheel state
        if(wheel1State > 3) { //don't let colour overflow
          wheel1State = 1;
        }
        if(currentMillis - lastWheel1TimerChange >= 20) { //has it been 500ms and the increment for delay change?
          lastWheel1TimerChange = currentMillis; //update timer
          wheel1ChangeTime = wheel1ChangeTime + 15; //increment colour delay
        }
      }
    }
    else {
      soloWheel1Spin = 0; //if timer has elapsed, turn this module off
      lastWheel1SpinningNorm = currentMillis; //update the last change time
    }
  }

  if(soloWheel2Spin) { //Same as above, for wheel 2
    if(currentMillis - lastWheel2SpinningNorm <= wheel2Time) {
      if(currentMillis - lastWheel2StateChange >= wheel2ChangeTime) {
        lastWheel2StateChange = currentMillis;
        wheel2State++;
        if(wheel2State > 3) {
          wheel2State = 1;
        }
        if(currentMillis - lastWheel2TimerChange >= 20) {
          lastWheel2TimerChange = currentMillis;
          wheel2ChangeTime = wheel2ChangeTime + 10;
        }
      }
    }
    else {
      soloWheel2Spin = 0;
      lastWheel2SpinningNorm = currentMillis;
    }
  }

  if(soloWheel3Spin) { //and same for wheel 3
    if(currentMillis - lastWheel3SpinningNorm <= wheel3Time) {
      if(currentMillis - lastWheel3StateChange >= wheel3ChangeTime) {
        lastWheel3StateChange = currentMillis;
        wheel3State++;
        if(wheel3State > 3) {
          wheel3State = 1;
        } 
        if(currentMillis - lastWheel3TimerChange >= 20) {
          lastWheel3TimerChange = currentMillis;
          wheel3ChangeTime = wheel3ChangeTime + 10;
        }
      }
    }
    else {
      soloWheel3Spin = 0;
      lastWheel3SpinningNorm = currentMillis;
    }
  }
}

void wheelLEDOutput() {  
  switch(wheel1State) {  //take the variable wheel1State
    case 0: WHEEL_1_OFF; break; //for each value it can assume (0-4), this corresponds to an LED command
    case 1: WHEEL_1_RED; break;
    case 2: WHEEL_1_YLW; break;
    case 3: WHEEL_1_GRN; break;
    default: WHEEL_1_OFF; break; //if for whatever reason it's not 0-4, it'll default to off.
  }
  switch(wheel2State) { //same as above
    case 0: WHEEL_2_OFF; break;
    case 1: WHEEL_2_RED; break;
    case 2: WHEEL_2_YLW; break;
    case 3: WHEEL_2_GRN; break;
    default: WHEEL_2_OFF; break;
  }
  switch(wheel3State) { //same here too
    case 0: WHEEL_3_OFF; break;
    case 1: WHEEL_3_RED; break;
    case 2: WHEEL_3_YLW; break;
    case 3: WHEEL_3_GRN; break;
    default: WHEEL_3_OFF; break;
  }
}

void jackpotDetect() { //try to revise and optimise this. It's awful.
  if(rollingAllNorm == 0 && soloWheel1Spin == 0 && soloWheel2Spin == 0 && soloWheel3Spin == 0) { //only check when rolling has stopped
    if(wheel1State == wheel2State && wheel2State == wheel3State && wheel1State != 0 && jackpotAllow == 1) { //determine jackpot conditions. not on empty wheels
      matchingWheels = 1; //update a variable
      if(currentMillis - jackpotPreviousTime >= jackpotHoldTime) { //hold jackpot winning state for defined interval. 
        if(matchingWheels == 1) { //if 3 wheels matching
          Serial.println("Jackpot");
          currentCredits = currentCredits + 10; //pay out 10 creds max
          jackpotPreviousTime = currentMillis; 
          matchingWheels = 0; //reset variable
          jackpotAllow = 0; //sets variable to avoid multiple payouts
        }
      }
    }
  }
}

void creditCounter() {
  orientation = accelState; //update working variable for this section

  if(currentMillis - orientationPreviousTime >= orientationInterval) { //has it been 200ms since we last checked?
    if(orientation == 4 && oldOrientation == 0) { //detect tilt given last state is flat. Add 1 (left tilt)
      currentCredits++; //increase credits
    }
    if(orientation == 5 && oldOrientation == 0) { //tilt given last state flat. take 1 (right tilt)
      currentCredits--; //decrement
    }
    orientationPreviousTime = currentMillis; //update check time
    oldOrientation = orientation; //update orientation
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
  byte sRegStateTemp; //temporary local var

  if(displayInvert == 1) { //Invert Display on command BEFORE Heartbeat
    sRegStateTemp = ~sRegStatePre; //Bitwise invert state
  }
  else if(displayInvert == 0) { //double check if necessary, though better safe than sorry.
    sRegStateTemp = sRegStatePre;
  }

  if(heartbeatState == 1) {  //This short block uses bitmasks to ensure the DP matches the heartbeat without affecting the rest of the segments.
    sRegState = sRegStateTemp | SEG_DP; //Add in DP with bitwise OR
  }
  else{
    sRegState = sRegStateTemp & ~SEG_DP; //remove DP with bitwise AND and inverse.
  }

  //Serial.println("ShiftStart");
  SREG_LATCH_LOW; //sets latch low before data sending
  for(int shiftBit=0; shiftBit<8;  shiftBit++) { //repeat 8 times, incrementing .
    if(bitRead(sRegState, 7-shiftBit) == 1) { //read the bit of the variable for sReg, and start from Right-most bit
      SREG_DATA_HIGH;
      //Serial.println(bitRead(sRegState, 7-shiftBit));
    }
    else{
      SREG_DATA_LOW;
      //Serial.println(bitRead(sRegState, 7-shiftBit));
    }
    SREG_CLOCK_HIGH; //cycle clock once data value stable
    SREG_CLOCK_LOW;
    SREG_DATA_LOW; //0 on data, if it was already high, after clock pulse
    }
  SREG_LATCH_HIGH; //latch to output data to register
  SREG_LATCH_LOW; //finish laggle toggle
}

  void serialPrint() { //Disable for final
  //Serial.println(BRegisterState[2]); //print state of Heartbeat from BReg
  //Serial.print('X');
  //Serial.print(AccX);
  //Serial.print('Y');
  //Serial.print(AccY);
  //Serial.print('Z');
  //Serial.println(AccZ);
  //Serial.println(accelState);
  //Serial.println(debugVar);
  }

/*
I entered this project earlier than I think
*/