// RSLK Self Test via UART

/* This example accompanies the books
   "Embedded Systems: Introduction to the MSP432 Microcontroller",
       ISBN: 978-1512185676, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Interfacing to the MSP432 Microcontroller",
       ISBN: 978-1514676585, Jonathan Valvano, copyright (c) 2017
   "Embedded Systems: Real-Time Operating Systems for ARM Cortex-M Microcontrollers",
       ISBN: 978-1466468863, , Jonathan Valvano, copyright (c) 2017
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2017, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include "msp.h"
#include <stdint.h>
#include <string.h>
#include "..\inc\UART0.h"
#include "..\inc\EUSCIA0.h"
#include "..\inc\FIFO0.h"
#include "..\inc\Clock.h"
//#include "..\inc\SysTick.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\CortexM.h"
#include "..\inc\TimerA1.h"
#include "..\inc\Bump.h"
#include "..\inc\BumpInt.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"
#include "../inc/IRDistance.h"
#include "../inc/ADC14.h"
#include "../inc/LPF.h"
#include "..\inc\Reflectance.h"
#include "../inc/TA3InputCapture.h"
#include "../inc/Tachometer.h"

#define P2_4 (*((volatile uint8_t *)(0x42098070)))
#define P2_3 (*((volatile uint8_t *)(0x4209806C)))
#define P2_2 (*((volatile uint8_t *)(0x42098068)))
#define P2_1 (*((volatile uint8_t *)(0x42098064)))
#define P2_0 (*((volatile uint8_t *)(0x42098060)))

// At 115200, the bandwidth = 11,520 characters/sec
// 86.8 us/character
// normally one would expect it to take 31*86.8us = 2.6ms to output 31 characters
// Random number generator
// from Numerical Recipes
// by Press et al.
// number from 0 to 31
uint32_t Random(void){
static uint32_t M=1;
  M = 1664525*M+1013904223;
  return(M>>27);
}
char WriteData,ReadData;
uint32_t NumSuccess,NumErrors;
void TestFifo(void){char data;
  while(TxFifo0_Get(&data)==FIFOSUCCESS){
    if(ReadData==data){
      ReadData = (ReadData+1)&0x7F; // 0 to 127 in sequence
      NumSuccess++;
    }else{
      ReadData = data; // restart
      NumErrors++;
    }
  }
}
uint32_t Size;
int Program5_1(void){
//int main(void){
    // test of TxFifo0, NumErrors should be zero
  uint32_t i;
  Clock_Init48MHz();
  WriteData = ReadData = 0;
  NumSuccess = NumErrors = 0;
  TxFifo0_Init();
  TimerA1_Init(&TestFifo,43);  // 83us, = 12kHz
  EnableInterrupts();
  while(1){
    Size = Random(); // 0 to 31
    for(i=0;i<Size;i++){
      TxFifo0_Put(WriteData);
      WriteData = (WriteData+1)&0x7F; // 0 to 127 in sequence
    }
    Clock_Delay1ms(10);
  }
}

char String[64];
uint32_t MaxTime,First,Elapsed;
int Program5_2(void){
//int main(void){
    // measurement of busy-wait version of OutString
  uint32_t i;
  DisableInterrupts();
  Clock_Init48MHz();
  UART0_Init();
  WriteData = 'a';
  SysTick_Init(0x1000000,2); //OHL - using systick INT api
  MaxTime = 0;
  while(1){
    Size = Random(); // 0 to 31
    for(i=0;i<Size;i++){
      String[i] = WriteData;
      WriteData++;
      if(WriteData == 'z') WriteData = 'a';
    }
    String[i] = 0; // null termination
    First = SysTick->VAL;
    UART0_OutString(String);
    Elapsed = ((First - SysTick->VAL)&0xFFFFFF)/48; // usec

    if(Elapsed > MaxTime){
        MaxTime = Elapsed;
    }
    UART0_OutChar(CR);UART0_OutChar(LF);
    Clock_Delay1ms(100);
  }
}

int Program5_3(void){
//int main(void){
    // measurement of interrupt-driven version of OutString
  uint32_t i;
  DisableInterrupts();
  Clock_Init48MHz();
  EUSCIA0_Init();
  WriteData = 'a';
  SysTick_Init(0x1000000,2); //OHL - using systick INT api
  MaxTime = 0;
  EnableInterrupts();
  while(1){
    Size = Random(); // 0 to 31
    for(i=0;i<Size;i++){
      String[i] = WriteData;
      WriteData++;
      if(WriteData == 'z') WriteData = 'a';
    }
    String[i] = 0; // null termination
    First = SysTick->VAL;
    EUSCIA0_OutString(String);
    Elapsed = ((First - SysTick->VAL)&0xFFFFFF)/48; // usec
    if(Elapsed > MaxTime){
        MaxTime = Elapsed;
    }
    EUSCIA0_OutChar(CR);EUSCIA0_OutChar(LF);
    Clock_Delay1ms(100);
  }
}
int Program5_4(void){
//int main(void){
    // demonstrates features of the EUSCIA0 driver
  char ch;
  char string[20];
  uint32_t n;
  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();
  EUSCIA0_OutString("\nLab 5 Test program for EUSCIA0 driver\n\rEUSCIA0_OutChar examples\n");
  for(ch='A'; ch<='Z'; ch=ch+1){// print the uppercase alphabet
     EUSCIA0_OutChar(ch);
  }
  EUSCIA0_OutChar(LF);
  for(ch='a'; ch<='z'; ch=ch+1){// print the lowercase alphabet
    EUSCIA0_OutChar(ch);
  }
  while(1){
    EUSCIA0_OutString("\n\rInString: ");
    EUSCIA0_InString(string,19); // user enters a string
    EUSCIA0_OutString(" OutString="); EUSCIA0_OutString(string); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUDec: ");   n=EUSCIA0_InUDec();
    EUSCIA0_OutString(" OutUDec=");  EUSCIA0_OutUDec(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix1="); EUSCIA0_OutUFix1(n); EUSCIA0_OutChar(LF);
    EUSCIA0_OutString(" OutUFix2="); EUSCIA0_OutUFix2(n); EUSCIA0_OutChar(LF);

    EUSCIA0_OutString("InUHex: ");   n=EUSCIA0_InUHex();
    EUSCIA0_OutString(" OutUHex=");  EUSCIA0_OutUHex(n); EUSCIA0_OutChar(LF);
  }
}



/*------------------------------------------------ Case 1-----------------------------------------------*/

void RSLK_MotorTest(void){
    while(1){
          EUSCIA0_OutString("Motor Moving Forward...."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
          Motor_Forward(3000,3000);  // Drive the robot forward by running left and right wheels forward with the given duty cycles.
          Clock_Delay1ms(1000);          // run for a while and stop
          Motor_Stop();
          EUSCIA0_OutString("Motor Moving Backward...."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
          Motor_Backward(3000,3000); // Drive the robot backward by running left and right wheels backward with the given duty cycles.
          Clock_Delay1ms(1000);          // run for a while and stop
          Motor_Stop();
          EUSCIA0_OutString("Motor Moving Left...."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
          Motor_Left(3000,3000);     // Drive the robot left by running left wheel backward and the right wheel forward with the given duty cycles.
          Clock_Delay1ms(1000);          // run for a while and stop
          Motor_Stop();
          EUSCIA0_OutString("Motor Moving Right...."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
          Motor_Right(3000,3000);    // Drive the robot right by running left wheel forward and the right wheel backward with the given duty cycles.
          Clock_Delay1ms(1000);          // run for a while and stop
          Motor_Stop();
    }
}

void RSLK_MotorMove(uint8_t direction){
    if(direction == 0x03){ //Output a 11 --> move forward
          Motor_Forward(1500,1500);  // your function
    }
    else if(direction == 0x01){ //Output a 01 --> right wheel forward --> move left
          Motor_Left(1500,1500);  // your function
    }
    else if(direction == 0x02){ //Output a 10 --> left wheel forward --> move right
          Motor_Right(1500,1500);  // your function
    }
    else if(direction == 0x04){ //Output a 100 --> move backward
          Motor_Backward(2000,2000);  // your function
    }
    else{
        Motor_Stop();
    }
}
/*------------------------------------------------ Case 2-----------------------------------------------*/

volatile uint32_t ADCflag = 0;
volatile uint32_t nr,nc,nl;

void RSLK_IRSensor(void){
    while(1){
        UART0_OutUDec5(LeftConvert(nl));UART0_OutString(" mm,");
        UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" mm,");
        UART0_OutUDec5(RightConvert(nr));UART0_OutString(" mm\r\n");
        Clock_Delay1ms(1000);
      }
}

void SensorRead_ISR(void){  // runs at 2000 Hz (For Case 2 IR sensor)
  uint32_t raw17,raw12,raw16;
  P1OUT ^= 0x01;         // profile
  P1OUT ^= 0x01;         // profile
  ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
  nr = LPF_Calc(raw17);  // right is channel 17 P9.0
  nc = LPF_Calc2(raw12); // center is channel 12, P4.1
  nl = LPF_Calc3(raw16); // left is channel 16, P9.1
  ADCflag = 1;           // semaphore
  P1OUT ^= 0x01;         // profile
}

void IRSensor_Init(void){
    uint32_t raw17,raw12,raw16;
    uint32_t s;
    s = 256; // replace with your choice
    ADC0_InitSWTriggerCh17_12_16();   // initialize channels 17,12,16
    ADC_In17_12_16(&raw17,&raw12,&raw16);  // sample
    LPF_Init(raw17,s);     // P9.0/channel 17 right
    LPF_Init2(raw12,s);    // P4.1/channel 12 center
    LPF_Init3(raw16,s);    // P9.1/channel 16 left
    TimerA1_Init(&SensorRead_ISR,250);    // 2000 Hz sampling
    ADCflag = 0;
}

/*------------------------------------------------ Case 3-----------------------------------------------*/

void RSLK_Bump(void){ // Case 3 -- Extra implementation: Get simultaneous bumps
    uint8_t Data;
    Data = Bump_Read();
    //EUSCIA0_OutUHex(Data); //Print out the data in hex
    //EUSCIA0_OutString(" ");
    for(uint8_t i=1; i<7;i++ ){ //6 switches to check
        if(Data %2 == 0){ //Checking if the particular bump i is pressed, if pressed will be logic 0
            EUSCIA0_OutString("Bumper Switch ");
            EUSCIA0_OutUDec(i);
            EUSCIA0_OutString(" Pressed. ");
            EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
        }
        Data = Data >> 1; //Shift right to get next bump switch
    }
}

/*------------------------------------------------ Case 4-----------------------------------------------*/

void RSLK_ReflectanceRead(void){
    uint8_t Data; // QTR-8RC
    Reflectance_Init(); // Initialize reflectance sensors (ports 5 & 7)
    Data = Reflectance_Read(1000); //Get 8 bit input from sensors at bottom of robot
    //EUSCIA0_OutUHex2(Data); //Print out the data in hex
    for (int j = 0; j < 8; j++){
      EUSCIA0_OutUDec(Data%2);
      EUSCIA0_OutString(" ");
      Data = Data >> 1; //Shift right to get next reflectance sensor
    }
    //Reflectance_Position(Data); //Finding position of robot from black line
    //EUSCIA0_OutUDec(Data);
    EUSCIA0_OutString(" ");

}

/*------------------------------------------------ Case 5-----------------------------------------------*/
char EUSCIA0_InCharNopolling(void){
  char letter;
  if(RxFifo0_Get(&letter) == FIFOFAIL){
      return('n');
  }
  return(letter);
}
void toggle_GPIO(void){
    P2_4 ^= 0x01;     // create output
}

uint16_t Period0;              // (1/SMCLK) units = 83.3 ns units
uint16_t First0;               // Timer A3 first edge, P10.4
int Done0;                     // set each rising
// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure0(uint16_t time){
  P2_0 = P2_0^0x01;           // thread profile, P2.0
  Period0 = (time - First0)&0xFFFF; // 16 bits, 83.3 ns resolution
  First0 = time;                   // setup for next
  Done0 = 1;
}
uint16_t Period2;              // (1/SMCLK) units = 83.3 ns units
uint16_t First2;               // Timer A3 first edge, P8.2
int Done2;                     // set each rising
// max period is (2^16-1)*83.3 ns = 5.4612 ms
// min period determined by time to run ISR, which is about 1 us
void PeriodMeasure2(uint16_t time){
  P2_2 = P2_2^0x01;           // thread profile, P2.4
  Period2 = (time - First2)&0xFFFF; // 16 bits, 83.3 ns resolution
  First2 = time;                   // setup for next
  Done2 = 1;
}

void TimedPause(uint32_t time){
  Clock_Delay1ms(time);          // run for a while and stop
  Motor_Stop();
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}

void RSLK_Tachometer(void){
    uint32_t main_count=0;
    double rpm_left =0;
    double rpm_right=0;
    TimerA1_Init(&toggle_GPIO,10);    // 50Khz sampling
    TimerA3Capture_Init(&PeriodMeasure0,&PeriodMeasure2);
    Clock_Delay1ms(500);
    Motor_Forward(4000,4000); // 50%
    EnableInterrupts();
    while(1){
        WaitForInterrupt();
        char speed = EUSCIA0_InCharNopolling();
        if(speed != 'n'){
            if(speed == 'f'){
                EUSCIA0_OutString("Fast Speed..."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                Motor_Forward(5000,5000);
            }
            else if(speed == 's'){
                EUSCIA0_OutString("Slow Speed..."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                Motor_Forward(3000,3000);
            }
        }
        main_count++;
        if(main_count%1000){
           //UART0_OutString("Period0 = ");UART0_OutUDec5(Period0);UART0_OutString(" Period2 = ");UART0_OutUDec5(Period2);UART0_OutString(" \r\n");
           //Unit = 83.3 ns
           rpm_left = ((60 * 1000 * 1000 )/(360*83.3*Period0/1000)); //Calculating RPM
           rpm_right = ((60 * 1000 * 1000)/(360*83.3*Period2/1000));
           UART0_OutString("RPM Left = ");UART0_OutUDec5(rpm_left);UART0_OutString(" RPM Right = ");UART0_OutUDec5(rpm_right);UART0_OutString(" \r\n");
           }
    }
    Motor_Stop();
}

/*------------------------------------------------ Case 6 (FSM Implementation) -----------------------------------------------*/
struct State {
  uint32_t out;                // 2-bit output
  uint32_t delay;              // time to delay in 1ms
  const struct State *next[4]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center    &fsm[0]
#define Left1      &fsm[1]
#define Left2      &fsm[2]
#define Right1     &fsm[3]
#define Right2     &fsm[4]
#define HardLeft   &fsm[5]
#define HardRight   &fsm[6]
#define Center2    &fsm[7]
#define Stop    &fsm[8]
#define Back    &fsm[9]

State_t fsm[10]={
  {0x03, 100, { Back, Left1,   Right2,  Center }},  // Center state 500 ms, 0x03 represents 11 -> both wheels move, {00,01,10,11} is the input for the next state
  {0x02, 100, { HardRight,  Left2, Left2,  Left2 }},  // Left1
  {0x03, 100, { HardRight,  Left1, Right2,  Center }},  // Left2
  {0x03, 100, { HardLeft, Left1,   Right2, Center }},   // Right1
  {0x01, 100, { HardLeft, Right1, Right1, Right1 }},   // Right2
  {0x02, 200, { Back,  Left1, Right2,  Center }},  // HardLeft
  {0x01, 200, { Back,  Left1, Right2,  Center }},  // HardRight
  {0x03, 100, { Back,  Left2, Right1,  Center }},  // Center2
  {0x00, 100, { Stop,  Left1, Right2,  Center }},  // Stop
  {0x04, 200, { Stop,  Left1, Right2,  Center }},  // Back
};

State_t *Spt;  // pointer to the current state
uint32_t Input;
uint32_t Output;

void RSLK_FSMImplementation(void){
    Spt = Center; //Start from center state
    while(1){
        Output = Spt->out;            // set output from FSM
        RSLK_MotorMove(Output);     // do output to two motors
        Clock_Delay1ms(Spt->delay);   // wait
        Input = Reflectance_Center(100);    // read reflectance sensors
        EUSCIA0_OutUDec(Input);    //To check reflectance sensors for center
        Spt = Spt->next[Input];       // next depends on input and state --> to go to next state depending on the new input
    }
}


/*------------------------------------------------ Case 7 (Obstacle Avoidance via IR Sensors & Bump switches) -----------------------------------------------*/
uint32_t direction;
void RSLK_ObstacleBump(void){ // Case 7 -- Obstacle Avoidance via bump polling
    uint8_t Data;
    Data = Bump_Read();
    //EUSCIA0_OutUHex(Data); //Print out the data in hex
    //EUSCIA0_OutString(" ");
    for(uint8_t i=1; i<7;i++ ){ //6 switches to check
        if(Data %2 == 0 && i <= 3){ //Checking if the particular bump i is pressed, if pressed will be logic 0
            EUSCIA0_OutString("Bumper Switch ");
            EUSCIA0_OutUDec(i);
            EUSCIA0_OutString(" Pressed. ");
            Motor_Backward(3000,3000);
            Clock_Delay1ms(1000);          // run for a while and stop
            Motor_Left(3000,3000);
            Clock_Delay1ms(1000);          // run for a while and stop
            Motor_Forward(3000,3000);
            EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
            break;
        }
        else if(Data %2 == 0 && i > 3){
            EUSCIA0_OutString("Bumper Switch ");
            EUSCIA0_OutUDec(i);
            EUSCIA0_OutString(" Pressed. ");
            Motor_Backward(3000,3000);
            Clock_Delay1ms(1000);          // run for a while and stop
            Motor_Right(3000,3000);
            Clock_Delay1ms(1000);          // run for a while and stop
            Motor_Forward(3000,3000);
            EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
            break;
        }
        Data = Data >> 1; //Shift right to get next bump switch
    }
}
void RSLK_ObstacleAvoidance(void){
    EUSCIA0_OutString("Press w to move forward, a to move left, d to move right, s to move backwards...."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
    while (1){
        char direction = EUSCIA0_InCharNopolling();
        if(direction != 'n'){ //If no keyboard input, just continue in a particular direction
            if(direction == 'w'){
                EUSCIA0_OutString("Moving Straight..."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                Motor_Forward(3000,3000);
            }
            else if(direction == 's'){
                EUSCIA0_OutString("Moving Back..."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                Motor_Backward(3000,3000);
            }
            else if(direction == 'a'){
                EUSCIA0_OutString("Turning left..."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                Motor_Left(3000,3000);
            }
            else if(direction == 'd'){
                EUSCIA0_OutString("Turning right..."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                Motor_Right(3000,3000);
            }
            else if(direction == 'm'){
                EUSCIA0_OutString("Stopping..."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
                Motor_Stop();
                break;
            }
        }
        if(LeftConvert(nl)<=10){
            Motor_Stop();
            Motor_Right(3000,3000);
            Clock_Delay1ms(500);
            Motor_Forward(3000,3000);
        }
        else if(CenterConvert(nc)<=10){
            Motor_Stop();
            Motor_Backward(3000,3000);
            Clock_Delay1ms(500);
            Motor_Right(3000,3000);
            Clock_Delay1ms(1400);
            Motor_Forward(3000,3000);
        }
        else if(RightConvert(nr)<=10){
            Motor_Stop();
            Motor_Left(3000,3000);
            Clock_Delay1ms(500);
            Motor_Forward(3000,3000);
        }
        RSLK_ObstacleBump();
        //UART0_OutUDec5(LeftConvert(nl));UART0_OutString(" cm,");
        //UART0_OutUDec5(CenterConvert(nc));UART0_OutString(" cm,");
        //UART0_OutUDec5(RightConvert(nr));UART0_OutString(" cm\r\n");
    }
    Motor_Stop();
}

/*------------------------------------------------ Case 0-----------------------------------------------*/

void RSLK_Reset(void){
    DisableInterrupts();

    LaunchPad_Init();
    //Initialise modules used e.g. Reflectance Sensor, Bump Switch, Motor, Tachometer etc
    // ... ...
    Clock_Init48MHz();  // makes SMCLK=12 MHz
    //SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts
    Motor_Init();
    Motor_Stop();
    LaunchPad_Init();
    Bump_Init();
    //BumpInt_Init(&HandleCollision);
    IRSensor_Init();
    Tachometer_Init();
    EUSCIA0_Init();     // initialize UART

    EnableInterrupts();
}

/*------------------------------------------------ RSLK Self Test-----------------------------------------------*/
int main(void) {
  uint32_t cmd=0xDEAD, menu=0;
  DisableInterrupts();
  Clock_Init48MHz();  // makes SMCLK=12 MHz
  //SysTick_Init(48000,2);  // set up SysTick for 1000 Hz interrupts
  Motor_Init();
  Motor_Stop();
  LaunchPad_Init();
  Bump_Init();
  //BumpInt_Init(&HandleCollision);
  IRSensor_Init();
  Tachometer_Init();
  EUSCIA0_Init();     // initialize UART
  EnableInterrupts();

  while(1){                     // Loop forever
      // write this as part of Lab 5
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("RSLK Testing"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[0] RSLK Reset"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[1] Motor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[2] IR Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[3] Bumper Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[4] Reflectance Sensor Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[5] Tachometer Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[6] FSM Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
      EUSCIA0_OutString("[7] Obstacle Avoidance Test"); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);

      EUSCIA0_OutString("CMD: ");
      cmd=EUSCIA0_InUDec(); //scanf
      EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF); //print nextline

      switch(cmd){
          case 0:
              RSLK_Reset();
              menu =1;
              cmd=0xDEAD;
              break;
          case 1:
              EUSCIA0_OutString("Commencing Motor Test...."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              RSLK_MotorTest();
              break;
          case 2:
              RSLK_IRSensor();
              break;
          case 3:
              EUSCIA0_OutString("Commencing Bumper Test...."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              while(1){
                  RSLK_Bump();
                  Clock_Delay1ms(1000);
              }
          case 4:
              EUSCIA0_OutString("Commencing Reflectance Sensor Test...."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              EUSCIA0_OutString("Hexadecimal value read at the bottom: "); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              while(1){
                  RSLK_ReflectanceRead();
                  Clock_Delay1ms(1000);
              }
              menu = 1;
              break;
          case 5: //Added speed settings of fast & slow as well
              EUSCIA0_OutString("Commencing Tachometer Sensor Test...."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              RSLK_Tachometer();
              // ....
              // ....
              menu = 1;
              break;

          case 6:
              EUSCIA0_OutString("Commencing FSM Test...."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              RSLK_FSMImplementation();
              break;

          case 7:
              //Obstacle Avoidance for IR Sensor & Bump switch
              //Keyboard Input of robot direction to move
              EUSCIA0_OutString("Commencing Obstacle Avoidance...."); EUSCIA0_OutChar(CR); EUSCIA0_OutChar(LF);
              RSLK_ObstacleAvoidance();
              break;

          default:
              menu=1;
              break;
      }

      if(!menu)Clock_Delay1ms(3000);
      else{
          menu=0;
      }

      // ....
      // ....
  }
}
