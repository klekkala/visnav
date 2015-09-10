### Author: Kiran Kumar Lekkala ###

#ifndef QUADARDU
#define QUADARDU

#include <Servo.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <helper_3dmath.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>

#define DEBUG
#define CHANNELS 8


/****  Arduino Pin configuration
   **ESP8266(ESP-07) Wifi Serial Adapter
	3.3V  <--> VCC
	3.3V  <--> CH_PD
	GND <--> GND
	D13 <--> TX 
	D2 <--> RX

   **Multiwii Flight Controller
	D12 <--> 33(CAM PITCH)
	D11 <--> 32(CAM ROLL)  
	D10 <--> A15(AUX-4) 
	D09 <--> A14(AUX-3)
	D08 <--> A13(AUX-2)
	D07 <--> A12(AUX-1)
	D06 <--> A11(YAW)
	D05 <--> A10(PITCH)
	D04 <--> A09(ROLL)
	D03 <--> A08(THROTTLE)

  ***USB to Raspberry
**/

/** Pin Macros **/
#define CAM_PITCH 11 
#define CAM_ROLL 10
#define AUX-1 9
#define AUX-2 8
#define AUX-3 7
#define AUX-4 6
#define YAW 5
#define PITCH 4
#define ROLL 3
#define THROTTLE 2


/*** Alternate MACROS ***/

#define CH5 AUX-1
#define CH6 AUX-2 
#define CH7 AUX-3
#define CH8 AUX-4
#define CH4 YAW 5
#define CH2 PITCH
#define CH1 ROLL
#define CH3 THROTTLE

/** H,M,L for Channels **/
#define HIGH_CH1 1000
#define MID_CH1 (HIGH_CH1 + LOW_CH1) / 2
#define LOW_CH1 3000

#define HIGH_CH2 1000
#define MID_CH2 (HIGH_CH2 + LOW_CH2) / 2
#define LOW_CH2 3000

#define HIGH_CH3 1000
#define MID_CH3 (HIGH_CH3 + LOW_CH3) / 2
#define LOW_CH3 3000

#define HIGH_CH4 1000
#define MID_CH4 (HIGH_CH4 + LOW_CH4) / 2
#define LOW_CH4 3000

#define HIGH_CH5 1000
#define MID_CH5 (HIGH_CH5 + LOW_CH5) / 2
#define LOW_CH5 3000

#define ROUNDING_BASE 50


/** Reciever settings **/

#define SERIAL_BAUD 38400
#define RECEIVER_TIMEOUT 1500 // 1.5s
#define MIN_RECEIVER_VALUE 0
#define MAX_RECEIVER_VALUE 250


/** Flight parameters **/

#define PITCH_MIN -30
#define PITCH_MAX 30
#define ROLL_MIN -30
#define ROLL_MAX 30
#define YAW_MIN -180
#define YAW_MAX 180
#define PID_PITCH_INFLUENCE 20
#define PID_ROLL_INFLUENCE 20
#define PID_YAW_INFLUENCE 20




/* Interrupt lock
 *
 */
 
boolean interruptLock = false;

/** RC variables **/

float ch1, ch2, ch3, ch4, ch5;         // RC channel inputs

const unsigned int defaultPulseWidths[CHANNELS] = {
  MIN_PULSE_TIME,      // Throttle
  HALF_PULSE_TIME,     // Roll
  HALF_PULSE_TIME,     // Pitch
  MIN_PULSE_TIME,      // Yaw
  
  MAX_PULSE_TIME,     // AUX1 (MODE in Aeroquad)
  MAX_PULSE_TIME,     // AUX2 (ALTITUDE in Aeroquad)
  MAX_PULSE_TIME,     // AUX3
  MAX_PULSE_TIME      // AUX4
};


/** Motor control variables **/

int velocity;                          // global velocity

float bal_ac, bal_bd;                 // motor balances can vary between -100 & 100
float bal_axes;                       // throttle balance between axes -100:ac , +100:bd

int va, vb, vc, vd;                    //velocities
int v_ac, v_bd;                        // velocity of axes

Servo a,b,c,d;

/*  PID variables
 *
 */

PID pitchReg(&ypr[1], &bal_bd, &ch2, PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL, REVERSE);
PID rollReg(&ypr[2], &bal_ac, &ch1, ROLL_P_VAL, ROLL_I_VAL, ROLL_D_VAL, REVERSE);
PID yawReg(&ypr[0], &bal_axes, &ch4, YAW_P_VAL, YAW_I_VAL, YAW_D_VAL, DIRECT);


/*  Filter variables
 *
 */
 
float ch1Last, ch2Last, ch4Last, velocityLast;

/*  Setup function
 *
 */

void setup(){
  
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_PPM, OUTPUT);

  Serial.begin(38400);
  
  setDefaultPulseWidths();
  
  // Start timer with sync pulse
  Timer1.initialize(SYNC_PULSE_TIME);
  Timer1.attachInterrupt(isr_sendPulses);
  isr_sendPulses();

}


void loop(){
  
  while(Interrupt && fifoCount < packetSize){
     
    /*gap*/
      
  }
  
  getYPR();                          
  computePID();
  calculateVelocities();
  updateMotors(
);
  
}



/*  calculateVelocities function
 *  
 *  calculates the velocities of every motor
 *  using the PID output
 */

void calculateVelocities(){

  acquireLock();

  ch3 = floor(ch3/ROUNDING_BASE)*ROUNDING_BASE;
  velocity = map(ch3, LOW_CH3, HIGH_CH3, ESC_MIN, ESC_MAX);
  
  releaseLock();

  if((velocity < ESC_MIN) || (velocity > ESC_MAX)) velocity = velocityLast;
  
  velocityLast = velocity;
  
  v_ac = (abs(-100+bal_axes)/100)*velocity;
  v_bd = ((100+bal_axes)/100)*velocity;
  
  va = ((100+bal_ac)/100)*v_ac;
  vb = ((100+bal_bd)/100)*v_bd;
  
  vc = (abs((-100+bal_ac)/100))*v_ac;
  vd = (abs((-100+bal_bd)/100))*v_bd;
  
  Serial.println(bal_bd);
  
  if(velocity < ESC_TAKEOFF_OFFSET){
  
    va = ESC_MIN;
    vb = ESC_MIN;
    vc = ESC_MIN;
    vd = ESC_MIN;
  
  }
  
}


inline void initRC(){
  pinMode(PWR, OUTPUT);
  digitalWrite(PWR, HIGH);
  
  // FIVE FUCKING INTERRUPTS !!!
  PCintPort::attachInterrupt(1, rcInterrupt1, CHANGE);
  PCintPort::attachInterrupt(2, rcInterrupt2, CHANGE);
  PCintPort::attachInterrupt(3, rcInterrupt3, CHANGE);
  PCintPort::attachInterrupt(4, rcInterrupt4, CHANGE);
  PCintPort::attachInterrupt(5, rcInterrupt5, CHANGE);
  
}



inline void initRegulators(){

  pitchReg.SetMode(AUTOMATIC);
  pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
  
  rollReg.SetMode(AUTOMATIC);
  rollReg.SetOutputLimits(-PID_ROLL_INFLUENCE, PID_ROLL_INFLUENCE);
  
  yawReg.SetMode(AUTOMATIC);
  yawReg.SetOutputLimits(-PID_YAW_INFLUENCE, PID_YAW_INFLUENCE);

}

inline void rcInterrupt1(){
   if(!interruptLock) ch1 = micros() - rcLastChange1;
   rcLastChange1 = micros(); 
}

inline void rcInterrupt2(){
  if(!interruptLock) ch2 = micros() - rcLastChange2;
  rcLastChange2 = micros();
}

inline void rcInterrupt3(){
  if(!interruptLock) ch3 = micros() - rcLastChange3;
  rcLastChange3 = micros();
}

inline void rcInterrupt4(){
  if(!interruptLock) ch4 = micros() - rcLastChange4;
  rcLastChange4 = micros();
}

inline void rcInterrupt5(){
  if(!interruptLock) ch5 = micros() - rcLastChange5;
  rcLastChange5 = micros();
}

inline void acquireLock(){
  interruptLock = true; 
}

inline void releaseLock(){
  interruptLock = false;
}

void handleSerial() {
  // Handle Serial Data
  if (Serial.available()) {
    lastReceived = millis();
    currentByte = Serial.read();

    if (currentByte == 254) {
      // Either packet is done, or we got corrupt data. Reset the packet
      bytesReceived = 0;
    } else {
      buffer[bytesReceived] = currentByte;
      bytesReceived++;
    }

    if (bytesReceived == CHANNELS) {
      bytesReceived = 0;
      armed = true;

      // Convert char (0-250) to pulse width (1000-2000)
      for (int i=0; i<CHANNELS; i++) {
        pulseWidths[i] = map(buffer[i], MIN_RECEIVER_VALUE, MAX_RECEIVER_VALUE, 
                                        MIN_PULSE_TIME, MAX_PULSE_TIME);
      }
    }
  }
}

#endif


