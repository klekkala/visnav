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



/****  Arduino Pin configuration
   **ESP8266(ESP-07) Wifi Serial Adapter
	5V  <--> VCC
	5V  <--> CH_PD
	GND <--> GND
	D13 <--> TX 
	D12 <--> RX

   **Multiwii Flight Controller
	D11 <--> 33(CAM PITCH)
	D10 <--> 32(CAM ROLL)  
	D09 <--> A15(AUX-4) 
	D08 <--> A14(AUX-3)
	D07 <--> A13(AUX-2)
	D06 <--> A12(AUX-1)
	D05 <--> A11(YAW)
	D04 <--> A10(PITCH)
	D03 <--> A09(ROLL)
	D02 <--> A08(THROTTLE)

   **USB to Raspberry
*/

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



#define RC_HIGH_CH1 1000
#define RC_LOW_CH1 2000
#define RC_HIGH_CH2 1000
#define RC_LOW_CH2 2000
#define RC_HIGH_CH3 1000
#define RC_LOW_CH3 2000
#define RC_HIGH_CH4 1000
#define RC_LOW_CH4 2000
#define RC_HIGH_CH5 1000
#define RC_LOW_CH5 2000
#define RC_ROUNDING_BASE 50


// Set these a little high since they are registering low on AQ
#define MIN_PULSE_TIME 1030   // 1000us
#define MAX_PULSE_TIME 1990   // 2000us
#define HALF_PULSE_TIME (MIN_PULSE_TIME + MAX_PULSE_TIME) / 2
#define SYNC_PULSE_TIME 3050  // 3000us

#define SERIAL_BAUD 38400

#define PIN_LED 13
#define PIN_PPM 9

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
  
  initRC();                            // Self explaining
  initESCs();
  initBalancing();
  initRegulators();
  
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

  ch3 = floor(ch3/RC_ROUNDING_BASE)*RC_ROUNDING_BASE;
  velocity = map(ch3, RC_LOW_CH3, RC_HIGH_CH3, ESC_MIN, ESC_MAX);
  
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
  pinMode(RC_PWR, OUTPUT);
  digitalWrite(RC_PWR, HIGH);
  
  // FIVE FUCKING INTERRUPTS !!!
  PCintPort::attachInterrupt(RC_1, rcInterrupt1, CHANGE);
  PCintPort::attachInterrupt(RC_2, rcInterrupt2, CHANGE);
  PCintPort::attachInterrupt(RC_3, rcInterrupt3, CHANGE);
  PCintPort::attachInterrupt(RC_4, rcInterrupt4, CHANGE);
  PCintPort::attachInterrupt(RC_5, rcInterrupt5, CHANGE);
  
}

void initMPU(){
  
  Wire.begin();
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if(devStatus == 0){
  
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    
  }
}

inline void initESCs(){

  a.attach(ESC_A);
  b.attach(ESC_B);
  c.attach(ESC_C);
  d.attach(ESC_D);
  
  delay(100);
  
  arm();

}

inline void initBalancing(){

  bal_axes = 0;
  bal_ac = 0;
  bal_bd = 0;

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

#endif


