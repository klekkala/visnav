/* ====== esp8266 HelloWorld Demo ======
 * Print out analog values
 * (Updated Dec 14, 2014)
 * ==========================
 *
 * Change SSID and PASS to match your WiFi settings.
 * The IP address is displayed to soft serial upon successful connection.
 *
 * Ray Wang @ Rayshobby LLC
 * http://rayshobby.net/?p=9734
 */

//Leonardo
//Serial_ & dbgTerminal = Serial;
//HardwareSerial & espSerial = Serial1;

////UNO & M328P

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

#include <SoftwareSerial.h>
SoftwareSerial dbgTerminal(2, 13); // RX, TX
HardwareSerial & espSerial = Serial;


#define BUFFER_SIZE 512

#define SSID  "ESP8266"      // change this to match your WiFi SSID
#define PASS  "qwertyuiop"  // change this to match your WiFi password
#define PORT_SERVER  "8080"           // using port 8080 by default

char buffer[BUFFER_SIZE];

// By default we are looking for OK\r\n
char OKrn[] = "OK\r\n";
byte wait_for_espSerial_respSerialonse(int timeout, char* term = OKrn) {
  unsigned long t = millis();
  bool found = false;
  int i = 0;
  int len = strlen(term);
  // wait for at most timeout milliseconds
  // or if OK\r\n is found
  while (millis() < t + timeout) {
    if (espSerial.available()) {
      buffer[i++] = espSerial.read();
      if (i >= len) {
        if (strncmp(buffer + i - len, term, len) == 0) {
          found = true;
          break;
        }
      }
    }
  }
  buffer[i] = 0;
  dbgTerminal.print(buffer);
  return found;
}

void setup() {
  espSerial.begin(9600);
  dbgTerminal.begin(9600);
  dbgTerminal.println("Begin...");
  setupWiFi();
  // print device IP address
  dbgTerminal.print("Device IP Address:");
  espSerial.println("AT+CIFSR");
  wait_for_espSerial_respSerialonse(1000);
}

bool read_till_eol() {
  static int i = 0;
  if (espSerial.available()) {
    buffer[i++] = espSerial.read();
    if (i == BUFFER_SIZE)  i = 0;
    if (i > 1 && buffer[i - 2] == 13 && buffer[i - 1] == 10) {
      buffer[i] = 0;
      i = 0;
      dbgTerminal.print(buffer);
      return true;
    }
  }
  return false;
}

void loop() {
  int ch_id, packet_len;
  char *pb;
  if (read_till_eol()) {
    if (strncmp(buffer, "+IPD,", 5) == 0) {
      // request: +IPD,ch,len:data
      sscanf(buffer + 5, "%d,%d", &ch_id, &packet_len);
      if (packet_len > 0) {
        // read serial until packet_len character received
        // start from :
        pb = buffer + 5;
        while (*pb != ':') pb++;
        pb++;
        if (strncmp(pb, "GET /", 5) == 0) {
          wait_for_espSerial_respSerialonse(1000);
          dbgTerminal.println("-> serve homepage");
          serve_homepage(ch_id);
        }
      }
    }
  }
}
void serve_homepage(int ch_id) {
  String header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\n";
  String content = "";
  content += "Hello World!!!";

  header += "Content-Length:";
  header += (int)(content.length());
  header += "\r\n\r\n";
  espSerial.print("AT+CIPSEND=");
  espSerial.print(ch_id);
  espSerial.print(",");
  espSerial.println(header.length() + content.length());
  if (wait_for_espSerial_respSerialonse(2000, "> ")) {
    espSerial.print(header);
    espSerial.print(content);
  } else {
    espSerial.print("AT+CIPCLOSE=");
    espSerial.println(ch_id);
  }
}


void setupWiFi() {
  // try empty AT command
  espSerial.println("AT");
  wait_for_espSerial_respSerialonse(1000);

  // set mode 1 (client)
  espSerial.println("AT+CWMODE=1");
  wait_for_espSerial_respSerialonse(1000);

  // reset WiFi module
  espSerial.print("AT+RST\r\n");
  wait_for_espSerial_respSerialonse(1500);

  // join AP
  espSerial.print("AT+CWJAP=\"");
  espSerial.print(SSID);
  espSerial.print("\",\"");
  espSerial.print(PASS);
  espSerial.println("\"");
  // this may take a while, so wait for 5 seconds
  wait_for_espSerial_respSerialonse(5000);

  espSerial.println("AT+CIPSTO=30");
  wait_for_espSerial_respSerialonse(1000);

  // start server
  espSerial.println("AT+CIPMUX=1");
  wait_for_espSerial_respSerialonse(1000);

  espSerial.print("AT+CIPSERVER=1,"); // turn on TCP service
  espSerial.println(PORT_SERVER);
  wait_for_espSerial_respSerialonse(1000);

}

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
#define CH4 YAW
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

/** Channels **/
#define CHANNELS 8

boolean interruptLock = false;


const unsigned int defaultPulseWidths[CHANNELS] = {
  MIN_PULSE_TIME,      // Throttle
  HALF_PULSE_TIME,     // Roll
  HALF_PULSE_TIME,     // Pitch
  MIN_PULSE_TIME,      // Yaw
  
  MAX_PULSE_TIME,     // AUX1 
  MAX_PULSE_TIME,     // AUX2 
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

/*  PID variables and regulators: Open a socket and get the variables for the configuration app
 *
 */

 
float ch1Last, ch2Last, ch4Last, velocityLast;

/** Setup function **/
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
  pin_pwm();
  
}



/** calculate Velocities function **/

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


inline void acquireLock(){
  interruptLock = true; 
}

inline void releaseLock(){
  interruptLock = false;
}

/** Function to handle serial data **/
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
