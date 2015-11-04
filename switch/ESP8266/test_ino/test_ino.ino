#include 
SoftwareSerial softSerial(2, 13); // RX, TX
String WSSID = "VISNAV";
String WPASS = "visnav";
bool r;

void setup() {
/* SETUP SERIAL COMMUNICATION */
espSerialSetup();
delay(2000); // Without this delay, sometimes, the program will not start until Serial Monitor is connected
r = espSendCommand( "AT+CIFSR" , "OK" , 5000 );
if( !r ) {
r = espSendCommand( "AT+CWMODE=1" , "OK" , 5000 );
r = espSendCommand( "AT+CWJAP=\""+WSSID+"\",\""+WPASS+"\"" , "OK" , 15000 );
}
}

void loop(){

r = espSendCommand( "AT+CIPSTART=\"TCP\",\"108.59.11.102\",80" , "OK" , 5000 );
String getRequest = "GET /shiznic/ HTTP/1.1\r\nHost: www.rootpower.com\r\n";
int getRequestLength = getRequest.length() + 2; // add 2 because \r\n will be appended by SoftwareSerial.println().
r = espSendCommand( "AT+CIPSEND=" + String(getRequestLength) , "OK" , 5000 );
r = espSendCommand( getRequest , "+IPD" , 15000 );
//r = espSendCommand( getRequest , "world" , 15000 ); // page should respond with Hello, world.
if( !r ) {
Serial.println( "Something wrong...Attempting reset...");
espSendCommand( "AT+RST" , "ready" , 20000);
espSendCommand( "AT+CWMODE=1" , "OK" , 5000 );
espSendCommand( "AT+CWJAP=\""+WSSID+"\",\""+WPASS+"\"" , "OK" , 15000 );
}

delay(3000);

}

void espSerialSetup() {

// change baud rate so SoftwareSerial works better -- need a certain version of ESP for this to work and be retained after power cycle
// make sure Serial Monitor port speed is set to 9600
softSerial.begin(115200); // default baud rate for ESP8266
delay(1000);
softSerial.println("AT+CIOBAUD=9600");
delay(1000);
softSerial.begin(9600);
Serial.begin(9600);

}

bool espSendCommand(String cmd, String goodResponse, unsigned long timeout) {
Serial.println("espSendCommand( " + cmd + " , " + goodResponse + " , " + String(timeout) + " )" );
softSerial.println(cmd);
unsigned long tnow = millis();
unsigned long tstart = millis();
unsigned long execTime = 0;
String response = "";
char c;
while( true ) {
if( tnow > tstart + timeout ) {
Serial.println("espSendCommand: FAILED - Timeout exceeded " + String(timeout) + " seconds" );
if( response.length() > 0 ) {
Serial.println("espSendCommand: RESPONSE:");
Serial.println( response );
} else {
Serial.println("espSendCommand: NO RESPONSE");
}
return false;
}
c = softSerial.read();
if( c >= 0 ) {
response += String(c);
if( response.indexOf(goodResponse) >= 0 ) {

execTime = ( millis() - tstart );
Serial.println("espSendCommand: SUCCESS - Response time: " + String(execTime) + "ms");
Serial.println("espSendCommand: RESPONSE:");
Serial.println(response);
while(softSerial.available() > 0) {
Serial.write(softSerial.read());
}
return true;
}
}
tnow = millis();
}
}

