

/*
  Basic console passthru for Quectel BG96 cellular modem.
  Created by Johjn Cobb  (hack3d), November 123, 2018.
*/

#if ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
    #include "pins.h"    
#endif

#include <SoftwareSerial.h>

SoftwareSerial BG96_AT(10,9); // RX, TX - 9600 baud rate 

#define USER_BUTTON 6
#define USER_LED 5
#define BG96_ENABLE 4
#define ALS_PT19_PIN A1
#define RELAY A0
#define BG96_POWERKEY A2 
#define STATUS A3 
#define AP_READY 7
#define RING_INDICATOR 8

/*
 * set echo off: ATE0
 * err loglvl 2: AT+CMEE=2
 * get sim stat: AT+QSIMSTAT?
 * check sim:    AT+CPIN?
 * operator:     AT+COPS?
 * get net reg:  AT+CREG?
 * connect apn:  AT+QICSGP=1,1,"hologram", "", "", 1
 * activate ctx: AT+QIACT=1
 * query ctx:    AT+QIACT?
 * disconnect:   AT+QIDEACT=1
 * tcp connect:  AT+QIOPEN=1,1,"TCP","220.180.239.201",8713,0,0
 * power down:   AT+QPOWD
 
 */



// baud rate used for both Serial ports
unsigned long baud = 115200;

char consoleInput[256];
uint8_t inputIndex = 0;

void initModem() {

  memset(consoleInput, 0, 256);
  pinMode(USER_LED, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(USER_BUTTON, INPUT);
  pinMode(ALS_PT19_PIN, INPUT);
  
  enable();
  
  while(getModemStatus()){
    Serial.println(getModemStatus());
    powerUp();  
    Serial.println(getModemStatus());
  }

  // setting serials
  BG96_AT.begin(9600);
}

void enable() {
  pinMode(BG96_ENABLE, OUTPUT);
  digitalWrite(BG96_ENABLE,HIGH);
}

void powerUp() {
  pinMode(BG96_POWERKEY,OUTPUT);
  delay(10);
  digitalWrite(BG96_POWERKEY,HIGH);
  delay(500);
  digitalWrite(BG96_POWERKEY,LOW);  
}

void setup() {

  initModem();
  Serial.begin(baud);
}

uint8_t getModemStatus() {
  pinMode(STATUS,INPUT);
  delay(10);
  return digitalRead(STATUS);  
  
}

void modemSend(uint8_t cmd) {
  BG96_AT.flush();
  BG96_AT.print(cmd);
}


void loop() {
  if (Serial.available()) {
    
    char input = Serial.read();

    if (input == '\n' || input == '\r') {
      BG96_AT.print(consoleInput);
      BG96_AT.print('\r');
      memset(consoleInput, 0, 256);
      inputIndex = 0;
    } else {
      consoleInput[inputIndex++] = input;
    }

  }

  if (BG96_AT.available()) {
    Serial.write(BG96_AT.read());
  }
}
