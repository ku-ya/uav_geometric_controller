/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/
#include "Wire.h"

bool on_flag = 0;
int mode;
double cmd = 20;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Wire.begin();
  pinMode(13, OUTPUT);
}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<5; i++) v+= analogRead(pin);
  return v/5;
}

// the loop routine runs over and over again forever:
void loop() {
//  if(on_flag){
    if(cmd < 180){
      cmd = cmd + 0.1;
    }else{
      delay(5000);
      cmd = 20;
    }
    Wire.beginTransmission(0x29);
    Wire.write((int) cmd);
    Wire.endTransmission();
//  }
  int send_cmd = (int)cmd;
  int sensorValue = averageAnalog(0);
  String str_out = "cmd: "+ (String)send_cmd;
  String str_out2 = " Vin: "+ (String)sensorValue;
  Serial.println(str_out + str_out2 + '\r');
  delay(10);        // delay in between reads for stability
}
