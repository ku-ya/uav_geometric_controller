/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/
#include "Wire.h"

bool on_flag = false, mode = false, s_mode = false;

double cmd = 20;
int cmd_out = 20;
int inPin0 = 22,inPin1 = 24,inPin2 = 26, k = 1;
int val = 0, count = 0;
int send_cmd, sensorValue;
String str_out,str_out2;



// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  Wire.begin();
  pinMode(13, OUTPUT);
  pinMode(inPin0, INPUT);
  pinMode(inPin1, INPUT);
  pinMode(inPin2, INPUT);
}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<3; i++) v+= analogRead(pin);
  return v/3;
}

void send_command(int cmd){
      Wire.beginTransmission(0x29);
      Wire.write(cmd);
      Wire.endTransmission();
}

void send_serial(int msg0){
    sensorValue = averageAnalog(0);
    str_out = "cmd: "+ (String)msg0;
    str_out2 = " Vin: "+ (String)sensorValue;
    Serial.println(str_out + str_out2 + '\r');
  }

// the loop routine runs over and over again forever:
void loop() {
  on_flag = (digitalRead(inPin0)==LOW);
  mode = (digitalRead(inPin1)==LOW);
  s_mode = digitalRead(inPin2);
 
  if(on_flag){
    if(mode){
      double sec = 20;
      while(!digitalRead(inPin0) && cmd < 180){
        cmd_out = (int)cmd;
        send_command(cmd_out);
        send_serial(cmd_out);
        cmd = cmd + 180.0/(sec/0.01);
        delay(10);
      }
      cmd = 20;
      delay(2000);
    }else{
      if(digitalRead(inPin2)){k = 5;}else{k = 1;}
      for(int i = 0; i<k;i++){
      while(!digitalRead(inPin0) && count < 300){
        send_command(cmd_out);
        send_serial(cmd_out);
        delay(10);
        count = count + 1;
       }
       count = 0;
       delay(2000);
      }
      cmd_out = cmd_out + 10;
      
    }

  }else{
    cmd_out = 20;
    send_command(0);
    Serial.println("Motor off, Mode: "+(String)mode+", Repeat: "+(String)s_mode);
    delay(100);
  }
}
