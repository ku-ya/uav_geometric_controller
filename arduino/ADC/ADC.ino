/*
 * rosserial ADC Example
 *
 * This is a poor man's Oscilloscope.  It does not have the sampling
 * rate or accuracy of a commerical scope, but it is great to get
 * an analog value into ROS in a pinch.
 */
#define USE_USBCON
#include "Wire.h"
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <rosserial_arduino/Adc.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

bool test = false;
int x;

void message( const std_msgs::Bool& cmd_val)
{
  test = cmd_val.data;
//  Wire.beginTransmission(0x29);
//  Wire.write(x);
//  Wire.endTransmission(); 
}

rosserial_arduino::Adc adc_msg;
rosserial_arduino::Adc cmd_msg;
ros::Publisher p("adc", &adc_msg);
ros::Publisher pub("cmd_ard", &cmd_msg);
ros::Subscriber<std_msgs::Bool> sub("cmd", &message );


void setup()
{
  Wire.begin();    
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(p);
  nh.advertise(pub);
}


//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

long adc_timer;

void loop()
{
   if(test){
    x = 25;
   }else{
    x = 0;
   }

  Wire.beginTransmission(0x29);
  Wire.write(x);
  Wire.endTransmission(); 
  adc_msg.adc0 = averageAnalog(0);
  cmd_msg.adc0 = x;
  p.publish(&adc_msg);
  pub.publish(&cmd_msg);
  nh.spinOnce();
  delay(10);
}
