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
ros::NodeHandle nh;

int x;
void message( const std_msgs::Float64& cmd_val)
{
  x = cmd_val.data;
  Wire.beginTransmission(0x29);
  Wire.write(x);
  Wire.endTransmission(); 
}

rosserial_arduino::Adc adc_msg;
ros::Publisher p("adc", &adc_msg);
ros::Subscriber<std_msgs::Float64> sub("cmd", &message );


void setup()
{
  Wire.begin();    
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(p);
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
  
//  Wire.beginTransmission(0x29);
//  Wire.write(20);
//  Wire.endTransmission();  
  
  adc_msg.adc0 = averageAnalog(0);
  p.publish(&adc_msg);

  nh.spinOnce();
}
