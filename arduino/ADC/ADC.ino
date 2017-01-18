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
#include <ros/time.h>
#include <std_msgs/Time.h>
#include <rosserial_arduino/Adc.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;

bool test = false;
int x= 0;

void message( const std_msgs::Bool& cmd_val)
{
  test = cmd_val.data;
//  Wire.beginTransmission(0x29);
//  Wire.write(x);
//  Wire.endTransmission();
}


std_msgs::Time timer;
rosserial_arduino::Adc adc_msg;
ros::Publisher pub("adc", &adc_msg);
ros::Subscriber<std_msgs::Bool> sub("cmd", &message );

void setup()
{
  Wire.begin();
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
  timer.data = nh.now();
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
  adc_msg.adc1 = x;
  pub.publish(&adc_msg);

  nh.spinOnce();
  delay(10);
}
