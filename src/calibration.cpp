#include <ros/ros.h>
#include <rosserial_arduino/Adc.h>
#include <dynamic_reconfigure/server.h>
#include <odroid/calibrationConfig.h>

int mode, on_off, cmd;

void callback(odroid::calibrationConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %d, %d, %d",
            config.double_cmd, config.on_off, config.mode);
  mode = config.mode;
  on_off = config.on_off;
  cmd = config.double_cmd;
}

void adcCallback(const rosserial_arduino::Adc msg)
{
  ROS_INFO("DAQ value: voltage[%d] cmd[%d]", msg.adc0, msg.adc1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "calibration_config");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<rosserial_arduino::Adc>("cmd", 100);
  ros::Subscriber sub = n.subscribe("/adc", 100, adcCallback);
  rosserial_arduino::Adc msg;

  dynamic_reconfigure::Server<odroid::calibrationConfig> server;
  dynamic_reconfigure::Server<odroid::calibrationConfig>::CallbackType f;

  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  ROS_INFO("Spinning node");
  ros::Rate loop_rate(10);

  while (ros::ok()){

  msg.adc0 = on_off;
  msg.adc1 = cmd;
  msg.adc2 = mode;
  pub.publish(msg);
  ros::spinOnce();
  loop_rate.sleep();
  }
  return 0;
}
