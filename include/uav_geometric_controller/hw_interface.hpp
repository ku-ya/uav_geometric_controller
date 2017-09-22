#ifndef HW_INTERFACE_H
#define HW_INTERFACE_H

#include "math.hpp"
#include <eigen3/Eigen/Dense>
// #include <odroid/node.hpp>
// #include <odroid/error.h>
#include <termios.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */
// Adafruit PCA9685 Servo Informatio
#define i2c_addr 0x40

#include <std_msgs/Int8MultiArray.h>
class hw_interface
{
public:
  hw_interface(const char * port, std::vector<int> address);
  ~hw_interface();

    // Open ports to I2C (motors)
  int fhi2c;
  int  mtr_addr[4]; //= {41, 42, 43, 44};// Motor addresses 1-6
  int thr[4];// i2c motor commands
  int servo_addr[4] = {41,42,43,44}; //= {0, 1, 2, 3, 4, 5};// Servo addresses 1-6
  const char * port;
  uint16_t servopl[6];// i2c servo pulse length (duty_cycle[i] = servopl[i]/4095 @ ~325 Hz)
  uint16_t zp[6] = {1300, 1285, 1230, 1280, 1215, 1275};

  uint8_t* motor_command(int thr[4], bool MotorWarmup, bool MOTOR_ON);
  void open_I2C();
};

#endif
