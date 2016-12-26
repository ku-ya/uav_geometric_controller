#ifndef ODROID_NODE_H
#define ODROID_NODE_H
// System header files (gcc compiler on ODROID)
#include <math.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <termios.h>

#include "AUX_Functions.h"
#include "Controllers.h"
#include "Gains.h"
// #include "serial1.h"
// #include "serial2.h"
// #include "Xbee.h"
// #include "PCA9685.h"
#include "ForcesTorques_i2c.h"
#include <linux/i2c-dev.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

using namespace Eigen;

// Pi constant and IMU offset
#define PI 3.14159265359

// Adafruit PCA9685 Servo Informatio
#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4
#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9
#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD
#define i2c_addr 0x40

#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
// #include <iomanip>
// #include <iostream>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <odroid/GainsConfig.h>

class odroid_node
{
private:
  //  m = body mass
  //  g = acceleration due to gravity
  //  J = moment of inertial matrix
  double m, g; Matrix3d J;
  // Control_Nonliner Inputs:
  //  xd = f(t) desired position in inertial frame
  //  xd_dot = f(xd) desired velocity in inertial frame
  //  xd_ddot = f(xd) desired acceleration in inertial frame
  Vector3d xd, xd_dot, xd_ddot;
  //  Rd = f(t) desired attitude (usually using local coordinates)
  //  Wd = f(Rd) desired angular velocity in the body-fixed frae
  //  Wd_dot = f(R) desired angular acceleration in body-fixed frame
  //  x_e = measured position by Vicon converted to inerial frame
  //  Vel_Inerial = estimated velocity (low-pass filter) in inertial frame
  //  W_b = angular velocity in the body-fixed frame
  //  R_eb = rotation matrix - linear transformation of body -> inertial frames
  Matrix3d Rd;
  Vector3d Wd, Wd_dot, W_b, W_raw;
  // Gains
  //  kx = proportional position control gain
  //  kv = derivative position control gain
  //  kiX = integral position control gain
  //  kR = proportional attitude control gain
  //  kW = derivative attitude control gain
  //  kiR = integral attitude control gain
  double kx, kv, kiX, c1, kR, kW, kiR, c2;
  float phi, theta, psi;
  Matrix2d e; //inertial frame,
  // Measured Values in Vicon Frame
  Vector3d x_v;// position in the Vicon frame
  double quat_vm[4];// attitude of markers measured by Vicon system

  bool IMU_flag, print_imu, print_f, print_thr;
  VectorXd f;
  Matrix3d R_bm;

  // Integral errors begin at zero
  VectorXd eiX, eiR;
  Eigen::Matrix<double, 6, 6> invFMmat;
  // Eigen::Matrix<int, 6, 1> thr;
  // Calibrating and Temporary Saving IMU Data
  double phi_bias, theta_bias, psi_bias, W1_bias, W2_bias, W3_bias;
  double W_b_[3], W_b_noisy[3];// Angular velocity of previous loop
  double EAngles_imu_[3];// Euler Angles of previous loop
  // double EAngles_imu[3];
  double x_e_init[3], R_eb_init[3][3];// Initial position and attitude
  double x_ss[3], x_ss_[3]; // Steady state error used for landing correction

  double EAngles_imu[3];

  // Threads (except command key) ON/OFF
  bool SYSTEM_RUN = true;
  // Command key thread ON/OFF
  bool Accept_Commands = true;
  // Motors ON/OFF
  bool MOTOR_ON = false;
  // Start Motors at 0 Pitch and 20 Throttle (getting going)
  bool MotorWarmup = true;
  // Special conditions
  bool SphericalJointTest = true;
  // Open ports to I2C (motors)
  int fhi2c;

  // Time stamps
  double t_IMU, del_t_IMU, t_prev_IMU;// I2C thread
  double t_v, del_t_v, t_prev_v;// Vicon thread
  double t_CADS, del_t_CADS, t_prev_CADS;// Control thread
  double Speed_Vicon_Thread, Speed_IMU_Thread, Speed_CADS_Thread;
  double RC_vel = 0.0139 , RC_IMU = 0.0054;


  // Intermediate Variables (V  // Integral errors begin at zero
  // VectorXd eiX, eiR;
  Matrix3d R_s0b;// Body (b) to initial sensor attitude (s0)
  Matrix3d R_eb_s;// Body (b) to inertial (e) calculated by IMU
  Matrix3d R_vm;// Markers (m) to Vicon (v)
  Matrix3d R_em;// Markers (m) to inertial (e)
  Matrix3d R_ev;

  // Measured Values for Controller
  double x_e[3], v_e[3];// Position and Velocity in inertial (e) frame
  Matrix3d R_eb;// Body (b) to inertial (e) frame
  double E_angles_save[3];

  // Error Functions
  Vector3d eX, eV, eR, eW;
  // Control_Nonlinear Outputs:
  //  eX = position error in inertial frame
  //  eV = velocity error in inertial frame
  //  eR = attitude error between R_eb and Rd
  //  eW = angular velocity error in body-fixed frame
  //  f = force requirement for each motor/prop
  // Control_Nonlinear Inputs & Outputs:
  //  eiX = integral position error
  //  eiR = integral attitude error


  // Output of Control_Nonlinear() and Command Execution
  // VectorXd f;// Force of each motor/propeller/servo
  int mtr_addr[6] = {41, 42, 43, 44, 45, 46};;// Motor addresses 1-6
  int thr[6];// i2c motor commands
  int servo_addr[6] = {0, 1, 2, 3, 4, 5};// Servo addresses 1-6
  uint16_t servopl[6];// i2c servo pulse length (duty_cycle[i] = servopl[i]/4095 @ ~325 Hz)
  uint16_t zp[6] = {1300, 1285, 1230, 1280, 1215, 1275};

  //TCP Communication
  int sockfd, newsockfd, port_number, n;
  socklen_t client_ln;
  char buffer[1000], buffer_[1000];
  struct sockaddr_in serv_addr, cli_addr;
public:
  //! Constructor.
  odroid_node();
  //! Destructor.
  ~odroid_node();
  //! Callback function for dynamic reconfigure server.
  // void configCallback(node_example::node_example_paramsConfig &config, uint32_t level);

  //! Publish the message.
  // void publishMessage(ros::Publisher *pub_message);

  //! Callback function for subscriber.
  //! IMU sensor message subscriber
  void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
  //! Keyboard input message subscriber
  void key_callback(const std_msgs::String::ConstPtr& msg);
  //! Controller
  void ctl_callback();
  //! Vicon sensor message subscriber
  void vicon_callback();
  //! dynamic reconfigure callback
  void callback(odroid::GainsConfig &config, uint32_t level);
  //! Controller function
  void GeometricControl_SphericalJoint_3DOF_eigen(Vector3d Wd, Vector3d Wddot, Vector3d W, Matrix3d R, double del_t, VectorXd eiR_last, double kiR_now);

  void GeometricController_6DOF(Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot, Matrix3d Rd, Vector3d Wd, Vector3d Wddot, Vector3d x_e, Vector3d v_e, Vector3d W, Matrix3d R, double del_t,  Vector3d eiX_last, Vector3d eiR_last, double kiX_now, double kiR_now);

  void motor_command();
  void open_I2C();
  void print_J();
  void print_force();
  bool getIMU();
};

#endif
