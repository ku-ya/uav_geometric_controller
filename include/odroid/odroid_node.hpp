#ifndef ODROID_NODE_H
#define ODROID_NODE_H
// System header files (gcc compiler on ODROID)
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <math.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <termios.h>
#include <stdint.h>
#include <stdbool.h>

#include "AUX_Functions.h"
#include "Controllers.h"
#include "Gains.h"
#include "serial1.h"
#include "serial2.h"
#include "Xbee.h"
// #include "PCA9685.h"
#include "ForcesTorques_i2c.h"
#include "char_parse.h"
// #include "VN100/include/vectornav.h"
// #include "AUX_Global.h"
#include <errno.h>
// #include <wiringPiI2C.h>
#include <linux/i2c-dev.h>

#include <iostream>
#include <eigen3/Eigen/Dense>
using namespace Eigen;

// 4 Threads: Keypad, Vicon data, Controller
#define NUM_THREADS 3

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
double t_IMU, del_t_IMU = 0, t_prev_IMU;// I2C thread
double t_v, del_t_v, t_prev_v;// Vicon thread
double t_CADS, del_t_CADS, t_prev_CADS;// Control thread
double Speed_Vicon_Thread, Speed_IMU_Thread, Speed_CADS_Thread;
double RC_vel = 0.0139;
double RC_IMU = 0.0054;

// Intermediate Variables (Vicon to inertial, misc.)
double R_s0b[3][3];// Body (b) to initial sensor attitude (s0)
double R_eb_s[3][3];// Body (b) to inertial (e) calculated by IMU
double R_vm[3][3];// Markers (m) to Vicon (v)
double R_em[3][3];// Markers (m) to inertial (e)
double R_ev[3][3]={{1.0  ,  0.0  ,  0.0},
{0.0  , -1.0  ,  0.0},
{0.0  ,  0.0  , -1.0}};// Vicon frame (v) to inertial frame (e) (fixed)

double R_bm[3][3]={{1.0  ,  0.0  ,  0.0},
{0.0  , -1.0  ,  0.0},
{0.0  ,  0.0  , -1.0}};// Markers frame (m) to the body frame (b) (fixed)

// Measured Values for Controller
double x_e[3], v_e[3];// Position and Velocity in inertial (e) frame
double R_eb[3][3];// Body (b) to inertial (e) frame
double E_angles_save[3];

// Error Functions
double eX[3], eV[3], eR[3], eW[3];
// Integral errors begin at zero
double eiX[3] = {0, 0, 0};
double eiR[3] = {0, 0, 0};

// Output of Control_Nonlinear() and Command Execution
double f[6];// Force of each motor/propeller/servo
int mtr_addr[6]={41, 42, 43, 44, 45, 46};// Motor addresses 1-6
int thr[6];// i2c motor commands
int servo_addr[6]={0, 1, 2, 3, 4, 5};// Servo addresses 1-6
uint16_t servopl[6];// i2c servo pulse length (duty_cycle[i] = servopl[i]/4095 @ ~325 Hz)
uint16_t zp[6] = {1300, 1285, 1230, 1280, 1215, 1275};

// Calibrating and Temporary Saving IMU Data
double phi_bias, theta_bias, psi_bias, W1_bias, W2_bias, W3_bias;
double W_b_[3], W_b_noisy[3];// Angular velocity of previous loop
double EAngles_imu_[3];// Euler Angles of previous loop
// double EAngles_imu[3];


double x_e_init[3], R_eb_init[3][3];// Initial position and attitude
double x_ss[3], x_ss_[3]; // Steady state error used for landing correction

//TCP Communication
int sockfd, newsockfd, port_number, n, count = 0;
socklen_t client_ln;
char buffer[1000], buffer_[1000];
struct sockaddr_in serv_addr, cli_addr;

int IMU_delay_for_CAD;


// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

// #include "ros/ros.h"
#include <sensor_msgs/Imu.h>
// #include <iomanip>
// #include <iostream>
// Custom message includes. Auto-generated from msg/ directory.
// #include "node_example/node_example_data.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
// #include <node_example/node_example_paramsConfig.h>

using std::string;

class odroid_node
{
private:
  double m, g, J[3][3];//mass, gravity, inertia
  Matrix3f Je;
  // Desired Variables
  // Position:
  double xd[3], xd_dot[3], xd_ddot[3];
  // Attitude:
  double Rd[3][3],  Wd[3],  Wd_dot[3];
  double W_b[3];
  // Gains
  double kx, kv, kiX, c1, kR, kW, kiR, c2;
  float phi, theta, psi, W_raw[3];
  Matrix2d e; //inertial frame,
  // Measured Values in Vicon Frame
  double x_v[3];// position in the Vicon frame
  double quat_vm[4];// attitude of markers measured by Vicon system

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
  void imu_Callback(const sensor_msgs::Imu::ConstPtr& msg);

  void key_callback();

  void ctl_callback();

  void vicon_callback();

  void attitude_controller(double Rd[3][3], double Wd[3], double Wddot[3],
        double W[3], double R[3][3], double del_t, double eiR_last[3],
        double eR[3], double eW[3], double eiR[3],
        double kR, double kW, double kiR_now, double f[6]);
      // Control_Nonliner Inputs:
      //  xd = f(t) desired position in inertial frame
      //  xd_dot = f(xd) desired velocity in inertial frame
      //  xd_ddot = f(xd) desired acceleration in inertial frame
      //  Rd = f(t) desired attitude (usually using local coordinates)
      //  Wd = f(Rd) desired angular velocity in the body-fixed frame
      //  Wd_dot = f(R) desired angular acceleration in body-fixed frame
      //  x_e = measured position by Vicon converted to inerial frame
      //  Vel_Inerial = estimated velocity (low-pass filter) in inertial frame
      //  W_b = angular velocity in the body-fixed frame
      //  R_eb = rotation matrix - linear transformation of body -> inertial frames
      //  kx = proportional position control gain
      //  kv = derivative position control gain
      //  kiX = integral position control gain
      //  kR = proportional attitude control gain
      //  kW = derivative attitude control gain
      //  kiR = integral attitude control gain
      //  m = body mass
      //  g = acceleration due to gravity
      //  J = moment of inertial matrix
      // Control_Nonlinear Outputs:
      //  eX = position error in inertial frame
      //  eV = velocity error in inertial frame
      //  eR = attitude error between R_eb and Rd
      //  eW = angular velocity error in body-fixed frame
      //  f = force requirement for each motor/prop
      // Control_Nonlinear Inputs & Outputs:
      //  eiX = integral position error
      //  eiR = integral attitude error
      void print_J();

};

#endif
