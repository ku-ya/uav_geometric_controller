#ifndef ODROID_NODE_H
#define ODROID_NODE_H
// System header files (gcc compiler on ODROID)
#include <math.h>
#include "AUX_Functions.h"

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "math.hpp"

#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ApplyBodyWrench.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <odroid/GainsConfig.h>
#include <odroid/hw_interface.hpp>
// #include <odroid/visualize.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <odroid/trajectory.h>
#include <list>
#include <std_msgs/Int8MultiArray.h>
using namespace Eigen;

class odroid_node
{
public:
  boost::mutex mutex_;
  int environment, mode;
  ros::NodeHandle n_;
  ros::NodeHandle nhSub_;

  ros::Publisher pub_;
  ros::Publisher vis_pub_0, vis_pub_1, vis_pub_2, vis_pub_3;
  ros::Time vicon_time, imu_time;
  double dt_vicon = 0, dt_imu = 0, dt_vicon_imu;
  //  m = body mass, g = gravity, J = moment of inertial
  double m, g, l, c_tf; Matrix3d J;
  Matrix4d Ainv;
  // Control_Nonliner Inputs: inertial frame
  //  xd = f(t) desired position, xd_dot = f(xd)
  //  desired velocity, xd_ddot = f(xd) desired acceleration
  Vector3d xd, xd_dot, xd_ddot;
  //  Rd = f(t) desired attitude (usually using local coordinates)
  //  Wd = f(Rd) desired angular velocity in the body-fixed frae
  //  Wd_dot = f(R) desired angular acceleration in body-fixed frame
  //  W_b = angular velocity in the body-fixed frame
  Matrix3d Rd;
  Vector3d Wd, Wd_dot, W_b, W_raw;
  // Gains
  // Position gains:
  //  kx = proportional, kv = derivative, kiX = integral
  // Attitude gains:
  // kR = proportional, kW = derivative, kiR = integral
  float kx, kv, kiX, cX, kR, kW, kiR, cR;
  Vector3d rpy;
  Matrix2d e; //inertial frame,
  // Measured Values in Vicon inettial frame
  std::string vicon_name;
  Vector3d x_v, v_v, prev_x_v,prev_v_v;

  Quaterniond q_imu, q_v;
  MatrixXd v_ave;
  bool IMU_flag, Vicon_flag, controller_flag;
  // Integral errors begin at zero
  Vector3d eiX, eiR, eiX_last, eiR_last;
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

  // Time stamps
  double del_t, t_IMU, del_t_IMU, t_prev_IMU;// I2C thread
  double t_v, del_t_v, t_prev_v;// Vicon thread
  double t_CADS, del_t_CADS, t_prev_CADS;// Control thread
  double Speed_Vicon_Thread, Speed_IMU_Thread, Speed_CADS_Thread;
  double RC_vel = 0.0139 , RC_IMU = 0.0054;
  double scale;

  Matrix3d R_b, R_conv; // inertial to body and convention conversion
  Vector3d x_e, v_e;// Position and Velocity in inertial (e) frame
  Matrix3d R_c, W_c, Wdot_c;
  Vector3d b1d;
  // Error Functions
  Vector3d eX, eV, eR, eW, F, M;
  double f_total;
  Vector4d f_motor;
  // Control_Nonlinear Outputs:
  //  eX = position error in inertial frame
  //  eV = velocity error in inertial frame
  //  eR = attitude error between R_eb and Rd
  //  eW = angular velocity error in body-fixed frame
  //  f = force requirement for each motor/prop
  // Control_Nonlinear Inputs & Outputs:
  //  eiX = integral position error
  //  eiR = integral attitude error
  double eiX_sat, eiR_sat;

  // Output of Control_Nonlinear() and Command Execution
  int thr[4] = {0,0,0,0};// i2c motor commands
  std::vector<int>  mtr_addr;
  uint8_t* motor_power;
  //! Constructor.
  odroid_node();
  //! Destructor.
  ~odroid_node();
  //! Callback function for dynamic reconfigure server.
  // void configCallback(node_example::node_example_paramsConfig
  // &config, uint32_t level);

  //! Callback function for subscriber
  bool getWarmup();
  //! IMU sensor message subscriber
  void get_sensor();
  void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
  //! Keyboard input message subscriber
  void key_callback(const std_msgs::String::ConstPtr& msg);
  //! Controller
  void cmd_callback(const odroid::trajectory::ConstPtr& msg);
  void control();
  //! Controller function
  void ctl_callback(hw_interface hw_intf);
  //! Vicon sensor message subscriber
  void vicon_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  // IMU and Vicon synchronization callback
  void imu_vicon_callback(const sensor_msgs::Imu::ConstPtr& msgImu,
      const geometry_msgs::TransformStamped::ConstPtr& msgVicon);
  //! dynamic reconfigure callback
  void callback(odroid::GainsConfig &config, uint32_t level);
  // node handle getter
  ros::NodeHandle getNH(){return n_;};
  // void gazebo_controll();
  int getEnv(){return environment;}
  // void motor_command();
  // void open_I2C();
  bool getIMU();
};

#endif
