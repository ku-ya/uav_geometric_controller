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
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */
#include "controller.h"

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include "math.hpp"

using namespace Eigen;
// Adafruit PCA9685 Servo Informatio
#define i2c_addr 0x40

#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
// #include <tf/Quaternion.h>

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <odroid/GainsConfig.h>

class odroid_node
{

  private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Publisher vis_pub_, vis_pub_y, vis_pub_z;

    ros::Time vicon_time;
    //  m = body mass
    //  g = acceleration due to gravity
    //  J = moment of inertial matrix
    double m, g; Matrix3d J;
    Matrix4d Ainv;
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
    double kx, kv, kiX, cX, kR, kW, kiR, cR;
    float phi, theta, psi;
    double roll, pitch, yaw;
    Matrix2d e; //inertial frame,
    // Measured Values in Vicon Frame
    Vector3d x_v, v_v, prev_x_v,prev_v_v;// position in the Vicon frame
    // VectorXd quat_vm(4);
    Matrix<double, 4, 1> quat_vm;// attitude of markers measured by Vicon system

    bool IMU_flag, print_imu, print_f, print_thr, print_test_variable, print_xd, print_x_v,
      print_eX, print_eV, print_vicon, print_F, print_M, print_R_eb;
    Matrix<double, 6, 1> f;
    Matrix3d R_bm;

    // Integral errors begin at zero
    Vector3d eiX, eiR, eiX_last, eiR_last;
    Eigen::Matrix<double, 6, 6> invFMmat;
    // Eigen::Matrix<int, 6, 1> thr;
    // Calibrating and Temporary Saving IMU Data
    double phi_bias, theta_bias, psi_bias, W1_bias, W2_bias, W3_bias;
    Vector3d W_b_, W_b_noisy;// Angular velocity of previous loop
    Vector3d EAngles_imu_;// Euler Angles of previous loop
    // double EAngles_imu[3];
    Vector3d x_e_init[3]; Matrix3d R_eb_init;// Initial position and attitude
    Vector3d x_ss, x_ss_; // Steady state error used for landing correction

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
    double del_t, t_IMU, del_t_IMU, t_prev_IMU;// I2C thread
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
    Matrix3d R_ev; // Measured Values for Controller
    Matrix3d R_v;
    Vector3d x_e, v_e;// Position and Velocity in inertial (e) frame
    Matrix3d R_eb;// Body (b) to inertial (e) frame
    Vector3d E_angles_save;

    // Error Functions
    Vector3d eX, eV, eR, eW, F, M;
    double f_quad;
    Matrix<double,4,1> f_motor;
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
    int thr[4];// i2c motor commands
    int servo_addr[6] = {0, 1, 2, 3, 4, 5};// Servo addresses 1-6
    uint16_t servopl[6];// i2c servo pulse length (duty_cycle[i] = servopl[i]/4095 @ ~325 Hz)
    uint16_t zp[6] = {1300, 1285, 1230, 1280, 1215, 1275};

    //TCP Communication
    int sockfd, newsockfd, port_number, n;
    socklen_t client_ln;
    char buffer[1000], buffer_[1000];
    struct sockaddr_in serv_addr, cli_addr;
  public:
    bool simulation; 
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
    void vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
    // IMU and Vicon synchronization callback
    void imu_vicon_callback(const sensor_msgs::Imu::ConstPtr& msgImu, const geometry_msgs::TransformStamped::ConstPtr& msgVicon);
    //! dynamic reconfigure callback
    void callback(odroid::GainsConfig &config, uint32_t level);
    //! Controller function
    void GeometricControl_SphericalJoint_3DOF_eigen(Vector3d Wd, Vector3d Wddot, Vector3d W, Matrix3d R, VectorXd eiR_last);
    // Position controller function
    void GeometricController_6DOF(Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot, Matrix3d Rd, Vector3d Wd, Vector3d Wddot, Vector3d x_v, Vector3d v_v, Vector3d W, Matrix3d R_v);
    // Quadrotor position controller
    void QuadGeometricPositionController(Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot,Vector3d Wd, Vector3d Wddot, Vector3d x_e, Vector3d v_e, Vector3d W_in, Matrix3d R);
    // node handle getter
    ros::NodeHandle getNH(){return n_;};
    void gazebo_controll();

    void motor_command();
    void open_I2C();
    void print_J();
    void print_force();
    bool getIMU();
};

#endif
