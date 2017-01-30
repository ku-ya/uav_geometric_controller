#include <odroid/sensor.hpp>
using namespace std;

void sensor::imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  // vector3Transfer(W_b, msg->angular_velocity);
  // if(!IMU_flag){ ROS_INFO("IMU ready");}
  // IMU_flag = true;
  // if(isnan(W_raw(0)) || isnan(W_raw(1)) || isnan(W_raw(2))){IMU_flag = false;}
  // if(print_imu){
  //  printf("IMU: Psi:[%f], Theta:[%f], Phi:[%f] \n", W_b(0), W_b(1), W_b(2));
  // }
};
void sensor::vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& msgVicon){


};
