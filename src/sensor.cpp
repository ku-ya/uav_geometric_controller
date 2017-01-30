#include <odroid/sensor.hpp>
using namespace std;

void sensor::imu_callback(const sensor_msgs::Imu::ConstPtr& msg, odroid_node& node){
  vector3Transfer(node.W_b, msg->angular_velocity);
  if(!node.IMU_flag){ ROS_INFO("IMU ready");}
  node.IMU_flag = true;
  // if(isnan(W_raw(0)) || isnan(W_raw(1)) || isnan(W_raw(2))){IMU_flag = false;}
  if(node.print_imu){
   printf("IMU: Psi:[%f], Theta:[%f], Phi:[%f] \n", node.W_b(0), node.W_b(1), node.W_b(2));
  }
};
void sensor::vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& msgVicon){

};
