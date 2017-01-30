#ifndef SENSOR_H
#define SENSOR_H

#include "math.hpp"
#include <eigen3/Eigen/Dense>
#include <odroid/odroid_node.hpp>


class sensor
{
public:
  static void imu_callback(const sensor_msgs::Imu::ConstPtr& msg, odroid_node& node);
  static void vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& msgVicon);

//odroid_node node,Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot,Vector3d Wd, Vector3d Wddot, Vector3d x_v, Vector3d v_v, Vector3d W_in, Matrix3d R_v);
// void GeometricControl_SphericalJoint_3DOF_eigen(Vector3d Wd, Vector3d Wddot, Vector3d W, Matrix3d R, double del_t, VectorXd eiR_last);
// void GeometricController_6DOF(Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot, Matrix3d Rd, Vector3d Wd, Vector3d Wddot, Vector3d x_e, Vector3d v_e, Vector3d W, Matrix3d R, double del_t);
};
#endif
