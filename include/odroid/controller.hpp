#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "math.hpp"
#include <eigen3/Eigen/Dense>
#include <odroid/odroid_node.hpp>

class controller
{
public:
  static void GeometricPositionController(odroid_node& node, Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot,Vector3d Wd, Vector3d Wddot, Vector3d x_v, Vector3d v_v, Vector3d W_in, Matrix3d R_v);

  static void GeometricControl_SphericalJoint_3DOF(odroid_node& node, Vector3d Wd, Vector3d Wddot, Vector3d W, Matrix3d R);

  static void gazebo_control(odroid_node& node);
//odroid_node node,Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot,Vector3d Wd, Vector3d Wddot, Vector3d x_v, Vector3d v_v, Vector3d W_in, Matrix3d R_v);
// void GeometricControl_SphericalJoint_3DOF_eigen(Vector3d Wd, Vector3d Wddot, Vector3d W, Matrix3d R, double del_t, VectorXd eiR_last);
// void GeometricController_6DOF(Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot, Matrix3d Rd, Vector3d Wd, Vector3d Wddot, Vector3d x_e, Vector3d v_e, Vector3d W, Matrix3d R, double del_t);
};
#endif
