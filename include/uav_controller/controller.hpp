#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "math.hpp"
#include <eigen3/Eigen/Dense>
#include <uav_controller/node.hpp>

class controller
{
public:
  static void GeometricPositionController(node& node, Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot,Vector3d Wd, Vector3d Wddot, Vector3d x_v, Vector3d v_v, Vector3d W_in, Matrix3d R_v);
  static void GeometricControl_SphericalJoint_3DOF(node& node, Vector3d Wd, Vector3d Wddot, Vector3d W, Matrix3d R);
  static void gazebo_control(node& node);
};
#endif
