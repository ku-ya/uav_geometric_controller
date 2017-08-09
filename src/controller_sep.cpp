#include <uav_geometric_controller/controller_sep.hpp>

int controller_sep::add(int a, int b)
{
  return a+b;
}

/*
Attitude Controller

Computes the control input for attitude control.

Inputs:
Outputs:
Citation to relevant paper
*/
void controller_sep::AttitudeControl(
  Vector3d W,Vector3d Wd, Vector3d Wddot, Matrix3d R, Matrix3d Rd, Vector3d& M)
{

  M << 1.0,2.0,3.0;

}
