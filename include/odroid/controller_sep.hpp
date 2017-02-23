#ifndef CONTROLLER_SEP_H
#define CONTROLLER_SEP_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;

class controller_sep
{

    public:

      int add(int , int );

      static void AttitudeControl(Vector3d W,Vector3d Wd, Vector3d Wddot, Matrix3d R, Matrix3d Rd, Vector3d& M);
};

#endif
