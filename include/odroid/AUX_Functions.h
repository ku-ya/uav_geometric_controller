#ifndef AUX_FUNCTIONS_H
#define AUX_FUNCTIONS_H
#include <math.h>
#include "math.hpp"
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
using namespace Eigen;

void vee_eigen(Matrix3d xhat, Vector3d& x){
    x << xhat(2,1), xhat(0,2), xhat(1,0);
}

void err_sat(double min_sat, double max_sat, Vector3d& err){

    for(int i = 0; i < 3; i++){
        if  (err(i) < min_sat)
            err(i) = min_sat;
        else if(err(i) > max_sat)
            err(i) = max_sat;
    }
    return;
}

Matrix3d hat_eigen(Vector3d x){
    Matrix3d xhat;
    xhat(0,0) =   0.0; xhat(0,1) = -x(2); xhat(0,2) =  x(1);
    xhat(1,0) =  x(2); xhat(1,1) =   0.0; xhat(1,2) = -x(0);
    xhat(2,0) = -x(1); xhat(2,1) =  x(0); xhat(2,2) =   0.0;
    return xhat;
}

void eigen_skew (Eigen::Vector3d&  x, Eigen::Matrix3d& skewM)
{// Obtains 3x3 skew-symmetric matrix from 3x1 vector
    skewM(0,0) = 0;
    skewM(0,1) = -x(2);
    skewM(0,2) = x(1);
    skewM(1,0) = x(2);
    skewM(1,1) = 0;
    skewM(1,2) = -x(0);
    skewM(2,0) = -x(1);
    skewM(2,1) = x(0);
    skewM(2,2) = 0;
}

void eigen_invskew (Eigen::Matrix3d& skewM, Eigen::Vector3d& x)
{// Obtains 3x1 vector from its skew-symmetric 3x3 matrix
    x(0) = skewM(2,1);
    x(1) = skewM(0,2);
    x(2) = skewM(1,0);
}

void LowPassRCFilter(
        double x_noisy[3], double x_last[3], double dt, double RC,// Input
        double x_smooth[3])// Output
{

    int i;
    double alpha;

    alpha = dt/(RC+dt);

    for(i=0; i<3; i++)
    {
        x_smooth[i] = alpha*x_noisy[i]+(1-alpha)*x_last[i];
    }
}


// Desired Trajectories

void SmoothSinusoidalCurveBtwnStationaryPts(
        double xd1, double xd2, double t1, double t2, double t_current,// Input
        double *xd_out, double *xd_dot_out, double *xd_ddot_out)// Output
{
    /*
     * Input:    initial & final desired position & time and current time
     * Function: creates a smooth trajectory of half of a cosine wave
     *           between the two points, thereby exhibiting zero
     *           initial & final velocity
     * Output:   The function value and two time derivatives at the
     *           current time specified
     */

    double del_xd12 = xd2-xd1;
    double del_t12 = t2-t1;

    *xd_out = xd1+del_xd12*(-0.5*cos(M_PI*(t_current-t1)/del_t12)+0.5);

    *xd_dot_out = del_xd12*0.5*sin(M_PI*(t_current-t1)/del_t12)*(M_PI/del_t12);

    *xd_ddot_out = del_xd12*0.5*cos(M_PI*(t_current-t1)/del_t12)*(M_PI/del_t12)*(M_PI/del_t12);

    // return;
}
#endif // AUX_H
