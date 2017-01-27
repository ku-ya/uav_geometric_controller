#ifndef AUX_FUNCTIONS_H
#define AUX_FUNCTIONS_H
#include <math.h>
#include "math.hpp"
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
using namespace Eigen;

void quatToMat(Matrix3d& mat, Vector4d v_in){
  double qx = v_in(0), qy = v_in(1), qz = v_in(2), qw = v_in(3);
  mat(0,0) = 1.0-2*qy*qy-2*qz*qz; mat(0,1) = 2*qx*qy-2*qz*qw;     mat(0,2) = 2*qx*qz+2*qy*qw;
  mat(1,0) = 2*qx*qy+2*qz*qw;     mat(1,1) = 1.0-2*qx*qx-2*qz*qz; mat(1,2) = 2*qy*qz-2*qx*qw;
  mat(2,0) = 2*qx*qz-2*qy*qw;     mat(2,1) = 2*qy*qz+2*qx*qw;     mat(2,2) = 1.0-2*qx*qx-2*qy*qy;
}

void vector3Transfer(Vector3d& v_out, geometry_msgs::Vector3 v_in){
    v_out(0) = v_in.x;
    v_out(1) = v_in.y;
    v_out(2) = v_in.z;
}
void vector4Transfer(Vector4d& v_out, geometry_msgs::Quaternion v_in){
    v_out(0) = v_in.x;
    v_out(1) = v_in.y;
    v_out(2) = v_in.z;
    v_out(3) = v_in.w;
}

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


void OutputMotor(Eigen::VectorXd f_motor, int thr[4]){

    // for(int k = 0; k < 4; k++){
    //   thr(k) = floor(1/0.03*(f_motor(k)+0.37)+0.5);
    // }

    // double a =  6.0252E-5;
    // double b =  8.4086E-3;
    // double c = -2.5194E-2;

    // double a_values[6] = {0.6252E-4, 0.8251E-4, 0.7659E-4, 0.8197E-4, 0.8162E-4, 0.7524E-4};
    // double b_values[6] = {0.0142, 0.0097, 0.0097, 0.0087, 0.0088, 0.0099};
    // double c_values[6] = {-0.3700, -0.2380, -0.2747, -0.2418, -0.2495, -0.2489};
    //
    // double a, b, c;
    //
    // f[1] = -f[1];
    // f[3] = -f[3];
    // f[5] = -f[5];
    // // Use a scale factor to correct for offset in motor calibration
    // // 12.5 V / 11.1 V = 1.1261
    // double scale_factor = 1;
    //
    // for(i = 0; i < 6; i++)
    // {
    //   a = a_values[i];
    //   b = b_values[i];
    //   c = c_values[i];
    //
    //     if(f[i] < 0.0)
    //         f[i] = 0.0;
    //     thr[i]=ceil((-b+sqrt(b*b-4.*a*(c-f[i])))/2./a/scale_factor);
    //     if(thr[i] > 240.0)
    //         thr[i]=240.0;
    //     if(thr[i] < 0.0)
    //         thr[i]=0.0;
    // }
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

void euler_Rvm(Eigen::Matrix3d& R_vm, Eigen::Vector3d& angle){
    double psi = angle(0);
    double theta = angle(1);
    double phi = angle(2);
    R_vm(0,0) = cos(theta)*cos(psi);
    R_vm(0,1) = cos(theta)*sin(psi);
    R_vm(0,2) = -sin(theta);
    R_vm(1,0) = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
    R_vm(1,1) = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
    R_vm(1,2) = sin(phi)*cos(theta);
    R_vm(2,0) = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    R_vm(2,1) = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
    R_vm(2,2) = cos(phi)*cos(theta);
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

//  Euler Angles to SO(3) (a variety of conventions)
// void EAngles123_to_R(double angles[3], double R[3][3]){
//     double phi, theta, psi;
//     phi = angles[0];
//     theta = angles[1];
//     psi = angles[2];
//
//     double R1[3][3], R2[3][3], R3[3][3];
//
//     R1[0][0] = 1.0;
//     R1[0][1] = 0.0;
//     R1[0][2] = 0.0;
//     R1[1][0] = 0.0;
//     R1[1][1] = cos(phi);
//     R1[1][2] = -sin(phi);
//     R1[2][0] = 0.0;
//     R1[2][1] = sin(phi);
//     R1[2][2] = cos(phi);
//
//     R2[0][0] = cos(theta);
//     R2[0][1] = 0.0;
//     R2[0][2] = sin(theta);
//     R2[1][0] = 0.0;
//     R2[1][1] = 1.0;
//     R2[1][2] = 0.0;
//     R2[2][0] = -sin(theta);
//     R2[2][1] = 0.0;
//     R2[2][2] = cos(theta);
//
//     R3[0][0] = cos(psi);
//     R3[0][1] = -sin(psi);
//     R3[0][2] = 0.0;
//     R3[1][0] = sin(psi);
//     R3[1][1] = cos(psi);
//     R3[1][2] = 0.0;
//     R3[2][0] = 0.0;
//     R3[2][1] = 0.0;
//     R3[2][2] = 1.0;
//
//     double R_intermediate[3][3];
//     Matrix_multipication(R1, R2, R_intermediate);
//     Matrix_multipication(R_intermediate, R3, R);
//
//     return;
// }
//
// void EAngles132_to_R(double angles[3], double R[3][3]){
//     double phi, theta, psi;
//     phi = angles[0];
//     theta = angles[1];
//     psi = angles[2];
//
//     double R1[3][3], R2[3][3], R3[3][3];
//
//     R1[0][0] = 1.0;
//     R1[0][1] = 0.0;
//     R1[0][2] = 0.0;
//     R1[1][0] = 0.0;
//     R1[1][1] = cos(phi);
//     R1[1][2] = -sin(phi);
//     R1[2][0] = 0.0;
//     R1[2][1] = sin(phi);
//     R1[2][2] = cos(phi);
//
//     R2[0][0] = cos(theta);
//     R2[0][1] = 0.0;
//     R2[0][2] = sin(theta);
//     R2[1][0] = 0.0;
//     R2[1][1] = 1.0;
//     R2[1][2] = 0.0;
//     R2[2][0] = -sin(theta);
//     R2[2][1] = 0.0;
//     R2[2][2] = cos(theta);
//
//     R3[0][0] = cos(psi);
//     R3[0][1] = -sin(psi);
//     R3[0][2] = 0.0;
//     R3[1][0] = sin(psi);
//     R3[1][1] = cos(psi);
//     R3[1][2] = 0.0;
//     R3[2][0] = 0.0;
//     R3[2][1] = 0.0;
//     R3[2][2] = 1.0;
//
//     double R_intermediate[3][3];
//     Matrix_multipication(R1, R3, R_intermediate);
//     Matrix_multipication(R_intermediate, R2, R);
//
//     return;
// }
//
// void EAngles213_to_R(double angles[3], double R[3][3]){
//     double phi, theta, psi;
//     phi = angles[0];
//     theta = angles[1];
//     psi = angles[2];
//
//     double R1[3][3], R2[3][3], R3[3][3];
//
//     R1[0][0] = 1.0;
//     R1[0][1] = 0.0;
//     R1[0][2] = 0.0;
//     R1[1][0] = 0.0;
//     R1[1][1] = cos(phi);
//     R1[1][2] = -sin(phi);
//     R1[2][0] = 0.0;
//     R1[2][1] = sin(phi);
//     R1[2][2] = cos(phi);
//
//     R2[0][0] = cos(theta);
//     R2[0][1] = 0.0;
//     R2[0][2] = sin(theta);
//     R2[1][0] = 0.0;
//     R2[1][1] = 1.0;
//     R2[1][2] = 0.0;
//     R2[2][0] = -sin(theta);
//     R2[2][1] = 0.0;
//     R2[2][2] = cos(theta);
//
//     R3[0][0] = cos(psi);
//     R3[0][1] = -sin(psi);
//     R3[0][2] = 0.0;
//     R3[1][0] = sin(psi);
//     R3[1][1] = cos(psi);
//     R3[1][2] = 0.0;
//     R3[2][0] = 0.0;
//     R3[2][1] = 0.0;
//     R3[2][2] = 1.0;
//
//     double R_intermediate[3][3];
//     Matrix_multipication(R2, R1, R_intermediate);
//     Matrix_multipication(R_intermediate, R3, R);
//
//     return;
// }
//
// void EAngles231_to_R(double angles[3], double R[3][3]){
//     double phi, theta, psi;
//     phi = angles[0];
//     theta = angles[1];
//     psi = angles[2];
//
//     double R1[3][3], R2[3][3], R3[3][3];
//
//     R1[0][0] = 1.0;
//     R1[0][1] = 0.0;
//     R1[0][2] = 0.0;
//     R1[1][0] = 0.0;
//     R1[1][1] = cos(phi);
//     R1[1][2] = -sin(phi);
//     R1[2][0] = 0.0;
//     R1[2][1] = sin(phi);
//     R1[2][2] = cos(phi);
//
//     R2[0][0] = cos(theta);
//     R2[0][1] = 0.0;
//     R2[0][2] = sin(theta);
//     R2[1][0] = 0.0;
//     R2[1][1] = 1.0;
//     R2[1][2] = 0.0;
//     R2[2][0] = -sin(theta);
//     R2[2][1] = 0.0;
//     R2[2][2] = cos(theta);
//
//     R3[0][0] = cos(psi);
//     R3[0][1] = -sin(psi);
//     R3[0][2] = 0.0;
//     R3[1][0] = sin(psi);
//     R3[1][1] = cos(psi);
//     R3[1][2] = 0.0;
//     R3[2][0] = 0.0;
//     R3[2][1] = 0.0;
//     R3[2][2] = 1.0;
//
//     double R_intermediate[3][3];
//     Matrix_multipication(R2, R3, R_intermediate);
//     Matrix_multipication(R_intermediate, R1, R);
//
//     return;
// }
//
// void EAngles312_to_R(double angles[3], double R[3][3]){
//     double phi, theta, psi;
//     phi = angles[0];
//     theta = angles[1];
//     psi = angles[2];
//
//     double R1[3][3], R2[3][3], R3[3][3];
//
//     R1[0][0] = 1.0;
//     R1[0][1] = 0.0;
//     R1[0][2] = 0.0;
//     R1[1][0] = 0.0;
//     R1[1][1] = cos(phi);
//     R1[1][2] = -sin(phi);
//     R1[2][0] = 0.0;
//     R1[2][1] = sin(phi);
//     R1[2][2] = cos(phi);
//
//     R2[0][0] = cos(theta);
//     R2[0][1] = 0.0;
//     R2[0][2] = sin(theta);
//     R2[1][0] = 0.0;
//     R2[1][1] = 1.0;
//     R2[1][2] = 0.0;
//     R2[2][0] = -sin(theta);
//     R2[2][1] = 0.0;
//     R2[2][2] = cos(theta);
//
//     R3[0][0] = cos(psi);
//     R3[0][1] = -sin(psi);
//     R3[0][2] = 0.0;
//     R3[1][0] = sin(psi);
//     R3[1][1] = cos(psi);
//     R3[1][2] = 0.0;
//     R3[2][0] = 0.0;
//     R3[2][1] = 0.0;
//     R3[2][2] = 1.0;
//
//     double R_intermediate[3][3];
//     Matrix_multipication(R3, R1, R_intermediate);
//     Matrix_multipication(R_intermediate, R2, R);
//
//     return;
// }
//
// void EAngles321_to_R(double angles[3], double R[3][3]){
//     double phi, theta, psi;
//     phi = angles[0];
//     theta = angles[1];
//     psi = angles[2];
//
//     double R1[3][3], R2[3][3], R3[3][3];
//
//     R1[0][0] = 1.0;
//     R1[0][1] = 0.0;
//     R1[0][2] = 0.0;
//     R1[1][0] = 0.0;
//     R1[1][1] = cos(phi);
//     R1[1][2] = sin(phi);
//     R1[2][0] = 0.0;
//     R1[2][1] = -sin(phi);
//     R1[2][2] = cos(phi);
//
//     R2[0][0] = cos(theta);
//     R2[0][1] = 0.0;
//     R2[0][2] = -sin(theta);
//     R2[1][0] = 0.0;
//     R2[1][1] = 1.0;
//     R2[1][2] = 0.0;
//     R2[2][0] = sin(theta);
//     R2[2][1] = 0.0;
//     R2[2][2] = cos(theta);
//
//     R3[0][0] = cos(psi);
//     R3[0][1] = sin(psi);
//     R3[0][2] = 0.0;
//     R3[1][0] = -sin(psi);
//     R3[1][1] = cos(psi);
//     R3[1][2] = 0.0;
//     R3[2][0] = 0.0;
//     R3[2][1] = 0.0;
//     R3[2][2] = 1.0;
//
//     double R_intermediate[3][3];
//     Matrix_multipication(R3, R2, R_intermediate);
//     Matrix_multipication(R_intermediate, R1, R);
//
//     return;
// }
//
// void EAngles313_to_R(double angles[3], double R[3][3]){
//     double phi, theta, psi;
//     phi = angles[0];
//     theta = angles[1];
//     psi = angles[2];
//
//     double R1[3][3], R2[3][3], R3[3][3];
//
//     R1[0][0] = 1.0;
//     R1[0][1] = 0.0;
//     R1[0][2] = 0.0;
//     R1[1][0] = 0.0;
//     R1[1][1] = cos(phi);
//     R1[1][2] = -sin(phi);
//     R1[2][0] = 0.0;
//     R1[2][1] = sin(phi);
//     R1[2][2] = cos(phi);
//
//     R2[0][0] = cos(theta);
//     R2[0][1] = 0.0;
//     R2[0][2] = sin(theta);
//     R2[1][0] = 0.0;
//     R2[1][1] = 1.0;
//     R2[1][2] = 0.0;
//     R2[2][0] = -sin(theta);
//     R2[2][1] = 0.0;
//     R2[2][2] = cos(theta);
//
//     R3[0][0] = cos(psi);
//     R3[0][1] = -sin(psi);
//     R3[0][2] = 0.0;
//     R3[1][0] = sin(psi);
//     R3[1][1] = cos(psi);
//     R3[1][2] = 0.0;
//     R3[2][0] = 0.0;
//     R3[2][1] = 0.0;
//     R3[2][2] = 1.0;
//
//     double R_intermediate[3][3];
//     Matrix_multipication(R3, R1, R_intermediate);
//     Matrix_multipication(R_intermediate, R3, R);
//
//     return;
// }



#endif // AUX_H
