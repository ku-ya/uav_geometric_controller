#ifndef AUX_FUNCTIONS_H
#define AUX_FUNCTIONS_H
#include "math.h"
#include <eigen3/Eigen/Dense>
// using namespace Eigen;
// Operational Functions

// Function to read keypad commands on the Gumstix
int getch(void)
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}

// Matrix Operations

double transpose (double xx[3][3], double transposex[3][3])
{// Transpose (3x3) to (3x3)'
    transposex[0][0] = xx[0][0];
    transposex[0][1] = xx[1][0];
    transposex[0][2] = xx[2][0];
    transposex[1][0] = xx[0][1];
    transposex[1][1] = xx[1][1];
    transposex[1][2] = xx[2][1];
    transposex[2][0] = xx[0][2];
    transposex[2][1] = xx[1][2];
    transposex[2][2] = xx[2][2];
    return 0;
}

double dot_product (double A[3], double B[3], double C)
{// C = A'*B
    C = A[0]*B[0]+A[1]*B[1]+A[2]*B[2];
    return 0;
}

double cross_product (double A[3], double B[3], double C[3])
{// C = AxB
    C[0] = A[1]*B[2]-A[2]*B[1];
    C[1] = A[2]*B[0]-A[0]*B[2];
    C[2] = A[0]*B[1]-A[1]*B[0];
    return 0;
}

double matrix_vector (double xx[3][3],double yy[3], double Mvector[3])
{// Multiplies (3x3)*(3x1) = (3x1)
    Mvector[0] = xx[0][0]*yy[0]+xx[0][1]*yy[1]+xx[0][2]*yy[2];
    Mvector[1] = xx[1][0]*yy[0]+xx[1][1]*yy[1]+xx[1][2]*yy[2];
    Mvector[2] = xx[2][0]*yy[0]+xx[2][1]*yy[1]+xx[2][2]*yy[2];
    return 0;
}

double vector_vector (double xx[3],double yy[3], double Mvector[3][3])
{// Multiplies (3x1)*(1x3) = (3x3)
    Mvector[0][0] = xx[0]*yy[0];
    Mvector[0][1] = xx[0]*yy[1];
    Mvector[0][2] = xx[0]*yy[2];

    Mvector[1][0] = xx[1]*yy[0];
    Mvector[1][1] = xx[1]*yy[1];
    Mvector[1][2] = xx[1]*yy[2];

    Mvector[2][0] = xx[2]*yy[0];
    Mvector[2][1] = xx[2]*yy[1];
    Mvector[2][2] = xx[2]*yy[2];
    return 0;
}


void matrix_vector6 (double A[6][6],double B[6], double C[6])
{// Multiplies (6x6)*(6x1) = (6x1)
    int i,k;
    for (i = 0; i < 6; i++) {
        C[i] = 0.0;
    }
    for (i = 0; i < 6; i++) {
        for (k = 0; k < 6; k++) {
            C[i] += A[i][k] * B[k];
        }
    }
    return;
}

void reshape_3by3_to_9by1(double Mat3by3[3][3], double Vec9by1[9])
{
    Vec9by1[0] = Mat3by3[0][0];
    Vec9by1[1] = Mat3by3[1][0];
    Vec9by1[2] = Mat3by3[2][0];
    Vec9by1[3] = Mat3by3[0][1];
    Vec9by1[4] = Mat3by3[1][1];
    Vec9by1[5] = Mat3by3[2][1];
    Vec9by1[6] = Mat3by3[0][2];
    Vec9by1[7] = Mat3by3[1][2];
    Vec9by1[8] = Mat3by3[2][2];
}

void reshape_9by1_to_3by3(double Vec9by1[9], double Mat3by3[3][3])
{
    Mat3by3[0][0] = Vec9by1[0];
    Mat3by3[1][0] = Vec9by1[1];
    Mat3by3[2][0] = Vec9by1[2];
    Mat3by3[0][1] = Vec9by1[3];
    Mat3by3[1][1] = Vec9by1[4];
    Mat3by3[2][1] = Vec9by1[5];
    Mat3by3[0][2] = Vec9by1[6];
    Mat3by3[1][2] = Vec9by1[7];
    Mat3by3[2][2] = Vec9by1[8];
}

void norm_vec3(double a[3], double *norm_a)
{
    *norm_a = sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
}


// Hat map
void hat(double x[3], double xhat[3][3])
{// Obtains 3x3 skew-symmetric matrix from 3x1 vector
    xhat[0][0] = 0;
    xhat[0][1] = -x[2];
    xhat[0][2] = x[1];
    xhat[1][0] = x[2];
    xhat[1][1] = 0;
    xhat[1][2] = -x[0];
    xhat[2][0] = -x[1];
    xhat[2][1] = x[0];
    xhat[2][2] = 0;
}

// Vee map
void vee(double xhat[3][3], double x[3])
{// Obtains 3x1 vector from its skew-symmetric 3x3 matrix
    x[0] = xhat[2][1];
    x[1] = xhat[0][2];
    x[2] = xhat[1][0];
}


double skew (double xx[3], double skewx[3][3])
{// Obtains 3x3 skew-symmetric matrix from 3x1 vector
    skewx[0][0] = 0;
    skewx[0][1] = -xx[2];
    skewx[0][2] = xx[1];
    skewx[1][0] = xx[2];
    skewx[1][1] = 0;
    skewx[1][2] = -xx[0];
    skewx[2][0] = -xx[1];
    skewx[2][1] = xx[0];
    skewx[2][2] = 0;
    return 0;
}

double invskew (double skewx[3][3], double xx[3])
{// Obtains 3x1 vector from its skew-symmetric 3x3 matrix
    xx[0] = skewx[2][1];
    xx[1] = skewx[0][2];
    xx[2] = skewx[1][0];
    return 0;
}


void eigen_skew (Eigen::Vector3d  x, Eigen::Matrix3d skewM)
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

void eigen_invskew (Eigen::Matrix3d skewM, Eigen::Vector3d x)
{// Obtains 3x1 vector from its skew-symmetric 3x3 matrix
    x(0) = skewM(2,1);
    x(1) = skewM(0,2);
    x(2) = skewM(1,0);
}

void euler_Rvm(Eigen::Matrix3d& R_vm, Eigen::Vector3d angle){
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

//double expmso3 (double r[3], double R[3][3])
//{// Matrix exponential to obtain rotation matrix transition
//    double nr = 0.0,Sr[3][3],Sr2[3][3],ff1=0.0,ff2=0.0;
//    nr = sqrt((r[0]*r[0])+(r[1]*r[1])+(r[2]*r[2]));

//    if (nr == 0)
//    {
//        R[0][0] = 1;
//        R[0][1] = 0;
//        R[0][2] = 0;
//        R[1][0] = 0;
//        R[1][1] = 1;
//        R[1][2] = 0;
//        R[2][0] = 0;
//        R[2][1] = 0;
//        R[2][2] = 1;
//    }
//    else
//    {
//        Sr[0][0] = 0.0;
//        Sr[0][1] = -r[2];
//        Sr[0][2] = r[1];
//        Sr[1][0] = r[2];
//        Sr[1][1] = 0.0;
//        Sr[1][2] = -r[0];
//        Sr[2][0] = -r[1];
//        Sr[2][1] = r[0];
//        Sr[2][2] = 0.0;

//        Matrix_multipication(Sr, Sr, Sr2);

//        ff1 = sin(nr)/nr;
//        ff2 = (2*sin(0.5*nr)*sin(0.5*nr))/(nr*nr);

//        R[0][0] = 1+ff1*Sr[0][0]+ff2*Sr2[0][0];
//        R[0][1] = ff1*Sr[0][1]+ff2*Sr2[0][1];
//        R[0][2] = ff1*Sr[0][2]+ff2*Sr2[0][2];
//        R[1][0] = ff1*Sr[1][0]+ff2*Sr2[1][0];
//        R[1][1] = 1+ff1*Sr[1][1]+ff2*Sr2[1][1];
//        R[1][2] = ff1*Sr[1][2]+ff2*Sr2[1][2];
//        R[2][0] = ff1*Sr[2][0]+ff2*Sr2[2][0];
//        R[2][1] = ff1*Sr[2][1]+ff2*Sr2[2][1];
//        R[2][2] = 1+ff1*Sr[2][2]+ff2*Sr2[2][2];
//    }
//    return 0;
//}

// Filters

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

//void matexp_to_RdWdWddot(double a, double theta, double thetadot, double thetaddot,
//                         double Rd[3][3], double Wd[3], Wddot[3])
//{// Attitude trajectory given axis and angle
//    Rd = expm(hat(a)*theta);
//    Rddot = hat(a)*expm(hat(a)*theta)*thetadot;
//    transpose(Rd, trpRd);
//    Wd = vee(trpRd*Rddot);
//    Rdddot = hat(a)^2*expm(hat(a)*theta)*thetadot^2+hat(a)*expm(hat(a)*theta)*thetaddot;
//    Wddot = Rd*vee(Rdddot-Rddot*hat(Wd));
//}


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
