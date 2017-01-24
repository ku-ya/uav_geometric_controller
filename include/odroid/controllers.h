#ifndef CONTROLLERS
#define CONTROLLERS

#include <eigen3/Eigen/Dense>
using namespace std;
using namespace Eigen;

// #include "subscriber_classes.h"

class controller_gains{
private:
    double norm_J;
public:
    double kx;
    double kv;
    double kiX;
    double cX;
    double kR;
    double kW;
    double kiR;
    double cR;

    void properties2gainsX(double wnx, double zetax, double m){
        kx = wnx*wnx*m;
        kv = 2*wnx*zetax*m;
    }

    void properties2gainsR(double wnR, double zetaR, Matrix3d J){
        norm_J = J.lpNorm<Infinity>();
        kR = wnR*wnR*norm_J;
        kW = 2*wnR*zetaR*norm_J;
    }
};

class controller_saturations{
public:
    double eiX_sat;
    double eiR_sat;
};

class inertial_properties{
public:
    double  m;
    Matrix3d J;
    double  g;
};

// vec3by1 err_sat(double min_sat, double max_sat, vec3by1& err){
//     vec3by1 err_checked;
//
//     for(int i = 0; i < 3; i++){
//         if     (err(i) < min_sat)
//             err_checked(i) = min_sat;
//         else if(err(i) > max_sat)
//             err_checked(i) = max_sat;
//         else
//             err_checked(i) = err(i);
//     }
//
//     return err_checked;
// }


// void QuadrotorGeometricPositionController(
//         inertial_properties& props, controller_gains& gains, controller_saturations& sat_limits,
//         vec3by1& xd,  vec3by1& xd_dot,  vec3by1& xd_2dot,  vec3by1& xd_3dot, vec3by1& xd_4dot,
//         vec3by1& b1d, vec3by1& b1d_dot, vec3by1& b1d_ddot, PoseSE3& pose, double del_t,
//         vec3by1& ex, vec3by1& ev, vec3by1& eR, vec3by1& eW, vec3by1& eiX, vec3by1& eiR,
//         double& f, vec3by1& M){
//
//     // Bring to controller frame (and back) with 180 degree rotation about b1
//     mat3by3 D;
//     D << 1.0,  0.0,  0.0,
//          0.0, -1.0,  0.0,
//          0.0,  0.0, -1.0;
//     vec3by1 e3(0.0,0.0,1.0);// commonly-used unit vector
//
//     vec3by1 x = D*pose.X;// LI
//     vec3by1 v = D*pose.V;// LI
//     mat3by3 R = D*pose.R*D;// LI<-LBFF
//     vec3by1 W = D*pose.W;// LBFF
//
//     xd = D*xd;
//     xd_dot = D*xd_dot;
//     xd_2dot = D*xd_2dot;
//     xd_3dot = D*xd_3dot;
//     xd_4dot = D*xd_4dot;
//
//     b1d = D*b1d;
//     b1d_dot = D*b1d_dot;
//     b1d_ddot = D*b1d_ddot;
//
//     // Properties
//     double m = props.m;
//     mat3by3 J = props.J;
//     double g = props.g;
//
//     // Gains
//     double kx  = gains.kx;
//     double kv  = gains.kv;
//     double kiX = gains.kiX;
//     double cX =  gains.cX;
//     double kR  = gains.kR;
//     double kW  = gains.kW;
//     double kiR = gains.kiR;
//     double cR =  gains.cR;
//
//     // Saturation Limits
//     double eiX_sat = sat_limits.eiX_sat;
//     double eiR_sat = sat_limits.eiR_sat;
//
//     // Translational Error Functions
//     ex = x-xd;
//     ev = v-xd_dot;
//     eiX = eiX+del_t*(ex+cX*ev);
//     eiX = err_sat(-eiX_sat, eiX_sat, eiX);
//
//     // Force 'f' along negative b3-axis
//     vec3by1 A = -kx*ex-kv*ev-kiX*eiX-m*g*e3+m*xd_2dot;
//     vec3by1 L = R*e3;
//     vec3by1 Ldot = R*hat_eigen(W)*e3;
//     f = -A.dot(R*e3);
//
//     // Intermediate Terms for Rotational Errors
//     vec3by1 ea = g*e3-f/m*L-xd_2dot;
//     vec3by1 Adot = -kx*ev-kv*ea+m*xd_3dot;// Lee Matlab: -ki*satdot(sigma,ei,ev+c1*ex);
//
//     double fdot = -Adot.dot(L)-A.dot(Ldot);
//     vec3by1 eb = -fdot/m*L-f/m*Ldot-xd_3dot;
//     vec3by1 Addot = -kx*ea-kv*eb+m*xd_4dot;// Lee Matlab: -ki*satdot(sigma,ei,ea+c1*ev);
//
//     double nA = A.norm();
//     vec3by1 Ld = -A/nA;
//     vec3by1 Lddot = -Adot/nA+A*A.dot(Adot)/pow(nA,3);
//     vec3by1 Ldddot = -Addot/nA+Adot/pow(nA,3)*(2*A.dot(Adot))
//             +A/pow(nA,3)*(Adot.dot(Adot)+A.dot(Addot))
//             -3*A/pow(nA,5)*pow(A.dot(Adot),2);
//
//     vec3by1 Ld2 = -hat_eigen(b1d)*Ld;
//     vec3by1 Ld2dot = -hat_eigen(b1d_dot)*Ld-hat_eigen(b1d)*Lddot;
//     vec3by1 Ld2ddot = -hat_eigen(b1d_ddot)*Ld-2*hat_eigen(b1d_dot)*Lddot-hat_eigen(b1d)*Ldddot;
//
//     double nLd2 = Ld2.norm();
//     vec3by1 Rd2 = Ld2/nLd2;
//     vec3by1 Rd2dot = Ld2dot/nLd2-Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2;
//     vec3by1 Rd2ddot = Ld2ddot/nLd2-Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2dot
//             -Ld2dot.dot(Ld2dot)/pow(nLd2,3)*Ld2-Ld2.dot(Ld2ddot)/pow(nLd2,3)*Ld2
//             -Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2dot
//             +3*pow(Ld2.dot(Ld2dot),2)/pow(nLd2,5)*Ld2;
//
//     vec3by1 Rd1 = hat_eigen(Rd2)*Ld;
//     vec3by1 Rd1dot = hat_eigen(Rd2dot)*Ld+hat_eigen(Rd2)*Lddot;
//     vec3by1 Rd1ddot = hat_eigen(Rd2ddot)*Ld+2*hat_eigen(Rd2dot)*Lddot+hat_eigen(Rd2)*Ldddot;
//
//     mat3by3 Rd, Rddot, Rdddot;
//     Rd << Rd1, Rd2, Ld;
//     Rddot << Rd1dot, Rd2dot, Lddot;
//     Rdddot << Rd1ddot, Rd2ddot, Ldddot;
//
//     vec3by1 Wd, Wddot;
//     vee_eigen(Rd.transpose()*Rddot, Wd);
//     vee_eigen(Rd.transpose()*Rdddot-hat_eigen(Wd)*hat_eigen(Wd), Wddot);
//
//     // Attitude Error 'eR'
//     vee_eigen(.5*(Rd.transpose()*R-R.transpose()*Rd), eR);
//
//     // Angular Velocity Error 'eW'
//     eW = W-R.transpose()*Rd*Wd;
//
//     // Attitude Integral Term
//     eiR += del_t*(eR+cR*eW);
//     eiR = err_sat(-eiR_sat, eiR_sat, eiR);
//
//     // 3D Moment
//     M = -kR*eR-kW*eW-kiR*eiR+hat_eigen(R.transpose()*Rd*Wd)*J*R.transpose()*Rd*Wd+J*R.transpose()*Rd*Wddot;// LBFF
//     M = D*M;// LBFF->GBFF
//     return;
// }


#endif // CONTROLLERS
