#include <odroid/controller_sep.hpp>

int controller_sep::add(int a, int b)
{
  return a+b;
}

void controller_sep::AttitudeControl(
  Vector3d W,Vector3d Wd, Vector3d Wddot, Matrix3d R, Matrix3d Rd, Vector3d& M)
{

  Matrix3d D = node.R_bm;
  Matrix3d R = D*Rin;// LI<-LBFF
  Vector3d W = Win;// LBFF

  Vector3d e3(0,0,1), b3(0,0,1), vee_3by1;
  
  Vector3d r = -node.l * b3;
  Vector3d F_g = node.m * node.g * R.transpose() * e3;
  Vector3d M_g = r.cross(F_g);
  //   Calculate eR (rotation matrix error)
  Matrix3d inside_vee_3by3 = Rd.transpose() * R - R.transpose() * Rd;
  eigen_invskew(inside_vee_3by3, vee_3by1);// 3x1
  node.eR = vee_3by1/2;
  // Calculate eW (angular velocity error in body-fixed frame)
  Vector3d eW = W - R.transpose() * Rd * Wd;
  node.eW = eW;
  // Update integral term of control
  // Attitude:
  node.eiR = node.eiR_last + node.del_t * node.eR;
  node.eiR_last = node.eiR;
  // Calculate 3 DOFs of M (controlled moment in body-fixed frame)
  // MATLAB: M = -kR*eR-kW*eW-kRi*eiR+cross(W,J*W)+J*(R'*Rd*Wddot-hat(W)*R'*Rd*Wd);
  Matrix3d What;
  eigen_skew(W, What);

  Vector3d M = -kR * eR - kW * eW + W.cross(J*W) - J*(What*R.transpose()*Rd*Wd - R.transpose()*Rd*Wddot);



}