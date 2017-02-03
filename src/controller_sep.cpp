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

  Matrix3d Rd = MatrixXd::Identity(3,3);
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
  // Vector3d What_J_W = What * node.J * W;
  // Vector3d Jmult = R.transpose() * Rd * Wddot - What * R.transpose() * Rd * Wd;
  // Vector3d J_Jmult = node.J * Jmult;
  Vector3d M = -node.kR * node.eR - node.kW * eW + W.cross(node.J*W) - node.J*(What*R.transpose()*Rd*Wd - R.transpose()*Rd*Wddot);
  // - node.kiR * node.eiR + What_J_W + J_Jmult - M_g;
  // To try different motor speeds, choose a force in the radial direction
  double f = - node.m*node.g;// N
  // Convert forces & moments to f_i for i = 1:6 (forces of i-th prop)
  VectorXd FM(4);
  FM << f, M(0), M(1), M(2);
  node.M = M;
  node.f_motor = node.Ainv * FM;

  odroid::error e_msg;
  Vector3d kR_eR = node.kR*node.eR;
  Vector3d kW_eW = node.kW*node.eW;
  e_msg.kW = node.kW; e_msg.kR = node.kR;
  e_msg.eR.x = node.eR(0); e_msg.eR.y = node.eR(1);e_msg.eR.z = node.eR(2);
  e_msg.kR_eR.x = kR_eR(0); e_msg.kR_eR.y = kR_eR(1);e_msg.kR_eR.z = kR_eR(2);
  e_msg.eW.x = node.eW(0); e_msg.eW.y = node.eW(1);e_msg.eW.z = node.eW(2);
  e_msg.kW_eW.x = kW_eW(0); e_msg.kW_eW.y = kW_eW(1);e_msg.kW_eW.z = kW_eW(2);
  e_msg.M.x = node.M(0); e_msg.M.y = node.M(1);e_msg.M.z = node.M(2);
  node.pub_.publish(e_msg);

}