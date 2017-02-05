#include <odroid/controller.hpp>
using namespace Eigen;
using namespace std;

void controller::GeometricPositionController(odroid_node& node, Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot,Vector3d Wd, Vector3d Wddot, Vector3d x_v, Vector3d v_v, Vector3d W_in, Matrix3d R_v){
  std::cout.precision(5);
  // Bring to controller frame (and back) with 180 degree rotation about b1
  Matrix3d D = node.R_bm;

  Vector3d xd_2dot = Vector3d::Zero();
  Vector3d xd_3dot = xd_2dot;
  Vector3d xd_4dot = xd_2dot;
  Vector3d b1d(1,0,0);
  Vector3d b1d_dot = xd_2dot;
  Vector3d b1d_ddot = xd_2dot;

  Vector3d e3(0.0,0.0,1.0);// commonly-used unit vector

  Vector3d x = D*x_v;// LI
  Vector3d v = D*v_v;// LI
  Matrix3d R = D*R_v*D;// LI<-LBFF
  Vector3d W = W_in;// LBFF

  xd = D*xd;
  xd_dot = D*xd_dot;
  xd_2dot = D*xd_2dot;
  xd_3dot = D*xd_3dot;
  xd_4dot = D*xd_4dot;

  b1d = D*b1d;
  b1d_dot = D*b1d_dot;
  b1d_ddot = D*b1d_ddot;

  // Translational Error Functions
  Vector3d ex = x - xd;
  Vector3d ev = v - xd_dot;

  if(node.mode == 0){
    ex = ev = Vector3d::Zero();
  }

  node.eiX = node.eiX_last+node.del_t*(ex+node.cX*ev);
  err_sat(-node.eiX_sat, node.eiX_sat, node.eiX);
  node.eiX_last = node.eiX;

  if(node.print_eX){cout<<"eX: "<<ex.transpose()<<endl;}
  if(node.print_eV){cout<<"eV: "<<ev.transpose()<<endl;}

  // Force 'f' along negative b3-axis
  double kx = node.kx;
  double kv = node.kv;
  double kiX = node.kiX;
  double kR = node.kR;
  double kW = node.kW;
  double kiR = node.kiR;

  double m = node.m;
  double g = node.g;

  Vector3d A = -kx*ex-kv*ev-kiX*node.eiX-m*g*e3+m*xd_2dot;
  Vector3d L = R*e3;
  Vector3d Ldot = R*hat_eigen(W)*e3;
  double f = -A.dot(R*e3);
  node.f_quad = f;

  // Intermediate Terms for Rotational Errors
  Vector3d ea = g*e3-f/m*L-xd_2dot;
  Vector3d Adot = -kx*ev-kv*ea+m*xd_3dot;// Lee Matlab: -ki*satdot(sigma,ei,ev+c1*ex);

  double fdot = -Adot.dot(L)-A.dot(Ldot);
  Vector3d eb = -fdot/m*L-f/m*Ldot-xd_3dot;
  Vector3d Addot = -kx*ea-kv*eb+m*xd_4dot;// Lee Matlab: -ki*satdot(sigma,ei,ea+c1*ev);

  double nA = A.norm();
  Vector3d Ld = -A/nA;
  Vector3d Lddot = -Adot/nA+A*A.dot(Adot)/pow(nA,3);
  Vector3d Ldddot = -Addot/nA+Adot/pow(nA,3)*(2*A.dot(Adot))
          +A/pow(nA,3)*(Adot.dot(Adot)+A.dot(Addot))
          -3*A/pow(nA,5)*pow(A.dot(Adot),2);

  Vector3d Ld2 = -hat_eigen(b1d)*Ld;
  Vector3d Ld2dot = -hat_eigen(b1d_dot)*Ld-hat_eigen(b1d)*Lddot;
  Vector3d Ld2ddot = -hat_eigen(b1d_ddot)*Ld-2*hat_eigen(b1d_dot)*Lddot-hat_eigen(b1d)*Ldddot;

  double nLd2 = Ld2.norm();
  Vector3d Rd2 = Ld2/nLd2;
  Vector3d Rd2dot = Ld2dot/nLd2-Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2;
  Vector3d Rd2ddot = Ld2ddot/nLd2-Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2dot
          -Ld2dot.dot(Ld2dot)/pow(nLd2,3)*Ld2-Ld2.dot(Ld2ddot)/pow(nLd2,3)*Ld2
          -Ld2.dot(Ld2dot)/pow(nLd2,3)*Ld2dot
          +3*pow(Ld2.dot(Ld2dot),2)/pow(nLd2,5)*Ld2;

  Vector3d Rd1 = hat_eigen(Rd2)*Ld;
  Vector3d Rd1dot = hat_eigen(Rd2dot)*Ld+hat_eigen(Rd2)*Lddot;
  Vector3d Rd1ddot = hat_eigen(Rd2ddot)*Ld+2*hat_eigen(Rd2dot)*Lddot+hat_eigen(Rd2)*Ldddot;

  Matrix3d Rd, Rddot, Rdddot;
  Rd << Rd1, Rd2, Ld;
  Rddot << Rd1dot, Rd2dot, Lddot;
  Rdddot << Rd1ddot, Rd2ddot, Ldddot;
  // Vector3d Wd, Wddot;
  vee_eigen(Rd.transpose()*Rddot, Wd);
  vee_eigen(Rd.transpose()*Rdddot-hat_eigen(Wd)*hat_eigen(Wd), Wddot);
  // Attitude Error 'eR'
  vee_eigen(.5*(Rd.transpose()*R-R.transpose()*Rd), node.eR);
  // Angular Velocity Error 'eW'
  node.eW = W-R.transpose()*Rd*Wd;
  // Attitude Integral Term
  node.eiR = node.del_t*(node.eR+node.cR*node.eW) + node.eiR_last;
  err_sat(-node.eiR_sat, node.eiR_sat, node.eiR);
  node.eiR_last = node.eiR;
  // 3D Moment
  node.M = -kR*node.eR-kW*node.eW-kiR*node.eiR+hat_eigen(R.transpose()*Rd*Wd)*node.J*R.transpose()*Rd*Wd+node.J*R.transpose()*Rd*Wddot;// LBFF

  Matrix<double, 4, 1> FM;
  FM[0] = f;
  FM[1] = node.M[0];
  FM[2] = node.M[1];
  FM[3] = node.M[2];

  node.f_motor = node.Ainv*FM;
}

void controller::GeometricControl_SphericalJoint_3DOF(odroid_node& node, Vector3d Wd, Vector3d Wddot, Vector3d Win, Matrix3d Rin){

  Matrix3d D = node.R_bm;
  Matrix3d R = D*Rin;// LI<-LBFF
  Vector3d W = Win;// LBFF

  Matrix3d Rd = MatrixXd::Identity(3,3);
  Vector3d e3(0,0,1), b3(0,0,1), vee_3by1;
  double l = 0;//.05;// length of rod connecting to the spherical joint
  Vector3d r = -l * b3;
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
}

void controller::gazebo_controll(odroid_node& node){
  ros::ServiceClient client_FM = node.n_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  gazebo_msgs::ApplyBodyWrench FMcmds_srv;

  Vector3d fvec_GB(0.0, 0.0, node.f_quad), fvec_GI;

  fvec_GI = node.R_v*fvec_GB;
  Vector3d M_out = node.R_v*node.R_bm*node.M;

  FMcmds_srv.request.body_name = "quadrotor::base_link";
  FMcmds_srv.request.reference_frame = "world";
  FMcmds_srv.request.reference_point.x = 0.0;
  FMcmds_srv.request.reference_point.y = 0.0;
  FMcmds_srv.request.reference_point.z = 0.0;
  FMcmds_srv.request.start_time = ros::Time(0.0);
  FMcmds_srv.request.duration = ros::Duration(0.01);// apply continuously until new command

  FMcmds_srv.request.wrench.force.x = fvec_GI(0);
  FMcmds_srv.request.wrench.force.y = fvec_GI(1);
  FMcmds_srv.request.wrench.force.z = fvec_GI(2);

  FMcmds_srv.request.wrench.torque.x = M_out(0);
  FMcmds_srv.request.wrench.torque.y = M_out(1);
  FMcmds_srv.request.wrench.torque.z = M_out(2);

  client_FM.call(FMcmds_srv);
  if(!FMcmds_srv.response.success)
      cout << "Fail! Response message:\n" << FMcmds_srv.response.status_message << endl;
}


// void odroid_node::GeometricController_6DOF(Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot, Matrix3d Rd, Vector3d Wd, Vector3d Wddot, Vector3d x_e, Vector3d v_e, Vector3d W, Matrix3d R)
//   {
//     // Calculate eX (position error in inertial frame)
//     Vector3d eX = x_e - xd;
//     //cout<<"eX\n"<<eX.transpose()<<endl;
//     if(print_eX){cout<<"eX: "<<eX.transpose()<<endl;}
//     // Calculate eV (velocity error in inertial frame)
//     Vector3d eV = v_e - xd_dot;
//     //cout<<"eV\n"<<eV.transpose()<<endl;
//     if(print_eV){cout<<"eV: "<<eV.transpose()<<endl;}
//     // Calculate eR (rotation matrix error)
//     // Take 9 elements of difference
//     Matrix3d inside_vee_3by3 = Rd.transpose() * R - R.transpose() * Rd;
//     //cout<<"inside_vee_3x3\n"<<inside_vee_3by3<<endl;
//     Vector3d vee_3by1;
//     eigen_invskew(inside_vee_3by3, vee_3by1);// 3x1
//     //cout<<"vee_3by1\n"<<vee_3by1.transpose()<<endl;
//     Vector3d eR = 0.5 * vee_3by1;
//     //cout<<"eR\n"<<eR.transpose()<<endl;
// 	if(print_test_variable){
//       printf("eR: %f | %f | %f \n", eR(0), eR(1), eR(2));
//   }
//   // Calculate eW (angular velocity error in body-fixed frame)
//   Vector3d eW =  W -  R.transpose() * Rd * Wd;
//   //cout<<"eW\n"<<eW.transpose()<<endl;
//   // Update integral term of control
//   // Position:
//   eiX = eiX_last + del_t * eX;
//   //cout<<"eiX\n"<<eiX<<endl;
//   eiX_last = eiX;
//   // Attitude:
//   eiR = eiR_last + del_t * eR;// (c2*eR + eW);
//   //cout<<"eiR\n"<<eiR<<endl;
//   eiR_last = eiR;
//   // Calculate 3 DOFs of F (controlled force in body-fixed frame)
//   // MATLAB: F = R'*(-kx*ex-kv*ev-Ki*eiX-m*g*e3+m*xd_2dot);
//   Vector3d A = - kx*eX - kv*eV - kiX*eiX + m*xd_ddot + Vector3d(0,0,-m*g);
//   //cout<<"A\n"<<A.transpose()<<endl;
//   F = R.transpose() * A;
//
//   if(print_F){cout<<"F: "<<F.transpose()<<endl;}
//   //cout<<"F\n"<<F.transpose()<<endl;
//   // Calculate 3 DOFs of M (controlled moment in body-fixed frame)
//   // MATLAB: M = -kR*eR-kW*eW-kRi*eiR+cross(W,J*W)+J*(R'*Rd*Wddot-hat(W)*R'*Rd*Wd);
//   Matrix3d What;
//   eigen_skew(W, What);
//   //cout<<"What\n"<<What<<endl;
//   M = -kR * eR - kW * eW-kiR * eiR + What * J * W + J * (R.transpose() * Rd * Wddot - What * R.transpose() * Rd * Wd);
//   if(print_M){cout<<"M: "<<M.transpose()<<endl;}
//   //cout<<"M\n"<<M.transpose()<<endl;
//   // M[0] = -kR*eR[0]-kW*eW[0]-kiR_now*eiR[0]+What_J_W[0]+J_Jmult[0];
//  // Convert forces & moments to f_i for i = 1:6 (forces of i-th prop)
//   Matrix<double, 6, 1> FM;
//   FM[0] = F[0];
//   FM[1] = F[1];
//   FM[2] = F[2];
//   FM[3] = M[0];
//   FM[4] = M[1];
//   FM[5] = M[2];
//     //cout<<"FM\n"<<FM.transpose()<<endl;
//
//   double kxeX[3], kveV[3], kiXeiX[3], kReR[3], kWeW[3], kiReiR[3];
//   for(int i = 0; i < 3; i++){
//     kxeX[i] = kx*eX[i];
//     kveV[i] = kv*eV[i];
//     kiXeiX[i] = kiX*eiX[i];
//     kReR[i] = kR*eR[i];
//     kWeW[i] = kW*eW[i];
//     kiReiR[i] = kiR*eiR[i];
//   }
//   f = invFMmat * FM;
//   //cout<<"f\n"<<f.transpose()<<endl;
//
//   }
//
