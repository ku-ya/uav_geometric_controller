

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
// void odroid_node::GeometricControl_SphericalJoint_3DOF_eigen(Vector3d Wd, Vector3d Wddot, Vector3d W, Matrix3d R, VectorXd eiR_last){
//   Matrix3d Rd = MatrixXd::Identity(3,3);
//   Vector3d e3(0,0,1), b3(0,0,1), vee_3by1;
//   double l = 0;//.05;// length of rod connecting to the spherical joint
//   Vector3d r = -l * b3;
//   Vector3d F_g = m * g * R.transpose() * e3;
//   Vector3d M_g = r.cross(F_g);
//   //   Calculate eR (rotation matrix error)
//   Matrix3d inside_vee_3by3 = Rd.transpose() * R - R.transpose() * Rd;
//   eigen_invskew(inside_vee_3by3, vee_3by1);// 3x1
//   Vector3d eR = 0.5 * vee_3by1;
//   // Calculate eW (angular velocity error in body-fixed frame)
//   Vector3d eW = W - R.transpose() * Rd * Wd;
//   // Update integral term of control
//   // Attitude:
//   Vector3d eiR = eiR_last + del_t * eR;
//   eiR_last = eiR;
//   // Calculate 3 DOFs of M (controlled moment in body-fixed frame)
//   // MATLAB: M = -kR*eR-kW*eW-kRi*eiR+cross(W,J*W)+J*(R'*Rd*Wddot-hat(W)*R'*Rd*Wd);
//   Matrix3d What;
//   eigen_skew(W, What);
//   Vector3d What_J_W = What * J * W;
//   Vector3d Jmult = R.transpose() * Rd * Wddot - What * R.transpose() * Rd * Wd;
//   Vector3d J_Jmult = J * Jmult;
//   Vector3d M = -kR * eR - kW * eW - kiR * eiR + What_J_W + J_Jmult - M_g;
//   // To try different motor speeds, choose a force in the radial direction
//   double F_req = -m*g;// N
//   // Convert forces & moments to f_i for i = 1:6 (forces of i-th prop)
//   VectorXd FM(6);
//   FM << 0.0, 0.0, F_req, M(0), M(1), M(2);
//   f = invFMmat * FM;
// }
