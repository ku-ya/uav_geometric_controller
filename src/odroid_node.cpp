#include <odroid/odroid_node.hpp>
#include <tf/transform_datatypes.h>

// User header files
using namespace std;

odroid_node::odroid_node(){
   m = 1.25;
   g = 9.81;
   J <<  55710.50413e-7, 617.6577e-7, -250.2846e-7,
            617.6577e-7,  55757.4605e-7, 100.6760e-7,
         -250.2846e-7, 100.6760e-7, 105053.7595e-7;// kg*m^2
   IMU_flag = false;

   f = VectorXd::Zero(6);
   R_bm <<  1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0;// Markers frame (m) to the body frame (b) (fixed)
   R_ev <<  1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0;// Vicon frame (v) to inertial frame (e) (fixed)
   eiX = VectorXd::Zero(3);
   eiR = VectorXd::Zero(3);
   // Given the UAV arm length of 0.31 m and a prop. angle of 15 deg.
   // invFMmat
   MatrixXd temp(6,6);
   temp <<  0.0000,    1.2879,   -0.1725,   -0.0000,    1.1132,    0.3071,
            -1.1154,    0.6440,    0.1725,    0.9641,   -0.3420,    0.7579,
            -1.1154,   -0.6440,   -0.1725,   -0.9641,   -0.7712,    1.7092,
            -0.0000,   -1.2879,    0.1725,         0,    1.1132,    0.3071,
            1.1154,   -0.6440,   -0.1725,    0.9641,   -0.3420,    0.7579,
            1.1154,    0.6440,    0.1725,   -0.9641,   -0.7712,    1.7092;
   invFMmat = temp;
   ROS_INFO("Odroid node initialized");
}
odroid_node::~odroid_node(){};

void odroid_node::print_J(){
    std::cout<<"J: \n"<<J<<std::endl;
}
void odroid_node::print_f(){
    std::cout<<"f: \n"<<this->f.transpose()<<std::endl;
}

// callback for IMU sensor deta
// void IMU_callback(const sensor_msgs::Imu::ConstPtr& msg){
    // ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
// }
void odroid_node::imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
   W_raw(0) = msg->angular_velocity.x;
   W_raw(1) = msg->angular_velocity.y;
   W_raw(2) = msg->angular_velocity.z;

   W_b = W_raw;
   tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
   tf::Matrix3x3 m(q);
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);
   roll = 13.433000;
   pitch = -17.971001;
   yaw = -42.332001;
   psi = roll; //msg->orientation.x;
   theta = pitch;//msg->orientation.y;
   phi = yaw; //msg->orientation.z;
   // cout<<"psi: "<<psi<<" theta: "<<theta<<" phi: "<<phi<<endl;
   R_vm(0,0) = cos(theta)*cos(psi);
   R_vm(0,1) = cos(theta)*sin(psi);
   R_vm(0,2) = -sin(theta);
   R_vm(1,0) = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
   R_vm(1,1) = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
   R_vm(1,2) = sin(phi)*cos(theta);
   R_vm(2,0) = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
   R_vm(2,1) = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
   R_vm(2,2) = cos(phi)*cos(theta);

   R_eb = R_vm.transpose();

   // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
   if(!IMU_flag){ ROS_INFO("IMU ready");}
   IMU_flag = true;
}

// callback for key Inputs
void odroid_node::key_callback(const std_msgs::String::ConstPtr&  msg){
   std::cout<<*msg<<std::endl;
}
// Action for controller
void odroid_node::ctl_callback(){
   VectorXd Wd, Wd_dot;
   Wd = VectorXd::Zero(3); Wd_dot = VectorXd::Zero(3);

   // double kR, kW, kiR_now = 0;
   double kiR_now = 0;
   GetControllerGain(&kx, &kv, &kiX, &c1, &kR, &kW, &kiR, &c2);
   // std::cout<<R_eb<<std::endl;
   del_t_CADS = 0.01;
   GeometricControl_SphericalJoint_3DOF_eigen(Wd, Wd_dot, W_b, R_eb, del_t_CADS, eiR, kiR_now);
   double dRd[3][3],  dWd[3],  dWddot[3],   dW[3],  dR[3][3],   deiR_last[3],  deR[3],  deW[3],  deiR[3], dkiR_now,  dJ[3][3],  df[6];
   for(int i = 0;i<W_b.size();i++){dW[i] = W_b(i);}
   for(int i = 0;i<3;i++){
      for(int j = 0; j < 3; j++){
         dR[i][j] = R_eb(i,j);
         dJ[i][j] = J(i,j);
      }
   }

   GeometricControl_SphericalJoint_3DOF(dRd, dWd, dWddot, dW,  dR,  del_t_CADS,  deiR_last, deR,  deW,  deiR,  kR,  kW,  dkiR_now,  m,  g,  dJ,  df);
   for(int i = 0;i<6;i++){
   cout<<df[i]<<",";
   }
   cout<<endl;
   // print_J();
   print_f();
}

// vicon information callback
void odroid_node::vicon_callback(){}

void odroid_node::GeometricControl_SphericalJoint_3DOF_eigen(Vector3d Wd, Vector3d Wddot, Vector3d W, Matrix3d R, double del_t, VectorXd eiR_last, double kiR_now){
   Matrix3d Rd = MatrixXd::Identity(3,3);
   Vector3d e3(0,0,1), b3(0,0,1), vee_3by1;
   double l = 0;//.05;// length of rod connecting to the spherical joint
   Vector3d r = -l * b3;
   Vector3d F_g = m * g * R.transpose() * e3;
   Vector3d M_g = r.cross(F_g);
   //   Calculate eR (rotation matrix error)
   Matrix3d inside_vee_3by3 = Rd.transpose() * R - R.transpose() * Rd;
   eigen_invskew(inside_vee_3by3, vee_3by1);// 3x1
   Vector3d eR = 0.5 * vee_3by1;
   // Calculate eW (angular velocity error in body-fixed frame)
   Vector3d eW = W - R.transpose() * Rd * Wd;
   // Update integral term of control
   // Attitude:
   Vector3d eiR = eiR_last + del_t * eR;
   eiR_last = eiR;
   // Calculate 3 DOFs of M (controlled moment in body-fixed frame)
   // MATLAB: M = -kR*eR-kW*eW-kRi*eiR+cross(W,J*W)+J*(R'*Rd*Wddot-hat(W)*R'*Rd*Wd);
   Matrix3d What;
   eigen_skew(W, What);
   Vector3d What_J_W = What * J * W;
   Vector3d Jmult = R.transpose() * Rd * Wddot - What * R.transpose() * Rd * Wd;
   Vector3d J_Jmult = J * Jmult;
   Vector3d M = -kR * eR - kW * eW - kiR_now * eiR + What_J_W + J_Jmult - M_g;
   // To try different motor speeds, choose a force in the radial direction
   double F_req = -m*g;// N
   // Convert forces & moments to f_i for i = 1:6 (forces of i-th prop)
   VectorXd FM(6);
   FM(0) = 0.0; FM(1) = 0.0; FM(2) = F_req;
   FM(3) = M(0); FM(4) = M(1); FM(5) = M(2);
   this->f = invFMmat * FM;
   // std::cout<<"force: \n"<<this->f<<std::endl;
}


int main(int argc, char **argv){
   // ros::init(argc, argv, "imu_listener");
   ros::init(argc,argv,"hexacopter");
   ros::NodeHandle nh;
   odroid_node odnode;
   ros::Subscriber sub2 = nh.subscribe("raw_imu",100,&odroid_node::imu_callback,&odnode);
   ros::Subscriber sub_key = nh.subscribe("cmd_key", 100, &odroid_node::key_callback, &odnode);

   ros::Rate loop_rate(10);
   int count = 0;
   while (ros::ok()){
      ros::spinOnce();
      odnode.ctl_callback();
      loop_rate.sleep();
      // std::cout<<count<<std::endl;
      ++count;
   }
   return 0;
}
