#include <odroid/odroid_node.hpp>
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
}
odroid_node::~odroid_node(){};

void odroid_node::print_J(){
    std::cout<<"J: \n"<<J<<std::endl;
}
void odroid_node::print_f(){
    std::cout<<"J: \n"<<this->f<<std::endl;
}

// callback for IMU sensor deta
// void IMU_callback(const sensor_msgs::Imu::ConstPtr& msg){
    // ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
// }
void odroid_node::imu_Callback(const sensor_msgs::Imu::ConstPtr& msg){
    W_raw[0] = msg->angular_velocity.x;
    W_raw[1] = msg->angular_velocity.y;
    W_raw[2] = msg->angular_velocity.z;
    psi = msg->orientation.x;
    theta = msg->orientation.y;
    phi = msg->orientation.z;
    // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    IMU_flag = true;
}

// callback for key Inputs
void odroid_node::key_callback(const std_msgs::String::ConstPtr&  msg){
   std::cout<<*msg<<std::endl;
}
// Action for controller
void odroid_node::ctl_callback(){
   VectorXd Wd, Wd_dot, W_b;
   Wd = VectorXd::Zero(3); Wd_dot = VectorXd::Zero(3);
   W_b = VectorXd::Zero(3);

   double kR = 1, kW = 1, kiR_now = 0;

   GetControllerGain(&kx, &kv, &kiX, &c1, &kR, &kW, &kiR, &c2);
   attitude_controller(Wd, Wd_dot, W_b, R_eb, del_t_CADS, eiR, eR, eW, eiR, kR, kW, kiR_now, f);
   // print_J();
   // print_f();
}

// vicon information callback
void odroid_node::vicon_callback(){}

void odroid_node::attitude_controller(Vector3d Wd, Vector3d Wddot, Vector3d W, Matrix3d R, double del_t, VectorXd eiR_last, VectorXd eR, VectorXd eW, VectorXd eiR, double kR, double kW, double kiR_now, VectorXd f){
   Matrix3d Rd = MatrixXd::Identity(3,3);
   Vector3d e3(0,0,1), b3(0,0,1) , F_g, r, M_g, vee_3by1, trpRe3;
   double l = 0;//.05;// length of rod connecting to the spherical joint
   r = -l * b3;
   F_g = m * g * R.transpose() * e3;
   M_g = r.cross(F_g);
   //   Calculate eR (rotation matrix error)
   Matrix3d trpRd_R, trpR_Rd, trpR, inside_vee_3by3;
   trpRd_R = Rd.transpose() * R;
   trpR_Rd = R.transpose() * Rd;
   inside_vee_3by3 = trpRd_R - trpR_Rd;
   //  invskew(inside_vee_3by3, vee_3by1);// 3x1
   eR = 0.5 * vee_3by1;
   // Calculate eW (angular velocity error in body-fixed frame)
   eW = W - trpR_Rd * Wd;
   // Update integral term of control
   // Attitude:
   eiR = eiR_last + del_t * eR;
   // Calculate 3 DOFs of M (controlled moment in body-fixed frame)
   // MATLAB: M = -kR*eR-kW*eW-kRi*eiR+cross(W,J*W)+J*(R'*Rd*Wddot-hat(W)*R'*Rd*Wd);
   // skew(W, What);
   Vector3d J_W, What_J_W,Jmult, J_Jmult;
   Matrix3d What;
   What_J_W = What * J * W;
   Jmult = trpR_Rd * Wddot - What * trpR_Rd * Wd;
   J_Jmult = J * Jmult;
   Vector3d M;
   M = -kR * eR - kW * eW - kiR_now * eiR + What_J_W + J_Jmult - M_g;
   // To try different motor speeds, choose a force in the radial direction
   double F_req = -m*g;// N
   // Convert forces & moments to f_i for i = 1:6 (forces of i-th prop)
   VectorXd FM(6);
   FM[0] = 0.0;
   FM[1] = 0.0;
   FM[2] = F_req;
   FM[3] = M[0];
   FM[4] = M[1];
   FM[5] = M[2];

   this->f = invFMmat * FM;
   // std::cout<<"force: \n"<<this->f<<std::endl;
}


int main(int argc, char **argv){
    // ros::init(argc, argv, "imu_listener");
    ros::init(argc,argv,"hexacopter");
    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("imu", 100, IMU_callback);
    odroid_node odnode;
    ros::Subscriber sub2 = nh.subscribe("imu",100,&odroid_node::imu_Callback,&odnode);
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
