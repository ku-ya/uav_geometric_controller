#include <odroid/odroid_node.hpp>
#include <tf/transform_datatypes.h>
// User header files
using namespace std;
using namespace Eigen;
odroid_node::odroid_node(){
   m = 1.25; g = 9.81;
   J <<  55710.50413e-7, 617.6577e-7, -250.2846e-7,
         617.6577e-7,  55757.4605e-7, 100.6760e-7,
         -250.2846e-7, 100.6760e-7, 105053.7595e-7;// kg*m^2
   IMU_flag = false;
   // f = VectorXd::Zero(6);
   R_bm <<  1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0;// Markers frame (m) to the body frame (b) (fixed)
   R_ev <<  1.0, 0.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, 0.0, -1.0;// Vicon frame (v) to inertial frame (e) (fixed)
   eiX = VectorXd::Zero(3);
   eiR = VectorXd::Zero(3);
   // Given the UAV arm length of 0.31 m and a prop. angle of 15 deg.
   invFMmat <<  0.0000,    1.2879,   -0.1725,   -0.0000,    1.1132,    0.3071,
            -1.1154,    0.6440,    0.1725,    0.9641,   -0.3420,    0.7579,
            -1.1154,   -0.6440,   -0.1725,   -0.9641,   -0.7712,    1.7092,
            -0.0000,   -1.2879,    0.1725,         0,    1.1132,    0.3071,
            1.1154,   -0.6440,   -0.1725,    0.9641,   -0.3420,    0.7579,
            1.1154,    0.6440,    0.1725,   -0.9641,   -0.7712,    1.7092;
   ROS_INFO("Odroid node initialized");
}
odroid_node::~odroid_node(){};

void odroid_node::print_J(){
    std::cout<<"J: \n"<<J<<std::endl;
}
void odroid_node::print_force(){
    std::cout<<"force: "<<this->f.transpose()<<std::endl;
}
// callback for IMU sensor det
bool odroid_node::getIMU(){return IMU_flag;}
void odroid_node::imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
   W_raw(0) = msg->angular_velocity.x;
   W_raw(1) = msg->angular_velocity.y;
   W_raw(2) = msg->angular_velocity.z;
   W_b = W_raw;
   tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
   tf::Matrix3x3 m(q);
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);
   // roll = 30/180*M_PI;
   // pitch = 0;
   // yaw = 0;
   psi = roll; //msg->orientation.x;
   theta = pitch;//msg->orientation.y;
   phi = yaw; //msg->orientation.z;
   // cout<<"psi: "<<psi<<" theta: "<<theta<<" phi: "<<phi<<endl;
   euler_Rvm(R_vm,Eigen::Vector3d(psi,theta,phi));
   R_eb = R_vm.transpose();
   // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
   if(!IMU_flag){ ROS_INFO("IMU ready");}
   IMU_flag = true;

   if(print_imu){
      printf("IMU: Psi:[%f], Theta:[%f], Phi:[%f] \n", psi, theta, phi);
   }
}
// callback for key Inputs
void odroid_node::key_callback(const std_msgs::String::ConstPtr&  msg){
   std::cout<<*msg<<std::endl;
   // TODO: case function here for changing parameter dynamically. Or just make ros dynamic reconfigure file.
}
// Action for controller
void odroid_node::ctl_callback(){
   VectorXd Wd, Wd_dot;
   Wd = VectorXd::Zero(3); Wd_dot = VectorXd::Zero(3);
   double kiR_now = 0.0;
   // W_b << 0,0.0,0.5;
   // psi = 30/180*M_PI; //msg->orientation.x;
   // theta = 30/180*M_PI;//msg->orientation.y;
   // phi = 30/180*M_PI; //msg->orientation.z;
   // cout<<"psi: "<<psi<<" theta: "<<theta<<" phi: "<<phi<<endl;
   euler_Rvm(R_vm,Eigen::Vector3d(psi,theta,phi));
   R_eb = R_vm.transpose();
   del_t_CADS = 0.01;
   GeometricControl_SphericalJoint_3DOF_eigen(Wd, Wd_dot, W_b, R_eb, del_t_CADS, eiR, kiR_now);
}

// vicon information callback
void odroid_node::vicon_callback(){}

void odroid_node::motor_command(){
   // Execute motor output commands
   for(int i = 0; i < 6; i++){
    printf("Motor %i I2C write command of %i to address %i (%e N).\n", i, thr[i], mtr_addr[i], f[i]);
    tcflush(fhi2c, TCIOFLUSH);
    usleep(500);
    if(ioctl(fhi2c, I2C_SLAVE, mtr_addr[i])<0)
      printf("ERROR: ioctl\n");
    if(MOTOR_ON == false)// set motor speed to zero
      thr[i] = 0;
    else if(MotorWarmup == true)// warm up motors at 20 throttle command
      thr[i] = 20;
    while(write(fhi2c, &thr[i], 1)!=1)
      printf("ERROR: Motor %i I2C write command of %i to address %i (%e N) not sent.\n", i, thr[i], mtr_addr[i], f[i]);
   }
}
void odroid_node::open_I2C(){
   // Open i2c:
   fhi2c = open("/dev/i2c-1", O_RDWR);// Chris
   printf("Opening i2c port...\n");
   if(fhi2c!=3)
    printf("ERROR opening i2c port.\n");
   else
    printf("The i2c port is open.\n");
    usleep(100000);
   tcflush(fhi2c, TCIFLUSH);
   usleep(100000);

   // Call and response from motors
   printf("Checking motors...\n");
   int motornum, motoraddr, motorworks;
   int thr0 = 0;// 0 speed test command
   char PressEnter;

   for(motornum = 1; motornum <= 6; motornum++){
      motoraddr = motornum+40;// 1, 2, 3, ... -> 41, 42, 43, ...
      while(1){
         motorworks = 1;// will remain 1 if all works properly
         if(ioctl(fhi2c, I2C_SLAVE, motoraddr)<0){
            printf("ERROR: motor %i ioctl\n", motornum);
            motorworks = 0;// i2c address not called
         }
         usleep(10);
         if(write(fhi2c, &thr0, 1)!=1){
            printf("ERROR: motor %i write\n", motornum);
            motorworks = 0;
         }
         usleep(10);
         tcflush(fhi2c, TCIOFLUSH);
         if(motorworks == 1){
            printf("Motor %i working...\n", motornum);
         break;
         }
         else{
         printf("Fix motor %i, then press ENTER.\n\n", motornum);
         printf("Note: another i2c device may interupt the signal if\n");
         printf("any i2c wires are attached to unpowered components.\n");
         scanf("%c",&PressEnter);
         break;
         }
      }
   }
   printf("All motors are working.\n");
}
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
   f = invFMmat * FM;

   if(print_f){print_force();}

   OutputMotor(f,thr);
   if(print_thr){
      cout<<"Thruttle motor out: ";
      for(int i = 0;i<6;i++){
      cout<<thr[i]<<", ";} cout<<endl;
   }
}
void odroid_node::GeometricController_6DOF(Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot, Matrix3d Rd, Vector3d Wd, Vector3d Wddot, Vector3d x_e, Vector3d v_e, Vector3d W, Matrix3d R, double del_t,  Vector3d eiX_last, Vector3d eiR_last, double kiX_now, double kiR_now)
  {
    //    // Given the UAV arm length of 0.13 m,
    //    double invFMmat[6][6] = {{ 0,       0.6667, -0.1925,  0,       2.9608, 2.5641},
    //                             {-0.5774,  0.3333,  0.1925,  2.5641, -1.4804, 2.5641},
    //                             {-0.5774, -0.3333, -0.1925, -2.5641, -1.4804, 2.5641},
    //                             { 0,      -0.6667,  0.1925,  0,       2.9608, 2.5641},
    //                             { 0.5774, -0.3333, -0.1925,  2.5641, -1.4804, 2.5641},
    //                             { 0.5774,  0.3333,  0.1925, -2.5641, -1.4804, 2.5641}};
    // Given the UAV arm length of 0.31 m,
    // double invFMmat[6][6] = {  {-0.0000,0.6667,   -0.1925,    0.0000,    1.2416,   1.0753},
    //                            {-0.5774,0.3333,    0.1925,    1.0753,   -0.6208,    1.0753},
    //                            {-0.5774,-0.3333,   -0.1925,   -1.0753,   -0.6208,    1.0753},
    //                            {-0.0000,-0.6667,    0.1925,    0.0000,    1.2416,    1.0753},
    //                             {0.5774,-0.3333,   -0.1925,    1.0753,   -0.6208,    1.0753},
    //                             {0.5774,0.3333,    0.1925,   -1.0753,   -0.6208,    1.0753}};
    // Calculate eX (position error in inertial frame)
    Vector3d eX = x_e - xd;
    // Calculate eV (velocity error in inertial frame)
    Vector3d eV = v_e - xd_dot;
    // Calculate eR (rotation matrix error)
    // Take 9 elements of difference
    Matrix3d inside_vee_3by3 = Rd.transpose() * R - R.transpose() * Rd;
    Vector3d vee_3by1;
    eigen_invskew(inside_vee_3by3, vee_3by1);// 3x1
    Vector3d eR = 0.5 * vee_3by1;
    // Calculate eW (angular velocity error in body-fixed frame)
    Vector3d eW =  W -  R.transpose() * Rd * Wd;
    // Update integral term of control
    // Position:
    Vector3d eiX = eiX_last + del_t * eX;
    // Attitude:
    Vector3d eiR = eiR_last + del_t * eR;// (c2*eR + eW);
    // Calculate 3 DOFs of F (controlled force in body-fixed frame)
    // MATLAB: F = R'*(-kx*ex-kv*ev-Ki*eiX-m*g*e3+m*xd_2dot);
    Vector3d A = - kx*eX - kv*eV - kiX_now*eiX + m*xd_ddot + Vector3d(0,0,-m*g);
    Vector3d F = R.transpose() * A;
    // Calculate 3 DOFs of M (controlled moment in body-fixed frame)
    // MATLAB: M = -kR*eR-kW*eW-kRi*eiR+cross(W,J*W)+J*(R'*Rd*Wddot-hat(W)*R'*Rd*Wd);
    Matrix3d What;
    eigen_skew(W, What);
    Vector3d M = -kR * eR - kW * eW-kiR_now * eiR + What * J * W + J * (R.transpose() * Rd * Wddot - What * R.transpose() * Rd * Wd);
    // M[0] = -kR*eR[0]-kW*eW[0]-kiR_now*eiR[0]+What_J_W[0]+J_Jmult[0];
   //  // Convert forces & moments to f_i for i = 1:6 (forces of i-th prop)
    Matrix<double, 6, 1> FM;
    FM[0] = F[0];
    FM[1] = F[1];
    FM[2] = F[2];
    FM[3] = M[0];
    FM[4] = M[1];
    FM[5] = M[2];

    double kxeX[3], kveV[3], kiXeiX[3], kReR[3], kWeW[3], kiReiR[3];
    for(int i = 0; i < 3; i++){
      kxeX[i] = kx*eX[i];
      kveV[i] = kv*eV[i];
      kiXeiX[i] = kiX_now*eiX[i];
      kReR[i] = kR*eR[i];
      kWeW[i] = kW*eW[i];
      kiReiR[i] = kiR_now*eiR[i];
    }
    f = invFMmat * FM;
 }

void odroid_node::callback(odroid::GainsConfig &config, uint32_t level) {
   ROS_INFO("Reconfigure Request: %f %s %s",
             config.kR,
             config.print_imu?"True":"False",
             config.Motor?"True":"False");
   print_f = config.print_f;
   print_imu = config.print_imu;
   print_thr = config.print_thr;
   // Attitude controller gains
   kR = config.kR;
   kW = config.kW;
   kiR = config.kiR;
   // Position controller gains
   kx = config.kx;
   kv = config.kv;
   kiX = config.kiX;
   MOTOR_ON = config.Motor;
   MotorWarmup = config.MotorWarmup;
  // ROS_INFO("Reconfigure Request: %d %f %s %s %d",
  //           config.int_param, config.double_param,
  //           config.str_param.c_str(),
  //           config.bool_param?"True":"False",
  //           config.size);
}

int main(int argc, char **argv){
   // ros::init(argc, argv, "imu_listener");
   ros::init(argc,argv,"hexacopter");
   ros::NodeHandle nh;
   odroid_node odnode;
   // IMU and keyboard input callback
   ros::Subscriber sub2 = nh.subscribe("imu/imu",100,&odroid_node::imu_callback,&odnode);
   ros::Subscriber sub_key = nh.subscribe("cmd_key", 100, &odroid_node::key_callback, &odnode);

   // dynamic reconfiguration server for gains and print outs
   dynamic_reconfigure::Server<odroid::GainsConfig> server;
   dynamic_reconfigure::Server<odroid::GainsConfig>::CallbackType dyn_serv;
   dyn_serv = boost::bind(&odroid_node::callback, &odnode, _1, _2);
   server.setCallback(dyn_serv);

   // open communication through I2C
   odnode.open_I2C();

   ros::Rate loop_rate(100); // rate for the node loop
   int count = 0;
   while (ros::ok()){
      ros::spinOnce();
      odnode.ctl_callback();
      loop_rate.sleep();
      ++count;
   }
   return 0;
}
