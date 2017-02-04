#include <odroid/odroid_node.hpp>
// #include <odroid/controllers.h>// robotic control
// User header files
#include <odroid/controller.hpp>
// #include <odroid/sensor.hpp>
// #include <odroid/visualize.hpp>
// #include <odroid/hw_interface.hpp>
#include <odroid/error.h>
#include <XmlRpcValue.h>
#include <boost/thread.hpp>
using namespace std;
using namespace Eigen;
using namespace message_filters;


void imu_callback(const sensor_msgs::Imu::ConstPtr& msg, odroid_node* node){
  cout<<node->W_b<<endl;
  // vector3Transfer(W_b, msg->angular_velocity);
  // if(!IMU_flag){ ROS_INFO("IMU ready");}
  // IMU_flag = true;
  // if(isnan(W_raw(0)) || isnan(W_raw(1)) || isnan(W_raw(2))){IMU_flag = false;}
  // if(print_imu){
  //  printf("IMU: Psi:[%f], Theta:[%f], Phi:[%f] \n", W_b(0), W_b(1), W_b(2));
  // }
}

void vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
  // vicon_time =msg->header.stamp
  // vector3Transfer(x_v, msg->transform.translation);
  // // x_v << 0, 0, 0;
  // vector4Transfer(quat_vm, msg->transform.rotation);
  //
  // tf::Quaternion q(quat_vm(0),quat_vm(1),quat_vm(2),quat_vm(3));
  // tf::Matrix3x3 m(q);
  // m.getRPY(roll, pitch, yaw);
  //
  // quatToMat(R_v, quat_vm);
  //
	// if(print_vicon){
  //   printf("Vicon: roll:[%f], pitch:[%f], yaw:[%f] \n", roll/M_PI*180, pitch/M_PI*180, yaw/M_PI*180);
  // }
  // if(print_x_v){
  //   cout<<"x_v: "<<x_v.transpose()<<endl;
  // }
  // static tf::TransformBroadcaster br;
  // tf::Transform transform;
  // transform.setOrigin( tf::Vector3(x_v(0),x_v(1), x_v(2)));
  // transform.setRotation(q);
  // br.sendTransform(tf::StampedTransform(transform, vicon_time, "world", "base_link"));
  //
  // if(!Vicon_flag){ ROS_INFO("Vicon ready");}
  // Vicon_flag = true;
}

void get_sensor(odroid_node* node){
  ros::NodeHandle nh_sens;
    // IMU and keyboard input callback
  // ros::Subscriber imu_sub = nh_sens.subscribe("imu/imu",100, boost::bind(imu_callback, _1, node));
  ros::Subscriber vicon_sub = nh_sens.subscribe("vicon/Maya/Maya",100, vicon_callback);
  ros::spin();
}

void control(odroid_node& node){
  ros::Rate loop_rate(100); // rate for the node loop
  while (ros::ok()){
    // ros::spinOnce();
    // if(odnode.getIMU() or odnode.getWarmup()){
    // odnode.ctl_callback(hw_intf);
    // cout<<"control"<<node.W_b<<endl;
    // if(odnode.getEnv() == 0){
    //   controller::gazebo_controll(odnode);
    // }
    // }
    loop_rate.sleep();
  }
}

int main(int argc, char **argv){
  ros::init(argc,argv,"hexacopter");
  odroid_node* odnode;
  ros::NodeHandle nh = odnode->getNH();
  boost::thread subscribe(&get_sensor, odnode);
  // boost::thread command(&control, odnode);
  // IMU and keyboard input callback
  // odnode.imu_sub = nh.subscribe("imu/imu",100, &odroid_node::imu_callback, &odnode);
  // odnode.vicon_sub = nh.subscribe("vicon/Maya/Maya",100,&odroid_node::vicon_callback,&odnode);
  // ros::Subscriber sub_key = nh.subscribe("cmd_key", 100, &odroid_node::key_callback, &odnode);

  // dynamic reconfiguration server for gains and print outs
  // dynamic_reconfigure::Server<odroid::GainsConfig> server;
  // dynamic_reconfigure::Server<odroid::GainsConfig>::CallbackType dyn_serv;
  // dyn_serv = boost::bind(&odroid_node::callback, &odnode, _1, _2);
  // server.setCallback(dyn_serv);

  // visualize vis_pub;
  // vis_pub.publisher_initialization(odnode);
  // open communication through I2C
  hw_interface hw_intf;
  if(odnode->getEnv() == 1){
    hw_intf.open_I2C();
  }
  ros::Rate loop_rate(10); // rate for the node loop
  while (ros::ok()){
    cout<<"node rate"<<endl;
    ros::spinOnce();
    // if(odnode.getIMU() or odnode.getWarmup()){
    odnode->ctl_callback(hw_intf);

    if(odnode->getEnv() == 0){
      // controller::gazebo_controll(&odnode);
    }
    // }
    loop_rate.sleep();
  }
  ros::shutdown();
  subscribe.join();
  return 0;
}

odroid_node::odroid_node(){
  ros::param::get("/controller/del_t",del_t);  cout<<"\ndel_t: "<< del_t<<endl;
  ros::param::get("/controller/g",g);
  ros::param::get("/controller/m",m); cout<<"m: "<< m<<endl;

  std::vector<double> J_vec;
  ros::param::param<std::vector<double>>("/controller/J", J_vec, J_vec);
  J=Matrix3d(J_vec.data());  std::cout<<"J: \n"<<J<<std::endl;

  ros::param::get("/controller/l",l); cout<<"l: "<< l<<endl;
  ros::param::get("/controller/c_tf",c_tf); cout<<"c_tf: "<< c_tf<<endl;
  Ainv = getAinv(l, c_tf);

  cout<<"Ainv:\n\n"<<Ainv<<endl;
  ros::param::get("/environment",environment);
  ros::param::get("/controller/mode",mode);
  cout<<"Mode: "<<mode<<" (0: Attitude, 1: Position)\n"<<endl;
  ros::param::param<std::vector<double>>("/controller/R_bm", J_vec, J_vec);
  R_bm=Matrix3d(J_vec.data());  std::cout<<"R_bm: \n"<<R_bm<<std::endl;

  IMU_flag = false; // IMU sensor reading check
  Vicon_flag = false; // IMU sensor reading check

  R_ev = R_bm;

  prev_x_v= Vector3d::Zero();  prev_v_v = Vector3d::Zero();
  eiX_last = Vector3d::Zero();  eiR_last = Vector3d::Zero();
	x_e = Vector3d::Zero(); eiR = Vector3d::Zero(); eiX = Vector3d::Zero();

  double wnx = 4, zetax = 0.7;
  kx = wnx*wnx*m;
  kv = 2*wnx*zetax*m;

  double wnR = 1, zetaR = 0.7;
  double norm_J = J.lpNorm<Infinity>();
  kR = wnR*wnR*norm_J;
  kW = 2*wnR*zetaR*norm_J;

  printf("Suggested gain from wnx %f wnR %f zeta %f\nkx %f kv %f kR %f kW %f\n\n\n",wnx,wnR,zetax,kx,kv,kR,kW);
  ros::param::get("/controller/gain/att/kp",kR);
  ros::param::get("/controller/gain/att/kd",kW);
  ros::param::get("/controller/gain/att/ki",kiR);
  ros::param::get("/controller/gain/att/kp",cR);

  ros::param::get("/controller/gain/pos/kp",kx);
  ros::param::get("/controller/gain/pos/kd",kv);
  ros::param::get("/controller/gain/pos/ki",kiX);
  ros::param::get("/controller/gain/pos/kp",cX);

  ros::param::get("/controller/saturation/x",eiX_sat);
  ros::param::get("/controller/saturation/R",eiR_sat);

  pub_ = n_.advertise<odroid::error>("/error_values",1);

  ros::Duration(1).sleep();
  ROS_INFO("Odroid node initialized");
}
odroid_node::~odroid_node(){};

void odroid_node::print_J(){
  std::cout<<"J: \n"<<J<<std::endl;
}

void odroid_node::print_force(){
  std::cout<<"force: "<<f_quad<<std::endl;
}


// callback for IMU sensor det
bool odroid_node::getIMU(){return IMU_flag;}
bool odroid_node::getWarmup(){return MotorWarmup;}

void odroid_node::imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  vector3Transfer(W_b, msg->angular_velocity);
  if(!IMU_flag){ ROS_INFO("IMU ready");}
  IMU_flag = true;
  // if(isnan(W_raw(0)) || isnan(W_raw(1)) || isnan(W_raw(2))){IMU_flag = false;}
  if(print_imu){
   printf("IMU: Psi:[%f], Theta:[%f], Phi:[%f] \n", W_b(0), W_b(1), W_b(2));
  }
}

// vicon information callback
void odroid_node::vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
  vicon_time = msg->header.stamp;
  vector3Transfer(x_v, msg->transform.translation);
  // x_v << 0, 0, 0;
  vector4Transfer(quat_vm, msg->transform.rotation);

  tf::Quaternion q(quat_vm(0),quat_vm(1),quat_vm(2),quat_vm(3));
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  quatToMat(R_v, quat_vm);

	if(print_vicon){
    printf("Vicon: roll:[%f], pitch:[%f], yaw:[%f] \n", roll/M_PI*180, pitch/M_PI*180, yaw/M_PI*180);
  }
  if(print_x_v){
    cout<<"x_v: "<<x_v.transpose()<<endl;
  }
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(x_v(0),x_v(1), x_v(2)));
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, vicon_time, "world", "base_link"));

  if(!Vicon_flag){ ROS_INFO("Vicon ready");}
  Vicon_flag = true;
}

// callback for key Inputs
void odroid_node::key_callback(const std_msgs::String::ConstPtr&  msg){
  std::cout<<*msg<<std::endl;
  // TODO: case function here for changing parameter dynamically. Or just make ros dynamic reconfigure file.
}
// Action for controller
void odroid_node::ctl_callback(hw_interface hw_intf){
  VectorXd Wd, Wd_dot;
  Wd = VectorXd::Zero(3); Wd_dot = VectorXd::Zero(3);
  //for attitude testing of position controller
  Vector3d xd_dot, xd_ddot;
  Matrix3d Rd;

  if(MOTOR_ON && !MotorWarmup){
    ros::param::get("/controller/gain/att/ki",kiR);
    ros::param::get("/controller/gain/pos/ki",kiX);
  }else{
    kiR = 0;
    kiX = 0;
  }

  xd_dot = VectorXd::Zero(3); xd_ddot = VectorXd::Zero(3);
  Rd = MatrixXd::Identity(3,3);
  Vector3d prev_x_e = x_e;
  // Vector3d prev_x_v = x_v;
  x_e = R_ev * x_v;
  v_e = (x_e - prev_x_e)*100;

  v_v = ((x_v - prev_x_v)*100)*0.5 + prev_v_v*0.5;
  prev_x_v = x_v;
  prev_v_v = v_v;

  quatToMat(R_vm, quat_vm);

  R_eb = R_ev * R_vm * R_bm;
  if(print_R_eb){cout<<"R_eb: "<<R_eb<<endl;}

  if(print_xd){
    cout<<"xd: "<<xd.transpose()<<endl;
  }

  if(print_gains){
    printf("Gain values: kx %f kv %f kiX %f kR %f kW %f kiR %f\n",kx,kv,kiX,kR,kW,kiR);
  }

  // QuadGeometricPositionController(xd, xd_dot, xd_ddot, Wd, Wd_dot, x_v, v_v, W_b, R_v);
  if(mode == 0){
    controller::GeometricControl_SphericalJoint_3DOF(*this, Wd, Wd_dot, W_b, R_v);
  }else if(mode == 1){
    controller::GeometricPositionController(*this, xd, xd_dot, xd_ddot, Wd, Wd_dot, x_v, v_v, W_b, R_v);
  }
  // controller::GeometricControl_SphericalJoint_3DOF(*this, Wd, Wd_dot, W_b, R_v);

  if(print_f_motor){
    cout<<"f_motor: "<<f_motor.transpose()<<endl;
  }
  for(int k = 0; k < 4; k++){
    thr[k] = floor(1/0.03*(f_motor(k)+0.37)+0.5);
  }

  if(print_thr){
    cout<<"Throttle motor out: ";
    for(int i = 0;i<4;i++){
      cout<<thr[i]<<", ";} cout<<endl;
  }

  double mean = f_motor.mean();
  scale = 0.1;

  // TODO: plave visualization command here
  // viz_pub(*this)

  if(environment == 1){
    hw_intf.motor_command(thr, MotorWarmup, MOTOR_ON);
  }

}

void odroid_node::callback(odroid::GainsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: Update");

  mode = config.mode;
  print_gains = config.print_gains;
  print_f = config.print_f;
  print_imu = config.print_imu;
  print_thr = config.print_thr;

  kR = config.kP;
  kW = config.kW;
  kiR = config.ki;

  if(MOTOR_ON && !MotorWarmup){
    ros::param::get("/controller/gain/att/ki",kiR);
    ros::param::get("/controller/gain/pos/ki",kiX);
  }else{
    kiR = 0;
    kiX = 0;
  }
  mode = config.mode;
  MOTOR_ON = config.Motor;
  MotorWarmup = config.MotorWarmup;
  xd(0) =  config.x;
  xd(1) =  config.y;
  xd(2) =  config.z;

  print_xd = config.print_xd;
  print_x_v = config.print_x_v;
  print_eX = config.print_eX;
  print_eV = config.print_eV;
  print_eR = config.print_eR;print_Rd = config.print_Rd;
  print_eW = config.print_eW;
  print_vicon = config.print_vicon;
  print_M = config.print_M;
  print_F = config.print_F;
  print_f_motor = config.print_f;
  print_R_eb = config.print_R_eb;
}
