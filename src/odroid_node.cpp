#include <odroid/odroid_node.hpp>
// User header files
#include <odroid/controller.hpp>
// #include <odroid/sensor.hpp>
// #include <odroid/visualize.hpp>
// #include <odroid/hw_interface.hpp>
#include <odroid/error.h>
#include <XmlRpcValue.h>

using namespace std;
using namespace Eigen;
using namespace message_filters;

void publish_error(odroid_node& node){
  odroid::error e_msg;
  e_msg.header.stamp = ros::Time::now();
  e_msg.header.frame_id = "drone";
  Vector3d kR_eR = node.kR*node.eR;
  Vector3d kW_eW = node.kW*node.eW;

  geo3toVec(e_msg.x_v, node.x_v); geo3toVec(e_msg.v_v, node.v_v);
  geo3toVec(e_msg.eR, node.eR); geo3toVec(e_msg.eW, node.eW);
  geo3toVec(e_msg.Moment, node.M);  geo3toVec(e_msg.rpy, node.rpy);
  geo3toVec(e_msg.ex, node.eX); geo3toVec(e_msg.IMU, node.W_b);
  geo3toVec(e_msg.ev, node.eV); geo3toVec(e_msg.xd, node.xd);
  e_msg.force = node.f_total;
  e_msg.dt_vicon_imu = (float)node.dt_vicon_imu;
  for(int i = 0; i<4;i++){
    e_msg.throttle[i] = (float) node.thr[i];
    e_msg.f_motor[i] = (float)node.f_motor(i);
  }
  e_msg.gainX = {node.kx, node.kv, node.kiX, node.cX};
  e_msg.gainR = {node.kR, node.kW, node.kiR, node.cR};
  node.pub_.publish(e_msg);
}

int main(int argc, char **argv){
  ros::init(argc,argv,"Xrotor");
  odroid_node odnode;
  ros::NodeHandle nh = odnode.getNH();
  // dynamic reconfiguration server for gains and print outs
  dynamic_reconfigure::Server<odroid::GainsConfig> server;
  dynamic_reconfigure::Server<odroid::GainsConfig>::CallbackType dyn_serv;
  dyn_serv = boost::bind(&odroid_node::callback, &odnode, _1, _2);
  server.setCallback(dyn_serv);

  // visualize vis_pub;
  // vis_pub.publisher_initialization(odnode);
  ros::Duration(1).sleep();
  // IMU and keyboard input callback
  boost::thread subscribe(&odroid_node::get_sensor, &odnode);
  boost::thread command(&odroid_node::control, &odnode);

  ros::Rate loop_rate(100); // rate for the node loop
  while (ros::ok()){
    ros::spinOnce();
    publish_error(odnode);
    loop_rate.sleep();
  }
  subscribe.join();
  command.join();
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

  cout<<"Ainv:\n"<<Ainv<<"\n"<<endl;
  ros::param::get("/environment",environment);
  ros::param::get("/controller/mode",mode);
  cout<<"Mode: "<<mode<<" (0: Attitude, 1: Position)\n"<<endl;
  ros::param::param<std::vector<double>>("/controller/R_bm", J_vec, J_vec);
  R_bm=Matrix3d(J_vec.data());  std::cout<<"R_bm: \n"<<R_bm<<std::endl;

  IMU_flag = false; // IMU sensor reading check
  Vicon_flag = false; // IMU sensor reading check

  R_ev = R_bm;
  R_v = Matrix3d::Zero();

  prev_x_v= prev_v_v = eiX =  eiX_last = eiR_last = Vector3d::Zero();
	x_e = v_e = eiR = eiX = Vector3d::Zero();
  xd = xd_dot = xd_ddot= Wd = Wd_dot = W_b = W_raw = Vector3d::Zero();
  x_v = v_v = prev_x_v = prev_v_v = Vector3d::Zero();
  quat_vm = f_motor =  Vector4d::Zero();

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
  ros::param::get("/controller/gain/att/c",cR);
  ros::param::get("/controller/gain/pos/kp",kx);
  ros::param::get("/controller/gain/pos/kd",kv);
  ros::param::get("/controller/gain/pos/ki",kiX);
  ros::param::get("/controller/gain/pos/c",cX);
  ros::param::get("/controller/saturation/x",eiX_sat);
  ros::param::get("/controller/saturation/R",eiR_sat);

  pub_ = n_.advertise<odroid::error>("/drone_variable",1);
  ROS_INFO("Odroid node initialized");
}

odroid_node::~odroid_node(){};

void odroid_node::print_J(){
  std::cout<<"J: \n"<<J<<std::endl;
}

void odroid_node::print_force(){
  std::cout<<"force: "<<f_total<<std::endl;
}

// callback for IMU sensor det
bool odroid_node::getIMU(){return IMU_flag;}
bool odroid_node::getWarmup(){return MotorWarmup;}

void odroid_node::imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  if(!IMU_flag){ ROS_INFO("IMU ready");}
  IMU_flag = true;
  dt_imu = (msg->header.stamp - imu_time).toSec();
  imu_time = msg->header.stamp;
  dt_vicon_imu = (imu_time - vicon_time).toSec();
  boost::mutex::scoped_lock scopedLock(mutex_);
  vector3Transfer(W_b, msg->angular_velocity);
}

void odroid_node::vicon_callback(const geometry_msgs::TransformStamped::ConstPtr& msg){
  if(!Vicon_flag){ ROS_INFO("Vicon ready");}
  Vicon_flag = true;
  dt_vicon = (msg->header.stamp - vicon_time).toSec();
  vicon_time = msg->header.stamp;
  Eigen::Affine3d pose;
  tf::transformMsgToEigen(msg->transform,pose);
  boost::mutex::scoped_lock scopedLock(mutex_);
  x_v  = pose.matrix().block<3,1>(0,3);
  R_v = pose.matrix().block<3,3>(0,0);
}

void odroid_node::cmd_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  Eigen::Affine3d pose;
  tf::poseMsgToEigen(msg->pose,pose);
  xd  = pose.matrix().block<3,1>(0,3);
  Rd = pose.matrix().block<3,3>(0,0);
}

void odroid_node::get_sensor(){
  ros::NodeHandle nh_sens;
    // IMU and keyboard input callback
  ros::Subscriber imu_sub = nh_sens.subscribe("imu/imu",100, &odroid_node::imu_callback, this);
  ros::Subscriber vicon_sub = nh_sens.subscribe("vicon/Maya/Maya",100, &odroid_node::vicon_callback, this);
  ros::Subscriber cmd_sub = nh_sens.subscribe("xd",100, &odroid_node::cmd_callback, this);
  ros::spin();
}

void odroid_node::control(){
  hw_interface hw_intf;  // open communication through I2C
  if(getEnv() == 1) hw_intf.open_I2C();
  ros::Rate loop_rate(100); // rate for the node loop
  while (ros::ok()){
    if(getIMU() or getWarmup()){
      ctl_callback(hw_intf);
    }
    loop_rate.sleep();
  }
}

// Action for controller
void odroid_node::ctl_callback(hw_interface hw_intf){
  VectorXd Wd, Wd_dot;
  Wd = VectorXd::Zero(3); Wd_dot = VectorXd::Zero(3);
  //for attitude testing of position controller
  Vector3d xd_dot, xd_ddot; Matrix3d Rd;

  xd_dot = VectorXd::Zero(3); xd_ddot = VectorXd::Zero(3);
  Rd = MatrixXd::Identity(3,3);

  v_v = ((x_v - prev_x_v)*100)*0.5 + prev_v_v*0.5;
  prev_x_v = x_v;
  prev_v_v = v_v;


  boost::mutex::scoped_lock scopedLock(mutex_);
  controller::GeometricPositionController(*this, xd, xd_dot, xd_ddot, Wd, Wd_dot, x_v, v_v, W_b, R_v);

  for(int k = 0; k < 4; k++){
    if(f_motor(k) < 0 ){f_motor(k)=0;}
    else if(f_motor(k) > 6.2){f_motor(k) = 6.2;}

    thr[k] = round(1/0.03*(f_motor(k)+0.37));
  }
  if(environment == 1){
    hw_intf.motor_command(thr, MotorWarmup, MOTOR_ON);
  }else{
    controller::gazebo_control(*this);
  }
}

void odroid_node::callback(odroid::GainsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: Update");

  mode = config.mode;
  MOTOR_ON = config.Motor;
  MotorWarmup = config.MotorWarmup;

  xd(0) =  config.x;
  xd(1) =  config.y;
  xd(2) =  config.z;

  kR = config.kR;
  kW = config.kW;

  kx = config.kx;
  kv = config.kv;

  if(MOTOR_ON && !MotorWarmup){
    kiR = config.kiR;
    kiX = config.kiX;
  }else{
    kiR = 0;
    kiX = 0;
  }
}
