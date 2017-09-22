#include <uav_geometric_controller/node.hpp>
// User header files
#include <uav_geometric_controller/controller.hpp>
#include <uav_geometric_controller/states.h>
#include <unistd.h>
using namespace std;
using namespace Eigen;
using namespace message_filters;

void publish_error(node& node){
 // UAV states publisher
  uav_geometric_controller::states e_msg;
  e_msg.header.stamp = ros::Time::now();
  e_msg.header.frame_id = "drone";
  Vector3d kR_eR = node.kR*node.eR;
  Vector3d kW_eW = node.kW*node.eW;

  tf::vectorEigenToMsg(node.x_v, e_msg.x_v);
  tf::vectorEigenToMsg(node.v_v, e_msg.v_v);
  tf::vectorEigenToMsg(node.b1d, e_msg.b1d);
  tf::vectorEigenToMsg(node.eR, e_msg.eR);
  tf::vectorEigenToMsg(node.eW, e_msg.eW);
  tf::vectorEigenToMsg(node.M, e_msg.moment);
  tf::vectorEigenToMsg(node.eX, e_msg.ex);
  tf::vectorEigenToMsg(node.W_b, e_msg.w_imu);
  tf::vectorEigenToMsg(node.eV, e_msg.ev);
  tf::vectorEigenToMsg(node.xd, e_msg.xc);
  tf::vectorEigenToMsg(node.xd_dot, e_msg.xc_dot);
  tf::vectorEigenToMsg(node.xd_ddot, e_msg.xc_2dot);
  tf::vectorEigenToMsg(node.xc_ned, e_msg.xc_ned);
  tf::vectorEigenToMsg(node.xc_ned_dot, e_msg.xc_ned_dot);
  tf::vectorEigenToMsg(node.xc_ned_2dot, e_msg.xc_ned_2dot);
  tf::vectorEigenToMsg(node.Wc, e_msg.Wc);
  tf::vectorEigenToMsg(node.Wc_dot, e_msg.Wc_dot);
  tf::vectorEigenToMsg(node.v, e_msg.v);
  tf::vectorEigenToMsg(node.x, e_msg.x);
  tf::vectorEigenToMsg(node.W, e_msg.W);
  tf::quaternionEigenToMsg(node.q_v, e_msg.q_v);
  tf::quaternionEigenToMsg(node.q_imu, e_msg.q_imu);
  e_msg.force = node.f_total;
  for(int i = 0; i<4;i++){
    e_msg.throttle[i] = (float) node.thr[i];
    e_msg.f_motor[i] = (float)node.f_motor(i);
    e_msg.f_motor_sat[i] = (float)node.f_motor_sat(i);
  }
 for(int i = 0;i<24;i++){
     if(node.motor_power == NULL){
        cout<<"motor power is pointed at NULL"<<endl;
     }else{
		e_msg.motor_power[i] = (float)node.motor_power[i];
    }
    }
  e_msg.R_v.data.clear();
  for(int i=0;i<9;i++){
    e_msg.Rc[i] = (float)node.Rc(i);
    e_msg.Rc_dot[i] = (float)node.Rc_dot(i);
    e_msg.Rc_2dot[i] = (float)node.Rc_2dot(i);
    e_msg.R_imu[i] = (float)node.R_imu(i);
    e_msg.R[i] = (float)node.R(i);
    e_msg.R_v.data.push_back((float)node.R_b(i));
  }
  e_msg.gain_position =
    {node.kx, node.kv, node.kiX, node.kxr, node.cX, node.eiX_sat,0,0,0};
  e_msg.gain_attitude =
    {node.kR, node.kW, node.kiR, node.kRr, node.cR, node.eiR_sat, 0,0,0};
  for(int i = 0; i <3;i++){
    e_msg.gain_position[6+i] = node.eiX[i];
    e_msg.gain_attitude[6+i] = node.eiR[i];

  }
  node.pub_.publish(e_msg);
}

int main(int argc, char **argv){
    // Main node for the controller
  ros::init(argc,argv,"Xrotor");
  node odnode;
  ros::NodeHandle nh = odnode.getNH();
  // dynamic reconfiguration server for gains and print outs
  dynamic_reconfigure::Server<uav_geometric_controller::GainsConfig> server;
  dynamic_reconfigure::Server<uav_geometric_controller::GainsConfig>::CallbackType dyn_serv;
  dyn_serv = boost::bind(&node::callback, &odnode, _1, _2);
  server.setCallback(dyn_serv);

  // visualize vis_pub;
  // vis_pub.publisher_initialization(odnode);
  ros::Duration(1).sleep();
  // IMU and keyboard input callback
  boost::thread subscribe(&node::get_sensor, &odnode);
  boost::thread command(&node::control, &odnode);

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

node::node(){
    // Controller node initialization
  ros::param::get("controller/del_t",del_t);  cout<<"\ndel_t: "<< del_t<<endl;
  ros::param::get("controller/g",g);
  ros::param::get("controller/m",m); cout<<"m: "<< m<<endl;

  std::vector<double> J_vec;
  ros::param::param<std::vector<double>>("controller/J", J_vec, J_vec);
  J=Matrix3d(J_vec.data());  std::cout<<"J: \n"<<J<<std::endl;

  ros::param::get("controller/l",l); cout<<"l: "<< l<<endl;
  ros::param::get("controller/c_tf",c_tf); cout<<"c_tf: "<< c_tf<<endl;

  Matrix4d A;
  A << 1.0,   1.0,  1.0,   1.0,
       0.0,   -l,   0.0,   l,
       l,     0.0,  -l,    0.0,
       c_tf, -c_tf, c_tf, -c_tf;

  Ainv = A.inverse();

  cout<<"Ainv:\n"<<Ainv<<"\n"<<endl;
  ros::param::get("environment",environment);
  ros::param::get("controller/mode",mode);
  cout<<"Mode: "<<mode<<" (0: Attitude, 1: Position)\n"<<endl;
  ros::param::param<std::vector<double>>("controller/R_conv", J_vec, J_vec);
  R_conv=Matrix3d(J_vec.data());  std::cout<<"R_conv: \n"<<R_conv<<std::endl;

  IMU_flag = false; // IMU sensor reading check
  Vicon_flag = false; // IMU sensor reading check

  R_b = R_imu =  R = Matrix3d::Identity();

  prev_x_v= prev_v_v = eiX =  eiX_last = eiR_last = Vector3d::Zero();
  eV = eX = x_e = v_e = eiR = eiX = eR =  Vector3d::Zero();
  xd = xd_dot = xd_ddot= Wd = Wd_dot = W_b = W_raw = Vector3d::Zero();
  x_v = v_v = prev_x_v = prev_v_v = b1d = b1d_ned =  Vector3d::Zero();
  W = xc_ned = xc_ned_dot = xc_ned_2dot = x = v = Vector3d::Zero();
  R_v_ned = Matrix3d::Identity();
  x = v = Wc = Wc_dot = Vector3d::Zero();
  b1d << 1,0,0;
  b1d_ned = R_conv*b1d;
  M = eX = eV = eR = eW = Vector3d::Zero();
  f_motor =  Vector4d::Zero();
  Rc = Matrix3d::Identity();
  Rc_dot = Rc_2dot = Matrix3d::Zero();
  v_ave = MatrixXd::Zero(3,10);
  q_v=q_imu=Quaterniond(0,0,0,1);
  f_total = 0.0;

  double wnx = 4, zetax = 0.7;
  kx = wnx*wnx*m;
  kv = 2*wnx*zetax*m;

  double wnR = 1, zetaR = 0.7;
  double norm_J = J.lpNorm<Infinity>();
  kR = wnR*wnR*norm_J;
  kW = 2*wnR*zetaR*norm_J;

  printf("Suggested gain from wnx %f wnR %f zeta %f\nkx %f kv %f kR %f kW %f\n\n\n",wnx,wnR,zetax,kx,kv,kR,kW);
  ros::param::get("controller/gain/att/kp",kR);
  ros::param::get("controller/gain/att/kRr",kRr);
  ros::param::get("controller/gain/att/kd",kW);
  ros::param::get("controller/gain/att/ki",kiR);
  ros::param::get("controller/gain/att/c",cR);
  ros::param::get("controller/gain/pos/kp",kx);
  ros::param::get("controller/gain/pos/kxr",kxr);
  ros::param::get("controller/gain/pos/kd",kv);
  ros::param::get("controller/gain/pos/ki",kiX);
  ros::param::get("controller/gain/pos/c",cX);
  ros::param::get("controller/saturation/x",eiX_sat);
  ros::param::get("controller/saturation/R",eiR_sat);
  ros::param::param<std::vector<int>>("port/address",mtr_addr,mtr_addr);
  ros::param::get("port/i2c",i2c_port);
  ros::param::get("name/vicon",vicon_name);
  ros::param::get("name/imu",imu_name);
  ros::param::get("name/xc",xc_name);
  pub_ = n_.advertise<uav_geometric_controller::states>("uav_states",1);
  pub_imu_ = n_.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
  ROS_INFO("UAV node initialized");
}

// callback for IMU sensor det
void node::get_sensor(){
    // Subscriber for all the topics
    ros::NodeHandle nh_sens;
    // IMU and keyboard input callback
    ros::Subscriber imu_sub =
    nh_sens.subscribe(imu_name,10, &node::imu_callback, this);
    ros::Subscriber vicon_sub =
    nh_sens.subscribe(vicon_name,10, &node::vicon_callback, this);
    ros::Subscriber cmd_sub =
    nh_sens.subscribe(xc_name,10, &node::cmd_callback, this);
    ros::spin();
}

bool node::getIMU(){return IMU_flag;}
bool node::getWarmup(){return MotorWarmup;}

void node::imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
    // IMU message subscriber to read in sensor data from the IMU driver
  if(!IMU_flag){ ROS_INFO("IMU ready");}
  IMU_flag = true;
  dt_imu = (msg->header.stamp - imu_time).toSec();
  imu_time = msg->header.stamp;
  dt_vicon_imu = (imu_time - vicon_time).toSec();
  tf::Quaternion temp;
  tf::quaternionMsgToTF(msg->orientation,temp);
  tf::quaternionMsgToEigen(msg->orientation,q_imu);
  tf::Matrix3x3 m(temp);
  tf::matrixTFToEigen(m, R_imu);

  tf::Matrix3x3 Mimu;
  Mimu.setEulerYPR(0.7854, 0, 0);
  Matrix3d MimuEig;
  tf::matrixTFToEigen(Mimu, MimuEig);
  tf::matrixEigenToTF(MimuEig*R, m);

  if(IMU_as_attitude)
  {
      R = MimuEig * R_imu;
  }


  m.getRotation(temp);
  sensor_msgs::Imu imu_out;
  imu_out.header = msg->header;
  imu_out.header.frame_id = "uav";
  // tf::quaternionTFToMsg(temp, imu_out.orientation);
  // cout<<temp<<endl;
  Vector3d acc(msg->linear_acceleration.x, msg->linear_acceleration.y,
    msg->linear_acceleration.z);
  tf::vectorEigenToMsg(MimuEig*acc, imu_out.linear_acceleration);
  pub_imu_.publish(imu_out);

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  boost::mutex::scoped_lock scopedLock(mutex_);
  Vector3d W_b_;
  tf::vectorMsgToEigen(msg->angular_velocity, W_b_);
  W_b = MimuEig * W_b_;
  rpy << roll, pitch, yaw;
}

void node::vicon_callback(
    const geometry_msgs::PoseStamped::ConstPtr& msg){
    // Vicon senser message subscriber
    // controller_flag = true;
  if(!Vicon_flag){ ROS_INFO("Vicon ready");}
  Vicon_flag = true;
  dt_vicon = (msg->header.stamp - vicon_time).toSec();
  vicon_time = msg->header.stamp;
  Eigen::Affine3d pose;
  tf::quaternionMsgToEigen(msg->pose.orientation, q_v);
  tf::poseMsgToEigen(msg->pose,pose);
  boost::mutex::scoped_lock scopedLock(mutex_);
  x_v  = pose.matrix().block<3,1>(0,3);
  R_b = pose.matrix().block<3,3>(0,0);
  v_v = (x_v - prev_x_v)/dt_vicon;
  vec_average(v_ave, v_v);
  // TODO: change to weighted or whatever
  v_v = v_ave.rowwise().mean();
  prev_x_v = x_v;
  // prev_v_v = v_v;
  x_v_ned = R_conv*x_v;
  v_v_ned = R_conv*v_v;
  R_v_ned = R_conv*R_b*R_conv;
  R = R_v_ned;

}

void node::cmd_callback(const uav_geometric_controller::trajectory::ConstPtr& msg){
  ros::param::get("uav/Motor", MOTOR_ON);
  ros::param::get("uav/MotorWarmup", MotorWarmup);
  // Desired command subscriber
  boost::mutex::scoped_lock scopedLock(mutex_);
  //tf::vectorMsgToEigen(msg->b1,b1d);
  //tf::vectorMsgToEigen(msg->xd,xd);
  //tf::vectorMsgToEigen(msg->xd_dot,xd_dot);
  //tf::vectorMsgToEigen(msg->xd_ddot,xd_ddot);
  b1d << msg->b1[0], msg->b1[1], msg->b1[2];
  b1d_ned = R_conv*b1d;
  xd << msg->xc[0], msg->xc[1], msg->xc[2];
  xd_dot << msg->xc_dot[0], msg->xc_dot[1], msg->xc_dot[2];
  xd_ddot << msg->xc_2dot[0], msg->xc_2dot[1], msg->xc_2dot[2];
  xc_ned = R_conv*xd;
  xc_ned_dot = R_conv*xd_dot;
  xc_ned_2dot = R_conv*xd_ddot;
}


void node::control(){
    // Controller 100Hz command sent to motor speed controller
  hw_interface hw_intf(i2c_port.c_str(), mtr_addr);  // open communication through I2C
  if(getEnv() == 1) hw_intf.open_I2C();
  ros::Rate loop_rate(100); // rate for the node loop
  ROS_INFO("Controller thread loop stapting");
  while (ros::ok()){
    // if(getIMU() or getWarmup()){
      ctl_callback(hw_intf);
    // }
    loop_rate.sleep();
  }
}
// Action for controller
void node::ctl_callback(hw_interface hw_intf){
  VectorXd Wd, Wd_dot;
  Wd = VectorXd::Zero(3); Wd_dot = VectorXd::Zero(3);

  // if(Vicon_flag){
  {
    boost::mutex::scoped_lock scopedLock(mutex_);
    if(mode == 1)
    {
        controller::GeometricPositionController(*this,
            xc_ned, xc_ned_dot, xc_ned_2dot, Wd, Wd_dot, x_v_ned, v_v_ned, W_b, R);
    }
    else
    {
        controller::GeometricControl_SphericalJoint_3DOF(*this,
            Wd, Wd_dot, W_b, R);
    }
  }
  for(int k = 0; k < 4; k++){
    f_motor_sat(k) = f_motor(k);
    if(f_motor(k) < 0 ){f_motor_sat(k)=0;}
    else if(f_motor(k) > 14.0){f_motor_sat(k) = 14.0;}
    thr[k] = floor((-1.054*pow(f_motor_sat(k),2)+f_motor_sat(k)*31.59+17.97));
  }
  if(environment == 1){
    motor_power = hw_intf.motor_command(thr, MotorWarmup, MOTOR_ON);
     }else{
    controller::gazebo_control(*this);
  }
}

void node::callback(uav_geometric_controller::GainsConfig &config, uint32_t level) {
    // Dynamic reconfiguration of the gains
  ROS_INFO("Reconfigure Request: Update");

  mode = config.mode;
  m = config.m;
  // MOTOR_ON = config.Motor;
  // MotorWarmup = config.MotorWarmup;
  xd(0) =  config.x;
  xd(1) =  config.y;
  xd(2) =  config.z;

  xc_ned = R_conv*xd;

  kR = config.kR;
  kW = config.kW;
  kRr = config.kRr;
  kx = config.kx;
  kv = config.kv;
  //kxr = config.kxr;

  kiR = config.kiR; //config.kiR;
  kiX = config.kiX; //config.kiX

  MOTOR_ON = config.Motor;
  MotorWarmup = config.MotorWarmup;
  IMU_as_attitude = config.IMU_as_attitude;
}
