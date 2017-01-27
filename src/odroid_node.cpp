#include <odroid/odroid_node.hpp>
// #include <odroid/controllers.h>// robotic control
// User header files
#include <XmlRpcValue.h>

using namespace std;
using namespace Eigen;
using namespace message_filters;

int main(int argc, char **argv){
  // ros::init(argc, argv, "imu_listener");
  ros::init(argc,argv,"hexacopter");
  //  ros::NodeHandle nh;
  odroid_node odnode;
  ros::NodeHandle nh = odnode.getNH();
  // IMU and keyboard input callback
  ros::Subscriber sub2 = nh.subscribe("imu/imu",100,&odroid_node::imu_callback,&odnode);
  ros::Subscriber sub_vicon = nh.subscribe("vicon/Maya/Maya",100,&odroid_node::vicon_callback,&odnode);
  // ros::Subscriber sub_key = nh.subscribe("cmd_key", 100, &odroid_node::key_callback, &odnode);

  // dynamic reconfiguration server for gains and print outs
  dynamic_reconfigure::Server<odroid::GainsConfig> server;
  dynamic_reconfigure::Server<odroid::GainsConfig>::CallbackType dyn_serv;
  dyn_serv = boost::bind(&odroid_node::callback, &odnode, _1, _2);
  server.setCallback(dyn_serv);

 // typedef sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::TransformStamped> MySyncPolicy;
 //
 //  message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh,"imu/imu", 100);
 //  message_filters::Subscriber<geometry_msgs::TransformStamped> vicon_sub(nh,"vicon/Maya/Maya", 100);
 //  Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), imu_sub, vicon_sub);
 //  sync.registerCallback(boost::bind(&odroid_node::imu_vicon_callback, &odnode, _1, _2));

  // open communication through I2C
  ros::param::get("simulation",odnode.simulation);
  if(!odnode.simulation){
    odnode.open_I2C();
  }
  ros::Rate loop_rate(100); // rate for the node loop
  while (ros::ok()){
    ros::spinOnce();
    if(odnode.getIMU() or odnode.getWarmup()){
      odnode.ctl_callback();

      if(odnode.simulation){
        odnode.gazebo_controll();
      }
    }
    loop_rate.sleep();
  }
  return 0;
}

odroid_node::odroid_node(){
  ros::param::get("/controller/del_t",del_t);  cout<<"\ndel_t: "<< del_t<<endl;
  ros::param::get("/controller/g",g);
  ros::param::get("/controller/m",m); cout<<"m: "<< m<<endl;
  std::vector<double> J_vec;
  ros::param::param<std::vector<double>>("/controller/J", J_vec, J_vec);
  J=Matrix3d(J_vec.data());  std::cout<<"J: \n"<<J<<std::endl;

  double l, c_tf;
  ros::param::get("/controller/l",l); cout<<"l: "<< l<<endl;
  ros::param::get("/controller/c_tf",c_tf); cout<<"c_tf: "<< c_tf<<endl;

  Matrix4d A;
  A << 1.0,   1.0,  1.0,   1.0,
       0.0,   -l,   0.0,   l,
       l,     0.0,  -l,    0.0,
       -c_tf, c_tf, -c_tf, c_tf;

  Ainv = A.inverse();
  cout<<"Ainv:\n\n"<<Ainv<<endl;

  ros::param::get("/controller/mode",mode);
  cout<<"Mode: "<<mode<<" (0: Attitude, 1: Position)\n"<<endl;
  ros::param::param<std::vector<double>>("/controller/R_bm", J_vec, J_vec);
  R_bm=Matrix3d(J_vec.data());  std::cout<<"R_bm: \n"<<R_bm<<std::endl;

  IMU_flag = false; // IMU sensor reading check
  Vicon_flag = false; // IMU sensor reading check

  R_ev = R_bm;

  prev_x_v= Vector3d::Zero();  prev_v_v = Vector3d::Zero();
  eiX_last = Vector3d::Zero();  eiR_last = Vector3d::Zero();
	x_e = Vector3d::Zero(); eiR = Vector3d::Zero();
  eiX = Vector3d::Zero();

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

  pub_ = n_.advertise<std_msgs::String>("/motor_command",1);
  vis_pub_0 = n_.advertise<visualization_msgs::Marker>("/force0",1);
  vis_pub_1 = n_.advertise<visualization_msgs::Marker>("/force1",1);
  vis_pub_2 = n_.advertise<visualization_msgs::Marker>("/force2",1);
  vis_pub_3 = n_.advertise<visualization_msgs::Marker>("/force3",1);

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

void odroid_node::imu_vicon_callback(const sensor_msgs::Imu::ConstPtr& msgImu, const geometry_msgs::TransformStamped::ConstPtr& msgVicon){

  vector3Transfer(W_raw, msgImu->angular_velocity);

  W_b = W_raw;
  if(!IMU_flag){ ROS_INFO("IMU ready");}
  IMU_flag = true;
  // if(isnan(W_raw(0)) || isnan(W_raw(1)) || isnan(W_raw(2))){IMU_flag = false;}

  if(print_imu){
   printf("IMU: Psi:[%f], Theta:[%f], Phi:[%f] \n", W_raw(0), W_raw(1), W_raw(2));
  }
  vector3Transfer(x_v, msgVicon->transform.translation);
  vector4Transfer(quat_vm, msgVicon->transform.rotation);

  quatToMat(R_v, quat_vm);
  tf::Quaternion q(quat_vm(0),quat_vm(1),quat_vm(2),quat_vm(3));
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

	if(print_vicon){
    printf("Vicon: xyz:[%f, %f, %f] roll:[%f], pitch:[%f], yaw:[%f] \n", x_v(0),x_v(1),x_v(2), roll/M_PI*180, pitch/M_PI*180, yaw/M_PI*180);
  }
  if(print_x_v){
    cout<<"x_v: "<<x_v.transpose()<<endl;
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
void odroid_node::ctl_callback(){
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

  // xd = VectorXd::Zero(3);
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

  // cout<<"R_vm\n"<<R_vm<<endl;
  // W_b << 0,0.0,0.5;
  // psi = 30/180*M_PI; //msg->orientation.x;
  // theta = 30/180*M_PI;//msg->orientation.y;
  // phi = 30/180*M_PI; //msg->orientation.z;
  // //cout<<"psi: "<<psi<<" theta: "<<theta<<" phi: "<<phi<<endl;
  // Eigen::Vector3d angle(roll,pitch,yaw);
  //euler_Rvm(R_vm, angle);

  R_eb = R_ev * R_vm * R_bm;
  if(print_R_eb){cout<<"R_eb: "<<R_eb<<endl;}
  //cout<<"R_eb\n"<<R_eb<<endl;
//cout<<"yaw"<<atan2(-R_eb(2,0),R_eb(0,0))<<endl;
//cout<<"roll"<<atan2(-R_eb(1,2),R_eb(1,1))<<endl;
//cout<<"pitch"<<asin(R_eb(1,0))<<endl;

  // R_eb = R_vm.transpose();
  del_t_CADS = 0.01;

  if(print_xd){
    cout<<"xd: "<<xd.transpose()<<endl;
  }

  QuadGeometricPositionController(xd, xd_dot, xd_ddot, Wd, Wd_dot, x_v, v_v, W_b, R_v);

  // OutputMotor(f_motor,thr);
  // cout<<f_motor.transpose()<<endl;
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
  visualization_msgs::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = vicon_time;
  marker.ns = "odroid";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.3;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = -0.707;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 0.707;
  marker.scale.x = f_motor(0) - mean;
  marker.scale.y = 0.05;
  marker.scale.z = 0.05;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  //only if using a MESH_RESOURCE marker type:
  // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  vis_pub_0.publish( marker);

  marker.pose.position.x = 0;
  marker.pose.position.y = -0.3;
  marker.scale.x = f_motor(1) - mean;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  vis_pub_1.publish( marker);

  marker.pose.position.x = -0.3;
  marker.pose.position.y = 0;
  marker.scale.x = f_motor(2) - mean;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  vis_pub_2.publish( marker);

  marker.pose.position.x = 0;
  marker.pose.position.y = 0.3;
  marker.scale.x = f_motor(3) - mean;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  vis_pub_3.publish( marker);


  if(!simulation){
    motor_command();
  }

}

void odroid_node::QuadGeometricPositionController(Vector3d xd, Vector3d xd_dot, Vector3d xd_ddot,Vector3d Wd, Vector3d Wddot, Vector3d x_v, Vector3d v_v, Vector3d W_in, Matrix3d R_v){
  std::cout.precision(5);
    // Bring to controller frame (and back) with 180 degree rotation about b1
    Matrix3d D = R_bm;

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

    if(mode == 0){
      ex = Vector3d::Zero();
      ev = Vector3d::Zero();
    }

    eiX = eiX_last+del_t*(ex+cX*ev);
    err_sat(-eiX_sat, eiX_sat, eiX);
    eiX_last = eiX;

    if(print_eX){cout<<"eX: "<<ex.transpose()<<endl;}
    if(print_eV){cout<<"eV: "<<ev.transpose()<<endl;}
    // std::cout<<"ex:\n"<<ex.transpose()<<std::endl;
    // std::cout<<"ev:\n"<<ev.transpose()<<std::endl;
    // std::cout<<"eiX:\n"<<eiX.transpose()<<std::endl;
    // Force 'f' along negative b3-axis
    Vector3d A = -kx*ex-kv*ev-kiX*eiX-m*g*e3+m*xd_2dot;
    Vector3d L = R*e3;
    Vector3d Ldot = R*hat_eigen(W)*e3;
    double f = -A.dot(R*e3);
    // std::cout<<f<<std::endl;
    f_quad = f;

    if(print_f){print_force();}
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
    vee_eigen(.5*(Rd.transpose()*R-R.transpose()*Rd), eR);
    if(print_eR){cout<<"eR: "<<eR.transpose()<<endl;}
    // cout<<"eR:\n"<<eR<<endl;
    // Angular Velocity Error 'eW'
    eW = W-R.transpose()*Rd*Wd;
    if(print_eW){cout<<"eW: "<<eW.transpose()<<endl;}
    // cout<<"eW:\n"<<eW<<endl;
    // Attitude Integral Term
    eiR = del_t*(eR+cR*eW) + eiR_last;
    err_sat(-eiR_sat, eiR_sat, eiR);
    eiR_last = eiR;
    // 3D Moment
    M = -kR*eR-kW*eW-kiR*eiR+hat_eigen(R.transpose()*Rd*Wd)*J*R.transpose()*Rd*Wd+J*R.transpose()*Rd*Wddot;// LBFF
    M = D*M;// LBFF->GBFF
    if(print_M){cout<<"M: "<<M.transpose()<<endl;}

    Matrix<double, 4, 1> FM;
    FM[0] = f;
    FM[1] = M[0];
    FM[2] = M[1];
    FM[3] = M[2];


    f_motor = Ainv*FM;
}

void odroid_node::gazebo_controll(){
  ros::ServiceClient client_FM = n_.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  gazebo_msgs::ApplyBodyWrench FMcmds_srv;

  Vector3d fvec_GB(0.0, 0.0, f_quad), fvec_GI;

  fvec_GI = R_v*fvec_GB;
  Vector3d M_out = R_v*M;

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

void odroid_node::callback(odroid::GainsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: Update");
  print_f = config.print_f;
  print_imu = config.print_imu;
  print_thr = config.print_thr;

  if(MOTOR_ON && !MotorWarmup){
    ros::param::get("/controller/gain/att/ki",kiR);
    ros::param::get("/controller/gain/pos/ki",kiX);
  }else{
    kiR = 0;
    kiX = 0;
  }

  MOTOR_ON = config.Motor;
  MotorWarmup = config.MotorWarmup;
  xd(0) =  config.x;
  xd(1) =  config.y;
  xd(2) =  config.z;

  print_xd = config.print_xd;
  print_x_v = config.print_x_v;
  print_eX = config.print_eX;
  print_eV = config.print_eV;
  print_eR = config.print_eR;
  print_eW = config.print_eW;
  print_vicon = config.print_vicon;
  print_M = config.print_M;
  print_F = config.print_F;
  print_f_motor = config.print_f_motor;
  print_R_eb = config.print_R_eb;
}


void odroid_node::motor_command(){
  // Execute motor output commands
  for(int i = 0; i < 4; i++){
    //printf("Motor %i I2C write command of %i to address %i (%e N).\n", i, thr[i], mtr_addr[i], f[i]);
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

  std_msgs::String out;
  pub_.publish(out);
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

  for(motornum = 1; motornum <= 4; motornum++){
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
