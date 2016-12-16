#include <odroid/odroid_node.hpp>
// User header files
// #include <ros/ros.h>

using namespace std;

odroid_node::odroid_node(){
    m = 1.25; g = 9.81;
    // J = {55710.50413e-7 ,  617.6577e-7   , -250.2846e-7 , 617.6577e-7    ,  55757.4605e-7 , 100.6760e-7 , -250.2846e-7  ,  100.6760e-7   , 105053.7595e-7};// kg*m^2

}
odroid_node::~odroid_node(){}

void odroid_node::imu_Callback(const sensor_msgs::Imu::ConstPtr& msg){
    W_raw[0] = msg->angular_velocity.x;
    W_raw[1] = msg->angular_velocity.y;
    W_raw[2] = msg->angular_velocity.z;
    psi = msg->orientation.x;
    theta = msg->orientation.y;
    phi = msg->orientation.z;
    // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
}


// callback for key Inputs
void odroid_node::key_callback(){}
// callback for IMU sensor deta
// void IMU_callback(const sensor_msgs::Imu::ConstPtr& msg){
    // ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
// }
// Action for controller
void odroid_node::ctl_callback(){

}
// vicon information callback
void odroid_node::vicon_callback(){}

int main(int argc, char **argv){
    // ros::init(argc, argv, "imu_listener");
    ros::init(argc,argv,"hexacopter");
    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("imu", 100, IMU_callback);
    odroid_node odnode;
    ros::Subscriber sub2 = nh.subscribe("imu",100,&odroid_node::imu_Callback,&odnode);
    ros::spin();
    return 0;
}


//   file = fopen("system_data.txt","w");
//
//   while(SYSTEM_RUN == true)
//   {
//     while(new_IMU_data_avail == true)
//     {
//       // Time stamp
//       gettimeofday(&t_before, NULL);
//       gettimeofday(&t_now, NULL);
//       t_IMU = (t_now.tv_sec - t_init.tv_sec) ;
//       t_IMU += (t_now.tv_usec - t_init.tv_usec)/1000000.;
//
//       if(IMU_Thread_First_Loop == true) {
//         del_t_IMU = 0;
//         IMU_Thread_First_Loop = false;
//       }
//       else {
//         del_t_IMU = t_IMU-t_prev_IMU;
//         Speed_IMU_Thread = 1/del_t_IMU;
//       }
//       t_prev_IMU = t_IMU;
//
//       // printf("IMU thread difference: %+#lf\n", difference);
//
//       //  printf("IMU Difference %+#lf\n", difference);
//       //  printf("del_t_IMU: %+#lf\n", del_t_IMU);
//
//       // Euler angles without bias (rad)
//
//       EAngles_imu[0] = (double)phi*PI/180;//roll
//       EAngles_imu[1] = (double)theta*PI/180; //pitch
//       EAngles_imu[2] = (double)psi*PI/180; //yaw
//
//       E_angles_save[0] = EAngles_imu[0];
//       E_angles_save[1] = EAngles_imu[1];
//       E_angles_save[2] = EAngles_imu[2];
//
//       // Angular velocity in IMU frame with its noise (rad/s)
//       W_b_noisy[0] = W_raw[0] - W1_bias;
//       W_b_noisy[1] = W_raw[1] - W2_bias;
//       W_b_noisy[2] = W_raw[2] - W3_bias;
//
//       psi = psi/180*PI;
//       theta = theta/180*PI;
//       phi = phi/180*PI;
//
//       // Compare with a low-pass RC filter
//       LowPassRCFilter(W_b_noisy, W_b_, del_t_IMU, RC_IMU, W_b);
//
//       //We measure the rotation matrix from the markers to the Vicon cameras (R_vm)
//       // from the mearsured quaternions (quat_vm)
//       //EAngles321_to_R(EAngles_imu, R_vm);
//       // phi = EAngles_imu[0]
//       // theta = EAngle[1]
//       // psi = EAngle[2]
//
//       R_vm[0][0] = cos(theta)*cos(psi);
//       R_vm[0][1] = cos(theta)*sin(psi);
//       R_vm[0][2] = -sin(theta);
//       R_vm[1][0] = sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
//       R_vm[1][1] = sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
//       R_vm[1][2] = sin(phi)*cos(theta);
//       R_vm[2][0] = cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
//       R_vm[2][1] = cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
//       R_vm[2][2] = cos(phi)*cos(theta);
//
//       // R_eb = R_ev*R_vm*R_bm' (note that R_bm' = R_bm)
//
//       // Matrix_multipication(R_ev, R_vm, R_em);
//       // Matrix_multipication(R_em, R_bm, R_eb);// R_eb: body to inertial
//       transpose(R_vm, R_eb);
//
//       cRDot[0][0] = (R_eb[0][0]-cRPrev[0][0])/del_t_IMU;
//       cRDot[0][1] = (R_eb[0][1]-cRPrev[0][1])/del_t_IMU;
//       cRDot[0][2] = (R_eb[0][2]-cRPrev[0][2])/del_t_IMU;
//       cRDot[1][0] = (R_eb[1][0]-cRPrev[1][0])/del_t_IMU;
//       cRDot[1][1] = (R_eb[1][1]-cRPrev[1][1])/del_t_IMU;
//       cRDot[1][2] = (R_eb[1][2]-cRPrev[1][2])/del_t_IMU;
//       cRDot[2][0] = (R_eb[2][0]-cRPrev[2][0])/del_t_IMU;
//       cRDot[2][1] = (R_eb[2][1]-cRPrev[2][1])/del_t_IMU;
//       cRDot[2][2] = (R_eb[2][2]-cRPrev[2][2])/del_t_IMU;
//
//       transpose(R_eb, ctrpR_eb);
//       Matrix_multipication(ctrpR_eb, cRDot, cOmegaHat);
//       vee(cOmegaHat, cOmega);
//
//       cRPrev[0][0] = R_eb[0][0];
//       cRPrev[0][1] = R_eb[0][1];
//       cRPrev[0][2] = R_eb[0][2];
//       cRPrev[1][0] = R_eb[1][0];
//       cRPrev[1][1] = R_eb[1][1];
//       cRPrev[1][2] = R_eb[1][2];
//       cRPrev[2][0] = R_eb[2][0];
//       cRPrev[2][1] = R_eb[2][1];
//       cRPrev[2][2] = R_eb[2][2];
//
//       reshape_3by3_to_9by1(R_eb, R_print);
//       reshape_3by3_to_9by1(cRDot, Rdot_print);
//
//       //            W_b[0] = W_b_noisy[0];
//       //            W_b[1] = W_b_noisy[1];
//       //            W_b[2] = W_b_noisy[2];
//
//       if(   abs(W_b[0]) < 100
//       && W_b[1] < 100
//       && W_b[2] < 100){// If signal is right, use it and save it.
//         W_b_[0] = W_b[0];
//         W_b_[1] = W_b[1];
//         W_b_[2] = W_b[2];
//         EAngles_imu_[0] = EAngles_imu[0];
//         EAngles_imu_[1] = EAngles_imu[1];
//         EAngles_imu_[2] = EAngles_imu[2];
//       }
//       else {// If signal is wrong, use the last correct measurement.
//         W_b[0] = W_b_[0];
//         W_b[1] = W_b_[1];
//         W_b[2] = W_b_[2];
//         EAngles_imu[0] = EAngles_imu_[0];
//         EAngles_imu[1] = EAngles_imu_[1];
//         EAngles_imu[2] = EAngles_imu_[2];
//         printf("Error over IMU serial port. Using IMU data from last step.\n");
//       }
//
//       old = t_now.tv_usec;
//
//       IMU_Thread_First_Loop == false;
//
//       IMU_delay_for_CAD = difference;
//
//       fprintf(file,"%f,%f,%f,%f,",del_t_IMU,EAngles_imu[0]/PI*180, EAngles_imu[1]/PI*180, EAngles_imu[2]/PI*180);
//       fprintf(file,"%e,%e,%e,",eR_print[0], eR_print[1], eR_print[2]);
//       fprintf(file,"%e,%e,%e,",eW_print[0], eW_print[1], eW_print[2]);
//       fprintf(file,"%e,%e,%e,",eI_print[0], eI_print[1], eI_print[2]);
//       fprintf(file,"%e,%e,%e,%e,",f_print, M_print[0], M_print[1], M_print[2]);
//       fprintf(file,"%e,%e,%e,%e,%e,%e,",fi_print[0],fi_print[1],fi_print[2],fi_print[3],fi_print[4],fi_print[5]);
//       fprintf(file,"%i,%i,%i,%i,%i,%i,", thr_print[0],thr_print[1],thr_print[2],thr_print[3],thr_print[4],thr_print[5]);
//       fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,",R_print[0],R_print[1],R_print[2],R_print[3],R_print[4],R_print[5],R_print[6],R_print[7],R_print[8]);
//       fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,",Rdot_print[0],Rdot_print[1],Rdot_print[2],Rdot_print[3],Rdot_print[4],Rdot_print[5],Rdot_print[6],Rdot_print[7],Rdot_print[8]);
//       fprintf(file,"%e,%e,%e \r\n",W_b[0], W_b[1], W_b[2]);
//
//       new_IMU_data_avail = false;
//     }
//   }
//   fclose(file);
//   usleep(250000);
//   pthread_exit(NULL);
// }


// Thread [3]: Exectute control & actuation, then save the data
// void *Control_Actuation_DataSaving(void *thread_id)
// {
//   double IMU_delay;
//   // Load Gains
//   GetControllerGain(&kx, &kv, &kiX, &c1, &kR, &kW, &kiR, &c2);
//   //    kR = 0;
//   //    kW = 0;
//
//   FILE *file_CADS;
//   file_CADS = fopen(FileName,"w");
//
//   while(SYSTEM_RUN == true){
//
//
//     if(m > 1.52)
//     m = 1.52;
//
//     usleep(50);
//
//     // Time stamp
//     gettimeofday(&t_before, NULL);
//     gettimeofday(&t_now, NULL);
//     t_CADS = (t_now.tv_sec - t_init.tv_sec) ;
//     t_CADS += (t_now.tv_usec - t_init.tv_usec)/1000000.;
//
//     if(CADS_Thread_First_Loop == true) {
//       del_t_CADS = 0;
//       CADS_Thread_First_Loop = false;
//     }
//     else {
//       del_t_CADS = t_CADS-t_prev_CADS;
//       Speed_CADS_Thread = 1/del_t_CADS;
//     }
//     t_prev_CADS = t_CADS;
//
//     int i, j;
//     if(mission_init == true){
//       for(i = 0; i < 3; i++){
//         x_e_start_mission[i] = x_e[i];
//         for(j = 0; j < 3; j++)
//         R_eb_start_mission[i][j] = R_eb[i][j];
//         t_start_mission = t_CADS;
//       }
//       mission_init = false;
//     }
//     t_mission = t_CADS - t_start_mission;
//
//     // Generate trajectory based on keypad commands
//     //CreateTrackingTrajectory();
//
//     // Only turn on integral control when the system is actually running (motors spinning, etc.)
//     double kiX_now, kiR_now;
//     if(MOTOR_ON == true && MotorWarmup == false){
//       kiX_now = kiX;
//       kiR_now = kiR;
//     }
//     else {
//       kiX_now = 0.0;
//       kiR_now = 0.0;
//     }
//
//     // if(SphericalJointTest == true){
//     //   if(flag_constrain == false)
//     //   }
//     //   else
//     //   {
//     //     GeometricControl_SphericalJoint_3DOF_Constrained(Rd, Wd, Wd_dot, W_b, R_eb, del_t_CADS, eiR, eR, eW, eiR, kR, kW, kiR_now, m, g, J, f);
//     //   }
//     // }
//     // else{
//     //   GeometricController_6DOF(xd, xd_dot, xd_ddot, Rd, Wd, Wd_dot, x_e, v_e, W_b, R_eb, del_t_CADS, eiX, eiR, eX, eV, eR, eW, eiX, eiR, kx, kv, kiX_now, kR, kW, kiR_now, m, g, J, f);// outputs errors & forces of motors 1-6
//     // }
//
//     GeometricControl_SphericalJoint_3DOF(Rd, Wd, Wd_dot, W_b, R_eb, del_t_CADS, eiR, eR, eW, eiR, kR, kW, kiR_now, m, g, J, f);
//     OutputMotor(f, thr);
//
//     // Execute motor output commands
//     for(i = 0; i < 6; i++)
//     {
//       //            printf("Motor %i I2C write command of %i to address %i (%e N).\n", i, thr[i], mtr_addr[i], f[i]);
//       tcflush(fhi2c, TCIOFLUSH);
//       usleep(500);
//       if(ioctl(fhi2c, I2C_SLAVE, mtr_addr[i])<0)
//       printf("ERROR: ioctl\n");
//       if(MOTOR_ON == false)// set motor speed to zero
//       thr[i] = 0;
//       // else if(MotorWarmup == true)// warm up motors at 20 throttle command
//       // thr[i] = 20;
//       while(write(fhi2c, &thr[i], 1)!=1)
//       printf("ERROR: Motor %i I2C write command of %i to address %i (%e N) not sent.\n", i, thr[i], mtr_addr[i], f[i]);
//     }
//
//     // Testing motors individually
//     // i = 4;
//     //
//     // tcflush(fhi2c, TCIOFLUSH);
//     // usleep(500);
//     // if(ioctl(fhi2c, I2C_SLAVE, mtr_addr[i])<0)
//     // printf("ERROR: ioctl\n");
//     //
//     // thr[i] = 20;
//     // // else if(MotorWarmup == true)// warm up motors at 20 throttle command
//     // // thr[i] = 20;
//     // while(write(fhi2c, &thr[i], 1)!=1)
//     // printf("ERROR: Motor %i I2C write command of %i to address %i (%e N) not sent.\n", i, thr[i], mtr_addr[i], f[i]);
//
//     //   printf("sprintf difference: %+#lf\n", difference);
//
//
//     CADS_Thread_First_Loop == false;
//
//     // Sample data from each measurement thread to see if it has changed since the last loop
//     ViconCurrentSample = x_e[0];
//     IMUCurrentSample = W_b[0];
//
//     // Save data iff data is new
//     if(ViconCurrentSample != ViconPrevSample || IMUCurrentSample != IMUPrevSample)
//     {
//       if(ViconCurrentSample != ViconPrevSample)
//       NewDataVicon = 1;
//       else
//       NewDataVicon = -1;
//
//       if(IMUCurrentSample != IMUPrevSample)
//       NewDataIMU = 1;
//       else
//       NewDataIMU = -1;
//
//       //printf("del_t_IMU: %+#lf\n", del_t_IMU);
//
//       int sprintf_size = sprintf(sprintf_buffer+sprintf_buffer_loc_CADS,"%E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E %E\n",
//       NewDataVicon, NewDataIMU, del_t_v, del_t_IMU,// 1-4
//       x_e[0], x_e[1], x_e[2],// 5-7
//       xd[0], xd[1], xd[2],// 8-10
//       v_e[0], v_e[1], v_e[2],// 11-13
//       xd_dot[0],  xd_dot[1],  xd_dot[2],// 14-16
//       R_eb[0][0], R_eb[0][1], R_eb[0][2],// 17-19
//       R_eb[1][0], R_eb[1][1], R_eb[1][2],// 20-22
//       R_eb[2][0], R_eb[2][1], R_eb[2][2],// 23-25
//       Rd[0][0], Rd[0][1], Rd[0][2],// 26-28
//       Rd[1][0], Rd[1][1], Rd[1][2],// 29-31
//       Rd[2][0], Rd[2][1], Rd[2][2],// 32-34
//       W_b[0], W_b[1], W_b[2],// 35-37
//       Wd[0], Wd[1], Wd[2]// 38-40
//     );
//
//     sprintf_buffer_loc_CADS += sprintf_size;
//
//     // Save sample data from each measurement thread to evaluate during the next loop
//     ViconPrevSample = x_e[0];
//     IMUPrevSample = W_b[0];
//   }
//
//   if(flag_printf == true)
//   {
//     printf("\n\n m = %e kg.\n\n", m);
//
//     printf("Thread Speeds:\n");
//     printf("The Vicon thread is %e Hz.\n", Speed_Vicon_Thread);
//     printf("The IMU thread is %e Hz.\n", Speed_IMU_Thread);
//     printf("The Control/Actuation/DataSaving thread is %e Hz.\n\n", Speed_CADS_Thread);
//
//     printf("Gains:\n");
//     printf("Position kx = %e, kv = %e, kiX = %e\n", kx, kv, kiX_now);
//     printf("Attitude: kR = %e, kW = %e, kiR = %e\n\n", kR, kW, kiR_now);
//
//     printf("Vicon Data: %lf, %lf, %lf, %lf, %lf, %lf, %lf\n", x_v[0], x_v[1], x_v[2], quat_vm[0], quat_vm[1], quat_vm[2], quat_vm[3]);
//     printf("Angular Velocity:  %+#7.3f  %+#7.3f  %+#7.3f\n", W_raw[0], W_raw[1], W_raw[2]);
//
//     // printf("Motors:\n");
//     // for(i = 0; i < 6; i++)
//     //     printf("Motor %i I2C write command of %i to address %i (%e N).\n", i, thr[i], mtr_addr[i], f[i]);
//     printf("\n\n\n\n\n\n\n\n\n\n\n\n\n");
//   }
// }
//
// // Save data if it exists
// if(CADS_Thread_First_Loop == false){
//   usleep(100000);
//   printf("\n\n\n\n\nSaving testing data buffer to file_CADS... ");
//   fwrite(sprintf_buffer, 1, sprintf_buffer_loc_CADS, file_CADS);
//   printf("done.\n");
//   fclose(file_CADS);
// }
//
// close(newsockfd);
// close(sockfd);
//
// usleep(250000);
// pthread_exit(NULL);
// }

// int main(int argc, char** argv)
// {
  // int i;
  // pthread_t threads[4];
  // pthread_attr_t attr;
  // struct sched_param  param;
  // int fifo_max_prio, fifo_min_prio;
  //
  // system("clear");
  // usleep(100000);
  //
  // char junk;
  //
  //
  // // Open i2c:
  // fhi2c = open("/dev/i2c-1", O_RDWR);// Chris
  // printf("Opening i2c port...\n");
  // // if(fhi2c!=3)
  // //     printf("ERROR opening i2c port.\n");
  // // else
  // //     printf("The i2c port is open.\n");
  // usleep(100000);
  // tcflush(fhi2c, TCIFLUSH);
  // usleep(100000);
  //
  // // Call and response from motors
  // printf("Checking motors...\n");
  // int motornum, motoraddr, motorworks;
  // int thr0 = 0;// 0 speed test command
  // char PressEnter;
  //
  // for(motornum = 1; motornum <= 6; motornum++){
  //   motoraddr = motornum+40;// 1, 2, 3, ... -> 41, 42, 43, ...
  //   while(1){
  //     motorworks = 1;// will remain 1 if all works properly
  //     if(ioctl(fhi2c, I2C_SLAVE, motoraddr)<0){
  //       printf("ERROR: motor %i ioctl\n", motornum);
  //       motorworks = 0;// i2c address not called
  //     }
  //     usleep(10);
  //     if(write(fhi2c, &thr0, 1)!=1){
  //       printf("ERROR: motor %i write\n", motornum);
  //       motorworks = 0;
  //     }
  //     usleep(10);
  //     tcflush(fhi2c, TCIOFLUSH);
  //     if(motorworks == 1){
  //       printf("Motor %i working...\n", motornum);
  //       break;
  //     }
  //     else{
  //       printf("Fix motor %i, then press ENTER.\n\n", motornum);
  //       printf("Note: another i2c device may interupt the signal if\n");
  //       printf("any i2c wires are attached to unpowered components.\n");
  //       scanf("%c",&PressEnter);
  //       break;
  //     }
  //   }
  // }
  // printf("All motors are working.\n");
  // usleep(5000);
  //
  //
  //   int k, num_trials;
  //   double phi_bias_trial, theta_bias_trial, psi_bias_trial,
  //   W1_bias_trial, W2_bias_trial, W3_bias_trial;
  //
  //   num_trials = 10;
  //   while(1){
  //
  //     phi_bias = 0.0;
  //     theta_bias = 0.0;
  //     W2_bias = 0.0;
  //     W3_bias = 0.0;
  //   W2_bias = 0.0;
  //     W3_bias = 0.0;
  //
  //     for(k = 1; k <= num_trials; k++){
  //
  //       // Collect data
  //       usleep(500);
  //
  //       // For debugging the IMU
  //       // printf("Angular Velocity:  %+#7.3f  %+#7.3f  %+#7.3f\n", W_raw[0], W_raw[1], W_raw[2]);
  //       // printf("Euler Angles:  %+#7.3f  %+#7.3f  %+#7.3f\n", phi, theta, psi);
  //
  //       phi_bias_trial = phi;
  //       theta_bias_trial = theta;
  //       psi_bias_trial = psi;
  //
  //       W1_bias_trial = W_raw[0];
  //       W2_bias_trial = W_raw[1];
  //       W3_bias_trial = W_raw[2];
  //
  //       // Add up trials for an average
  //       phi_bias += (double)phi_bias_trial*PI/180;
  //       theta_bias += (double)theta_bias_trial*PI/180;
  //       psi_bias += (double)psi_bias_trial*PI/180;
  //       W1_bias += (double)W1_bias_trial;
  //       W2_bias += (double)W2_bias_trial;
  //       W3_bias += (double)W3_bias_trial;
  //       usleep(10);// clear buffer
  //     }
  //
  //     // // Take the average
  //     phi_bias /= num_trials;
  //     theta_bias /= num_trials;
  //     psi_bias /= num_trials;
  //     W1_bias /= num_trials;
  //     W2_bias /= num_trials;
  //     W3_bias /= num_trials;
  //
  //     if(
  //       abs(phi_bias) < 10
  //       && abs(theta_bias) < 10
  //       && abs(psi_bias) < 10
  //       && abs(W1_bias) < 10
  //       && abs(W2_bias) < 10
  //       && abs(W3_bias) < 10){
  //         break;
  //       }
  //       else {
  //         printf("Error communicating over serial port. Trying again...\n");
  //       }
  //     }
  //
  //     printf("phi_bias = %F\n", phi_bias);
  //     printf("theta_bias = %F\n", theta_bias);
  //     printf("psi_bias = %F\n", psi_bias);
  //     printf("W1_bias = %F\n", W1_bias);
  //     printf("W2_bias = %F\n", W2_bias);
  //     printf("W3_bias = %F\n", W3_bias);
  //     // These values are subtracted from the output of the IMU
  //
  //
  //     printf("____________________________________\n");
  //     printf("____________________________________\n");
  //     printf("         Command Choices\n");
  //     printf("Basic:\n");
  //     printf("c: Stop program    #: Spec. Traj.\n");
  //     printf("q: On/Off motors   e: Motor warm/run\n");
  //
  //     printf("Gains:\n");
  //     printf("t: Increase kx     b: Increase kR\n");
  //     printf("g: Decrease kx     v: Decrease kR\n");
  //     printf("y: Increase kv     m: Increase kW\n");
  //     printf("h: Decrease kv     n: Decrease kW\n");
  //
  //     printf("UAV Location:\n");
  //     printf("j: UAV x+0.1m      l: UAV x-0.1m\n");
  //     printf("i: UAV y+0.1m      k: UAV y-0.1m\n");
  //     printf("s: UAV z+0.1m      w: UAV z-0.1m\n");
  //     printf("r: Keep UAV where it is.\n");
  //     printf("__________________________________________________________\n");
  //     printf("__________________________________________________________\n");
  //     printf("    ____   _______   __   __   ____    __   _     ____   \n");
  //     printf("   / __ |    | |    | |__| |  | ___|  |  | | |   | __ |  \n");
  //     printf("   |    |    | |    |  __| |  | __|   | |\\ | |   |    |  \n");
  //     printf("   |_/|_|    |_|    |_|  |_|  |____|  |_| \\ _|   |__|_|  \n");
  //     printf("__________________________________________________________\n");
  //     printf("__________________________________________________________\n");
  //     printf("\n   Press ENTER to begin threads.\n");
  //     scanf("%c", &PressEnter);
  //
  //     // Start the clock!
  //     gettimeofday(&t_init, NULL);
  //
  //     // Initialize mutex and condition variables
  //     pthread_mutex_init(&data_acq_mutex, NULL);
  //
  //     // Set thread attributes
  //     pthread_attr_init(&attr);
  //     pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  //     pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  //     fifo_max_prio = sched_get_priority_max(SCHED_FIFO);
  //     fifo_min_prio = sched_get_priority_min(SCHED_FIFO);
  //
  //     // Create threads and associate priorities (equal):
  //
  //     // Command_Key_Thread
  //     param.sched_priority = fifo_max_prio/NUM_THREADS;
  //     pthread_attr_setschedparam(&attr, &param);
  //     pthread_create(&threads[0], &attr, Command_Key_Thread, (void *) 2);
  //
  //     // Vicon_Thread (position, attitude, and velocity)
  //     // param.sched_priority = fifo_max_prio/NUM_THREADS;
  //     // pthread_attr_setschedparam(&attr, &param);
  //     // pthread_create(&threads[1], &attr, Vicon_Thread, (void *) 1);
  //
  //     // IMU_Thread (angular velocity)
  //     param.sched_priority = fifo_max_prio/NUM_THREADS;
  //     pthread_attr_setschedparam(&attr, &param);
  //     pthread_create(&threads[2], &attr, IMU_Thread, (void *) 0);
  //
  //     // Control_Actuation_DataSaving (control, I2C, data collection)
  //     param.sched_priority = fifo_max_prio/NUM_THREADS;
  //     pthread_attr_setschedparam(&attr, &param);
  //     pthread_create(&threads[3], &attr, Control_Actuation_DataSaving, (void *) 0);
  //
  //     printf("Beginning threads...\n");
  //     // Wait for all threads to complete
  //     for (i = 0; i < NUM_THREADS; i++)
  //     {
  //       pthread_join(threads[i], NULL);
  //     }
  //
  //     close(fhi2c);
  //
  //     pthread_attr_destroy(&attr);
  //     pthread_mutex_destroy(&data_acq_mutex);
  //
  //     errorCode = vn100_unregisterAsyncDataReceivedListener(&vn100, &asyncDataListener);
  //
  //     errorCode = vn100_disconnect(&vn100);
  //
  //     if (errorCode != VNERR_NO_ERROR)
  //     {
  //       printf("Error encountered when trying to disconnect from the sensor.\n");
  //
  //       return 0;
  //     }
  //     close(newsockfd);
  //     close(sockfd);
  //
