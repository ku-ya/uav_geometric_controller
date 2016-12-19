#include <odroid/odroid_node.hpp>
// User header files
// #include <ros/ros.h>

using namespace std;

odroid_node::odroid_node(){
    m = 1.25; g = 9.81;
    //  0 ,0,0
    Je <<  55710.50413e-7 ,  617.6577e-7   , -250.2846e-7 , 617.6577e-7    ,  55757.4605e-7 , 100.6760e-7 , -250.2846e-7  ,  100.6760e-7   , 105053.7595e-7;// kg*m^2


}
odroid_node::~odroid_node(){};

void odroid_node::print_J(){
    std::cout<<"J: "<<Je<<std::endl;
}
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
void odroid_node::attitude_controller(double Rd[3][3], double Wd[3], double Wddot[3],
      double W[3], double R[3][3], double del_t, double eiR_last[3],
      double eR[3], double eW[3], double eiR[3],
      double kR, double kW, double kiR_now, double f[6]){
        double trpR[3][3], trpRd[3][3], trpRd_R[3][3], trpR_Rd[3][3], inside_vee_3by3[3][3], vee_3by1[3], trpR_Rd_Wd[3],
        What[3][3], J_W[3], What_J_W[3], trpR_Rd_Wddot[3], What_trpR_Rd[3][3], What_trpR_Rd_Wd[3],
        Jmult[3], J_Jmult[3], M[3], FM[6], r[3], F_g[3], trpRe3[3], M_g[3], F_req;

        // Given the UAV arm length of 0.31 m and a prop. angle of 15 deg.
        double invFMmat[6][6] = {  {  0.0000,    1.2879,   -0.1725,   -0.0000,    1.1132,    0.3071},
        { -1.1154,    0.6440,    0.1725,    0.9641,   -0.3420,    0.7579},
        { -1.1154,   -0.6440,   -0.1725,   -0.9641,   -0.7712,    1.7092},
        { -0.0000,   -1.2879,    0.1725,         0,    1.1132,    0.3071},
        {  1.1154,   -0.6440,   -0.1725,    0.9641,   -0.3420,    0.7579},
        {  1.1154,    0.6440,    0.1725,   -0.9641,   -0.7712,    1.7092}};

        double e3[3] = {0.0, 0.0, 1.0};
        double b3[3] = {0.0, 0.0, 1.0};

        double l = 0;//.05;// length of rod connecting to the spherical joint
        transpose(R, trpR);// 3x3
        r[0] = -l*b3[0];
        r[1] = -l*b3[1];
        r[2] = -l*b3[2];
        matrix_vector(trpR, e3, trpRe3);
        F_g[0] = m*g*trpRe3[0];
        F_g[1] = m*g*trpRe3[1];
        F_g[2] = m*g*trpRe3[2];
        cross_product(r, F_g, M_g);// Moment due to gravity about arm
        // Calculate eR (rotation matrix error)
        transpose(Rd, trpRd);// 3x3
        Matrix_multipication(trpRd, R, trpRd_R);// 3x3
        Matrix_multipication(trpR, Rd, trpR_Rd);// 3x3
        // Take 9 elements of difference
        inside_vee_3by3[0][0] = trpRd_R[0][0]-trpR_Rd[0][0];
        inside_vee_3by3[0][1] = trpRd_R[0][1]-trpR_Rd[0][1];
        inside_vee_3by3[0][2] = trpRd_R[0][2]-trpR_Rd[0][2];
        inside_vee_3by3[1][0] = trpRd_R[1][0]-trpR_Rd[1][0];
        inside_vee_3by3[1][1] = trpRd_R[1][1]-trpR_Rd[1][1];
        inside_vee_3by3[1][2] = trpRd_R[1][2]-trpR_Rd[1][2];
        inside_vee_3by3[2][0] = trpRd_R[2][0]-trpR_Rd[2][0];
        inside_vee_3by3[2][1] = trpRd_R[2][1]-trpR_Rd[2][1];
        inside_vee_3by3[2][2] = trpRd_R[2][2]-trpR_Rd[2][2];
        invskew(inside_vee_3by3, vee_3by1);// 3x1
        eR[0] = 0.5*vee_3by1[0];
        eR[1] = 0.5*vee_3by1[1];
        eR[2] = 0.5*vee_3by1[2];

        // Calculate eW (angular velocity error in body-fixed frame)
        matrix_vector(trpR_Rd, Wd, trpR_Rd_Wd);// 3x1
        eW[0] = W[0]-trpR_Rd_Wd[0];
        eW[1] = W[1]-trpR_Rd_Wd[1];
        eW[2] = W[2]-trpR_Rd_Wd[2];

        // Update integral term of control
        // Attitude:
        eiR[0] = eiR_last[0]+(del_t)*eR[0];//(c2*eR[0]+(eW[0]));
        eiR[1] = eiR_last[1]+(del_t)*eR[1];//(c2*eR[1]+(eW[1]));
        eiR[2] = eiR_last[2]+(del_t)*eR[2];//(c2*eR[2]+(eW[2]));

        // Calculate 3 DOFs of M (controlled moment in body-fixed frame)
        // MATLAB: M = -kR*eR-kW*eW-kRi*eiR+cross(W,J*W)+J*(R'*Rd*Wddot-hat(W)*R'*Rd*Wd);
        skew(W, What);
        //printf("What is %f.\n", What);
        matrix_vector(J, W, J_W);
        //printf("J_W is %f.\n", J_W);
        matrix_vector(What, J_W, What_J_W);
        //printf("What_J_W is %f.\n", What_J_W);
        matrix_vector(trpR_Rd, Wddot, trpR_Rd_Wddot);
        Matrix_multipication (What, trpR_Rd, What_trpR_Rd);
        matrix_vector(What_trpR_Rd, Wd, What_trpR_Rd_Wd);
        Jmult[0] = trpR_Rd_Wddot[0]-What_trpR_Rd_Wd[0];
        Jmult[1] = trpR_Rd_Wddot[1]-What_trpR_Rd_Wd[1];
        Jmult[2] = trpR_Rd_Wddot[2]-What_trpR_Rd_Wd[2];
        matrix_vector(J, Jmult, J_Jmult);

        M[0] = -kR*eR[0]-kW*eW[0]-kiR_now*eiR[0]+What_J_W[0]+J_Jmult[0]-M_g[0];
        M[1] = -kR*eR[1]-kW*eW[1]-kiR_now*eiR[1]+What_J_W[1]+J_Jmult[1]-M_g[1];
        M[2] = -kR*eR[2]-kW*eW[2]-kiR_now*eiR[2]+What_J_W[2]+J_Jmult[2]-M_g[2];
        // To try different motor speeds, choose a force in the radial direction
        F_req = -m*g;// N
        // Convert forces & moments to f_i for i = 1:6 (forces of i-th prop)
        FM[0] = 0.0;
        FM[1] = 0.0;
        FM[2] = F_req;
        FM[3] = M[0];
        FM[4] = M[1];
        FM[5] = M[2];
        matrix_vector6(invFMmat, FM, f);
}

int main(int argc, char **argv){
    // ros::init(argc, argv, "imu_listener");
    ros::init(argc,argv,"hexacopter");
    ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("imu", 100, IMU_callback);
    odroid_node odnode;
    ros::Subscriber sub2 = nh.subscribe("imu",100,&odroid_node::imu_Callback,&odnode);

    // Matrix3f m(2,2);
    double Rd[3][3] = {{1.0 , 0.0 , 0.0},
                       {0.0 , 1.0 , 0.0},
                       {0.0 , 0.0 , 1.0}},
           Wd[3]     = {0.0 , 0.0 , 0.0},
           Wd_dot[3] = {0.0 , 0.0 , 0.0};
    double  W_b[3] = {0,0,0};
    double kR = 0, kW = 1, kiR_now = 0;
   odnode.attitude_controller(Rd, Wd, Wd_dot, W_b, R_eb, del_t_CADS, eiR, eR, eW, eiR, kR, kW, kiR_now, f);
   odnode.print_J();
    ros::spin();
    return 0;
}
