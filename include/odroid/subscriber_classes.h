#ifndef SUBSCRIBER_CLASSES
#define SUBSCRIBER_CLASSES


#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

using namespace std;

// STD_MSGS
#include <std_msgs/String.h>

class sub_class_std_msgs_String
{
public:
    std_msgs::String msg;
    void callback(const std_msgs::String::ConstPtr& msg_sub){msg = *msg_sub;}
};

#include <std_msgs/Float64MultiArray.h>
class sub_class_std_msgs_Float64MultiArray
{
public:
    std_msgs::Float64MultiArray msg;
    void callback(const std_msgs::Float64MultiArray::ConstPtr& msg_sub){msg = *msg_sub;}
};



// GEOMETRY_MSGS

#include <geometry_msgs/Twist.h>
class sub_class_geometry_msgs_Twist
{
public:
    geometry_msgs::Twist msg;
    void callback(const geometry_msgs::Twist::ConstPtr& msg_sub){msg = *msg_sub;}
};

#include <geometry_msgs/TransformStamped.h>
class sub_class_geometry_msgs_TransformStamped
{
public:
    geometry_msgs::TransformStamped msg;
    void callback(const geometry_msgs::TransformStamped::ConstPtr& msg_sub){msg = *msg_sub;}
};

#include <geometry_msgs/PoseStamped.h>
class sub_class_geometry_msgs_PoseStamped
{
public:
    geometry_msgs::PoseStamped msg;
    void callback(const geometry_msgs::PoseStamped::ConstPtr& msg_sub){msg = *msg_sub;}
};


// SENSOR_MSGS
#include <sensor_msgs/LaserScan.h>
class sub_class_sensor_msgs_LaserScan
{
public:
    sensor_msgs::LaserScan msg;
    void callback(const sensor_msgs::LaserScan::ConstPtr& msg_sub){msg = *msg_sub;}
};


//// GEOMETRY_MSGS
//#include <gazebo/gazebo.hh>
//#include <gazebo_msgs/ModelState.h>
//#include <gazebo_msgs/GetModelState.h>
//#include <gazebo_msgs/GetModelStates.h>
//class sub_class_gazebo_msgs_ModelStates
//{
//public:
//    string model_name;
//    gazebo_msgs::ModelState msg;
//    void callback(const gazebo_msgs::GetModelStates& msg_sub){
//        for(int i = 0; i < *msg_sub.name.size(); i++){
//            if(*msg_sub.name[i] == model_name){
//                msg.pose = *msg_sub.pose[i];
//                msg.twist = *msg_sub.twist[i];
//                break;
//            }
//            if(i == *msg_sub.name.size()-1)
//                cout << "Error: no model named " << model_name << endl;
//        }
//    }
//};


/*
// Template:
#include <Msg/Type.h>
class sub_class_MsgType
{
public:
    MsgType msg;
    void callback(const MsgType::ConstPtr& msg_sub){msg = *msg_sub;}
};
*/


// OGM_AE-Specific typedefs/structs/classes/generic functions

typedef Eigen::Vector3d vec3by1;
typedef Eigen::Matrix3d mat3by3;

mat3by3 hat_eigen(vec3by1 x){
    mat3by3 xhat;
    xhat(0,0) =   0.0; xhat(0,1) = -x(2); xhat(0,2) =  x(1);
    xhat(1,0) =  x(2); xhat(1,1) =   0.0; xhat(1,2) = -x(0);
    xhat(2,0) = -x(1); xhat(2,1) =  x(0); xhat(2,2) =   0.0;
    return xhat;
}

void vee_eigen(mat3by3 xhat, vec3by1& x){
    x << xhat(2,1), xhat(0,2), xhat(1,0);
    return;
}

class PoseSE3 {
private:
    double qx, qy, qz, qw;
public:
    // 3D Pose:
    Eigen::Vector3d X;// position
    Eigen::Vector3d V;// linear velocity
    Eigen::Matrix3d R;// attitude
    Eigen::Vector3d W;// angular velocity

    // Common Conversions
    void Quat2R(geometry_msgs::Quaternion quat){

        qx = quat.x;
        qy = quat.y;
        qz = quat.z;
        qw = quat.w;

        R(0,0) = 1.0-2*qy*qy-2*qz*qz; R(0,1) = 2*qx*qy-2*qz*qw;     R(0,2) = 2*qx*qz+2*qy*qw;
        R(1,0) = 2*qx*qy+2*qz*qw;     R(1,1) = 1.0-2*qx*qx-2*qz*qz; R(1,2) = 2*qy*qz-2*qx*qw;
        R(2,0) = 2*qx*qz-2*qy*qw;     R(2,1) = 2*qy*qz+2*qx*qw;     R(2,2) = 1.0-2*qx*qx-2*qy*qy;

        return;
    }

    void Transform2XR(geometry_msgs::Transform tf){

        X(0) = tf.translation.x;
        X(1) = tf.translation.y;
        X(2) = tf.translation.z;

        Quat2R(tf.rotation);

    }

    void Twist2VW(geometry_msgs::Twist twist){

        // Gazebo-Inertial Frame
        V(0) = twist.linear.x;
        V(1) = twist.linear.y;
        V(2) = twist.linear.z;

        W(0) = twist.angular.x;
        W(1) = twist.angular.y;
        W(2) = twist.angular.z;

        // Gazebo-Body-Fixed Frame
        W = R.transpose()*W;

    }
};

// Generic Conversions

// Conversions

void quat2R(geometry_msgs::Quaternion& q, Eigen::Matrix3d& R){
    R << 1.0-2*q.y*q.y-2*q.z*q.z, 2*q.x*q.y-2*q.z*q.w,     2*q.x*q.z+2*q.y*q.w,
         2*q.x*q.y+2*q.z*q.w,     1.0-2*q.x*q.x-2*q.z*q.z, 2*q.y*q.z-2*q.x*q.w,
         2*q.x*q.z-2*q.y*q.w,     2*q.y*q.z+2*q.x*q.w,     1.0-2*q.x*q.x-2*q.y*q.y;
}

void R2quat(Eigen::Matrix3d& R, geometry_msgs::Quaternion& q){
//    q.w = 0.5*pow(R(0,0)+R(1,1)+R(2,2)+1, 0.5);
//    q.z = (R(1,0)-R(0,1))/(4*q.w);
//    q.y = (R(0,2)-R(2,0))/(4*q.w);
//    q.x = (R(2,1)-R(1,2))/(4*q.w);

    double tr = R(0,0)+R(1,1)+R(2,2);

    if (tr > 0){
        double S = sqrt(tr+1.0)*2; // S = 4*q.w
        q.w = 0.25*S;
        q.x = (R(2,1)-R(1,2))/S;
        q.y = (R(0,2)-R(2,0))/S;
        q.z = (R(1,0)-R(0,1))/S;
    }
    else if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) {
        double S = sqrt(1.0+R(0,0)-R(1,1)-R(2,2))*2; // S = 4*q.x
        q.w = (R(2,1)-R(1,2))/S;
        q.x = 0.25*S;
        q.y = (R(0,1)+R(1,0))/S;
        q.z = (R(0,2)+R(2,0))/S;
    }
    else if (R(1,1) > R(2,2)) {
        double S = sqrt(1.0+R(1,1)-R(0,0)-R(2,2))*2; // S = 4*q.y
        q.w = (R(0,2)-R(2,0))/S;
        q.x = (R(0,1)+R(1,0))/S;
        q.y = 0.25*S;
        q.z = (R(1,2)+R(2,1))/S;
    }
    else {
        double S = sqrt(1.0+R(2,2)-R(0,0)-R(1,1))*2; // S = 4*q.z
        q.w = (R(1,0)-R(0,1))/S;
        q.x = (R(0,2)+R(2,0))/S;
        q.y = (R(1,2)+R(2,1))/S;
        q.z = 0.25*S;
    }
}



#endif // SUBSCRIBER_CLASSES
