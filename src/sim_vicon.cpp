#include <uav_controller/node.hpp>

class Converter
{
public:
  Converter()
  {
    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::TransformStamped>("vicon/Maya/Maya", 10);
    //Topic you want to subscribe
    sub_ = n_.subscribe("ground_truth_to_tf/pose", 10, &Converter::callback, this);
  }

  void callback(const geometry_msgs::PoseStamped::ConstPtr& input)
  {
  geometry_msgs::TransformStamped msg;
  msg.child_frame_id = "vicon";
  msg.header.stamp = input->header.stamp;
  msg.transform.translation.x = input->pose.position.x;
  msg.transform.translation.y = input->pose.position.y;
  msg.transform.translation.z = input->pose.position.z;
  msg.transform.rotation.x = input->pose.orientation.x;
  msg.transform.rotation.y = input->pose.orientation.y;
  msg.transform.rotation.z = input->pose.orientation.z;
  msg.transform.rotation.w = input->pose.orientation.w;
    pub_.publish(msg);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv){
  // ros::init(argc, argv, "imu_listener");
  ros::init(argc,argv,"sim_vicon");

  Converter gazebo_to_vicon;
  ros::spin();


  return 0;
}
