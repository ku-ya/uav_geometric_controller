#include <odroid/visualize.hpp>


void visualize::publisher_initialization(odroid_node& node){
  ros::NodeHandle nh = node.getNH();
  vis_pub_0 = nh.advertise<visualization_msgs::Marker>("/force0",1);
  vis_pub_1 = nh.advertise<visualization_msgs::Marker>("/force1",1);
  vis_pub_2 = nh.advertise<visualization_msgs::Marker>("/force2",1);
  vis_pub_3 = nh.advertise<visualization_msgs::Marker>("/force3",1);
}
void visualize::force_markers(odroid_node& node){
    Vector4d f_motor = node.f_motor;
    double scale = node.scale;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = node.vicon_time;
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
    marker.scale.x = f_motor(0) * scale;
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
    marker.scale.x = f_motor(1)* scale;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    vis_pub_1.publish( marker);

    marker.pose.position.x = -0.3;
    marker.pose.position.y = 0;
    marker.scale.x = f_motor(2)* scale;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    vis_pub_2.publish( marker);

    marker.pose.position.x = 0;
    marker.pose.position.y = 0.3;
    marker.scale.x = f_motor(3)* scale;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    vis_pub_3.publish( marker);
  }
