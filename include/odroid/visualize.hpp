#ifndef VISUALIZE_H
#define VISUALIZE_H
#include <odroid/odroid_node.hpp>

class visualize
{
public:
  ros::Publisher vis_pub_0, vis_pub_1, vis_pub_2, vis_pub_3;
  visualize(){};
  ~visualize(){};
  void publisher_initialization(odroid_node& node);
  void force_markers(odroid_node& node);
};
#endif
