#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/core/core.hpp>
// #if CV_MAJOR_VERSION == 2
// // do opencv 2 code
// #elif CV_MAJOR_VERSION == 3
// // do opencv 3 code
// #endif
#include <opencv2/opencv.hpp>


static const char WINDOW_NAME[] = "Depth View";
double min_range_;
double max_range_;
image_transport::Publisher *imagePub;
cv::Mat resize;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr bridge;
  sensor_msgs::ImagePtr imagePtr;
  cv_bridge::CvImage resizeRos;
  try
  {
    // cv::imshow("view", cv_bridge::toCvShare(msg, "32FC1")->image);
    // cv::waitKey(30);
    bridge = cv_bridge::toCvCopy(msg, "32FC1");
    cv::resize(bridge->image,resize, cv::Size(320.5,210.5));
  resizeRos.encoding = "32FC1";
  resizeRos.image = resize;


  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
  }



  // cv::Mat resize;//(bridge->image.rows/2, bridge->image.cols/2, CV_8UC1);
try{

  // republish image
  imagePtr=resizeRos.toImageMsg();
  imagePtr->width=320.5;
  imagePtr->height=210.5;

  imagePub->publish(imagePtr);
}
catch(cv_bridge::Exception e){
 ROS_ERROR("cv_bridge exception: %s", e.what());
  return;

}


  // for(int i = 0; i < bridge->image.rows; i = i+2)
  // {
  //     float* Di = bridge->image.ptr<float>(i);
  //     char* Ii = img.ptr<char>(i);
  //     for(int j = 0; j < bridge->image.cols; j = j+2)
  //     {
  //         Ii[j/2] = (char) (255*((Di[j]-min_range_)/(max_range_-min_range_)));
  //     }
  // }

  // display
  cv::imshow(WINDOW_NAME, resize);
  cv::waitKey(3);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  // cv::namedWindow("view");
  // cv::startWindowThread();

  nh.param("min_range", min_range_, 0.5);
  nh.param("max_range", max_range_, 5.5);

  cv::namedWindow(WINDOW_NAME);

  image_transport::ImageTransport it(nh);
  // image_transport::Publisher pub = it.advertise(topic_out, 1);
  // imagePub=&pub;

  image_transport::Subscriber sub = it.subscribe("/camera/depth/image_raw", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
