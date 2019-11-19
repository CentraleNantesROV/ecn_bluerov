#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_publisher.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char ** argv)
{
  // initialisation du node ROS
  ros::init(argc, argv, "blurr_cam");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("image", 1);

  cv_bridge::CvImage bridge;
  bridge.encoding = sensor_msgs::image_encodings::BGR8;
  bridge.header.frame_id = "camera";

  ros::Rate loop(10);
  cv::VideoCapture cap(0);

  while(ros::ok())
  {
    cap.read(bridge.image);
    bridge.header.stamp = ros::Time::now();

    pub.publish(bridge.toImageMsg());

    ros::spinOnce();
    loop.sleep();
  }
}
