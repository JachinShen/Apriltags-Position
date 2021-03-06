#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/features2d/features2d.hpp>
#include "opencv2/imgproc/imgproc.hpp"
using namespace cv;

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <stdlib.h>
using namespace std;

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "AprilTags/QRCode.h"
#define M100_CAMERA 1
#define VIDEO_STREAM 2
  #define CURRENT_IMAGE_SOURCE VIDEO_STREAM
//#define CURRENT_IMAGE_SOURCE M100_CAMERA
#define VISABILITY false

/**global publisher*/
ros::Publisher vision_pillar_pub;
ros::Publisher vision_line_pub;
ros::Publisher vision_base_pub;
image_transport::Subscriber vision_image_sub;

QRCode qr_code;
std::stringstream ss;

/**global video capture and image*/
// cv::Mat g_pillar_image;
// cv::Mat g_line_image;
// cv::Mat g_base_image;
// int g_processed_time = 0;
/**callback of timer*/
// void cap_timer_callback( const ros::TimerEvent &evt );
// void pillar_timer_callback( const ros::TimerEvent &evt );
// void line_timer_callback( const ros::TimerEvent &evt );
// void base_timer_callback( const ros::TimerEvent &evt );

void imageCallBack(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Mat m_copy;
  try
  {
    m_copy=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
	//cv::Mat m_copy = m_origin.clone();
	//cv::cvtColor(m_copy, m_copy, CV_BGR2HSV);
    //cv::imshow("copy", m_copy);
    //cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
  bool base_found;
  float baseDirection;
  ROS_INFO_STREAM("before");
  if(qr_code.getBasePosition(m_copy, 2.4))
  {
    ROS_INFO_STREAM("base position :" << qr_code.getBaseX() << " "
        << qr_code.getBaseY());
    base_found= true;
    qr_code.getBaseDirection(baseDirection);
  }
  else
  {
    ROS_INFO_STREAM("can't find base");
    ROS_INFO_STREAM("base position :" << qr_code.getBaseX() << " "
        << qr_code.getBaseY());
    base_found= false;
  }
  ROS_INFO_STREAM("after");
  ss.str("");
  std_msgs::String base_msg;
  ss << base_found << " " << qr_code.getBaseX() << " "
    << qr_code.getBaseY();
  base_msg.data= ss.str();
  vision_base_pub.publish(base_msg);
  cv::waitKey(1);

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rm_challenge_qrcode_node");
  ros::NodeHandle node;

  vision_base_pub= node.advertise<std_msgs::String>("tpp/base", 1);

  qr_code.setup();

  image_transport::ImageTransport image_transport(node);
  vision_image_sub= image_transport.subscribe("m100/image", 1, imageCallBack);

  // ros::Timer cap_timer =
  //     node.createTimer( ros::Duration( 1.0 / 100.0 ),
  //     cap_timer_callback );
  // ros::Timer pillar_timer =
  //     node.createTimer( ros::Duration( 1.0 / 100.0 ),
  //     pillar_timer_callback );
  // ros::Timer line_timer =
  //     node.createTimer( ros::Duration( 1.0 / 100.0 ),
  //     line_timer_callback );
  // ros::Timer base_timer =
  //     node.createTimer( ros::Duration( 1.0 / 100.0 ),
  //     base_timer_callback );
  // cv::VideoCapture cap(
  // "/home/zby/uav_slam_ws/src/rm_uav/res/color_ball4.avi"
  // );
  ros::spin();

  return 1;
}

// void cap_timer_callback( const ros::TimerEvent &evt )
// {
//     ROS_INFO_STREAM( "process time" << g_processed_time );
//     if ( g_processed_time > 0 )
//         return;

//     if ( g_cap.get( CV_CAP_PROP_POS_FRAMES ) >
//          g_cap.get( CV_CAP_PROP_FRAME_COUNT ) - 2 )
//     {
//         g_cap.set( CV_CAP_PROP_POS_FRAMES, 0 );
//     }
//     Mat frame;
//     g_cap >> frame;
//     ROS_INFO_STREAM( "loop :"
//                      << "\n" );
//     g_pillar_image = frame.clone();
//     g_line_image = frame.clone();
//     g_base_image = frame.clone();
//     g_processed_time = 3;
// }
// void pillar_timer_callback( const ros::TimerEvent &evt )
// {
//     if ( g_processed_time <= 0 )
//         return;

//     RMChallengeVision::PILLAR_RESULT pillar_result;
//     float pos_err_x = 0, pos_err_y = 0, height = 0;
//     vision.detectPillar( g_pillar_image, pillar_result );
//     if ( pillar_result.circle_found )
//     {
//         // calculate height and pos_error
//         height = vision.imageToHeight( pillar_result.radius, 250.0
//         );
//         pos_err_x = vision.imageToRealDistance(
//             pillar_result.radius, pillar_result.circle_center.x,
//             250.0 );
//         pos_err_y = vision.imageToRealDistance(
//             pillar_result.radius, pillar_result.circle_center.y,
//             250.0 );
//     }
//     // publish result to uav
//     std_msgs::String pillar_msg;
//     std::stringstream ss;
//     ss << pillar_result.triangle[0] << " " <<
//     pillar_result.triangle[1] << " "
//        << pillar_result.triangle[2] << " " <<
//        pillar_result.triangle[3] << " "
//        << pillar_result.circle_found << " " << pos_err_x << " " <<
//        pos_err_y << "
//        "
//        << height;
//     pillar_msg.data = ss.str();
//     vision_pillar_pub.publish( pillar_msg );

//     g_processed_time--;
// }
// void line_timer_callback( const ros::TimerEvent &evt )
// {
//     if ( g_processed_time <= 0 )
//         return;

//     float distance_x, distance_y, line_vector_x, line_vector_y;
//     vision.detectLine( g_line_image, distance_x, distance_y,
//     line_vector_x,
//                         line_vector_y );
//     ROS_INFO_STREAM( "line :" << distance_x << " " << distance_y <<
//     line_vector_x
//                               << " " << line_vector_y );
//     // publish result
//     std::stringstream ss;
//     std_msgs::String line_msg;
//     ss << distance_x << " " << distance_y << " " << line_vector_x
//     << " "
//        << line_vector_y;
//     line_msg.data = ss.str();
//     vision_line_pub.publish( line_msg );

//     g_processed_time--;
// }
// void base_timer_callback( const ros::TimerEvent &evt )
// {
//     if ( g_processed_time <= 0 )
//         return;

//     g_processed_time--;
// }
