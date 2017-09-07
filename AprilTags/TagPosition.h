#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
using namespace std;

#ifndef __APPLE__
#define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef EXPOSURE_CONTROL
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#endif

// OpenCV library for easy access to USB camera and drawing of images
// on screen
#include "opencv2/opencv.hpp"

// April tags detector and various families that can be selected by command line option
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic();

#include <cmath>

#ifndef PI
const double PI = 3.14159265358979323846;
#endif
const double TWOPI = 2.0*PI;

/**
 * Normalize angle to be within the interval [-pi,pi].
 */
inline double standardRad(double t) {
    if (t >= 0.) {
        t = fmod(t+PI, TWOPI) - PI;
    } else {
        t = fmod(t-PI, -TWOPI) + PI;
    }
    return t;
}

class TagPosition {

    private:

    AprilTags::TagDetector* m_tagDetector;
    AprilTags::TagCodes m_tagCodes;

    bool m_draw; // draw image and April tag detections?
    bool m_arduino; // send tag detections to serial port?
    bool m_timing; // print timing information for each tag extraction call

    int m_width; // image size in pixels
    int m_height;
    double m_tagSize; // April tag side length in meters of square black frame
    double m_fx; // camera focal length in pixels
    double m_fy;
    double m_px; // camera principal point
    double m_py;

    
    int m_deviceId; // camera id (in case of multiple cameras)

    list<string> m_imgNames;

    cv::VideoCapture m_cap;

    int m_exposure;
    int m_gain;
    int m_brightness;

    /**
     * Convert rotation matrix to Euler angles
     */
    void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll);

    bool getCenterPointFrom3Circles(cv::Point2f Point1, float radius1,
            cv::Point2f Point2, float radius2,
            cv::Point2f Point3, float radius3,
            cv::Point2f& centerPoint);

    bool reduceErrorByExtraCircle(cv::Point circlePoint, float radius,
            float& centerPoint_x, float& centerPoint_y);

    public:

    float base_position_x;
    float base_position_y;

    vector<AprilTags::TagDetection> detections;

    // default constructor
    TagPosition();
    // changing the tag family
    void setTagCodes(string s);

    // parse command line options to change default behavior
    void parseOptions(int argc, char* argv[]);

    void setup();

    void setVisability(bool v);
    //void setupVideo();

    void print_detection(AprilTags::TagDetection& detection) const ;
    void getDetectionLocationAndDistance(vector< cv::Point2f >& detections_location,
            vector< float >& detections_distance,
            float detections_height);

    bool getBasePostion(vector< cv::Point2f >& detections_location,
            vector< float >& detections_distance);

    void processImage(const cv::Mat& image, cv::Mat& image_gray);
    // Load and process a single image
    //void loadImages();
    // Video or image processing?
    bool isVideo();

    // The processing loop where images are retrieved, tags detected,
    // and information about detections generated
    //void loop(int argc, char* argv[]);

    bool getBasePosition(const cv::Mat& src, float detections_height);

    float getBaseX();
    float getBaseY();

    bool getBaseDirection(float& baseDirectionCita);

}; // Demo


// here is were everything begins
/*int main(int argc, char* argv[]) {

  Demo demo;

// process command line options
demo.parseOptions(argc, argv);

demo.setup();

// ros::init(argc, argv, "my_tf_broadcaster");
// ros::NodeHandle node;
// tf::TransformBroadcaster br;
// tf::Transform transform_test;

// vector<AprilTags::TagDetection> final_detections;

// transform_test.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
// transform_test.setRotation( tf::Quaternion(0, 0, 0, 1) );
// br.sendTransform(tf::StampedTransform(transform_test, ros::Time::now(), "camera_rgb_optical_frame", "marker_frame"));

if (demo.isVideo()) {
cout << "Processing video" << endl;

// setup image source, window for drawing, serial port...
demo.setupVideo();

// the actual processing loop where tags are detected and visualized
demo.loop(argc,argv);

} else {
cout << "Processing image" << endl;

// process single image
demo.loadImages();

}

    return 0;
}*/

