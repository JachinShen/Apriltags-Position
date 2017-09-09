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

class CameraPosition {

    private:

    AprilTags::TagDetector* m_tagDetector;
    AprilTags::TagCodes m_tagCodes;

    bool m_draw; // draw image and April tag detections?

    int m_width; // image size in pixels
    int m_height;
    double m_tagSize; // April tag side length in meters of square black frame
    double m_fx; // camera focal length in pixels
    double m_fy;
    double m_px; // camera principal point
    double m_py;

    float shadow_x;
    float shadow_y;

    vector<AprilTags::TagDetection> detections;

    cv::Mat image_gray;

    vector< cv::Point2f > shadow_location;
    vector< float > shadow_radius;

    float height;

    float getDistance2Points(cv::Point2f Point1, cv::Point2f Point2);

    bool calculateCenterPointFrom3Circles(cv::Point2f Point1, float radius1,
            cv::Point2f Point2, float radius2,
            cv::Point2f Point3, float radius3,
            cv::Point2f& centerPoint);

    void getShadowLocationRadius();

    bool calculateShadowXY();

    void cleanLastTags();

    public:

    // default constructor
    CameraPosition();
    // changing the tag family
    void setTagCodes(string s);

    void setup();

    /* draw window or not*/
    void setVisability(bool v);

    /* set input image */
    void setImage(const cv::Mat& src);

    /* camera's height */
    void setHeight(float height);

    /* is there enough tags to calculate position */
    bool isTagEnough();
    
    /* coords of camera's shadow on the ground */
    float getX();
    float getY();

    /* the degrees of camera's rotation to ground
     * >0 if the tags is clockwise in the image
     * <0 reverse
     */
    float getDirection();

};
