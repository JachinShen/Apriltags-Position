#include "AprilTags/CameraPosition.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

#include <iostream>
using namespace std;

#define VISABILITY true

int main(int argc, char** argv)
{
    CameraPosition camera_position;
    camera_position.setVisability(VISABILITY);
    camera_position.setTagCodes("16h5");
    camera_position.setup();
    cout<<"CameraPosition OK!"<<endl;

    VideoCapture m_cap("/home/jachinshen/Projects/Cpp/apriltags/src/demo.avi");
    Mat frame;
    if(m_cap.isOpened())
        cout<<"load video OK!"<<endl;
    else
        cout<<"load video failed"<<endl;

    camera_position.setHeight(2.4);
    while(m_cap.read(frame))
    {
        camera_position.setImage(frame);
        if(camera_position.isTagEnough())
        {
            cout<<"X:"<<camera_position.getX()<<" Y:"<<camera_position.getY()<<endl;
            cout<<"Direction: "<<camera_position.getDirection()<<endl;
        }
        waitKey(0);
    }
    m_cap.release();
    return 0;
}
