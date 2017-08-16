#include "AprilTags/QRCode.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;

#include <iostream>
using namespace std;

#define VISABILITY true

int main(int argc, char** argv)
{
  QRCode qr_code;
  qr_code.setVisability(VISABILITY);
  qr_code.setup();
  cout<<"QRCode OK!"<<endl;

  VideoCapture m_cap("~/Projects/Cpp/apriltags/src/demo.avi");
  Mat frame;
  if(m_cap.isOpened())
    cout<<"load video OK!"<<endl;
  else
    cout<<"load video failed"<<endl;

  float direction_degree=0;
  while(m_cap.read(frame))
  {
    cout<<"if has base:"<<qr_code.getBasePosition(frame, 2.4)<<endl;
    cout<<"X:"<<qr_code.getBaseX()<<" Y:"<<qr_code.getBaseY()<<endl;
    qr_code.getBaseDirection(direction_degree);
    cout<<"base direction: "<<direction_degree<<endl;
    //imshow("frame", frame);
    waitKey(0);
  }
  m_cap.release();
  return 0;
}
