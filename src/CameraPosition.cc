/**
 * @file CameraPosition.cc
 * @use April tags library to get position
 * @April tags author: Michael Kaess
 * @author: Jachin Shen
 * @E-mail: jachinshen@foxmail.com
 *
 * use april tags on the ground
 * to get the coords and direction
 * of camera's shadow on the ground
 * in the ground coordinate
 */
 

#define M_DRAW true
#define radiusThreshold 0.2
#include "AprilTags/CameraPosition.h"
const char* windowName= "apriltags_demo";

// default constructor
CameraPosition::CameraPosition()
    :  // default settings, most can be modified through command line
        // options (see below)
    m_tagDetector(NULL)
    , m_tagCodes(AprilTags::tagCodes16h5)
    ,

    m_draw(M_DRAW)
    ,

    m_width(640)
    , m_height(480)
    , m_tagSize(0.2286)
    , m_fx(508.013)
    , m_fy(507.49)
    , m_px(322.632)
, m_py(231.39)
    ,

    shadow_x(0.0)
, shadow_y(0.0)
{
}

float CameraPosition::getDistance2Points(cv::Point2f Point1, cv::Point2f Point2)
{
    return sqrt( pow( Point1.x-Point2.x, 2) + 
            pow( Point1.y-Point2.y, 2)); 
}

/* calculate the intersection point of 2 public secant of 3 circles:
 * substract 2 circle equations to get public secant line equation
 * use determinant to get the intersection point of 2 lines
 * so, the point is in the triangle in the overlap region of 3 circles
 */

bool CameraPosition::calculateCenterPointFrom3Circles(cv::Point2f Point1, float radius1,
        cv::Point2f Point2, float radius2,
        cv::Point2f Point3, float radius3,
        cv::Point2f& centerPoint)
{
    float x1= Point1.x, x2= Point2.x, x3= Point3.x;
    float y1= Point1.y, y2= Point2.y, y3= Point3.y;
    float D= 2 * ((x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2));
    if(abs(D) < 1e-7)
        return false;
    float C1= radius1 * radius1 - radius2 * radius2 + x2 * x2 - x1 * x1 +
        y2 * y2 - y1 * y1;
    float C2= radius2 * radius2 - radius3 * radius3 + x3 * x3 - x2 * x2 +
        y3 * y3 - y2 * y2;
    float Dx= C1 * (y3 - y2) - C2 * (y2 - y1);
    float Dy= C2 * (x2 - x1) - C1 * (x3 - x2);
    float centerPoint_x= Dx / D, centerPoint_y= Dy / D;
    /* remove unreasonable points */
    if(centerPoint_x > 0.85 || centerPoint_x < -0.85 ||
            centerPoint_y > 0.85 || centerPoint_y < -0.85)
    {
        return false;
    }
    centerPoint = cv::Point2f(centerPoint_x, centerPoint_y);
    return true;
}

// changing the tag family
void CameraPosition::setTagCodes(string s)
{
    if(s == "16h5")
    {
        m_tagCodes= AprilTags::tagCodes16h5;
    }
    else if(s == "25h7")
    {
        m_tagCodes= AprilTags::tagCodes25h7;
    }
    else if(s == "25h9")
    {
        m_tagCodes= AprilTags::tagCodes25h9;
    }
    else if(s == "36h9")
    {
        m_tagCodes= AprilTags::tagCodes36h9;
    }
    else if(s == "36h11")
    {
        m_tagCodes= AprilTags::tagCodes36h11;
    }
    else
    {
        cout << "Invalid tag family specified" << endl;
        exit(1);
    }
}

void CameraPosition::setup()
{
    m_tagDetector= new AprilTags::TagDetector(m_tagCodes);

    // prepare window for drawing the images
    if(m_draw)
    {
        cv::namedWindow(windowName, 1);
    }

}

void CameraPosition::getShadowLocationRadius()
{
    /* tag coordinates to ground, origin point is in the center
     * same coordinate as uav
     * x -- up arrow
     * y -- right arrow
     */
    static cv::Point2f id2location[12]= {
        cv::Point2f(-0.85, 0.85),   cv::Point2f(-0.85, 0.00),  cv::Point2f(-0.85, -0.85),
        cv::Point2f(0.00, 0.85),  cv::Point2f(0.00, -0.85), cv::Point2f(0.85, 0.85),
        cv::Point2f(0.85, 0.00), cv::Point2f(0.00, 0.00),   cv::Point2f(0.00, 0.00),
        cv::Point2f(0.00, 0.00),   cv::Point2f(0.85, -0.85)
    };
    /* distance from camera's shadow to origin point */
    float shadow_distance;

    /* find the right id tags and get the distance 
     * from camera's shadow on ground to tag*/
    for(int i= 0; i < detections.size(); i++)
    {
        if(detections[i].hammingDistance != 0)
            continue;
        if(!((detections[i].id >= 0 && detections[i].id <= 6) ||
                    detections[i].id == 10))
            continue;
        shadow_location.push_back(id2location[detections[i].id]);

        Eigen::Vector3d translation;
        Eigen::Matrix3d rotation;
        detections[i].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px,
                m_py, translation, rotation);

        shadow_distance=
            sqrt(abs(pow(translation.norm(), 2) - pow(height, 2)));
        shadow_radius.push_back(shadow_distance);
    }
}

bool CameraPosition::calculateShadowXY()
{
    int detections_cnt= shadow_location.size();
    /* no tag then calculate nothing */
    if(detections_cnt == 0)
    {
        shadow_x= 0.0;
        shadow_y= 0.0;
        return false;
    }
    /* less than 3 tags then calculate roughly */
    if(detections_cnt == 1)
    {
        shadow_x= shadow_location[0].x;
        shadow_y= shadow_location[0].y;
        return true;
    }
    if(detections_cnt == 2)
    {
        shadow_x= (shadow_location[0].x + shadow_location[1].x) / 2;
        shadow_y= (shadow_location[0].y + shadow_location[1].y) / 2;
        return true;
    }
    /* calculate accurately */
    if(detections_cnt >= 3)
    {
        cv::Point2f centerPoint;
        vector<cv::Point2f> centerPoints;
        float radius;
        // get centerpoint(s) of all circles
        for(int i= 0; i < shadow_location.size() - 2; i++)
        {
            for(int j= i + 1; j < shadow_location.size() - 1; j++)
            {
                for(int k= j + 1; k < shadow_location.size(); k++)
                {
                    if(calculateCenterPointFrom3Circles(
                                shadow_location[i], shadow_radius[i],
                                shadow_location[j], shadow_radius[j],
                                shadow_location[k], shadow_radius[k], centerPoint))
                    {
                        centerPoints.push_back(centerPoint);
                    }
                }
            }
        }

        /* can't calculate centerPoints 
         * then give rough centerPoint
         */
        if(centerPoints.size() == 0)
        {
            float x_sum= 0, y_sum= 0;
            for(int i= 0; i < detections_cnt; i++)
            {
                x_sum+= shadow_location[i].x;
                y_sum+= shadow_location[i].y;
            }
            centerPoint.x= x_sum / detections_cnt;
            centerPoint.y= y_sum / detections_cnt;
        }
        /* calculate average */
        if(centerPoints.size() > 1)
        {
            float x_sum= 0, y_sum= 0;
            for(int i= 0; i < centerPoints.size(); i++)
            {
                x_sum+= centerPoints[i].x;
                y_sum+= centerPoints[i].y;
            }
            centerPoint.x= x_sum / centerPoints.size();
            centerPoint.y= y_sum / centerPoints.size();
        }
        /* if size = 1, then give the only centerPoint directly
         * if size > 1, then give the average
         */
        shadow_x= centerPoint.x;
        shadow_y= centerPoint.y;
        return true;
    }
}

void CameraPosition::cleanLastTags()
{
    detections.clear();
    shadow_location.clear();
    shadow_radius.clear();
}

bool CameraPosition::isTagEnough()
{
    if(detections.size()==0)
        return false;

    getShadowLocationRadius();
    return calculateShadowXY();

}

void CameraPosition::setImage(const cv::Mat& src)
{
    cleanLastTags();
    cv::cvtColor(src, image_gray, CV_BGR2GRAY);
    detections= m_tagDetector->extractTags(image_gray);
    if(m_draw)
    {
        cv::Mat tagDraw = src.clone();
        for(int i= 0; i < detections.size(); i++)
        {
            // also highlight in the image
            detections[i].draw(tagDraw);
        }
        imshow(windowName, tagDraw);  // OpenCV call
    }
}

void CameraPosition::setHeight(float height)
{
    this->height = height;
}
float CameraPosition::getX()
{
    return shadow_x;
}

float CameraPosition::getY()
{
    return shadow_y;
}

void CameraPosition::setVisability(bool visable)
{
    m_draw= visable;
}

float CameraPosition::getDirection()
{
    if( detections.size() <= 2)
    {
        return 0.0;
    }
    /* id=2 tag as origin point 
     * same coordinate as opencv
     * x -- right arrow
     * y -- down arrow
     */
    static cv::Point2f id2location[12]= {
        cv::Point2f(1.9, 1.9),   cv::Point2f(1.05, 1.9),  cv::Point2f(0.2, 1.9),
        cv::Point2f(1.9, 1.05),  cv::Point2f(0.2, 1.05), cv::Point2f(1.90, 0.2),
        cv::Point2f(1.05, 0.2), cv::Point2f(0.0, 0.0),   cv::Point2f(0.0, 0.0),
        cv::Point2f(0.0, 0.0),   cv::Point2f(0.2, 0.2)
    };
    /* calculate each two-tag line's degree
     * in the ground coordinate and image coordinate.
     * And sub two degree to get camera's rotation to ground
     */
    float tag_vector_x, tag_vector_y, tag_alpha,
          img_vector_x, img_vector_y, img_beta;
    float degree;
    vector< float > degrees;
    float degree_sum = 0;
    for( int i=0; i<detections.size()-1; i++)
    {
        if(detections[i].hammingDistance != 0)
            continue;
        if(!((detections[i].id >= 0 && detections[i].id <= 6) ||
                    detections[i].id == 10))
            continue;    
        for(int j=i+1; j<detections.size(); j++)
        {
            if(detections[j].hammingDistance != 0 ||
                    !( (detections[j].id>=0 &&
                            detections[j].id<=6) ||
                        detections[j].id==10) )
                continue;

            tag_vector_x = id2location[detections[j].id].x
                - id2location[detections[i].id].x;
            tag_vector_y = id2location[detections[j].id].y
                - id2location[detections[i].id].y;
            img_vector_x = detections[j].cxy.first 
                - detections[i].cxy.first;
            img_vector_y = detections[j].cxy.second 
                - detections[i].cxy.second;
            tag_alpha = atan2(tag_vector_x, tag_vector_y)*180/PI;
            img_beta = atan2(img_vector_x, img_vector_y)*180/PI;
            degree = img_beta - tag_alpha;
            degrees.push_back(degree);
        }
    }
    if(degrees.size()==0)
    {
        return 0.0;
    }
    for(int i=0; i<degrees.size(); i++)
    {
        degree_sum += degrees[i];
    }
    return degree_sum / degrees.size();
}
