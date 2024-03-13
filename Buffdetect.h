#pragma once
#include "main.h"
#include "ArmorDeep.h"
#include "ArmorBox.h"
#include "AngleSolver.h"
#include "FrameReceiver.h"


using namespace std;
using namespace cv;

//#define SHOW_ALL_CONTOUR
//#define LEAF_IMG
#define USE_TEMPLATE
//#define USE_SVM
//#define DEBUG
//#define DEBUG_LOG
#define SHOW_CIRCLE
#define SHOW_RESULT

class Buffdetect
{
public :
    Buffdetect();
    ~Buffdetect();

    void BuffReset();

    void setBuffColor(int buffColornum);

    void calAngle(Mat cam, Mat dis, Point2f pxy);

    double getDistance(Point2f A, Point2f B);

    double Square(Point2f A);

    Point2f P_R(Mat& image);

    int getDirection(double sinta1, double sinta2) ;

    Point2f minbuff(double& sinta1, double& R, Point2f q,int direcetion);

    Point2f maxbuff(double& sinta3, double& R, Point2f q, double &V,int direcetion);

    void beneficiate(Mat frame, DetectMode detectMode);

    void buffmsgTran(double& yaw, double& pitch);

    bool isFindbuff();

    vector<float> stander(Mat im);

    Mat get(Mat input);

    bool CircleInfo2(std::vector<cv::Point2f>& pts, cv::Point2f& center, float& radius);

    double TemplateMatch(cv::Mat image, cv::Mat tepl, cv::Point &point, int method);

    void get_temp_buff(Mat srcImage);

    void Get_gyro_pos(float yaw,float pitch);

//    void Set_circle_center();

//    Point2f get_circle_center();

private :

    float gyr_yaw;
    float gyr_pitch;
    int Buffcolor;
    KalmanFilter KF;
    Mat CAMERA_MATRIX,DISTORTION_COEFF;
    double minT;
    int DirectionDetec_count;
//     KalmanInfo KFx,KFy;
    bool findfirstpose;
    bool FindBuff;
    double angx;
    double angy;
    double center_yaw;
    double center_pitch;
    int anglecount;
    Point2f sumangle;
    Point2f lastsumangle;

    //rebuild
    Mat templ[9];
    vector<Point2f> cirV;
    Point2f cc=Point2f(0,0);
    Ptr<SVM>buffSVM;
    Mat drawcircle;

};

