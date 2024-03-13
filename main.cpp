/*
*	@Author: USTL-COD
*	@Date:	 2022.06.05
*	@Brief:  multi-thread starts//奇点，利用互斥锁mutex加锁线程
*/
#include "Buffdetect.h"
#include "AngleSolver.h"
#include "FrameReceiver.h"
#include "SerialTest.h"

#define pi 3.1415926
using namespace cv;
using namespace std;

mutex mtx_pre;
bool rwable_pre=false;
condition_variable cv_pre;
mutex mtx_serialr;
bool rwable_serialr=false;
condition_variable cv_serialr;
mutex mtx_serialt;
bool rwable_serialt=false;
condition_variable cv_serialt;
mutex mtx_date;
bool rwable_date=false;
condition_variable cv_date;
DetectMode detectMode;

int frameSize=6,stateNum = 5, measureNum = 5, controlNum = 5;
double timecolck=0;//as the same of frame
Mat src = Mat::zeros(480, 640, CV_8UC3);   // Transfering buffer
KalmanFilter KF(stateNum, measureNum, controlNum);//状态值测量值5×1向量(x,y,△x,△y,distance)
FrameReceiver frameStream(frameSize);
ArmorMedia armorReceive;
ArmorMedia armorTransmit;
ArmorMedia buffTransmit;


//import armor detector
ArmorDetector armorDetector;
//import angle solver
AngleSolver angleSolver;
AngleSolver angleSolverBuff;

Buffdetect buffdetector;
SerialPort myserial;



//void calAngle(Mat cam, Mat dis, Point2f pxy)
//{

//    double fx = cam.at<double>(0, 0);
//    double fy = cam.at<double>(1, 1);
//    double cx = cam.at<double>(0, 2);
//    double cy = cam.at<double>(1, 2);
//    double k1 = dis.at<double>(0);
//    double k2 = dis.at<double>(1);
//    double p1 = dis.at<double>(2);
//    double p2 = dis.at<double>(3);
//    Point2f pnt;
//    vector<cv::Point2f>in;
//    vector<cv::Point2f>out;
//    in.push_back(pxy);
//    undistortPoints(in, out, cam, dis, noArray(), cam);
//    pnt = out.front();
//    double rx = (pnt.x - cx) / fx;
//    double ry = (pnt.y - cy) / fy;

//    double tanx = (rx);
//    double tany = (ry);
//    cout << out << endl;
//    angx=atan2((pnt.x - cx) , fx) / CV_PI * 180 ;
//    angy= -(atan2((pnt.y - cy) , fy) / CV_PI * 180 );
//    angx = atan(rx) / CV_PI * 180+0.5;
//    angy = -(atan(ry) / CV_PI * 180+1.5);
//    cout << "xscreen: " << pxy.x << " xNew:" << pnt.x << endl;
//    cout << "yscreen: " << pxy.y << " yNew:" << pnt.y << endl;
//    cout << "angx: " << angx << " angleNew:" << atan(rx) / CV_PI * 180 << endl;
//    cout << "angy: " << angy<< " angleNew:" << atan(ry) / CV_PI * 180 << endl;
//}
//typedef  struct {
//    double filterValue;
//    double kalmanGain;
//    double A;
//    double H;
//    double Q;
//    double R;
//    double P;
//}  KalmanInfo;
//double KalmanFilter(KalmanInfo* kalmanInfo, double lastMeasurement);
//void Init_KalmanInfo(KalmanInfo* info, double Q, double R);
/*
* @brief Init_KalmanInfo
* @param info
* @param Q
* @param R
*/
//void Init_KalmanInfo(KalmanInfo* info, double Q, double R)
//{
//    info->A = 1;
//    info->H = 1;
//    info->P = 10;
//    info->Q = Q;
//    info->R = R;
//    info->filterValue = 0;
//}
//double KalmanFilter(KalmanInfo* kalmanInfo, double lastMeasurement)
//{
//    double predictValue = kalmanInfo->A * kalmanInfo->filterValue;

//    kalmanInfo->P = kalmanInfo->A * kalmanInfo->A * kalmanInfo->P + kalmanInfo->Q;
//    double preValue = kalmanInfo->filterValue;

//    kalmanInfo->kalmanGain = kalmanInfo->P * kalmanInfo->H / (kalmanInfo->P * kalmanInfo->H * kalmanInfo->H + kalmanInfo->R);
//    kalmanInfo->filterValue = predictValue + (lastMeasurement - predictValue) * kalmanInfo->kalmanGain;
//    kalmanInfo->P = (1 - kalmanInfo->kalmanGain * kalmanInfo->H) * kalmanInfo->P;

//    return  kalmanInfo->filterValue;
//}


int main()
{
    XInitThreads();
    thread thread1(imageUpdatingThread);
    thread thread2(preimgUpdatingThread);
    thread thread3(serialRUpdatingThread);
    thread thread4(datasolutingThread);
    thread thread5(serialTUpdatingThread);
    thread thread6(buffDetectionThread);
    thread1.join();
    thread2.join();
    thread3.join();
    thread4.join();
    thread5.join();
    thread6.join();
    return 0;
}

void buffDetectionThread(void)
{

    while(true)
    {
        if(detectMode == DetectMode::Armor)
        {
//            buffdetector.BuffReset();
            continue;
        }
        Frame Buff_frame;
        buffdetector.setBuffColor(armorReceive.enemycolor);
        if(frameStream.getLatest(Buff_frame))
        {
            src=Buff_frame.frame;
        }
        else
        {
            //cout<<"getLast test"<<endl;
            continue;
        }
//        std::unique_lock<std::mutex> lck_serialr(mtx_serialr);
//        cv_serialr.wait(lck_serialr,[](){return rwable_serialr;});
    
//        std::unique_lock<std::mutex> lck_date(mtx_date);
//        cv_date.wait(lck_date,[](){return rwable_date;});
//        buffdetector.beneficiate(src,detectMode);
        buffdetector.Get_gyro_pos(armorReceive.gyro_yaw,armorReceive.gyro_pitch);
        buffdetector.get_temp_buff(src);
        buffdetector.buffmsgTran(buffTransmit.yaw, buffTransmit.pitch);
        rwable_serialr=false;
        rwable_date=false;
        rwable_serialt=true;
        cv_serialt.notify_one();
    }
}
//KalmanInfo KFx,KFy;

void imageUpdatingThread(void)
{
    frameStream.getFrames();
}

void preimgUpdatingThread(void)
{

    int n=0;
    while(true)
    {
        if(detectMode ==DetectMode::Minbuff || detectMode == DetectMode::Bigbuff)
            continue;
        // FPS
        Frame frame;
        double t = (double)getTickCount();

        if(frameStream.getLatest(frame))
        {
            src=frame.frame;
        }
        else
        {
            //cout<<"getLast test"<<endl;
            continue;
        }
        //装甲板检测识别子核心集成函数
        armorDetector.prerun(src);

        //FPS
        t = (getTickCount() - t) / getTickFrequency();
        cout<<"wasteime_preimg:"<<t*1000<<"n_pre:"<<n++<<endl;
        //printf("FPS_preimg: %f\n", 1/t);

        timecolck=frame.ftime;
        cout<<"timecolck:"<<timecolck;

        rwable_serialr=true;
        cv_serialr.notify_one();
    }
}


void serialRUpdatingThread(void)
{

    int n=0;
    while(true)
    {
        // FPS
        double t = (double)getTickCount();

        myserial.SerialWaterFish(armorReceive.enemycolor,armorReceive.targetnum,
                                 armorReceive.predict,detectMode,armorReceive.gyro_pitch,armorReceive.gyro_yaw);
        //FPS
        t = (getTickCount() - t) / getTickFrequency();
//        cout<<"wasteime_serial:"<<t*1000<<"n_serial:"<<n++<<endl;
        //printf("FPS_serial: %f\n", 1/t);

        rwable_date=true;
        cv_date.notify_one();
    }
}

void datasolutingThread(void)
{
    //waitKey(3000);
    int n=0;
    mutex copy_pre;
    bool flag_Num;
    while (true)
    {
        // FPS
        double t = (double)getTickCount();

        std::unique_lock<std::mutex> lck_serialr(mtx_serialr);
        cv_serialr.wait(lck_serialr,[](){return rwable_serialr;});

        std::unique_lock<std::mutex> lck_date(mtx_date);
        cv_date.wait(lck_date,[](){return rwable_date;});

        copy_pre.lock();
        armorDetector.daterun();
        copy_pre.unlock();

        //FPS
        double t1 = (getTickCount() - t) / getTickFrequency();
        cout<<"wasteime_copy:"<<t1*1000<<endl;

        armorDetector.setEnemyColor(armorReceive.enemycolor); //here set enemy color
        armorDetector.setTargetNum(armorReceive.targetnum);
        angleSolver.setBulletSpeed(armorReceive.bullet_v);
        angleSolver.setGyroYaw(armorReceive.gyro_yaw);
        angleSolver.setGyroPitch(armorReceive.gyro_pitch);

        //给角度解算传目标装甲板值的实例
        if (armorDetector.isFoundArmor())
        {
            armorDetector.getTargetInfo(armorReceive.contourPoints, armorReceive.centerPoint, armorReceive.type, flag_Num);
            angleSolver.getAngle(armorReceive.contourPoints, armorReceive.centerPoint, armorReceive.type, flag_Num,
                armorTransmit.yaw, armorTransmit.pitch, armorTransmit.distance, armorTransmit.tar_predict);
        }


        if (armorDetector.isFoundArmor())
        {
            //printf("Found Target! Center(%d,%d)\n", (int)centerPoint.x, (int)centerPoint.y);
            cout << "Yaw: " << armorTransmit.yaw << "Pitch: " << armorTransmit.pitch << "Distance: " << armorTransmit.distance << endl;
        }

//#ifdef DEBUG_MODE
            //********************** DEGUG **********************//
            //装甲板检测识别调试参数是否输出,disable motor action
            //param:
            //		1.showSrcImg_ON,		  是否展示原图
            //		2.bool showSrcBinary_ON,  是否展示二值图
            //		3.bool showLights_ON,	  是否展示灯条图
            //		4.bool showArmors_ON,	  是否展示装甲板图
            //		5.bool textLights_ON,	  是否输出灯条信息
            //		6.bool textArmors_ON,	  是否输出装甲板信息
            //		7.bool textScores_ON	  是否输出打击度信息
            //					    1  2  3  4  5  6  7
        armorDetector.showDebugInfo(0, 1, 0, 1, 0, 0, 0);
        if (armorDetector.isFoundArmor())
        {
            //角度解算调试参数是否输出
            //param:
            //		1.showCurrentResult,	  是否展示当前解算结果
            //		2.bool showTVec,          是否展示目标坐标
            //		3.bool showP4P,           是否展示P4P算法计算结果
            //		4.bool showPinHole,       是否展示PinHole算法计算结果
            //		5.bool showCompensation,  是否输出补偿结果
            //		6.bool showCameraParams	  是否输出相机参数
            //					      1  2  3  4  5  6
            angleSolver.showDebugInfo(0, 0, 0, 0, 0, 0);
        }
        waitKey(1);
//#endif

        //FPS
        t = (getTickCount() - t) / getTickFrequency();
        cout<<"wasteime_date:"<<t*1000<<"n_date:"<<n++<<endl;
        printf("FPS_date: %f\n", 1/t);

        rwable_serialr=false;
        rwable_date=false;
        rwable_serialt=true;
        cv_serialt.notify_one();
    }
}
void serialTUpdatingThread(void)
{
//     cout<<"@@@@@@@@@@@@@@@@@@@@@@"<<endl;
    while(true)
    {

        cout<<"@@@@@@@@@@@@@@@@@@@@@@"<<endl;
        std::unique_lock<std::mutex> lck_serialt(mtx_serialt);
        cv_serialt.wait(lck_serialt,[](){return rwable_serialt;});
        if(detectMode == DetectMode::Minbuff || detectMode == DetectMode::Bigbuff)
            myserial.SerialFireFish(buffTransmit.yaw,buffTransmit.pitch,armorTransmit.distance, true, buffdetector.isFindbuff() );
        else
            myserial.SerialFireFish(armorTransmit.yaw, armorTransmit.pitch,armorTransmit.distance, true, armorDetector.isFoundArmor());

        rwable_serialt=false;
    }
}
