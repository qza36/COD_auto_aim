#include <Buffdetect.h>

Buffdetect::Buffdetect()
{
    minT=0.001;
    anglecount = 0;
    DirectionDetec_count = 0;
    //     KalmanInfo KFx,KFy;
    FindBuff = false;
    FileStorage fsRead;
    for(int i=1;i<=8;i++)
    {
        templ[i]=imread("/home/cod-2th/Desktop/HeroVidion_dafu_rebuild/template/template"+to_string(i)+".jpg",IMREAD_GRAYSCALE);
    }
    buffSVM = StatModel::load<SVM>("/home/cod-2th/Desktop/HeroVidion_dafu_rebuild/SVM4_9.xml");
    fsRead.open("/home/cod-2th/Desktop/file_calibration/HK_Infantry2_0609_640x480.xml", FileStorage::READ);
    if (!fsRead.isOpened())
        cout << "Failed to open xml" << endl;
    switch (1)
    {
    case 1:
        fsRead["camera_matrix"] >> CAMERA_MATRIX;
        fsRead["distortion_coefficients"] >> DISTORTION_COEFF;
        break;
    default:
        cout << "WRONG CAMID GIVEN!" << endl;
        break;
    }
    fsRead.release();
    drawcircle=Mat(480,640,CV_8UC3,Scalar(0,0,0));

}

Buffdetect::~Buffdetect() {}

void Buffdetect::setBuffColor(int buffColornum) {
    buffColornum == 0 ? this->Buffcolor = 2 : this->Buffcolor = 0;
}

void Buffdetect::BuffReset()
{
    anglecount = 0;
    DirectionDetec_count = 0;
    sumangle = Point2f(0.0,0.0);
    lastsumangle = Point2f(0.0,0.0);
    findfirstpose = false;
}

void Buffdetect::buffmsgTran(double& yaw, double& pitch)
{
    yaw = angx;
    pitch = angy;
}

void Buffdetect::Get_gyro_pos(float yaw, float pitch)
{
    gyr_yaw = yaw;
    gyr_pitch = pitch;
}

bool Buffdetect::isFindbuff()
{
    return FindBuff;
}

void Buffdetect::calAngle(Mat cam, Mat dis, Point2f pxy)
{

    double fx = cam.at<double>(0, 0);
    double fy = cam.at<double>(1, 1);
    double cx = cam.at<double>(0, 2);
    double cy = cam.at<double>(1, 2);
    double k1 = dis.at<double>(0);
    double k2 = dis.at<double>(1);
    double p1 = dis.at<double>(2);
    double p2 = dis.at<double>(3);
    Point2f pnt;
    vector<cv::Point2f>in;
    vector<cv::Point2f>out;
    in.push_back(pxy);
    undistortPoints(in, out, cam, dis, noArray(), cam);
    pnt = out.front();
    double rx = (pnt.x - cx) / fx;
    double ry = (pnt.y - cy) / fy;

    double tanx = (rx);
    double tany = (ry);
    cout << out << endl;
    angx=atan2((pnt.x - cx) , fx) / CV_PI * 180 +0.5;
    angy= -(atan2((pnt.y - cy) , fy) / CV_PI * 180 +2.5);
    center_yaw = atan2((pnt.x - cx) , fx) / CV_PI * 180;
    center_pitch = -(atan2((pnt.y - cy) , fy) / CV_PI * 180);
    //    angx = atan(rx) / CV_PI * 180+0.5;
    //    angy = -(atan(ry) / CV_PI * 180+1.5);
    //    cout << "xscreen: " << pxy.x << " xNew:" << pnt.x << endl;
    //    cout << "yscreen: " << pxy.y << " yNew:" << pnt.y << endl;
    //    cout << "angx: " << angx << " angleNew:" << atan(rx) / CV_PI * 180 << endl;
    //    cout << "angy: " << angy<< " angleNew:" << atan(ry) / CV_PI * 180 << endl;
}

double Buffdetect::getDistance(Point2f A, Point2f B)
{
    double dis;
    dis = pow((A.x - B.x), 2) + pow((A.y - B.y), 2);
    return sqrt(dis);
}

double Buffdetect::Square(Point2f A)
{
    double dis;
    dis = pow(A.x, 2) + pow(A.y, 2);
    //cout << "R=" << dis << endl;
    return sqrt(dis);
}

Point2f Buffdetect::P_R(Mat& image)
{
    Mat image1;
    image.copyTo(image1);
    threshold(image, image, 50, 255, THRESH_BINARY);
    Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));

    erode(image, image, element2);
    dilate(image, image, element2);

    //imshow("image1", image);
    vector<vector<Point>> contours2;
    vector<Vec4i> hierarchy2;
    findContours(image, contours2, hierarchy2, RETR_TREE, CHAIN_APPROX_SIMPLE);


    RotatedRect rect_tmp2;
    //    bool findTarget = 0;
    vector<Point>P_R;
    //    Point centerP;
    RotatedRect rect_tmp, rect;
    //    Point PP;
    //    bool flag_R = false;
    for (int i = 0; i < contours2.size(); i++)
    {
        //        float area = contourArea(contours2[i]);
        // cout << "contours2[" << i << "]:" << area<<endl;
        rect = minAreaRect(contours2[i]);


    }

    if (hierarchy2.size()) {
        for (int i = 0, j = 0; i >= 0; i = hierarchy2[i][0], j++)
        {

            rect_tmp2 = minAreaRect(contours2[i]);
            Point2f P[4];
            rect_tmp2.points(P);

            Point2f srcRect[4];
            //            Point2f dstRect[4];

            double width;
            double height;

            //������ȡ��ҶƬ�Ŀ���
            width = getDistance(P[0], P[1]);
            height = getDistance(P[1], P[2]);
            if (width > height)
            {
                srcRect[0] = P[0];
                srcRect[1] = P[1];
                srcRect[2] = P[2];
                srcRect[3] = P[3];
            }
            else
            {
                swap(width, height);
                srcRect[0] = P[1];
                srcRect[1] = P[2];
                srcRect[2] = P[3];
                srcRect[3] = P[0];
            }

            double area = height * width;
            double area1 =  width /height ;
            //cout << "area1=" << area1 << endl;
            //cout << "area "<<area << endl;
            if (area > 70 && area < 300 && area1>0.9 && area1<1.5)
                //if (area > 50 && area < 100  && area1>0.8 && area1<1.2)//R��//�����ϱ�ֵ�����ģ��ģ�����svmѵ���õ�R //15 50100
            {
                P_R.push_back(rect_tmp2.center);

                return rect_tmp2.center;
            }
            else if (P_R.size() > 1)break;
        }
    }
}

int Buffdetect::getDirection(double sinta1, double sinta2)
{
    int   T = 1;
    //if (sinta1 = sinta2) { return 0; }; ;
    //    cout << "sinta2=" << sinta2 << endl;
    //    cout << "sinta1=" << sinta1 << endl;
    if (sinta2 > sinta1) {
        //sinta2 = sinta2+pi;

        // T = -T;
        //        cout << "shun" << endl;
        //        cout << T << endl;
        return T;
    }
    if(sinta2<sinta1) {
        //        cout << "ni" << endl;
        return -T;
    }
}

Point2f Buffdetect::minbuff(double& sinta1, double& R, Point2f q,int direcetion) {
    double  X0 = 0, Y0 = 0, sinta2 = 0, W=166.666666;

    sinta2 = W * minT*direcetion  + sinta1;//�õ��ٶȺ󣬽���Ҫ����Ŀ����ǰ��ʱ�����ú�
    cout << "t=" << minT<< endl;
    X0 = (cos(sinta2) * R + q.x); Y0 = (sin(sinta2) * R + q.y);//������Ԥ������ͼ���е�����ϵ��
    //�õ�Ԥ��Ŀ��
    //cout << "X0=" << X0 << "Y0=" << Y0 << endl;
    return Point2f(X0, Y0);
    //cv::circle(binary, Point2f(X0, Y0), 1, cv::Scalar(0, 255, 255), 5);
}

Point2f Buffdetect::maxbuff(double& sinta3, double& R, Point2f q, double &V,int direcetion) {
    //    double T = 30, X0 = 0, Y0 = 0, Totaldistance = 0, distance = 0, sinta2 = 0;
    //    sinta2 = (((-1 / W) * A * cos(W * T) + 2.090 * T - A * T) - ((-1 / W) * A * cos(W * end1) + 2.090 * end1 - A * end1)) / R + sinta1;
    //    X0 = (cos(sinta2) * R + q.x); Y0 = (sin(sinta2) * R + q.y);
    //    return Point2f(X0, Y0);
    double  X0 = 0, Y0 = 0,  sinta2 = 0,tmax=7,distanceangle=0;
    if(V>20.090){V=20.090;}
    sinta2 = V * tmax*direcetion / R + sinta3;//�õ��ٶȺ󣬽���Ҫ����Ŀ����ǰ��ʱ�����ú�
    cout << "simta2=" << sinta2 << endl;
    cout << "simta3=" << sinta3 << endl;
    distanceangle=abs(sinta2-sinta3);
    cout << "distanceangle=" << distanceangle << endl;
    //     if(distanceangle>0.4){
    //         sinta2=sinta3;
    //     }

    /* if ((3*pi/2)<sinta2<(pi/2))
     {
         X0 = (cos(sinta2) * R + q.x); Y0 = (sin(sinta2) * R + q.y);
     }*/
    // cout << "q= " << q.y << endl;
    X0 = (cos(sinta2) * R + q.x); Y0 = (sin(sinta2) * R + q.y);//������Ԥ������ͼ���е�����ϵ��
    //�õ�Ԥ��Ŀ��
    // cout << "X0=" << X0 << "Y0=" << Y0 << endl;
    return Point2f(X0, Y0);
    //cv::circle(binary, Point2f(X0, Y0), 1, cv::Scalar(0, 255, 255), 5);

}

void Buffdetect::beneficiate(Mat image, DetectMode detectMode) {
    //    time_t begin, end, begin1 = 0, end1 = 0;


    //       Init_KalmanInfo(&KFx, 1, 13);
    //        Init_KalmanInfo(&KFy, 1, 13);

    Point2f maxcenter[10] ;
    double   ret1[10];


    Mat  binary;
    Mat midImage1, midImage2;
    //    int stateNum = 4;
    //    int measureNum = 2;

    FindBuff = false;

    //        std::unique_lock<std::mutex> lck_serialr(mtx_serialr);
    //        cv_serialr.wait(lck_serialr,[](){return rwable_serialr;});

    //        std::unique_lock<std::mutex> lck_date(mtx_date);
    //        cv_date.wait(lck_date,[](){return rwable_date;});

    //    image = framemsg.frame;
    vector<Mat> imgChannels;
    //capture.copyTo(image);
    auto beginTime = chrono::high_resolution_clock::now();

    image.copyTo(binary);
    //resize(image, image, Size(image.cols * 0.5, binary.rows * 0.5));
    //resize(binary, binary, Size(binary.cols * 0.5, binary.rows * 0.5));
    // resize(midImage2, midImage2, Size(midImage2.cols * 0.5, midImage2.rows * 0.5));
    split(image, imgChannels);

    if (Buffcolor == 0)
    {
        midImage1 = imgChannels[0] - imgChannels.at(2);
    }
    else if (Buffcolor == 2)
    {
        midImage1 = imgChannels.at(2) - imgChannels.at(0);
    }

    midImage1.copyTo(midImage2);
    Point2f q, Vcenter;
    q = P_R(midImage2);
    cv::circle(binary, Point(q.x, q.y), 5, cv::Scalar(0, 111, 255), -1);
    Mat element1 = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    //    Mat element2 = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    threshold(midImage1, midImage1, 50, 255, THRESH_BINARY);//80
    //    erode(midImage1, midImage1, element2);
    dilate(midImage1, midImage1, element1);

    //imshow("2", midImage1);
    floodFill(midImage1, Point(5, 50), Scalar(255), 0, FLOODFILL_FIXED_RANGE);
    threshold(midImage1, midImage1, 80, 255, THRESH_BINARY_INV);
    //imshow("1", midImage1);
    vector<vector<Point>> contours;
    findContours(midImage1, contours, RETR_LIST, CHAIN_APPROX_NONE);

    for (size_t i = 0; i < contours.size(); i++) {

        vector<Point> points;
        double area = contourArea(contours[i]);
        // cout << "area=" << area << endl;
        if (area < 50 || 3000 < area) continue;

        drawContours(image, contours, static_cast<int>(i), Scalar(0), 2);

        points = contours[i];
        RotatedRect rrect = fitEllipse(points);
        cv::Point2f* vertices = new cv::Point2f[4];
        rrect.points(vertices);
        float aim = rrect.size.height / rrect.size.width;
        if (aim > 1.7 && aim < 2.6) {//2.6
            for (int j = 0; j < 4; j++)
            {

                cv::line(binary, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0), 4);
            }
            float middle = 100000;

            for (size_t j = 1; j < contours.size(); j++) {

                vector<Point> pointsA;
                double area = contourArea(contours[j]);
                //cout << "area=" << area << endl;
                if (area < 50 || 3000 < area) continue;
                cout << "area=" << area << endl;
                pointsA = contours[j];
                RotatedRect rrectA = fitEllipse(pointsA);
                float aimA = rrectA.size.height / rrectA.size.width;
                cout << "tiaoshi1=" << aimA << endl;
                if (aimA > 3.00) {//3.0||aimA<1.9

                    float distance = sqrt((rrect.center.x - rrectA.center.x) * (rrect.center.x - rrectA.center.x) +
                                          (rrect.center.y - rrectA.center.y) * (rrect.center.y - rrectA.center.y));

                    if (middle > distance) {
                        middle = distance;
                    }

                }
            }
            cout << "tiaoshi2=" <<middle << endl;
            if (middle > 60) {

                cv::circle(binary, Point(rrect.center.x, rrect.center.y), 15, cv::Scalar(0, 0, 255), 4);
                //cout <<" c = "<<rrect.center<< endl;
                double R = 0;

                double ret = (static_cast<chrono::duration<double, std::milli>>(chrono::high_resolution_clock::now() - beginTime)).count();
                double Direction1[10] ,sumret=0,Direction=0, V=0, Vx=0, Vy=0, Vsumx=0, Vsumy=0;

                double maxsumx1=0,maxsumx2=0,maxsumy1 = 0, maxsumy2 = 0;
                ret1[9] = ret;
                cout<< " ret = " << ret << endl;
                double  sinta1 = 0, sinta2 = 0, sinta3 = 0, acceleration = 0, A, T ;

                // cout << "q" << q << endl;
                Vcenter = rrect.center - q;
                maxcenter[9] = Vcenter;
                //cout << " Vcenter = " << Vcenter << endl;

                R = Square(Vcenter);
                /* cout.setf(ios::fixed);
                    cout << setprecision(2);*/

                //sumcenter=center1[1] - center1[0];
                //cout << "center11=" << center1[0] << endl;
                //cout << "center12=" << center1[1] << endl;
                //                    Vsumx = (sumx2 - sumx1)/5;
                //                    Vsumy = (sumy2 - sumy1)/5;

                //                for (n = 0; n < 2; n++) {
                //                    //cout<<"zheng="<<n<<endl;
                //                    if (n < 1) {
                //                        sumx1 = center[n].x;
                //                        sumy1 = center[n].y;
                //                        //cout <<"x" << center[n].x << endl;
                //                        //cout << center[zheng].y << endl;
                //                    }
                //                    else {
                //                        sumx2 = center[n].x;
                //                        sumy2 = center[n].y;

                //                    }
                //                }
                //                for (n = 0; n < 1; n++) {
                //                    center[n] = center[n + 1];//�ƶ���������Ԫ�أ���Ϊ����ͼ���仯����Ҫ�����͸�������
                //                }
                if (findfirstpose == false)
                {
                    sumangle = Vcenter;
                    findfirstpose = true;
                    return;
                }
                lastsumangle = sumangle;
                sumangle = Vcenter;
                //sumcenter=center1[1] - center1[0];
                //cout << "center11=" << center1[0] << endl;
                //cout << "center12=" << center1[1] << endl;
                sinta2 = atan(lastsumangle.y / lastsumangle.x);
                sinta1 = atan(sumangle.y / sumangle.x);
                if (DirectionDetec_count < 10) {
                    Direction1[DirectionDetec_count] = getDirection(sinta2, sinta1);
                    //cout <<"********Direction"<< Direction1[f] << endl;
                    anglecount=Direction1[DirectionDetec_count]>0.0 ? anglecount+1 : anglecount-1;
                    cout << "******anglecount"<<anglecount << endl;
                    DirectionDetec_count++;
                    return;
                }
                //cout << "******anglecount"<<anglecount << endl;
                Direction = anglecount > 0 ? 1 : -1;
                //cout << "T=****"<<Direction << endl;

                for (int zheng = 0; zheng < 10; zheng++) {
                    //cout<<"zheng="<<zheng<<endl;
                    if (zheng < 5) {
                        maxsumx1 += maxcenter[zheng].x;
                        maxsumy1 += maxcenter[zheng].y;
                        //cout <<"x=" << maxcenter[zheng].x << endl;
                        //cout << maxcenter[zheng].y << endl;
                    }
                    else {
                        maxsumx2 += maxcenter[zheng].x;
                        maxsumy2  += maxcenter[zheng].y;

                    }
                    //continue;
                }
                for ( int zheng = 0; zheng < 9; zheng++) {
                    maxcenter[zheng] = maxcenter[zheng + 1];//�ƶ���������Ԫ�أ���Ϊ����ͼ���仯����Ҫ�����͸�������
                }
                Vsumx = (maxsumx2 - maxsumx1);
                Vsumy = (maxsumy2 - maxsumy1);
                //                   //lowpass
                //                   /*float addx,Kx;
                //                   double Vsumx_last;
                //                   Kx = 0.6;
                //                   addx = (Vsumx - Vsumx_last) * Kx;
                //                   Vsumx = add + Vsumx_last;
                //                   Vsumx_last = Vsumx;
                //                   float addy,Ky;
                //                   double Vsumy_last;
                //                   Ky = 0.6;
                //                   addy = (Vsumy - Vsumy_last) * Ky;
                //                   Vsumy = add + Vsumy_last;
                //                   Vsumy_last = Vsumy;*/

                // Vsumx = ::KalmanFilter(&KFx,  Vsumx);
                //Vsumy = ::KalmanFilter(&KFy,  Vsumy);
                cout << " Vsumx=" << Vsumx << endl;
                cout << " Vsumy=" << Vsumy << endl;
                //lowpass
                Vx = pow(Vsumx ,2);
                Vy = pow(Vsumy ,2);
                static float addx,Kx;
                static double Vx_last;
                Kx = 0.8;
                addx = (Vx - Vx_last) * Kx;
                Vx = addx + Vx_last;
                Vx_last = Vx;
                static float addy,Ky;
                static double Vy_last;
                Ky = 0.8;
                addy = (Vy - Vy_last) * Ky;
                Vy = addy + Vy_last;
                Vy_last = Vy;



                for (int n = 0; n < 10; n++) {
                    // cout << "time=" << ret1[n] << endl;//��Ϊͼ��ɸѡԭ�򣬵õ�����Ҷ
                    //�������ݲ��ȶ��������Ҳ���ȡ5��ͼ������Ҷ����λ��
                    sumret += (ret1[n]/10);              //��ƽ��ֵ���ȶ���Ҷ��������ϵ��λ��
                    //10������Ϊ��Ҫ�õ�λ����dr��dr=rĩ-r��
                }
                cout<<"@@@@@@@@@@@@@@@@"<<sumret<<endl;

                /*cout<<"Vx="<< Vx
                        <<"Vy="<<Vy<<endl;*/
                for (int n = 0; n < 9; n++) {                //Ҳ��Ҫ�ۼ�
                    ret1[n] = ret1[n + 1];
                }
                //V = sqrt(Vx + Vy)/sumret;
                V = sqrt(Vx_last + Vy_last)/sumret;//dv=dr/dt,�õ��ٶ�
                cout << "V=" << V   << endl;//ע�������õ�������ϵ�Ѿ�������ͼ��������ϵ��


                //                    cout << "sumx2 =" << sumx2 << endl;
                //                    cout << "sumy2 =" << sumy2 << endl;
                //                    cout << "sumx1 =" << sumx1 << endl;
                //                    cout << "sumy1 =" << sumy1 << endl;



                sinta3 = atan(Vcenter.y / Vcenter.x);
                if (Vcenter.x < 0) {
                    sinta3 = sinta3 + M_PI;
                }
                // acceleration = V / sumret;
                Point2f pxy;
                //KalmanInfo KF;
                //                    int cfd = 0;
                //                    if (cfd == 0) {
                //                        Init_KalmanInfo(&KF, 30, 1);

                //                        cfd = 1;
                //                    }
                // if ( acceleration < 0.02) {
                //V = ::KalmanFilter(&KF, V);
                //chuankou 1 minbuff 2maxbuff
                if(detectMode == DetectMode::Minbuff)
                    pxy = minbuff(sinta3, R, q,Direction);
                else
                    pxy = maxbuff(sinta3, R, q,V,Direction);
                FindBuff = true;
                calAngle (CAMERA_MATRIX, DISTORTION_COEFF,  pxy);

                //angleSolver.PinHole_solver(CAMERA_MATRIX, DISTORTION_COEFF, pxy);
                //cout << "tiaoshi3=" << cfd << endl;
                cv::circle(binary, pxy, 1, cv::Scalar(0, 255, 255), 5);

            }
        }
    }
    imshow("frame", binary);
    waitKey(1);
}

vector<float> Buffdetect::stander(Mat im)
{

    if(im.empty()==1)
    {
        cout<<"filed open"<<endl;
    }
    resize(im,im,Size(48,48));

    vector<float> result;

    HOGDescriptor hog(Size(48,48),Size(16,16),Size(8,8),Size(8,8),9,1,-1,
                      HOGDescriptor::L2Hys,0.2,false,HOGDescriptor::DEFAULT_NLEVELS);           //初始化HOG描述符
    hog.compute(im,result);
    return result;
}


Mat Buffdetect::get(Mat input)
{
    vector<float> vec=stander(input);
    if(vec.size()!=900) cout<<"wrong not 900"<<endl;
    Mat output(1,900,CV_32FC1);

    Mat_<float> p=output;
    int jj=0;
    for(vector<float>::iterator iter=vec.begin();iter!=vec.end();iter++,jj++)
    {
        p(0,jj)=*(iter);
    }
    return output;
}

bool Buffdetect::CircleInfo2(std::vector<cv::Point2f>& pts, cv::Point2f& center, float& radius)
{
    center = cv::Point2d(0, 0);
    radius = 0.0;
    if (pts.size() < 3) return false;;

    double sumX = 0.0;
    double sumY = 0.0;
    double sumX2 = 0.0;
    double sumY2 = 0.0;
    double sumX3 = 0.0;
    double sumY3 = 0.0;
    double sumXY = 0.0;
    double sumX1Y2 = 0.0;
    double sumX2Y1 = 0.0;
    const double N = (double)pts.size();
    for (int i = 0; i < pts.size(); ++i)
    {
        double x = pts.at(i).x;
        double y = pts.at(i).y;
        double x2 = x * x;
        double y2 = y * y;
        double x3 = x2 *x;
        double y3 = y2 *y;
        double xy = x * y;
        double x1y2 = x * y2;
        double x2y1 = x2 * y;

        sumX += x;
        sumY += y;
        sumX2 += x2;
        sumY2 += y2;
        sumX3 += x3;
        sumY3 += y3;
        sumXY += xy;
        sumX1Y2 += x1y2;
        sumX2Y1 += x2y1;
    }
    double C = N * sumX2 - sumX * sumX;
    double D = N * sumXY - sumX * sumY;
    double E = N * sumX3 + N * sumX1Y2 - (sumX2 + sumY2) * sumX;
    double G = N * sumY2 - sumY * sumY;
    double H = N * sumX2Y1 + N * sumY3 - (sumX2 + sumY2) * sumY;

    double denominator = C * G - D * D;
    if (std::abs(denominator) < DBL_EPSILON) return false;
    double a = (H * D - E * G) / (denominator);
    denominator = D * D - G * C;
    if (std::abs(denominator) < DBL_EPSILON) return false;
    double b = (H * C - E * D) / (denominator);
    double c = -(a * sumX + b * sumY + sumX2 + sumY2) / N;

    center.x = a / (-2);
    center.y = b / (-2);
    radius = std::sqrt(a * a + b * b - 4 * c) / 2;
    return true;
}

double Buffdetect::TemplateMatch(cv::Mat image, cv::Mat tepl, cv::Point &point, int method)
{
    int result_cols =  image.cols - tepl.cols + 1;
    int result_rows = image.rows - tepl.rows + 1;
    //    cout <<result_cols<<" "<<result_rows<<endl;
    cv::Mat result = cv::Mat( result_cols, result_rows, CV_32FC1 );
    cv::matchTemplate( image, tepl, result, method );

    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

    switch(method)
    {
    case TM_SQDIFF:
    case TM_SQDIFF_NORMED:
        point = minLoc;
        return minVal;

    default:
        point = maxLoc;
        return maxVal;

    }
}

//void Buffdetect::Set_circle_center()
//{
//    calAngle (CAMERA_MATRIX, DISTORTION_COEFF,  cc);
//    center_yaw += gyr_yaw;
//    center_pitch += gyr_pitch;
//}

//void Buffdetect::Get_circle_center()
//{
//    center_yaw -= gyr_yaw;
//    center_pitch -= gyr_pitch;
//}

void Buffdetect::get_temp_buff(Mat srcImage)
{
    //    Mat drawcircle=Mat(srcImage.rows,srcImage.cols, CV_8UC3, Scalar(0, 0, 0));
    FindBuff = false;

    auto t1 = chrono::high_resolution_clock::now();
    vector<Mat> imgChannels;
    split(srcImage,imgChannels);
    Mat midImage2;
    if (Buffcolor == 0)
    {
        midImage2 = imgChannels.at(0) - imgChannels.at(2);
    }
    else if (Buffcolor == 2)
    {
        midImage2 = imgChannels.at(2) - imgChannels.at(0);
    }
    threshold(midImage2,midImage2,100,255,THRESH_BINARY);
    int structElementSize=3;
    Mat element=getStructuringElement(MORPH_RECT,Size(2*structElementSize+1,2*structElementSize+1),Point(structElementSize,structElementSize));
    //膨胀
    dilate(midImage2,midImage2,element);
    //开运算，消除扇叶上可能存在的小洞
    structElementSize=3;
    element=getStructuringElement(MORPH_RECT,Size(2*structElementSize+1,2*structElementSize+1),Point(structElementSize,structElementSize));
    morphologyEx(midImage2,midImage2, MORPH_CLOSE, element);
    vector<vector<Point>> contours2;
    vector<Vec4i> hierarchy2;
    findContours(midImage2,contours2,hierarchy2,RETR_TREE,CHAIN_APPROX_SIMPLE);

    RotatedRect rect_tmp2;
    bool findTarget=0;

    //遍历轮廓
    if(hierarchy2.size())
        for(int i=0;i>=0;i=hierarchy2[i][0])
        {
            rect_tmp2=minAreaRect(contours2[i]);
            Point2f P[4];
            rect_tmp2.points(P);

            Point2f srcRect[4];
            Point2f dstRect[4];

            double width;
            double height;

            //矫正提取的叶片的宽高
            width=getDistance(P[0],P[1]);
            height=getDistance(P[1],P[2]);
            if(width>height)
            {
                srcRect[0]=P[0];
                srcRect[1]=P[1];
                srcRect[2]=P[2];
                srcRect[3]=P[3];
            }
            else
            {
                swap(width,height);
                srcRect[0]=P[1];
                srcRect[1]=P[2];
                srcRect[2]=P[3];
                srcRect[3]=P[0];
            }
#ifdef SHOW_ALL_CONTOUR
            Scalar color(rand() & 255, rand() & 255, rand() & 255);
            drawContours(srcImage, contours2, i, color, 4, 8, hierarchy2);
#endif
            //通过面积筛选
            double area=height*width;
            if(area>2000){
#ifdef DEBUG_LOG
                cout <<hierarchy2[i]<<endl;

#endif
                dstRect[0]=Point2f(0,0);
                dstRect[1]=Point2f(width,0);
                dstRect[2]=Point2f(width,height);
                dstRect[3]=Point2f(0,height);
                // 应用透视变换，矫正成规则矩形
                Mat transform = getPerspectiveTransform(srcRect,dstRect);
                Mat perspectMat;
                warpPerspective(midImage2,perspectMat,transform,midImage2.size());
#ifdef DEBUG
                imshow("warpdst",perspectMat);
#endif
                // 提取扇叶图片
                Mat testim;
                testim = perspectMat(Rect(0,0,width,height));
#ifdef LEAF_IMG
                //用于保存扇叶图片，以便接下来训练svm
                string s="leaf"+to_string(cnnt)+".jpg";
                cnnt++;
                imwrite("./img/"+s,testim);
#endif

#ifdef DEBUG
                imshow("testim",testim);
#endif
                if(testim.empty())
                {
                    cout<<"filed open"<<endl;
                    return;
                }
#ifdef USE_TEMPLATE
                cv::Point matchLoc;
                double value;
                Mat tmp1;
                resize(testim,tmp1,Size(42,20));
#endif
#if (defined DEBUG)&&(defined USE_TEMPLATE)
                imshow("temp1",tmp1);
#endif
#ifdef USE_TEMPLATE
                vector<double> Vvalue1;
                vector<double> Vvalue2;
                for(int j=1;j<=6;j++)
                {
                    value = TemplateMatch(tmp1, templ[j], matchLoc, TM_CCOEFF_NORMED);
                    Vvalue1.push_back(value);
                }
                for(int j=7;j<=8;j++)
                {
                    value = TemplateMatch(tmp1, templ[j], matchLoc, TM_CCOEFF_NORMED);
                    Vvalue2.push_back(value);
                }
                int maxv1=0,maxv2=0;

                for(int t1=0;t1<6;t1++)
                {
                    if(Vvalue1[t1]>Vvalue1[maxv1])
                    {
                        maxv1=t1;
                    }
                }
                for(int t2=0;t2<2;t2++)
                {
                    if(Vvalue2[t2]>Vvalue2[maxv2])
                    {
                        maxv2=t2;
                    }
                }
#endif
#if (defined DEBUG_LOG)&&(defined USE_TEMPLATE)
                cout<<Vvalue1[maxv1]<<endl;
                cout<<Vvalue2[maxv2]<<endl;
#endif
#ifdef USE_SVM
                //转化为svm所要求的格式
                Mat test=get(testim);
#endif
                //预测是否是要打击的扇叶
#ifdef USE_TEMPLATE
                if(Vvalue1[maxv1]>Vvalue2[maxv2]&&Vvalue1[maxv1]>0.6)
#endif
#ifdef USE_SVM
                    if(buffSVM->predict(test)>=0.9)
#endif
                    {
                        findTarget=true;
                        //查找装甲板
                        if(hierarchy2[i][2]>=0)
                        {
                            RotatedRect rect_tmp=minAreaRect(contours2[hierarchy2[i][2]]);
                            Point2f Pnt[4];
                            rect_tmp.points(Pnt);
                            const float maxHWRatio=0.7153846;
                            const float maxArea=2000;
                            const float minArea=500;

                            float width=rect_tmp.size.width;
                            float height=rect_tmp.size.height;
                            if(height>width)
                                swap(height,width);
                            float area=width*height;

                            if(height/width>maxHWRatio||area>maxArea ||area<minArea){
#ifdef DEBUG
                                cout<<"hw "<<height/width<<"area "<<area<<endl;
                                for(int j=0;j<4;++j)
                                {
                                    line(srcImage,Pnt[j],Pnt[(j+1)%4],Scalar(255,0,255),4);
                                }
                                for(int j=0;j<4;++j)
                                {
                                    line(srcImage,P[j],P[(j+1)%4],Scalar(255,255,0),4);
                                }
                                imshow("debug",srcImage);
                                waitKey(0);
#endif
                                continue;
                            }
                            Point centerP=rect_tmp.center;
                            //打击点
                            circle(srcImage,centerP,1,Scalar(0,255,0),2);
#ifdef SHOW_CIRCLE
                            srcImage.copyTo(drawcircle);
                            circle(drawcircle,centerP,1,Scalar(0,0,255),1);
                            //用于拟合圆，用30个点拟合圆
                            if(cirV.size()<30)
                            {
                                cirV.push_back(centerP);
                                return;
                            }
                            else
                            {
                                float R;
                                //得到拟合的圆心
                                CircleInfo2(cirV,cc,R);
//                                Set_circle_center(cc);
                                circle(drawcircle,cc,1,Scalar(255,0,0),2);
#endif
#if (defined DEBUG_LOG)&& (defined SHOW_CIRCLE)
                                cout<<endl<<"center "<<cc.x<<" , "<<cc.y<<endl;
#endif
#ifdef SHOW_CIRCLE
                                cirV.erase(cirV.begin());

                            }
                            if(cc.x!=0&&cc.y!=0){
                                Mat rot_mat=getRotationMatrix2D(cc,30,1);
#endif
#if (defined DEBUG_LOG)&&(defined SHOW_CIRCLE)
                                cout<<endl<<"center1 "<<cc.x<<" , "<<cc.y<<endl;
#endif
#ifdef SHOW_CIRCLE
                                float sinA=rot_mat.at<double>(0,1);//sin(60);
                                float cosA=rot_mat.at<double>(0,0);//cos(60);
                                float xx=-(cc.x-centerP.x);
                                float yy=-(cc.y-centerP.y);
                                Point2f resPoint=Point2f(cc.x+cosA*xx-sinA*yy,cc.y+sinA*xx+cosA*yy);
                                circle(srcImage,resPoint,1,Scalar(0,255,0),10);
                            }
#endif
                            for(int j=0;j<4;++j)
                            {
                                line(srcImage,Pnt[j],Pnt[(j+1)%4],Scalar(0,255,255),2);
                            }
                        }
                    }
            }

#ifdef DEBUG_LOG
            cout<<"width2 " << width << " height2 "<<height<<" Hwratio2 "<<height/width<<" area2 "<<area<<endl;
#endif

        }

#if (defined SHOW_CIRCLE)&&(defined SHOW_RESULT)
    imshow("circle",drawcircle);
#endif
#ifdef SHOW_RESULT
    imshow("Result",srcImage);
    waitKey(1);
#endif
    //函数所花的时间
    auto t2 = chrono::high_resolution_clock::now();
    cout << "Total period: " << (static_cast<chrono::duration<double, std::milli>>(t2 - t1)).count() << " ms" << endl;
    //        t1 = chrono::high_resolution_clock::now();


}


