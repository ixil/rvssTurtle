/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  Cristian Rodriguez <u5419700@anu.edu.au>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
// ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include "rvss_workshop/cylMsg.h"
#include "rvss_workshop/cylDataArray.h"
#include "rvss_workshop/objMsg.h"
#include "rvss_workshop/objDataArray.h"


//OpenCV2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>

#include "tape.hpp"
#include "cylinder.hpp"
#include "object.hpp"

static const std::string RGB_WINDOW   = "RGB window";
static const std::string DEPTH_WINDOW = "DEPTH_window";


using namespace sensor_msgs;
using namespace message_filters;

float median(std::vector<float> depths)
{
    if(depths.size() > 0) {
        float median;
        size_t size = depths.size();

        sort(depths.begin(), depths.end());

        if (size  % 2 == 0)
            median = (depths[size / 2 - 1] + depths[size / 2]) / 2;
        else 
            median = depths[size / 2];

        return median;
    }
}

class CylinderDetector
{
private:
    boost::mutex m;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber depth_sub_;
    std::map< std::pair<int,int>,std::pair<float,float> > covariances;

    ros::Publisher cyl_pub_;
    ros::Publisher obj_pub_;
    rvss_workshop::cylDataArray data;
    rvss_workshop::objDataArray objData;
    sensor_msgs::ImageConstPtr depthImage;
    sensor_msgs::ImageConstPtr rgbImage;    
    
    std::vector<cv::Rect> regions;
    int red1_high_H;
    int red1_high_S;
    int red1_high_V;
    int red1_low_H;
    int red1_low_S;
    int red1_low_V;
    
    int red2_high_H;
    int red2_high_S;
    int red2_high_V;
    int red2_low_H;
    int red2_low_S;
    int red2_low_V;    
    
    int blue_high_H;
    int blue_high_S;
    int blue_high_V;
    int blue_low_H;
    int blue_low_S;
    int blue_low_V;
    
    int green_high_H;
    int green_high_S;
    int green_high_V;
    int green_low_H;
    int green_low_S;
    int green_low_V;
    
    int yellow_high_H;
    int yellow_high_S;
    int yellow_high_V;
    int yellow_low_H;
    int yellow_low_S;
    int yellow_low_V;

    int object1_high_H;
    int object1_high_S;
    int object1_high_V;
    int object1_low_H;
    int object1_low_S;
    int object1_low_V;

    int object2_high_H;
    int object2_high_S;
    int object2_high_V;
    int object2_low_H;
    int object2_low_S;
    int object2_low_V;

    int object3_high_H;
    int object3_high_S;
    int object3_high_V;
    int object3_low_H;
    int object3_low_S;
    int object3_low_V;

    
public:
    CylinderDetector(cv::FileStorage colorCfg_): it_(nh_)
    {
        red1_high_H = colorCfg_["Red1.High.H"];
        red1_high_S = colorCfg_["Red1.High.S"];
        red1_high_V = colorCfg_["Red1.High.V"];        
        
        red1_low_H = colorCfg_["Red1.Low.H"];
        red1_low_S = colorCfg_["Red1.Low.S"];
        red1_low_V = colorCfg_["Red1.Low.V"];
        
        red2_high_H = colorCfg_["Red2.High.H"];
        red2_high_S = colorCfg_["Red2.High.S"];
        red2_high_V = colorCfg_["Red2.High.V"];        
        
        red2_low_H = colorCfg_["Red2.Low.H"];
        red2_low_S = colorCfg_["Red2.Low.S"];
        red2_low_V = colorCfg_["Red2.Low.V"];
        
        blue_high_H = colorCfg_["Blue.High.H"];
        blue_high_S = colorCfg_["Blue.High.S"];
        blue_high_V = colorCfg_["Blue.High.V"];        
        
        blue_low_H = colorCfg_["Blue.Low.H"];
        blue_low_S = colorCfg_["Blue.Low.S"];
        blue_low_V = colorCfg_["Blue.Low.V"];
        
        green_high_H = colorCfg_["Green.High.H"];
        green_high_S = colorCfg_["Green.High.S"];
        green_high_V = colorCfg_["Green.High.V"];        
        
        green_low_H = colorCfg_["Green.Low.H"];
        green_low_S = colorCfg_["Green.Low.S"];
        green_low_V = colorCfg_["Green.Low.V"];
        
        yellow_high_H = colorCfg_["Yellow.High.H"];
        yellow_high_S = colorCfg_["Yellow.High.S"];
        yellow_high_V = colorCfg_["Yellow.High.V"];        
        
        yellow_low_H = colorCfg_["Yellow.Low.H"];
        yellow_low_S = colorCfg_["Yellow.Low.S"];
        yellow_low_V = colorCfg_["Yellow.Low.V"];
        
        object1_high_H = colorCfg_["Object1.High.H"];
        object1_high_S = colorCfg_["Object1.High.S"];
        object1_high_V = colorCfg_["Object1.High.V"];
        object1_low_H = colorCfg_["Object1.Low.H"];
        object1_low_S = colorCfg_["Object1.Low.S"];
        object1_low_V = colorCfg_["Object1.Low.V"];

        object2_high_H = colorCfg_["Object2.High.H"];
        object2_high_S = colorCfg_["Object2.High.S"];
        object2_high_V = colorCfg_["Object2.High.V"];
        object2_low_H = colorCfg_["Object2.Low.H"];
        object2_low_S = colorCfg_["Object2.Low.S"];
        object2_low_V = colorCfg_["Object2.Low.V"];

        object3_high_H = colorCfg_["Object3.High.H"];
        object3_high_S = colorCfg_["Object3.High.S"];
        object3_high_V = colorCfg_["Object3.High.V"];
        object3_low_H = colorCfg_["Object3.Low.H"];
        object3_low_S = colorCfg_["Object3.Low.S"];
        object3_low_V = colorCfg_["Object3.Low.V"];


        cv::namedWindow(RGB_WINDOW);
        cv::namedWindow(DEPTH_WINDOW);
        
        std::pair<int,int> aux;
        std::pair<float,float> covAux;
        
        aux.first  = -1;
        aux.second = -1;
        covAux.first  = 0.04;
        covAux.second = 0.04;
        covariances[aux] = covAux;
        
        aux.first  = 0;
        aux.second = 0;
        covAux.first  = 0.0064;
        covAux.second = 0.01;
        covariances[aux] = covAux;
        
        aux.first  = 0;
        aux.second = 1;
        covAux.first  = 0.003025;
        covAux.second = 0.0064;
        covariances[aux] = covAux;
        
        aux.first  = 0;
        aux.second = 2;
        covAux.first  = 0.001024;
        covAux.second = 0.0036;
        covariances[aux] = covAux;
        
        aux.first  = 0;
        aux.second = 3;
        covAux.first  = 0.000625;
        covAux.second = 0.0009;
        covariances[aux] = covAux;
        
        aux.first  = 0;
        aux.second = 4;
        covAux.first  = 0.000121;
        covAux.second = 0.0001;
        covariances[aux] = covAux;
        
        aux.first  = 0;
        aux.second = 5;
        covAux.first  = 0.000625;
        covAux.second = 0.0009;
        covariances[aux] = covAux;
        
        aux.first  = 0;
        aux.second = 6;
        covAux.first  = 0.0009;
        covAux.second = 0.0036;
        covariances[aux] = covAux;
        
        aux.first  = 0;
        aux.second = 7;
        covAux.first  = 0.0025;
        covAux.second = 0.0064;
        covariances[aux] = covAux;
        
        aux.first  = 0;
        aux.second = 8;
        covAux.first  = 0.0064;
        covAux.second = 0.01;
        covariances[aux] = covAux;
        
        aux.first  = 1;
        aux.second = 0;
        covAux.first  = 0.0064;
        covAux.second = 0.01;
        covariances[aux] = covAux;
        
        aux.first  = 1;
        aux.second = 1;
        covAux.first  = 0.003025;
        covAux.second = 0.0064;
        covariances[aux] = covAux;
        
        aux.first  = 1;
        aux.second = 2;
        covAux.first  = 0.001024;
        covAux.second = 0.0036;
        covariances[aux] = covAux;
        
        aux.first  = 1;
        aux.second = 3;
        covAux.first  = 0.000625;
        covAux.second = 0.0009;
        covariances[aux] = covAux;
        
        aux.first  = 1;
        aux.second = 4;
        covAux.first  = 0.000121;
        covAux.second = 0.0001;
        covariances[aux] = covAux;
        
        aux.first  = 1;
        aux.second = 5;
        covAux.first  = 0.000625;
        covAux.second = 0.0009;
        covariances[aux] = covAux;
        
        aux.first  = 1;
        aux.second = 6;
        covAux.first  = 0.0009;
        covAux.second = 0.0036;
        covariances[aux] = covAux;
        
        aux.first  = 1;
        aux.second = 7;
        covAux.first  = 0.0025;
        covAux.second = 0.0064;
        covariances[aux] = covAux;
        
        aux.first  = 1;
        aux.second = 8;
        covAux.first  = 0.0064;
        covAux.second = 0.01;
        covariances[aux] = covAux;
        
        aux.first  = 2;
        aux.second = 0;
        covAux.first  = 0.0064;
        covAux.second = 0.01;
        covariances[aux] = covAux;
        
        aux.first  = 2;
        aux.second = 1;
        covAux.first  = 0.003025;
        covAux.second = 0.0064;
        covariances[aux] = covAux;
        
        aux.first  = 2;
        aux.second = 2;
        covAux.first  = 0.001024;
        covAux.second = 0.0036;
        covariances[aux] = covAux;
        
        aux.first  = 2;
        aux.second = 3;
        covAux.first  = 0.000625;
        covAux.second = 0.0009;
        covariances[aux] = covAux;
        
        aux.first  = 2;
        aux.second = 4;
        covAux.first  = 0.000121;
        covAux.second = 0.0001;
        covariances[aux] = covAux;
        
        aux.first  = 2;
        aux.second = 5;
        covAux.first  = 0.000625;
        covAux.second = 0.0009;
        covariances[aux] = covAux;
        
        aux.first  = 2;
        aux.second = 6;
        covAux.first  = 0.0009;
        covAux.second = 0.0036;
        covariances[aux] = covAux;
        
        aux.first  = 2;
        aux.second = 7;
        covAux.first  = 0.0025;
        covAux.second = 0.0064;
        covariances[aux] = covAux;
        
        aux.first  = 2;
        aux.second = 8;
        covAux.first  = 0.0064;
        covAux.second = 0.01;
        covariances[aux] = covAux;
        
        
        aux.first  = 3;
        aux.second = 0;
        covAux.first  = 0.0144;
        covAux.second = 0.0225;
        covariances[aux] = covAux;
        
        aux.first  = 3;
        aux.second = 1;
        covAux.first  = 0.003025;
        covAux.second = 0.01;
        covariances[aux] = covAux;
        
        aux.first  = 3;
        aux.second = 2;
        covAux.first  = 0.001024;
        covAux.second = 0.0064;
        covariances[aux] = covAux;
        
        aux.first  = 3;
        aux.second = 3;
        covAux.first  = 0.000625;
        covAux.second = 0.0016;
        covariances[aux] = covAux;
        
        aux.first  = 3;
        aux.second = 4;
        covAux.first  = 0.000121;
        covAux.second = 0.0001;
        covariances[aux] = covAux;
        
        aux.first  = 3;
        aux.second = 5;
        covAux.first  = 0.000625;
        covAux.second = 0.0016;
        covariances[aux] = covAux;
        
        aux.first  = 3;
        aux.second = 6;
        covAux.first  = 0.0009;
        covAux.second = 0.0064;
        covariances[aux] = covAux;
        
        aux.first  = 3;
        aux.second = 7;
        covAux.first  = 0.0025;
        covAux.second = 0.01;
        covariances[aux] = covAux;
        
        aux.first  = 3;
        aux.second = 8;
        covAux.first  = 0.0064;
        covAux.second = 0.0225;
        covariances[aux] = covAux;
        
        
        aux.first  = 4;
        aux.second = 0;
        covAux.first  = 0.0225;
        covAux.second = 0.0256;
        covariances[aux] = covAux;
        
        aux.first  = 4;
        aux.second = 1;
        covAux.first  = 0.003025;
        covAux.second = 0.0121;
        covariances[aux] = covAux;
        
        aux.first  = 4;
        aux.second = 2;
        covAux.first  = 0.001024;
        covAux.second = 0.0064;
        covariances[aux] = covAux;
        
        aux.first  = 4;
        aux.second = 3;
        covAux.first  = 0.000625;
        covAux.second = 0.0016;
        covariances[aux] = covAux;
        
        aux.first  = 4;
        aux.second = 4;
        covAux.first  = 0.000121;
        covAux.second = 0.0001;
        covariances[aux] = covAux;
        
        aux.first  = 4;
        aux.second = 5;
        covAux.first  = 0.000625;
        covAux.second = 0.0016;
        covariances[aux] = covAux;
        
        aux.first  = 4;
        aux.second = 6;
        covAux.first  = 0.0009;
        covAux.second = 0.0064;
        covariances[aux] = covAux;
        
        aux.first  = 4;
        aux.second = 7;
        covAux.first  = 0.0025;
        covAux.second = 0.0121;
        covariances[aux] = covAux;
        
        aux.first  = 4;
        aux.second = 8;
        covAux.first  = 0.0064;
        covAux.second = 0.0256;
        covariances[aux] = covAux;
        
        
        
        aux.first  = 5;
        aux.second = 0;
        covAux.first  = 0.0324;
        covAux.second = 0.0324;
        covariances[aux] = covAux;
        
        aux.first  = 5;
        aux.second = 1;
        covAux.first  = 0.003025;
        covAux.second = 0.0121;
        covariances[aux] = covAux;
        
        aux.first  = 5;
        aux.second = 2;
        covAux.first  = 0.001024;
        covAux.second = 0.0064;
        covariances[aux] = covAux;
        
        aux.first  = 5;
        aux.second = 3;
        covAux.first  = 0.000625;
        covAux.second = 0.0016;
        covariances[aux] = covAux;
        
        aux.first  = 5;
        aux.second = 4;
        covAux.first  = 0.000121;
        covAux.second = 0.0001;
        covariances[aux] = covAux;
        
        aux.first  = 5;
        aux.second = 5;
        covAux.first  = 0.000625;
        covAux.second = 0.0016;
        covariances[aux] = covAux;
        
        aux.first  = 5;
        aux.second = 6;
        covAux.first  = 0.0009;
        covAux.second = 0.0064;
        covariances[aux] = covAux;
        
        aux.first  = 5;
        aux.second = 7;
        covAux.first  = 0.0025;
        covAux.second = 0.0121;
        covariances[aux] = covAux;
        
        aux.first  = 5;
        aux.second = 8;
        covAux.first  = 0.0324;
        covAux.second = 0.0324;
        covariances[aux] = covAux;
        
        message_filters::Subscriber<Image> image_sub_(nh_,"/camera/rgb/image_color",5);
        message_filters::Subscriber<Image> depth_sub_(nh_,"/camera/depth_registered/image_raw",5);
        typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
        // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
        Synchronizer<MySyncPolicy>* sync = new Synchronizer<MySyncPolicy>(MySyncPolicy(10), image_sub_, depth_sub_);
        sync->registerCallback(boost::bind(&CylinderDetector::callback, this, _1, _2));

        cyl_pub_ = nh_.advertise<rvss_workshop::cylDataArray>("cylinderTopic",1);
        obj_pub_ = nh_.advertise<rvss_workshop::objDataArray>("objectTopic",1);
        ros::Rate spin_rate(30);

        while( ros::ok() ) {
            ros::spinOnce();
            spin_rate.sleep();
        }
    }

    ~CylinderDetector()
    {
        cv::destroyWindow(RGB_WINDOW);
        cv::destroyWindow(DEPTH_WINDOW);
    }

std::pair<float,float> getCov(float Zrobot, float Xrobot)
    {
        int covZ = -1;
        int covX = -1;
        std::cout << "comparison" << Zrobot << " " << Xrobot << std::endl; 
        if ( Zrobot < 0.80 )
            covZ = 0;
        else if( Zrobot >= 0.80 && Zrobot < 1.00)
            covZ = 1;
        else if( Zrobot >= 1.00 && Zrobot < 1.50)
            covZ = 2;
        else if( Zrobot >= 1.50 && Zrobot < 2.00)
            covZ = 3;        
        else if( Zrobot >= 2.00 && Zrobot < 2.50)
            covZ = 4;
        else if( Zrobot >= 2.50)
            covZ = 5;
        
        
        if ( Xrobot < -0.80 )
            covX = 0;
        else if( Xrobot >= -0.80 && Xrobot < -0.50)
            covX = 1;
        else if( Xrobot >= -0.50 && Xrobot < -0.10)
            covX = 2;
        else if( Xrobot >= -0.20 && Xrobot < -0.05)
            covX = 3;        
        else if( Xrobot >= -0.05 && Xrobot < 0.05)
            covX = 4;
        else if( Xrobot >= 0.05 && Xrobot < 0.20)
            covX = 5;
        else if( Xrobot >= 0.20 && Xrobot < 0.50)
            covX = 6;
        else if( Xrobot >= 0.50 && Xrobot < 0.80)
            covX = 7;
        else if( Xrobot >= 0.80)
            covX = 8;
        std::pair<int,int> aux;
        aux.first  = covZ;
        aux.second = covX;
        std::cout << "covZX" << covZ << " " << covX << std::endl;
        return covariances.find(aux)->second;
    }
    void callback(const sensor_msgs::ImageConstPtr& rgbImage, const sensor_msgs::ImageConstPtr& depthImage)
    {
        cv_bridge::CvImagePtr rgbPtr,depthPtr;
        std::vector<Tape> tapes;
	std::vector<Object> objects;
        std::vector<Cylinder> cylinders;
        try
        {
            rgbPtr   = cv_bridge::toCvCopy(rgbImage, sensor_msgs::image_encodings::TYPE_8UC3);
            depthPtr = cv_bridge::toCvCopy(depthImage, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            exit;
        }
        cv::Rect cropRoI(0,rgbPtr->image.rows/2,rgbPtr->image.cols,rgbPtr->image.rows/2);
        rgbPtr->image = rgbPtr->image(cropRoI);
        depthPtr->image = depthPtr->image(cropRoI);
        cv::imshow(RGB_WINDOW, rgbPtr->image);
        cv::imshow(DEPTH_WINDOW, depthPtr->image);
        cv::Mat imgHSV;
        cv::waitKey(3);
        cvtColor(rgbPtr->image, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        std::vector<cv::Rect> redAreas = getAreasbyColor(imgHSV,RED);
        std::vector<cv::Rect> blueAreas = getAreasbyColor(imgHSV,BLUE);
        std::vector<cv::Rect> yellowAreas = getAreasbyColor(imgHSV,YELLOW);
        std::vector<cv::Rect> greenAreas = getAreasbyColor(imgHSV,GREEN);
        //objects
        std::vector<cv::Rect> object1Areas = getAreasbyColor(imgHSV,OBJECT1);
        std::vector<cv::Rect> object2Areas = getAreasbyColor(imgHSV,OBJECT2);
	std::vector<cv::Rect> object3Areas = getAreasbyColor(imgHSV,OBJECT3);

        for( uint i=0; i< redAreas.size(); i++)
        {
            double centerX = redAreas[i].x + (double)(redAreas[i].width)/2;
            double centerY = redAreas[i].y + (double)(redAreas[i].height)/2;
            cv::Mat depthI = depthPtr->image(redAreas[i]); 
            int x1 = round(depthI.cols/2 * 0.3);
            int x2 = round(depthI.cols/2 * 0.7);
            int y1 = round(depthI.rows/2 * 0.3);
            int y2 = round(depthI.rows/2 * 0.7);
            double depth = 0.0;
            if(x1>0 && x2>0 && y2 > 0 && y1 > 0)
            {
                cv::Rect RoI(x1,y1, x2 - x1, y2 - y1);
                cv::Mat depthIRoI = depthI(RoI);
                std::vector<float> depths;
                for( int r = 0; r < depthIRoI.rows; r++)
                {
                    for( int c = 0; c < depthIRoI.cols; c++)
                    {
                        if(depthIRoI.at<float>(r,c) > 0.0)
                            depths.push_back(depthIRoI.at<float>(r,c));         
                    }
                }
    //             double depth = 0.0;
                    depth = median(depths);
                }
//             std::cout << "Median " << depth << std::endl;
            Tape aux = {centerX,centerY,depth,RED};
            tapes.push_back(aux);
        }
        for( uint i=0; i< greenAreas.size(); i++)
        {
            double centerX = greenAreas[i].x + (double)(greenAreas[i].width)/2;
            double centerY = greenAreas[i].y + (double)(greenAreas[i].height)/2;
            cv::Mat depthI = depthPtr->image(greenAreas[i]); 
            int x1 = round(depthI.cols/2 * 0.3);
            int x2 = round(depthI.cols/2 * 0.7);
            int y1 = round(depthI.rows/2 * 0.3);
            int y2 = round(depthI.rows/2 * 0.7);
            double depth = 0.0;
            if(x1>0 && x2>0 && y2 > 0 && y1 > 0)
            {
                cv::Rect RoI(x1,y1, x2 - x1, y2 - y1);
                cv::Mat depthIRoI = depthI(RoI);
                std::vector<float> depths;
                for( int r = 0; r < depthIRoI.rows; r++)
                {
                    for( int c = 0; c < depthIRoI.cols; c++)
                    {
                        if(depthIRoI.at<float>(r,c) > 0.0)
                            depths.push_back(depthIRoI.at<float>(r,c));         
                    }
                }
    //             double depth = 0.0;
                depth = median(depths);
            }
            Tape aux = {centerX,centerY,depth,GREEN};
            tapes.push_back(aux);
        }
        for( uint i=0; i< blueAreas.size(); i++)
        {
            double centerX = blueAreas[i].x + (double)(blueAreas[i].width)/2;
            double centerY = blueAreas[i].y + (double)(blueAreas[i].height)/2;
            cv::Mat depthI = depthPtr->image(blueAreas[i]); 
            int x1 = round(depthI.cols/2 * 0.3);
            int x2 = round(depthI.cols/2 * 0.7);
            int y1 = round(depthI.rows/2 * 0.3);
            int y2 = round(depthI.rows/2 * 0.7);
            double depth = 0.0;
            if(x1>0 && x2>0 && y2 > 0 && y1 > 0)
            {
                cv::Rect RoI(x1,y1, x2 - x1, y2 - y1);
    //            std::cout << "med depth" << depthI.at<float>(depthI.cols/2,depthI.rows/2) << std::endl;
//                 std::cout << "Green " << depthI(RoI) << std::endl; 
                cv::Mat depthIRoI = depthI(RoI);
                std::vector<float> depths;
                for( int r = 0; r < depthIRoI.rows; r++)
                {
                    for( int c = 0; c < depthIRoI.cols; c++)
                    {
                        if(depthIRoI.at<float>(r,c) > 0.0)
                            depths.push_back(depthIRoI.at<float>(r,c));         
                    }
                }
    //             double depth = 0.0;
                depth = median(depths);
            }
            Tape aux = {centerX,centerY,depth,BLUE};
            tapes.push_back(aux);
        }
        for( uint i=0; i< yellowAreas.size(); i++)
        {
            double centerX = yellowAreas[i].x + (double)(yellowAreas[i].width)/2;
            double centerY = yellowAreas[i].y + (double)(yellowAreas[i].height)/2;
            cv::Mat depthI = depthPtr->image(yellowAreas[i]); 
            int x1 = round(depthI.cols/2 * 0.3);
            int x2 = round(depthI.cols/2 * 0.7);
            int y1 = round(depthI.rows/2 * 0.3);
            int y2 = round(depthI.rows/2 * 0.7);
            double depth = 0.0;
            if(x1>0 && x2>0 && y2 > 0 && y1 > 0)
            {
                cv::Rect RoI(x1,y1, x2 - x1, y2 - y1);
    //            std::cout << "med depth" << depthI.at<float>(depthI.cols/2,depthI.rows/2) << std::endl;
//                 std::cout << "Green " << depthI(RoI) << std::endl; 
                cv::Mat depthIRoI = depthI(RoI);
                std::vector<float> depths;
                for( int r = 0; r < depthIRoI.rows; r++)
                {
                    for( int c = 0; c < depthIRoI.cols; c++)
                    {
                        if(depthIRoI.at<float>(r,c) > 0.0)
                            depths.push_back(depthIRoI.at<float>(r,c));         
                    }
                }
                depth = median(depths);
            }
            Tape aux = {centerX,centerY,depth,YELLOW};
            tapes.push_back(aux);
        }
	// objects
        for( uint i=0; i< object1Areas.size(); i++)
        {
            double centerX = object1Areas[i].x + (double)(object1Areas[i].width)/2;
            double centerY = object1Areas[i].y + (double)(object1Areas[i].height)/2;
            cv::Mat depthI = depthPtr->image(object1Areas[i]); 
            int x1 = round(depthI.cols/2 * 0.3);
            int x2 = round(depthI.cols/2 * 0.7);
            int y1 = round(depthI.rows/2 * 0.3);
            int y2 = round(depthI.rows/2 * 0.7);
            double depth = 0.0;
            if(x1>0 && x2>0 && y2 > 0 && y1 > 0)
            {
                cv::Rect RoI(x1,y1, x2 - x1, y2 - y1);
    //            std::cout << "med depth" << depthI.at<float>(depthI.cols/2,depthI.rows/2) << std::endl;
//                 std::cout << "Green " << depthI(RoI) << std::endl; 
                cv::Mat depthIRoI = depthI(RoI);
                std::vector<float> depths;
                for( int r = 0; r < depthIRoI.rows; r++)
                {
                    for( int c = 0; c < depthIRoI.cols; c++)
                    {
                        if(depthIRoI.at<float>(r,c) > 0.0)
                            depths.push_back(depthIRoI.at<float>(r,c));         
                    }
                }
                depth = median(depths);
            }
            Object aux(centerX,centerY,depth, OBJECT1);
            objects.push_back(aux);
        }
        for( uint i=0; i< object2Areas.size(); i++)
        {
            double centerX = object2Areas[i].x + (double)(object2Areas[i].width)/2;
            double centerY = object2Areas[i].y + (double)(object2Areas[i].height)/2;
            cv::Mat depthI = depthPtr->image(object2Areas[i]); 
            int x1 = round(depthI.cols/2 * 0.3);
            int x2 = round(depthI.cols/2 * 0.7);
            int y1 = round(depthI.rows/2 * 0.3);
            int y2 = round(depthI.rows/2 * 0.7);
            double depth = 0.0;
            if(x1>0 && x2>0 && y2 > 0 && y1 > 0)
            {
                cv::Rect RoI(x1,y1, x2 - x1, y2 - y1);
    //            std::cout << "med depth" << depthI.at<float>(depthI.cols/2,depthI.rows/2) << std::endl;
//                 std::cout << "Green " << depthI(RoI) << std::endl; 
                cv::Mat depthIRoI = depthI(RoI);
                std::vector<float> depths;
                for( int r = 0; r < depthIRoI.rows; r++)
                {
                    for( int c = 0; c < depthIRoI.cols; c++)
                    {
                        if(depthIRoI.at<float>(r,c) > 0.0)
                            depths.push_back(depthIRoI.at<float>(r,c));         
                    }
                }
                depth = median(depths);
            }
            Object aux(centerX,centerY,depth,OBJECT2);
            objects.push_back(aux);
        }
        for( uint i=0; i< object3Areas.size(); i++)
        {
            double centerX = object3Areas[i].x + (double)(object3Areas[i].width)/2;
            double centerY = object3Areas[i].y + (double)(object3Areas[i].height)/2;
            cv::Mat depthI = depthPtr->image(object3Areas[i]); 
            int x1 = round(depthI.cols/2 * 0.3);
            int x2 = round(depthI.cols/2 * 0.7);
            int y1 = round(depthI.rows/2 * 0.3);
            int y2 = round(depthI.rows/2 * 0.7);
            double depth = 0.0;
            if(x1>0 && x2>0 && y2 > 0 && y1 > 0)
            {
                cv::Rect RoI(x1,y1, x2 - x1, y2 - y1);
    //            std::cout << "med depth" << depthI.at<float>(depthI.cols/2,depthI.rows/2) << std::endl;
//                 std::cout << "Green " << depthI(RoI) << std::endl; 
                cv::Mat depthIRoI = depthI(RoI);
                std::vector<float> depths;
                for( int r = 0; r < depthIRoI.rows; r++)
                {
                    for( int c = 0; c < depthIRoI.cols; c++)
                    {
                        if(depthIRoI.at<float>(r,c) > 0.0)
                            depths.push_back(depthIRoI.at<float>(r,c));         
                    }
                }
                depth = median(depths);
            }
            Object aux(centerX, centerY, depth, OBJECT3);
            objects.push_back(aux);
        }


        for(std::vector<Tape>::iterator ii=tapes.begin(); ii != tapes.end(); ii++)
        {
            Tape tap =  (*ii);
            std::cout << tap << std::endl;
            /**
            * Join tapes that belong to the same cylinder.
            */
            if(cylinders.empty())
            {
                Cylinder Aux;
                Aux.addTape(tap);
                cylinders.push_back(Aux);
            }
            else
            {
                bool isNewCylinder = true;
                for(std::vector<Cylinder>::iterator jj=cylinders.begin(); jj != cylinders.end() ;jj++)
                {
                    Cylinder cyl = (*jj);
                    if(cyl.addTape(tap) == true)
                    {
                        isNewCylinder = false;
                        break;
                    }
                }    
                if(isNewCylinder == true)
                {                
                    Cylinder Aux;
                    Aux.addTape(tap);
                    cylinders.push_back(Aux);
                }
            }
        }
        
        // Publisher!
        rvss_workshop::cylDataArray arrayMsg;
        for (std::vector<Cylinder>::iterator ii = cylinders.begin(); ii != cylinders.end(); ii++)
        {
            if((*ii).getTape().size()==4 && (*ii).getLabel()!= 100 && (*ii).getZw() != 0.0)
            {
                rvss_workshop::cylMsg aux;
                aux.header.stamp = ros::Time::now();
                aux.header.frame_id = "cylinderTopic";
                aux.Zrobot = (*ii).getZw()/1000;
                aux.Xrobot = (*ii).getXw()/1000;                
                aux.label  = (*ii).getLabel();
                std::pair<float,float> cov1 = getCov(aux.Zrobot,aux.Xrobot);
                std::cout << "covariance " << cov1.first << " " << cov1.second <<std::endl;
                const float covAux[] = { cov1.first, 0.0f, 0.0f, cov1.second};
                std::vector<float> data( covAux,covAux + sizeof( covAux ) / sizeof( covAux[0] ) );
                aux.covariance = data;
                arrayMsg.cylinders.push_back(aux);
            }
        }
        arrayMsg.header.stamp = ros::Time::now();
        arrayMsg.header.frame_id = "cylinderTopic";
        cyl_pub_.publish(arrayMsg);

        // Publisher Objects!
        rvss_workshop::objDataArray arrayObjMsg;
        for (std::vector<Object>::iterator ii = objects.begin(); ii != objects.end(); ii++)
        {

	  if((*ii).getZw() != 0.0){
	    rvss_workshop::objMsg aux;
            aux.header.stamp = ros::Time::now();
            aux.header.frame_id = "objectTopic";
            aux.Zrobot = (*ii).getZw()/1000;
            aux.Xrobot = (*ii).getXw()/1000;                
            aux.label  = (*ii).getLabel();
            std::pair<float,float> cov1 = getCov(aux.Zrobot,aux.Xrobot);
            std::cout << "covariance " << cov1.first << " " << cov1.second <<std::endl;
            const float covAux[] = { cov1.first, 0.0f, 0.0f, cov1.second};
            std::vector<float> data( covAux,covAux + sizeof( covAux ) / sizeof( covAux[0] ) );
            aux.covariance = data;
            if(aux.Zrobot < 3.0){
            	arrayObjMsg.objects.push_back(aux);
            }
          }
        }
        arrayObjMsg.header.stamp = ros::Time::now();
        arrayObjMsg.header.frame_id = "objectTopic";
        obj_pub_.publish(arrayObjMsg);
    }

    std::vector<cv::Rect> getAreasbyColor(cv::Mat hsv_image,uint color=0)
    {
        cv::Mat imgThresholded;
        switch(color)
        {
        case GREEN:{
            cv::inRange(hsv_image, cv::Scalar(green_low_H, green_low_S, green_low_V), cv::Scalar(green_high_H, green_high_S, green_high_V), imgThresholded);
            //morphological opening (remove small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );

            //morphological closing (fill small holes in the foreground)
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
//             cv::imshow("GREEN",imgThresholded);
            break;}
        case YELLOW:{
            cv::inRange(hsv_image, cv::Scalar(yellow_low_H, yellow_low_S, yellow_low_V), cv::Scalar(yellow_high_H, yellow_high_S, yellow_high_V), imgThresholded);
            //morphological opening (remove small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );

            //morphological closing (fill small holes in the foreground)
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
//             cv::imshow("YELLOW",imgThresholded);
            break;}
        case BLUE:{
            cv::inRange(hsv_image, cv::Scalar(blue_low_H, blue_low_S, blue_low_V), cv::Scalar(blue_high_H, blue_high_S, blue_high_V), imgThresholded);
            //morphological opening (remove small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );

            //morphological closing (fill small holes in the foreground)
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
//             cv::imshow("BLUE",imgThresholded);
            break;}
        case RED:{
            cv::Mat lower_red_hue_range,upper_red_hue_range;
            cv::inRange(hsv_image, cv::Scalar(red1_low_H, red1_low_S, red1_low_V), cv::Scalar(red1_high_H, red1_high_S, red1_high_V), lower_red_hue_range);
            cv::inRange(hsv_image, cv::Scalar(red2_low_H, red2_low_S, red2_low_V), cv::Scalar(red2_high_H, red2_high_S,  red2_high_V), upper_red_hue_range);
            cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, imgThresholded);
            //morphological opening (remove small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );

            //morphological closing (fill small holes in the foreground)
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
//             cv::imshow("RED",imgThresholded);
            break;}	
	case OBJECT1:{           
            cv::inRange(hsv_image, cv::Scalar(object1_low_H, object1_low_S, object1_low_V), cv::Scalar(object1_high_H, object1_high_S, object1_high_V), imgThresholded);         
            //morphological opening (remove small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );

            //morphological closing (fill small holes in the foreground)
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );	 
            cv::imshow("OBJECT1",imgThresholded);
            break;}
	case OBJECT2:{
	cv::Mat lower_red_hue_range,upper_red_hue_range, object2;
            cv::inRange(hsv_image, cv::Scalar(red1_low_H, red1_low_S, red1_low_V), cv::Scalar(red1_high_H, red1_high_S, red1_high_V), lower_red_hue_range);
            cv::inRange(hsv_image, cv::Scalar(red2_low_H, red2_low_S, red2_low_V), cv::Scalar(red2_high_H, red2_high_S,  red2_high_V), upper_red_hue_range);
            cv::inRange(hsv_image, cv::Scalar(object2_low_H, object2_low_S, object2_low_V), cv::Scalar(object2_high_H, object2_high_S, object2_high_V), object2);
            
            cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, imgThresholded);
            cv::addWeighted(object2, 1.0, imgThresholded, 1.0, 0.0, imgThresholded);
            
            //morphological opening (remove small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );

            //morphological closing (fill small holes in the foreground)
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
             cv::imshow("OBJECT2",imgThresholded);
            break;}
	case OBJECT3:{
            cv::inRange(hsv_image, cv::Scalar(object3_low_H, object3_low_S, object3_low_V), cv::Scalar(object3_high_H, object3_high_S, object3_high_V), imgThresholded);
            //morphological opening (remove small objects from the foreground)
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );

            //morphological closing (fill small holes in the foreground)
            dilate( imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
            erode(imgThresholded, imgThresholded, getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)) );
             cv::imshow("OBJECT3",imgThresholded);
        }}

        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        cv::Mat canny_output;
        cv::Canny(imgThresholded, canny_output, 100, 100*2, 3 );
        cv::findContours( canny_output, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

        /// Approximate contours to polygons + get bounding rects.
        std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
        std::vector<cv::Rect> boundRect( contours.size() ), ret;

        for(uint i=0; i < contours.size(); i++ ) {
            approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 3, true );
            boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
        }

        std::vector<cv::Point2d> pts;
        for(uint i=0; i<boundRect.size(); i++)
        {
            cv::Mat region   = hsv_image(boundRect[i]);
            if(region.cols<=0 && region.rows<=0)
                continue;
            float proportion = (float)(region.cols)/(float)(region.rows);
            float area       = region.cols*region.rows;
            bool isObject = (color == OBJECT1 || color == OBJECT2 || color == OBJECT3) && (proportion <= 1.7 || proportion >= 2.3) && (area > 200 && area < 5000);
            bool isCylinder = (color==RED || color==GREEN || color== BLUE || color==YELLOW)&& (proportion > 1.6 && proportion < 2.4 && area > 200 && area < 5000);    
            if(isCylinder || isObject)
            {
		
                bool flag = 1;
                for(uint j=0; j<pts.size(); j++)
                {
                    int deltaCx = (boundRect[i].width/2 + boundRect[i].x) - pts[j].x;
                    int deltaCy = (boundRect[i].height/2 + boundRect[i].y) - pts[j].y;
                    if( -10 <= deltaCx && deltaCx <= 10)
                    {
                        if(-10 <=  deltaCy && deltaCy <= 10)
                        {
                            flag = 0;
                        }
                    }
                }
                if(flag==1)
                {
                    cv::Point2d auxPt;
                    auxPt.x = boundRect[i].width/2 + boundRect[i].x;
                    auxPt.y = boundRect[i].height/2 + boundRect[i].y;
                    pts.push_back(auxPt);
                    ret.push_back(boundRect[i]);
                }
            }
        }

        return ret;
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_converter");
    std::string strSettingsFile =argv[1];
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << std::endl << "Wrong path to settings. Path must be absolut or relative to package directory." << std::endl;
        return 1;
    }
    CylinderDetector cylinder(fsSettings);

    return 0;
}
