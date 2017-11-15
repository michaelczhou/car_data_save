#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>

#include <limits>
#include <cmath>
#include <string>

using namespace cv;
using namespace std;


//double timeOmniImage0=0;
//double timeBumImageLeft=0;
//double timeVelodyne=0;
double timeZedImageLeft=0;
double timeZedImageRight=0;

//bool newOmniImage0=false;
//bool newBumImageLeft=false;
//bool newVelodyne=false;
bool newZedImageLeft=false;
bool newZedImageRight=false;


//pcl::PointCloud<pcl::PointXYZ>::Ptr current_sensor_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
//Mat omniImage0;
//Mat bumImageLeft;
Mat ZedImageLeft;
Mat ZedImageRight;


//string imageName;
//string veloName;
//string omniName;
string ZedRightName;
string ZedLeftName;

string intToString(int i)
{
    stringstream ss;
    ss<<i;
    return ss.str();
}
string doubleToString(double d)
{
    stringstream ss;
    ss<<d;
    return ss.str();
}
//indirectly use std::ToString?

/*
void saveScan(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_ptr,string fileName)
{
    ofstream outfile;
    //definite the output file class outfile.
    outfile.open(fileName.c_str());
    //output to fileName by style of C.
    for (unsigned int i=0; i<in_cloud_ptr->points.size(); i++)
    {
        double x=in_cloud_ptr->points[i].x;
        double y=in_cloud_ptr->points[i].y;
        double z=in_cloud_ptr->points[i].z;

        outfile<<doubleToString(x)+" "+doubleToString(y)+" "+doubleToString(z)<<endl;
    }
    outfile.close();
}

void saveScan(const ,string fileName )
{
    ofstream outfile;
    outfile.open(fileName.c_str());
    for ()
    {

    }
    outfile.close();
}
void omniImageCallback(const sensor_msgs::ImageConstPtr& img)
{
    timeOmniImage0=img->header.stamp.toSec();

    omniImage0 = cv_bridge::toCvCopy(img, "bgr8")->image;

    newOmniImage0=true;
}
void bumImageCallback(const sensor_msgs::ImageConstPtr& img)
{
    timeBumImageLeft=img->header.stamp.toSec();

    bumImageLeft = cv_bridge::toCvCopy(img, "bgr8")->image;

    newBumImageLeft=true;
}
void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& in_sensor_cloud)
{
    timeVelodyne=in_sensor_cloud->header.stamp.toSec();

    pcl::fromROSMsg(*in_sensor_cloud, *current_sensor_cloud_ptr);

    newVelodyne=true;
}
*/
void ZedRightCallback(const sensor_msgs::ImageConstPtr& img)
{
    timeZedImageRight=img->header.stamp.toSec();
    ZedImageRight =cv_bridge::toCvCopy(img,"bgr8")->image;
    newZedImageRight=true;
}
void ZedLeftCallback(const sensor_msgs::ImageConstPtr& img)
{
    timeZedImageLeft=img->header.stamp.toSec();
    ZedImageLeft = cv_bridge::toCvCopy(img,"bgr8")->image;
    newZedImageLeft=true;
}
//callback function will get called when a new message has arrived on the Zed topic.


int main(int argc,char** argv)
{
    ros::init(argc, argv, "car_data_save");
    //Initialize ros.This is also where we specify the unique name of our node.
    ros::NodeHandle h;
    //creat a handle to this process' node.
    ros::NodeHandle private_nh("~");
    //std::string points_topic;
    //std::string omni_topic;
    //std::string bum_topic;

    string baseName;
    //private_nh.param<std::string>("points_topic", points_topic, "velodyne");	ROS_INFO("points_topic: %s", points_topic.c_str());
    //private_nh.param<std::string>("omni_topic", omni_topic, "velodyne");		ROS_INFO("omni_topic: %s", omni_topic.c_str());
    //private_nh.param<std::string>("bum_topic", bum_topic, "velodyne");		ROS_INFO("bum_topic: %s", bum_topic.c_str());
    //private_nh.param<std::string>("zed_topic",zed_topic,"velodyne");            ROS_INFO("zed_topic: %s", zed_topic.c_str());
    private_nh.param<std::string>("baseName", baseName, "/home/zc/Music/");	    ROS_INFO("baseName: %s", baseName.c_str());
    //ROS_INTO and friends are our replacement for pointf/cout.
    //ros::Subscriber sub1 = h.subscribe(points_topic, 1, velodyne_callback);
    //ros::Subscriber sub2 = h.subscribe(omni_topic, 10, omniImageCallback);
    ros::Subscriber sub1 = h.subscribe("/camera/right/image_raw", 10, ZedRightCallback);
    ros::Subscriber sub2 = h.subscribe("/camera/left/image_raw", 10, ZedLeftCallback);
    //ros::Subscriber sub3 = h.subscribe (bum_topic, 10, bumImageCallback);

    int count=1;


    //imageName=baseName+"img_target";
    //veloName=baseName+"laser_target";
    //omniName=baseName+"img_target";
    ZedRightName=baseName+"right_0";
    ZedLeftName=baseName+"left_0";

    ros::Rate rate(20);
    //specify a frequency that you would like to loop at.how long since the last call to Rate::sleep(),and sleep for the correct amount of time.
    //In this case we tell it we want to run at 100Hz.

/*
    ros::ok() will return false if:
    1. a SIGINT is received (Ctrl-C)
    2. we have been kicked off the network by another node with the same name
    3. ros::shutdown() has been called by another part of the application.
    4. all ros::NodeHandles have been destroyed
*/
    while (ros::ok())
    {
        ros::spinOnce();
        //If you were to add a subcription into this application,and did not have ros::spinOnce() here,
        //you callbacks would never get called.

        //if(newOmniImage0 && newVelodyne &&
           //fabs(timeOmniImage0-timeVelodyne)<0.01)
        if(newZedImageRight && newZedImageLeft &&
            fabs(timeZedImageRight - timeZedImageLeft)<0.01)
        {
            //newOmniImage0=false;
            //newBumImageLeft=false;
            //newVelodyne=false;
            newZedImageRight=false;
            newZedImageLeft=false;

            Mat image0,image1;
	        //resize(omniImage0,image0,Size(1280,240));
	        //resize(bumImageLeft,image1,Size(640,480));
            resize(ZedImageRight,image0,Size(1280,720));
            resize(ZedImageLeft,image1,Size(1280,720));
            //imshow("OmniImage0",image0);
            //imshow("BumImageLeft",image1);
            imshow("ZedImageRight",image0);
            imshow("ZedImageLeft",image1);

            //按s保存，按q或esc退出
            char c=waitKey(1);
            if(c==27||c=='q')
            {
                break;
            }
            else if(c=='s')
            {
                cout<<count<<endl;
                //保存
                //imwrite(imageName+intToString(count)+".jpg",bumImageLeft);
                //imwrite(omniName+intToString(count)+".jpg",omniImage0);
                imwrite(ZedRightName + intToString(count)+".jpg",ZedImageRight);
                imwrite(ZedLeftName  + intToString(count)+".jpg",ZedImageLeft);

                //saveScan(current_sensor_cloud_ptr,veloName+intToString(count)+".xyz");
                //saveScan();
                count++;
            }
        }
        rate.sleep();
        //Now we use Ros::Rate object to sleep for the time remaining to let us hit our 100Hz publish rate.
    }
    return 0;
}
