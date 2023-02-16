#include "core.h"
#include "ros/ros.h"
#include "pcl_ros/impl/transforms.hpp"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud.h"
#include "tf2/buffer_core.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "cv_bridge/cv_bridge.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf/transform_datatypes.h"

class RosRGBDRect
{
    private :
    ros::NodeHandle nh_; 
    ros::Subscriber  cam_info_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber cam_image_sub_;
    ros::Publisher  rect_color_pub_;
    ros::Publisher  rect_depth_pub_;
    ros::Publisher rect_cloud_pub_;
    ros::Publisher rect_info_pub_;
    sensor_msgs::CameraInfo cam_info_; 
    tf::TransformListener listener_;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_in_; 
    pcl::PointCloud<pcl::PointXYZRGB> cloud_out_;
    cv::Mat3b image_in_;
    cv::Mat1f intrinsicsK_in_;
    cv::Mat1f intrinsicsD_in_;
    sensor_msgs::Image image_out_;
    sensor_msgs::Image depth_out_;
    sensor_msgs::CameraInfo info_out_;
    cv::Mat1f intrinsics_out_;
    public: 
    RosRGBDRect(ros::NodeHandle nh,std::string cam_info_topic, std::string cloud_topic,std::string cam_image_topic);
    void cam_info_callback(const sensor_msgs::CameraInfo::ConstPtr& info); 
    void cam_image_callback(const sensor_msgs::Image::ConstPtr& msg);
    void cloud_callback(const sensor_msgs::PointCloud::ConstPtr& msg);

};

RosRGBDRect::RosRGBDRect(ros::NodeHandle nh,std::string cam_info_topic, std::string cloud_topic,std::string cam_image_topic):
nh_(nh)
{
    cam_info_sub_ = nh.subscribe(cam_info_topic,1,cam_info_callback,this);
    cam_image_sub_ = nh.subscribe(cam_image_topic,1,cam_image_callback,this);
    cloud_sub_ = nh.subscribe(cloud_topic,1,cloud_callback,this);
    rect_color_pub_ = nh.advertise<sensor_msgs::Image>("rect/image",1,true);
    rect_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud>("rect/cloud",1,true);
    rect_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("rect/info",1,true); 
    rect_depth_pub_ = nh.advertise<sensor_msgs::Image>("rect/depth",1,true); 
}
void RosRGBDRect::cam_image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
       {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
       }
       catch (cv_bridge::Exception& e)
       {
            ROS_ERROR("cv_bridge exception: %s", e.what());
       }
    image_in_ = cv_ptr->image ;
}

void RosRGBDRect::cam_info_callback(const sensor_msgs::CameraInfo::ConstPtr& info)
{
    cam_info_ = *info;
    cv::Mat intrK(3,3,CV_64FC1,(void*) info.K.data());
    cv::Mat intrD(5,1,CV_64FC1);
    if(info.D.size())
    {
        cv::Mat buff(info.D.size(),1,CV_64FC1,(void*) info.D.data());
        intrD = buff ;
    }
    else
    {
        for(int i=0; i<5; i++)
            intrD.at<float>(i) = 0;
    }
    intrinsicsK_in_ = intrK;
    intrinsicsD_in_ = intrD;
}

void RosRGBDRect::cloud_callback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    pcl::fromROSMsg(*msg,cloud_in_);
    cloud_in_.header = msg->header; 
    tf::Transform transform;
    try{
        listener_.lookupTransform(cloud_in_.header.frame_id,cam_info_.header.frame_id
                                  ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.5).sleep();
        continue;
    } 
    Eigen::Affine3d e; 
    tf::transformTFToEigen(transform,e); 
    auto result = colorize_rectify(cloud_in_,image_in_,e,intrinsicsK_in_,intrinsicsD_in_);
    sensor_msgs::Image
    //next task : 
    //* add tf listener to atributes 
    // lookup the transform with exception handling 
    //convert the transform to eigen (see doc : http://docs.ros.org/en/kinetic/api/tf_conversions/html/c++/tf__eigen_8h.html) 
    // use the core functions
}
