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
    tf2_ros::Buffer tfBuffer_; 
    tf2_ros::TransformListener tfListener_;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_in_; 
    pcl::PointCloud<pcl::PointXYZRGB> cloud_out_;
    cv::Mat3b image_in_;
    cv::Mat1f intrinsicsK_in_;
    cv::Mat1f intrinsicsD_in_;
    sensor_msgs::Image image_out_;
    sensor_msgs::Image depth_out_;
    sensor_msgs::CameraInfo info_out_; 

    public: 
    RosRGBDRect(ros::NodeHandle nh,std::string cam_info_topic, std::string cloud_topic,std::string cam_image_topic);
    void cam_info_callback(const sensor_msgs::CameraInfo::ConstPtr& info); 
    void cam_image_callback(const sensor_msgs::Image::ConstPtr& msg);
    void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

};
