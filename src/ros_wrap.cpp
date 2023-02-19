#include "ros_wrap.h"

RosRGBDRect::RosRGBDRect(ros::NodeHandle nh,std::string cam_info_topic, std::string cloud_topic,std::string cam_image_topic):
nh_(nh),
tfBuffer_(),
tfListener_(tfBuffer_)
{
    cam_info_sub_ = nh.subscribe(cam_info_topic,1,&RosRGBDRect::cam_info_callback,this);
    cam_image_sub_ = nh.subscribe(cam_image_topic,1,&RosRGBDRect::cam_image_callback,this);
    cloud_sub_ = nh.subscribe(cloud_topic,1,&RosRGBDRect::cloud_callback,this);
    rect_color_pub_ = nh.advertise<sensor_msgs::Image>("rect/image",1,true);
    rect_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("rect/cloud",1,true);
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
    cv::Mat intrK(3,3,CV_64FC1,(void*) info->K.data());
    cv::Mat intrD(5,1,CV_64FC1);
    if(info->D.size())
    {
        cv::Mat buff(info->D.size(),1,CV_64FC1,(void*) info->D.data());
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

void RosRGBDRect::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::fromROSMsg(*msg,cloud_in_);
    cloud_in_.header.frame_id = msg->header.frame_id; 
    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer_.lookupTransform(cam_info_.header.frame_id,cloud_in_.header.frame_id,ros::Time(0));
    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.5).sleep();
    } 
    Eigen::Affine3d t_eigen; 
    tf2::Stamped<Eigen::Affine3d> t_in; //to do : initialize this to zero transform
    tf2::doTransform(t_in,t_in,transformStamped); 
    t_eigen.affine() = t_in.affine();
    t_eigen.linear() = t_in.linear();
    auto result = colorize_rectify(cloud_in_,image_in_,t_eigen,intrinsicsK_in_,intrinsicsD_in_);
    cloud_out_ = std::get<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(result);
    image_out_ = *cv_bridge::CvImage(cam_info_.header,sensor_msgs::image_encodings::BGR8,std::get<cv::Mat3b>(result)).toImageMsg();
    depth_out_ = *cv_bridge::CvImage(cam_info_.header,sensor_msgs::image_encodings::TYPE_64FC1,std::get<cv::Mat1d>(result)).toImageMsg();
    info_out_ = cam_info_ ;
    sensor_msgs::PointCloud2 cloud_out2;
    pcl::toROSMsg<pcl::PointXYZRGB>(*cloud_out_,cloud_out2);
    rect_cloud_pub_.publish(cloud_out2);
    rect_color_pub_.publish(image_out_);
    rect_depth_pub_.publish(depth_out_);
    rect_info_pub_.publish(info_out_);
}
