#include "core.h"

std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,cv::Mat3b,cv::Mat1d,cv::Mat1f> colorize_rectify(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_in,
                                                                                            const cv::Mat3b& color_in, 
                                                                                            Eigen::Affine3d transform,
                                                                                            cv::Mat intrinsics,
                                                                                            cv::Mat distorsion)
{
    cv::Mat1d depth_result(color_in.rows,color_in.cols);
    int max_col(0) ,max_row(0);
    int min_col(0), min_row(0); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::transformPointCloud(cloud_in, *transformed_cloud, transform);
    pcl::PointIndices z_indices;//to filter on z >0
    std::vector<cv::Point3f> cv_points; 
    for(auto p : *transformed_cloud)
        cv_points.push_back(cv::Point3f(p.x,p.y,p.z));
    cv::Mat tVec = cv::Mat::zeros(3, 1, cv::DataType<double>::type); // Translation vector
    cv::Mat rotVec; 
    cv::Rodrigues(cv::Mat::eye(3,3,cv::DataType<double>::type),rotVec);
    std::vector<cv::Point2f> cv_pixels;
    cv::projectPoints(cv_points,tVec,rotVec,intrinsics,distorsion,cv_pixels);
    for(auto p:cv_points)
    {
        min_col = std::min((float) min_col,p.x); 
        max_col = std::max((float) max_col,p.x);
        min_row = std::min((float) min_row,p.y); 
        max_row = std::max((float) max_row,p.y);  
    }
    min_col = std::max(0,min_col); 
    max_col = std::min(color_in.cols,max_col); 
    min_row = std::max(0,min_row); 
    max_row = std::min(color_in.rows,max_row);
    cv::Mat3b color_out(max_row-min_row,max_col-min_col);
    cv::Mat1d depth_out(max_row-min_row,max_col-min_col);
    cv::Mat1f intrinsics_out = intrinsics;
    //now translate the center of the intrinsics 
    intrinsics_out.at<float>(0,2) -= min_col;
    intrinsics_out.at<float>(1,2) -= min_row;
    for (int i =0; i<cv_points.size(); i++)
    {
        auto pixel = cv_pixels[i];
        auto point = cv_points[i]; 
        auto pcl_point = transformed_cloud->points[i];
        if(pixel.x<=max_col && pixel.x>= min_col && pixel.y<=max_row && pixel.y>=min_row)
        {
            transformed_cloud->points[i].b =  color_out.at<cv::Vec3b>((int)pixel.y,(int)pixel.x)[0];
            transformed_cloud->points[i].g =  color_out.at<cv::Vec3b>((int)pixel.y,(int)pixel.x)[1];
            transformed_cloud->points[i].r =  color_out.at<cv::Vec3b>((int)pixel.y,(int)pixel.x)[2];
            color_out.at<cv::Vec3b>((int)pixel.y-min_row,(int)pixel.x-min_col) = color_out.at<cv::Vec3b>((int)pixel.y,(int)pixel.x);
            depth_out.at<double>((int)pixel.y-min_row,(int)pixel.x-min_col) = point.z; ///have to verify the condition z>0. We nevze know. actually the best os to filter transformed_cloud to retain only z>0;
        } 
    }
    return std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,cv::Mat3b,cv::Mat1d,cv::Mat1f>{transformed_cloud,color_out,depth_out,intrinsics_out};
}