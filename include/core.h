#include <opencv4/opencv2/rgbd.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <tuple>
#include <math.h>

/// Need a function to take a point cloud , an image  and 
/// The transform between the two frames of the cloud and the image as well as the intrinsics 
/// of the color frame.
/// The output have to be the point_cloud colorized , the image rectified ans the depth relative to the color frame (ie the z component of the pcl in that frame) 
/// and the new camera intrinsic since some pixels will likely be lost due to misalignement

std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr,cv::Mat3b,cv::Mat1d,cv::Mat1f> colorize_rectify(pcl::PointCloud<pcl::PointXYZRGB> cloud_in,
                                                                                            cv::Mat3b color_in, 
                                                                                            Eigen::Affine3d transform,
                                                                                            cv::Mat intrinsics,
                                                                                            cv::Mat distorsion);
