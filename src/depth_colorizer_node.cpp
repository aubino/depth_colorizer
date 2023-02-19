#include <iostream>
#include "ros_wrap.h"
#include <string>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

int main(int argc, char** argv)
{
    ros::init(argc, argv,"depth_colorizer");
    ros::NodeHandle nh;
    std::string cam_info_topic,cloud_topic,cam_image_topic,yaml_file;
    /*po::options_description desc("Mandatory options");
    desc.add_options ()
    ("help,h","This node is used to send cartesian commands to the robot arm. Allowed options are : ")
    ("info,i",po::value(&cam_info_topic),"Te topic on which the program will find camera infos. Mandatory option")
    ("cloud,c",po::value(&cloud_topic),"The topic of the pointcloud. Mandatory option")
    ("image,f",po::value(&cam_image_topic),"The topic of which to find the images. Mandatory option")
    ("yaml_file,y",po::value(&yaml_file),"A yaml file from which all above informations can be taken, A template can be foung in the config folder. The yaml parameters are prioritized over the options");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if(vm.count("help")||vm.count("info")||vm.count("cloud")||vm.count("image"))
    {
        std::cout<<desc;
        return 1;
    }
    
    if(vm.count("yaml_file"))
    {
        std::string ns = ros::this_node::getNamespace();
        // load the rosparams here.
        if(!nh.getParam("info",cam_info_topic))
        {
            ROS_ERROR_STREAM("Failed to load Param info");
            exit (EXIT_FAILURE);
        }
        if(!nh.getParam("cloud",cloud_topic))
        {
            ROS_ERROR_STREAM("Failed to load Param cloud");
            exit (EXIT_FAILURE);
        }
        if(!nh.getParam("image",cam_image_topic))
        {
            ROS_ERROR_STREAM("Failed to load Param image");
            exit (EXIT_FAILURE);
        }
    }
    */
    RosRGBDRect rectifier(nh,cam_info_topic,cloud_topic,cam_image_topic);
    ros::spin();
}