#include <iostream>
#include "ros_wrap.h"
#include <string>
#include <boost/program_options.hpp>

int main(int argc, char** argv)
{
    std::string cam_info_topic,cloud_topic,cam_image_topic,yaml_file;
    namespace po = boost::program_options;
    po::options_description desc("Mandatory options");
    desc.add_options ()
    ("help,h","This node is used to send cartesian commands to the robot arm. Allowed options are : ")
    ("info,i",po::value<std::string>(&cam_info_topic),"Te topic on which the program will find camera infos. Mandatory option")
    ("cloud,c",po::value<std::string>(&cloud_topic),"The topic of the pointcloud. Mandatory option")
    ("image,f",po::value<std::string>(&cam_image_topic),"The topic of which to find the images. Mandatory option")
    ("yaml_file,y",po::value<std::string>(&yaml_file),"A yaml file from which all above informations can be taken, A template can be foung in the config file");
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
        // load the rosparams here.
    }
    ros::init(argc, argv,"depth_colorizer");
    ros::NodeHandle nh;
    RosRGBDRect rectifier(nh,cam_info_topic,cloud_topic,cam_image_topic);
    ros::spin();
}