#include "ros/ros.h"
#include "mastering_ros_demo_pkg/demo_srv.h"
#include <iostream>

using mastering_ros_demo_pkg::demo_srv;

bool demo_service_callback(demo_srv::Request& req, demo_srv::Response& res){
    std::string response = "I am a server!";
    res.out = response;
    ROS_INFO("From client [%s], Server says [%s]", req.in.c_str(), res.out.c_str());
    return true;
}
int main(int argc, char** argv){
    ros::init(argc, argv, "demo_service_server");
    ros::NodeHandle n;
    /*ros::ServiceServer*/ auto myServer = n.advertiseService("demo_service", demo_service_callback);
    ROS_INFO("Ready to react to the client");
    ros::spin();
    return 0;
}