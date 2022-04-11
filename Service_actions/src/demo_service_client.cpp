#include "ros/ros.h"
#include "mastering_ros_demo_pkg/demo_srv.h"
#include <iostream>

using mastering_ros_demo_pkg::demo_srv;

int main(int argc, char** argv){
    ros::init(argc, argv, "demo_service_client");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    /*ros::ServiceClient*/auto myClient = n.serviceClient<demo_srv>("demo_service");
    demo_srv srv;
    std::string req_comment;
    while(ros::ok()){
        req_comment = "I am a client!";
        srv.request.in = req_comment;
        if(myClient.call(srv)){
            ROS_INFO("From client [%s], the server says [%s]", srv.request.in.c_str(), srv.response.out.c_str());
        }
        else{
            ROS_ERROR("Failed to call service");
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}