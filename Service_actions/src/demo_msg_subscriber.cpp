#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "mastering_ros_demo_pkg/demo_msg.h"
#include <iostream>

void msg_callback(const mastering_ros_demo_pkg::demo_msg& msg){
    ROS_INFO("Received greeting: [%s]", msg.greeting.c_str());
    ROS_INFO("Received number: [%d]", msg.number);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "demo_msg_subscriber");
    ros::NodeHandle node_obj;
    /*Register callback function on the master*/
    ros::Subscriber number_subscriber = node_obj.subscribe("/demo_msg_topic", 10, msg_callback);
    /*Yield the control to the master and wait*/
    /*Callback function may be called for each publish*/
    ros::spin();
    return 0;
}