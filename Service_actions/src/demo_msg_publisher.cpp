#include "ros/ros.h"
/*Message definition is written in ../msg/demo_msg.msg */
#include "mastering_ros_demo_pkg/demo_msg.h"
#include <iostream>

int main(int argc, char **argv){
    /*Initialize with a name of demo_topic_publisher*/
    ros::init(argc, argv, "demo_msg_publisher");
    /*Create a node handle*/
    ros::NodeHandle node_obj;
    /*Create a publisher and advertise it with name of "/demo_msg_topic" */
    /*Type of the message is "mastering_ros_demo_pkg::demo_msg"*/
    ros::Publisher number_publisher = node_obj.advertise<mastering_ros_demo_pkg::demo_msg>("/demo_msg_topic", 10);
    /*Create delation object*/
    ros::Rate loop_rate(10);
    int number_count = 0;
    mastering_ros_demo_pkg::demo_msg msg;
    std::string text_msg[2];
    text_msg[0] = "Hello, world!";
    text_msg[1] = "Bye, world!!";
    while(ros::ok()){
        /*String message shall be given with string class.*/
        msg.greeting = text_msg[number_count%2];
        msg.number = number_count;
        /*But ROS_INFO() receives traditional pointer for strings.*/
        ROS_INFO("%s", msg.greeting.c_str());
        ROS_INFO("%d", msg.number);

        /*Publish a message and sleep*/
        number_publisher.publish(msg);
        loop_rate.sleep();


        ++number_count;
        number_count %= 10;
    }
}