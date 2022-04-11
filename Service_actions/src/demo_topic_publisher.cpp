#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>

int main(int argc, char **argv){
    /*Initialize with a name of demo_topic_publisher*/
    ros::init(argc, argv, "demo_topic_publisher");
    /*Create a node handle*/
    ros::NodeHandle node_obj;
    /*Create a publisher and advertise it with name of "/numbers" */
    ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/numbers", 10);
    /*Create delation object*/
    ros::Rate loop_rate(10);
    int number_count = 0;
    std_msgs::Int32 msg;
    while(ros::ok()){
        msg.data = number_count;
        ROS_INFO("%d", msg.data);
        number_publisher.publish(msg);
        loop_rate.sleep();
        ++number_count;
        number_count %= 10;
    }
}