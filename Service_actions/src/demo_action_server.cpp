#include <iostream>
#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include "mastering_ros_demo_pkg/Demo_actionAction.h"

class Demo_actionAction
{
    protected:
    /*What is name of the action?*/
    std::string action_name;
    int progress;
    ros::Rate cycle_freq;
    /*Declare the action server*/
    actionlib::SimpleActionServer<mastering_ros_demo_pkg::Demo_actionAction> as_;
    mastering_ros_demo_pkg::Demo_actionFeedback fb_;
    mastering_ros_demo_pkg::Demo_actionResult result_;

    /*Callback fncs. for exec, preempt*/
    void ExecCallback(const mastering_ros_demo_pkg::Demo_actionGoalConstPtr& _goal){
        const auto goal_tmp = _goal->count;
        if( (as_.isActive() == false) && (as_.isPreemptRequested() == true) ) return; /*Reject when it is not activated or preempted*/
        ROS_INFO("This action trying to reach goal %d", goal_tmp);
        progress = 0;
        /*Iterates until the job is complete*/
        while(!isJobCplt(goal_tmp)){
            /*When the user input Ctrl + C to shutdown this node*/
            if(!ros::ok()){
                /*Report progress to final count and set this action as aborted.*/
                result_.final_count = progress;
                as_.setAborted(result_, "I failed!");
                ROS_INFO("%s is shutted down.", action_name.c_str());
                break;
            }
            /*Reject when it is not activated or preempted*/
            if( (as_.isActive() == false) && (as_.isPreemptRequested() == true) ) return;
            /*Do job something*/
            job();
            /*Is the job complete?*/
            if(isJobCplt(goal_tmp)){
                /*Report progress to final count and set this action as succeeded.*/
                result_.final_count = progress;
                as_.setSucceeded(result_, "I succeeded!");
                ROS_INFO("%s is reached the goal %d.", action_name.c_str(), goal_tmp);
            }
            else{
                /*Set the feedback and publish a feedback.*/
                fb_.current_number = progress;
                as_.publishFeedback(fb_);
                ROS_INFO("%s is %d of the goal %d.", action_name.c_str(), progress,  goal_tmp);
            }
            cycle_freq.sleep();
        }
    }
    void PreemptCallback(){
        /*Report progress to final count and set this action as preempted.*/
	    result_.final_count = progress;
	    as_.setPreempted(result_,"I got Preempted"); 
	    ROS_WARN("%s got preempted!", action_name.c_str());
    }


    inline void job(){++progress;}
    inline bool isJobCplt(const std::int32_t& _goal){ return static_cast<bool>(progress >= _goal);}

    public:
    Demo_actionAction(const ros::NodeHandle& _nh, const std::string& _action_name, const unsigned int& _cycle_freq = 5)
    /*Register execution callback fnc. with initialization list.*/
    :as_(_nh, _action_name, boost::bind(&Demo_actionAction::ExecCallback, this, _1), false)
    /*Set name and cycle frequency of this action.*/
    ,action_name(_action_name), cycle_freq(_cycle_freq)
    {
        /*Register Preempt callback fnc..*/
        as_.registerPreemptCallback(boost::bind(&Demo_actionAction::PreemptCallback, this));
    }

    
    void StartAction(){as_.start();}
};
int main(int argc, char** argv){
    ros::init(argc, argv, "Demo_action_server");
    ros::NodeHandle nh_;
    ros::NodeHandle nh2_;
    ROS_INFO("Starting Demo Action Server");
    auto myaction = new Demo_actionAction(nh_, "Demo_action", 5);
    myaction->StartAction();
    while(ros::ok()){
        ros::spinOnce();
    }
    delete myaction;
    return 0;
}