#include "ros/ros.h"
#include <iostream>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "mastering_ros_demo_pkg/Demo_actionAction.h"
/*Shorten long and dirty type definition*/
using GOAL_STATE = actionlib::SimpleClientGoalState::StateEnum;

class MyActionClient{
    protected:
    /*Actual action client*/
    actionlib::SimpleActionClient<mastering_ros_demo_pkg::Demo_actionAction> ac_;
    /*Goal input*/
    mastering_ros_demo_pkg::Demo_actionGoal goal_;


    void DoneCB(const actionlib::SimpleClientGoalState &state, const mastering_ros_demo_pkg::Demo_actionResultConstPtr& _result){
        /*Report the status when it finished in such cases.*/
        auto state_now = ac_.getState();
        if(state_now == GOAL_STATE::SUCCEEDED){
            ROS_INFO("The action has finished successfully.: %d", _result->final_count);
        }else if(state_now == GOAL_STATE::PREEMPTED){
            ROS_INFO("The action has Preempted.");
        }else{
            ROS_ERROR("The action has terminated unexpectly.");
        }
    }
    void ActiveCB(){
        ROS_INFO("Now the action has activated");
    }
    void FeedbackCB(const mastering_ros_demo_pkg::Demo_actionFeedbackConstPtr& _feedback){
        /*Report when the server published feedback.*/
        ROS_INFO("Feedback: Now the state is %d", _feedback->current_number);
    }

    public:
    MyActionClient(ros::NodeHandle& _nh, const std::string& _to_subscribe = ""):
    ac_(_nh, _to_subscribe,true)    {
        ac_.waitForServer();
    };
    void SendGoal(const std::int32_t& _goal){
        goal_.count = _goal;
        ROS_INFO("Sending Goal [%d]",goal_.count);
        ac_.sendGoal(goal_,
        boost::bind(&MyActionClient::DoneCB, this, _1, _2)
        ,boost::bind(&MyActionClient::ActiveCB, this)
        ,boost::bind(&MyActionClient::FeedbackCB, this, _1)    );
    }
    void CancelGoal(){
        ac_.cancelGoal();
    }
    const actionlib::SimpleClientGoalState GetState() const {
        auto state_now = ac_.getState();
        return state_now;
    }
};

int main (int argc, char **argv) {
    ros::init(argc, argv, "demo_action_client");
    ros::NodeHandle nh_;

    if(argc != 3){
        ROS_INFO("%d",argc);
        ROS_WARN("Usage: demo_action_client <goal> <time_to_preempt_in_sec>");
        return 1;
    }
    /*Set a goal from argument 1*/
    mastering_ros_demo_pkg::Demo_actionGoal goal;
    goal.count = atoi(argv[1]);


    ROS_INFO("Waiting for action server to start.");
    auto myActClient = new MyActionClient(nh_, "Demo_action");
    ROS_INFO("Action server started, sending goal.");

    myActClient->SendGoal(atoi(argv[1]));

    unsigned int lmt_count, count;
    count = 0;
    lmt_count = atoi(argv[2]);

    ros::Rate sleep_rate(1);
    while(ros::ok()){
        if(++count > lmt_count){
            /*Cancel the goal. Then, the client will be killed in the next sentence.*/
            myActClient->CancelGoal();
        }
        auto GOAL_STATE = myActClient->GetState();
        ROS_INFO("Goal status: %s", GOAL_STATE.toString().c_str());
        if((GOAL_STATE != GOAL_STATE::ACTIVE)&&(GOAL_STATE != GOAL_STATE::PENDING))
        {
            delete myActClient; /*Goal is deadvertised by delete*/
            break;/*E.N.D*/
        }
        sleep_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}