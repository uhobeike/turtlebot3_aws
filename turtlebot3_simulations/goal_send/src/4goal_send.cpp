#include <ros/ros.h>
#include <std_msgs/String.h>
#include <goal_send_msgs/goal_vector.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using std::vector;
using std::cin;
using std::string;
using std::stoi;

string vec_num = "0";

void goal_key_Callback(const std_msgs::StringConstPtr& msg)
{
    ROS_INFO("receive goal number");

    vec_num = msg->data;
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "goal_4");
    ros::NodeHandle nh;
    ros::Publisher pub_pose,pub_goal_data;
    ros::Time tmp_time = ros::Time::now();

    ros::Subscriber sub_key = nh.subscribe("goal_key", 1,  goal_key_Callback);

    pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 2, true);
    pub_goal_data = nh.advertise<goal_send_msgs::goal_vector>("goal_data", 1, true);

    geometry_msgs::PoseWithCovarianceStamped initPose;
    initPose.header.stamp = tmp_time; 
    initPose.header.frame_id = "map"; 
    initPose.pose.pose.position.x = 0;
    initPose.pose.pose.position.y = 0;
    initPose.pose.pose.position.z = 0;
    initPose.pose.pose.orientation.w = 1.0;

    vector<vector<double>> goal_point = 
    {
        { 2.33, 0.339, 0.99847},
        { 5.0958, 1.37547, 0.9735},
        { 5.1, 1.37547, 0.002},
        { 3.08666, 1.8847, 0.3479},
        { 0.943, 2.557, 0.1768}
    };

    goal_send_msgs::goal_vector goal_point_role;
    goal_point_role.data = 
    {   
        "first goal",
        "second gaol",
        "Turning",
        "third goal",
        "final goal"
    };

    pub_pose.publish(initPose);

    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ROS_INFO("The server comes up");

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";                                                                     
    goal.target_pose.header.stamp = ros::Time::now();

    ros::Rate loop_rate(10);//10Hz

    while(ros::ok())
    {  
        if(stoi(vec_num) < goal_point.size() )
        {        
            goal.target_pose.pose.position.x =  goal_point[stoi(vec_num)][0];
            goal.target_pose.pose.position.y =  goal_point[stoi(vec_num)][1];
            goal.target_pose.pose.orientation.w = goal_point[stoi(vec_num)][2];

            ac.sendGoal(goal);

        }

        pub_goal_data.publish(goal_point_role);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
