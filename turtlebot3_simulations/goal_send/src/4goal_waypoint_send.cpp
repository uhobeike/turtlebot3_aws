#include <ros/ros.h>
#include <std_msgs/String.h>
#include <goal_send_msgs/goal_vector.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <string>
#include <fstream> 
#include<stdio.h>
#include<stdlib.h>
#include <cmath>

using std::vector;
using std::string;
using std::stoi;
using std::stod;
using std::ifstream;
using std::istringstream;
using std::pow;

int goal_restart_flag = 0;
string vec_num = "0";
vector<double> posi_set = 
{
    0,0,0,0
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void goal_key_Callback(const std_msgs::StringConstPtr& msg)
{
    ROS_INFO("receive goal number");

    vec_num = msg->data;

    goal_restart_flag = 1;
}

void posi_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    posi_set.at(0) = msg->pose.pose.position.x;
    posi_set.at(1) = msg->pose.pose.position.y;
    posi_set.at(2) = msg->pose.pose.orientation.z;
    posi_set.at(3) = msg->pose.pose.orientation.w;
}  

void goal_check(vector<vector<string>>& waypoint, int& point_number, int& next_point_flag, int& goal_point_flag)
{   
    if(waypoint[point_number].size() >= 0 && waypoint[point_number].size() <= 4)
    {
        next_point_flag = 1;
    }
    else if(waypoint[point_number].size() > 4)
    {
        goal_point_flag = 1;
    }

    if(goal_restart_flag)
    {
        point_number++;

        goal_restart_flag = 0;
    }
}

void waypoint_nearly_check(vector<vector<string>>& waypoint, vector<double>& odom, int& point_number, int& goal_point_flag)
{
    double nealy_check_area = 0;
    double x_way = 0,y_way = 0,x_odm = 0,y_odm = 0;
    x_way = stod(waypoint[point_number][0]);
    y_way = stod(waypoint[point_number][1]);
    x_odm = odom[0];
    y_odm = odom[1];
    nealy_check_area = sqrt(pow( x_way - x_odm, 2) + pow( y_way - y_odm, 2) );
    
    if(nealy_check_area == 0.1) point_number++;
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "goal_4_waypoint");
    ros::NodeHandle nh;
    ros::Publisher pub_pose,pub_goal_data;
    ros::Time tmp_time = ros::Time::now();

    ros::Subscriber sub_key = nh.subscribe("goal_key", 1,  goal_key_Callback);
    ros::Subscriber sub_pos = nh.subscribe("odom", 100,  posi_Callback);

    pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 2, true);
    pub_goal_data = nh.advertise<goal_send_msgs::goal_vector>("goal_data", 1, true);

    geometry_msgs::PoseWithCovarianceStamped initPose;
    initPose.header.stamp = tmp_time; 
    initPose.header.frame_id = "map"; 
    initPose.pose.pose.position.x = 0;
    initPose.pose.pose.position.y = 0;
    initPose.pose.pose.position.z = 0;
    initPose.pose.pose.orientation.w = 1;

    char *c = getenv("HOME");
    string HOME = c; 
    ifstream f_r(HOME + "/waypoint.csv",std::ios::in);

    vector<vector<string>> waypoint_read;
    string line,field;
    int vec_num_int = 1;

    waypoint_read.emplace_back();

    while (getline(f_r, line)) 
    {
        istringstream stream(line);
        while (getline(stream, field, ',') )
        {
            waypoint_read[vec_num_int-1].push_back(field);
        }

        waypoint_read.resize(++vec_num_int);
    }
    waypoint_read.resize(--vec_num_int);
    waypoint_read.resize(--vec_num_int);

    goal_send_msgs::goal_vector goal_point_role;
    goal_point_role.data = 
    {   
        "first goal",
        "second gaol",
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

    int point_number=0;
    int next_point_flag = 0;
    int goal_point_flag = 0;

    while(ros::ok())
    {  
        goal_check(waypoint_read,  point_number, next_point_flag, goal_point_flag);

        if(next_point_flag)
        {        
            goal.target_pose.pose.position.x    = stod(waypoint_read[point_number][0]);
            goal.target_pose.pose.position.y    = stod(waypoint_read[point_number][1]);
            goal.target_pose.pose.orientation.z = stod(waypoint_read[point_number][2]);
            goal.target_pose.pose.orientation.w = stod(waypoint_read[point_number][3]);

            ac.sendGoal(goal);

            waypoint_nearly_check(waypoint_read, posi_set, point_number, goal_point_flag);
            next_point_flag = 0; 
        }

        else if (goal_point_flag)
        {
            goal.target_pose.pose.position.x    = stod(waypoint_read[point_number][0]);
            goal.target_pose.pose.position.y    = stod(waypoint_read[point_number][1]);
            goal.target_pose.pose.orientation.z = stod(waypoint_read[point_number][2]);
            goal.target_pose.pose.orientation.w = stod(waypoint_read[point_number][3]);

            ac.sendGoal(goal);

            goal_point_flag = 0;
        }

        pub_goal_data.publish(goal_point_role);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
