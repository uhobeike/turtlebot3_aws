#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <string>
#include <fstream> 
#include <sstream>
#include <cmath>

using std::vector;
using std::string;
using std::stoi;
using std::stod;
using std::ifstream;
using std::istringstream;
using std::pow;
using std::cout;
using std::endl;

bool start_flag;
bool lock_flag;
bool goal_reached_flag;
bool goal_restart_flag;
bool final_goal_flag;
uint16_t goal_reached_cnt;
string vec_num = "0";
vector<double> posi_set = 
{
    0,0,0,0
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void goal_key_Callback(const std_msgs::StringConstPtr& msg)
{
    if(msg->data == "q")
    {
        ROS_INFO("Shutdown now ('o')/ bye bye~~~");
        ros::shutdown();
    }

    ROS_INFO("Received a control command");
    
    if(start_flag) goal_restart_flag = 1;
    
    start_flag = 1;
}

void posi_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    posi_set.at(0) = msg->pose.pose.position.x;
    posi_set.at(1) = msg->pose.pose.position.y;
    posi_set.at(2) = msg->pose.pose.orientation.z;
    posi_set.at(3) = msg->pose.pose.orientation.w;
}  

void goal_reached_Callback(const actionlib_msgs::GoalStatusArray::ConstPtr &status)
{
    if (!status->status_list.empty() )
    {
        actionlib_msgs::GoalStatus goalStatus = status->status_list[0];

        if(goalStatus.status == 3 && goal_reached_flag == 0 && lock_flag == 0) goal_reached_cnt++;

        if(goal_reached_cnt >= 10)
        {
            goal_reached_flag = 1;
            goal_reached_cnt = 0;
        }

        if(goalStatus.status != 0 && goalStatus.status != 3) lock_flag = 0;
    }
}  

void goal_check(vector<vector<string>>& waypoint, int& point_number, int& vec_size, int& next_point_flag, int& goal_point_flag)
{   
    if(waypoint[point_number].size() >= 0 && waypoint[point_number].size() <= 4) next_point_flag = 1;

    else if(waypoint[point_number].size() > 4) goal_point_flag = 1;

    if(point_number == (vec_size - 2) && goal_reached_flag) final_goal_flag = 1;

    if(goal_reached_flag)
    {
        ROS_INFO("Goal reached");
        ROS_INFO("go Comand Only ,but 'q' is shutdown");
        goal_reached_flag = 0;
        lock_flag = 1;

        if(waypoint[point_number].size() == 4)
        {
            point_number++;
            goal_restart_flag = 0;
        }
    }

    if(goal_restart_flag)
    {
        ROS_INFO("RESTART");
        
        point_number++;
        goal_restart_flag = 0;
    }

    if(final_goal_flag)
    {
        ROS_INFO("Final goal reached");
        ROS_INFO("please send q command");
    }
}

void waypoint_nearly_check(vector<vector<string>>& waypoint, vector<double>& estimated_pos, int& point_number, int& goal_point_flag, double& area_threshold)
{
    double nealy_check_area = 0;
    double x_way = 0,y_way = 0,x_posi = 0,y_posi = 0;
    x_way = stod(waypoint[point_number][0]);
    y_way = stod(waypoint[point_number][1]);
    x_posi = estimated_pos[0];
    y_posi = estimated_pos[1];
    nealy_check_area = sqrt(pow( x_way - x_posi, 2) + pow( y_way - y_posi, 2) );
    
    if(nealy_check_area <= area_threshold)
    {
        ROS_INFO("WAY_POINT PASSING");
        ROS_INFO("NEXT MOVE PLAN");

        point_number++;
    }
}

void waypoint_pose_array(vector<vector<string>>& waypoint_read, geometry_msgs::PoseArray& pose_array, geometry_msgs::Pose pose)
{
    uint16_t vec_cnt_out = 0, vec_cnt_in = 0;
    for (auto it_t = waypoint_read.begin(); it_t != waypoint_read.end(); ++it_t) 
    {
        vec_cnt_in = 0;
        for (auto it = (*it_t).begin(); it != (*it_t).end(); ++it) 
        {
            vec_cnt_in++;
            if(vec_cnt_in == 5) break;

            if(vec_cnt_in == 1) pose.position.x = stod(*it);
            if(vec_cnt_in == 2) pose.position.y = stod(*it);
            
            pose.position.z = 0.2;
            
            if(vec_cnt_in == 3) pose.orientation.z = stod(*it);
            if(vec_cnt_in == 4) pose.orientation.w = stod(*it);
        }

        pose_array.poses.push_back(pose);
    }
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "goal_4_waypoint");
    ros::NodeHandle nh;
    ros::Publisher pub_pose_ini, pub_pose_way;
    ros::Time tmp_time = ros::Time::now();

    ros::Subscriber sub_key = nh.subscribe("goal_key", 1,  goal_key_Callback);
    ros::Subscriber sub_pos = nh.subscribe("amcl_pose", 100,  posi_Callback);
    ros::Subscriber sub_goal = nh.subscribe("move_base/status", 100,  goal_reached_Callback);

    pub_pose_ini = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 2, true);
    pub_pose_way = nh.advertise<geometry_msgs::PoseArray>("waypoint", 2, true);

    geometry_msgs::PoseWithCovarianceStamped initPose;
    initPose.header.stamp = tmp_time; 
    initPose.header.frame_id = "map"; 
    initPose.pose.pose.position.x = 0;
    initPose.pose.pose.position.y = 0;
    initPose.pose.pose.position.z = 0;
    initPose.pose.pose.orientation.w = 1;

    vector<vector<string>> waypoint_read = 
    {
        {"2.373438","0.238057","0.005113","0.999987","goal","0.4"},
        {"4.479971","0.376918","-0.002169","0.999998"},
        {"4.515158","0.761937","0.703631","0.710565"},
        {"4.515285","1.282344","0.702802","0.711386"},
        {"5.127082","1.294671","0.999952","0.009825","goal","0.4"},
        {"3.240542","1.380090","0.999999","0.001407"},
        {"3.161743","1.831384","0.830963","0.556328","goal","0.3"},
        {"2.947154","2.108109","-0.999995","0.003270"},
        {"2.017048","2.120343","0.959572","0.281462"},
        {"1.684018","2.434848","0.952581","0.304284"},
        {"1.143563","2.450880","-0.873733","0.486406","goal","0.4"},
        {"0.999856","2.142421","-0.725193","0.688546"},
        {"1.023423","1.189682","-0.704009","0.710191"},
        {"0.717609","1.037424","-0.997880","0.065081"},
        {"0.213692","0.954463","-0.998536","0.054091"},
        {"0.483041","0.473004","-0.438033","0.898959"},
        {"0.774195","0.061594","0.999990","0.004416"},
        {"0.0","0.0","-0.999944","0.010547","goal","0.4"}
    };

    geometry_msgs::PoseArray pose_array;
    geometry_msgs::Pose pose;
    pose_array.header.stamp = ros::Time::now(); 
    pose_array.header.frame_id = "map"; 
    
    waypoint_pose_array(waypoint_read, pose_array, pose);

    pub_pose_way.publish(pose_array);

    pub_pose_ini.publish(initPose);

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

    int vec_size = waypoint_read.size();
    int point_number=0;
    int next_point_flag = 0;
    int goal_point_flag = 0;
    double area_threshold = 0.4;
    while(ros::ok())
    {  
        if(start_flag)
        {
            goal_check(waypoint_read,  point_number, vec_size, next_point_flag, goal_point_flag);

            if(next_point_flag)
            {   
                goal.target_pose.pose.position.x    = stod(waypoint_read[point_number][0]);
                goal.target_pose.pose.position.y    = stod(waypoint_read[point_number][1]);
                goal.target_pose.pose.orientation.z = stod(waypoint_read[point_number][2]);
                goal.target_pose.pose.orientation.w = stod(waypoint_read[point_number][3]);

                ac.sendGoal(goal);

                waypoint_nearly_check(waypoint_read, posi_set, point_number, goal_point_flag, area_threshold);
                next_point_flag = 0; 
            }

            else if (goal_point_flag)
            {
                goal.target_pose.pose.position.x    = stod(waypoint_read[point_number][0]);
                goal.target_pose.pose.position.y    = stod(waypoint_read[point_number][1]);
                goal.target_pose.pose.orientation.z = stod(waypoint_read[point_number][2]);
                goal.target_pose.pose.orientation.w = stod(waypoint_read[point_number][3]);

                ac.sendGoal(goal);
                
                if(waypoint_read[point_number].size() == 6) area_threshold = stod(waypoint_read[point_number][5]);
                goal_point_flag = 0;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
