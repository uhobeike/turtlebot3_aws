#include <ros/ros.h>
#include <std_msgs/String.h>
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
string vec_num = "0";
vector<double> posi_set = 
{
    0,0,0,0
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void goal_key_Callback(const std_msgs::StringConstPtr& msg)
{
    string goal_key = msg->data;
    if(goal_key.find("quit") != string::npos)
    {
        ROS_INFO("Shutdown now ('o')/ bye bye~~~");
        ros::shutdown();
    }

    ROS_INFO("Received a control command");
    
    if(start_flag) goal_restart_flag = 1;
    
    start_flag = 1;
}

void posi_Callback(const nav_msgs::Odometry::ConstPtr& msg)
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

        if(goalStatus.status == 3 && goal_reached_flag == 0 && lock_flag == 0) goal_reached_flag = 1;
        
        if(goalStatus.status != 0 && goalStatus.status != 3) lock_flag = 0;
    }
}  

void goal_check(vector<vector<string>>& waypoint, int& point_number, int& vec_size, int& next_point_flag, int& goal_point_flag)
{   
    if(waypoint[point_number].size() >= 0 && waypoint[point_number].size() <= 4)
    {
        next_point_flag = 1;
    }
    else if(waypoint[point_number].size() > 4)
    {
        goal_point_flag = 1;
    }

    if(point_number == (vec_size - 2) && goal_reached_flag) final_goal_flag = 1;

    if(goal_reached_flag)
    {
        ROS_INFO("Goal reached");
        ROS_INFO("go Comand Only ,but 'q' is shutdown");
        goal_reached_flag = 0;
        lock_flag = 1;
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

void waypoint_nearly_check(vector<vector<string>>& waypoint, vector<double>& odom, int& point_number, int& goal_point_flag, double& area_threshold)
{
    double nealy_check_area = 0;
    double x_way = 0,y_way = 0,x_odm = 0,y_odm = 0;
    x_way = stod(waypoint[point_number][0]);
    y_way = stod(waypoint[point_number][1]);
    x_odm = odom[0];
    y_odm = odom[1];
    nealy_check_area = sqrt(pow( x_way - x_odm, 2) + pow( y_way - y_odm, 2) );
    
    if(nealy_check_area <= area_threshold)
    {
        ROS_INFO("WAY_POINT PASSING");
        ROS_INFO("NEXT MOVE PLAN");

        point_number++;
    }
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "goal_4_waypoint");
    ros::NodeHandle nh;
    ros::Publisher pub_pose;
    ros::Time tmp_time = ros::Time::now();
    nh.setParam("waypoint_area_threshold", 0.2);
    
    ros::Subscriber sub_key = nh.subscribe("awsiot_to_ros", 1,  goal_key_Callback);
    //ros::Subscriber sub_key = nh.subscribe("goal_key", 1,  goal_key_Callback);
    ros::Subscriber sub_pos = nh.subscribe("odom", 100,  posi_Callback);
    ros::Subscriber sub_goal = nh.subscribe("move_base/status", 100,  goal_reached_Callback);

    pub_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 2, true);

    geometry_msgs::PoseWithCovarianceStamped initPose;
    initPose.header.stamp = tmp_time; 
    initPose.header.frame_id = "map"; 
    initPose.pose.pose.position.x = 0;
    initPose.pose.pose.position.y = 0;
    initPose.pose.pose.position.z = 0;
    initPose.pose.pose.orientation.w = 1;

    vector<vector<string>> waypoint_read = 
    {
        {"1.682101","0.298596","0.080469","0.996750"},
        {"2.293195","0.385175","0.052547","0.998611","goal"},
        {"4.433884","0.630851","0.322288","0.946634"},
        {"4.545833","1.225776","0.380812","0.924644"},
        {"5.062553","1.350851","0.999918","-0.012198","goal"},
        {"3.380071","1.284730","0.987302","0.158811"},
        {"3.033352","1.867196","0.950085","0.311968","goal"},
        {"1.938033","2.178706","0.964381","0.264490"},
        {"0.879312","2.512087","-0.986923","-0.161146","goal"}
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

    int vec_size = waypoint_read.size();
    int point_number=0;
    int next_point_flag = 0;
    int goal_point_flag = 0;
    double area_threshold = 0.2;
    while(ros::ok())
    {  
        nh.getParam("waypoint_area_threshold", area_threshold);

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

                goal_point_flag = 0;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
