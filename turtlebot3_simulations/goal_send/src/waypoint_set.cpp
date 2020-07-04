#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <string>
#include <iterator>
#include <fstream> 

using std::vector;
using std::string;
using std::stoi;
using std::to_string;
using std::ofstream;

int first_vec = 1;
int waypoint_nunber = 1;
vector<vector<string>> waypoint_set;
vector<string> posi_set = 
{
    "0","0","0","0"
};

void posi_Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO("pos[x: [%lf]  y: [%lf]],ori[z: [%lf]  w: [%lf]] ", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

    posi_set.at(0) = (to_string(msg->pose.pose.position.x) );
    posi_set.at(1) = (to_string(msg->pose.pose.position.y) );
    posi_set.at(2) = (to_string(msg->pose.pose.orientation.z) );
    posi_set.at(3) = (to_string(msg->pose.pose.orientation.w) );
}  

void finish_and_file_write_waypoint(vector<vector<string>>& waypoint_file_write)
{
    ROS_INFO("finish_and_file_write_waypoint");

    waypoint_set.resize(waypoint_nunber--);
    
    ofstream f_w("~/waypoint.csv",std::ios::app);
    
    for (auto it_t = waypoint_file_write.begin(); it_t != waypoint_file_write.end(); ++it_t) 
    {
        for (auto it = (*it_t).begin(); it != (*it_t).end(); ++it) 
        {
            string str;
            str = *it;
            ROS_INFO("%s",str.c_str());
        } 
    }
}

void goal_set(vector<vector<string>>& goal_set)
{
    ROS_INFO("goal_set");

    for (auto it_t = posi_set.begin(); it_t != posi_set.end(); ++it_t)
    {
        goal_set[waypoint_nunber-1].push_back(*it_t);
    }

    goal_set[waypoint_nunber-1].push_back("goal");
}

void way_point_add(vector<vector<string>>& waypoint_add)
{
    ROS_INFO("way_point_add");

    for (auto it_t = posi_set.begin(); it_t != posi_set.end(); ++it_t)
    {
        waypoint_add[waypoint_nunber-1].push_back(*it_t);
    }
}

void way_point_remove(vector<vector<string>>& waypoint_remove)
{
    ROS_INFO("way_point_remove");

    waypoint_remove[waypoint_nunber-1].pop_back();
}

void way_point_Callback(const std_msgs::StringConstPtr& str)
{   
    if(first_vec)
    {
        waypoint_set.emplace_back();
        first_vec = 0;
    }

    if(waypoint_nunber >= 1)
    {   
        if(str->data == "finish and file_write_waypoint") finish_and_file_write_waypoint(waypoint_set);
        else if(str->data == "goal_set") goal_set(waypoint_set);
        else if(str->data ==  "way_point_add") way_point_add(waypoint_set);
        else if(str->data ==  "way_point_remove") way_point_remove(waypoint_set);
        waypoint_nunber++;
        waypoint_set.resize(waypoint_nunber);
    }

    else waypoint_nunber = 1;
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "waypoint_set");
    ros::NodeHandle nh;

    ros::Subscriber sub_pos = nh.subscribe("odom", 100,  posi_Callback);
    ros::Subscriber sub_way = nh.subscribe("waypoint_set_flag", 1,  way_point_Callback); 
    
    ros::spin();
    
    return 0;
}
