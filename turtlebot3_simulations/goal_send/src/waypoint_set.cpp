#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <string>
#include <iterator>

using std::vector;
using std::string;
using std::stoi;
using std::to_string;

int x,y,z,w;
int waypoint_nunber;
vector<vector<string>> waypoint_set = 
{
    {"0"}
};
vector<string> posi_set;

void posi_Callback(nav_msgs::Odometry::ConstPtr& msg)
{
    //ROS_INFO("pos[x: [%lf]  y: [%lf]],ori[z: [%lf]  w: [%lf]] ", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);

    posi_set[0] = to_string(msg->pose.pose.position.x);
    posi_set[1] = to_string(msg->pose.pose.position.y);
    posi_set[2] = to_string(msg->pose.pose.orientation.x);
    posi_set[3] = to_string(msg->pose.pose.orientation.y);
}  

void finish_and_file_write_waypoint(vector<vector<string>>& waypoint_file_write)
{
int i;
}

void goal_set(vector<vector<string>>& goal_set)
{
    int posi_num = 0;

    for (auto it_t = posi_set.begin(); it_t != posi_set.end(); ++it_t)
    {
        goal_set[waypoint_nunber].push_back(*it_t);

        posi_num++;
    }

    goal_set[waypoint_nunber][posi_num+1] = "goal";
}

void way_point_add(vector<vector<string>>& waypoint_add)
{
    int posi_num = 0;

    for (auto it_t = posi_set.begin(); it_t != posi_set.end(); ++it_t)
    {
        waypoint_add[waypoint_nunber].push_back(*it_t);

        posi_num++;
    }
}

void way_point_remove(vector<vector<string>>& waypoint_remove)
{
    waypoint_remove[waypoint_nunber].pop_back();
}

void way_point_Callback(const std_msgs::StringConstPtr& str)
{
    if(waypoint_nunber >= 0){
        if(str->data == "finish and file_write_waypoint") finish_and_file_write_waypoint(waypoint_set);
        else if(str->data == "goal_set") goal_set(waypoint_set);
        else if(str->data ==  "way_point_add") way_point_add(waypoint_set);
        else if(str->data ==  "way_point_remove") way_point_remove(waypoint_set);

        waypoint_nunber++;
    }

    else waypoint_nunber = 0;
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
