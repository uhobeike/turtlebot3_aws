#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <string>
#include <iterator>
#include <fstream> 
#include<stdio.h>
#include<stdlib.h>

using std::vector;
using std::string;
using std::stoi;
using std::to_string;
using std::ofstream;
using std::cout;
using std::endl;

class waypoint_rviz
{
  private:
    void way_point_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
    
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;

    geometry_msgs::PoseArray pose_array_;
    geometry_msgs::Pose pose_rviz_;

  public:
    waypoint_rviz();
    ~waypoint_rviz(){};
};

waypoint_rviz::waypoint_rviz() : nh_("")
{
    subscriber_ = nh_.subscribe("waypoint_set", 1, &waypoint_rviz::way_point_Callback, this);
    publisher_ = nh_.advertise<geometry_msgs::PoseArray>("waypoint", 1, true);
}

void waypoint_rviz::way_point_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
{       
    pose_array_.header.stamp = ros::Time::now(); 
    pose_array_.header.frame_id = "map";
    
    pose_rviz_.position.x = pose->pose.pose.position.x;
    pose_rviz_.position.y = pose->pose.pose.position.y;
    pose_rviz_.position.z = 0.2;
    pose_rviz_.orientation.z = pose->pose.pose.orientation.z;
    pose_rviz_.orientation.w = pose->pose.pose.orientation.w;

    pose_array_.poses.push_back(pose_rviz_);

    publisher_.publish(pose_array_);
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "waypoint_rviz_set");

    waypoint_rviz node;

    ros::spin();
    
    return 0;
}
