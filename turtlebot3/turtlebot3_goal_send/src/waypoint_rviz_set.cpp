#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <string>
#include <iterator>
#include <fstream> 
#include <stdio.h>
#include <stdlib.h>

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
    void waypoint_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
    void control_Callback(const std_msgs::StringConstPtr& command);

    void waypoint_csv_(vector<string>& posi_set, vector<vector<string>>& csv_array, uint16_t &waypoint_number);
    void way_point_remove_(vector<vector<string>>& waypoint_remove, geometry_msgs::PoseArray& pose_array, uint16_t &waypoint_number);
    void way_point_goal_set_(vector<vector<string>>& waypoint_goal, uint16_t &waypoint_number);
    void way_point_corner_set_(vector<vector<string>>& waypoint_corner, uint16_t &waypoint_number);
    void finish_and_file_write_waypoint_(vector<vector<string>>& waypoint_file_write, uint16_t &waypoint_number);
    
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    ros::Subscriber subscriber_set_, subscriber_control_;

    geometry_msgs::PoseArray pose_array_;
    geometry_msgs::Pose pose_rviz_;

    vector<string> posi_set_;
    vector<vector<string>> csv_array_;

    uint16_t waypoint_number_;
    uint16_t waypoint_index_;

  public:
    waypoint_rviz();
    ~waypoint_rviz(){};
};

waypoint_rviz::waypoint_rviz() : nh_("")
{
  subscriber_set_ = nh_.subscribe("waypoint_set", 1, &waypoint_rviz::waypoint_Callback, this);
  subscriber_control_ = nh_.subscribe("waypoint_control", 1, &waypoint_rviz::control_Callback, this);
  publisher_ = nh_.advertise<geometry_msgs::PoseArray>("waypoint", 1, true);

  posi_set_ = 
  {
    "0","0","0","0"
  };

  waypoint_number_ = 0;
}

void waypoint_rviz::waypoint_Callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose)
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

  posi_set_.at(0) = to_string(pose->pose.pose.position.x);
  posi_set_.at(1) = to_string(pose->pose.pose.position.y);
  posi_set_.at(2) = to_string(pose->pose.pose.orientation.z);
  posi_set_.at(3) = to_string(pose->pose.pose.orientation.w);

  waypoint_csv_(posi_set_, csv_array_, waypoint_number_);
}

void waypoint_rviz::control_Callback(const std_msgs::StringConstPtr& command)
{
  if(command->data == "remove")
  {
    way_point_remove_(csv_array_, pose_array_, waypoint_number_);
  }

  else if(command->data == "goal")
  {
    way_point_goal_set_(csv_array_, waypoint_number_);
  }

  else if(command->data == "corner")
  {
    way_point_corner_set_(csv_array_, waypoint_number_);
  }

  else if(command->data == "finish")
  {
    finish_and_file_write_waypoint_(csv_array_, waypoint_number_);
  }
}

void waypoint_rviz::waypoint_csv_(vector<string>& posi_set, vector<vector<string>>& csv_array, uint16_t &waypoint_number)
{
  csv_array.push_back(posi_set);
  waypoint_number++;

  cout << csv_array.size() << endl;
  cout << waypoint_number_ << endl;
}

void waypoint_rviz::way_point_remove_(vector<vector<string>>& waypoint_remove, geometry_msgs::PoseArray& pose_array, uint16_t &waypoint_number)
{
  ROS_INFO("way_point_remove");

  if(0 < waypoint_number)
  {
    waypoint_number--;
    waypoint_remove.resize(waypoint_number);
    pose_array_.poses.resize(waypoint_number);
  }
  cout << csv_array_.size() << endl;
  cout << waypoint_number << endl;

  publisher_.publish(pose_array_);
}

void waypoint_rviz::way_point_goal_set_(vector<vector<string>>& waypoint_goal, uint16_t &waypoint_number)
{
  uint16_t array_number = waypoint_number - 1;
  waypoint_goal[array_number].push_back("goal");
}

void waypoint_rviz::way_point_corner_set_(vector<vector<string>>& waypoint_corner, uint16_t &waypoint_number)
{
  uint16_t array_number = waypoint_number - 1;
  waypoint_corner[array_number].push_back("corner");
}

void waypoint_rviz::finish_and_file_write_waypoint_(vector<vector<string>>& waypoint_file_write, uint16_t &waypoint_number)
{
    ROS_INFO("finish_and_file_write_waypoint q(^_^)p");

    char *c = getenv("HOME");
    string HOME = c; 
    ofstream f_w(HOME + "/waypoint.csv",std::ios::app);

    for (auto it_t = waypoint_file_write.begin(); it_t != waypoint_file_write.end(); ++it_t) 
    {
        for (auto it = (*it_t).begin(); it != (*it_t).end(); ++it) 
        {
            f_w << *it << ",";
            cout << *it << endl;
        } 

        f_w << endl;
    }
    
    f_w.close();

    ros::shutdown();
}

int main(int argc, char** argv)
{   
  ros::init(argc, argv, "waypoint_rviz_set");

  waypoint_rviz node;

  ros::spin();
  
  return 0;
}
