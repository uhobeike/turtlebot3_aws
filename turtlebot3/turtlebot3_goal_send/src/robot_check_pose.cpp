#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

void posi_Callback(const nav_msgs::Odometry::ConstPtr &msg){
  ROS_INFO("\n x: [%lf] \n y: [%lf] \n w: [%lf] \n", msg->pose.pose.position.x, msg->pose.pose.position.y,msg->pose.pose.orientation.w);

}  

int main(int argc, char** argv){

    ros::init(argc, argv, "robot_check_pose");

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("odom", 10,  posi_Callback);

    ros::spin();
}
