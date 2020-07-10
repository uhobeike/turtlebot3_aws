#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iterator>
#include <string>
#include <algorithm>

using std::cin;
using std::iswdigit;
using std::string;
using std::stoi;

int sub_flag;
int index_cnt;
string key = "0";

string getKey()
{
    ROS_INFO("go Comand Only ,but 'q' is shutdown");

    while (ros::ok())
    {
        cin >> key;
        
        if(key == "go") break;

        else if(key == "q") ros::shutdown();
    }
    return key;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "goal_key_send");
    ros::NodeHandle nh;
    ros::Publisher pub;

    pub = nh.advertise<std_msgs::String>("goal_key", 1, true);

    ros::Rate loop_rate(10);//10Hz

    while (ros::ok())
    {
        std_msgs::String msg_key;
        msg_key.data = getKey();

        if(msg_key.data == "go") pub.publish(msg_key);
    
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
