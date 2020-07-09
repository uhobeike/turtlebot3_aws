#include <ros/ros.h>
#include <std_msgs/String.h>
#include <goal_send_msgs/goal_vector.h>
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
    ROS_INFO("Number Only ,but 'q' is shutdown");
    ROS_INFO("Wait for numbers or 'q'(shutdown)");
    ROS_INFO("select goal number is %s now",key.c_str());

    while (ros::ok())
    {
        cin >> key;
        
        if(std::all_of(key.cbegin(), key.cend(), isdigit) ) break;

        else if(key == "q") ros::shutdown();
    }
    return key;
}

void goal_data_Callback(const goal_send_msgs::goal_vectorConstPtr& msg)
{
    int index_prt;
    string str;

    ROS_INFO("~~~please select goal number~~~");

    for (auto it = msg->data.begin(); it != msg->data.end(); ++it)
    {
        auto index = std::distance(msg->data.begin(), it);
        index_prt = (unsigned char) index;
        str = *it;
        ROS_INFO("%d is %s",index_prt,str.c_str());
    }

    ROS_INFO("~~~~~~~~~~~~~~~~~~~~~~~~~~");

    index_cnt = index_prt;
    sub_flag = 1;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "goal_key_send");
    ros::NodeHandle nh;
    ros::Publisher pub;

    ros::Subscriber sub = nh.subscribe("goal_data", 1,  goal_data_Callback);

    pub = nh.advertise<std_msgs::String>("goal_key", 1, true);

    ros::Rate loop_rate(10);//10Hz

    while (ros::ok())
    {
        if(sub_flag)
        {
            std_msgs::String msg_key;
            msg_key.data = getKey();

            if(stoi(key) <= index_cnt) pub.publish(msg_key);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
