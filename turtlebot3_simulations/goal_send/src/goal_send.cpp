#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  ros::Publisher  pub;

  ros::init(argc, argv, "simple_goal");

  ros::NodeHandle nh;
  pub= nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 2, true);

  // 時間の取得
  ros::Time tmp_time = ros::Time::now();
  //create msg
  geometry_msgs::PoseWithCovarianceStamped initPose;

  // 初期位置の設定 turtlebot3_worldの初期位置が(-2.0, -0.5)なのでそれに設定
  initPose.header.stamp = tmp_time;  //  時間
  initPose.header.frame_id = "map";  //  フレーム
  initPose.pose.pose.position.x = 0;
  initPose.pose.pose.position.y = 0;
  initPose.pose.pose.position.z = 0;
  initPose.pose.pose.orientation.w = 1.0;

  // パブリッシュ amclパッケージに初期位置を送る
  pub.publish(initPose);
  
  // アクションクライアンを作成。1番目の引数は接続するアクションサーバー名。
  // アクションサーバーが立ち上がっていないとだめ。
  // ２番目の引数はtrueならスレッドを自動的に回す(ros::spin()。
  MoveBaseClient ac("move_base", true);
  // アクションサーバーが起動するまで待つ。引数はタイムアウトする時間(秒）。
  // この例では５秒間待つ(ブロックされる)
  while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("The server comes up");
  move_base_msgs::MoveBaseGoal goal;
  // base_link座標系（ロボット座標系)
  goal.target_pose.header.frame_id = "map";
  // 現在時刻                                                                       
  goal.target_pose.header.stamp = ros::Time::now();

  while (ros::ok()) {
    // ROSではロボットの進行方向がx座標、左方向がy座標、上方向がz座標
    goal.target_pose.pose.position.x =  5.4;
    goal.target_pose.pose.position.y =  1.1;
    goal.target_pose.pose.orientation.w = 1; 

   
    // サーバーにgoalを送信
    ac.sendGoal(goal);


  }
  return 0;
}
