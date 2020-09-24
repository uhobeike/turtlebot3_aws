# turtlebot3_aws

# takagon_delivery_challenge開催中で〜す:satisfied:

# [開催場所](https://github.com/uhobeike/turtlebot3_aws/tree/takagon_delivery_challenge):feet::feet:

### パッケージ概要
AWS Robot Delivery Challengeのためにturtlebot3のパッケージを使用して色々実装していき、戦うためのパッケージ。

### パッケージ内容
こちらは使用しているワークスペースのsrcにcloneして使用するソースコードになっています。


### 導入方法
下記のようにapt-getで必要なパッケージをインストールします。
```
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard \
ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan \
ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server \
ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server \
ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport \
ros-melodic-rqt-image-view ros-melodic-navigation

```

新しいワークスペースを作成し、src内にcloneを行いcatkin_makeにてコンパイルを行います。

＊二度catkin_makeを行う必要があります。それはメッセージファイル(自作)を生成するのに、一回分行うためです。

おそらく何か方法があると思いますが、今のところはcatkin_make2回でお願いします。
```
~$ mkdir -p ~/turtlebot_ws/src && cd ~/turtlebot_ws/src
~/turtlebot_ws/src$ git clone -b develop https://github.com/uhobeike/turtlebot3_aws.git
~/turtlebot_ws/src$ cd ~/turtlebot_ws
~/turtlebot_ws$ catkin_make
~/turtlebot_ws$ catkin_make
~/turtlebot_ws$ source ~/turtlebot_ws/devel/setup.bash
```
ビルドがうまく行ったらテストスクリプトを試してみます
```
~$ export TURTLEBOT3_MODEL=burger
~$ roslaunch turtlebot3_gazebo turtlebot3_aws.launch
```

うまくコースとロボットを読み込めたらOKです。

.bashrcの最後に`source ~/turtlebot_ws/devel/setup.bash`と`export TURTLEBOT3_MODEL=burger`を追加しておくと便利です

追加し終わったら
```
~$ source ~/.bashrc
```
___
# 実践編

## 1.マッピング
```
~$ roslaunch turtlebot3_gazebo turtlebot3_aws.launch
~$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
~$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
~$ rosrun map_server map_saver -f ~/map
```
## 2.ナビゲーション(指定位置)
```
~$ roslaunch turtlebot3_gazebo turtlebot3_aws.launch
~$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
~$ rosrun goal_send 4goal_send
~$ rosrun goal_send goal_control_key
```
## 3.waypoint用 file生成方法
```
~$ roslaunch turtlebot3_gazebo turtlebot3_aws.launch
~$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
~$ rosrun goal_send waypoint_set
```
$HOME以下にwaypoint.csvが作られる

## 4.waypointを利用したnavigation
```
~$ roslaunch turtlebot3_gazebo turtlebot3_aws.launch
~$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
~$ rosrun turtlebot3_goal_send 4goal_waypoint_send
~$ rosrun turtlebot3_goal_send goal_waypoint_control_key
```

## 5.waypoint_navgationしきい値の変更方法
```
~$ roslaunch turtlebot3_gazebo turtlebot3_aws.launch
~$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
~$ rosrun turtlebot3_goal_send 4goal_waypoint_send
~$ rosrun turtlebot3_goal_send goal_waypoint_control_key
```
の後に
```
~$ rosparam list
```
を実行すると
```
/waypoint_area_threshold
```
が一番下に表示される（おそらく）
その値はwaypoint_navgationしきい値である(defalrt=0.2)
変更都度ビルドを行うのは面倒なので
```
~$ rosparam set /waypoint_area_threshold 1
```
のように変更を行う（goする前に）

## 6.rvizにてwaypointをセットする方法
```
~$ roslaunch turtlebot3_gazebo turtlebot3_aws.launch
~$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
~$ rosrun turtlebot3_goal_send control_waypoint_rviz
```
rosrunをすると以下のような操作一覧が表示される。

基本的な操作の流れとして

①sを押して、nodeの立ち上げ

②rviz上の右の方にある(以下写真の場所)の2d_pose_estimateのボタンを押しセットしていく

③コーナやゴールとして置きたかったら、cまたはgを押す

④間違えて作成したら、jで消す

⑤終わったら、fを押してhomeディレクトリにcsvファイルを作成する

![](https://i.gyazo.com/2f89af1e11addde05472ce07cb79bd01.png"操作一覧")

以上の操作により、以下のようにwaypointを設定できます。
![](https://i.gyazo.com/47c8cf93ff124f921bcca2185bee47d7.png"rviz")

### [yotubeリンク](https://youtu.be/eKGirAP-iAE)

___

### 以下のリンクより、マッピングデータ(ナビゲーションしたいだけの人用,yamlファイルなどが入手できます)

#### [障害物有り](https://drive.google.com/drive/folders/1ZoOuWc71f-aDIaHJTL2VshTnQ7ywS9pz?usp=sharing)

#### [障害物なし](https://drive.google.com/drive/folders/1b1o4oH_89qXoQqh7ODTk0VraAkTbX-jM?usp=sharing)

### turtlebot3 e-manial

#### http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#ros-1-simulation


