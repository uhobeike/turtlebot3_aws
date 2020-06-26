# turtlebot3_aws

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

実践編

1.マッピング
```
~$ roslaunch turtlebot3_gazebo turtlebot3_aws.launch
~$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
~$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
~$ rosrun map_server map_saver -f ~/map
```
2.ナビゲーション
```
~$ roslaunch turtlebot3_gazebo turtlebot3_aws.launch
~$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
~$ rosrun goal_send 4goal_send
~$ rosrun goal_send goal_control_key
```
マッピングデータ(ナビゲーションしたいだけの人用,yamlファイルなどが入手できます)

https://drive.google.com/drive/folders/1ZoOuWc71f-aDIaHJTL2VshTnQ7ywS9pz?usp=sharing

turtlebot3 e-manial

http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#ros-1-simulation


