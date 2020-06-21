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

新しいワークスペースを作成しsrc内にcloneを行いcatkin_makeにてコンパイルを行います。
```
~$ mkdir -p ~/turtlebot_ws/src && cd ~/turtlebot_ws/src
~/turtlebot_ws/src$ git clone https://github.com/uhobeike/turtlebot3_aws.git
~/turtlebot_ws/src$ cd ~/turtlebot_ws
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

開発用ブランチを作成しましょう

```
git branch develop
```
turtlebot3 e-manial

http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#ros-1-simulation

