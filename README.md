# turtlebot3_aws(takagon_delivery_challenge :notes:)
[![Build Status](https://travis-ci.org/uhobeike/turtlebot3_aws.svg?branch=takagon_delivery_challenge)](https://travis-ci.org/uhobeike/turtlebot3_aws)
### パッケージ概要
takagon_delivery_challengeを行うことでturtlebot3及び自律移動ロボットにおけるパラメータを
学習することを目的としている。

開催期間(2020年7月17〜2020年9月下旬まで)

### パッケージ内容 
パラメータについてはデフォルトです。ワールドだけ追加を行ったぐらいで他に変更は行ってません。

このパッケージに適宜修正を各自で行ってもらいます。

### :dizzy:大会競技内容:dizzy:

マッピングを行うと以下のようなマップ（pgm）(yaml)を手に入れると思います。それを使用してnavigationを
行います。Sがスタート地点でGがゴール地点です。完走できるように各自で、ゴール用node(c++)の作成とパラメータ調整を行いましょう。オレンジの線を通るようにしてください。質問はslackで受け付けています。

![エビフライトライアングル](https://i.gyazo.com/7811cd6dd3add602cf3b1e9f5225a2c6.png "コースマップ")

### 導入方法 :sunglasses:
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

```
~$ mkdir -p ~/turtlebot_ws/src && cd ~/turtlebot_ws/src
~/turtlebot_ws/src$ git clone -b takagon_delivery_challenge https://github.com/uhobeike/turtlebot3_aws.git
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
___
### 実践編 :alien:

1.マッピング
```
~$ roslaunch turtlebot3_gazebo turtlebot3_aws.launch
~$ roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
~$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
~$ rosrun map_server map_saver -f ~/map
```
2.ナビゲーション(指定位置)
```
~$ roslaunch turtlebot3_gazebo turtlebot3_aws.launch
~$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
これ以外にゴールを指定するプログラム必要(作ってみましょー)
```
### とても参考になる本 :heart_eyes:

[ROSロボットプログラミングバイブル（パラメータについて書いてある）（丸善でも売ってる）](https://www.amazon.co.jp/ROS%E3%83%AD%E3%83%9C%E3%83%83%E3%83%88%E3%83%97%E3%83%AD%E3%82%B0%E3%83%A9%E3%83%9F%E3%83%B3%E3%82%B0%E3%83%90%E3%82%A4%E3%83%96%E3%83%AB-%E8%A1%A8-%E5%85%81%E3%80%93/dp/4274221962)
### 参考になるサイトたち :kissing_heart:
* パラメータについて

[狭いところ通りたい場合](http://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#tuning-guide)

[turtlebotのナビゲーションスタックの設定](http://wiki.ros.org/ja/turtlebot_navigation/Tutorials/Setup%20the%20Navigation%20Stack%20for%20TurtleBot)

[ROSのナビゲーションamclについて理解を深めてみる](https://sy-base.com/myrobotics/ros/ros-amcl/)

[ROS move baseでdwa local plannerを使ってみる](https://sy-base.com/myrobotics/ros/ros-dwa-local-planner/)

[ROSのナビゲーションmove_baseについて理解を深めてみる](https://sy-base.com/myrobotics/ros/ros-move_base/)

[ROS gmapping のパラメータ解説](https://sy-base.com/myrobotics/ros/gmapping/)

[パラメータを調整する](https://github.com/TukamotoRyuzo/rostest/wiki/%E3%83%91%E3%83%A9%E3%83%A1%E3%83%BC%E3%82%BF%E3%82%92%E8%AA%BF%E6%95%B4%E3%81%99%E3%82%8B)


* navigation(goalさせるためのプログラム)

[Sending Goals to the Navigation Stack](http://wiki.ros.org/ja/navigation/Tutorials/SendingSimpleGoals)


[ロボットプログラミングⅡ：第6週　簡単なゴール（ActionLib）！](https://demura.net/education/lecture/12372.html)

* Pub&Sub書き方例

[ROS講座03 Pub & Sub 通信](https://qiita.com/srs/items/26ca826802d07a9e3d4e)

* ros::spin()について

[【ROS】ros::spin()とros::spinOnce()の違い](http://lilaboc.work/archives/16182817.html)


* 以下のリンクより、マッピングデータ(ナビゲーションしたいだけの人用,yamlファイルなどが入手できます)

[マッピングデータほしィィ方〜〜〜](https://drive.google.com/drive/folders/1ZoOuWc71f-aDIaHJTL2VshTnQ7ywS9pz?usp=sharing)

* turtlebot3 e-manial

[turtlebot3_e-manual](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#ros-1-simulation)


:poop: :poop: :poop: :poop: :poop: :poop: :poop: :poop: :poop: :poop:  :poop: :poop: :poop: :poop: :poop: :poop: :poop: :poop: :poop: :poop:  :poop: :poop: :poop: :poop: :poop: :poop: :poop: :poop: :poop: :poop:  :poop: :poop: :poop: :poop: :poop: :poop: :poop: :poop: :poop: :poop: :poop: 