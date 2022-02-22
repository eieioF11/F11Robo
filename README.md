# F11Robo
対向二輪ロボットを動作させるROSアプリケーション用リポジトリ\
[対向二輪ロボット情報と基板のファーム](https://github.com/eieioF11/ESP32)
# 環境構築
ubuntuバージョン: 18.04\
ROSバージョン: melodic\
[ROSインストール参考サイト](https://qiita.com/applepieqiita/items/4cd57e337d8756c8db44)
## rosserial
```bash
sudo apt install ros-melodic-rosserial-python
```
## キーボード操作
```bash
sudo apt-get install ros-melodic-teleop-twist-keyboard
```
## gmapping
```bash
cd ~/catkin_ws/src
git clone https://github.com/ros-perception/slam_gmapping
git clone https://github.com/ros-perception/openslam_gmapping.git
cd ~/catkin_ws
catkin_make
```
## navigation
```bash
sudo apt install libbullet-dev libsdl-image1.2-dev libsdl-dev
cd ~/catkin_ws/src
git clone -b melodic-devel https://github.com/ros-planning/navigation.git
git clone -b ros1 https://github.com/ros-planning/navigation_msgs.git
git clone -b melodic-devel https://github.com/ros/geometry2.git
cd ~/catkin_ws
catkin_make
```
## xtion pro live
```bash
cd catkin_ws/src
git clone https://github.com/ros-drivers/openni2_camera.git
cd ../
catkin_make
sudo apt-get install ros-melodic-rgbd-launch
sudo apt install ros-melodic-pointcloud-to-laserscan
```
## robot_localization
[ドキュメント](http://docs.ros.org/en/kinetic/api/robot_localization/html/index.html)
```bash
sudo apt-get install -y ros-melodic-robot-localization
```
## jsk visualization
[ドキュメント](https://jsk-visualization.readthedocs.io/en/latest/)
```bash
sudo apt-get install -y ros-melodic-jsk-visualization
```
# ロボット側のノード
ロボット側ではないPCでroscoreを実行したあと以下のコマンドを実行する
```bash
roslaunch F11Robo F11Robo_core.launch
```
Xtion Pro Liveを使用する場合
```bash
roslaunch F11Robo rsj_pointcloud_to_laserscan.launch
```
カメラを使用する場合は以下のコマンドを実行する
```bash
rosrun cv_camera cv_camera_node _property_0_code:=404 _property_0_code:=1
```
# キーボード操作
以下のコマンドを実行するとキーボードからロボットを操作できる。
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
gazeboで動かす場合
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/F11Robo/diff_drive_controller/cmd_vel
```
# Gazebo
```bash
roslaunch F11Robo gazebo.launch
```
![gazebo](/image/gazebo.png)
# mapping
以下のコマンドを入力して実行する
```bash
roslaunch F11Robo gmapping.launch
```
gazeboで動かす場合
```bash
roslaunch F11Robo gmapping_gazebo.launch
```
## マップ保存
```bash
rosrun map_server map_saver -f ファイル名
```
Ex. ファイル名がmapのとき
```bash
rosrun map_server map_saver -f  map
```
以下のようなマップが保存される(画像はmapディレクトリにあるサンプル地図)
![map](/image/mymap1.jpg)
# navigation
以下のコマンドを入力して起動する
```bash
roslaunch F11Robo navigation.launch
```
マップファイルを指定して起動する場合(以下はマップファイル名がmap.yamlのとき)
```bash
roslaunch F11Robo navigation.launch map_file:=$HOME/map.yaml
```
gazeboで動かす場合
```bash
roslaunch F11Robo navigation_gazebo.launch
```
navigation実行時のノード図
![node](/image/rosgraph.png)
# Path Planning & Path Following
自作のナビゲーションノードで
Path PlanningのアルゴリズムにはA*、Path FollowingのアルゴリズムにはPurePursuitを使用した
```bash
roslaunch F11Robo navi.launch map_file:=$HOME/map.yaml
```
gazeboで動かす場合
```bash
roslaunch F11Robo navi.launch gazebo:=true
```
# Goal generator
navigationを起動後以下のコマンドを実行する
```bash
rosrun  F11Robo F11Robo_node
```
![goal](/image/goal.png)
# 開発メモ
## ラズパイでcatkin_make_isolatedが実行できない場合
```bash
sudo chown $USER: -R /home/pi/ros_catkin_ws
```
## ROS 複数台接続方法
メインPCのIPが192.168.0.117のとき
```bash
export ROS_IP=192.168.0.117
roscore
```
接続するPCのIPが192.168.0.111のとき
```bash
export ROS_MASTER_URI=http://192.168.0.117:11311
export ROS_IP=192.168.0.111
```
## bashrcへの記入
geditがインストールされている場合。gedit以外のエディタを使用してもよ。
```bash
gedit ~/.bashrc
```
以下の内容を記載すると開くたびに上記のexportコマンドが実行される。
(ホスト名がそれぞれspectref11,F11のときの場合）
```bash
export ROS_MASTER_URI=http://spectref11.local:11311
export ROS_IP=F11.local
```
## ROS serial
デバイスを確認
```bash
ls -l /dev/serial/by-id/
```
デバイス名が/dev/ttyUSB0のときのROSserialの実行
```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=115200
```
### デバイス名での接続
#### デバイスを接続する前
```bash
sudo adduser LOGIN_NAME dialout
tail -f /var/log/syslog | grep tty
```
#### デバイスを差したあと
F11Roboのシリアル変換ICの情報を確認　(vendor ID(ID_VENDOR_ID) product ID(ID_MODEL_ID))
```bash
udevadm info -q all -n /dev/ttyUSB0
```
vendor ID = 0403 product ID = 6015のときのシリアルナンバーの確認
```bash
sudo lsusb -d 0403:6015 -v | grep iSerial
```
F11Robo.rulesの作成
```bash
cd /etc/udev/rules.d/
sudo gedit F11Robo.rules
```
F11Robo.rulesの中身
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="vendor ID", ATTRS{idProduct}=="product ID",ATTRS{serial}=="シリアルナンバー", MODE:="0666", GROUP:="dialout",  SYMLINK+="F11Robo"
```
例
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015",ATTRS{serial}=="DN048E5K", MODE:="0666", GROUP:="dialout",  SYMLINK+="F11Robo"
```
udevのリロード
```bash
sudo udevadm control --reload-rules
sudo service udev reload
```
確認
```bash
ls /dev/F11Robo
```
上記のコマンドでF11Roboが表示されれば成功
## カメラを使用するためのノードインストール方法(ラズパイ)
ワークスペースのsrcに移動して以下のコマンドを実行
```bash
git clone https://github.com/ros-perception/image_common.git
git clone https://github.com/ros-perception/vision_opencv.git
git clone https://github.com/OTL/cv_camera.git
cd ../
catkin_make_isolated
```
## Raspberry Pi4 メモ
### 時刻手動設定
ラズパイ単体ではなく外部のPCと接続してロボットを動作させる場合時刻の同期が必要になる
```bash
sudo date --set='YYYY/MM/DD hh:mm:ss'
sudo date MMDDhhmmYYYY.ss
```
Ex. 2021年5月1日 1時30分30秒
```bash
sudo date --set='2021/05/01 01:30:30'
sudo date 050101302021.30
```
