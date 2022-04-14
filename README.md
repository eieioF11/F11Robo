# F11Robo
対向二輪ロボットを動作させるROSアプリケーション用リポジトリ\
[対向二輪ロボット情報と基板のファーム](https://github.com/eieioF11/ESP32)
# 環境構築
ubuntuバージョン: 20.04\
ROSバージョン: noetic\
[ROSインストール参考サイト](https://qiita.com/take4eng/items/70f167320ede46e4139c)
## ssh
### インストール
```bash
sudo apt install openssh-server
```
### サーバーの実行
```bash
sudo systemctl enable ssh
sudo systemctl start ssh
```
### 自動起動設定
```bash
sudo systemctl enable ssh
```
## Serial
```bash
sudo apt install ros-noetic-rosserial-python
```
## LiDAR
```bash
cd ~/catkin_ws/src
git clone https://github.com/EAIBOT/ydlidar.git
cd ~/catkin_ws
catkin_make
```
```bash
cd ~/catkin_ws/src/ydlidar/startup
chmod 777 *
sudo sh initenv.sh
```
scan topicがRvizで表示されない\\
ydlidar/src/ydlidar_node.cppを以下のように修正
```bash
scan_msg.scan_time = scan.config.scan_time;
scan_msg.time_increment = scan.config.time_increment;
```
↓
```bash
//scan_msg.scan_time = scan.config.scan_time;
//scan_msg.time_increment = scan.config.time_increment;
```
## Xtion Pro Live
```bash
cd catkin_ws/src
git clone https://github.com/ros-drivers/openni2_camera.git
cd ../
catkin_make
sudo apt-get install ros-noetic-rgbd-launch
```
## キーボード操作
```bash
sudo apt-get install ros-noetic-teleop-twist-keyboard
```
## gmapping
```bash
sudo apt-get install ros-noetic-gmapping
```
## navigation
```bash
sudo apt install libbullet-dev libsdl-image1.2-dev libsdl-dev
sudo apt-get install ros-noetic-navigation
```
## robot_localization
[ドキュメント](http://docs.ros.org/en/kinetic/api/robot_localization/html/index.html)
```bash
sudo apt-get install -y ros-noetic-robot-localization
```
## jsk visualization
[ドキュメント](https://jsk-visualization.readthedocs.io/en/latest/)
```bash
sudo apt-get install -y ros-noetic-jsk-visualization
```
# ロボット側のノード
ロボット側ではないPCでroscoreを実行したあと以下のコマンドを実行する
```bash
roslaunch F11Robo F11Robo_core.launch
roslaunch openni2_launch openni2.launch
```
ssh
```bash
ssh ubuntu@f11robo.local
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
## bashrcへの記入
geditがインストールされている場合。gedit以外のエディタを使用してもよ。
```bash
gedit ~/.bashrc
```
以下の内容を記載すると開くたびに上記のexportコマンドが実行される。
(ホスト名がそれぞれspectref11,F11のときの場合）
```bash
export ROS_MASTER_URI=http://f11.local:11311
export ROS_IP=f11robo.local
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
