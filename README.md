# F11Robo
#ROS 複数台接続方法
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
#ROS serial
デバイスを確認
```bash
ls -l /dev/serial/by-id/
```
デバイス名が/dev/ttyUSB0のときのROSserialの実行
```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=115200
```
デバイス名での接続
デバイスを接続する前
```bash
sudo adduser LOGIN_NAME dialout
tail -f /var/log/syslog | grep tty
```
デバイスを差したあと
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
