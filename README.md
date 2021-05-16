# F11Robo
##ROS 複数台接続方法
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
##ROS serial
デバイスを確認
```bash
ls -l /dev/serial/by-id/
```
デバイス名が/dev/ttyUSB0のときのROSserialの実行
```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=115200
```
###デバイス名での接続
デバイスを接続する前
```bash
sudo adduser LOGIN_NAME dialout
tail -f /var/log/syslog | grep tty
```
デバイスを差したあと\\
F11Roboのシリアル変換ICの情報を確認　(vendor ID(ID_VENDOR_ID) product ID(ID_MODEL_ID))
```bash
udevadm info -q all -n /dev/ttyUSB0
```
