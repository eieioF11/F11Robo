#!/bin/bash
echo  'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6015", MODE:="0666", GROUP:="dialout",  SYMLINK+="F11Robo"' >/etc/udev/rules.d/F11Robo.rules
service udev reload
sleep 2
service udev restart