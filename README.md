# GeRotMini

you can follow these stepsin  to complete raspiberry cm4 image burning;
open the raspiberrConfig(The folder contains a Instructional Videos, as well as Raspberry Pi burn-in software and instructional videos.)
First, download the two software in the file, and then check your cm4. You will find a small dial on it. Move it to the on position, connect it to the computer with a data cable, use the rpiboot downloaded "boot" to find your cm4, and then use another image burning software to burn it
by the way, you can visit https://www.raspberrypi.com/software/ to download the lastest image file.

目前仅支持的树莓派系统版本 Raspbian GNU 10 (buster)

使用如下命令查看系统版本
```
cat /etc/os-release
```
## 检查环境
### 1.wiringpi
```
gpio -v
```
### 2.ttyAMA0端口
```
ls -l /dev
```
确定serial0是否连接至ttyAMA0
### 3.各种库

## 环境配置
### 1.wiringpi
http://wiringpi.com/download-and-install/
```
sudo apt-get install wiringpi
```

If could not install, try the following method. https://blog.csdn.net/weixin_42194402/article/details/123764537
```
cd /tmp
wget https://project-downloads.drogon.net/wiringpi-latest.deb
sudo dpkg -i wiringpi-latest.deb
```


### 2.ttyAMA0端口
https://blog.csdn.net/sinat_37939098/article/details/119344651

(1)图形界面

raspberry pi configuration-interfaces

（sudo raspi-config）

serial console --disable

serial port --enable

I2C --enable


（2）命令行
```
sudo vim /boot/config.txt
```

在末尾加入
dtoverlay=pi3-miniuart-bt

### 3.Dynamixel API
https://github.com/bishopAL/GeRot/tree/master/API/dynamixel_cpp%20Ver2.0

```
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git
cd DynamixelSDK/c++/build/linux_sbc
```

E: Makefile:57: *** missing separator (did you mean TAB instead of 8 spaces?)

修改Makefile第57行与上一行的空格为回车换行

```
make
sudo make install
```

### 4.Eigen库
```
sudo apt-get install libeigen3-dev
```
OR

自己安装Eigen库 http://eigen.tuxfamily.org/index.php?title=Main_Page

下载安装位置	/usr/include/
```
cd /usr/include/eigen3.4.x
mkdir build
cd build
cmake ..
make
sudo make install
```

#软链接
```
sudo ln -s /usr/include/eigen3.4.x/Eigen /usr/include/Eigen
sudo ln -s /usr/include/eigen3.4.x/unsupported /usr/include/unsupporte
```



### 5.qp库
```
git clone https://github.com/coin-or/qpOASES.git
cd qpOASES
mkdir build && cd build
cmake ..
sudo make
sudo make install
```



# GebotMini
