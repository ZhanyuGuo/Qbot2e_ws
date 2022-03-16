# 教程

## 配置环境

安装[Ubuntu](https://blog.csdn.net/FRIGIDWINTER/article/details/122888036)和[ROS](http://wiki.ros.org/cn/ROS/Installation). (Optional: OpenCV `pip install opencv-python`) (直接使用实验室的台式电脑, 可以免去配置环境的繁琐步骤. 且台式电脑通过网线连接路由器, 不需要下述连接Wi-Fi的步骤)

## 连接Wi-Fi

连接实验室Wi-Fi, 名称: `Quanser_UVS`, 密码: `UVS_wifi`.

## 下载并编译代码

```bash
# Download the code
cd ~
git clone https://gitee.com/guo_zhanyu/Qbot2e_ws.git Qbot2e_ws
cd Qbot2e_ws/

# Build the code
catkin_make

# Config the environment
echo "source ~/Qbot2e_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc # Or reload the teminal

# Check
echo $ROS_PACKAGE_PATH  # Qbot2e_ws is among them.
```

## 运行

### 连接小车远程控制.

```bash
# Method 1: password required
ssh pi@192.168.2.xxx  # password: raspberry

# Method 2: enter the password once
ssh keygen  # ENTER ENTER ENTER...
ssh-copy-id pi@192.168.2.xxx  # Enter the password
ssh pi@192.168.2.xxx  # No password required

# Ctrl + D to exit.
```

### 修改小车master

```bash
# In local terminal
ifconfig  # MASTER_IP = 192.168.2.xxx
nano ~/.bashrc
# Change ROS_IP to MASTER_IP
source ~/.bashrc

# In remote terminal
ifconfig  # ROBOT_IP = 192.168.2.XXX
nano ~/.zshrc
# Or nano ~/.bashrc

# Change ROS_MASTER_URI to http://<MASTER_IP>:11311
# Change ROS_IP to ROBOT_IP
# Ctrl + X to save and exit.

source ~/.zshrc # Or reload remote teminal
```

### 运行小车底盘

```bash
# In remote terminal
roslaunch kobuki_node minimal.launch  # Basic driver
roslaunch kobuki_keyop keyop.launch  # Keyboard operation
```

### 跟踪, 镇定和围捕

打开底盘后, 只需要在本地终端运行相应脚本即可. 难点在于代码中ROS的操作.

举例: 运行`src/stabilization/scripts/stabilization_demo.py`

```bash
# In remote terminal
roslaunch kobuki_node minimal.launch

# In local terminal
rosrun stabilization stabilization_demo.py
```

