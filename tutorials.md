# Qbot2e手册（20231115）
## 环境准备
安装[Ubuntu](https://blog.csdn.net/FRIGIDWINTER/article/details/122888036)和[ROS](http://wiki.ros.org/cn/ROS/Installation).

## 网络配置
- 连接实验室WiFi，名称: `IVCM-LAB`, 密码: `IVCM123123`.
- 登陆路由器地址: `10.1.1.1`.
- 开启一个Qbot, 查看得到其IP地址.

## 下载代码
```bash
# Download the code
cd ~/

git clone https://gitee.com/guo_zhanyu/Qbot2e_ws.git
# or git@github.com:ZhanyuGuo/Qbot2e_ws.git

cd Qbot2e_ws/

# Build the code
catkin_make

# Config the environment
echo "source ~/Qbot2e_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc # or reload the teminal

# Check
echo $ROS_PACKAGE_PATH  # Qbot2e_ws is among them.
```

## 运行环境
### 远程连接
```bash
# Method 1: password required
ssh pi@10.1.1.xxx  # Password: raspberry

# Method 2: enter the password once
ssh keygen  # input: ENTER ENTER ENTER
ssh-copy-id pi@10.1.1.xxx  # Enter the password
ssh pi@10.1.1.xxx  # No password required

# Ctrl + D to exit.
```

### 修改主从IP地址
#### 主机
```bash
# In local terminal
ifconfig  # MASTER_IP := 10.1.1.xxx
nano ~/.bashrc

# Change ROS_MASTER_URI to http://<MASTER_IP>:11311
# Change ROS_IP to MASTER_IP
# Ctrl + X to save and exit

source ~/.bashrc # or reload the teminal
```

#### 从机
```bash
# In remote terminal
ifconfig  # ROBOT_IP := 10.1.1.xxx
nano ~/.zshrc
# or nano ~/.bashrc

# Change ROS_MASTER_URI to http://<MASTER_IP>:11311
# Change ROS_IP to ROBOT_IP
# Ctrl + X to save and exit

source ~/.zshrc # or reload the teminal
# or source ~/.bashrc
```

### 运行底盘
```bash
# In remote terminal
roslaunch kobuki_node minimal.launch  # Basic driver
roslaunch kobuki_keyop keyop.launch   # Keyboard operation
```

当有多机需要同时启动底盘时, 需要增加命名空间, 通过:
```bash
roslaunch kobuki_node minimal.launch __ns:=/Qbot_xxx
```
或者
```bash
roslaunch kobuki_node qbot2e.launch
roslaunch kobuki_keyop qbot2e_keyop.launch
```
其中include了前面的launch文件并指定了命名空间为环境变量`$ROBOT_NAME`.

### 跟踪, 镇定和围捕
打开底盘后, 只需要在本地终端运行相应脚本即可. 难点在于代码中ROS的操作.

举例: 运行`src/stabilization/scripts/stabilization_demo.py`

```bash
# In remote terminal
roslaunch kobuki_node minimal.launch

# In local terminal
rosrun stabilization stabilization_demo.py
```

## 系统克隆
使用[Win32DiskImager](https://sourceforge.net/projects/win32diskimager/)进行读取和写入, [PiShrink](https://github.com/Drewsif/PiShrink)来压缩镜像.

1. Win32DiskImager读取SD卡到空白镜像中(命名为`*.img`).

2. 使用PiShrink压缩, 具体指令见其readme. (实测效果不是很明显, 从15G缩小到12G, 可以不浪费这个时间)

3. Win32DiskImager写入SD卡.
   
4. 更改系统变量.
