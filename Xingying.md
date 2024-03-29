# Xingying动捕系统手册
## 流程
### 主机配置

- 开启主机（IP: 10.1.1.198），打开Xingying程序

- （默认为实时模式）点击下方面板的摄像头，搜索到摄像头

- 点击播放按钮，开始发布定位信息。

### 从机配置

- 安装`vrpn_client_ros`
    ```bash
    sudo apt install ros-noetic-vrpn-client-ros
    ```

- 运行`vrpn_client_ros`，注意将launch文件中的IP地址设置为主机地址

- 环境变量ROS_MASTER_URI按需设置

- 此时可以通过话题订阅到定位信息

## 创建刚体

- 将机器人放置在坐标原点，面朝X轴正方向（黑板所处方向为Y轴正方向，Z轴朝上，根据右手法则确定X轴）。

- 冻结帧，按住`ctrl`键逐个选定标记点，右键创建刚体

- 命名后创建成功，进入后处理模式

- 添加刚刚新建的刚体，在右侧选中segment->Segment_1，将`x(y,z) offset`设置为0

- 点击右下方`文件`->`保存模型`，覆盖保存

- 回到实时模式，可以看到刚体与坐标原点重合

- p.s. 刚体角度默认为创建时的全局坐标系Y轴正方向，因此不需要调整。