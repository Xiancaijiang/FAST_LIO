## 相关工作和扩展应用

**SLAM相关项目:**

1. [ikd-Tree](https://github.com/hku-mars/ikd-Tree): 用于3D kNN搜索的最先进的动态KD-Tree
2. [R2LIVE](https://github.com/hku-mars/r2live): 使用FAST-LIO作为LiDAR-IMU前端的高精度LiDAR-IMU-Vision融合系统
3. [LI_Init](https://github.com/hku-mars/LiDAR_IMU_Init): 鲁棒的实时LiDAR-IMU外参初始化和同步工具包
4. [FAST-LIO-LOCALIZATION](https://github.com/HViktorTsoi/FAST_LIO_LOCALIZATION): 集成重定位功能模块的FAST-LIO
5. [FAST-LIVO](https://github.com/hku-mars/FAST-LIVO) | [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2): 具有高计算效率、鲁棒性和像素级精度的最先进LiDAR-IMU-Visual里程计系统

**控制与规划相关项目:**

1. [IKFOM](https://github.com/hku-mars/IKFoM): 快速高精度流形卡尔曼滤波工具箱
2. [UAV Avoiding Dynamic Obstacles](https://github.com/hku-mars/dyn_small_obs_avoidance): FAST-LIO在机器人路径规划中的应用之一
3. [UGV Demo](https://www.youtube.com/watch?v=wikgrQbE6Cs): 在可微流形上进行轨迹跟踪的模型预测控制
4. [Bubble Planner](https://arxiv.org/abs/2202.12177): 使用后退走廊规划高速平滑四旋翼轨迹

<!-- 10. [**FAST-LIVO**](https://github.com/hku-mars/FAST-LIVO): Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry. -->

## FAST-LIO
**FAST-LIO** (快速LiDAR-IMU里程计) 是一个计算效率高且鲁棒的LiDAR-IMU里程计包。它使用紧耦合迭代扩展卡尔曼滤波器融合LiDAR特征点和IMU数据，可以在快速运动、噪声或杂乱环境中实现鲁棒的导航。我们的软件包解决了许多关键问题：
1. 用于里程计优化的快速迭代卡尔曼滤波器
2. 在大多数稳定环境中自动初始化
3. 并行KD-Tree搜索以减少计算量

## FAST-LIO 2.0 (2021-07-05 更新)
<!-- ![image](doc/real_experiment2.gif) -->
<!-- [![Watch the video](doc/real_exp_2.png)](https://youtu.be/2OvjGnxszf8) -->
<div align="left">
<img src="doc/real_experiment2.gif" width=49.6% />
<img src="doc/ulhkwh_fastlio.gif" width = 49.6% >
</div>

**相关视频:**  [FAST-LIO2](https://youtu.be/2OvjGnxszf8),  [FAST-LIO1](https://youtu.be/iYCY6T79oNU)

**系统流程:**
<div align="center">
<img src="doc/overview_fastlio2.svg" width=99% />
</div>

**新特性:**
1. 使用[ikd-Tree](https://github.com/hku-mars/ikd-Tree)进行增量建图，实现更快的速度和超过100Hz的LiDAR速率
2. 在原始LiDAR点云上进行直接里程计（scan to map）（可以禁用特征提取），实现更好的精度
3. 由于不需要特征提取，FAST-LIO2可以支持多种类型的LiDAR，包括旋转式（Velodyne, Ouster）和固态（Livox Avia, Horizon, MID-70）LiDAR，并且可以轻松扩展以支持更多LiDAR
4. 支持外部IMU
5. 支持ARM平台，包括Khadas VIM3、Nvidia TX2、Raspberry Pi 4B(8G RAM)

**相关论文**: 

[FAST-LIO2: Fast Direct LiDAR-inertial Odometry](doc/Fast_LIO_2.pdf)

[FAST-LIO: A Fast, Robust LiDAR-inertial Odometry Package by Tightly-Coupled Iterated Kalman Filter](https://arxiv.org/abs/2010.08196)

**贡献者**

[Wei Xu 徐威](https://github.com/XW-HKU)，[Yixi Cai 蔡逸熙](https://github.com/Ecstasy-EC)，[Dongjiao He 贺东娇](https://github.com/Joanna-HE)，[Fangcheng Zhu 朱方程](https://github.com/zfc-zfc)，[Jiarong Lin 林家荣](https://github.com/ziv-lin)，[Zheng Liu 刘政](https://github.com/Zale-Liu), [Borong Yuan](https://github.com/borongyuan)

<!-- <div align="center">
    <img src="doc/results/HKU_HW.png" width = 49% >
    <img src="doc/results/HKU_MB_001.png" width = 49% >
</div> -->

## 1. 环境要求
### 1.1 **Ubuntu** 和 **ROS**
**Ubuntu >= 16.04**

对于 **Ubuntu 18.04 或更高版本**，**默认**的PCL和Eigen就足以让FAST-LIO正常工作。

ROS    >= Melodic. [ROS安装指南](http://wiki.ros.org/ROS/Installation)

### 1.2. **PCL 和 Eigen**
PCL    >= 1.8,   参考[PCL安装指南](http://www.pointclouds.org/downloads/linux.html)

Eigen  >= 3.3.4, 参考[Eigen安装指南](http://eigen.tuxfamily.org/index.php?title=Main_Page)

### 1.3. **livox_ros_driver**
参考[livox_ros_driver安装指南](https://github.com/Livox-SDK/livox_ros_driver)

*备注:*
- 由于FAST-LIO必须首先支持Livox系列LiDAR，因此在运行任何FAST-LIO启动文件之前，必须安装并**source** **livox_ros_driver**
- 如何source？最简单的方法是将``` source $Livox_ros_driver_dir$/devel/setup.bash ```添加到``` ~/.bashrc ```文件的末尾，其中``` $Livox_ros_driver_dir$ ```是livox ros驱动工作空间的目录（如果完全按照livox官方文档操作，应该是``` ws_livox ```目录）


## 2. 构建
如果你想使用docker容器运行fastlio2，请先在机器上安装docker。
参考[Docker安装指南](https://docs.docker.com/engine/install/ubuntu/)
### 2.1 Docker容器
用户可以在linux中通过以下命令创建任意名称的脚本：
```
touch <your_custom_name>.sh
```
将以下代码放入``` <your_custom_name>.sh ```脚本中：
```
#!/bin/bash
mkdir docker_ws
# 在Docker中运行带GUI支持的ROS Kinetic的脚本

# 允许本地机器访问X server
xhost +local:

# 容器名称
CONTAINER_NAME="fastlio2"

# 运行Docker容器
docker run -itd \
  --name=$CONTAINER_NAME \
  --user mars_ugv \
  --network host \
  --ipc=host \
  -v /home/$USER/docker_ws:/home/mars_ugv/docker_ws \
  --privileged \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/etc/localtime:/etc/localtime:ro" \
  -v /dev/bus/usb:/dev/bus/usb \
  --device=/dev/dri \
  --group-add video \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --env="DISPLAY=$DISPLAY" \
  kenny0407/marslab_fastlio2:latest \
  /bin/bash
```
执行以下命令授予脚本可执行权限：
```
sudo chmod +x <your_custom_name>.sh
```
执行以下命令下载镜像并创建容器：
```
./<your_custom_name>.sh
```

*脚本说明:*
- 下面提供的docker run命令使用Docker Hub中的镜像创建一个带标签的容器。镜像下载时间可能因用户网络速度而异。
- 该命令还建立了一个名为``` docker_ws ```的新工作空间，它作为Docker容器和主机之间的共享文件夹。这意味着如果用户想运行rosbag示例，他们需要下载rosbag文件并将其放在主机上的``` docker_ws ```目录中。
- 随后，Docker容器内同名的文件夹将接收此文件。用户就可以轻松地在Docker中播放文件。
- 在这个例子中，我们将主机的网络与Docker容器共享。因此，如果用户执行``` rostopic list ```命令，无论是在主机上还是在Docker容器内运行，他们都会看到相同的输出。"
### 2.2 从源码构建
克隆仓库并执行catkin_make：

```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/hku-mars/FAST_LIO.git
    cd FAST_LIO
    git submodule update --init
    cd ../..
    catkin_make
    source devel/setup.bash
```
- 记得在构建前source livox_ros_driver（参考1.3 **livox_ros_driver**）
- 如果你想使用自定义构建的PCL，请将以下行添加到~/.bashrc
```export PCL_ROOT={CUSTOM_PCL_PATH}```
## 3. Directly run
Noted:

A. Please make sure the IMU and LiDAR are **Synchronized**, that's important.

B. The warning message "Failed to find match for field 'time'." means the timestamps of each LiDAR points are missed in the rosbag file. That is important for the forward propagation and backwark propagation.

C. We recommend to set the **extrinsic_est_en** to false if the extrinsic is give. As for the extrinsic initiallization, please refer to our recent work: [**Robust Real-time LiDAR-inertial Initialization**](https://github.com/hku-mars/LiDAR_IMU_Init).

### 3.1 For Avia
Connect to your PC to Livox Avia LiDAR by following  [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then
```
    cd ~/$FAST_LIO_ROS_DIR$
    source devel/setup.bash
    roslaunch fast_lio mapping_avia.launch
    roslaunch livox_ros_driver livox_lidar_msg.launch
```
- For livox serials, FAST-LIO only support the data collected by the ``` livox_lidar_msg.launch ``` since only its ``` livox_ros_driver/CustomMsg ``` data structure produces the timestamp of each LiDAR point which is very important for the motion undistortion. ``` livox_lidar.launch ``` can not produce it right now.
- If you want to change the frame rate, please modify the **publish_freq** parameter in the [livox_lidar_msg.launch](https://github.com/Livox-SDK/livox_ros_driver/blob/master/livox_ros_driver/launch/livox_lidar_msg.launch) of [Livox-ros-driver](https://github.com/Livox-SDK/livox_ros_driver) before make the livox_ros_driver pakage.

### 3.2 For Livox serials with external IMU

mapping_avia.launch theratically supports mid-70, mid-40 or other livox serial LiDAR, but need to setup some parameters befor run:

Edit ``` config/avia.yaml ``` to set the below parameters:

1. LiDAR point cloud topic name: ``` lid_topic ```
2. IMU topic name: ``` imu_topic ```
3. Translational extrinsic: ``` extrinsic_T ```
4. Rotational extrinsic: ``` extrinsic_R ``` (only support rotation matrix)
- The extrinsic parameters in FAST-LIO is defined as the LiDAR's pose (position and rotation matrix) in IMU body frame (i.e. the IMU is the base frame). They can be found in the official manual.
- FAST-LIO produces a very simple software time sync for livox LiDAR, set parameter ```time_sync_en``` to ture to turn on. But turn on **ONLY IF external time synchronization is really not possible**, since the software time sync cannot make sure accuracy.

### 3.3 For Velodyne or Ouster (Velodyne as an example)

Step A: Setup before run

Edit ``` config/velodyne.yaml ``` to set the below parameters:

1. LiDAR point cloud topic name: ``` lid_topic ```
2. IMU topic name: ``` imu_topic ``` (both internal and external, 6-aixes or 9-axies are fine)
3. Set the parameter ```timestamp_unit``` based on the unit of **time** (Velodyne) or **t** (Ouster) field in PoindCloud2 rostopic
4. Line number (we tested 16, 32 and 64 line, but not tested 128 or above): ``` scan_line ```
5. Translational extrinsic: ``` extrinsic_T ```
6. Rotational extrinsic: ``` extrinsic_R ``` (only support rotation matrix)
- The extrinsic parameters in FAST-LIO is defined as the LiDAR's pose (position and rotation matrix) in IMU body frame (i.e. the IMU is the base frame).

Step B: Run below
```
    cd ~/$FAST_LIO_ROS_DIR$
    source devel/setup.bash
    roslaunch fast_lio mapping_velodyne.launch
```

Step C: Run LiDAR's ros driver or play rosbag.

### 3.4 For MARSIM Simulator

Install MARSIM: https://github.com/hku-mars/MARSIM and run MARSIM as below

```
cd ~/$MARSIM_ROS_DIR$
roslaunch test_interface single_drone_avia.launch
```

Then Run FAST-LIO:

```
roslaunch fast_lio mapping_marsim.launch
```

### 3.5 PCD file save

Set ``` pcd_save_enable ``` in launchfile to ``` 1 ```. All the scans (in global frame) will be accumulated and saved to the file ``` FAST_LIO/PCD/scans.pcd ``` after the FAST-LIO is terminated. ```pcl_viewer scans.pcd``` can visualize the point clouds.

*Tips for pcl_viewer:*
- change what to visualize/color by pressing keyboard 1,2,3,4,5 when pcl_viewer is running. 
```
    1 is all random
    2 is X values
    3 is Y values
    4 is Z values
    5 is intensity
```

## 4. Rosbag Example
### 4.1 Livox Avia Rosbag
<div align="left">
<img src="doc/results/HKU_LG_Indoor.png" width=47% />
<img src="doc/results/HKU_MB_002.png" width = 51% >

Files: Can be downloaded from [google drive](https://drive.google.com/drive/folders/1CGYEJ9-wWjr8INyan6q1BZz_5VtGB-fP?usp=sharing)

Run:
```
roslaunch fast_lio mapping_avia.launch
rosbag play YOUR_DOWNLOADED.bag

```

### 4.2 Velodyne HDL-32E Rosbag

**NCLT Dataset**: Original bin file can be found [here](http://robots.engin.umich.edu/nclt/).

We produce [Rosbag Files](https://drive.google.com/drive/folders/1blQJuAB4S80NwZmpM6oALyHWvBljPSOE?usp=sharing) and [a python script](https://drive.google.com/file/d/1QC9IRBv2_-cgo_AEvL62E1ml1IL9ht6J/view?usp=sharing) to generate Rosbag files: ```python3 sensordata_to_rosbag_fastlio.py bin_file_dir bag_name.bag```
    
Run:
```
roslaunch fast_lio mapping_velodyne.launch
rosbag play YOUR_DOWNLOADED.bag
```

## 5.Implementation on UAV
In order to validate the robustness and computational efficiency of FAST-LIO in actual mobile robots, we build a small-scale quadrotor which can carry a Livox Avia LiDAR with 70 degree FoV and a DJI Manifold 2-C onboard computer with a 1.8 GHz Intel i7-8550U CPU and 8 G RAM, as shown in below.

The main structure of this UAV is 3d printed (Aluminum or PLA), the .stl file will be open-sourced in the future.

<div align="center">
    <img src="doc/uav01.jpg" width=40.5% >
    <img src="doc/uav_system.png" width=57% >
</div>

## 6.Acknowledgments

Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), [Livox_Mapping](https://github.com/Livox-SDK/livox_mapping), [LINS](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM) and [Loam_Livox](https://github.com/hku-mars/loam_livox).
