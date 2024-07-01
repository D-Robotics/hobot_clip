[English](./README.md) | 简体中文

# 功能介绍

CLIP（https://github.com/openai/CLIP/）是由OpenAI提出的一种多模态机器学习模型。该模型通过对大规模图像和文本对进行对比学习，能够同时处理图像和文本，并将它们映射到一个共享的向量空间中。

本项目是 CLIP 图像编码器边缘端推理节点, 目前支持两种模式：
1. 本地模式：支持回灌输入, 输出图像编码特征。
2. 服务模式：基于Ros Action Server, 支持Clinet节点发送推理请求, 计算返回的图像编码特征。

算法输出的数据结构ClipItem：
```python
# item 类型, true: 图片, false: 文本
bool type

# item 文本, 仅item为文本时有效
string text

# item 文件路径地址, 仅item为图片时有效
string url

# item 对应的特征值
float32[] feature

# item 额外信息
string[] extra
```

# 开发环境

- 编程语言: C/C++
- 开发平台: X5
- 系统版本：Ubuntu 22.04
- 编译工具链:Linux GCC 11.4.0

# 编译

- X5版本：支持在X5 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

## 依赖库

- opencv:3.4.5

ros package：

- dnn node
- cv_bridge
- sensor_msgs
- hbm_img_msgs
- ai_msgs

hbm_img_msgs为自定义的图片消息格式，用于shared mem场景下的图片传输，hbm_img_msgs pkg定义在hobot_msgs中，因此如果使用shared mem进行图片传输，需要依赖此pkg。

## 编译选项

1、SHARED_MEM

- shared mem（共享内存传输）使能开关，默认打开（ON），编译时使用-DSHARED_MEM=OFF命令关闭。
- 如果打开，编译和运行会依赖hbm_img_msgs pkg，并且需要使用tros进行编译。
- 如果关闭，编译和运行不依赖hbm_img_msgs pkg，支持使用原生ros和tros进行编译。
- 对于shared mem通信方式，当前只支持订阅nv12格式图片。

## X5 Ubuntu系统上编译

1、编译环境确认

- 板端已安装X5 Ubuntu系统。
- 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon，需要手动安装colcon。colcon安装命令：`pip install -U colcon-common-extensions`
- 已编译dnn node package

2、编译

- 编译命令：`colcon build --packages-select clip_encode_image`

## docker交叉编译 X5 版本

1、编译环境确认

- 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。
- 已编译 clip_encode_image package
- 已编译 hbm_img_msgs package（编译方法见Dependency部分）

2、编译

- 编译命令：

  ```shell
  # RDK X5
  bash robot_dev_config/build.sh -p X5 -s clip_encode_image
  ```

- 编译选项中默认打开了shared mem通信方式。

## 注意事项

# 使用介绍

## 参数

| 参数名             | 解释                                  | 是否必须             | 默认值              | 备注                                                                    |
| ------------------ | ------------------------------------- | -------------------- | ------------------- | ----------------------------------------------------------------------- |
| feed_type          | 图片来源，0：本地；1：服务            | 否                   | 0                   |
| image              | 本地图片地址                          | 否                   | config/CLIP.png     |
| is_shared_mem_sub  | 使用shared mem通信方式订阅图片        | 否                   | 0                   | 
| is_sync_mode  | 推理模式，0：同步；1：异步        | 否                   | 0                   | 
| model_file_name        | 模型文件            | 否 | config/full_model_11.bin                   |


## 运行

## X5 Ubuntu系统上运行

运行方式1，使用可执行文件启动：
```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash
# config中为示例使用的模型，回灌使用的本地图片
cp -r install/lib/clip_encode_image/config/ .

# 运行模式1：使用本地png格式深度图通过同步模式进行回灌预测
ros2 run clip_encode_image clip_encode_image --ros-args -p feed_type:=0 -p image:=config/CLIP.png

# 运行模式2：设置订阅/服务模型, 使用订阅到的image msg(topic为/image_raw) 通过异步模式进行预测, 等待action client 服务请求, 并设置log级别为warn
ros2 run clip_encode_image clip_encode_image --ros-args -p feed_type:=1 --log-level warn -p is_sync_mode:=1
```

## X5 buildroot系统上运行

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为示例使用的模型，回灌使用的本地图片
cp -r install/lib/clip_encode_image/config/ .

# 运行模式1：使用本地png格式深度图通过同步模式进行回灌预测
./install/lib/clip_encode_image/clip_encode_image --ros-args -p  feed_type:=0 -p image:=config/CLIP.png

# 运行模式2：设置订阅/服务模型, 使用订阅到的image msg(topic为/image_raw) 通过异步模式进行预测, 等待action client 服务请求, 并设置log级别为warn
./install/lib/clip_encode_image/clip_encode_image --ros-args -p feed_type:=1 --log-level warn -p is_sync_mode:=1

```

## 注意事项

# 结果分析

## X5结果展示

log：

运行命令：
```shell
# 运行终端1：启动订阅/服务 模式
ros2 run clip_encode_image clip_encode_image --ros-args -p feed_type:=1 --log-level warn -p is_sync_mode:=1

ros2 action send_goal /clip_image_action clip_msgs/action/GetFeatures "{type: true, urls: ['config/CLIP.png', 'config/CLIP.png', 'config/CLIP.png', 'config/CLIP.png', 'config/CLIP.png', 'config/CLIP.png', 'config/CLIP.png', 'config/CLIP.png', 'config/CLIP.png', 'config/CLIP.png'], texts: []}"
```

```shell
[WARN] [0000108045.035041019] [ClipImageNode]: Parameter:
 feed_type(0:loacl, 1:sub): 1
 is_shared_mem_sub: 1
 is_sync_mode_: 1
 image: config/CLIP.png
 model_file_name_: config/full_model_11.bin
[BPU_PLAT]BPU Platform Version(1.3.6)!
[HBRT] set log level as 0. version = 3.15.47.0
[DNN] Runtime version = 1.23.5_(3.15.47 HBRT)
[A][DNN][packed_model.cpp:247][Model](1970-01-02,06:00:46.778.143) [HorizonRT] The model builder version = 1.23.5
[WARN] [0000108047.628604104] [ClipImageNode]: Create hbmem_subscription with topic_name: /hbmem_img
[WARN] [0000108052.237163606] [encode_image_server]: Received goal request with type: 1
[WARN] [0000108052.238126314] [encode_image_server]: Executing goal
[WARN] [0000108053.780792607] [ClipImageNode]: Sub img fps: -1.00, Smart fps: 8.25, preprocess time ms: 59, infer time ms: 518, post process time ms: 0
[WARN] [0000108054.791171774] [ClipImageNode]: Sub img fps: 17.31, Smart fps: 7.92, preprocess time ms: 59, infer time ms: 496, post process time ms: 0
[WARN] [0000108054.926602107] [encode_image_server]: Goal complete, task_result: 1
```
