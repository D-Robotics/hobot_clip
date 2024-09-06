[English](./README.md) | 简体中文

# 功能介绍

CLIP（https://github.com/openai/CLIP/）是由OpenAI提出的一种多模态机器学习模型。该模型通过对大规模图像和文本对进行对比学习，能够同时处理图像和文本，并将它们映射到一个共享的向量空间中。

本项目是 CLIP 文本编码器推理节点, 目前支持两种模式：
1. 本地模式：支持回灌输入, 输出文本编码特征。
2. 服务模式：基于Ros Action Server, 支持Clinet节点发送推理请求, 计算返回的文本编码特征。

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

ros package：

- clip_msgs

## 编译选项

## X5 Ubuntu系统上编译

1、编译环境确认

- 板端已安装X5 Ubuntu系统。
- 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon，需要手动安装colcon。colcon安装命令：`pip install -U colcon-common-extensions`
- 已编译dnn node package

2、编译

- 编译命令：`colcon build --packages-select clip_encode_text`

## docker交叉编译 X5 版本

1、编译环境确认

- 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2、编译

- 编译命令：

  ```shell
  # RDK X5
  bash robot_dev_config/build.sh -p X5 -s clip_encode_text
  ```

- 编译选项中默认打开了shared mem通信方式。

## 注意事项

# 使用介绍

## 参数

| 参数名             | 解释                                  | 是否必须             | 默认值              | 备注                                                                    |
| ------------------ | ------------------------------------- | -------------------- | ------------------- | ----------------------------------------------------------------------- |
| feed_type          | 文本来源，False：本地；True：服务            | 否                   | False                   |
| dump_result              | 是否保存特征结果                          | 否                   | False     |
| model_file_name        | 模型文件            | 否 | config/text_encoder.onnx                   |


## 运行

## X5 Ubuntu系统上运行

运行方式1，使用可执行文件启动：
```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash
# 下载模型并解压
wget http://archive.d-robotics.cc/models/clip_encode_text/text_encoder.tar.gz
mkdir -p config
sudo tar -xf text_encoder.tar.gz -C config

# 运行模式1：使用文本进行回灌预测并保存特征值
ros2 run clip_encode_text clip_encode_text_node --ros-args -p feed_type:=false -p dump_result:=true

# 运行模式2：设置服务模型,, 等待action client 服务请求, 并设置log级别为warn
ros2 run clip_encode_text clip_encode_text_node --ros-args -p feed_type:=true --log-level warn
```

## X5 buildroot系统上运行

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# 下载模型并解压
wget http://archive.d-robotics.cc/models/clip_encode_text/text_encoder.tar.gz
mkdir -p config
sudo tar -xf text_encoder.tar.gz -C config

# 运行模式1：使用文本进行回灌预测并保存特征值
./install/lib/clip_encode_text/clip_encode_text_node --ros-args -p feed_type:=false -p dump_result:=true

# 运行模式2：设置服务模型,, 等待action client 服务请求, 并设置log级别为info
./install/lib/clip_encode_text/clip_encode_text_node --ros-args -p feed_type:=true --log-level info
```

## 注意事项

# 结果分析

## X5结果展示

log：

运行命令：
```shell
# 运行终端1：启动服务 模式
ros2 run clip_encode_text clip_encode_text_node --ros-args -p feed_type:=true --log-level info

# 运行终端2：发送请求
ros2 action send_goal /clip_text_action clip_msgs/action/GetFeatures "{type: false, urls: [], texts: ['a diagram', 'a dog', 'a cat']}"
```

```shell
[WARN] [0000110738.339832137] [clip_encode_text_node]: Clip Encode Text Node has been started.
[INFO] [0000110742.630265305] [clip_encode_text_node]: Received goal request
[INFO] [0000110742.641228972] [clip_encode_text_node]: Executing goal
[WARN] [0000110742.644821597] [clip_encode_text_node]: Request texts: ['a diagram', 'a dog', 'a cat']
[INFO] [0000110742.869918264] [clip_encode_text_node]: clip input: ['a diagram'], token time: 0.002 s, inference time: 0.221 s
[INFO] [0000110743.073876181] [clip_encode_text_node]: clip input: ['a dog'], token time: 0.001 s, inference time: 0.198 s
[INFO] [0000110743.278893264] [clip_encode_text_node]: clip input: ['a cat'], token time: 0.001 s, inference time: 0.199 s
[WARN] [0000110743.286456097] [clip_encode_text_node]: Clip Encode Text Node work success.
```
