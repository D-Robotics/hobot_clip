[English](./README.md) | 简体中文

# 功能介绍

CLIP（https://github.com/openai/CLIP/）是由OpenAI提出的一种多模态机器学习模型。该模型通过对大规模图像和文本对进行对比学习，能够同时处理图像和文本，并将它们映射到一个共享的向量空间中。

本项目是 CLIP 中继节点,负责收发 目前支持两种模式：
1. 入库模式：向图像编码节点 clip_manage 发送编码请求, 获取目标文件夹中图像编码特征, 将图像编码特征存储到本地SQLite数据库中。
2. 查询模式：向图像编码节点 clip_encode_text 发送编码请求, 获取目标文本编码特征。进一步将文本特征与数据库图像特征进行匹配, 获得匹配结果。

算法输出的数据结构ClipItem：
```c++
struct ClipItem {
  bool type;        // item 类型, true: 图片, false: 文本
  std::string name; // item 名
  std::string text; // item text, 仅item为文本时有效
  std::string url;  // item 文件路径地址, 仅item为图片时有效
  std::vector<float> feature; // item 对应的特征, 长度512
  std::vector<std::string> extra;   // item 额外信息, 如对应深度图路径、IMU数据路径、其他说明等。

  int id;           // item 唯一id
  long timestamp;   // item 入库时间戳
  float similarity; // item 的相似度
};
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

- ai_msgs

## X5 Ubuntu系统上编译

1、编译环境确认

- 板端已安装X5 Ubuntu系统。
- 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
- 已安装ROS2编译工具colcon。安装的ROS不包含编译工具colcon，需要手动安装colcon。colcon安装命令：`pip install -U colcon-common-extensions`
- 已编译dnn node package

2、编译

- 编译命令：`colcon build --packages-select clip_manage`

## docker交叉编译 X5 版本

1、编译环境确认

- 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2、编译

- 编译命令：

  ```shell
  # RDK X5
  bash robot_dev_config/build.sh -p X5 -s clip_manage
  ```

- 编译选项中默认打开了shared mem通信方式。

## 注意事项

# 使用介绍

## 参数

| 参数名             | 解释                                  | 是否必须             | 默认值              | 备注                                                                    |
| ------------------ | ------------------------------------- | -------------------- | ------------------- | ----------------------------------------------------------------------- |
| mode          | 模式，0：入库；1：检索            | 否                   | 0                   |
| text              | 查询文本                          | 否                   | "a diagram"     |
| db_file              | db文件名字                          | 否                   | clip.db     |
| storage_folder  | 待入库的文件存储路径        | 否                   | config                   | 
| result_folder  | 检索结果存放路径        | 否                   | result                   | 
| topk  | 检索结果数TopK        | 否                   | 10                   | 


## 运行

运行模式 1 前, 请先启动[clip_encode_image](./../clip_encode_image/README.md)

运行模式 2 前, 请先启动[clip_encode_image](./../clip_encode_text/README.md)

## X5 Ubuntu系统上运行

运行方式1，使用可执行文件启动：

```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash

# 运行模式1：入库, 使用本地图片进行入库操作(需要先开启clip_encode_image的示例) 
ros2 run clip_manage clip_manage --ros-args -p mode:=0 -p db_file:=clip.db -p storage_folder:=config

# 运行模式2：查询, 输入检索文本, 查询数据库中的图片(需要先开启clip_encode_text的示例)
ros2 run clip_manage clip_manage --ros-args -p mode:=1 --log-level info -p db_file:=clip.db -p result_folder:=result -p text:="a diagram"
```

## X5 buildroot系统上运行

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# 运行模式2：查询, 输入检索文本, 查询数据库中的图片(需要先开启clip_encode_text的示例)
./install/lib/clip_manage/clip_manage --ros-args -p mode:=0 -p db_file:=clip.db -p storage_folder:=config

# 运行模式2：查询, 输入检索文本, 查询数据库中的图片
./install/lib/clip_manage/clip_manage --ros-args -p mode:=1 --log-level info -p db_file:=clip.db -p result_folder:=result -p text:="a diagram"
```

## 注意事项

# 结果分析

## X5结果展示

### 模式1 入库
运行命令：
```shell
ros2 run clip_encode_image clip_encode_image --ros-args -p feed_type:=1 --log-level warn -p is_sync_mode:=1

ros2 run clip_manage clip_manage --ros-args -p mode:=0 -p db_file:=clip.db -p storage_folder:=config
```

结果
```shell

```

### 模式2 查询

运行命令:
```shell
ros2 run clip_encode_text clip_encode_text_node --ros-args -p feed_type:=true --log-level info

ros2 run clip_manage clip_manage --ros-args -p mode:=1 --log-level info -p db_file:=clip.db -p result_folder:=result -p text:="a diagram"
```

结果
```shell

```