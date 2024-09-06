English| [简体中文](./README_cn.md)

Getting Started with Clip Encode Text
=======


# Feature Introduction

CLIP (https://github.com/openai/CLIP/) is a multimodal machine learning model proposed by OpenAI. This model uses contrastive learning on large-scale image-text pairs to process both images and text, mapping them into a shared vector space.

This project is an dnn node for the CLIP text encoder, currently supporting two modes:

Local mode: Supports input backpropagation, outputting text encoding features.
Service mode: Based on ROS Action Server, supports client nodes sending inference requests and calculating the returned text encoding features.

The data structure of the algorithm output, ClipItem:
```python
# item type, true: image, false: text
bool type

# item text, valid only when item is text
string text

# item file path, valid only when item is an image
string url

# feature values corresponding to the item
float32[] feature

# additional information about the item
string[] extra
```

# Development Environment

- Programming Language: C/C++
- Development Platform: X5
- System Version: Ubuntu 22.04
- Compilation Toolchain: Linaro GCC 11.4.0

# Compilation

- X5 Version: Supports compilation on the X5 Ubuntu system and cross-compilation using Docker on a PC.

It also supports controlling the dependencies and functionality of the compiled pkg through compilation options.

## Dependency Libraries

- OpenCV: 3.4.5

ROS Packages:

- clip_msgs

hbm_img_msgs is a custom image message format used for image transmission in shared memory scenarios. The hbm_img_msgs pkg is defined in hobot_msgs; therefore, if shared memory is used for image transmission, this pkg is required.

## Compilation Options

1. SHARED_MEM

- Shared memory transmission switch, enabled by default (ON), can be turned off during compilation using the -DSHARED_MEM=OFF command.
- When enabled, compilation and execution depend on the hbm_img_msgs pkg and require the use of tros for compilation.
- When disabled, compilation and execution do not depend on the hbm_img_msgs pkg, supporting compilation using native ROS and tros.
- For shared memory communication, only subscription to nv12 format images is currently supported.## Compile on X3/Rdkultra Ubuntu System

1. Compilation Environment Verification

- The X3 Ubuntu system is installed on the board.
- The current compilation terminal has set up the TogetherROS environment variable: `source PATH/setup.bash`. Where PATH is the installation path of TogetherROS.
- The ROS2 compilation tool colcon is installed. If the installed ROS does not include the compilation tool colcon, it needs to be installed manually. Installation command for colcon: `pip install -U colcon-common-extensions`.
- The dnn node package has been compiled.

2. Compilation

- Compilation command: `colcon build --packages-select clip_encode_text`

## Docker Cross-Compilation for X5 Version

1. Compilation Environment Verification

- Compilation within docker, and TogetherROS has been installed in the docker environment. For instructions on docker installation, cross-compilation, TogetherROS compilation, and deployment, please refer to the README.md in the robot development platform's robot_dev_config repo.
- The dnn node package has been compiled.
- The hbm_img_msgs package has been compiled (see Dependency section for compilation methods).

2. Compilation

- Compilation command:

  ```shell
  # RDK X5
  bash robot_dev_config/build.sh -p X5 -s clip_encode_text
  ```

- Shared memory communication method is enabled by default in the compilation options.

## Notes


# Instructions

## Parameters

| Parameter Name      | Explanation                            | Mandatory            | Default Value       | Remarks                                                                 |
| ------------------- | -------------------------------------- | -------------------- | ------------------- | ----------------------------------------------------------------------- |
| feed_type           | Text source, False: local; True: server   | No                   | False                   |                                                                         |
| dump_result               | is dump result                      | No                   | False     |                                                                         |
| model_file_name  | dnn model file name | No | config/full_model_11.bin |                                                                      |

## Running

## Running on X5 Ubuntu System

Running method 1, use the executable file to start:
```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash
# Download the model and decompress
wget http://archive.d-robotics.cc/models/clip_encode_text/text_encoder.tar.gz
sudo tar -xf text_encoder.tar.gz -C config

# Run mode 1: Use text for recharge prediction and save feature values:
ros2 run clip_encode_text clip_encode_text_node --ros-args -p feed_type:=false -p dump_result:=true

# Run mode 2: Set the service mode, wait for action client service requests, and set the log level to warn:
ros2 run clip_encode_text clip_encode_text_node --ros-args -p feed_type:=true --log-level warn
```

## Run on X5 buildroot system:

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# Download the model and decompress
wget http://archive.d-robotics.cc/models/clip_encode_text/text_encoder.tar.gz
sudo tar -xf text_encoder.tar.gz -C config

# Run mode 1: Use text for recharge prediction and save feature values:
./install/lib/clip_encode_text/clip_encode_text_node --ros-args -p feed_type:=false -p dump_result:=true

# Run mode 2: Set the service mode, wait for action client service requests, and set the log level to warn:
./install/lib/clip_encode_text/clip_encode_text_node --ros-args -p feed_type:=true --log-level info
```


# Results Analysis

## X5 Results Display

log:
```shell
# Run Terminal 1: Start Subscription/Service Mode
ros2 run clip_encode_text clip_encode_text_node --ros-args -p feed_type:=true --log-level info

# Run Terminal 2: Send Reasoning Request
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