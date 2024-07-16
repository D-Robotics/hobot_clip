English| [简体中文](./README_cn.md)

Getting Started with Clip Encode Image
=======


# Feature Introduction

CLIP (https://github.com/openai/CLIP/) is a multimodal machine learning model proposed by OpenAI. This model uses contrastive learning on large-scale image-text pairs to process both images and text, mapping them into a shared vector space.

This project is an dnn node for the CLIP image encoder, currently supporting two modes:

Local mode: Supports input backpropagation, outputting image encoding features.
Service mode: Based on ROS Action Server, supports client nodes sending inference requests and calculating the returned image encoding features.

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

- dnn node
- cv_bridge
- sensor_msgs
- hbm_img_msgs
- ai_msgs
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

- Compilation command: `colcon build --packages-select clip_emcpde_image`

## Docker Cross-Compilation for X5 Version

1. Compilation Environment Verification

- Compilation within docker, and TogetherROS has been installed in the docker environment. For instructions on docker installation, cross-compilation, TogetherROS compilation, and deployment, please refer to the README.md in the robot development platform's robot_dev_config repo.
- The dnn node package has been compiled.
- The hbm_img_msgs package has been compiled (see Dependency section for compilation methods).

2. Compilation

- Compilation command:

  ```shell
  # RDK X5
  bash robot_dev_config/build.sh -p X5 -s clip_emcpde_image
  ```

- Shared memory communication method is enabled by default in the compilation options.

## Notes


# Instructions

## Parameters

| Parameter Name      | Explanation                            | Mandatory            | Default Value       | Remarks                                                                 |
| ------------------- | -------------------------------------- | -------------------- | ------------------- | ----------------------------------------------------------------------- |
| feed_type           | Image source, 0: local; 1: server   | No                   | 0                   |                                                                         |
| image               | Local image path                       | No                   | config/CLIP.png     |                                                                         |
| is_shared_mem_sub   | Subscribe to images using shared memory communication method | No  | 0                   |                                                                         |
| is_sync_mode     | Installer inference mode, 0: synchronous; 1: Asynchronous       | No                   | 0                   |                                                                         |
| model_file_name  | dnn model file name | No | config/full_model_11.bin |                                                                      |

## Running

## Running on X3 Ubuntu System

Running method 1, use the executable file to start:
```shell
export COLCON_CURRENT_PREFIX=./install
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash
# The model used as an example in config and the local image used for backfilling
cp -r install/lib/clip_encode_image/config/ .

# Run mode 1: Use local PNG format depth map to perform recharge prediction through synchronous mode:
ros2 run clip_encode_image clip_encode_image --ros-args -p feed_type:=0 -p image:=config/CLIP.png

# Run mode 2: Set up a subscription/service model, use the subscribed image msg (topic/image_raw) to make predictions through asynchronous mode, wait for action client service requests, and set the log level to warn:
ros2 run clip_encode_image clip_encode_image --ros-args -p feed_type:=1 --log-level warn -p is_sync_mode:=1
```

## Run on X5 buildroot system:

```shell
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# Copy the configuration used by the example and the local image used for inference
cp -r install/lib/clip_emcpde_image/config/ .

# Run mode 1: Use local PNG format depth map to perform recharge prediction through synchronous mode:
./install/lib/clip_encode_image/clip_encode_image --ros-args -p  feed_type:=0 -p image:=config/CLIP.png

# Run mode 2: Set up a subscription/service model, use the subscribed image msg (topic/image_raw) to make predictions through asynchronous mode, wait for action client service requests, and set the log level to warn:
./install/lib/clip_encode_image/clip_encode_image --ros-args -p feed_type:=1 --log-level warn -p is_sync_mode:=1
```


# Results Analysis

## X5 Results Display

log:
```shell
# Run Terminal 1: Start Subscription/Service Mode
ros2 run clip_encode_image clip_encode_image --ros-args -p feed_type:=1 --log-level warn -p is_sync_mode:=1

# Run Terminal 2: Send Reasoning Request
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