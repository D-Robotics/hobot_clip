[English](./README.md) | 简体中文

# 功能介绍

CLIP（https://github.com/openai/CLIP/）是由OpenAI提出的一种多模态机器学习模型。该模型通过对大规模图像和文本对进行对比学习，能够同时处理图像和文本，并将它们映射到一个共享的向量空间中。

项目包含几个节点：

- [clip_encode_image](./clip_encode_image): 图像编码器边缘端推理节点, 支持两种模式：
  - 本地模式：支持回灌输入, 输出图像编码特征。
  - 服务模式：基于Ros Action Server, 支持Clinet节点发送推理请求, 计算返回的图像编码特征。
- [clip_encode_text](./clip_encode_text): 图像编码器边缘端推理节点, 支持两种模式：
  - 本地模式：支持回灌输入, 输出文本编码特征。
  - 服务模式：基于Ros Action Server, 支持Clinet节点发送推理请求, 计算返回的文本编码特征。
- [clip_manage](./clip_manage): CLIP中继节点, 负责收发, 支持两种模式：
  - 入库模式：向图像编码节点 clip_encode_image 发送编码请求, 获取目标文件夹中图像编码特征, 将图像编码特征存储到本地SQLite数据库中。
  - 查询模式：向文本编码节点 clip_encode_text 发送编码请求, 获取目标文本编码特征。进一步将文本特征与数据库图像特征进行匹配, 获得匹配结果。
- [clip_msgs](./clip_msgs): CLIP系统的话题消息, action server的控制消息。


# 功能使用

### 获取模型
运行命令：
```shell
cp -r ./install/clip_encode_image/config .
wget http://sunrise.horizon.cc/models/clip_encode_text/text_encoder.tar.gz
sudo tar -xf text_encoder.tar.gz -C config
```

### 模式1 入库
运行命令：
```shell
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ros2 launch clip_manage hobot_clip_manage.launch.py clip_mode:=0
```

### 模式2 查询

运行命令:
```shell
source /opt/ros/humble/setup.bash
source ./install/setup.bash
ros2 launch clip_manage hobot_clip_manage.launch.py clip_mode:=1
```

### web效果展示

```shell
# 打开另一个终端：启动Web服务查看检索结果, 确保/userdata为检索结果result的上一级目录。
cp -r install/lib/clip_manage/config/index.html /userdata
cd /userdata
python -m http.server
```
使用谷歌浏览器或Edge，输入<http://IP:8000>，即可查看图像检索结果（IP为设备IP地址）。

![image](./clip_manage/img/query_display.png)

结果分析：按顺序依次可以看到检索文本与图片相似度依次检索结果。