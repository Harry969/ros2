# 基于ROS2通信的目标检测项目

## 项目简介

本项目实现了基于ROS2通信的目标检测系统，包含：
- 发布者节点：模拟摄像头，循环发送图像数据
- 订阅者节点：接收图像并使用YOLOv8进行目标检测

## 项目结构

```
ros2_ws/
├── src/
│   └── yolo_ros2/          # ROS2包
│       ├── yolo_ros2/
│       │   ├── __init__.py
│       │   ├── publisher.py    # 发布者节点
│       │   └── subscriber.py   # 订阅者节点
│       ├── resource/
│       ├── package.xml
│       ├── setup.py
│       └── bus.jpg             # 测试图片
├── yolov8n.pt                  # YOLO权重文件
└── README.md
```

## 快速开始

### 1. 克隆项目
```bash
git clone https://github.com/您的用户名/ros2-yolo-detection.git
cd ros2-yolo-detection
```

### 2. 安装依赖
```bash
# 安装ROS2 Humble（如果未安装）
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# 安装Python依赖
pip3 install torch torchvision torchaudio ultralytics opencv-python numpy matplotlib
```

### 3. 构建项目
```bash
cd ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select yolo_ros2
source install/setup.bash
```

### 4. 运行项目
```bash
# 终端1 - 发布者
source install/setup.bash
ros2 run yolo_ros2 publisher

# 终端2 - 订阅者
source install/setup.bash
ros2 run yolo_ros2 subscriber
```

## 系统要求

- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+
- 至少4GB内存

## 详细操作手册

请参考 `ROS2_YOLO目标检测操作步骤.txt` 文件获取详细的操作步骤。

## 许可证

Apache-2.0 