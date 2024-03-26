English| [简体中文](./README_cn.md)

# Function Introduction

The mono3d_indoor_detection package is an example of indoor 3D object detection algorithm developed using the hobot_dnn package, which utilizes a 3D detection model and indoor data on the Horizon Sunrise X3 development board for model inference with the BPU processor.
The 3D detection model is related to sensors and sensor parameters. The performance may vary under different sensor and parameter settings. Therefore, this package does not directly subscribe to topics of type sensors/msg/Image by default, but conducts inference by reading local images to detect object categories and 3D positioning information. The AI information will be published through topics, and the results will also be rendered and saved on images in the 'result' directory during program execution.

The supported indoor object detection categories of the algorithm are as follows:

```
1. Charging base
2. Trash can
3. Slipper
```
Each category includes information such as length, width, height, rotation, and depth. The length, width, and height of an object correspond to the dimensions of a hexahedron, and the depth refers to the distance from the camera to the object. The units for the parameters above are in millimeters. Rotation represents the orientation of an object relative to the camera in radians, with a range of -pi to pi, indicating the angle between the forward direction of the object in the camera coordinate system and the x-axis of the camera coordinate system.

The package releases AI Msg containing 3D detection information for external use, and users can subscribe to the published AI Msg for application development.
The detailed description of the AI Msg is as follows:

```
# Target Message

# Target type name, such as:Charging dock, trash can, slippers
# charging_base/trash_can/slipper
string type

# Tracking ID of the target, currently set as 0
uint64 track_id

# Detection box, 3D detection vertex information described using points, see "Key Points" section below
Roi[] rois

# Attribute information, attributes include the 'type' and 'value' fields.
Attribute[] attributes
The Attribute[] array includes the following information:
type       description
length     Object length
width      Object width
height     Object height
depth      Object depth
rotation   Object orientation relative to the camera
score      Confidence score of the object category, closer to 1 indicates higher confidence
x          x-coordinate of the object in the camera coordinate system
y          y-coordinate of the object in the camera coordinate system
z          z-coordinate of the object in the camera coordinate system

# Hexahedral corner points, the spatial occupation of a three-dimensional object in the image is described by a hexahedron, which includes 8 vertices.
The order of the 8 vertices is as follows, each vertex contains the x, y coordinates on the image.
The left hexahedron indicates facing away from the camera, and the right hexahedron indicates facing the camera
      4----------5    6----------7
     /|         /|   /|         /|
    / |        / |  / |        / |
   /  |       /  | /  |       /  |
  7---|------6   |5---|------4   |
  |   |      |   ||   |      |   |
  |   |      |   ||   |      |   |
  |   0------|---1|   2------|---3
  |  /       |  / |  /       |  /
  | /     ^  | /  | /     v  | /
  |/         |/   |/         |/
  3----------2    1----------0
Point[] points

# Track target capture images, features, and feature library retrieval results are empty
Capture[] captures
````

# Compilation

## Dependencies

ROS packages:

- dnn_node
- ai_msgs
- OpenCV

dnn_node is a package that performs model inference using the BPU processor on the Horizon X3 development board and is defined in hobot_dnn.

ai_msgs is a custom message format used to publish inference results after algorithm model inference, and the ai_msgs package is defined in hobot_msgs.

OpenCV is used for image processing.

## Development Environment

- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.0.4
- Compilation Toolchain: Linux GCC 9.3.0 / Linaro GCC 9.3.0

## Compilation

Supports compiling on X3 Ubuntu system and cross-compiling using docker on a PC.

### Compilation for X3 on Ubuntu Board

1. Compile Environment Verification
   - X3 Ubuntu system is installed on the board.
   - The current compilation terminal has set the TogetherROS environment variable: `source PATH/setup.bash`. Here, PATH is the installation path of TogetherROS.
   - ROS2 compilation tool colcon is installed, installation command: `pip install -U colcon-common-extensions`
2. Compilation
   - Compilation command：`colcon build --packages-select mono3d_indoor_detection`
   
## Cross-compile X3 version with Docker

1. Verify the compilation environment

   - Compile in Docker with TogetherROS already installed. For Docker installation, cross-compilation instructions, TogetherROS compilation and deployment instructions, refer to the README.md in the robot development platform robot_dev_config repository.

2. Compilation

   - Compilation command:

   ```
   export TARGET_ARCH=aarch64
   export TARGET_TRIPLE=aarch64-linux-gnu
   export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

   colcon build --packages-select mono3d_indoor_detection \
      --merge-install \
      --cmake-force-configure \
      --cmmake-args \
      --no-warn-unused-cli \
      -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
   ```

## Compile X86 version on X86 Ubuntu system

1. Verify the compilation environment

   - X86 Ubuntu version: ubuntu20.04

2. Compilation

   - Compilation command:

   ```
   colcon build --packages-select mono3d_indoor_detection \
      --merge-install \
      --cmake-args \
      -DPLATFORM_X86=ON \
      -DTHIRD_PARTY=`pwd`/../sysroot_docker \
   ```

## Notes

# Instructions

## Dependencies
- mono3d_indoor_detection package: Publishes 3D detection information

## Parameters

| Parameter Name        | Type        | Description                                 | Required | Supported Configuration | Default Value |
| ---------------------- | ----------- | ------------------------------------------- | -------- | ----------------------- | ------------- |
| config_file_path | std::string | Path of the configuration file used for inference, including model files | No | Configure according to actual path | ./config |
| feed_image | std::string | Image used for inference | No | Configure according to actual path | "" |
| ai_msg_pub_topic_name  | std::string | Topic name for publishing AI messages containing 3D detection results | No | Configure according to actual deployment environment | /ai_msg_3d_detection |
| dump_render_img  | int | Whether to render, 0: No, 1: Yes | No | 0/1 | 0 |
| shared_mem  | int | Whether to use shared memory mode for subscribing images, 0: No, 1: Yes | No | 0/1 | 0 |


## Run

After successful compilation, copy the generated install path to the Horizon X3 development board (if compiling on X3, ignore the copying step), and execute the following command to run:

### **X3 Ubuntu**


**Using local images as input and saving the rendered images**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# Copy according to the actual installation path, using the model and image in the configuration as an example
# If compiling on board side (without --merge-install compilation option), the copy command is cp -r install/PKG_NAME/lib/PKG_NAME/config/ ., where PKG_NAME is the specific package name.
cp -r install/lib/mono3d_indoor_detection/config/ .

# Launch the 3D detection package
ros2 launch mono3d_indoor_detection mono3d_indoor_detection.launch.py 

```

**Subscribe to hobot_image_publisher to publish local images for web display**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# Copy according to the actual installation path, using the model and image in the configuration as an example
cp -r install/lib/mono3d_indoor_detection/config/ .

export CAM_TYPE=fb

# Launch the 3D detection package
ros2 launch mono3d_indoor_detection mono3d_indoor_detection_pipeline.launch.py

```

**Subscribe to MIPI camera to publish images for web display**
```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# Copy models and images used in config according to actual installation path
cp -r install/lib/mono3d_indoor_detection/config/ .

# Default to using F37 camera
export CAM_TYPE=mipi

# Launch the 3D detection package
ros2 launch mono3d_indoor_detection mono3d_indoor_detection_pipeline.launch.py

```

### **X3 Linux**

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# Copy models used in config according to actual installation path
cp -r install/lib/mono3d_indoor_detection/config/ .

# Launch the 3D detection node
./install/lib/mono3d_indoor_detection/mono3d_indoor_detection --ros-args -p feed_image:=./config/images/3d_detection.png -p dump_render_img:=1

```

### **X86 Ubuntu**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# Copy models used in config according to actual installation path
cp -r install/lib/mono3d_indoor_detection/config/ .

# Launch the 3D detection node
./install/lib/mono3d_indoor_detection/mono3d_indoor_detection --ros-args -p feed_image:=./config/images/3d_detection.png -p dump_render_img:=1

```

# Results Analysis
After mono3d_indoor_detection processes a frame of image data, it will print the processing result on the console:
## X3 Result Display

```
[INFO] [1654858490.168592166] [mono3d_detection]: target type: trash_can
[INFO] [1654858490.168644750] [mono3d_detection]: target type: width, value: 236.816406
[INFO] [1654858490.168704333] [mono3d_detection]: target type: height, value: 305.664062
[INFO] [1654858490.168759584] [mono3d_detection]: target type: length, value: 224.182129
[INFO] [1654858490.168812334] [mono3d_detection]: target type: rotation, value: -1571.989179
[INFO] [1654858490.168862543] [mono3d_detection]: target type: x, value: -191.977829
[INFO] [1654858490.168916168] [mono3d_detection]: target type: y, value: -143.963307
[INFO] [1654858490.168966502] [mono3d_detection]: target type: z, value: 714.024127
[INFO] [1654858490.169016794] [mono3d_detection]: target type: depth, value: 714.024127
[INFO] [1654858490.169067461] [mono3d_detection]: target type: score, value: 0.973215
[INFO] [1654858490.169168795] [mono3d_detection]: target type: trash_can
[INFO] [1654858490.169212837] [mono3d_detection]: target type: width, value: 253.051758
[INFO] [1654858490.169265004] [mono3d_detection]: target type: height, value: 282.348633
[INFO] [1654858490.169317046] [mono3d_detection]: target type: length, value: 257.934570
[INFO] [1654858490.169368921] [mono3d_detection]: target type: rotation, value: -1542.727947
[INFO] [1654858490.169418671] [mono3d_detection]: target type: x, value: 552.459776
[INFO] [1654858490.169470588] [mono3d_detection]: target type: y, value: -164.073169
[INFO] [1654858490.169517505] [mono3d_detection]: target type: z, value: 1088.358164
[INFO] [1654858490.169566839] [mono3d_detection]: target type: depth, value: 1088.358164
[INFO] [1654858490.169616464] [mono3d_detection]: target type: score, value: 0.875521
```
The above log snippet captures the processing results of a frame, indicating that the target type received in the AI message is "trash_can",
providing three-dimensional information, distance, and rotation angle for the trash can.

In the example, the result of inference on a locally read image is rendered on the image and saved in the "result" directory in the current path.

# Frequently Asked Questions
