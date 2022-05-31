# 功能介绍

mono3d_indoor_detection package是使用hobot_dnn package开发的室内物体3D检测算法示例，
在地平线X3开发板上使用3D检测模型和室内数据利用BPU处理器进行模型推理。
package订阅sensors/msg/Image(encoding必须为“nv12”)类型的话题，检测出物体的类别以及3D定位信息，并通过话题发布。

算法支持的室内物体检测类别如下：

```
1. 充电座
2. 垃圾桶
3. 拖鞋
```
每个类别包含长、宽、高、转向以及深度等信息。package对外发布包含3D检测信息的ai msg，用户可以订阅发布的ai msg用于应用开发。
完整的ai msg描述如下所示：
````
# 目标消息

# 目标类型名称，如：充电座、垃圾桶、拖鞋
# charging_base/trash_can/slipper
string type

# 目标跟踪ID号，暂时为0
uint64 track_id

# 检测框，3D检测顶点信息使用points描述，见下“关键点”描述
Roi[] rois

# 属性信息，attributes包括 type 和 value 字段。
  value字段在发布时乘了1000的系数，实际使用时，务！必！除以1000
Attribute[] attributes
Attribute[]数组包括以下信息：
type    description
length   物体长度
width    物体宽度
height   物体高度
depth    物体深度
rotation 物体相对于相机的朝向
score    物体类别的置信度，越接近1，说明置信度越高
x        物体在相机坐标系的x坐标
y        物体在相机坐标系的y坐标
z        物体在相机坐标系的z坐标

# 六面体角点，三维物体在图片占据的空间用六面体描述，六面体包含8个顶点，
8个顶点的顺序如下表示，每个顶点包含在图片的x，y坐标，
左面的六面体表示背对相机，右面的六面体表示面对相机
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

# 跟踪目标抓拍图、特征、特征的底库检索结果信息，为空
Capture[] captures
````

# 编译

## 依赖库


ros package：

- dnn_node
- ai_msgs
- OpenCV

dnn_node是在地平线X3开发板上利用BPU处理器进行模型推理的pkg，定义在hobot_dnn中。

ai_msgs为自定义的消息格式，用于算法模型推理后，发布推理结果，ai_msgs pkg定义在hobot_msgs中。

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### Ubuntu板端编译

1. 编译环境确认 
   - 板端已安装X3 Ubuntu系统。
   - 当前编译终端已设置TogetherROS环境变量：`source PATH/setup.bash`。其中PATH为TogetherROS的安装路径。
   - 已安装ROS2编译工具colcon，安装命令：`pip install -U colcon-common-extensions`
2. 编译

编译命令：`colcon build --packages-select mono3d_indoor_detection`

### Docker交叉编译

1. 编译环境确认

   - 在docker中编译，并且docker中已经安装好TogetherROS。docker安装、交叉编译说明、TogetherROS编译和部署说明详见机器人开发平台robot_dev_config repo中的README.md。

2. 编译

   - 编译命令：

```
export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE=/usr/bin/$TARGET_TRIPLE-

colcon build --packages-select mono3d_indoor_detection \
   --merge-install \
   --cmake-force-configure \
   --cmake-args \
   --no-warn-unused-cli \
   -DCMAKE_TOOLCHAIN_FILE=`pwd`/robot_dev_config/aarch64_toolchainfile.cmake
```

## 注意事项

# 使用介绍

## 依赖

- mono3d_indoor_detection package：发布3D检测信息

## 参数

| 参数名                 | 类型        | 解释                                        | 是否必须 | 支持的配置           | 默认值                        |
| ---------------------- | ----------- | ------------------------------------------- | -------- | -------------------- | ----------------------------- |
| is_sync_mode           | int         | 同步/异步推理模式。0：异步模式；1：同步模式 | 否       | 0/1                  | 0                             |
| model_file_name        | std::string | 推理使用的模型文件                          | 否       | 根据实际模型路径配置 | config/centernet.hbm           |
| ai_msg_pub_topic_name  | std::string | 发布包含3D检测结果的AI消息的topic名 | 是       | 根据实际部署环境配置 | /ai_msg_3d_detection |
| image_sub_topic_name | std::string | 订阅sensor_msgs::msg::Image     | 是       | 根据实际部署环境配置 | /image_raw     |

## 运行

编译成功后，将生成的install路径拷贝到地平线X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：

### **Ubuntu**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/mono3d_indoor_detection/config/ .

# 启动3D检测node
ros2 run mono3d_indoor_detection mono3d_indoor_detection &
# 启动图片发布node
ros2 run mono3d_indoor_detection image_publisher ./config/images/

```

### **Ubuntu Launch启动**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/mono3d_indoor_detection/config/ .

# 启动3D检测 package
ros2 launch mono3d_indoor_detection mono3d_indoor_detection.launch.py 

```

### **Linux**

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为示例使用的模型，根据实际安装路径进行拷贝
cp -r install/lib/mono3d_indoor_detection/config/ .

# 启动3D检测node
./install/lib/mono3d_indoor_detection/mono3d_indoor_detection &
# 启动图片发布node
./install/lib/mono3d_indoor_detection/image_publisher ./config/images/

```

# 结果分析
当 mono3d_indoor_detection处理完一帧图片数据后会在控制台打印出处理结果：
## X3结果展示

```
[INFO] [1653655461.402558532] [ai_msg_cb]: target type: slipper
[INFO] [1653655461.402727573] [ai_msg_cb]: attribute type: width, value: 0.105
[INFO] [1653655461.402782698] [ai_msg_cb]: attribute type: height, value: 0.105
[INFO] [1653655461.402833490] [ai_msg_cb]: attribute type: length, value: 0.256
[INFO] [1653655461.402882615] [ai_msg_cb]: attribute type: rotation, value: 0.233
[INFO] [1653655461.402932948] [ai_msg_cb]: attribute type: x, value: -0.001
[INFO] [1653655461.402981698] [ai_msg_cb]: attribute type: y, value: -0.021
[INFO] [1653655461.403030448] [ai_msg_cb]: attribute type: z, value: 0.596
[INFO] [1653655461.403079157] [ai_msg_cb]: attribute type: depth, value: 0.596
[INFO] [1653655461.403128115] [ai_msg_cb]: attribute type: score, value: 0.812
[INFO] [1653655461.403174032] [ai_msg_cb]: point type: corners
[INFO] [1653655461.403232448] [ai_msg_cb]: x: 1135.99 y: 575.944
[INFO] [1653655461.403286490] [ai_msg_cb]: x: 1133.35 y: 583.35
[INFO] [1653655461.403339782] [ai_msg_cb]: x: 792.333 y: 578.721
[INFO] [1653655461.403393240] [ai_msg_cb]: x: 846.214 y: 572.665
[INFO] [1653655461.403446448] [ai_msg_cb]: x: 1135.99 y: 448.325
[INFO] [1653655461.403499573] [ai_msg_cb]: x: 1133.35 y: 430.423
[INFO] [1653655461.403553073] [ai_msg_cb]: x: 792.333 y: 441.612
[INFO] [1653655461.403606282] [ai_msg_cb]: x: 846.214 y: 456.252
```
以上log截取了一帧的处理结果，结果显示，订阅到的ai msg中的target type即分类结果为slipper，
同时也给出了slipper的三维和距离以及旋转角度信息。




# 常见问题
