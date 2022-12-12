# 功能介绍

mono3d_indoor_detection package是使用hobot_dnn package开发的室内物体3D检测算法示例，在地平线旭日X3开发板上使用3D检测模型和室内数据利用BPU处理器进行模型推理。
3D检测模型与sensor以及sensor参数相关，在不同sensor不同参数情况下展示出来的效果不一样，因此本package默认并不直接订阅sensors/msg/Image类型的话题，而是通过读取本地图片的形式进行推理，检测出物体的类别以及3D定位信息，将AI信息通过话题发布的同时会将结果渲染在图片上保存在程序运行的result目录。

算法支持的室内物体检测类别如下：

```
1. 充电座
2. 垃圾桶
3. 拖鞋
```
每个类别包含长、宽、高、转向以及深度等信息。物体的长、宽、高对应于六面体的长、宽、高，深度是指摄像机到物体的距离。上面参数单位为毫米。转向是指物体相对于相机的朝向，单位弧度，取值范围为：-pi ~ pi（单位：rad），表示在照相机坐标系下物体前进方向与相机坐标系x轴的夹角。

package对外发布包含3D检测信息的AI Msg，用户可以订阅发布的AI Msg用于应用开发。
完整的AI Msg描述如下所示：

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

dnn_node是在地平线旭日X3开发板上利用BPU处理器进行模型推理的package，定义在hobot_dnn中。

ai_msgs为自定义的消息格式，用于算法模型推理后，发布推理结果，ai_msgs package定义在hobot_msgs中。

OpenCV用于图像处理。

## 开发环境

- 编程语言: C/C++
- 开发平台: X3/X86
- 系统版本：Ubuntu 20.0.4
- 编译工具链:Linux GCC 9.3.0/Linaro GCC 9.3.0

## 编译

 支持在X3 Ubuntu系统上编译和在PC上使用docker交叉编译两种方式。

### 编译选项

1. SHARED_MEM
   - 零拷贝使能开关，默认打开(ON), 编译时可以通过--cmake-args -DSHARED_MEM=OFF关闭。
   - 打开时，编译会依赖hbm_img_msgs package。

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
| config_file_path | std::string | 推理使用的配置文件路径，内含模型文件               | 否       | 根据实际路径配置 | ./config       |
| feed_image | std::string | 推理使用的图片 | 否 | 根据实际路径配置 | "" |
| ai_msg_pub_topic_name  | std::string | 发布包含3D检测结果的AI消息的topic名 | 否      | 根据实际部署环境配置 | /ai_msg_3d_detection |
| dump_render_img  | int | 是否进行渲染，0：否；1：是 | 否      | 0/1 | 0 |
| shared_mem  | int | 是否使用shared_mem模式订阅图片，0：否；1：是 | 否      | 0/1 | 0 |


## 运行

编译成功后，将生成的install路径拷贝到地平线旭日X3开发板上（如果是在X3上编译，忽略拷贝步骤），并执行如下命令运行：

### **Ubuntu**


**使用本地图片作为输入，并保存渲染后的图片**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型和图片，根据实际安装路径进行拷贝
# 如果是板端编译（无--merge-install编译选项），拷贝命令为cp -r install/PKG_NAME/lib/PKG_NAME/config/ .，其中PKG_NAME为具体的package名。
cp -r install/lib/mono3d_indoor_detection/config/ .

# 启动3D检测 package
ros2 launch mono3d_indoor_detection mono3d_indoor_detection.launch.py 

```

**订阅hobot_image_publisher发布本地图片，web端展示**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型和图片，根据实际安装路径进行拷贝
cp -r install/lib/mono3d_indoor_detection/config/ .

export CAM_TYPE=fb

# 启动3D检测 package
ros2 launch mono3d_indoor_detection mono3d_indoor_detection_pipeline.launch.py

```

**订阅MIPI摄像头发布图片，web端展示**

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash
# config中为示例使用的模型和图片，根据实际安装路径进行拷贝
cp -r install/lib/mono3d_indoor_detection/config/ .

# 默认使用F37摄像头
export CAM_TYPE=mipi

# 启动3D检测 package
ros2 launch mono3d_indoor_detection mono3d_indoor_detection_pipeline.launch.py

```


### **Linux**

```
export ROS_LOG_DIR=/userdata/
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:./install/lib/

# config中为示例使用的模型，根据实际安装路径进行拷贝
cp -r install/lib/mono3d_indoor_detection/config/ .

# 启动3D检测node
./install/lib/mono3d_indoor_detection/mono3d_indoor_detection

```

# 结果分析
当 mono3d_indoor_detection处理完一帧图片数据后会在控制台打印出处理结果：
## X3结果展示

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
以上log截取了一帧的处理结果，结果显示，订阅到的ai msg中的target type即分类结果为trash_can，
同时也给出了trash_can的三维和距离以及旋转角度信息。

示例中读取本地图片推理的结果会渲染到图片上，并且保存在当前目录的result目录下。




# 常见问题
