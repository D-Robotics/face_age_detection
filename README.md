# face_age_detection

## 描述

年龄检测功能包

## 支持平台

| 物料名称        | 生产厂家 | 参考链接                                                                                                                                              |
| :-------------- | -------- | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| RDK X3 / RDK X5 | 多厂家   | [RDK X3](https://developer.d-robotics.cc/rdkx3)<br>[RDK X5](https://developer.d-robotics.cc/rdkx5)                                                    |
| camera          | 多厂家   | [MIPI相机](https://developer.horizon.cc/nodehubdetail/168958376283445781)<br>[USB相机](https://developer.horizon.cc/nodehubdetail/168958376283445777) |

## 编译

- 编译依赖[ai_msgs](https://github.com/D-Robotics/hobot_msgs)
- 编译依赖[hobot_dnn](https://github.com/D-Robotics/hobot_dnn)

```shell
# X5交叉编译
bash build.sh -p X5 -s img_msgs
bash build.sh -p X5 -s hbm_img_msgs
bash build.sh -p X5 -s ai_msgs
bash build.sh -p X5 -s dnn_node
bash build.sh -p X5 -s face_age_detection
```

## 运行指令

1. 启动人体、人脸、人手检测算法和年龄检测算法：

```shell
===============================================================================================================================
# 离线推理
ros2 launch face_age_detection face_age_det_node.launch.py feed_type:=1 feed_image_path:=图片.png roi_xyxy:=x1,y1,x2,y2,x3,y3,x4,y4,...
# 例如
ros2 launch face_age_detection face_age_det_node.launch.py feed_type:=1 feed_image_path:=image.png roi_xyxy:=251,242,328,337
===============================================================================================================================
# 在线推理，需要同时启动身体部分检测和年龄检测节点
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .

# 使用usb相机
export CAM_TYPE=usb
# 使用mipi相机
export CAM_TYPE=mipi

ros2 launch face_age_detection body_det_face_age_det.launch.py
```


2. 启动人体、人脸、人手检测算法，手势识别算法和年龄检测算法，并使用 [tros_perception_fusion node](https://github.com/D-Robotics/tros_perception_common/tree/develop/tros_perception_fusion) 融合所有感知结果：

```shell
source /opt/tros/humble/setup.bash

cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_lmk_detection/config/ .
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_gesture_detection/config/ .

source install/local_setup.bash

# 使用usb相机
export CAM_TYPE=usb
# 使用mipi相机
export CAM_TYPE=mipi

ros2 launch face_age_detection age_gesture_fusion.launch.py max_slide_window_size:=100
```


## 运行结果

输出检测的年龄

## 功能包参数

| 名称                  | 默认参数值           | 说明                                                 |
| --------------------- | -------------------- | ---------------------------------------------------- |
| feed_type             | 0                    | 0：使用相机采集的图像实时推理，1：使用离线图像推理   |
| feed_image_path       | ./config/image.png   | 输入的图像，png、jpg等格式均可                       |
| is_sync_mode          | 0                    | 0：异步推理，1：同步推理                             |
| model_file_name       | ./config/faceAge.hbm | 模型文件                                             |
| is_shared_mem_sub     | 1                    | 0：不使用shared mem通信方式，1：用shared mem通信方式 |
| dump_render_img       | 0                    | 0：不保存渲染图像，1：保存渲染图像                   |
| ai_msg_pub_topic_name | /face_age_detection  | 发布ai_msg的话题名称，只有实时推理会发布             |
| max_slide_window_size | 15                   | 投票队列最大长度                                     |

