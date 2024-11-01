# face_age_detection

## 描述

年龄检测功能包

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

```shell
===============================================================================================================================
# 目前只支持离线图片推理，而且只支持设置一个roi区域，多个roi区域存在bug
ros2 launch face_age_detection face_age_det_node.launch.py feed_type:=1 feed_image_path:=图片.png roi_xyxy:=x1,y1,x2,y2
# 例如
ros2 launch face_age_detection face_age_det_node.launch.py feed_type:=1 feed_image_path:=image.png roi_xyxy:=251,242,328,337
===============================================================================================================================
```

## 运行结果

输出检测的年龄

## 功能包参数

| 名称            | 参数值             | 说明                                                 |
| --------------- | ------------------ | ---------------------------------------------------- |
| feed_type       | 0                  | 0：使用相机采集的图像实时推理，1：使用离线图像推理   |
| feed_image_path | ./config/image.png | 输入的图像，png、jpg等格式均可                       |
| roi_xyxy        | ""                 | 图像roi区域，xyxy格式，以字符串格式输入，通过","分割 |

