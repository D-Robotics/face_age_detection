# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    max_slide_window_size_cmd = DeclareLaunchArgument(
        "max_slide_window_size", default_value="30", description="max_slide_window_size"
    )

    # 零拷贝环境配置
    shared_mem_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_shm"), "launch/hobot_shm.launch.py"
            )
        )
    )

    # ros2 run hobot_image_publisher hobot_image_pub --ros-args -p image_source:=./config -p fps:=5 -p output_image_w:=960 -p output_image_h:=544 -p image_format:=jpg -p source_image_w:=960 -p source_image_h:=544
    # 图像需要16bit对齐，例如480*3*8/16=645、736*3*8/16=1104
    img_pub_node = Node(
        package="hobot_image_publisher",
        executable="hobot_image_pub",
        output="screen",
        parameters=[
            {"fps": 5},
            {"image_source": "./face.png"},
            {"image_format": "png"},
            {"source_image_w": 480},
            {"source_image_h": 736},
            {"output_image_w": 480},
            {"output_image_h": 736},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    mono2d_body_det_node = Node(
        package="mono2d_body_detection",
        executable="mono2d_body_detection",
        output="screen",
        parameters=[{"ai_msg_pub_topic_name": "/hobot_mono2d_body_detection"}],
        arguments=["--ros-args", "--log-level", "info"],
    )

    face_age_det_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("face_age_detection"),
                "launch/face_age_det_node.launch.py",
            )
        ),
        launch_arguments={
            "ai_msg_sub_topic_name": "/hobot_mono2d_body_detection",
            "ai_msg_pub_topic_name": "/hobot_face_age_detection",
            "is_shared_mem_sub": "1",
            "max_slide_window_size": LaunchConfiguration("max_slide_window_size"),
            "log_level": "info",
        }.items(),
    )

    # nv12->jpeg
    jpeg_codec_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("hobot_codec"),
                "launch/hobot_codec_encode.launch.py",
            )
        ),
        launch_arguments={
            "codec_sub_topic": "/hbmem_img",
            "codec_pub_topic": "/image",
            "codec_in_mode": "shared_mem",
            "codec_in_format": "nv12",
            "codec_out_mode": "ros",
            "codec_out_format": "jpeg",
        }.items(),
    )

    # web
    web_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("websocket"), "launch/websocket.launch.py"
            )
        ),
        launch_arguments={
            "websocket_image_topic": "/image",
            "websocket_smart_topic": "/hobot_face_age_detection",
        }.items(),
    )

    return LaunchDescription(
        [
            max_slide_window_size_cmd,
            shared_mem_node,
            img_pub_node,
            mono2d_body_det_node,
            face_age_det_node,
            jpeg_codec_node,
            web_node,
        ]
    )
