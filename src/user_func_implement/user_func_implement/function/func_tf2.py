import rclpy
from rclpy import time
from rclpy.node import Node

import sys
from geometry_msgs.msg import TransformStamped
import numpy as np

import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import transforms3d as tfs
from .error_list import ErrorListUser

#模块12


class FuncTf2(Node):
    # 初始化函数
    # 01
    def __init__(self,error:ErrorListUser):
        super().__init__('FuncTf2')
        # 建立tf监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # tf2静态广播
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        #tf2实时广播
        self.tf_broadcaster = TransformBroadcaster(self)
        #定时发布器
        #self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        now = rclpy.time.Time()
        t = self.tf_buffer.lookup_transform(
            "fr_baseLink",
            "tool0",
            now)

