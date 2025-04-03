import rclpy
from rclpy import time
from rclpy.node import Node

from fairino_msgs.srv import RemoteCmdInterface
from fairino_msgs.msg import RobotNonrtState

import time
from datetime import datetime
import threading
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import asyncio
import math
from geometry_msgs.msg import TransformStamped
import numpy as np

from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import transforms3d as tfs
from .error_list import ErrorListUser
from .func_tf2 import FuncTf2
import struct

#模块号12
class FrStateSub(Node):
    # 初始化函数
    def __init__(self,error:ErrorListUser,funcTf2:FuncTf2):
        super().__init__('FuncFrRos2')

        self.err_log = error
        self.func_tf2 = funcTf2
        cb_group_state = MutuallyExclusiveCallbackGroup()
        #fr 状态话题
        self.fr_state_subscription = (
            self.create_subscription(RobotNonrtState,'/nonrt_state_data',
                                     self.fr_state_callback,10,callback_group=cb_group_state))
        #self.fr_state_subscription
        #本地保留 fr robot状态信息
        self.fr_state_msg = None
        self.fr_prg_state = 0  #fr机器人程序状态
        self.robot_motion_done = 0   #机器人运动到位

        #发送给界面的整型信号数据
        self.signal_list_int=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        #发送给界面的双精度信号数据
        self.signal_list_float = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        #发送信号
        self.send_signal = None

        # 界面选中的工具坐标系
        self.tool_selected = "tool0"  #手臂工具坐标默认值
        # 界面选中的用户坐标系
        self.user_selected = "fr_baseLink"  #手臂基坐标默认值
        # 工作空间 0-关节点动, 2-基坐标系下点动, 4-工具坐标系下点动, 8-工件坐标系下点动
        self.work_space = 0
        #robotIO 列表
        self.robotDI = [False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False]
        self.robotDO = [False,False,False,False,False,False,False,False,False,False,False,False,False,False,False,False]
        self.dic_robotDI = {}  #机器人输入标签
        self.dic_robotDO = {}  #机器人输出标签

    # 从tf中获得当前指定坐标系坐标
    #单位是米m和弧度rad
    #01
    def getTfPos_currentCoordinate(self, tool, user):
        position = []
        try:
            t = self.func_tf2.tf_buffer.lookup_transform(
                user,
                tool,
                rclpy.time.Time())

            # 四元数转固定轴欧拉角
            roll, pitch, yaw = tfs.euler.quat2euler([t.transform.rotation.w,t.transform.rotation.x,
                      t.transform.rotation.y, t.transform.rotation.z], "sxyz")
            #x,y,z单位是m，xRot，yRot,zRot单位是rad
            position = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z,
                        roll, pitch, yaw]
            return True,position
        except TransformException as ex:
            self.err_log.append_err(112502, f'获取指定坐标系tf坐标错误：{ex}')
        except Exception as e :
            self.err_log.append_err(112501, "获取指定坐标系tf坐标错误:"+str(e))
            return False,[]

    #法奥手臂状态回调
    def fr_state_callback(self, msg):
        try:
            # 信息更新到本地
            self.fr_state_msg = msg  #所有信息更新
            self.fr_prg_state = msg.prg_state   #程序运行状态更新
            self.robot_motion_done = msg.robot_motion_done  #机器人运动到位
            # 把选定的 tool0 in fr_baseLink tf发布出去
            self.fr_baseTool_tf_broadcaster(msg)
            #将IO状态保存
            self.DIO_state(msg)
            # 状态更新并发布到界面显示
            self.async_fr_state_send(msg)
        except Exception as e:
            self.err_log.append_err(112501, "fr_state_callback错误:" + str(e))

    #发布手臂的tf2关系
    def fr_baseTool_tf_broadcaster(self,msg):
        t = TransformStamped()
        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'fr_baseLink'
        t.child_frame_id = 'tool0'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.cart_x_cur_pos/1000  #毫米mm单位转化为米单位m
        t.transform.translation.y = msg.cart_y_cur_pos/1000  #毫米mm单位转化为米单位m
        t.transform.translation.z = msg.cart_z_cur_pos/1000  #毫米mm单位转化为米单位m
        x_rot_rad = msg.cart_a_cur_pos * math.pi/180  #角度转弧度
        y_rot_rad = msg.cart_b_cur_pos * math.pi/180  #角度转弧度
        z_rot_rad = msg.cart_c_cur_pos * math.pi/180  #角度转弧度

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = tfs.euler.euler2quat(x_rot_rad,y_rot_rad,z_rot_rad)
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]
        # Send the transformation
        self.func_tf2.tf_broadcaster.sendTransform(t)


    def DIO_state(self,msg):
        #输入 化16位整数
        input_h = msg.dgt_input_h & 0xFF
        input_l = msg.dgt_input_l & 0xFF
        input_data = input_h<<8 | input_l
        #输出 化16位整数
        output_h = msg.dgt_output_h & 0xFF
        output_l = msg.dgt_output_l & 0xFF
        output_data = output_h<<8 | output_l

        # 遍历输入 16位（0到15）
        for i in range(0, 15, 1):  # 从最低位到最高位
            # 使用位操作获取每一位的值，并转换为布尔值
            bit = (input_data >> i) & 1
            self.robotDI[i] = (bool(bit))

        # 遍历输出 16位（0到15）
        for i in range(0, 15, 1):  # 从最低位到最高位
            # 使用位操作获取每一位的值，并转换为布尔值
            bit = (output_data >> i) & 1
            self.robotDO[i] = (bool(bit))
        #构建空标签列表
        tags_input = ["", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""]
        tags_output = ["", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""]
        #字典数据正常
        if len(self.dic_robotDI.keys())==16 and len(self.dic_robotDO.keys())==16 :
            for i in range(0,15,1):
                tags_input[i] = self.dic_robotDI[str(i)]
                tags_output[i] = self.dic_robotDO[str(i)]
        #将输入输出状态回传给界面
        self.send_signal.signal_update_IO_state_robotDIO.emit(self.robotDI,self.robotDO,tags_input,tags_output)

    #数据发送到界面
    def async_fr_state_send(self, msg):
        # 这里可以执行异步操作，例如网络请求或者耗时计算
        try:
            #判断如果界面选择显示的坐标变换不是默认的 工具在基坐标系的关系，则查询tf关系树
            position_temp = [0.0,0.0,0.0,0.0,0.0,0.0]
            position_trans = [0.0,0.0,0.0,0.0,0.0,0.0]
            if self.tool_selected != "tool0" or self.user_selected != "fr_baseLink":
                #获取GUI指定坐标系的变换关系
                res,position_temp = self.getTfPos_currentCoordinate(self.tool_selected,self.user_selected)
                # 转换成mm和角度发送到界面
                if res:
                    position_trans[0] = position_temp[0] * 1000
                    position_trans[1] = position_temp[1] * 1000
                    position_trans[2] = position_temp[2] * 1000
                    position_trans[3] = position_temp[3] * 180 / math.pi
                    position_trans[4] = position_temp[4] * 180 / math.pi
                    position_trans[5] = position_temp[5] * 180 / math.pi

            # 尝试执行的代码

            # 整型数据列表
            self.signal_list_int[0] = msg.prg_state
            self.signal_list_int[1] = msg.error_code
            self.signal_list_int[2] = msg.robot_mode
            self.signal_list_int[3] = msg.tool_num
            self.signal_list_int[4] = msg.dgt_output_h
            self.signal_list_int[5] = msg.dgt_output_l
            self.signal_list_int[6] = msg.tl_dgt_output_l
            self.signal_list_int[7] = msg.dgt_input_h
            self.signal_list_int[8] = msg.dgt_output_l
            self.signal_list_int[9] = msg.tl_dgt_input_l
            self.signal_list_int[10] = msg.emg
            self.signal_list_int[11] = msg.robot_motion_done
            self.signal_list_int[12] = msg.grip_motion_done
            # 发射fr整型列表数据
            self.send_signal.signal_robot_int_state_update.emit(self.signal_list_int)

            # 双精度数据列表
            self.signal_list_float[0] = msg.j1_cur_pos
            self.signal_list_float[1] = msg.j2_cur_pos
            self.signal_list_float[2] = msg.j3_cur_pos
            self.signal_list_float[3] = msg.j4_cur_pos
            self.signal_list_float[4] = msg.j5_cur_pos
            self.signal_list_float[5] = msg.j6_cur_pos
            #笛卡尔坐标 根据界面选择的坐标系，反馈坐标值
            if self.tool_selected != "tool0" or self.user_selected != "fr_baseLink":
                self.signal_list_float[6] = position_trans[0]
                self.signal_list_float[7] = position_trans[1]
                self.signal_list_float[8] = position_trans[2]
                self.signal_list_float[9] = position_trans[3]
                self.signal_list_float[10] = position_trans[4]
                self.signal_list_float[11] = position_trans[5]
            else:
                self.signal_list_float[6] = msg.cart_x_cur_pos
                self.signal_list_float[7] = msg.cart_y_cur_pos
                self.signal_list_float[8] = msg.cart_z_cur_pos
                self.signal_list_float[9] = msg.cart_a_cur_pos
                self.signal_list_float[10] = msg.cart_b_cur_pos
                self.signal_list_float[11] = msg.cart_c_cur_pos

            self.signal_list_float[12] = msg.j1_cur_tor
            self.signal_list_float[13] = msg.j2_cur_tor
            self.signal_list_float[14] = msg.j3_cur_tor
            self.signal_list_float[15] = msg.j4_cur_tor
            self.signal_list_float[16] = msg.j5_cur_tor
            self.signal_list_float[17] = msg.j6_cur_tor
            # 发射fr双精度列表数据
            self.send_signal.signal_robot_float_state_update.emit(self.signal_list_float)

        except:
            # 处理特定异常的代码
            print("数据更新到界面报错")

