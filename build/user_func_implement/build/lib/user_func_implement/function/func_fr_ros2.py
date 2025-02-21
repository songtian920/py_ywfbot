import rclpy
from rclpy import time
from rclpy.node import Node
from frhal_msgs import msg
from frhal_msgs.srv import ROSCmdInterface
from frhal_msgs.msg import FRState
import time
from datetime import datetime
import threading
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
import asyncio
import sqlite3
from sqlite3 import OperationalError
import os
import math
import sys
from geometry_msgs.msg import TransformStamped
import numpy as np

import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import transforms3d as tfs

from .fr_state_sub import FrStateSub
from .error_list import ErrorListUser
from .func_tf2 import FuncTf2

#011号模块
class FuncFrRos2(Node):
    # 初始化函数
    # 01
    def __init__(self,error:ErrorListUser,frState:FrStateSub,funcTf2:FuncTf2):
        super().__init__('FuncFrRos2')
        self.err_log = error
        cb_group1 = MutuallyExclusiveCallbackGroup()
        cb_group2 = MutuallyExclusiveCallbackGroup()
        #fr API接口
        self.cli = self.create_client(ROSCmdInterface,'FR_ROS_API_service',callback_group=cb_group1)
        self.req = ROSCmdInterface.Request()
        self.resp = ROSCmdInterface.Response()

        #本地保留 FrStateSub对象
        self.fr_state = frState
        #tf2监视，发布器
        self.func_tf2 = funcTf2
        #发送给界面的整型信号数据
        self.signal_list_int = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        #发送给界面的双精度信号数据
        self.signal_list_float = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        #点动速度
        self.JOG_vel = 10  #JOG初始速度
        #全局加速度
        self.acc = 30
        #全局速度
        self.speed = 10
        # 手臂模式 1手动  0自动
        self.fr_mode = 0
        # 手臂使能状态 1-机械臂使能,0-机械臂去使能
        self.fr_enable = 0
        #工作空间 0-关节点动, 2-基坐标系下点动, 4-工具坐标系下点动, 8-工件坐标系下点动
        self.work_space=0
        #界面选中的工具坐标系
        self.tool_selected = "tool0"
        #界面选中的用户坐标系
        self.user_selected = "fr_baseLink"
        #发送信号
        self.send_signal = None
        #建立robot点位 字典类型
        self.dic_point = {}
        # 工具坐标系字典
        self.dic_tool = {}
        # 用户坐标系字典
        self.dic_user = {}

        #自定义工具坐标系标定
        self.list_pose_calibTool = [[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0],[0.0,0.0,0.0,0.0,0.0,0.0]]  #默认工具坐标系在基坐标系位姿列表
        self.x_trans_toolCalib = 0.0
        self.y_trans_toolCalib = 0.0
        self.z_trans_toolCalib = 0.0

    #配置
    # 02
    def config(self):
        # 加载点位数据
        if not self.fr_load_pointsFromDB():
            self.err_log.append_err(110201, "fr_load_pointsFromDB加载点位数据错误，请检查fr_data.db是否损坏")
            return False
        # 加载工具坐标系表
        if not self.fr_load_toolsFromDB():
            self.err_log.append_err(110202, "fr_load_toolsFromDB加载工具坐标系数据错误，请检查fr_data.db是否损坏")
            return False
        # 加载用户坐标系表
        if not self.fr_load_usersFromDB():
            self.err_log.append_err(110203,
                                    "fr_load_usersFromDB加载用户坐标系数据错误，请检查fr_data.db是否损坏")
            return False
        # 将工具坐标系tf发布出去
        if not self.tools_tf_static_broadcaster():
            self.err_log.append_err(110204,
                                    "tools_tf_static_broadcaster发布工具坐标系tf数据错误，请检查dic_tool数据是否正确")
            return False
        #将用户坐标系tf发布出去
        if not self.users_tf_static_broadcaster():
            self.err_log.append_err(110205,
                                    "users_tf_static_broadcaster发布用户坐标系tf数据错误，请检查dic_user数据是否正确")
            return False
        # 将点位坐标系tf发布出去
        if not self.points_tf_static_broadcaster():
            self.err_log.append_err(110206,
                                    "points_tf_static_broadcaster发布用户坐标系tf数据错误，请检查dic_points数据是否正确")
            return False

        #将工具坐标系键值发送给GUI RobotManual
        if not self.send_Signal_load_tool_list():
            self.err_log.append_err(110207,
                                    "send_Signal_load_tool_list发布工具坐标系键值发送给GUI RobotManual错误，请检查dic_tool数据是否正确")
            return  False
        # 将用户坐标系键值发送给GUI RobotManual
        if not self.send_Signal_load_user_list():
            self.err_log.append_err(110208,
                                    "send_Signal_load_user_list发布用户坐标系键值发送给GUI RobotManual错误，请检查dic_user数据是否正确")
            return False
        #将点表名称发送给 RobotManual
        if not self.send_Signal_load_pointNameComment_list():
            self.err_log.append_err(110209,
                                    "send_Signal_load_pointName_list发布用户坐标系键值发送给GUI RobotManual错误，请检查dic_point数据是否正确")
            return False

        return True

    # 广播发布坐标系
    #位姿变换为transform
    # 03
    def make_transforms(self, transformation):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = transformation[0]
        t.child_frame_id = transformation[1]

        t.transform.translation.x = float(transformation[2])
        t.transform.translation.y = float(transformation[3])
        t.transform.translation.z = float(transformation[4])

        # 固定轴欧拉角转四元数
        quat = tfs.euler.euler2quat(float(transformation[5]),
                float(transformation[6]), float(transformation[7]), "sxyz")

        t.transform.rotation.w = quat[0]
        t.transform.rotation.x = quat[1]
        t.transform.rotation.y = quat[2]
        t.transform.rotation.z = quat[3]
        return t

    #广播发布所有工具坐标系tf2
    # 04
    def tools_tf_static_broadcaster(self):
        try:
            #遍历工具坐标系字典
            for key in self.dic_tool:
                if key != "tool0": #tool0为默认的工具坐标系不需要发布变换
                    #获取工具坐标系的值列表
                    tool_var = self.dic_tool[key]
                    child_frame = key
                    x = tool_var[1] / 1000   #毫米mm单位转化为米单位m
                    y = tool_var[2] / 1000   #毫米mm单位转化为米单位m
                    z = tool_var[3] / 1000  # 毫米mm单位转化为米单位m
                    x_rot_rad = tool_var[4] * 3.1415926 / 180  #角度转弧度
                    y_rot_rad = tool_var[5] * 3.1415926 / 180  # 角度转弧度
                    z_rot_rad = tool_var[6] * 3.1415926 / 180  # 角度转弧度
                    #转化信息
                    trans_mes=["tool0",child_frame,x,y,z,x_rot_rad,y_rot_rad,z_rot_rad]
                    #生成transform类型
                    transform_tool = self.make_transforms(trans_mes)
                    #将tf2的transform广播发送
                    self.func_tf2.tf_static_broadcaster.sendTransform(transform_tool)

            return True
        except:
            print("tools_tf_static_broadcaster函数执行错误")
            return False

    # 广播发布单个工具坐标系tf2
    # 05
    def single_tool_tf_static_broadcaster(self,key):
        try:
            if key != "tool0":  # tool0为默认的工具坐标系不需要发布变换
                # 获取工具坐标系的值列表
                tool_var = self.dic_tool[key]
                child_frame = key
                x = tool_var[1] / 1000  # 毫米mm单位转化为米单位m
                y = tool_var[2] / 1000  # 毫米mm单位转化为米单位m
                z = tool_var[3] / 1000  # 毫米mm单位转化为米单位m
                x_rot_rad = tool_var[4] * 3.1415926 / 180  # 角度转弧度
                y_rot_rad = tool_var[5] * 3.1415926 / 180  # 角度转弧度
                z_rot_rad = tool_var[6] * 3.1415926 / 180  # 角度转弧度
                # 转化信息
                trans_mes = ["tool0", child_frame, x, y, z, x_rot_rad, y_rot_rad, z_rot_rad]
                # 生成transform类型
                transform_tool = self.make_transforms(trans_mes)
                # 将tf2的transform广播发送
                self.func_tf2.tf_static_broadcaster.sendTransform(transform_tool)
                return True
        except:
            print("single_tool_tf_static_broadcaster 函数执行错误")
            return  False

    # 广播发布所有用户坐标系tf2
    # 06
    def users_tf_static_broadcaster(self):
        try:
            # 遍历工具坐标系字典
            for key in self.dic_user:
                if key != "fr_baseLink":  # tool0为默认的工具坐标系不需要发布变换
                    # 获取工具坐标系的值列表
                    user_var = self.dic_user[key]
                    child_frame = key
                    x = user_var[1] / 1000  # 毫米mm单位转化为米单位m
                    y = user_var[2] / 1000  # 毫米mm单位转化为米单位m
                    z = user_var[3] / 1000  # 毫米mm单位转化为米单位m
                    x_rot_rad = user_var[4] * 3.1415926 / 180  # 角度转弧度
                    y_rot_rad = user_var[5] * 3.1415926 / 180  # 角度转弧度
                    z_rot_rad = user_var[6] * 3.1415926 / 180  # 角度转弧度
                    # 转化信息
                    trans_mes = ["fr_baseLink", child_frame, x, y, z, x_rot_rad, y_rot_rad, z_rot_rad]
                    # 生成transform类型
                    transform_user = self.make_transforms(trans_mes)
                    # 将tf2的transform广播发送
                    self.func_tf2.tf_static_broadcaster.sendTransform(transform_user)
            return True
        except:
            print("users_tf_static_broadcaster 函数执行错误")
            return False

    # 广播发布单个用户坐标系tf2
    # 07
    def single_user_tf_static_broadcaster(self,key):
        try:
            if key != "fr_baseLink":  # tool0为默认的工具坐标系不需要发布变换
                # 获取工具坐标系的值列表
                user_var = self.dic_user[key]
                child_frame = key
                x = user_var[1] / 1000  # 毫米mm单位转化为米单位m
                y = user_var[2] / 1000  # 毫米mm单位转化为米单位m
                z = user_var[3] / 1000  # 毫米mm单位转化为米单位m
                x_rot_rad = user_var[4] * 3.1415926 / 180  # 角度转弧度
                y_rot_rad = user_var[5] * 3.1415926 / 180  # 角度转弧度
                z_rot_rad = user_var[6] * 3.1415926 / 180  # 角度转弧度
                # 转化信息
                trans_mes = ["fr_baseLink", child_frame, x, y, z, x_rot_rad, y_rot_rad, z_rot_rad]
                # 生成transform类型
                transform_user = self.make_transforms(trans_mes)
                # 将tf2的transform广播发送
                self.func_tf2.tf_static_broadcaster.sendTransform(transform_user)
                return True
        except:
            print("single_user_tf_static_broadcaster 函数执行错误")
            return False


    # 广播发布所有点位坐标系tf2
    #
    def points_tf_static_broadcaster(self):
        try:
            # 遍历点位坐标系字典
            for key in self.dic_point:
                # 获取工具坐标系的值列表
                point_var = self.dic_point[key]
                child_frame = key  #子框架是点位名称
                father_frame = point_var[2]  #父框架是user name
                x = point_var[9] / 1000  # 毫米mm单位转化为米单位m
                y = point_var[10] / 1000  # 毫米mm单位转化为米单位m
                z = point_var[11] / 1000  # 毫米mm单位转化为米单位m
                x_rot_rad = point_var[12] * 3.1415926 / 180  # 角度转弧度
                y_rot_rad = point_var[13] * 3.1415926 / 180  # 角度转弧度
                z_rot_rad = point_var[14] * 3.1415926 / 180  # 角度转弧度
                # 转化信息
                trans_mes = [father_frame, child_frame, x, y, z, x_rot_rad, y_rot_rad, z_rot_rad]
                # 生成transform类型
                transform_point = self.make_transforms(trans_mes)
                # 将tf2的transform广播发送
                self.func_tf2.tf_static_broadcaster.sendTransform(transform_point)
            return True
        except:
            print("users_tf_static_broadcaster 函数执行错误")
            return False

    # 给GUI RobotManual发送点表名称
    def send_Signal_load_pointNameComment_list(self):
        try:
            point_names = []  #点位名称
            comments = []  #注释名称
            keys = self.dic_point.keys()
            point_names.extend(list(keys))
            #获取点位注释
            for name in point_names:
                comment = self.dic_point[name][0]
                comments.append(comment)
            self.send_signal.signal_robot_pointsName_list.emit(point_names)  # 发送更新用户坐标系信号
            self.send_signal.signal_robot_pointsComment_list.emit(comments)  # 发送更新用户坐标系信号
            return True
        except:
            print("send_Signal_load_pointName_list 函数执行错误")
            return False


    #给GUI RobotManual发送工具坐标系名称
    def send_Signal_load_tool_list(self):
        try:
            tool_names = ["tool0"]  #默认工具坐标系名称
            keys = self.dic_tool.keys()
            tool_names.extend(list(keys))
            self.send_signal.signal_robot_tool_list.emit(tool_names) #发送更新工具坐标系信号
            return  True
        except:
            print("send_Signal_load_tool_list 函数执行错误")
            return False

    # 给GUI RobotManual发送用户坐标系名称
    def send_Signal_load_user_list(self):
        try:
            user_names = ["fr_baseLink"]  # 默认基坐标系名称
            keys = self.dic_user.keys()
            user_names.extend(list(keys))
            self.send_signal.signal_robot_user_list.emit(user_names)  # 发送更新用户坐标系信号
            return True
        except:
            print("send_Signal_load_user_list 函数执行错误")
            return False

    #获取fr手臂当前的关节角度
    #26
    def getJoints_currentValue(self):
        try:
            return True,[self.fr_state.fr_state_msg.j1_cur_pos,self.fr_state.fr_state_msg.j2_cur_pos,
                    self.fr_state.fr_state_msg.j3_cur_pos,self.fr_state.fr_state_msg.j4_cur_pos,
                    self.fr_state.fr_state_msg.j5_cur_pos,self.fr_state.fr_state_msg.j6_cur_pos]
        except:
            self.err_log.append_err(112601, "getJoints_currentValue获取指定关节角错误，请检查机器人通讯")
            return False,[]


    # 从tf中获得当前指定坐标系坐标
    # 单位是米m和弧度rad
    #25
    def getTfPos_currentCoordinate(self, child_frame, father_frame):
        position = []
        try:
            now = rclpy.time.Time()
            t = self.func_tf2.tf_buffer.lookup_transform(
                father_frame,
                child_frame,
                now)

            # 四元数转固定轴欧拉角
            roll, pitch, yaw = tfs.euler.quat2euler([t.transform.rotation.w,t.transform.rotation.x,
                      t.transform.rotation.y, t.transform.rotation.z], "sxyz")
            #x,y,z单位是m，xRot，yRot,zRot单位是rad
            position = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z,
                        roll, pitch, yaw]
            return True,position
        except TransformException as ex:
            self.err_log.append_err(112502, f'获取指定坐标系tf坐标错误：{ex}')
            return False, []
        except :
            self.err_log.append_err(112501, "获取指定坐标系tf坐标错误，请检查tf中是否存在指定坐标")
            return False,[]

    # 从tf中获得当前指定坐标系变换关系transform
    # 单位是米m和弧度rad
    # 25
    def getTf_currentCoordinate(self, child_frame, father_frame):
        position = []
        try:
            t = self.func_tf2.tf_buffer.lookup_transform(
                father_frame,
                child_frame,
                tf2_ros.Time())
            q = [t.transform.rotation.w,t.transform.rotation.x,
                 t.transform.rotation.y,t.transform.rotation.z]
            t = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
            return  True,t,q
        except TransformException as ex:
            self.err_log.append_err(112502, f'getTf_currentCoordinate获取指定坐标系tf坐标错误：{ex}')
            return  False,None,None
        except:
            self.err_log.append_err(112501, "getTf_currentCoordinate获取指定坐标系tf坐标错误，请检查tf中是否存在指定坐标")
            return  False,None,None

    # GUI和数据库操作
    # GUI单行点表更新到 数据库 本地点位字典
    # 08
    def updateDbRow_robotPointList(self,name_selected,list_var):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + '/user_func_implement/user_func_implement/sqlite/fr_data.db')
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 执行更新语句，更新user表的数据:
            update_stmt='UPDATE fr_PointData SET comment=?,tool=?,user=?,joint1=?,joint2=?,joint3=?,joint4=?,joint5=?,joint6=?,cart_x=?,cart_y=?,cart_z=?,cart_xRot=?,cart_yRot=?,cart_zRot=? WHERE name = ?'
            comment=str(list_var[0])
            tool = str(list_var[1])
            user = str(list_var[2])
            joint1 = float(list_var[3])
            joint2 = float(list_var[4])
            joint3 = float(list_var[5])
            joint4 = float(list_var[6])
            joint5 = float(list_var[7])
            joint6 = float(list_var[8])
            x = float(list_var[9])
            y = float(list_var[10])
            z = float(list_var[11])
            x_rot = float(list_var[12])
            y_rot = float(list_var[13])
            z_rot = float(list_var[14])

            update_values=(comment,tool,user,joint1,joint2,joint3,joint4,joint5,joint6,
                           x,y,z,x_rot,y_rot,z_rot,name_selected)
            cursor.execute(update_stmt, update_values)
            conn.commit()
            # 更新到点表字典 dic_point
            self.dic_point[name_selected] = [comment, tool, user, joint1, joint2, joint3, joint4, joint5, joint6,
                                             x, y, z, x_rot, y_rot, z_rot]
            return True, ""

        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(110801, str(o))
            return False, str(o)
        except Exception as e:
            print(e)
            self.err_log.append_err(110802, str(e))
            return False, str(e)
        except:
            print("fr_ros2：机器人删除点位错误")
            self.err_log.append_err(110803, "fr_ros2：机器人删除点位错误")
            return False, "fr_ros2：机器人删除点位错误"
        finally:
            cursor.close()
            conn.close()

    # 删除数据库点表一行
    # 09
    def deleteDbRow_robotPointList(self,name_str):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + '/user_func_implement/user_func_implement/sqlite/fr_data.db')
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 读取表格信息
            cursor.execute("delete from fr_PointData where name= ?;",(name_str,))
            conn.commit()
            return True,""

        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(110901, str(o))
            return False,str(o)
        except Exception as e:
            print(e)
            self.err_log.append_err(110902, str(e))
            return False,str(e)
        except:
            print("fr_ros2：机器人删除点位错误")
            self.err_log.append_err(110903, "fr_ros2：机器人删除点位错误")
            return False,"fr_ros2：机器人删除点位错误"
        finally:
            cursor.close()
            conn.close()

    #插入新的一行到数据库，并更新到内存
    # 10
    def insert_newRowInRobotPointList(self,name_str):
        #路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 =os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2+'/user_func_implement/user_func_implement/sqlite/fr_data.db')
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 读取表格信息
            cursor.execute('INSERT INTO fr_PointData (name) VALUES (?)', (name_str,))
            conn.commit()
            #发送信号给GUI添加一行
            self.send_signal.signal_insertNewRowToTM_DB_frPoint.emit(name_str)
            return True,""

        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111001, str(o))
            return False,str(o)
        except Exception as e:
            print(e)
            self.err_log.append_err(111002, str(e))
            return False,str(e)
        except:
            print("fr_load_pointsFromDB：加载机器人点位错误")
            self.err_log.append_err(111003, "fr_load_pointsFromDB：加载机器人点位错误")
            return False,"fr_load_pointsFromDB：加载机器人点位错误"
        finally:
            cursor.close()
            conn.close()

    # sqlite3加载点位数据 到tableView ，加载到内存字典，注册到fr_ros2服务器
    # 11
    def fr_load_pointsFromDB(self):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)

        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + '/user_func_implement/user_func_implement/sqlite/fr_data.db')
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            #读取表格信息
            cursor.execute('SELECT * FROM fr_PointData')
            #获取所有值
            values = cursor.fetchall()
            #点表字典清空
            self.dic_point.clear()
            #获取的每行数据添加到点表，更新到tableView
            for val in values:
                dic_name = ''
                data_list=[]
                #点表数据逐行添加到本地 字典
                dic_name=val[0]  #数据库中的name
                data_list.append(val[1])  #数据库中的comment注释
                data_list.append(val[2])  # 数据库中的tool
                data_list.append(val[3])  # 数据库中的user
                data_list.append(val[4])  # 数据库中的joint1
                data_list.append(val[5])  # 数据库中的joint2
                data_list.append(val[6])  # 数据库中的joint3
                data_list.append(val[7])  # 数据库中的joint4
                data_list.append(val[8])  # 数据库中的joint5
                data_list.append(val[9])  # 数据库中的joint6
                data_list.append(val[10])  # 数据库中的cart_x
                data_list.append(val[11])  # 数据库中的cart_y
                data_list.append(val[12])  # 数据库中的cart_z
                data_list.append(val[13])  # 数据库中的cart_xRot
                data_list.append(val[14])  # 数据库中的cart_yRot
                data_list.append(val[15])  # 数据库中的cart_zRot
                self.dic_point[dic_name]=data_list #点表字典添加值

                # 点表数据逐行添加到界面
                #凑成列表
                list_var=[str(val[0]),str(val[1]),str(val[2]) ,str(val[3]),str(val[4]),
                str(val[5]),str(val[6]),str(val[7]),str(val[8]),str(val[9]),str(val[10]),
                str(val[11]),str(val[12]),str(val[13]),str(val[14]),str(val[15])]
                # 发送信号给GUI RobotPointsTable添加一行
                self.send_signal.signal_appendRow_tableModel_frPoint.emit(list_var)
            return True
        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111101, str(o))
            return False
        except Exception as e:
            print(e)
            self.err_log.append_err(111102, str(e))
            return False
        except:
            print("fr_load_pointsFromDB：加载机器人点位错误")
            self.err_log.append_err(111102, "fr_load_pointsFromDB：加载机器人点位错误")
            return False
        finally:
            cursor.close()
            conn.close()

    # GUI单行工具坐标表更新到 数据库 本地工具坐标系字典
    # 12
    def updateDbRow_robotToolList(self, name_selected, list_var):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + '/user_func_implement/user_func_implement/sqlite/fr_data.db')
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:

            # 执行更新语句，更新user表的数据:
            update_stmt = 'UPDATE fr_ToolData SET comment=?,cart_x=?,cart_y=?,cart_z=?,cart_xRot=?,cart_yRot=?,cart_zRot=? WHERE name = ?'
            comment = str(list_var[0])
            x = float(list_var[1])
            y = float(list_var[2])
            z = float(list_var[3])
            x_rot = float(list_var[4])
            y_rot = float(list_var[5])
            z_rot = float(list_var[6])

            update_values = (comment,x, y, z, x_rot, y_rot, z_rot,name_selected)
            cursor.execute(update_stmt, update_values)
            conn.commit()
            # 更新到本地工具坐标系表
            self.dic_tool[name_selected] = [comment, x, y, z, x_rot, y_rot, z_rot]
            return True, ""

        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111201, str(o))
            return False, str(o)
        except Exception as e:
            print(e)
            self.err_log.append_err(111202, str(e))
            return False, str(e)
        except:
            print("fr_ros2：机器人删除工具坐标错误")
            self.err_log.append_err(111203, "fr_ros2：机器人删除工具坐标错误")
            return False, "fr_ros2：机器人删除工具坐标错误"
        finally:
            cursor.close()
            conn.close()

    # 删除数据库点表一行
    # 13
    def deleteDbRow_robotToolList(self, name_str):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + '/user_func_implement/user_func_implement/sqlite/fr_data.db')
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 读取表格信息
            cursor.execute("delete from fr_ToolData where name= ?;", (name_str,))
            conn.commit()
            return True, ""

        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111301, str(o))
            return False, str(o)
        except Exception as e:
            print(e)
            self.err_log.append_err(111302, str(e))
            return False, str(e)
        except:
            print("fr_ros2：机器人删除工具坐标错误")
            self.err_log.append_err(111303, "fr_ros2：机器人删除工具坐标错误")
            return False, "fr_ros2：机器人删除工具坐标错误"
        finally:
            cursor.close()
            conn.close()

    # 插入新的一行到数据库，并更新到内存
    # 14
    def insert_newRowInRobotToolList(self, name_str):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + '/user_func_implement/user_func_implement/sqlite/fr_data.db')
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 读取表格信息
            cursor.execute('INSERT INTO fr_ToolData (name) VALUES (?)', (name_str,))
            conn.commit()
            # 发送信号给GUI添加一行
            self.send_signal.signal_insertNewRowToTM_DB_frTool.emit(name_str)
            return True, ""

        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111401, str(o))
            return False, str(o)
        except Exception as e:
            print(e)
            self.err_log.append_err(111402, str(e))
            return False, str(e)
        except:
            print("fr_load_pointsFromDB：加载机器人工具坐标错误")
            self.err_log.append_err(111403, "fr_load_pointsFromDB：加载机器人工具坐标错误")
            return False, "fr_load_pointsFromDB：加载机器人工具坐标错误"
        finally:
            cursor.close()
            conn.close()

    #从数据库加载工具坐标系表
    # 15
    def fr_load_toolsFromDB(self):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)

        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + '/user_func_implement/user_func_implement/sqlite/fr_data.db')
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 读取表格信息
            cursor.execute('SELECT * FROM fr_ToolData')
            # 获取所有值
            values = cursor.fetchall()
            # 点表字典清空
            self.dic_tool.clear()
            # 获取的每行数据添加到点表，更新到tableView
            for val in values:
                dic_name = ''
                data_list = []
                # 点表数据逐行添加到本地 字典
                dic_name = val[0]  # 数据库中的name
                data_list.append(val[1])  # 数据库中的comment注释
                data_list.append(val[2])  # 数据库中的cart_x
                data_list.append(val[3])  # 数据库中的cart_y
                data_list.append(val[4])  # 数据库中的cart_z
                data_list.append(val[5])  # 数据库中的cart_xRot
                data_list.append(val[6])  # 数据库中的cart_yRot
                data_list.append(val[7])  # 数据库中的cart_zRot
                self.dic_tool[dic_name] = data_list  # 点表字典添加值

                # 点表数据逐行添加到界面
                # 凑成列表
                list_var = [str(val[0]), str(val[1]), str(val[2]), str(val[3]), str(val[4]),
                            str(val[5]), str(val[6]), str(val[7])]
                # 发送信号给GUI添加一行
                self.send_signal.signal_appendRow_tableModel_frTool.emit(list_var)

            return True
        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111501, str(o))
            return False
        except Exception as e:
            print(e)
            self.err_log.append_err(111502, str(e))
            return False
        except:
            print("fr_load_pointsFromDB：加载机器人点位错误")
            self.err_log.append_err(111503, "fr_load_pointsFromDB：加载机器人点位错误")
            return False
        finally:
            cursor.close()
            conn.close()

    #工具坐标系 示教点位
    #todo
    def teach_point_calibTool(self,point_index):
        res,pose = self.getTfPos_currentCoordinate("tool0","fr_baseLink")
        if res:
            self.list_pose_calibTool[point_index] = pose
            return True
        else:
            self.err_log.append_err(111601, "工具坐标系获取当前坐标失败，请检查机器人当前坐标是否正常")
            return False

    # GUI单行用户坐标表更新到 数据库  本地用户坐标系字典
    #16
    def updateDbRow_robotUserList(self, name_selected, list_var):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + '/user_func_implement/user_func_implement/sqlite/fr_data.db')
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:

            # 执行更新语句，更新user表的数据:
            update_stmt = 'UPDATE fr_UserData SET comment=?,cart_x=?,cart_y=?,cart_z=?,cart_xRot=?,cart_yRot=?,cart_zRot=? WHERE name = ?'
            comment = str(list_var[0])
            x = float(list_var[1])
            y = float(list_var[2])
            z = float(list_var[3])
            x_rot = float(list_var[4])
            y_rot = float(list_var[5])
            z_rot = float(list_var[6])

            update_values = (comment, x, y, z, x_rot, y_rot, z_rot, name_selected)
            cursor.execute(update_stmt, update_values)
            conn.commit()
            # 更新到本地用户坐标系表
            self.dic_user[name_selected] = [comment, x, y, z, x_rot, y_rot, z_rot]
            return True

        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111601, str(o))
            return False
        except Exception as e:
            print(str(e))
            self.err_log.append_err(111602, str(e))
            return False
        except:
            print("fr_ros2：机器人删除工具坐标错误")
            self.err_log.append_err(111603, "fr_ros2：机器人删除工具坐标错误")
            return False
        finally:
            cursor.close()
            conn.close()

    # 删除数据库点表一行
    # 17
    def deleteDbRow_robotUserList(self, name_str):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + '/user_func_implement/user_func_implement/sqlite/fr_data.db')
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 读取表格信息
            cursor.execute("delete from fr_UserData where name= ?;", (name_str,))
            conn.commit()
            return True

        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111701, str(o))
            return False
        except Exception as e:
            print(str(e))
            self.err_log.append_err(111702, str(e))
            return False
        except:
            print("fr_ros2：机器人删除工具坐标错误")
            self.err_log.append_err(111703, "fr_ros2：机器人删除工具坐标错误")
            return False
        finally:
            cursor.close()
            conn.close()

    # 插入新的一行到数据库，并更新到内存
    #18
    def insert_newRowInRobotUserList(self, name_str):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + '/user_func_implement/user_func_implement/sqlite/fr_data.db')
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 读取表格信息
            cursor.execute('INSERT INTO fr_UserData (name) VALUES (?)', (name_str,))
            conn.commit()
            # 发送信号给GUI添加一行
            self.send_signal.signal_insertNewRowToTM_DB_frUser.emit(name_str)
            return True

        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111801, str(o))
            return False
        except Exception as e:
            print(str(e))
            self.err_log.append_err(111802, str(e))
            return False
        except:
            print("fr_load_pointsFromDB：加载机器人工具坐标错误")
            self.err_log.append_err(111803, "fr_load_pointsFromDB：加载机器人工具坐标错误")
            return False
        finally:
            cursor.close()
            conn.close()

    # 从数据库加载工具坐标系表
    #19
    def fr_load_usersFromDB(self):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)

        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + '/user_func_implement/user_func_implement/sqlite/fr_data.db')
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 读取表格信息
            cursor.execute('SELECT * FROM fr_UserData')
            # 获取所有值
            values = cursor.fetchall()
            # 点表字典清空
            self.dic_user.clear()
            # 获取的每行数据添加到点表，更新到tableView
            for val in values:
                dic_name = ''
                data_list = []
                # 点表数据逐行添加到本地 字典
                dic_name = val[0]  # 数据库中的name
                data_list.append(val[1])  # 数据库中的comment注释
                data_list.append(val[2])  # 数据库中的cart_x
                data_list.append(val[3])  # 数据库中的cart_y
                data_list.append(val[4])  # 数据库中的cart_z
                data_list.append(val[5])  # 数据库中的cart_xRot
                data_list.append(val[6])  # 数据库中的cart_yRot
                data_list.append(val[7])  # 数据库中的cart_zRot
                self.dic_user[dic_name] = data_list  # 点表字典添加值

                # 点表数据逐行添加到界面
                # 凑成列表
                list_var = [str(val[0]), str(val[1]), str(val[2]), str(val[3]), str(val[4]),
                            str(val[5]), str(val[6]), str(val[7])]
                # 发送信号给GUI添加一行
                self.send_signal.signal_appendRow_tableModel_frUser.emit(list_var)
            return True
        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111901, str(o))
            return False
        except Exception as e:
            print(str(e))
            self.err_log.append_err(111902, str(e))
            return False
        except:
            print("fr_load_pointsFromDB：加载机器人点位错误")
            self.err_log.append_err(111903, "fr_load_pointsFromDB：加载机器人点位错误")
            return False
        finally:
            cursor.close()
            conn.close()

    #工具坐标系的标定
    #用户通过执行器末端对针，围绕针尖至少示教出3个点以上，将默认工具坐标系坐标传入，计算出用户自定义的新工具坐标的平移变化
    #todo
    def calib_translate_tool(self,poseList_toolInBase):
        # 变量声明
        # 位姿的旋转矩阵数组
        T_rot_toolInBase = []
        # 位姿的平移矩阵
        T_trans_toolInBase = []

        # 先判断位姿数量是否大于等于3个
        count_pose = len(poseList_toolInBase)
        if count_pose < 3:
            self.err_log.append_err(111903,
                                    "calib_tool错误：输入的位姿数据少于3个，请至少示教3个以上点位")
            return False
        # 遍历计算每组位姿，计算出相应的矩阵
        for pose in poseList_toolInBase:
            # 平移变换矩阵添加到矩阵列表
            T_trans_ = np.array([pose[0] , pose[1] , pose[2] ])  # 单位转换
            T_trans_toolInBase.append(T_trans_)  # 添加到列表
            # 计算出旋转矩阵
            # 欧拉角转四元数
            q_ = tfs.euler.euler2quat(pose[3] , pose[4] ,pose[5] )  # 转换为弧度单位计算
            # 四元数转成旋转矩阵
            T_rot_ = tfs.quaternions.quat2mat(q_)
            # 旋转矩阵列表添加
            T_rot_toolInBase.append(T_rot_)

        # 配置 相邻点位 旋转矩阵差 T_rot_(i-1) - T_rot_i
        T_rot_sub = None
        T_trans_sub = None
        for i in range(count_pose):
            if i == 1:
                T_rot_sub = np.array(T_rot_toolInBase[i - 1]) - np.array(T_rot_toolInBase[i])
                T_trans_sub = np.array(T_trans_toolInBase[i]) - np.array(T_trans_toolInBase[i - 1])
            elif i>1:
                T_rot_sub = np.vstack((T_rot_sub, np.array(T_rot_toolInBase[i - 1]) - np.array(T_rot_toolInBase[i])))
                T_trans_sub = np.hstack((T_trans_sub,np.array(T_trans_toolInBase[i]) - np.array(T_trans_toolInBase[i - 1])))

        # T_rot_sub的转置T_rot_sub_transpose
        T_rot_sub_transpose = T_rot_sub.transpose()
        # 最小二乘法 T = (AT.A){-1}.AT.b
        # 这里，AT 是 A 的转置，而 (AT A){-1} 是 AT A 的逆矩阵
        # T0 = AT.A  T1 = [AT.A]-1  矩阵A转置乘A，再求逆
        T0 = np.dot(T_rot_sub_transpose, T_rot_sub)
        T1 = np.linalg.inv(T0)
        # T2 = AT.b
        T2 = np.dot(T_rot_sub_transpose, T_trans_sub)
        # T3 = T1.T2 T3为求出的 新工具坐标系 在旧坐标系的平移 变换
        T3 = np.dot(T1, T2)
        # 计算的平移变化 保存在对象变量
        self.x_trans_toolCalib = T3[0]
        self.y_trans_toolCalib = T3[1]
        self.z_trans_toolCalib = T3[2]
        # 返回新工具坐标系在原始坐标系的 平移值
        return self.x_trans_toolCalib, self.y_trans_toolCalib, self.z_trans_toolCalib

    # 用户坐标系的标定
    #新工具姿势保持不变，标定出三个点，第一个点是新用户坐标系原点B_uo，
    # 第二个点是x轴正方向上一任意点B_ux，第三个点为xy平面上接近y轴的点B_uxy
    #这三个点构成了代表用户坐标轴x的向量V_ux，代表用户坐标 xy轴平面内的接近y轴向量V_uxy
    #通过叉乘 可以得到垂直于用户轴xy平面的向量，此向量为用户坐标z轴的向量，V_uz = V_ux 差乘 V_uxy
    #然后通过叉乘 得到 V_uy = V_uz x V_ux(叉乘方向右手螺旋定则)
    #求出各轴单位向量 V_ux_unit, V_uy_unit, V_uz_unit，
    # 根据原点point_uo,计算出用户坐标系三个轴向单位向量在基坐标系中的坐标
    #然后通过三点在两个坐标系的坐标构建4x4正规矩阵，TU = [Ux,Uy,Uz,1→] TB = [Bx,By,Bz,1→]
    #定义的用户坐标系在基坐标系的齐次变换矩阵 T_userInBase
    #公式  TB = T_userInBase x TU 推出 T_userInBase = TB x TU_inverse
    def calib_user_coord(self,pointList_toolInBase):
        # 先判断位姿数量是否等于3个
        count_pose = len(pointList_toolInBase)
        if count_pose != 4:
            self.err_log.append_err(111903,
                                    "calib_user_coord错误：输入的位姿数据不等于3个，请示教3个点位")
            return False
        #构建矩阵
        point_uo = np.array(pointList_toolInBase[0])  # o点在基坐标系位姿
        point_ux = np.array(pointList_toolInBase[1])  # x点在基坐标系位姿
        point_uxy = np.array(pointList_toolInBase[2])  # y点在基坐标系位姿
        #代表用户坐标轴x的向量V_ux（注意此时向量空间在基坐标系）
        V_ux = point_ux - point_uo
        #代表用户坐标 xy轴平面内的任意向量V_uxy（注意此时向量空间在基坐标系）
        V_uxy = point_uxy - point_uo
        #通过叉乘 可以得到垂直于用户轴xy平面的向量，此向量为用户坐标z轴的向量，V_uz = V_ux 差乘 V_uxy(注意叉乘方向)
        V_uz = np.cross(V_ux, V_uxy)
        #此时已知用户坐标系上 x轴上的向量V_ux和z轴上的向量V_uz，则再通过叉乘得到y轴上的V_uy向量(注意叉乘方向)
        V_uy = np.cross(V_uz, V_ux)
        #此时分别求出了x,y,z轴上的向量，然后分别求出各轴上单位向量
        V_ux_unit = V_ux / np.linalg.norm(V_ux)
        V_uy_unit = V_uy / np.linalg.norm(V_uy)
        V_uz_unit = V_uz / np.linalg.norm(V_uz)
        #根据原点point_uo,计算出用户坐标系三个轴向单位向量在基坐标系中的坐标
        point_ux_unit = V_ux_unit + point_uo
        point_uy_unit = V_uy_unit + point_uo
        point_uz_unit = V_uz_unit + point_uo
        #构建用户坐标系U的单位正规方程，做转置
        TU = np.array([[1.0, 0.0, 0.0, 1.0],[0.0, 1.0, 0.0, 1.0],[0.0, 0.0, 1.0, 1.0],[0.0, 0.0, 0.0, 1.0]]).transpose()
        #构建基坐标系三个轴单位 点坐标矩阵 做转置
        T_1 = np.array([point_ux_unit,point_uy_unit,point_uz_unit]).transpose()
        T_2 = np.vstack((T_1,[1.0,1.0,1.0]))
        T_3 = np.array([0.0, 0.0, 0.0, 1.0]).transpose()
        TB = np.hstack((T_2, T_3))
        #计算出用户坐标系在极坐标系的变换
        T_userInBase = np.dot(TB,np.linalg.inv(TU))
        print("结束")
        pass

    # 函数功能描述:机器人点动
    # uint8_t ref - 0-关节点动, 2-基坐标系下点动, 4-工具坐标系下点动, 8-工件坐标系下点动
    # uint8_t nb - 1-关节1(或x轴),2-关节2(或y轴),3-关节3(或z轴),4-关节4(或绕x轴旋转),5-关节5(或绕y轴旋转),6-关节6(或绕z轴旋转)
    # uint8_t dir - 0-负方向, 1-正方向
    # float vel - 速度百分比, 范围为0-100
    #20
    def StartJOG(self,nb,direct):
        #pass
        self.async_StartJOG(nb,direct)

    #21
    def async_StartJOG(self,nb,direct):
        try:
            #todo 点动目前借助于法奥的的点动，工具坐标系及用户坐标系点动未实现，后续借助moveit2实现轨迹点动
            # 定义命令
            work_space_temp = 0
            if self.work_space == 0:
                work_space_temp = 0
            else:
                work_space_temp = 2

            cmd_str = "StartJOG(" + str(work_space_temp) + ',' + str(nb) + ',' + str(direct) + ',' + str(
                self.JOG_vel) + ')'
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(112302, "StartJOG，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future,timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res !='1' and result.cmd_res !='0' :
                self.err_log.append_err(112103, "StartJOG，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False
            # time.sleep(0.5)
        except:
            # 处理特定异常的代码
            print("StartJOG命令报错")
            self.err_log.append_err(112101, "StartJOG命令报错")


    #停止点动
    #22
    def StopJOG(self):
        #pass
        self.async_StopJOG()

    #23
    def async_StopJOG(self):
        try:
            # todo 点动目前借助于法奥的的点动，工具坐标系及用户坐标系点动未实现，后续借助moveit2实现轨迹点动
            # 定义命令
            work_space_temp = 0
            if self.work_space == 0:
                work_space_temp = 0
            else:
                work_space_temp = 1
            cmd_str = "StopJOG(" + str(work_space_temp) + ')'
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(112301, "StopJOG，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future,timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res !='1' and result.cmd_res !='0':
                self.err_log.append_err(112302, "StopJOG，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False
            return True
        except:
            # 处理特定异常的代码
            print("StopJOG命令报错")
            self.err_log.append_err(112301, "StopJOG命令报错")
            return False


    #关节移动指令
    #24
    def MoveJoint(self,point_name,speed = 30,acc = 30,blendT = -1,offset_j1=0,offset_j2=0,
                    offset_j3=0,offset_j4=0,offset_j5=0,offset_j6=0):
        try:
            point_values = self.dic_point[point_name]
            tool = point_values[1]
            user = point_values[2]
            j1 = point_values[3] + offset_j1
            j2 = point_values[4] + offset_j2
            j3 = point_values[5] + offset_j3
            j4 = point_values[6] + offset_j4
            j5 = point_values[7] + offset_j5
            j6 = point_values[8] + offset_j6

            # 定义命令
            cmd_str = ("MoveJoint_ywf(" + str(j1) + "," + str(j2) + "," + str(j3) + "," +
                       str(j4) + "," +str(j5) + "," + str(j6) + "," +
                       str(speed) + "," + str(acc) + "," + str(blendT) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(112403, "MoveJoint，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future,timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res !='1' and result.cmd_res !='0' :
                self.err_log.append_err(112401, "MoveJoint，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            #等待运动结束
            count_motion_done = 0
            while True:
                if self.fr_state.robot_motion_done == 1:
                    break
                else:
                    if count_motion_done>1000:
                        count_motion_done += 1
                        self.err_log.append_err(112401,
                                                "MoveJoint，等待motion done超时报警，超过100s未等到robot_motion_done信号")
                        return False
                    time.sleep(0.1)
            #运动结束 返回真
            return True
        except KeyError:
            print("MoveJoint指令错误，点位名称在字典中不存在")
            self.err_log.append_err(112403,"MoveJoint指令错误，点位名称在字典中不存在")
            return False

        except:
            # 处理特定异常的代码
            print("MoveJoint指令错误，请检查参数")
            self.err_log.append_err(112402, "MoveJoint指令错误，请检查参数")
            return False

    # 关节直线移动指令
    # 24
    def MoveL_Joint(self, point_name, speed=30, acc=30, blendT=-1, offset_j1=0, offset_j2=0,
                  offset_j3=0, offset_j4=0, offset_j5=0, offset_j6=0):
        try:
            point_values = self.dic_point[point_name]
            tool = point_values[1]
            user = point_values[2]
            j1 = point_values[3] + offset_j1
            j2 = point_values[4] + offset_j2
            j3 = point_values[5] + offset_j3
            j4 = point_values[6] + offset_j4
            j5 = point_values[7] + offset_j5
            j6 = point_values[8] + offset_j6

            # 定义命令
            cmd_str = ("MoveL_Joint_ywf(" + str(j1) + "," + str(j2) + "," + str(j3) + "," +
                       str(j4) + "," + str(j5) + "," + str(j6) + "," +
                       str(speed) + "," + str(acc) + "," + str(blendT) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(112403, "MoveL_Joint，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(112401,
                                        "MoveL_Joint，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False
            # 等待运动结束
            count_motion_done = 0
            while True:
                if self.fr_state.robot_motion_done == 1:
                    break
                else:
                    if count_motion_done > 1000:
                        count_motion_done += 1
                        self.err_log.append_err(112401,
                                                "MoveL_Joint，等待motion done超时报警，超过100s未等到robot_motion_done信号")
                        return False
                    time.sleep(0.1)
            # 运动结束 返回真
            return True

        except KeyError:
            print("MoveL_Joint指令错误，点位名称在字典中不存在")
            self.err_log.append_err(112403, "MoveL_Joint指令错误，点位名称在字典中不存在")
            return False

        except:
            # 处理特定异常的代码
            print("MoveL_Joint指令错误，请检查参数")
            self.err_log.append_err(112402, "MoveL_Joint指令错误，请检查参数")
            return False

    # 关节移动指令
    #
    # 24
    def MovePose(self, point_name, speed=30, acc=30, blendT=-1, offset_x=0, offset_y=0,
                  offset_z=0, offset_xRot=0, offset_yRot=0, offset_zRot=0):
        try:
            #将数据从tf中获取并转换坐标系之间的变换
            #从点表中获取点位信息
            point_values = self.dic_point[point_name]
            tool = point_values[1]
            user = point_values[2]
            x = point_values[9] + offset_x
            y = point_values[10] + offset_y
            z = point_values[11] + offset_z
            x_rot = point_values[12] + offset_xRot
            y_rot = point_values[13] + offset_yRot
            z_rot = point_values[14] + offset_zRot

            #点位point在用户坐标的变换
            x_m = x/1000.0  #单位毫米转米
            y_m = y/1000.0  #单位毫米转米
            z_m = z/1000.0  #单位毫米转米
            x_rot_rad = x_rot * 3.1415926 / 180   #角度转弧度
            y_rot_rad = y_rot * 3.1415926 / 180   #角度转弧度
            z_rot_rad = z_rot * 3.1415926 / 180  # 角度转弧度
            #欧拉角转四元数
            q_user_tool = tfs.euler.euler2quat(x_rot_rad,y_rot_rad,z_rot_rad)
            #将四元数转旋转矩阵
            R_user_tool = tfs.quaternions.quat2mat(q_user_tool)
            #第一步point点数据就是 点位指定user到指定的tool的变换矩阵T_user_tool
            T_user_tool = tfs.affines.compose([x_m,y_m,z_m],R_user_tool,[1,1,1])

            # 从tf树中获取基坐标base和user的关系
            res_b_u,t_b_u,q_base_user = self.getTf_currentCoordinate(user, "fr_baseLink")
            if not res_b_u:
                self.err_log.append_err(112404, f"MovePose指令错误，未获取到{user}和 fr_baseLink 的tf关系，检查是否定义用户坐标系")
            #将四元数q_base_user转换为旋转矩阵
            R_base_user = tfs.quaternions.quat2mat(q_base_user)
            #第二步 将base到user的平移变换和旋转变换合成 齐次变换矩阵
            T_base_user = tfs.affines.compose(t_b_u,R_base_user,[1,1,1])

            #从tf树读取点位指定tool到默认工具坐标系tool0的变换
            res_tool,t_tool,q_tool = self.getTf_currentCoordinate(tool, "tool0")
            if not res_tool:
                self.err_log.append_err(112404,
                                        f"MovePose指令错误，未获取到{tool}和 tool0 的tf关系，检查是否定义工具坐标系")
            #将q_tool四元数转换为旋转矩阵
            R_tool0_tool = tfs.quaternions.quat2mat(q_tool)
            #第三步将tool0到指定tool的齐次变换矩阵合成，然后求出逆矩阵 T_tool_tool0
            T_tool0_tool = tfs.affines.compose(t_tool,R_tool0_tool,[1,1,1])
            T_tool_tool0 = np.linalg.inv(T_tool0_tool)  #求出逆矩阵 T_tool_tool0
            #第四步 计算 机器人指定工具坐标系运动到点位时 tool0 在 fr_baseLink的齐次矩阵
            T_base_tool0 =  np.dot(T_base_user,np.dot(T_user_tool,T_tool_tool0))
            #齐次矩阵，转换成 平移量及欧拉角
            xRot_base_tool0, yRot_base_tool0, zRot_base_tool0 = tfs.euler.mat2euler(T_base_tool0[0:3, 0:3])
            T_base_tool0 = T_base_tool0[0:3, 3:4]  #获取平移矩阵
            x_trans = T_base_tool0[0]  #x平移量
            y_trans = T_base_tool0[1]  #y平移量
            z_trans = T_base_tool0[2]  #z平移量
            #将平移转换为mm，角度转换为弧度
            x_trans_mm = x_trans[0] * 1000.0
            y_trans_mm = y_trans[0] * 1000.0
            z_trans_mm = z_trans[0] * 1000.0
            xRot_angle = xRot_base_tool0 * 180 / 3.1415926
            yRot_angle = yRot_base_tool0 * 180 / 3.1415926
            zRot_angle = zRot_base_tool0 * 180 / 3.1415926

            #将tool0在base的坐标变换发送给命令
            # 定义命令
            cmd_str = ("MovePose_ywf(" + str(x_trans_mm) + "," + str(y_trans_mm) + "," + str(z_trans_mm) + "," +
                       str(xRot_angle) + "," + str(yRot_angle) + "," + str(zRot_angle) + "," +
                       str(speed) + "," + str(acc) + "," + str(blendT) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(112403, "MovePose，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(112401,
                                        "MovePose，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False
            # 等待运动结束
            count_motion_done = 0
            while True:
                if self.fr_state.robot_motion_done == 1:
                    break
                else:
                    count_motion_done += 1
                    if count_motion_done > 1000:
                        self.err_log.append_err(112401,
                                                "MovePose，等待motion done超时报警，超过100s未等到robot_motion_done信号")
                        return False
                    time.sleep(0.1)
            # 运动结束 返回真
            return True

        except KeyError:
            print("MovePose指令错误，点位名称在字典中不存在")
            self.err_log.append_err(112403, "MovePose指令错误，点位名称在字典中不存在")
            return False

        except:
            # 处理特定异常的代码
            print("MovePose指令错误，请检查参数")
            self.err_log.append_err(112402, "MovePose指令错误，请检查参数")
            return False

    # 关节移动指令
    # 24
    def MoveL_Pose(self, point_name, speed=30, acc=30, blendT=-1, offset_x=0, offset_y=0,
                 offset_z=0, offset_xRot=0, offset_yRot=0, offset_zRot=0):
        try:
            # 将数据从tf中获取并转换坐标系之间的变换
            # 从点表中获取点位信息
            point_values = self.dic_point[point_name]
            tool = point_values[1]
            user = point_values[2]
            x = point_values[9] + offset_x
            y = point_values[10] + offset_y
            z = point_values[11] + offset_z
            x_rot = point_values[12] + offset_xRot
            y_rot = point_values[13] + offset_yRot
            z_rot = point_values[14] + offset_zRot

            # 点位point在用户坐标的变换
            x_m = x / 1000.0  # 单位毫米转米
            y_m = y / 1000.0  # 单位毫米转米
            z_m = z / 1000.0  # 单位毫米转米
            x_rot_rad = x_rot * 3.1415926 / 180  # 角度转弧度
            y_rot_rad = y_rot * 3.1415926 / 180  # 角度转弧度
            z_rot_rad = z_rot * 3.1415926 / 180  # 角度转弧度
            # 欧拉角转四元数
            q_user_tool = tfs.euler.euler2quat(x_rot_rad, y_rot_rad, z_rot_rad)
            # 将四元数转旋转矩阵
            R_user_tool = tfs.quaternions.quat2mat(q_user_tool)
            # 第一步求出user到指定tool的变换矩阵T_user_tool
            T_user_tool = tfs.affines.compose([x_m, y_m, z_m], R_user_tool, [1, 1, 1])

            # 从tf树中获取基坐标base和user的关系
            res_b_u, t_b_u, q_base_user = self.getTf_currentCoordinate(user, "fr_baseLink")
            if not res_b_u:
                self.err_log.append_err(112404,
                                        f"MovePose指令错误，未获取到{user}和 fr_baseLink 的tf关系，检查是否定义用户坐标系")
            # 将四元数q_base_user转换为旋转矩阵
            R_base_user = tfs.quaternions.quat2mat(q_base_user)
            # 第二步 将base到user的平移变换和旋转变换合成 齐次变换矩阵
            T_base_user = tfs.affines.compose(t_b_u, R_base_user, [1, 1, 1])

            # 从tf树读取点位指定tool到默认工具坐标系tool0的变换
            res_tool, t_tool, q_tool = self.getTf_currentCoordinate(tool, "tool0")
            if not res_tool:
                self.err_log.append_err(112404,
                                        f"MovePose指令错误，未获取到{tool}和 tool0 的tf关系，检查是否定义工具坐标系")
            # 将q_tool四元数转换为旋转矩阵
            R_tool0_tool = tfs.quaternions.quat2mat(q_tool)
            # 第三步将tool0到指定tool的齐次变换矩阵合成，然后求出逆矩阵 T_tool_tool0
            T_tool0_tool = tfs.affines.compose(t_tool, R_tool0_tool, [1, 1, 1])
            T_tool_tool0 = np.linalg.inv(T_tool0_tool)  # 求出逆矩阵 T_tool_tool0
            # 第四步 计算 机器人指定工具坐标系运动到点位时 tool0 在 fr_baseLink的齐次矩阵
            T_base_tool0 = np.dot(T_base_user, np.dot(T_user_tool, T_tool_tool0))
            # 齐次矩阵，转换成 平移量及欧拉角
            xRot_base_tool0, yRot_base_tool0, zRot_base_tool0 = tfs.euler.mat2euler(T_base_tool0[0:3, 0:3])
            T_base_tool0 = T_base_tool0[0:3, 3:4]  # 获取平移矩阵
            x_trans = T_base_tool0[0]  # x平移量
            y_trans = T_base_tool0[1]  # y平移量
            z_trans = T_base_tool0[2]  # z平移量
            # 将平移转换为mm，角度转换为弧度
            x_trans_mm = x_trans[0] * 1000.0
            y_trans_mm = y_trans[0] * 1000.0
            z_trans_mm = z_trans[0] * 1000.0
            xRot_angle = xRot_base_tool0 * 180 / 3.1415926
            yRot_angle = yRot_base_tool0 * 180 / 3.1415926
            zRot_angle = zRot_base_tool0 * 180 / 3.1415926

            # 将tool0在base的坐标变换发送给命令
            # 定义命令
            cmd_str = ("MoveL_Pose_ywf(" + str(x_trans_mm) + "," + str(y_trans_mm) + "," + str(z_trans_mm) + "," +
                       str(xRot_angle) + "," + str(yRot_angle) + "," + str(zRot_angle) + "," +
                       str(speed) + "," + str(acc) + "," + str(blendT) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(112403, "MoveL_Pose，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(112401,
                                        "MoveL_Pose，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False
            # 等待运动结束
            count_motion_done = 0
            while True:
                if self.fr_state.robot_motion_done == 1:
                    break
                else:
                    count_motion_done += 1
                    if count_motion_done > 1000:
                        self.err_log.append_err(112401,
                                                "MovePose，等待motion done超时报警，超过100s未等到robot_motion_done信号")
                        return False
                    time.sleep(0.1)
            # 运动结束 返回真
            return True

        except KeyError:
            print("MoveL_Pose指令错误，点位名称在字典中不存在")
            self.err_log.append_err(112403, "MoveL_Pose指令错误，点位名称在字典中不存在")
            return False

        except:
            # 处理特定异常的代码
            print("MoveL_Pose指令错误，请检查参数")
            self.err_log.append_err(112402, "MoveL_Pose指令错误，请检查参数")
            return False

    # 停止运动
    # 30
    def StopMotion(self):
        try:
            # 定义命令
            cmd_str = ("StopMotion("  + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(113001, "StopMotion指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future,timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res !='1' and result.cmd_res !='0':
                self.err_log.append_err(113002, "StopMotion指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False
            return True

        except:
            # 处理特定异常的代码
            print("StopMotion指令错误，请检查参数")
            self.err_log.append_err(113003, "StopMotion指令错误，请检查参数")
            return False

    # 机械臂使能开关
    # uint8_t state - 1-机械臂使能,0-机械臂去使能
    # 32
    def RobotEnable(self, state: int):
        try:
            # 定义命令
            cmd_str = ("RobotEnable(" + str(state) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(113201, "RobotEnable指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(113202,
                                        "RobotEnable指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            self.fr_enable = state
            return True

        except:
            # 处理特定异常的代码
            print("RobotEnable指令错误，请检查参数")
            self.err_log.append_err(113203, "RobotEnable指令错误，请检查参数")
            return False

    # 函数功能描述:模式切换
    # uint8_t state - 1-手动模式,0-自动模式
    # 33
    def Mode(self, state: int):
        try:
            # 定义命令
            cmd_str = ("Mode(" + str(state) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(113301, "Mode指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(113302,
                                        "Mode指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False
            self.fr_mode = state
            return True

        except:
            # 处理特定异常的代码
            print("Mode指令错误，请检查参数")
            self.err_log.append_err(113303, "Mode指令错误，请检查参数")
            return False

    # 函数功能描述:设置当前模式下机械臂速度
    # float vel - 速度百分比,范围为1-100
    # 34
    def SetSpeed(self, vel: float):
        try:
            # 定义命令
            cmd_str = ("SetSpeed(" + str(vel) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(113401, "SetSpeed指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(113402,
                                        "SetSpeed指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            # 全局速度赋值
            self.speed = vel
            return True

        except:
            # 处理特定异常的代码
            print("SetSpeed指令错误，请检查参数")
            self.err_log.append_err(113403, "SetSpeed指令错误，请检查参数")
            return False

    # 函数功能描述:设置末端负载重量
    # float weight - 负载重量,单位kg
    # 35
    def SetLoadWeight(self, weight: float):
        try:
            # 定义命令
            cmd_str = ("SetLoadWeight(" + str(weight) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(113501, "SetLoadWeight指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(113502,
                                        "SetLoadWeight指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("SetLoadWeight指令错误，请检查参数")
            self.err_log.append_err(113503, "SetLoadWeight指令错误，请检查参数")
            return False

    # 函数功能描述:设置末端负载质心坐标
    # float x,y,z - 质心坐标,单位为mm
    # 36
    def SetLoadCoord(self, x: float, y: float, z: float):
        try:
            # 定义命令
            cmd_str = ("SetLoadCoord(" + str(x) + "," + str(y) + "," + str(z) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(113601, "SetLoadCoord指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(113602,
                                        "SetLoadCoord指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("SetLoadCoord指令错误，请检查参数")
            self.err_log.append_err(113603, "SetLoadCoord指令错误，请检查参数")
            return False

    # 函数功能描述:设置机器人安装方式
    # uint8_t install - 安装方式,0-正装,1-侧装,2-倒装
    # 37
    def SetRobotInstallPos(self, install: int):
        try:
            # 定义命令
            cmd_str = ("SetRobotInstallPos(" + str(install) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(113701, "SetRobotInstallPos指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(113702,
                                        "SetRobotInstallPos指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("SetRobotInstallPos指令错误，请检查参数")
            self.err_log.append_err(113703, "SetRobotInstallPos指令错误，请检查参数")
            return False

    # 函数功能描述:设置机器人安装角度,自由安装
    # double y_angle - 倾斜角
    # double z_angle - 旋转角
    # 38
    def SetRobotInstallAngle(self, y_angle:float, z_angle:float):
        try:
            # 定义命令
            cmd_str = ("SetRobotInstallAngle(" + str(y_angle) + "," + str(z_angle) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(113801, "SetRobotInstallAngle指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(113802,
                                        "SetRobotInstallAngle指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("SetRobotInstallAngle指令错误，请检查参数")
            self.err_log.append_err(113803, "SetRobotInstallAngle指令错误，请检查参数")
            return False

    # 函数功能描述:设置机器人碰撞等级
    # float level1-level6 - 1-6轴的碰撞等级,范围是1-10
    # 39
    def SetAnticollision(self, level1:float , level2, level3:float , level4: float, level5: float, level6: float):
        try:
            # 定义命令
            cmd_str = ("SetAnticollision(" + str(level1) + "," + str(level2)+ "," +
                       str(level3)+ "," + str(level4)+ "," + str(level5)+ "," + str(level6) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(113901, "SetAnticollision指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(113902,
                                        "SetAnticollision指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("SetAnticollision指令错误，请检查参数")
            self.err_log.append_err(113903, "SetAnticollision指令错误，请检查参数")
            return False

    # 函数功能描述:设置碰撞后策略
    # int strategy - 0-报错停止,1-继续运行
    # 40
    def SetCollisionStrategy(self, strategy: int):
        try:
            # 定义命令
            cmd_str = ("SetCollisionStrategy(" + str(strategy) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(114001, "SetCollisionStrategy指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(114002,
                                        "SetCollisionStrategy指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("SetCollisionStrategy指令错误，请检查参数")
            self.err_log.append_err(114003, "SetCollisionStrategy指令错误，请检查参数")
            return False

    # 函数功能描述:设置正限位,注意设置值必须在硬限位范围内
    # float limit1-limit6 - 6个关节限位值
    # 41
    def SetLimitPositive(self, limit1: float, limit2: float, limit3: float,
                         limit4: float, limit5: float, limit6: float):
        try:
            # 定义命令
            cmd_str = ("SetLimitPositive(" + str(limit1) + "," + str(limit2)+ "," +
                       str(limit3)+ "," + str(limit4)+ "," + str(limit5)+ "," + str(limit6) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(114101, "SetLimitPositive指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(114102,
                                        "SetLimitPositive指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("SetLimitPositive指令错误，请检查参数")
            self.err_log.append_err(114103, "SetLimitPositive指令错误，请检查参数")
            return False

    # 函数功能描述:设置负限位,注意设置值必须在硬限位范围内
    # float limit1-limit6 - 6个关节限位值
    # 42
    def SetLimitNegative(self, limit1: float, limit2: float, limit3: float,
                         limit4: float, limit5: float, limit6: float):
        try:
            # 定义命令
            cmd_str = ("SetLimitNegative(" + str(limit1) + "," + str(limit2)+ "," +
                       str(limit3)+ "," + str(limit4)+ "," + str(limit5)+ "," + str(limit6) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=2.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(114201, "SetLimitNegative指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(114202,
                                        "SetLimitNegative指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("SetLimitNegative指令错误，请检查参数")
            self.err_log.append_err(114203, "SetLimitNegative指令错误，请检查参数")
            return False


    # 函数功能描述:错误状态清除
    # 43
    def ResetAllError(self):
        try:
            # 定义命令
            cmd_str = ("ResetAllError(" + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(114301, "ResetAllError指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(114302,
                                        "ResetAllError指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("ResetAllError指令错误，请检查参数")
            self.err_log.append_err(114303, "ResetAllError指令错误，请检查参数")
            return False

    #外设控制
    #函数功能描述:激活夹爪
    #int index - 夹爪编号
    #uint8_t act - 0-复位, 1-激活
    #44
    def ActGripper(self, index: int, act: int):
        try:
            # 定义命令
            cmd_str = ("ActGripper(" + str(index) + "," + str(act) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(114401, "ActGripper指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(114402,
                                        "ActGripper指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("ActGripper指令错误，请检查参数")
            self.err_log.append_err(114403, "ActGripper指令错误，请检查参数")
            return False

    #函数功能描述: 控制夹爪
    #int index - 夹爪编号
    #int pos - 位置百分比, 范围0 - 100
    #45
    def MoveGripper(self, index: int, pos: int):
        try:
            # 定义命令
            cmd_str = ("MoveGripper(" + str(index) + "," + str(pos) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(114501, "MoveGripper指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(114502,
                                        "MoveGripper指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("MoveGripper指令错误，请检查参数")
            self.err_log.append_err(114503, "MoveGripper指令错误，请检查参数")
            return False

    # IO控制
    # 函数功能描述: 设置控制箱数字量输出
    # int id_io - io编号, 范围0 - 15
    # uint_t status - 0 - 关, 1 - 开
    #46
    def SetDO(self, id_io: int, status: int):
        try:
            # 定义命令
            cmd_str = ("SetDO(" + str(id_io) + "," + str(status) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=2.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(114601, "SetDO指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(114602,
                                        "SetDO指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("SetDO指令错误，请检查参数")
            self.err_log.append_err(114603, "SetDO指令错误，请检查参数")
            return False

    # 函数功能描述:设置工具数字量输出
    # int id - io编号,范围0-1
    # uint_t status - 0-关, 1-开
    # 47
    def SetToolDO(self, id_io: int, status: int):
        try:
            # 定义命令
            cmd_str = ("SetToolDO(" + str(id_io) + "," + str(status) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(114701, "SetToolDO指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(114702,
                                        "SetToolDO指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("SetToolDO指令错误，请检查参数")
            self.err_log.append_err(114703, "SetToolDO指令错误，请检查参数")
            return False

    # 函数功能描述:设置控制箱模拟量输出
    # int id - io编号,范围0-1
    # float value - 电流或者电压值百分比,范围0-100
    # 48
    def SetAO(self, id_AO: int, value: float):
        try:
            # 定义命令
            cmd_str = ("SetAO(" + str(id_AO) + "," + str(value) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(114801, "SetAO指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(114802,
                                        "SetAO指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("SetAO指令错误，请检查参数")
            self.err_log.append_err(114803, "SetAO指令错误，请检查参数")
            return False

    # 函数功能描述:设置工具模拟量输出
    # int id - io编号,范围0
    # float value - 电流或者电压值百分比,范围0-100
    # 49
    def SetToolAO(self, id_AO: int, value: float):
        try:
            # 定义命令
            cmd_str = ("SetToolAO(" + str(id_AO) + "," + str(value) + ')')
            self.req.cmd_str = cmd_str
            # 等待服务器响应
            if not self.cli.wait_for_service(timeout_sec=5.0):
                print('service not available, waiting again...')
                # 服务器超时未响应，返回报错
                self.err_log.append_err(114901, "SetToolAO指令错误，fr_ros2 service超时，检查网络")
                return False
            # 发送请求
            future = self.cli.call_async(self.req)
            # 等待返回预期
            rclpy.spin_until_future_complete(self, future, timeout_sec=1000)
            # 获取结果
            result = future.result()
            if result.cmd_res != '1' and result.cmd_res !='0':
                self.err_log.append_err(114902,
                                        "SetToolAO指令错误，service返回结果错误，检查fr service服务端是否正常返回结果")
                return False

            return True

        except:
            # 处理特定异常的代码
            print("SetToolAO指令错误，请检查参数")
            self.err_log.append_err(114903, "SetToolAO指令错误，请检查参数")
            return False