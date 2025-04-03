import rclpy
from rclpy import time
from rclpy.node import Node

import time
from datetime import datetime

import sqlite3
from sqlite3 import OperationalError
import os
import math
import numpy as np
import tf2_ros
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import transforms3d as tfs
from .fr_state_sub import FrStateSub
from .error_list import ErrorListUser

class PalletProcess:
    def __init__(self,error:ErrorListUser):
        self.err_log = error
        #数据库相对路径
        self.path_dataBase = '/user_func_implement/user_func_implement/sqlite/palletizingProcess.db'
        #码垛工艺包参数 指定了工具 用户 及偏移值
        self.dic_paramPalletProcess = {}
        #加载的工艺包 层堆叠字典
        self.dic_layerStack = {}

        #给界面发送信号
        self.send_signal = None

    #从数据库读取码垛工艺参数
    def loadParam_From_palletProcessDB(self):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)

        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + self.path_dataBase)
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 读取表格信息
            cursor.execute('SELECT * FROM param_palletProcess')
            # 获取所有值
            values = cursor.fetchall()
            # 点表字典清空
            self.dic_paramPalletProcess.clear()
            # 获取的每行数据添加到点表，更新到tableView
            for val in values:
                dic_name = ''
                data_list = []
                # 点表数据逐行添加到本地 字典
                dic_name = val[0]  # 数据库中的name
                data_list.append(val[1])  # 数据库中的comment注释
                data_list.append(val[2])  # 数据库中的tool
                data_list.append(val[3])  # 数据库中的user
                data_list.append(val[4])  # 数据库中的tool_x_offset
                data_list.append(val[5])  # 数据库中的tool_y_offset
                data_list.append(val[6])  # 数据库中的tool_z_offset
                data_list.append(val[7])  # 数据库中的tool_xRot_offset
                data_list.append(val[8])  # 数据库中的tool_yRot_offset
                data_list.append(val[9])  # 数据库中的tool_zRot_offset
                data_list.append(val[10])  # 数据库中的user_x_offset
                data_list.append(val[11])  # 数据库中的user_y_offset
                data_list.append(val[12])  # 数据库中的user_z_offset
                data_list.append(val[13])  # 数据库中的user_xRot_offset
                data_list.append(val[14])  # 数据库中的user_yRot_offset
                data_list.append(val[15])  # 数据库中的user_zRot_offset
                data_list.append(val[16])  # 数据库中的passPointName
                self.dic_paramPalletProcess[dic_name] = data_list  # 点表字典添加值

                # 点表数据逐行添加到界面
                # 凑成列表
                list_var = [str(val[0]), str(val[1]), str(val[2]), str(val[3]), str(val[4]),
                            str(val[5]), str(val[6]), str(val[7]), str(val[8]), str(val[9]),
                            str(val[10]), str(val[11]), str(val[12]), str(val[13]), str(val[14]),
                            str(val[15]),str(val[16])]
                # 发送信号给GUI添加一行
                self.send_signal.signal_appendParam_palletProcess_callback.emit(list_var)
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

    # 插入新的一行到数据库，并更新到内存
    # 10
    def insert_newPalletProcess(self, name_str):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + self.path_dataBase)
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 读取表格信息
            cursor.execute('INSERT INTO param_palletProcess (name) VALUES (?)', (name_str,))
            conn.commit()
            # 更新到点表字典 dic_point
            self.dic_paramPalletProcess[name_str] = ["注释", "", "", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,""]
            # 发送信号给GUI添加一行
            self.send_signal.signal_insert_newPalletProcess_callback.emit(name_str)
            return True, ""

        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111001, str(o))
            return False, str(o)
        except Exception as e:
            print(e)
            self.err_log.append_err(111002, str(e))
            return False, str(e)
        except:
            print("insert_newRowInParamPalletProcess：加载机器人点位错误")
            self.err_log.append_err(111003, "insert_newRowInParamPalletProcess：加载机器人点位错误")
            return False, "insert_newRowInParamPalletProcess：加载机器人点位错误"
        finally:
            cursor.close()
            conn.close()

    # 删除数据库点表一行
    def deleteRow_ParamPalletProcess(self, name_str):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + self.path_dataBase)
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 读取表格信息
            cursor.execute("delete from param_palletProcess where name= ?;", (name_str,))
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

    def updateDbRow_palletProcess(self, name_selected, list_var):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + self.path_dataBase)
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 执行更新语句，更新user表的数据:
            update_stmt = 'UPDATE param_palletProcess SET comment=?,tool=?,user=?,tool_x_offset=?,tool_y_offset=?,tool_z_offset=?,tool_xRot_offset=?,tool_yRot_offset=?,tool_zRot_offset=?,user_x_offset=?,user_y_offset=?,user_z_offset=?,user_xRot_offset=?,user_yRot_offset=?,user_zRot_offset=?,passPointName=? WHERE name = ?'
            comment = str(list_var[0])
            tool = list_var[1]
            user = list_var[2]
            tool_x_offset = float(list_var[3])
            tool_y_offset = float(list_var[4])
            tool_z_offset = float(list_var[5])
            tool_xRot_offset = float(list_var[6])
            tool_yRot_offset = float(list_var[7])
            tool_zRot_offset = float(list_var[8])
            user_x_offset = float(list_var[9])
            user_y_offset = float(list_var[10])
            user_z_offset = float(list_var[11])
            user_xRot_offset = float(list_var[12])
            user_yRot_offset = float(list_var[13])
            user_zRot_offset = float(list_var[14])
            passPointName = list_var[15]
            #更新值
            update_values = (comment, tool, user, tool_x_offset, tool_y_offset, tool_z_offset,
                             tool_xRot_offset,tool_yRot_offset,tool_zRot_offset,
                             user_x_offset,user_y_offset,user_z_offset,
                             user_xRot_offset,user_yRot_offset,user_zRot_offset,passPointName,name_selected)
            cursor.execute(update_stmt, update_values)
            conn.commit()
            # 更新到本地用户坐标系表
            self.dic_paramPalletProcess[name_selected] = [comment, tool, user, tool_x_offset, tool_y_offset, tool_z_offset,
                             tool_xRot_offset,tool_yRot_offset,tool_zRot_offset,
                             user_x_offset,user_y_offset,user_z_offset,
                             user_xRot_offset,user_yRot_offset,user_zRot_offset,passPointName]
            return True

        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111601, str(o))
            return False
        except Exception as e:
            print(str(e))
            self.err_log.append_err(111602, "码垛工艺包错误:"+str(e))
            return False
        except:
            self.err_log.append_err(111603, "码垛工艺包错误：更新参数出错")
            return False
        finally:
            cursor.close()
            conn.close()

    #新建层堆叠表
    def createPalletStackTB(self,name_pallet):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        conn = sqlite3.connect(split_path2 + self.path_dataBase)
        cur = conn.cursor()
        try:
            # 检查表是否存在
            cur.execute(f"SELECT name FROM sqlite_master WHERE type='table' AND name=?", (name_pallet,))
            if cur.fetchone() is not None:
                raise ValueError(f"表 '{name_pallet}' 已存在")

            sql = f"""CREATE TABLE {name_pallet} (                    
                            layerIndex INT primary key,
                            layerModelName TEXT not null DEFAULT "",
                            layerHeight FLOAT not null DEFAULT 0,
                            x_offset FLOAT not null DEFAULT 0,
                            y_offset FLOAT not null DEFAULT 0
                        );"""
            cur.execute(sql)
            print("create table success")
            return True

        except Exception as e:
            print(e)
            self.err_log.append_err(111603, "新建的层堆叠名称已经存在，请更换名称")
            return False
        finally:
            cur.close()
            conn.close()

    #删除层堆叠表
    def delete_PalletStackTB(self,Name_pallet):

        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        conn = sqlite3.connect(split_path2 + self.path_dataBase)
        cursor = conn.cursor()
        try:
            # 码垛工艺包 堆叠表
            name_PalletStack = Name_pallet + "_StackTB"
            # 执行删除表的SQL语句
            cursor.execute(f'DROP TABLE IF EXISTS {name_PalletStack}')
            # 提交事务
            conn.commit()
        except Exception as e:
            print(e)
            self.err_log.append_err(111603, "删除层堆叠错误")
            return False
        finally:
            cursor.close()
            conn.close()

    #添加层 单层添加
    def insertNewLayerInStack(self,name_pallet):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + self.path_dataBase)
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 使用COUNT(*)获取行数
            cursor.execute(f"SELECT COUNT(*) FROM {name_pallet}")
            row_count = cursor.fetchone()[0]
            # 读取表格信息
            cursor.execute(f"""INSERT INTO {name_pallet} (layerIndex) VALUES (?)""", (row_count,))
            conn.commit()
            # 堆叠层字典数据添加
            self.dic_layerStack[row_count] = ["", 0.0, 0.0, 0.0]
            # todo发送信号给GUI添加一行
            return True, ""

        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111001, str(o))
            return False, str(o)
        except Exception as e:
            print(e)
            self.err_log.append_err(111002, str(e))
            return False, str(e)

        finally:
            cursor.close()
            conn.close()

    #删除层  删除操作会将层索引之后的层都删除
    def delete_layerToStack(self,name_pallet,layerIndex):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + self.path_dataBase)
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 使用COUNT(*)获取行数
            cursor.execute(f"SELECT COUNT(*) FROM {name_pallet}")
            row_count = cursor.fetchone()[0]
            for i in range(layerIndex,row_count):
                # 读取表格信息
                cursor.execute(f"""delete from {name_pallet} where layerIndex= ?;""", (layerIndex,))
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

    #更新堆叠中的层数据
    def update_layerDataInStack(self,name_pallet,layerIndex,list_var):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + self.path_dataBase)
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 执行更新语句，更新user表的数据:
            update_stmt = f'UPDATE {name_pallet} SET comment=?,tool=?,user=?,tool_x_offset=?,tool_y_offset=?,tool_z_offset=?,tool_xRot_offset=?,tool_yRot_offset=?,tool_zRot_offset=?,user_x_offset=?,user_y_offset=?,user_z_offset=?,user_xRot_offset=?,user_yRot_offset=?,user_zRot_offset=?,passPointName=? WHERE layerIndex = ?'
            #层模版
            layerModelName = list_var[0]
            #层高
            layerHeight = float(list_var[1])
            #层x偏移
            x_offset = float(list_var[2])
            #层y偏移
            y_offset = float(list_var[3])
            # 更新值
            update_values = (layerModelName,layerHeight,x_offset,y_offset, layerIndex)
            cursor.execute(update_stmt, update_values)
            conn.commit()
            # 更新到本地用户坐标系表
            self.dic_layerStack[layerIndex] = [layerModelName,layerHeight,x_offset,y_offset]
            return True

        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111601, str(o))
            return False
        except Exception as e:
            print(str(e))
            self.err_log.append_err(111602, "码垛工艺包错误:" + str(e))
            return False
        except:
            self.err_log.append_err(111603, "码垛工艺包错误：更新参数出错")
            return False
        finally:
            cursor.close()
            conn.close()

    #加载指定托盘的层堆叠
    def load_layerStack(self,name_pallet):
        # 路径连接
        current_path = os.getcwd()
        split_path1, folder1 = os.path.split(current_path)
        split_path2, folder2 = os.path.split(split_path1)
        # 连接数据库 并读取数据
        conn = sqlite3.connect(split_path2 + self.path_dataBase)
        # 创建一个Cursor:
        cursor = conn.cursor()
        try:
            # 读取表格信息
            cursor.execute(f'SELECT * FROM {name_pallet}')
            # 获取所有值
            values = cursor.fetchall()
            # 点表字典清空
            self.dic_layerStack.clear()
            # 获取的每行数据添加到点表，更新到tableView
            for val in values:
                dic_layerIndex = 0
                data_list = []
                # 点表数据逐行添加到本地 字典
                dic_layerIndex = val[0]  #layerIndex
                data_list.append(val[1])  #层模版
                data_list.append(val[2])  #层高
                data_list.append(val[3])  #层x偏移
                data_list.append(val[4])  #层y偏移
                self.dic_layerStack[dic_layerIndex] = data_list  # 点表字典添加值
                # 点表数据逐行添加到界面
                # 凑成列表
                list_var = [str(val[0]), str(val[1]), str(val[2]), str(val[3]), str(val[4])]
                # todo发送信号给GUI添加一行
                #self.send_signal.signal_appendParam_palletProcess_callback.emit(list_var)
            return True
        except OperationalError as o:
            print(str(o))
            self.err_log.append_err(111901, str(o))
            return False
        except Exception as e:
            print(str(e))
            self.err_log.append_err(111902, str(e))
            return False
        finally:
            cursor.close()
            conn.close()