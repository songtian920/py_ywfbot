import string
import subprocess
import time
import threading
import logging
from logging.handlers import TimedRotatingFileHandler
import datetime
from multiprocessing.util import get_logger
from PySide6.QtCore import  QObject,Signal
from user_func_implement.function.func_fr_ros2 import FuncFrRos2
from user_func_implement.function.fr_state_sub import FrStateSub
from user_func_implement.function.error_list import ErrorListUser
import launch
import launch_ros.actions
from user_func_implement.function.func_tf2 import FuncTf2
from user_func_implement.function.palletProcess import PalletProcess
import asyncio

#模块号03
#GUI显示开关状态，通过控制显示开关来决定哪些信号需要反馈，
# 用于减小界面显示的运行开销
class GuiDispState:
    # 构造函数
    def __init__(self):
        print("CommandScript初始化")
        #界面的类型 0：本地GUI，1：远程网络GUI
        self.GUIType = 0  #默认本地GUI
        #主界面状态显示开关AmrStatusDisp
        self.mian_statusDisp_switch = True
        #机器人状态显示RobotManual
        self.RobotManualDisp_switch = False
        #Io监控显示开关
        self.IoSetDisp_switch = False
        #

#部件库类
#包含了 状态控制 异常处理日志 sqlite数据库  机械手臂API 运动底盘API GPIO 视觉API tcp_socket通讯 tcp_modbus通讯
class Components:
    # 初始化函数
    def __init__(self,err_log:ErrorListUser,func_FrRos2:FuncFrRos2,FrState:FrStateSub,funcTf2:FuncTf2,pallet_Process:PalletProcess,gui_DispState:GuiDispState):
        print("初始化")
        #moveit2 api部件
        self.FuncMoveit2=None
        #fr_ros2模块
        self.FuncFrRos2:FuncFrRos2=func_FrRos2
        #fr robot 状态订阅
        self.FrStateSub:FrStateSub=FrState
        #tf2状态监控及发布
        self.func_tf2 = funcTf2
        #码垛工艺
        self.palletProcess = pallet_Process
        #信息显示开关
        self.guiDispState:GuiDispState = gui_DispState

        #手自动切换
        self.__manuel_auto_status = False
        #暂停
        self.__pause_start = False
        #初始化完成
        self.init_success = False
        #退出
        self.existApp = False

        # 错误处理
        self.errListUser = err_log
        # 将启动暂停方法传递给错误处理 用于报错停止
        self.errListUser.start_pause_switch = self.start_pause_switch

        # 反馈给界面的信号对象，从外部赋值
        self.Send_signal = None

        #ros2 launch process列表
        self.__process_list = []
        #用户命令
        self.commandScript = CommandScript()

    def add_Send_signal(self,send_signal_obj):
        self.Send_signal=send_signal_obj

        self.FuncFrRos2.send_signal=send_signal_obj
        self.FrStateSub.send_signal = send_signal_obj
        self.errListUser.Send_signal=send_signal_obj

    def get_status_pause_start(self):
            return self.__pause_start

    def get_status_manuel_auto(self):
            return self.__manuel_auto_status

    def config(self):
        print("config")

    #启动ros2 launch
    def launch_ros2_launch(self,package_name, launch_file_name, args=[]):
        #cmd = ['ros2', 'launch', package_name, launch_file_name]
        #cmd.extend(args)
        #subprocess.run(cmd)
        process = subprocess.Popen(['ros2', 'launch', package_name, launch_file_name])
        self.__process_list.append(process)

    # 启动ros2 node
    def ros2_run_node(self,package_name, node_name, args=[]):
        #cmd = ['ros2', 'launch', package_name, launch_file_name]
        #cmd.extend(args)
        #subprocess.run(cmd)
        process = subprocess.Popen(['ros2', 'run', package_name, node_name])
        self.__process_list.append(process)

    #关闭ros2 launch
    def close_ros2_launch(self):
        for proc in self.__process_list:
            proc.terminate()
            proc.wait()

    #手自动切换开关
    def manuel_auto_switch(self,mode:bool):
        # 启动状态切位暂停
        self.__pause_start = False
        #切换自动状态 mode为True
        if mode:
            if self.init_success & (not self.errListUser.errStatus): #初始化成功及无报警状态才能切自动
                result = self.FuncFrRos2.Mode(0)  # 自动
                if result:
                    # 解除阻塞
                    self.commandScript.block_cmd(False)
                    # 命令赋初始值
                    self.commandScript.set_command("start")
                    self.__manuel_auto_status = True
                    # 切换后为了安全，将速度设置为10%
                    self.FuncFrRos2.SetSpeed(10.0)

            else:
                print("初始化未成功或有异常，不能切换自动，请完成初始化并清除异常")
                #报警处理
                self.errListUser.append_err(1000,"初始化未成功或有异常，不能切换自动，请完成初始化并清除异常")

        else: #切换手动
            # 阻塞脚本命令 其他用户无法改变命令 直至解除阻塞
            result =  self.FuncFrRos2.Mode(1)  # 手动
            if result:
                self.commandScript.block_cmd(True)
                # 命令值清空
                self.commandScript.set_command("")
                #todo 向各个模块发送终止命令
                #状态 切换成手动
                self.__manuel_auto_status = False
                # 切换后为了安全，将速度设置为10%
                self.FuncFrRos2.SetSpeed(10.0)


        # 给界面发送 手动自动状态信号
        self.Send_signal.manuel_auto_signal.emit(self.__manuel_auto_status)
        #反馈手自动状态
        return self.__manuel_auto_status

    #启动暂停切换开关
    def start_pause_switch(self,mode:bool):
        #启动 mode为True
        if mode:
            if self.__manuel_auto_status and (not self.errListUser.errStatus) and self.init_success: #状态为自动
                self.FuncFrRos2.Start()  # 给机器人发送开始
                #启动状态
                self.__pause_start = True

            else:
                #警告提示 切换到自动状态
                print("当前的状态为手动状态，请先切换到自动")
                self.errListUser.append_err(1001,"当前的状态为手动状态，请先切换到自动")

        else:  #状态切换暂停
            self.__pause_start = False
            #asyncio.run(self.stopMotion_task())
            #self.FuncFrRos2.StopMotion()  # 给机器人发送停止
            self.FuncFrRos2.Pause()
            #self.stopMotion_task()
        # 给界面发信号 告知是否启动
        self.Send_signal.start_status_signal.emit(self.__pause_start)
        return self.__pause_start

    #向组件发送运动终止
    #async def stopMotion_task(self):
    #    self.FuncFrRos2.StopMotion()  # 运动终止
    def stopMotion_task(self):
        self.FuncFrRos2.StopMotion()  # 运动终止

#command控制命令
#主函数main中的状态机使用此对象，完成状态切换的操作
#封装是为了能够让其他模块能够控制状态机，例如界面按钮 手自动事件
# 通过状态机完成命令清除并阻塞其他模块写入命令
class CommandScript:
    #构造函数
    def __init__(self):
        print("CommandScript初始化")
        self.__command=""
        self._lock = threading.Lock()
        self.__block_cmd=False  #阻塞命令

    #设置当前命令
    def set_command(self,command:string):
        with self._lock:
            if not self.__block_cmd:
                self.__command=command

    #获取当前命令
    def get_command(self):
            return self.__command

    #阻塞命令
    def block_cmd(self,block:bool):
        self.__block_cmd = block

#主界面需要的部件尽量都放在此处 互相依赖方便 避免循环调用
# 创建一个 QObject 子类来管理信号，用于给界面发信号
class SendSignal(QObject):
    # 创建信号
    #主程序初始化完成
    init_success_signal = Signal(int)
    #手自动切换
    manuel_auto_signal = Signal(bool)
    #启动停止状态
    start_status_signal = Signal(bool)
    #错误信息添加
    signal_errorMesAppend = Signal(str)
    #错误清除
    signal_errorMesClear = Signal()

    #RobotManual机器人 状态数据更新
    #用于向界面回传手臂整型状态数据
    signal_robot_int_state_update = Signal(list)
    # 用于向界面回传手臂双精度型状态数据
    signal_robot_float_state_update = Signal(list)
    #更新工具坐标系列表
    signal_robot_tool_list = Signal(list)
    #更新用户坐标系列表
    signal_robot_user_list = Signal(list)
    #更新点位名称到机器人手动控制RobotManual
    signal_robot_pointsName_list = Signal(list)
    #点表的注释加载到机器人手动
    signal_robot_pointsComment_list = Signal(list)

    #AmrStatusDisp
    signal_robotSpeed_callback = Signal(float)

    #机器人点表数据回传
    #更新一行到tableView
    signal_appendRow_tableModel_frPoint = Signal(list)
    signal_insertNewRowToTM_DB_frPoint = Signal(str)
    #点位示教
    signal_teach_frPointsList = Signal(list)

    # 机器人工具表数据回传
    # 更新一行到tableView
    signal_appendRow_tableModel_frTool = Signal(list)   #工具添加一行数据
    signal_insertNewRowToTM_DB_frTool = Signal(str)     #工具插入一行空白行
    signal_teachPoint_calibTool_callBack = Signal(list,int)     #新工具标定 示教点位回传,给点位索引0-2是平移数据，3-5是旋转数据
    signal_calculateTool_calibTool_callback = Signal(list,str)        #新工具计算结果回传, 给定类型translate,rotate
    signal_updateToolData_calibTool_callback = Signal(list,str)   #保存更新新工具数据后回传数据到界面，给定类型translate,rotate

    # 机器人工具表数据回传
    # 更新一行到tableView
    signal_appendRow_tableModel_frUser = Signal(list)
    signal_insertNewRowToTM_DB_frUser = Signal(str)
    signal_teachPoint_calibUser_callBack = Signal(list, int)  # 新用户标定 示教点位回传,给点位索引0-2
    signal_calculateUser_calibUser_callback = Signal(list)  # 新工具计算结果回传, 给定类型translate,rotate
    signal_updateUserData_calibUser_callback = Signal(list)  # 保存更新新工具数据后回传数据到界面，给定类型translate,rotate

    #机器人手动控制
    #RobotDIO
    signal_update_IO_state_robotDIO = Signal(list,list,list,list)

    #码垛工艺包
    #插入新工艺参数
    signal_insert_newPalletProcess_callback = Signal(str)
    #码垛工艺参数增加一行
    signal_appendParam_palletProcess_callback = Signal(list)
    #向界面传递 tool user 名称值
    signal_sendToolUserPointNames_callback = Signal(list,list,list)




