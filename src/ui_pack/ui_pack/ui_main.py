import sys
import os
import string
import subprocess
import time
import rclpy
import threading
from GUI import YwfWidget
from components import  Components
from components import SendSignal
from components import GuiDispState
from rclpy.executors import MultiThreadedExecutor

from src.user_func_implement.user_func_implement.function.palletProcess import PalletProcess
from user_func_implement.function.Func_moveit2 import FuncMoveit2
from user_func_implement.function.func_fr_ros2 import FuncFrRos2
from user_func_implement.function.fr_state_sub import FrStateSub
from user_func_implement.function.error_list import ErrorListUser
from user_func_implement.function.func_tf2 import FuncTf2

#模块01

def spin_node(node):
        rclpy.spin(node)

def execute_spin(executor):
    executor.spin()

def open_gui_ywf(ywf_widget:YwfWidget):
    ywf_widget.show_widget()

#主函数
#主函数是整个机器人启动的接口，分为三个部分 1.初始化部分 2.逻辑脚本执行状态机 3.APP退出
#3个部分采用平铺的方式展开，可使用户了解整个机器人运行脉络，尽可能避免过度的封装及跳跃
#总之作者的目的是使用户能够简单的理解整个机器人设计思路，而非机械式填充代码。需要单独执行的模块基本都在封装components类
#让用户减少因阅读代码反复切换文件，所以作者尽可能把 部件对象 直接放入components类，大型类还是尽量单独文件夹存放
def main():
    #ros2库初始化
    rclpy.init()

    # 部件实例化
    # Todo待完善
    #错误处理
    err_log = ErrorListUser()

    #GUI显示开关状态
    gui_dispState = GuiDispState()
    #tf2监控及发布器
    func_tf2 = FuncTf2(err_log)
    # FR state订阅
    fr_state_sub = FrStateSub(err_log,func_tf2)
    # 构造fr_ros2模块
    func_fr_ros2 = FuncFrRos2(err_log, fr_state_sub,func_tf2)  # FR API控制
    palletProcess = PalletProcess(err_log)
    # 部件对象 包含了各个子模块，及一些共用变量，方便各个模块间的调用
    _components = Components(err_log,func_fr_ros2,fr_state_sub,func_tf2,palletProcess,gui_dispState)

    #启动moveit2 demo launch
    #_components.launch_ros2_launch('moveit2_tutorials','move_group.launch.py')

    #启动法奥的命令服务器 ros2_cmd_server
    #_components.ros2_run_node('fairino_hardware', 'ros2_cmd_server')
    #time.sleep(10)

    #启动moveit2 c++ API launch
    #_components.launch_ros2_launch('moveit2_ywf_interface','start_ywfMoveit_intf.launch.py')

    time.sleep(1)

    #构造moveit2子模块
    #func_moveit2 = FuncMoveit2()

    #部件对象添加子模块
    #_components.FuncMoveit2 = func_moveit2
    #_components.FuncFrRos2 = func_fr_ros2
    #_components.FrStateSub = fr_state_sub
    #开启moveit2 python节点node  spin线程，用于ros2的信息处理
    #thread_func_moveit2 = threading.Thread(target=spin_node, args=(func_moveit2,))
    #thread_func_moveit2.start()
    #time.sleep(1)

    # 开启GUI窗口线程
    ywf_widget = YwfWidget(_components)

    #python开线程自动 带入方法的对象，和c++不同，无需将对象作为参数带入,套壳带入函数更标准
    thread_gui = threading.Thread(target=open_gui_ywf,args=(ywf_widget,))
    thread_gui.start()
    count_win_time = 0
    while not ywf_widget.win_open_success:
        time.sleep(0.1)
        count_win_time+=1
        if count_win_time >= 1000:
            return 0
    # 将GUI中的 反馈给界面的信号对象 给每个部件，用于给GUI发送信号
    _components.Send_signal = ywf_widget.signal_obj
    fr_state_sub.send_signal = ywf_widget.signal_obj
    func_fr_ros2.send_signal = ywf_widget.signal_obj
    err_log.Send_signal = ywf_widget.signal_obj
    palletProcess.send_signal = ywf_widget.signal_obj
    #_components.add_Send_signal(ywf_widget.signal_obj)

    # 开始部件config配置操作


    # todo moveit组件配置 在节点启动前配置
    # func_moveit2.config()

    #多线程执行器
    executor = MultiThreadedExecutor()
    executor.add_node(func_fr_ros2)
    executor.add_node(fr_state_sub)
    executor.add_node(func_tf2)
    time.sleep(1)
    # fr机器人配置
    result_frConfig = func_fr_ros2.config()

    # 开启fr ros2 python节点node  spin线程，用于ros2的信息处理
    thread_func_fr_ros2 = threading.Thread(target=execute_spin, args=(executor,))
    thread_func_fr_ros2.start()
    time.sleep(1)

    #组件初始化成功状态 赋值true
    _components.init_success= result_frConfig
    # 初始化完成 发送成功信号给界面
    if result_frConfig:
        ywf_widget.init_success_fail(1)
    else:
        ywf_widget.init_success_fail(0)

    # 等待fr机器人正确反馈程序状态
    count_cycle =0  #超时
    while fr_state_sub.fr_prg_state != 1:
        time.sleep(0.1)
        count_cycle += 1
        if count_cycle >= 100:
            _components.errListUser.append_err(2, "fr机器人连接失败，请检查控制器是否连接fr机器人，及电源是否打开")
            break

    time.sleep(1)
    # 机器人初始设置
    func_fr_ros2.Mode(0)  # 手动模式
    func_fr_ros2.Mode(1)  #手动模式
    func_fr_ros2.RobotEnable(1)  #手臂上电
    func_fr_ros2.SetSpeed(10)  #全局速度10%
    func_fr_ros2.SetAnticollision(9,9,9,9,7,7)

    ####用户逻辑代码部分
    #逻辑运动 用if else if实现，目的是简单，易懂
    # 状态机的状态用 部件库 components中的command引用赋值
    command_script = _components.commandScript
    command_script.set_command("start")  # 给初始值
    r_pallet = 0  # 栈板行
    c_pallet = 0  # 栈板列
    z_pallet = 0  # 栈板层
    obj_width = 200  # 列偏移
    obj_length = 200  # 行偏移
    obj_high = 100  # 层偏移
    while not _components.existApp:
        # 暂停
        while not _components.get_status_pause_start():
            time.sleep(0.5)  # 休眠时间
        #刷新最新命令
        command = command_script.get_command()

        #状态机周期
        time.sleep(0.1)

        #状态机 python用 if elif做状态机
        #首个状态从start开始
        if command=='start':  #系统状态机禁止修改
            print("start")
            #func_fr_ros2.MovePose("p2", speed=100, blendT=300, block=False)
            command_script.set_command("move_getObj_ready")

        #取料准备位
        elif command=='move_getObj_ready':
            func_fr_ros2.MoveJoint("p0",speed=100,blendT=300,block=False)
            command_script.set_command("move_getObj")

        # 取料位
        elif command == 'move_getObj':
            func_fr_ros2.MoveJoint("p1", speed=100,  block=True)
            command_script.set_command("move_getObj_ready")

        #回到 取料待机位
        elif command == 'move_getObj_ready_1':
            func_fr_ros2.MoveJoint("p0",speed=100,blendT=-1,block=True)
            command_script.set_command("move_pass")

        # 过渡位
        elif command == 'move_pass':
            func_fr_ros2.MoveJoint("p2",offset_z=z_pallet*obj_high,speed=100,blendT=-1, block=True)
            command_script.set_command("move_getObj_ready")

        # 栈板放料 过渡位
        elif command == 'move_putObj_pass':
            func_fr_ros2.MovePose("p3", speed=100,offset_x=obj_width*c_pallet,
                                  offset_y=obj_length * r_pallet,
                                  offset_z=obj_high*z_pallet+(obj_high*1.2),blendT=300, block=False)
            command_script.set_command("move_PutObj_ready")

        # 栈板放料 准备位
        elif command == 'move_PutObj_ready':
            func_fr_ros2.MovePose("p3", speed=100, offset_x=obj_width * c_pallet,
                                  offset_y= obj_length*r_pallet,
                                  offset_z=obj_high * z_pallet,blendT=500, block=False)
            command_script.set_command("move_putObj")

        # 栈板放料 放料位
        elif command == 'move_putObj':
            func_fr_ros2.MovePose("p3", speed=100, offset_x=obj_width * c_pallet,
                                  offset_y= obj_length*r_pallet,
                                  offset_z=obj_high * z_pallet, blendT=500, block=False)
            command_script.set_command("move_putObj_leave")

        # 栈板放料 离开位
        elif command == 'move_putObj_leave':
            func_fr_ros2.MovePose("p3", speed=100, offset_x=obj_width * c_pallet,
                                  offset_y=obj_length * r_pallet,
                                  offset_z=obj_high * z_pallet + (obj_high * 1.2), blendT=500, block=False)
            command_script.set_command("move_pass_2")

        # 回到过渡位
        elif command == 'move_pass_2':
            func_fr_ros2.MovePose("p2", speed=100,offset_z=obj_high * z_pallet, blendT=500, block=False)
            #结束当前
            c_pallet += 1   #列加1
            if c_pallet>=3:  #列满，则行加1 列清零
                c_pallet = 0
                r_pallet +=1 #
            if r_pallet >= 4:  #行满 则z_pallet 加1 行列清零
                z_pallet +=1
                c_pallet = 0
                r_pallet = 0
            if z_pallet >= 4:  #高度满则全清零
                c_pallet = 0
                r_pallet = 0
                z_pallet = 0
            command_script.set_command("move_getObj_ready")



        #执行动作 用户可修改，和增加状态机
        # elif command == 'moveToP1':
        #     p1 = [0.1, 0.2, 0.3, 0, 0, 0]
        #     result=func_moveit2.move_pose(p1)
        #     if not result[0]:
        #         print("move_pose运动失败,执行错误处理")
        #         _components.errListUser.append_err(2001,"move_pose运动失败："+result[1])
        #         break
        #     command_script.set_command("moveToJoint")

        # elif command == 'moveToJoint':
        #     joint_angles_result = func_moveit2.get_current_joint_angles(7)
        #     joint_angles = joint_angles_result[1]
        #     joint_angles[5] += 3
        #     result=func_moveit2.move_joint(joint_angles)
        #     if not result[0]:
        #         print("move_joint运动失败,执行错误处理")
        #         _components.errListUser.append_err(2002, "move_joint运动失败:"+result[1])

        else:
            #todo 抛异常
            print("用户状态机值无效，检查command的值是否不存在状态， 抛出异常")
            _components.errListUser.append_err(2003, "用户状态机值无效，检查command的值是否不存在状态， 抛出异常")

    print("脚本结束")
    #####用户代码部分


    #APP退出 线程关闭
    #func_moveit2.destroy_node() #销毁moveit2 API通讯节点
    func_fr_ros2.destroy_node()
    fr_state_sub.destroy_node()
    #_components.close_ros2_launch() #关闭进程启动的ros2 launch节点
    rclpy.shutdown()  #关闭主节点启动的ros2
    #thread_func_moveit2.join()  #
    thread_gui.join()



if __name__ == "__main__":
    main()
    sys.exit()
