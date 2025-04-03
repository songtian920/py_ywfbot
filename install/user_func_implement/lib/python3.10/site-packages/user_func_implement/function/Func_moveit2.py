import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_ywf_msg.action import MoveitYwfIntf
import time
from datetime import datetime
import threading

#模块号14
class FuncMoveit2(Node):
    #初始化函数
    def __init__(self):
        super().__init__('FuncMoveit2')
        self._get_result_future_moveit = None
        self._send_goal_future_moveit = None
        self._action_client_moveit = ActionClient(self, MoveitYwfIntf, 'MoveitYwfIntf')
        self.moveit_result_back_moveit = False
        self.result_code_str_moveit = ""
        self.result_mes_str_moveit = ""


    def send_goal(self, order):
        goal_msg = MoveitYwfIntf.Goal()
        goal_msg.order = order
        self._action_client_moveit.wait_for_server()
        self._send_goal_future_moveit = self._action_client_moveit.send_goal_async(goal_msg)
        self._send_goal_future_moveit.add_done_callback(self.goal_response_callback_moveit)


    def goal_response_callback_moveit(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        #返回异步result
        self._get_result_future_moveit = goal_handle.get_result_async()
        #异步result添加回调函数
        self._get_result_future_moveit.add_done_callback(self.get_result_callback_moveit)


    def get_result_callback_moveit(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        #解析结果
        str_split=result.result.split(":")
        self.result_code_str_moveit = str_split[0]
        self.result_mes_str_moveit = str_split[1]
        self.moveit_result_back_moveit = True


    # 返回值 (bool,mes_string)   bool反馈成功和失败，mes_string返回信息
    def config(self):
        # 将pose转换成字符创，分隔符是’,’  然后给moveit服务器发送命令
        self.get_logger().info("config in")
        goal_msg = MoveitYwfIntf.Goal()
        string_param = ""
        goal_msg.order = "config" + "(" + string_param + ")"
        self.moveit_result_back_moveit = False  # 重置结果返回标志
        self._action_client_moveit.wait_for_server()
        self._send_goal_future_moveit = self._action_client_moveit.send_goal_async(goal_msg)  # 发送命令
        self._send_goal_future_moveit.add_done_callback(self.goal_response_callback_moveit)  # 添加返回回调
        # 等待回调结果，并计算超时 一般是50s
        time_start = datetime.now()
        while  not self.moveit_result_back_moveit:
            time.sleep(0.1)
            time_stamp = (datetime.now() - time_start ).total_seconds()
            if time_stamp > 500:
                return False, "movePose指令超时，执行时间超过50s"
        # 等到结果返回，判断结果是否成功 1为成功
        if self.result_code_str_moveit == "1":
            return True, ""
        else:
            return False, self.result_mes_str_moveit


    #返回值 (bool,mes_string)   bool反馈成功和失败，mes_string返回信息
    #笛卡尔坐标位姿 运动
    def move_pose(self,pose) ->(bool,""):
        #将pose转换成字符创，分隔符是’,’  然后给moveit服务器发送命令
        goal_msg = MoveitYwfIntf.Goal()
        string_param=','.join(str(i) for i in pose) 
        goal_msg.order ="move_pose"+"("+string_param+")"
        self.moveit_result_back_moveit=False  #重置结果返回标志
        self._action_client_moveit.wait_for_server()
        self._send_goal_future_moveit = self._action_client_moveit.send_goal_async(goal_msg) #发送命令
        self._send_goal_future_moveit.add_done_callback(self.goal_response_callback_moveit) #添加返回回调
        #等待回调结果，并计算超时 一般是50s
        time_start=datetime.now()
        while not self.moveit_result_back_moveit:
            time.sleep(0.1)
            time_stamp=(datetime.now()-time_start).total_seconds()
            if time_stamp>50:
                return False, "movePose指令超时，执行时间超过50s"
        #等到结果返回，判断结果是否成功
        if self.result_code_str_moveit== "1":
            return True, ""
        else:
            return False,self.result_mes_str_moveit


    # 返回值 (bool,mes_string)   bool反馈成功和失败，mes_string返回信息
    #关节角度 运动
    def move_joint(self, joint_val) ->(bool,""):
        # 将pose转换成字符创，分隔符是’,’  然后给moveit服务器发送命令
        goal_msg = MoveitYwfIntf.Goal()
        string_param = ','.join(str(i) for i in joint_val)
        goal_msg.order = "move_joint" + "(" + string_param + ")"

        self.moveit_result_back_moveit = False  # 重置结果返回标志
        #等待服务器ok
        self._action_client_moveit.wait_for_server()
        # 发送命令
        self._send_goal_future_moveit = self._action_client_moveit.send_goal_async(goal_msg)
        # 添加accept回调
        self._send_goal_future_moveit.add_done_callback(self.goal_response_callback_moveit)  # 添加返回回调
        # 等待回调结果，并计算超时 一般是50s
        time_start = datetime.now()
        while not self.moveit_result_back_moveit:
            time.sleep(0.1)
            time_stamp = (datetime.now() - time_start).total_seconds()
            if time_stamp > 50:
                return False, "movePose指令超时，执行时间超过50s"
        # 等到结果返回，判断结果是否成功
        if self.result_code_str_moveit == "1":
            return True, ""
        else:
            return False, self.result_mes_str_moveit


    # 返回值 (bool,mes_string)   bool反馈成功和失败，mes_string返回信息
    #获取当前的关节角度
    def get_current_joint_angles(self,axis_num=6) -> [float]:
        # 将pose转换成字符创，分隔符是’,’  然后给moveit服务器发送命令
        goal_msg = MoveitYwfIntf.Goal()

        goal_msg.order = "get_current_joint_angles" + "(" +"none"+ ")"

        self.moveit_result_back_moveit = False  # 重置结果返回标志
        #等待服务器ok
        self._action_client_moveit.wait_for_server()
        # 发送命令
        self._send_goal_future_moveit = self._action_client_moveit.send_goal_async(goal_msg)
        # 添加accept回调
        self._send_goal_future_moveit.add_done_callback(self.goal_response_callback_moveit)  # 添加返回回调
        # 等待回调结果，并计算超时 一般是50s
        time_start = datetime.now()
        while not self.moveit_result_back_moveit:
            time.sleep(0.1)
            time_stamp = (datetime.now() - time_start).total_seconds()
            if time_stamp > 50:
                return False, "movePose指令超时，执行时间超过50s"
        # 等到结果返回，判断结果是否成功
        #构建浮点数角度list
        angles_float_list=[]
        #返回结果为1成功，转换数据
        if self.result_code_str_moveit == "1":
            #迭代转换字符串为浮点数list
            angle_str_list=self.result_mes_str_moveit.split(',')
            for index, angle in enumerate(angle_str_list):
                #Todo 关节数量
                if index<axis_num:
                    angles_float_list.append(float(angle))
                else :
                    break
            return True, angles_float_list #返回结果元组
        else:
            return False, angles_float_list #获取角度失败，返回结果元组



