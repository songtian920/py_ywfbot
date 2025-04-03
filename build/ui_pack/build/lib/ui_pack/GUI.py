from PySide6.QtWidgets import QApplication
from PySide6.QtQuick import QQuickView, QQuickItem
from PySide6.QtCore import QUrl, Slot, QObject,Signal, QThread,Qt,QAbstractTableModel,QModelIndex
from PySide6.QtQml import QJSValue, QmlElement,QQmlApplicationEngine
from PySide6.QtGui import QGuiApplication

import time
from components import Components
from components import SendSignal
from rclpy.parameter import Parameter
import asyncio
import os
import math

#模块号02
class YwfWidget:
    # 初始化函数
    def __init__(self,_components):
        #super().__init__()  # 调用超类的构造函数
        self.view = None
        self.engine = None
        self.context = None
        self.qml = None
        self.signal_obj = None
        self.root_object = None
        self.components = _components
        self.win_open_success = False
        print("初始化")

    # 定义槽函数
    #主界面 手动自动切换
    @Slot(bool)
    def manuel_auto_switch(self,mode:bool):
        #print(mode)
        try:
            # mode true为自动 false为手动
            #手自动模式切换
            self.components.manuel_auto_switch(mode)

        except Exception as e:
            self.components.errListUser.append_err(112601, "设置Mode错误："+str(e))

    #主界面 开始和暂停状态切换
    @Slot(bool)
    def start_pause_switch(self,mode:bool):
        start_status = self.components.start_pause_switch(mode)

    #主界面 清除报警
    @Slot()
    def clear_error_list(self):
        self.components.FuncFrRos2.ResetAllError()  #给机器人发送清除报警
        self.components.errListUser.clear_err() #清除本地报警信息

    @Slot()
    def signal_subQml_test(self):
        print("subQml test successful!")

    #主界面打开关闭 fr机器人手动控制信息回传
    @Slot()
    def robotManuelSwitch(self,_switch):
        self.components.guiDispState.RobotManualDisp_switch = _switch

    #主界面打开IO监控信息回传
    @Slot()
    def IoSetSwitch(self,_switch):
        self.components.guiDispState.IoSetDisp_switch = _switch

    #robotManual手臂手动 JOG
    @Slot()
    def StartJOG(self,nb,direct):
        print("JOG start")
        #asyncio.run(self.async_StartJOG(nb,direct))
        self.components.FuncFrRos2.StartJOG(nb,direct)

    #robotManual手臂手动 停止JOG
    @Slot()
    def StopJOG(self):
        print("JOG stop")
        #asyncio.run(self.async_StopJOG())
        self.components.FuncFrRos2.StopJOG()

    #robotManual手臂手动 改变工作空间
    @Slot()
    def ChangeWorkSpace(self,ref_workspace):
        self.components.FuncFrRos2.work_space=ref_workspace
        self.components.FrStateSub.work_space=ref_workspace

    #robotManual手臂手动 改变工具坐标系
    @Slot()
    def ChangeFrTool(self,tool_name):
        self.components.FuncFrRos2.tool_selected=tool_name
        self.components.FrStateSub.tool_selected=tool_name

    #robotManual手臂手动改变用户坐标系Point
    @Slot()
    def ChangeFrUser(self,user_name):
        self.components.FuncFrRos2.user_selected = user_name
        self.components.FrStateSub.user_selected = user_name

    #robotManual手臂伺服使能
    @Slot()
    def robotEnable_robotManual(self,state):
        self.components.FuncFrRos2.RobotEnable(state)
        #robotEnable关闭，则强制切换为手动模式
        if state==0:
            # mode false为手动
            self.components.manuel_auto_switch(False)

    #robotManual手臂手动 点位运动
    @Slot()
    def pointMove_robotManual(self, pointName:str,  moveType:str, exeFlag:int):

        if exeFlag == 1:  #开始运动
            if moveType == "MoveJoint":
                self.components.FuncFrRos2.MoveJoint(pointName,block = False)
            elif moveType == "MovePose":
                self.components.FuncFrRos2.MovePose(pointName,block = False)
            elif moveType == "MoveL_Joint":
                self.components.FuncFrRos2.MoveL_Joint(pointName,block = False)
            elif moveType == "MoveL_Pose":
                self.components.FuncFrRos2.MoveL_Pose(pointName,block = False)

        else:
            #停止运动
            #self.components.FuncFrRos2.StopMotion()
            pass

    #AmrStatusDisp状态显示
    @Slot()
    def robotSetSpeed(self,speed_val):
        try:
            self.components.FuncFrRos2.SetSpeed(speed_val)
        except Exception as e:
            self.components.errListUser.append_err(112601, "robotSetSpeed错误："+str(e))

    #手臂点表删除一行
    @Slot()
    def deleteDbRow_robotPointList_main(self,name_str):
        rlt=self.components.FuncFrRos2.deleteDbRow_robotPointList(name_str)

    # 手臂点位从 tableView更新到数据库及内存
    @Slot()
    def updateDbRow_robotPointList_main(self,name_str, str_list):
        print(str_list)
        self.components.FuncFrRos2.updateDbRow_robotPointList(name_str,str_list)

    #手臂点表插入
    @Slot()
    def insert_row_robotPointList_main(self,name_str):
        rlt = self.components.FuncFrRos2.insert_newRowInRobotPointList(name_str)

    #手臂点表刷新，从数据库读取到 点表tableView，并加载到内存及注册到fr_ros2服务器
    @Slot()
    def refresh_DBPointToTableView(self):
        self.components.FuncFrRos2.fr_load_pointsFromDB()  #加载点位表
        self.components.FuncFrRos2.points_tf_static_broadcaster()  #发布点表tf位姿变换

    #手臂点表示教
    @Slot()
    def teach_pointList(self,point_name):
        try:
            #FrState取出数据转换并发送给gui表格
            tool = self.components.FuncFrRos2.dic_point[point_name][1]
            user = self.components.FuncFrRos2.dic_point[point_name][2]
            res_joint,joint = self.components.FuncFrRos2.getJoints_currentValue()
            res_cart,point_cart_data = self.components.FuncFrRos2.getTfPos_currentCoordinate(tool,user)
            #转换结果成功，则向GUI发送数据
            point_cart_data_tran =[0.0,0.0,0.0,0.0,0.0,0.0]
            if res_joint & res_cart:
                #转换成mm和角度发送到界面
                point_cart_data_tran[0] = point_cart_data[0] * 1000
                point_cart_data_tran[1] = point_cart_data[1] * 1000
                point_cart_data_tran[2] = point_cart_data[2] * 1000
                point_cart_data_tran[3] = point_cart_data[3] * 180 / math.pi
                point_cart_data_tran[4] = point_cart_data[4] * 180 / math.pi
                point_cart_data_tran[5] = point_cart_data[5] * 180 / math.pi
                data_send = [joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],
                             point_cart_data_tran[0],point_cart_data_tran[1],point_cart_data_tran[2],
                             point_cart_data_tran[3],point_cart_data_tran[4],point_cart_data_tran[5]]

                #将数据更新到数据库
                data_DB = [self.components.FuncFrRos2.dic_point[point_name][0],
                           self.components.FuncFrRos2.dic_point[point_name][1],
                           self.components.FuncFrRos2.dic_point[point_name][2],
                           data_send[0],data_send[1],data_send[2],data_send[3],data_send[4],data_send[5],
                           data_send[6],data_send[7],data_send[8],data_send[9],data_send[10],data_send[11]]
                self.components.FuncFrRos2.updateDbRow_robotPointList(point_name, data_DB)
                #数据发送到GUI
                self.signal_obj.signal_teach_frPointsList.emit(data_send)
        except:
            self.components.errListUser.append_err(112601, "getJoints_currentValue获取指定关节角错误，请检查机器人通讯")

    @Slot()
    def fr_execute_point(self,point_name,flag_exe):
        if flag_exe == 1:
            self.components.FuncFrRos2.MoveJoint(point_name,block = False)
        else:
            pass
            #self.components.FuncFrRos2.StopMotion()


    #fr工具坐标系Tool
    # 手臂工具坐标系表删除一行
    @Slot()
    def deleteDbRow_robotToolList_main(self, name_str):
        rlt = self.components.FuncFrRos2.deleteDbRow_robotToolList(name_str)

    # 手臂工具坐标从 tableView更新到数据库及内存
    @Slot()
    def updateDbRow_robotToolList_main(self, name_str, str_list):
        print(str_list)
        self.components.FuncFrRos2.updateDbRow_robotToolList(name_str, str_list)

    # 手臂工具坐标系插入插入
    @Slot()
    def insert_row_robotToolList_main(self, name_str):
        rlt = self.components.FuncFrRos2.insert_newRowInRobotToolList(name_str)

    # 手臂工具坐标系刷新，从数据库读取到 点表tableView，并加载到内存及注册到fr_ros2服务器
    @Slot()
    def refresh_DBToolToTableView(self):
        self.components.FuncFrRos2.fr_load_toolsFromDB()  #加载工具坐标系表
        self.components.FuncFrRos2.tools_tf_static_broadcaster()  #工具坐标系tf发布

    #自定义新工具坐标系 示教点位
    @Slot()
    def teach_point_calibTool(self,point_index):
        try:
            res = self.components.FuncFrRos2.teach_point_calibTool(point_index)  #示教点位 标定工具坐标系
            if res:
                #浅拷贝数据  转换单位
                pose_rb = self.components.FuncFrRos2.list_pose_calibTool_temp[point_index]
                pose_rb_trans =[0.0,0.0,0.0,0.0,0.0,0.0]
                # 转换成mm和角度发送到界面
                pose_rb_trans[0] = round(pose_rb[0] * 1000,3)
                pose_rb_trans[1] = round(pose_rb[1] * 1000,3)
                pose_rb_trans[2] = round(pose_rb[2] * 1000,3)
                pose_rb_trans[3] = round(pose_rb[3] * 180 / math.pi,3)
                pose_rb_trans[4] = round(pose_rb[4] * 180 / math.pi,3)
                pose_rb_trans[5] = round(pose_rb[5] * 180 / math.pi,3)
                self.signal_obj.signal_teachPoint_calibTool_callBack.emit(pose_rb_trans,point_index)
        except Exception as e:
            self.components.errListUser.append_err(112601, "teach_point_calibTool错误:" + str(e))


    #计算新工具的平移变换
    @Slot()
    def calculate_tool_trans(self):
        try:
            #获取示教的标定点位
            pose =  self.components.FuncFrRos2.list_pose_calibTool_temp
            pose_toolInBase = [pose[0], pose[1], pose[2]]  #需要给定的坐标列表
            #将点位赋值给计算新工具平移变换，计算新工具平移变换
            x,y,z = self.components.FuncFrRos2.calib_translate_tool(pose_toolInBase)

            #计算平移变换结果 回传到界面
            list_pose = [0.0,0.0,0.0]
            list_pose[0] = round(float(x),3)
            list_pose[1] = round(float(y),3)
            list_pose[2] = round(float(z),3)
            self.signal_obj.signal_calculateTool_calibTool_callback.emit(list_pose,"translate")
        except Exception as e:
            self.components.errListUser.append_err(112601, "calculate_tool_trans错误:" + str(e))

    #更新保存新工具的平移变换
    @Slot()
    def update_tool_trans(self,name_select):
        try:
            # 更新保存数据库的新工具坐标系 平移变换数据
            res_update_db = self.components.FuncFrRos2.update_tool_translate_DB(name_select)
            if not res_update_db:
                self.components.errListUser.append_err(112601, "update_tool_trans更新到数据库错误")
            #同时将 新工具坐标系 平移变换数据更新到界面显示
            list_data = [float(self.components.FuncFrRos2.x_trans_toolCalib_temp),float(self.components.FuncFrRos2.y_trans_toolCalib_temp),
                         float(self.components.FuncFrRos2.z_trans_toolCalib_temp)]
            self.signal_obj.signal_updateToolData_calibTool_callback.emit(list_data,"translate")

        except Exception as e:
            self.components.errListUser.append_err(112601, "update_tool_trans错误:"+str(e))

    #计算新工具 旋转变换
    @Slot()
    def calculate_tool_rotate(self):
        try:
            # 获取示教的标定点位
            pose = self.components.FuncFrRos2.list_pose_calibTool_temp
            pose_toolInBase = [pose[3], pose[4], pose[5]]  # 需要给定的坐标列表
            # 将点位赋值给计算新工具平移变换，计算新工具平移变换
            x_rotate,y_rotate,z_rotate = self.components.FuncFrRos2.calib_rotate_tool(pose_toolInBase)
            # 计算平移变换结果 回传到界面
            list_pose = [0.0, 0.0, 0.0]
            list_pose[0] = round(float(x_rotate), 3)
            list_pose[1] = round(float(y_rotate), 3)
            list_pose[2] = round(float(z_rotate), 3)
            self.signal_obj.signal_calculateTool_calibTool_callback.emit(list_pose, "rotate")
        except Exception as e:
            self.components.errListUser.append_err(112601, "calculate_tool_rotate错误:" + str(e))

    # 更新保存新工具的旋转变换
    @Slot()
    def update_tool_rotate(self, name_select):
        try:
            # 更新保存数据库的新工具坐标系 平移变换数据
            res_update_db = self.components.FuncFrRos2.update_tool_rotate_DB(name_select)
            if not res_update_db:
                self.components.errListUser.append_err(112601, "update_tool_rotate更新到数据库错误")
            # 同时将 新工具坐标系 平移变换数据更新到界面显示
            list_data = [float(self.components.FuncFrRos2.x_rotate_toolCalib_temp),
                         float(self.components.FuncFrRos2.y_rotate_toolCalib_temp),
                         float(self.components.FuncFrRos2.z_rotate_toolCalib_temp)]
            self.signal_obj.signal_updateToolData_calibTool_callback.emit(list_data, "rotate")

        except Exception as e:
            self.components.errListUser.append_err(112601, "update_tool_rotate错误:" + str(e))

    # fr用户坐标系User
    # 手臂用户坐标系表删除一行
    @Slot()
    def deleteDbRow_robotUserList_main(self, name_str):
        try:
            rlt = self.components.FuncFrRos2.deleteDbRow_robotUserList(name_str)
        except Exception as e:
            self.components.errListUser.append_err(112601, "deleteDbRow_robotUserList_main错误:" + str(e))

    # 手臂用户坐标从 tableView更新到数据库及内存
    @Slot()
    def updateDbRow_robotUserList_main(self, name_str, str_list):
        try:
            print(str_list)
            self.components.FuncFrRos2.updateDbRow_robotUserList(name_str, str_list)
        except Exception as e:
            self.components.errListUser.append_err(112601, "updateDbRow_robotUserList_main错误:" + str(e))

    # 手臂用户坐标系插入插入
    @Slot()
    def insert_row_robotUserList_main(self, name_str):
        try:
            rlt = self.components.FuncFrRos2.insert_newRowInRobotUserList(name_str)
        except Exception as e:
            self.components.errListUser.append_err(112601, "insert_row_robotUserList_main错误:" + str(e))

    # 手臂用户坐标点表刷新，从数据库读取到 点表tableView，并加载到内存及注册到fr_ros2服务器
    @Slot()
    def refresh_DBUserToTableView(self):
        try:
            self.components.FuncFrRos2.fr_load_usersFromDB()  #用户坐标系表加载
            self.components.FuncFrRos2.users_tf_static_broadcaster()  #用户坐标系tf发布
        except Exception as e:
            self.components.errListUser.append_err(112601, "refresh_DBUserToTableView错误:" + str(e))

    # 自定义新用户坐标系 示教点位
    @Slot()
    def teach_point_calibUser(self, point_index):
        try:
            res = self.components.FuncFrRos2.teach_point_calibUser(point_index)  # 示教点位 标定用户坐标系
            if res:
                # 浅拷贝数据  转换单位
                pose_rb = self.components.FuncFrRos2.list_pose_calibUser_temp[point_index]
                pose_rb_trans = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                # 转换成mm和角度发送到界面
                pose_rb_trans[0] = round(pose_rb[0] * 1000, 3)
                pose_rb_trans[1] = round(pose_rb[1] * 1000, 3)
                pose_rb_trans[2] = round(pose_rb[2] * 1000, 3)
                pose_rb_trans[3] = round(pose_rb[3] * 180 / math.pi, 3)
                pose_rb_trans[4] = round(pose_rb[4] * 180 / math.pi, 3)
                pose_rb_trans[5] = round(pose_rb[5] * 180 / math.pi, 3)
                self.signal_obj.signal_teachPoint_calibUser_callBack.emit(pose_rb_trans, point_index)
        except Exception as e:
            self.components.errListUser.append_err(112601, "teach_point_calibTool错误:" + str(e))

    # 计算新 用户坐标系
    @Slot()
    def calculate_user_coord(self):
        try:
            # 获取示教的标定点位
            pose = self.components.FuncFrRos2.list_pose_calibUser_temp
            point_o = [pose[0][0], pose[0][1], pose[0][2]]
            point_x = [pose[1][0], pose[1][1], pose[1][2]]
            point_y = [pose[2][0], pose[2][1], pose[2][2]]
            # 将点位赋值给计算新工具平移变换，计算新工具平移变换
            x,y,z,x_rotate,y_rotate,z_rotate = (
                self.components.FuncFrRos2.calib_user_coord([point_o,point_x,point_y]))
            # 计算平移变换结果 回传到界面
            list_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            list_pose[0] = round(float(x), 3)
            list_pose[1] = round(float(y), 3)
            list_pose[2] = round(float(z), 3)
            list_pose[3] = round(float(x_rotate), 3)
            list_pose[4] = round(float(y_rotate), 3)
            list_pose[5] = round(float(z_rotate), 3)
            self.signal_obj.signal_calculateUser_calibUser_callback.emit(list_pose)
        except Exception as e:
            self.components.errListUser.append_err(112601, "calculate_tool_rotate错误:" + str(e))

    # 更新保存新用户的旋转变换
    @Slot()
    def update_user_coord(self, name_select):
        try:
            # 更新保存数据库的新用户坐标系
            res_update_db = self.components.FuncFrRos2.update_user_coord_DB(name_select)
            if not res_update_db:
                self.components.errListUser.append_err(112601, "update_user_coord更新到数据库错误")
            # 同时将 新工具坐标系 平移变换数据更新到界面显示
            list_data = [float(self.components.FuncFrRos2.x_trans_userCalib_temp),
                         float(self.components.FuncFrRos2.y_trans_userCalib_temp),
                         float(self.components.FuncFrRos2.z_trans_userCalib_temp),
                         float(self.components.FuncFrRos2.x_rotate_userCalib_temp),
                         float(self.components.FuncFrRos2.y_rotate_userCalib_temp),
                         float(self.components.FuncFrRos2.z_rotate_userCalib_temp)]
            self.signal_obj.signal_updateUserData_calibUser_callback.emit(list_data)

        except Exception as e:
            self.components.errListUser.append_err(112601, "update_user_coord:" + str(e))

    #manual
    #robotSetDo
    @Slot()
    def robot_setDo(self,id_,status_):
        try:
            self.components.FuncFrRos2.SetDO(id_,status_)
        except Exception as e:
            self.components.errListUser.append_err(112601, "robot_setDo:" + str(e))

    #robotIO save tag保存标签
    @Slot()
    def robot_save_DIO_Tag(self,type_,id_,status):
        try:
            if type_ == "input":
                #更新输入坐标标签
                self.components.FuncFrRos2.update_tag_robotDI(id_,status)
            elif type_ == "output":
                self.components.FuncFrRos2.update_tag_robotDO(id_,status)
        except Exception as e:
            self.components.errListUser.append_err(112601, "保存机器人IO点位标签:" + str(e))

    #码垛工艺包
    #新增一行 码垛包
    @Slot()
    def insert_NewPalletProcess(self,name_str):
        try:
            self.components.palletProcess.insert_newPalletProcess(name_str)
        except Exception as e:
            self.components.errListUser.append_err(112601, "插入新码垛工艺参数:" + str(e))

    #刷新码垛工艺表
    @Slot()
    def refreshData_palletProcess(self):
        try:
            self.components.palletProcess.loadParam_From_palletProcessDB()
        except Exception as e:
            self.components.errListUser.append_err(112601, "刷新工艺包数据报错:" + str(e))

    #删除码垛工艺表 一行
    @Slot()
    def deleteRow_palletProcess(self,name_str):
        try:
            self.components.palletProcess.deleteRow_ParamPalletProcess(name_str)
        except Exception as e:
            self.components.errListUser.append_err(112601, "删除工艺包数据报错:" + str(e))

    #更新一行数据
    @Slot()
    def updateRow_palletProcess(self,name_str,str_list):
        try:
            self.components.palletProcess.updateDbRow_palletProcess(name_str,str_list)
        except Exception as e:
            self.components.errListUser.append_err(112601, "更新工艺包数据报错:" + str(e))

    @Slot()
    def getTool_User_PointNameToLoadCombox(self):
        tool_list = list(self.components.FuncFrRos2.dic_tool.keys())
        user_list = list(self.components.FuncFrRos2.dic_user.keys())
        pointName_list = list(self.components.FuncFrRos2.dic_point.keys())
        # 向qml发送信号传递值
        self.signal_obj.signal_sendToolUserPointNames_callback.emit(tool_list,user_list,pointName_list)


    # 主界面 初始化成功后 给界面发信号反馈 初始化成功
    def init_success_fail(self, status):
        self.signal_obj.init_success_signal.emit(1)

    #显示窗口
    def show_widget(self):
        # 显示窗口，以便根对象可以被创建
        print("打开用户窗口")
        self.win_open_success = False
        # 设置环境变量
        os.environ["QT_IM_MODULE"] = "qtvirtualkeyboard"

        # #配置robot point表格数据模型
        # data = []
        # headers = ["update", "delete", "teach","exec","name","comment",
        #            "tool","user","joint1","joint2","joint3","joint4","joint5","joint6",
        #            "x","y","z","xRot","yRot","zRot"]
        # model = TableModel(data, headers)

        app = QApplication([])
        self.view = QQuickView()
        self.engine = self.view.engine()  # 获取QQuickView的引擎
        self.context = self.engine.rootContext()  # 获取引擎的根上下文

        # 创建信号对象
        self.signal_obj = SendSignal()
        # 设置上下文属性，使其在QML中可访问
        self.context.setContextProperty("signal_obj", self.signal_obj)
        # self.context.setContextProperty("tableModel", model)
        self.qml = "qml/qml_main.qml"
        self.view.setSource(QUrl.fromLocalFile(self.qml))
        self.view.show()
        print("已打开")
        # 确保 QML 文件已成功加载
        self.root_object = self.view.rootObject()
        if self.root_object is None:
            print("Failed to load QML file")
            return -1

        # 将槽函数连接到 QML 中的信号
        #总控
        self.root_object.signal_manuel_auto_switch.connect(self.manuel_auto_switch)
        self.root_object.signal_start_pause_switch.connect(self.start_pause_switch)
        self.root_object.signal_clear_errorList.connect(self.clear_error_list)
        self.root_object.signal_robotManuelSwitch.connect(self.robotManuelSwitch)
        self.root_object.signal_IoSetSwitch.connect(self.IoSetSwitch)

        #手臂手动RobotManual
        self.root_object.signal_StartJOG_main.connect(self.StartJOG,Qt.QueuedConnection)  #开始JOG信号绑定 槽函数
        self.root_object.signal_StopJOG_main.connect(self.StopJOG)   #停止JOG信号绑定 槽函数
        self.root_object.signal_WorkSpaceChange_main.connect(self.ChangeWorkSpace)  #改变工作空间
        self.root_object.signal_toolChange_main.connect(self.ChangeFrTool)  #改变fr工具坐标系
        self.root_object.signal_userChange_main.connect(self.ChangeFrUser)  #改变fr用户坐标系
        self.root_object.signal_pointMove_main.connect(self.pointMove_robotManual)  #点位运动执行
        self.root_object.signal_robotEnable_main.connect(self.robotEnable_robotManual)   #机器人使能

        #AmrStatusDisp状态显示
        self.root_object.signal_robotSpeed_main.connect(self.robotSetSpeed)  #机器人速度改变

        #手臂点表
        #删
        self.root_object.signal_deleteDbRow_robotPointList_main.connect(self.deleteDbRow_robotPointList_main)
        #改
        self.root_object.signal_updateDbRow_robotPointList_main.connect(self.updateDbRow_robotPointList_main)
        #增
        self.root_object.signal_insert_row_robotPointList_main.connect(self.insert_row_robotPointList_main)
        #查
        self.root_object.signal_refresh_DBPointToTableView_main.connect(self.refresh_DBPointToTableView)
        #点位示教信号
        self.root_object.signal_point_teach_main.connect(self.teach_pointList)
        #点位执行信号
        self.root_object.signal_execute_point_main.connect(self.fr_execute_point)

        # 手臂工具坐标系表
        #删
        self.root_object.signal_deleteDbRow_robotToolList_main.connect(self.deleteDbRow_robotToolList_main)
        #改
        self.root_object.signal_updateDbRow_robotToolList_main.connect(self.updateDbRow_robotToolList_main)
        #增
        self.root_object.signal_insert_row_robotToolList_main.connect(self.insert_row_robotToolList_main)
        #查
        self.root_object.signal_refresh_DBToolToTableView_main.connect(self.refresh_DBToolToTableView)
        #新工具坐标系标定 示教点位
        self.root_object.signal_teach_tool_point_main.connect(self.teach_point_calibTool)
        #新工具坐标系标定 计算平移变换
        self.root_object.signal_calculate_tool_trans_main.connect(self.calculate_tool_trans)
        #新工具坐标系标定 平移变换更新到界面及数据库
        self.root_object.signal_update_tool_trans_main.connect(self.update_tool_trans)
        # 新工具坐标系标定 计算旋转变换
        self.root_object.signal_calculate_tool_rotate_main.connect(self.calculate_tool_rotate)
        # 新工具坐标系标定 保存旋转变换
        self.root_object.signal_update_tool_rotate_main.connect(self.update_tool_rotate)

        # 手臂用户坐标系表
        #删
        self.root_object.signal_deleteDbRow_robotUserList_main.connect(self.deleteDbRow_robotUserList_main)
        #改
        self.root_object.signal_updateDbRow_robotUserList_main.connect(self.updateDbRow_robotUserList_main)
        #增
        self.root_object.signal_insert_row_robotUserList_main.connect(self.insert_row_robotUserList_main)
        #查
        self.root_object.signal_refresh_DBUserToTableView_main.connect(self.refresh_DBUserToTableView)
        #新用户示教点位
        self.root_object.signal_teach_user_point_userTable_main.connect(self.teach_point_calibUser)
        #计算新用户位姿变换
        self.root_object.signal_calculate_user_coord_userTable_main.connect(self.calculate_user_coord)
        #更新新用户在基坐标系位姿
        self.root_object.signal_update_user_coord_userTable_main.connect(self.update_user_coord)

        #manual
        #手臂IO点控制
        self.root_object.signal_SetDO_RobotIoSet_main.connect(self.robot_setDo)
        #输入输出标签更新
        self.root_object.signal_update_RobotDIO_tag_main.connect(self.robot_save_DIO_Tag)

        #码垛工艺包
        #新增一个码垛工艺
        self.root_object.signal_insert_newPalletProcess_main.connect(self.insert_NewPalletProcess)
        #刷新码垛工艺表
        self.root_object.signal_refreshData_PalletProcess_main.connect(self.refreshData_palletProcess)
        #删除参数行
        self.root_object.signal_deleteDbRow_palletProcess_main.connect(self.deleteRow_palletProcess)
        #更新数据
        self.root_object.signal_updateDbRow_palletProcess_main.connect(self.updateRow_palletProcess)
        #获取tool user names信号
        self.root_object.signal_getTool_User_PointNameToLoadCombox_main.connect(self.getTool_User_PointNameToLoadCombox)

        print("已打开")
        self.win_open_success = True
        app.exec()
        self.components.existApp = True
        return 0


