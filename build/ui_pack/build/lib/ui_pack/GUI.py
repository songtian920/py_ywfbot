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

#
# class TableModel(QAbstractTableModel):
#     def __init__(self, data, headers):
#         super().__init__()
#         self._data = data
#         self._headers = headers
#
#     def rowCount(self, parent=QModelIndex()):
#         return len(self._data)
#
#     def columnCount(self, parent=QModelIndex()):
#         return len(self._headers)
#
#     def data(self, index, role=Qt.DisplayRole):
#         if role == Qt.DisplayRole:
#             return self._data[index.row()][index.column()]
#         return None
#
#     def headerData(self, section, orientation, role=Qt.DisplayRole):
#         if role == Qt.DisplayRole and orientation == Qt.Horizontal:
#             return self._headers[section]
#         return None
#
#     def setData(self, index, value, role=Qt.EditRole):
#         if role == Qt.EditRole:
#             self._data[index.row()][index.column()] = value
#             # 触发数据变化信号：通知视图刷新指定单元格
#             self.dataChanged.emit(index, index, [role])
#             return True
#         return False
#
#     def roleNames(self):
#         # 定义角色名，QML 通过角色名访问数据
#         roles = {
#             Qt.DisplayRole: b"display"
#         }
#         return roles
#
#     def insertRow(self, row, data, parent=QModelIndex()):
#         self.beginInsertRows(parent, row, row)
#         self._data.insert(row, data)
#         self.endInsertRows()
#         return True
#     def append(self, row, data, parent=QModelIndex()):
#         self.beginInsertRows(parent, row, row)
#         #self._data.insert(row, data)
#         self._data.append(data)
#         self.endInsertRows()
#         return True
#
#     def removeRow(self, row, parent=QModelIndex()):
#         self.beginRemoveRows(parent, row, row)
#         self._data.pop(row)
#         self.endRemoveRows()
#         return True


class YwfWidget:
    # 初始化函数
    def __init__(self):
        #super().__init__()  # 调用超类的构造函数
        self.view = None
        self.engine = None
        self.context = None
        self.qml = None
        self.signal_obj = None
        self.root_object = None
        self.components = None
        self.win_open_success = False
        print("初始化")


    #配置参数
    def config(self,_components:Components):
        self.components = _components


    # 定义槽函数
    #主界面 手动自动切换
    @Slot(bool)
    def manuel_auto_switch(self,mode:bool):
        #print(mode)
        # mode true为自动
        self.components.manuel_auto_switch(mode)


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

    #主界面 初始化成功后 给界面发信号反馈 初始化成功
    @Slot()
    def init_success_fail(self, status):
        self.signal_obj.init_success_signal.emit(1)

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

    #robotManual手臂手动 点位运动
    @Slot()
    def pointMove_robotManual(self, pointName:str,  moveType:str, exeFlag:int):

        if exeFlag == 1:  #开始运动
            if moveType == "MoveJoint":
                self.components.FuncFrRos2.MoveJoint(pointName)
            elif moveType == "MovePose":
                self.components.FuncFrRos2.MovePose(pointName)
            elif moveType == "MoveL_Joint":
                self.components.FuncFrRos2.MoveL_Joint(pointName)
            elif moveType == "MoveL_Pose":
                self.components.FuncFrRos2.MoveL_Pose(pointName)

        else:
            #停止运动
            self.components.FuncFrRos2.StopMotion()
            pass

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
                point_cart_data_tran[3] = point_cart_data[3] * 180 / 3.1415926
                point_cart_data_tran[4] = point_cart_data[4] * 180 / 3.1415926
                point_cart_data_tran[5] = point_cart_data[5] * 180 / 3.1415926
                data_send = [joint[0],joint[1],joint[2],joint[3],joint[4],joint[5],
                             point_cart_data_tran[0],point_cart_data_tran[1],point_cart_data_tran[2],
                             point_cart_data_tran[3],point_cart_data_tran[4],point_cart_data_tran[5]]

                #数据发送到GUI
                self.signal_obj.signal_teach_frPointsList.emit(data_send)
        except:
            self.components.errListUser.append_err(112601, "getJoints_currentValue获取指定关节角错误，请检查机器人通讯")

    @Slot()
    def fr_execute_point(self,point_name,flag_exe):
        if flag_exe == 1:
            self.components.FuncFrRos2.MoveJoint(point_name)
        else:
            self.components.FuncFrRos2.StopMotion()


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
        res = self.components.FuncFrRos2.teach_point_calibTool(point_index)  #示教点位 标定工具坐标系
        if res:
            #浅拷贝数据  转换单位
            pose_rb = self.components.FuncFrRos2.list_pose_calibTool[point_index]
            pose_rb_trans =[0.0,0.0,0.0,0.0,0.0,0.0]
            # 转换成mm和角度发送到界面
            pose_rb_trans[0] = round(pose_rb[0] * 1000,3)
            pose_rb_trans[1] = round(pose_rb[1] * 1000,3)
            pose_rb_trans[2] = round(pose_rb[2] * 1000,3)
            pose_rb_trans[3] = round(pose_rb[3] * 180 / 3.1415926,3)
            pose_rb_trans[4] = round(pose_rb[4] * 180 / 3.1415926,3)
            pose_rb_trans[5] = round(pose_rb[5] * 180 / 3.1415926,3)
            self.signal_obj.signal_teachPoint_calibTool_callBack.emit(pose_rb_trans,point_index)

    #计算新工具的平移变换
    @Slot()
    def calculate_tool_trans(self):
        #获取示教的标定点位
        pose =  self.components.FuncFrRos2.list_pose_calibTool
        pose_toolInBase = [pose[0], pose[1], pose[3]]  #需要给定的坐标列表
        #将点位赋值给计算新工具平移变换，计算新工具平移变换
        x,y,z = self.components.FuncFrRos2.calib_translate_tool(pose_toolInBase)
        #回传到界面

    #计算新 用户坐标系
    @Slot()
    def calculate_user_coord(self):
        try:
            #测试生成 位姿
            pose_o = [0.0,0.0,0.0]
            pose_x = [2.0,2.0,0.0]
            pose_y = [-2.0,2.0,0.0]
            pose_z = [0.0,0.0,1.0]
            #计算出用户新坐标
            self.components.FuncFrRos2.calib_user_coord([pose_o,pose_x,pose_y,pose_z])
            #
        except:
            print("calculate_user_coord")


    # fr用户坐标系User
    # 手臂用户坐标系表删除一行
    @Slot()
    def deleteDbRow_robotUserList_main(self, name_str):
        rlt = self.components.FuncFrRos2.deleteDbRow_robotUserList(name_str)

    # 手臂用户坐标从 tableView更新到数据库及内存
    @Slot()
    def updateDbRow_robotUserList_main(self, name_str, str_list):
        print(str_list)
        self.components.FuncFrRos2.updateDbRow_robotUserList(name_str, str_list)

    # 手臂用户坐标系插入插入
    @Slot()
    def insert_row_robotUserList_main(self, name_str):
        rlt = self.components.FuncFrRos2.insert_newRowInRobotUserList(name_str)

    # 手臂用户坐标点表刷新，从数据库读取到 点表tableView，并加载到内存及注册到fr_ros2服务器
    @Slot()
    def refresh_DBUserToTableView(self):
        self.components.FuncFrRos2.fr_load_usersFromDB()  #用户坐标系表加载
        self.components.FuncFrRos2.users_tf_static_broadcaster()  #用户坐标系tf发布

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
        #手臂手动
        self.root_object.signal_StartJOG_main.connect(self.StartJOG,Qt.QueuedConnection)  #开始JOG信号绑定 槽函数
        self.root_object.signal_StopJOG_main.connect(self.StopJOG)   #停止JOG信号绑定 槽函数
        self.root_object.signal_WorkSpaceChange_main.connect(self.ChangeWorkSpace)  #改变工作空间
        self.root_object.signal_toolChange_main.connect(self.ChangeFrTool)  #改变fr工具坐标系
        self.root_object.signal_userChange_main.connect(self.ChangeFrUser)  #改变fr用户坐标系
        self.root_object.signal_pointMove_main.connect(self.pointMove_robotManual)  #点位运动执行

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
        #self.root_object.signal_update_tool_trans_main.connect()
        #todo 测试新工具坐标系 计算旋转变换
        self.root_object.signal_calculate_tool_rotate_main.connect(self.calculate_user_coord)

        # 手臂用户坐标系表
        #删
        self.root_object.signal_deleteDbRow_robotUserList_main.connect(self.deleteDbRow_robotUserList_main)
        #改
        self.root_object.signal_updateDbRow_robotUserList_main.connect(self.updateDbRow_robotUserList_main)
        #增
        self.root_object.signal_insert_row_robotUserList_main.connect(self.insert_row_robotUserList_main)
        #查
        self.root_object.signal_refresh_DBUserToTableView_main.connect(self.refresh_DBUserToTableView)

        #测试
        #self.root_object.mySignal.connect(self.signal_subQml_test)
        print("已打开")
        self.win_open_success = True
        app.exec()
        self.components.existApp = True
        return 0


