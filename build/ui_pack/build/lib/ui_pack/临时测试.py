from PySide6.QtWidgets import QApplication
from PySide6.QtQuick import QQuickView, QQuickItem
from PySide6.QtCore import QUrl, Slot, QObject
from PySide6.QtQml import QJSValue, QmlElement
import time
from ui_main import Components



#
# # 定义槽函数
# @Slot(str)
# def manuel_auto_switch(self,mode):
#     print(mode)


# #主函数
# def open_gui(_components:Components):
#     #启动界面
#     app = QApplication([])
#     # 创建 QQuickView 实例
#     view = QQuickView()
#     # 加载 QML 文件
#     qml_file = "qml_main.qml"
#
#     view.setSource(QUrl.fromLocalFile(qml_file))
#     # 显示窗口，以便根对象可以被创建
#     view.show()
#     # 确保 QML 文件已成功加载
#     root_object = view.rootObject()
#     if root_object is None:
#         print("Failed to load QML file")
#         return -1
#
#     # 将槽函数连接到 QML 中的信号
#     root_object.win_home_manuel_auto_switch.connect(manuel_auto_switch)
#
#     return app.exec()
