import string
import subprocess
import time
import threading
import logging
from logging.handlers import TimedRotatingFileHandler
import datetime
from multiprocessing.util import get_logger


#错误处理类
class ErrorListUser:

    # 错误编码规则  (模块号3位,0可以缺省) 001 + (函数号两位01) + (错误号2位)01
    # 初始化函数
    def __init__(self):
        print("ErrorUser初始化")

        #报警信息 列表
        self.__err_list=[]

        #报警日志
        self.__errLogger=get_logger()

        # 异常状态
        self.errStatus = False

        #反馈给界面的信号对象，从外部赋值
        self.Send_signal = None

        #暂停主脚本运行方法 对象
        self.start_pause_switch = None

    #获取日志logger引用
    def get_logger(self):
        # 获取当前日期字符串
        today = datetime.datetime.now().strftime("%Y-%m-%d")
        # 创建日志文件的路径，包含当前日期
        log_file_path = f"~/logs/error/{today}err.log"
        # 创建一个TimedRotatingFileHandler，每天轮转一次
        handler = TimedRotatingFileHandler(log_file_path, when="midnight")
        #
        logger_new = logging.getLogger('errorLog')
        logger_new.setLevel(logging.ERROR)
        # 将handler添加到logger中
        logger_new.addHandler(handler)
        #返回logger
        return logger_new


    #添加异常
    def append_err(self,err_code:int,mes:string):
        #状态切位异常
        self.errStatus=True

        #报错后 暂停脚本运行
        self.start_pause_switch(False)

        # 转换为字符串，格式为 YYYY-MM-DD HH:MM:SS
        str_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        #连接错误信息  时间 错误代码 错误信息
        err_mes=str_time+' 错误代码: '+str(err_code)+' '+mes
        #本地添加错误信息
        self.__err_list.append(err_mes)
        #异常日志存储错误信息
        self.__errLogger.error(err_mes)
        #给GUI发送添加异常信息 信号
        self.Send_signal.signal_errorMesAppend.emit(err_mes)


    #清除异常
    def clear_err(self):
        #清除报警
        self.errStatus = False
        self.__err_list.clear() #本地列表
        #界面清除显示界面
        self.Send_signal.signal_errorMesClear.emit()  # 清除界面报警信息

