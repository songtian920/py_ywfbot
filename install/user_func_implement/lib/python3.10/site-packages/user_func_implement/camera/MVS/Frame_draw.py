# -*- coding:utf-8 -*-
"""
time:2022-09-01
开发用于功能测试的文件，工业相机的取流，加速等功能。输出图片
开发前请前往海康工业相机官网下载工业相机客户端：链接地址：https://www.hikrobotics.com/cn/machinevision/service/download?module=0
开发参考链接：https://blog.csdn.net/qq_39570716/article/details/114066097?spm=1001.2014.3001.5506
应用平台：ubuntu 20.04
"""
import numpy as np
import cv2
import sys
import os
import datetime
curPath = os.path.abspath(os.path.dirname(__file__))
# curPath = os.path.abspath(os.path.dirname(os.getcwd()))
rootPath = os.path.split(curPath)[0]
sys.path.insert(0, rootPath)  # 建议程序前都添加此代码，指定相对路径
print("rootpath", rootPath)
# /home/ubuntu/Ai_project/Liming/PaddleDetection/codes/Fetch/util
# /home/ubuntu/Ai_project/Liming/PaddleDetection/codes/Fetch/lib/MvImport/MvCameraControl_class.py
from MvImport.MvCameraControl_class import *
from util import config as config
import requests
import time
import json
import base64
# 日志所需依赖
# 日志
import logging
from logging import handlers
# 创建全局变量
global alllog
global errorlog

# 类-日志
class Logger(object):
    level_relations = {
        'debug': logging.DEBUG,
        'info': logging.INFO,
        'warning': logging.WARNING,
        'error': logging.ERROR,
        'crit': logging.CRITICAL
    }  # 日志级别关系映射

    def __init__(self, filename, level='info', when='D', backCount=3,
                 fmt='%(asctime)s - %(pathname)s[line:%(lineno)d] - %(levelname)s: %(message)s'):
        self.logger = logging.getLogger(filename)
        format_str = logging.Formatter(fmt)  # 设置日志格式
        self.logger.setLevel(self.level_relations.get(level))  # 设置日志级别
        # sh = logging.StreamHandler()#往屏幕上输出
        # sh.setFormatter(format_str) #设置屏幕上显示的格式
        th = handlers.TimedRotatingFileHandler(filename=filename, when=when, backupCount=backCount,
                                               encoding='utf-8')  # 往文件里写入#指定间隔时间自动生成文件的处理器
        # 实例化TimedRotatingFileHandler
        # interval是时间间隔，backupCount是备份文件的个数，如果超过这个个数，就会自动删除，when是间隔的时间单位，单位有以下几种：
        # S 秒
        # M 分
        # H 小时、
        # D 天、
        # W 每星期（interval==0时代表星期一）
        # midnight 每天凌晨
        th.setFormatter(format_str)  # 设置文件里写入的格式
        # self.logger.addHandler(sh) #把对象加到logger里
        self.logger.addHandler(th)
        if not os.path.exists(rootPath + '/Fetch'+ '/log'):
            os.mkdir(rootPath + '/Fetch'+ '/log')
        # 初始化日志模块
        alllog = Logger(rootPath + '/Fetch'+ '/log/all.log', level='debug')
        errorlog = Logger(rootPath + '/Fetch' + '/log/error.log', level='error')
        # errorlog = Logger('../log/error.log', level='error')

def enum_devices(device = 0 , device_way = False):
    """
    device = 0  枚举网口、USB口、未知设备、cameralink 设备，默认
    device = 1 枚举GenTL设备
    """
    if device_way == False:
        if device == 0:
            tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE | MV_UNKNOW_DEVICE | MV_1394_DEVICE | MV_CAMERALINK_DEVICE
            deviceList = MV_CC_DEVICE_INFO_LIST()
            # 枚举设备
            ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
            if ret != 0:
                print("enum devices fail! ret[0x%x]" % ret)
                sys.exit()
            if deviceList.nDeviceNum == 0:
                print("find no device!")
                sys.exit()
            print("Find %d devices!" % deviceList.nDeviceNum)
            return deviceList
        else:
            pass
    elif device_way == True:
        pass


# 判断不同类型设备
def identify_different_devices(deviceList):
    # 判断不同类型设备，并输出相关信息
    for i in range(0, deviceList.nDeviceNum):
        mvcc_dev_info = cast(deviceList.pDeviceInfo[i], POINTER(MV_CC_DEVICE_INFO)).contents
        # 判断是否为网口相机
        if mvcc_dev_info.nTLayerType == MV_GIGE_DEVICE:
            print("\n网口设备序号: [%d]" % i)
            # 获取设备名
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chModelName:
                strModeName = strModeName + chr(per)
            print("当前设备型号名: %s" % strModeName)
            # 获取当前设备 IP 地址
            nip1_1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24)
            nip1_2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16)
            nip1_3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8)
            nip1_4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff)
            print("当前 ip 地址: %d.%d.%d.%d" % (nip1_1, nip1_2, nip1_3, nip1_4))
            # 获取当前子网掩码
            nip2_1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentSubNetMask & 0xff000000) >> 24)
            nip2_2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentSubNetMask & 0x00ff0000) >> 16)
            nip2_3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentSubNetMask & 0x0000ff00) >> 8)
            nip2_4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nCurrentSubNetMask & 0x000000ff)
            print("当前子网掩码 : %d.%d.%d.%d" % (nip2_1, nip2_2, nip2_3, nip2_4))
            # 获取当前网关
            nip3_1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nDefultGateWay & 0xff000000) >> 24)
            nip3_2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nDefultGateWay & 0x00ff0000) >> 16)
            nip3_3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nDefultGateWay & 0x0000ff00) >> 8)
            nip3_4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nDefultGateWay & 0x000000ff)
            print("当前网关 : %d.%d.%d.%d" % (nip3_1, nip3_2, nip3_3, nip3_4))
            # 获取网口 IP 地址
            nip4_1 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0xff000000) >> 24)
            nip4_2 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0x00ff0000) >> 16)
            nip4_3 = ((mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0x0000ff00) >> 8)
            nip4_4 = (mvcc_dev_info.SpecialInfo.stGigEInfo.nNetExport & 0x000000ff)
            print("当前连接的网口 IP 地址 : %d.%d.%d.%d" % (nip4_1, nip4_2, nip4_3, nip4_4))
            # 获取制造商名称
            strmanufacturerName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chManufacturerName:
                strmanufacturerName = strmanufacturerName + chr(per)
            print("制造商名称 : %s" % strmanufacturerName)
            # 获取设备版本
            stdeviceversion = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chDeviceVersion:
                stdeviceversion = stdeviceversion + chr(per)
            print("设备当前使用固件版本 : %s" % stdeviceversion)
            # 获取制造商的具体信息
            stManufacturerSpecificInfo = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chManufacturerSpecificInfo:
                stManufacturerSpecificInfo = stManufacturerSpecificInfo + chr(per)
            print("设备制造商的具体信息 : %s" % stManufacturerSpecificInfo)
            # 获取设备序列号
            stSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chSerialNumber:
                stSerialNumber = stSerialNumber + chr(per)
            print("设备序列号 : %s" % stSerialNumber)
            # 获取用户自定义名称
            stUserDefinedName = ""
            for per in mvcc_dev_info.SpecialInfo.stGigEInfo.chUserDefinedName:
                stUserDefinedName = stUserDefinedName + chr(per)
            print("用户自定义名称 : %s" % stUserDefinedName)

        # 判断是否为 USB 接口相机
        elif mvcc_dev_info.nTLayerType == MV_USB_DEVICE:
            print("\nU3V 设备序号e: [%d]" % i)
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chModelName:
                if per == 0:
                    break
                strModeName = strModeName + chr(per)
            print("当前设备型号名 : %s" % strModeName)
            strSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                if per == 0:
                    break
                strSerialNumber = strSerialNumber + chr(per)
            print("当前设备序列号 : %s" % strSerialNumber)
            # 获取制造商名称
            strmanufacturerName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chVendorName:
                strmanufacturerName = strmanufacturerName + chr(per)
            print("制造商名称 : %s" % strmanufacturerName)
            # 获取设备版本
            stdeviceversion = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chDeviceVersion:
                stdeviceversion = stdeviceversion + chr(per)
            print("设备当前使用固件版本 : %s" % stdeviceversion)
            # 获取设备序列号
            stSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chSerialNumber:
                stSerialNumber = stSerialNumber + chr(per)
            print("设备序列号 : %s" % stSerialNumber)
            # 获取用户自定义名称
            stUserDefinedName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chUserDefinedName:
                stUserDefinedName = stUserDefinedName + chr(per)
            print("用户自定义名称 : %s" % stUserDefinedName)
            # 获取设备 GUID
            stDeviceGUID = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chDeviceGUID:
                stDeviceGUID = stDeviceGUID + chr(per)
            print("设备GUID号 : %s" % stDeviceGUID)
            # 获取设备的家族名称
            stFamilyName = ""
            for per in mvcc_dev_info.SpecialInfo.stUsb3VInfo.chFamilyName:
                stFamilyName = stFamilyName + chr(per)
            print("设备的家族名称 : %s" % stFamilyName)
        # 判断是否为 1394-a/b 设备
        elif mvcc_dev_info.nTLayerType == MV_1394_DEVICE:
            print("\n1394-a/b device: [%d]" % i)
        # 判断是否为 cameralink 设备
        elif mvcc_dev_info.nTLayerType == MV_CAMERALINK_DEVICE:
            print("\ncameralink device: [%d]" % i)
            # 获取当前设备名
            strModeName = ""
            for per in mvcc_dev_info.SpecialInfo.stCamLInfo.chModelName:
                if per == 0:
                    break
                strModeName = strModeName + chr(per)
            print("当前设备型号名 : %s" % strModeName)
            # 获取当前设备序列号
            strSerialNumber = ""
            for per in mvcc_dev_info.SpecialInfo.stCamLInfo.chSerialNumber:
                if per == 0:
                    break
                strSerialNumber = strSerialNumber + chr(per)
            print("当前设备序列号 : %s" % strSerialNumber)
            # 获取制造商名称
            strmanufacturerName = ""
            for per in mvcc_dev_info.SpecialInfo.stCamLInfo.chVendorName:
                strmanufacturerName = strmanufacturerName + chr(per)
            print("制造商名称 : %s" % strmanufacturerName)
            # 获取设备版本
            stdeviceversion = ""
            for per in mvcc_dev_info.SpecialInfo.stCamLInfo.chDeviceVersion:
                stdeviceversion = stdeviceversion + chr(per)
            print("设备当前使用固件版本 : %s" % stdeviceversion)
# 输入需要连接的相机的序号
def input_num_camera(deviceList):
    nConnectionNum = input("please input the number of the device to connect:")
    if int(nConnectionNum) >= deviceList.nDeviceNum:
        print("intput error!")
        sys.exit()
    return nConnectionNum

# 创建相机实例并创建句柄,(设置日志路径)
def creat_camera(deviceList , nConnectionNum ,log = True , log_path = os.getcwd()):
    """
    :param deviceList:        设备列表
    :param nConnectionNum:    需要连接的设备序号，默认序号0
    :param log:               是否创建日志
    :param log_path:          日志保存路径
    :return:                  相机实例和设备列表
    """
    # 创建相机实例
    cam = MvCamera()
    if int(nConnectionNum) >= deviceList.nDeviceNum:
        print("nConnectionNum intput error!")
        sys.exit()
    # 选择设备并创建句柄
    stDeviceList = cast(deviceList.pDeviceInfo[int(nConnectionNum)], POINTER(MV_CC_DEVICE_INFO)).contents
    if log == True:
        ret = cam.MV_CC_SetSDKLogPath(log_path)
        print(log_path)
        if ret != 0:
            print("set Log path  fail! ret[0x%x]" % ret)
            sys.exit()
        # 创建句柄,生成日志
        ret = cam.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            print("create handle fail! ret[0x%x]" % ret)
            sys.exit()
    elif log == False:
        # 创建句柄,不生成日志，返回0创建句柄成功
        ret = cam.MV_CC_CreateHandleWithoutLog(stDeviceList)
        print("ret",ret)
        if ret != 0:
            print("create handle fail! ret[0x%x]" % ret)
            sys.exit()
    return cam, stDeviceList

# 打开设备
def open_device(cam):
    # ch:打开设备 | en:Open device  返回0成功
    ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
    if ret != 0:
        print("open device fail! ret[0x%x]" % ret)
        sys.exit()

# 获取各种类型节点参数
def get_Value(cam, param_type="int_value", node_name="PayloadSize"):
    """
    :param cam:            相机实例
    :param_type:           获取节点值得类型
    :param node_name:      节点名 可选 int 、float 、enum 、bool 、string 型节点
    :return:               节点值
    """
    if param_type == "int_value":
        stParam = MVCC_INTVALUE_EX()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE_EX))
        ret = cam.MV_CC_GetIntValueEx(node_name, stParam)
        if ret != 0:
            print("获取 int 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        int_value = stParam.nCurValue
        return int_value

    elif param_type == "float_value":
        stFloatValue = MVCC_FLOATVALUE()
        memset(byref(stFloatValue), 0, sizeof(MVCC_FLOATVALUE))
        ret = cam.MV_CC_GetFloatValue(node_name, stFloatValue)
        if ret != 0:
            print("获取 float 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        float_value = stFloatValue.fCurValue
        return float_value

    elif param_type == "enum_value":
        stEnumValue = MVCC_ENUMVALUE()
        memset(byref(stEnumValue), 0, sizeof(MVCC_ENUMVALUE))
        ret = cam.MV_CC_GetEnumValue(node_name, stEnumValue)
        if ret != 0:
            print("获取 enum 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        enum_value = stEnumValue.nCurValue
        return enum_value

    elif param_type == "bool_value":
        stBool = c_bool(False)
        ret = cam.MV_CC_GetBoolValue(node_name, stBool)
        if ret != 0:
            print("获取 bool 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        return stBool.value

    elif param_type == "string_value":
        stStringValue = MVCC_STRINGVALUE()
        memset(byref(stStringValue), 0, sizeof(MVCC_STRINGVALUE))
        ret = cam.MV_CC_GetStringValue(node_name, stStringValue)
        if ret != 0:
            print("获取 string 型数据 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        string_value = stStringValue.chCurValue
        return string_value


# 设置各种类型节点参数
def set_Value(cam, param_type="int_value", node_name="PayloadSize", node_value=None):
    """
    :param cam:               相机实例
    :param param_type:        需要设置的节点值得类型
        int:
        float:
        enum:     参考于客户端中该选项的 Enum Entry Value 值即可
        bool:     对应 0 为关，1 为开
        string:   输入值为数字或者英文字符，不能为汉字
    :param node_name:         需要设置的节点名
    :param node_value:        设置给节点的值
    :return:
    """
    if param_type == "int_value":
        stParam = int(node_value)
        ret = cam.MV_CC_SetIntValueEx(node_name, stParam)
        if ret != 0:
            print("设置 int 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 int 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))

    elif param_type == "float_value":
        stFloatValue = float(node_value)
        ret = cam.MV_CC_SetFloatValue(node_name, stFloatValue)
        if ret != 0:
            print("设置 float 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 float 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))

    elif param_type == "enum_value":
        stEnumValue = node_value
        ret = cam.MV_CC_SetEnumValue(node_name, stEnumValue)
        if ret != 0:
            print("设置 enum 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 enum 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))

    elif param_type == "bool_value":
        ret = cam.MV_CC_SetBoolValue(node_name, node_value)
        if ret != 0:
            print("设置 bool 型数据节点 %s 失败 ！ 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 bool 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))

    elif param_type == "string_value":
        stStringValue = str(node_value)
        ret = cam.MV_CC_SetStringValue(node_name, stStringValue)
        if ret != 0:
            print("设置 string 型数据节点 %s 失败 ! 报错码 ret[0x%x]" % (node_name, ret))
            sys.exit()
        print("设置 string 型数据节点 %s 成功 ！设置值为 %s !" % (node_name, node_value))


def access_get_image(cam, active_way="getImagebuffer"):
    """
    :param cam:     相机实例
    :active_way:主动取流方式的不同方法 分别是（getImagebuffer）（getoneframetimeout）
    :return:
    """
    if active_way == "getImagebuffer":
        # 图像结构体，输出图像地址及图像信息（图像指针地址、图像信息（stFrameInfo）、预留）
        stOutFrame = MV_FRAME_OUT()
        # stOutFrame = MV_FRAME_OUT_INFO_EX()  # 初始化变量 采用C的方式

        memset(byref(stOutFrame), 0, sizeof(stOutFrame))
        init_num = 0
        while True:
            init_num += 1
            ret = cam.MV_CC_GetImageBuffer(stOutFrame, 1000)  # 输入INFINITE时表示无限等待，直到收到一帧数据或者停止取流
            # 如果指针地址不为空且有图像返回，图片像素格式为17301505
            print("pBufAddr",stOutFrame.pBufAddr)
            print("ret",ret)
            print("enPixelType",stOutFrame.stFrameInfo.enPixelType)
            if None != stOutFrame.pBufAddr and 0 == ret and stOutFrame.stFrameInfo.enPixelType == 17301505:  #17301505为黑白图片 mono
                print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
                pData = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)()
                cdll.msvcrt.memcpy(byref(pData), stOutFrame.pBufAddr,
                                   stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)
                data = np.frombuffer(pData, count=int(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight),
                                     dtype=np.uint8)
                image_control(init_num, data=data, stFrameInfo=stOutFrame.stFrameInfo)
            elif None != stOutFrame.pBufAddr and 0 == ret and stOutFrame.stFrameInfo.enPixelType == 17301514:  # 17301514为Bater图像
                print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
                pData = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)()
                cdll.msvcrt.memcpy(byref(pData), stOutFrame.pBufAddr,
                                   stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight)
                data = np.frombuffer(pData, count=int(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight),
                                     dtype=np.uint8)
                image_control(init_num, data=data, stFrameInfo=stOutFrame.stFrameInfo)
            elif None != stOutFrame.pBufAddr and 0 == ret and stOutFrame.stFrameInfo.enPixelType == 35127316: #35127316原始图像通道是RGB的（需要在MVS中设置图像格式为RGB）
                print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
                pData = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3)()
                cdll.msvcrt.memcpy(byref(pData), stOutFrame.pBufAddr,
                                   stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3)
                data = np.frombuffer(pData,
                                     count=int(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 3),
                                     dtype=np.uint8)
                image_control(init_num, data=data, stFrameInfo=stOutFrame.stFrameInfo)  # RGB图像调用
            elif None != stOutFrame.pBufAddr and 0 == ret and stOutFrame.stFrameInfo.enPixelType == 34603039:  # 原始34603039 为YUV图像
                print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum))
                pData = (c_ubyte * stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2)()
                cdll.msvcrt.memcpy(byref(pData), stOutFrame.pBufAddr,
                                   stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2)
                data = np.frombuffer(pData,
                                     count=int(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 2),
                                     dtype=np.uint8)
                image_control(init_num, data=data, stFrameInfo=stOutFrame.stFrameInfo)
            else:
                print("no data[0x%x]" % ret)
            # 权限释放
            nRet = cam.MV_CC_FreeImageBuffer(stOutFrame)
    elif active_way == "getoneframetimeout":
        #  work_thread_rgb82bgr(cam=0, pData=0, nDataSize=0):

        # ch:获取数据包大小 | en:Get payload size
        stParam = MVCC_INTVALUE()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
        ret = cam.MV_CC_GetIntValue("PayloadSize", stParam)
        if ret != 0:
            print("get payload size fail! ret[0x%x]" % ret)
            sys.exit()
        nPayloadSize = stParam.nCurValue
        pData = (c_ubyte * nPayloadSize)()
        nDataSize = nPayloadSize
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
        init_num = 0
        while True:
            init_num += 1
            time_flag = flag_time()
            if time_flag is False:
                print("未在抓拍时间段")
                continue
            ret = cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
            if ret == 0:
                print("get one frame: Width[%d], Height[%d], PixelType[0x%x], nFrameNum[%d]" % (
                stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.enPixelType, stFrameInfo.nFrameNum))
                print('----', stFrameInfo.enPixelType)
                # if stFrameInfo.enPixelType=='PixelType_Gvsp_RGB8_Packed':
                data = np.asarray(pData)
                # temp = temp.reshape((img_h, img_w, img_c))
                # data = cv2.cvtColor(temp, cv2.COLOR_BGR2RGB)
                image_control(init_num, data=data, stFrameInfo=stFrameInfo)  # ubuntu 主动取图
                # cv2.namedWindow("temp", cv2.WINDOW_NORMAL)
                # cv2.imshow('temp', temp)
                # cv2.waitKey(1)
                # else:
                # print("图像输出格式不是BGR8,请先使用MVS软件设置相机默认输出图像格式为BGR8....")
            else:
                print("no data[0x%x]" % ret)
                errorlog.logger.error("no data[0x%x]" % ret)
                break
                # raise RuntimeError('ret')
            # # ch:停止取流 | en:Stop grab image
            # ret = cam.MV_CC_StopGrabbing()
            # if ret != 0:
            #     print("stop grabbing fail! ret[0x%x]" % ret)
            #     del pData
            #     sys.exit()
            # 权限释放
            # nRet = cam.MV_CC_FreeImageBuffer(stParam)
    cv2.destroyAllWindows()  # 释放所有窗口

# 显示图像
def image_show(image):
    # image = cv2.resize(image, (600, 400), interpolation=cv2.INTER_AREA)  # 对图片进行裁剪
    cv2.imshow('fgmask', image)
    cv2.waitKey(1)

# 需要显示的图像数据转换
def image_control(init_num, data, stFrameInfo):
    if stFrameInfo.enPixelType == 17301505:  # 17301505为黑白图片 mono
        image = data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
        image_show(image=image)
    elif stFrameInfo.enPixelType == 17301514:  # 17301514为Bater图像
        data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
        image = cv2.cvtColor(data, cv2.COLOR_BAYER_GB2RGB)
        image_show(image=image)
    elif stFrameInfo.enPixelType == 35127316:  # 35127316原始图像通道是RGB的,设置RGB8
        data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
        print("data.shape", data.shape)
        image = cv2.cvtColor(data, cv2.COLOR_RGB2BGR)
        # image_show(image=image)
        if init_num % config.frame_jump == 0:  # 满足抽帧策略，则将图片传输给算法进行解析或存储
            call_post_image(image, image)  # 将图片传输给后台进行检测
    elif stFrameInfo.enPixelType == 34603039:  # 34603039 为YUV图像
        data = data.reshape(stFrameInfo.nHeight, stFrameInfo.nWidth, -1)
        image = cv2.cvtColor(data, cv2.COLOR_YUV2BGR_Y422)
        image_show(image=image)
    return image

# 开启取流并获取数据包大小
def start_grab_and_get_data_size(cam):
    ret = cam.MV_CC_StartGrabbing()
    if ret != 0:
        print("开始取流失败! ret[0x%x]" % ret)
        sys.exit()
    else:
        print("已开启取流")
    # 获取数据包大小
    stParam = MVCC_INTVALUE_EX()
    memset(byref(stParam), 0, sizeof(MVCC_INTVALUE_EX))
    # ret = cam.MV_CC_GetIntValueEx("PayloadSize", stParam)
    # ret = cam.MV_CC_GetIntValueEx("Width", stParam)
    # if ret != 0:
    #     print("get payload size fail! ret[0x%x]" % ret)
    #     sys.exit()
    nPayloadSize = stParam.nCurValue
    return nPayloadSize

# 关闭设备与销毁句柄
def close_and_destroy_device(cam , data_buf=None):
    # 停止取流
    ret = cam.MV_CC_StopGrabbing()
    if ret != 0:
        print("stop grabbing fail! ret[0x%x]" % ret)
        sys.exit()
    # 关闭设备
    ret = cam.MV_CC_CloseDevice()
    if ret != 0:
        print("close deivce fail! ret[0x%x]" % ret)
        del data_buf
        sys.exit()
    # 销毁句柄
    ret = cam.MV_CC_DestroyHandle()
    if ret != 0:
        print("destroy handle fail! ret[0x%x]" % ret)
        del data_buf
        sys.exit()
    del data_buf

def call_post_image(image, image1):
    """
    将图片编码传输给算法
    :return:
    """
    request_url = config.request_url  # 图片接收地址
    # params = {"alarm_content": "碧波路518号"}
    timestamp = time.time()
    # 将图片转为base64
    # 将cv读取的ndarray转为base64
    img_str = cv2.imencode('.jpg', image)[1].tostring()  # 将图片编码成流数据，放到内存缓存中，然后转化成string格式
    img_b64encode = base64.b64encode(img_str)  # 编码成base64  bytes
    # 将base64转为字符串
    img_b64encode = img_b64encode.decode('utf-8')

    img_str1 = cv2.imencode('.jpg', image1)[1].tostring()  # 将图片编码成流数据，放到内存缓存中，然后转化成string格式
    img_b64encode1 = base64.b64encode(img_str1)  # 编码成base64  bytes
    # 将base64转为字符串
    img_b64encode1 = img_b64encode1.decode('utf-8')
    params = {
        "constructionId":"1333",
        "deviceId":"20220915",
        "timestamp": timestamp,
        "content": "",
        "image": img_b64encode,
        # "image1": img_b64encode1
        # "ceshi":111
    }
    # access_token = '24.60fa5da981283c32148b5ca38e73f45e.2592000.1593582788.282335-20169217'

    print("准备发送物体检测")
    headers = {'Content-Type': 'application/json', "Connection": "close"}
    # ka_body_start = time.time()
    response = requests.post(request_url, data=json.dumps(params), headers=headers, timeout=10)
    time.sleep(0.5)
    print("res_json", response.json())


def flag_time():
    time_flag = False
    # 范围时间
    start_time = datetime.datetime.strptime(str(datetime.datetime.now().date()) + '7:30', '%Y-%m-%d%H:%M')
    # 开始时间
    print(start_time)
    end_time = datetime.datetime.strptime(str(datetime.datetime.now().date()) + '19:30', '%Y-%m-%d%H:%M')
    # 结束时间
    print(end_time)
    # 当前时间
    now_time = datetime.datetime.now()
    print("now_time", now_time)
    # 方法一：
    # 判断当前时间是否在范围时间内
    if start_time < now_time < end_time:
        time_flag = True
    return time_flag


def main():
    try:
        err_flag = False
        print(os.getcwd())
        #SDK初始化
        MvCamera.MV_CC_Initialize()
        # 枚举设备
        deviceList = enum_devices(device=0, device_way=False)
        # 判断不同类型设备
        identify_different_devices(deviceList)
        # 输入需要被连接的设备
        # nConnectionNum = input_num_camera(deviceList)
        nConnectionNum = 0
        # 创建相机实例并创建句柄,(设置日志路径)
        cam, stDeviceList = creat_camera(deviceList, nConnectionNum, log=False)
        # decide_divice_on_line(cam)  ==============
        # 打开设备
        open_device(cam)
        # # 设置缓存节点个数
        # set_image_Node_num(cam, Num=10)
        # # 设置取流策略
        # set_grab_strategy(cam, grabstrategy=2, outputqueuesize=10)
        # 设置设备的一些参数
        # set_Value(cam, param_type="bool_value", node_name="TriggerCacheEnable", node_value=1)
        # 获取设备的一些参数
        # get_value = get_Value(cam , param_type = "int_value" , node_name = "PayloadSize")

        # stdcall = input("回调方式取流显示请输入 0    主动取流方式显示请输入 1:")
        # if int(stdcall) == 0:
        #     # 回调方式抓取图像
        #     call_back_get_image(cam)
        #     # 开启设备取流
        #     start_grab_and_get_data_size(cam)
        #     # 当使用 回调取流时，需要在此处添加
        #     print("press a key to stop grabbing.")
        #     msvcrt.getch()
        #     # 关闭设备与销毁句柄
        #     close_and_destroy_device(cam)
        # elif int(stdcall) == 1:
        # 开启设备取流
        start_grab_and_get_data_size(cam)
        # 主动取流方式抓取图像
        access_get_image(cam, active_way="getoneframetimeout")
        # # 关闭设备与销毁句柄
        # close_and_destroy_device(cam)
    except BaseException as base:
        # 关闭设备与销毁句柄
        close_and_destroy_device(cam)
        # 把错误记录至log文件中
        errorlog.logger.error(base)
        err_flag = False
    return err_flag

if __name__ == '__main__':
    # time_flag = flag_time()
    # print(time_flag)
    # exit()
    while True:
        try:
            main()
        except BaseException as base:
            pass
        sleep_time = config.sleep_time
        while True:
            time.sleep(1)
            minute = int(sleep_time / 60)
            secend = sleep_time - (minute * 60)
            print("%d分%d秒后重新连接摄像头" % (minute, secend))
            sleep_time -= 1
            if minute == 0 and secend == 0:
                break
    # deviceList = enum_devices()
    # identify_different_devices(deviceList)
    # input_num_camera(deviceList)
    # cam, stDeviceList = creat_camera(deviceList, nConnectionNum=0, log=False, log_path=os.getcwd())
    # open_device(cam)
    # # ret = get_Value(cam, param_type="int_value", node_name="PayloadSize")
    # # ret = set_Value(cam, param_type="int_value", node_name="PayloadSize", node_value = None)
    #
    # # print(deviceList)


