#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy                                     # ROS2 Python接口库
from rclpy.node import Node                      # ROS2 节点类
from message_interfaces.msg import PointTemprInfo
from sensor_msgs.msg import Image

from .BaseData import *
from ctypes import *
import sys
import time
import os
import threading
import queue
import numpy as np

iImageWidth = 360  # 假设 iImageWidth 为图片宽度
iImageHeight = 288  # 假设 iImageHeight 为图片高度
iPixels = 3  # 假设 iPixels 为像素数

g_vecDevices = []

g_lstIrData = queue.Queue()  # 使用队列作为 g_lstIrData
g_lstIrDataLock = threading.Lock()

XJTech = cdll.LoadLibrary("libxjtech.so")


# 回调函数的类型定义
SearchDeviceCallbackFunc = CFUNCTYPE(
    None, POINTER(X_DEVICE_INFO), c_void_p)

# Python的替代回调函数


def search_device_callback_func(pDeviceInfo, pUserData):
    pDevInfo = X_DEVICE_INFO()
    memmove(byref(pDevInfo), pDeviceInfo,
            sizeof(X_DEVICE_INFO))

    print("search a device:", pDevInfo.szIpAddr.decode(
        "utf-8"))  # 使用decode方法将字节数组转换为字符串

    # 假设g_vecDevices是一个全局变量用于存储设备信息
    g_vecDevices.append(pDevInfo)

    # 不需要返回任何内容，因此不需要使用return语句


# 将Python回调函数转换为ctypes回调函数
search_device_callback_func_ptr = SearchDeviceCallbackFunc(
    search_device_callback_func)

# 使用ctypes回调函数的例子（假设这是调用C++代码的地方）
# ctypes_callback_func = SearchDeviceCallbackFunc(search_device_callback_func)
# 使用ctypes_callback_func作为回调函数传递给C++代码进行设备搜索


# 回调函数类型定义
UploadDataCallbackFunc = CFUNCTYPE(None, POINTER(c_ubyte), POINTER(X_TEMPR_INFO), POINTER(X_TEMPR_INFO), POINTER(X_ALARM_LINDED_AREA),
                                   c_int, POINTER(c_ubyte), c_void_p)

# Python 的替代回调函数


def upload_data_callback_func(pImage, pMaxTemprInfo, pMinTemprInfo, pAlarmLinkedArea, iAlarmAreaCount, pAlarmMask, pUserData):

    pData = IrData(iImageWidth, iImageHeight, iPixels)

    # # 创建一个 ctypes 数组对象，使用指针和数组大小
    # c_array = (c_ubyte * (iImageWidth * iImageHeight * iPixels)
    #            ).from_address(addressof(pImage.contents))
    # pData.pImage = list(c_array)

    # 将pImage的数据复制到Python的bytes对象中
    # data = string_at(pImage, iImageWidth * iImageHeight * iPixels)
    pData.pImage = np.frombuffer(
        string_at(pImage, iImageWidth * iImageHeight * iPixels), dtype=np.uint8)

    # 将 X_TEMPR_INFO 对象的地址赋给 pMaxTemprInfo 指针
    pData.pMaxTemprInfo.iX = pMaxTemprInfo.contents.iX
    pData.pMaxTemprInfo.iY = pMaxTemprInfo.contents.iY
    pData.pMaxTemprInfo.iTempr = pMaxTemprInfo.contents.iTempr

    g_lstIrDataLock.acquire()
    g_lstIrData.put(pData)
    g_lstIrDataLock.release()


# 将 Python 回调函数转换为 ctypes 回调函数
upload_data_callback_func_ptr = UploadDataCallbackFunc(
    upload_data_callback_func)

# 使用 ctypes 回调函数的例子（假设这是调用 C++ 代码的地方）
# ctypes_callback_func = UploadDataCallbackFunc(upload_data_callback_func)
# 使用 ctypes_callback_func 作为回调函数传递给 C++ 代码进行数据上传


class XJTechNode(Node):
    def __init__(self, name):
        super().__init__(name)                       # ROS2节点父类初始化

        # 创建机芯参数
        self.iUserId = 0
        # 创建值为 E_TCP_HISI 的 X_DEVICE_TYPE 实例
        self.eDeviceType = X_DEVICE_TYPE(2)
        # 分配内存给设备信息结构体 pCurDevInfo
        self.pCurDevInfo = X_DEVICE_INFO()
        # 创建 X_DEVICE_PARAM 实例并设置字段的值
        self.pDeviceParam = X_DEVICE_PARAM()

        # 创建 PlatHistDimmerParam 实例并设置字段的值
        self.pPlatHistDimmer = PlatHistDimmerParam()

        # 创建图片结构体
        self.imageSize = ImageSize()
        # 是否显示图像
        self.showImage = False
        # 测温模式
        self.TemperatureMeasurementMode = 0
        # 标准温度
        self.StandardTemperature = 0
        # 测温点
        self.PointX = 0
        self.PointY = 0
        self.LeftTopPointX = 0
        self.LeftTopPointY = 0
        self.RightBottomPointX = 0
        self.RightBottomPointY = 0

        # 加载参数
        self.defined_parameters()
        self.update_parameters()
        self.print_parameters()

        # 搜索设备并打开
        self.search_and_open_devices()
        # 设置设备参数
        self.set_devices_parameters()

        # 创建一个定时器（单位为秒的周期，定时执行的回调函数）
        self.timer = self.create_timer(0.02, self.timer_callback)
        # 定义发布测温点的话题
        self.temprPointPublisher = self.create_publisher(
            PointTemprInfo, "point_tempr", 10)
        # 定义发布温度图像话题
        self.imagePublisher = self.create_publisher(Image, "tempr_image", 10)

    def defined_parameters(self):
        # 创建ros参数，并设置参数的默认值
        self.declare_parameter("Pixels", 3)
        self.declare_parameter("DeviceType", 2)
        self.declare_parameter("FrameRate", 25)
        self.declare_parameter("SrcWidth", 384)
        self.declare_parameter("SrcHeight", 288)
        self.declare_parameter("DstWidth", 360)
        self.declare_parameter("DstHeight", 288)
        self.declare_parameter("CutHorOffset", 0)
        self.declare_parameter("CutVerOffset", 0)
        self.declare_parameter("Cut", 1)
        self.declare_parameter("AlarmType", 0)
        self.declare_parameter("FlipType", 0)
        self.declare_parameter("PlatThresholdValue", 30)
        self.declare_parameter("MappingMidValue", 145)
        self.declare_parameter("LowerDiscardRatio", 0.01)
        self.declare_parameter("UpperDiscardRatio", 0.01)
        self.declare_parameter("DynamicRangeCoef", 10)
        self.declare_parameter("MappingRange", 220)
        self.declare_parameter("ImageShow", False)
        self.declare_parameter("Mode", 0)
        self.declare_parameter("StandardTemperature", 50.0)
        self.declare_parameter("Point.x", 0)
        self.declare_parameter("Point.y", 0)
        self.declare_parameter("Region.leftTopPoint.x", 0)
        self.declare_parameter("Region.leftTopPoint.y", 0)
        self.declare_parameter("Region.rightBottomPoint.x", 0)
        self.declare_parameter("Region.rightBottomPoint.y", 0)

    def update_parameters(self):
        # # 尝试从参数服务器读取参数
        # # 如果参数不存在，则使用默认值 "default_value"
        self.imageSize.iPixels = self.get_parameter(
            "Pixels").get_parameter_value().integer_value
        self.imageSize.iImageWidth = self.get_parameter(
            "DstWidth").get_parameter_value().integer_value
        self.imageSize.iImageHeight = self.get_parameter(
            "DstHeight").get_parameter_value().integer_value
        self.showImage = self.get_parameter(
            "ImageShow").get_parameter_value().bool_value
        self.TemperatureMeasurementMode = self.get_parameter(
            "Mode").get_parameter_value().integer_value
        self.StandardTemperature = self.get_parameter(
            "StandardTemperature").get_parameter_value().double_value
        self.PointX = self.get_parameter(
            "Point.x").get_parameter_value().integer_value
        self.PointY = self.get_parameter(
            "Point.y").get_parameter_value().integer_value
        self.LeftTopPointX = self.get_parameter(
            "Region.leftTopPoint.x").get_parameter_value().integer_value
        self.LeftTopPointY = self.get_parameter(
            "Region.leftTopPoint.y").get_parameter_value().integer_value
        self.RightBottomPointX = self.get_parameter(
            "Region.rightBottomPoint.x").get_parameter_value().integer_value
        self.RightBottomPointY = self.get_parameter(
            "Region.rightBottomPoint.y").get_parameter_value().integer_value
        # E_TCP_HISI # TCP_HISI设备（红外热像仪）
        self.eDeviceType = X_DEVICE_TYPE(self.get_parameter(
            "DeviceType").get_parameter_value().integer_value)
        self.pDeviceParam.iFrameRate = self.get_parameter(
            "FrameRate").get_parameter_value().integer_value	   # 图像帧频
        self.pDeviceParam.iSrcWidth = self.get_parameter(
            "SrcWidth").get_parameter_value().integer_value	   # 原始图像/温度宽度
        self.pDeviceParam.iSrcHeight = self.get_parameter(
            "SrcHeight").get_parameter_value().integer_value	   # 原始图像/温度高度
        self.pDeviceParam.iDstWidth = self.get_parameter(
            "DstWidth").get_parameter_value().integer_value	   # 目标图像/温度宽度
        self.pDeviceParam.iDstHeight = self.get_parameter(
            "DstHeight").get_parameter_value().integer_value	   # 目标图像/温度高度
        self.pDeviceParam.iCutHorOffset = self.get_parameter(
            "CutHorOffset").get_parameter_value().integer_value  # 数据裁剪水平偏移量
        self.pDeviceParam.iCutVerOffset = self.get_parameter(
            "CutVerOffset").get_parameter_value().integer_value  # 数据裁剪垂直偏移量
        self.pDeviceParam.bCut = self.get_parameter(
            "Cut").get_parameter_value().integer_value				   # 数据裁剪标志
        # 报警类型
        self.pDeviceParam.eAlarmType = X_ALARM_TYPE(
            self.get_parameter("AlarmType").get_parameter_value().integer_value)
        # 数据翻转类型
        self.pDeviceParam.eFlipType = X_FLIP_TYPE(
            self.get_parameter("FlipType").get_parameter_value().integer_value)

        # 平台阈值（范围1 ~ 200，默认值100）
        self.pPlatHistDimmer.iPlatThresholdValue = self.get_parameter(
            "PlatThresholdValue").get_parameter_value().integer_value
        # 映射中间值，调整亮度（范围0 ~ 255，默认值128）
        self.pPlatHistDimmer.iMappingMidValue = self.get_parameter(
            "MappingMidValue").get_parameter_value().integer_value
        self.pPlatHistDimmer.dLowerDiscardRatio = self.get_parameter(
            "LowerDiscardRatio").get_parameter_value().double_value  # 下抛点率（默认值1%）
        self.pPlatHistDimmer.dUpperDiscardRatio = self.get_parameter(
            "UpperDiscardRatio").get_parameter_value().double_value  # 上抛点率（默认值1%）
        self.pPlatHistDimmer.iDynamicRangeCoef = self.get_parameter(
            "DynamicRangeCoef").get_parameter_value().integer_value	   # 动态范围系数
        self.pPlatHistDimmer.iMappingRange = self.get_parameter(
            "MappingRange").get_parameter_value().integer_value			   # 映射范围

    def print_parameters(self):
        # // 打印读取到的参数值
        self.get_logger().info("Pixels: %d" % self.imageSize.iPixels)
        self.get_logger().info("DstWidth: %d" % self.imageSize.iImageWidth)
        self.get_logger().info("DstHeight: %d" % self.imageSize.iImageHeight)
        self.get_logger().info("ImageShow: %d" % self.showImage)
        self.get_logger().info("DeviceType: %d" % self.eDeviceType.value)
        self.get_logger().info("FrameRate: %d" % self.pDeviceParam.iFrameRate)
        self.get_logger().info("SrcWidth: %d" % self.pDeviceParam.iSrcWidth)
        self.get_logger().info("SrcHeight: %d" % self.pDeviceParam.iSrcHeight)
        self.get_logger().info("DstWidth: %d" % self.pDeviceParam.iDstWidth)
        self.get_logger().info("DstHeight: %d" % self.pDeviceParam.iDstHeight)
        self.get_logger().info("CutHorOffset: %d" % self.pDeviceParam.iCutHorOffset)
        self.get_logger().info("CutVerOffset: %d" % self.pDeviceParam.iCutVerOffset)
        self.get_logger().info("Cut: %d" % self.pDeviceParam.bCut)
        self.get_logger().info("AlarmType: %d" % self.pDeviceParam.eAlarmType.value)
        self.get_logger().info("FlipType: %d" % self.pDeviceParam.eFlipType.value)
        self.get_logger().info("PlatThresholdValue: %d" %
                               self.pPlatHistDimmer.iPlatThresholdValue)
        self.get_logger().info("MappingMidValue: %d" %
                               self.pPlatHistDimmer.iMappingMidValue)
        self.get_logger().info("LowerDiscardRatio: %f" %
                               self.pPlatHistDimmer.dLowerDiscardRatio)
        self.get_logger().info("UpperDiscardRatio: %f" %
                               self.pPlatHistDimmer.dUpperDiscardRatio)
        self.get_logger().info("DynamicRangeCoef: %d" %
                               self.pPlatHistDimmer.iDynamicRangeCoef)
        self.get_logger().info("MappingRange: %d" % self.pPlatHistDimmer.iMappingRange)
        self.get_logger().info("Mode: %d" % self.TemperatureMeasurementMode)
        self.get_logger().info("StandardTemperature: %f" % self.StandardTemperature)
        self.get_logger().info("Point X: %d" % self.PointX)
        self.get_logger().info("Point Y: %d" % self.PointY)
        self.get_logger().info("Left Top Point X: %d" % self.LeftTopPointX)
        self.get_logger().info("Left Top Point Y: %d" % self.LeftTopPointY)
        self.get_logger().info("Right Bottom Point X: %d" % self.RightBottomPointX)
        self.get_logger().info("Right Bottom Point Y: %d" % self.RightBottomPointY)

    def timer_callback(self):                                      # 创建定时器周期执行的回调函数
        if not g_lstIrData.empty():
            g_lstIrDataLock.acquire()
            mData = g_lstIrData.get()  # 从队列中取出数据
            g_lstIrDataLock.release()

            # self.get_logger().info("max temperature info : x = {} y = {} temperature : {}".format(
            #     mData.pMaxTemprInfo.iX, mData.pMaxTemprInfo.iY, mData.pMaxTemprInfo.iTempr / 10.0))

            # # 测试用，发布全局最高温度
            # self.publish_point_tempr(
            #     mData.pMaxTemprInfo.iX, mData.pMaxTemprInfo.iY, mData.pMaxTemprInfo.iTempr / 10.0)

            # 发布测温图像
            image_msg = Image()
            image_msg.header.frame_id = "grb_image"
            image_msg.header.stamp = self.get_clock().now().to_msg()
            image_msg.height = mData.iImageHeight
            image_msg.width = mData.iImageWidth
            image_msg.encoding = "rgb8"
            image_msg.is_bigendian = False
            image_msg.step = mData.iImageWidth * mData.iPixels
            image_msg.data = np.array(mData.pImage).tobytes()
            self.imagePublisher.publish(image_msg)

            # 发布测温点温度
            if (self.TemperatureMeasurementMode == 0):
                # 全局最高温度
                self.publish_point_tempr(
                    mData.pMaxTemprInfo.iX, mData.pMaxTemprInfo.iY, mData.pMaxTemprInfo.iTempr / 10.0)

            elif (self.TemperatureMeasurementMode == 1):
                pRect = X_RECT()
                pRect.iLeft = self.LeftTopPointX
                pRect.iTop = self.LeftTopPointY
                pRect.iRight = self.RightBottomPointX
                pRect.iBottom = self.RightBottomPointY
                pTemprInfo = X_TEMPR_INFO()
                # pTemprInfo = pointer(sTemprInfo)
                XJTech.XJTech_GetRectangleTemptr(
                    self.iUserId, byref(pRect), byref(pTemprInfo))
                self.publish_point_tempr(
                    pTemprInfo.iX, pTemprInfo.iY, pTemprInfo.iTempr / 10.0)
            elif (self.TemperatureMeasurementMode == 2):
                fpoint_tempr = c_float()
                fpoint_tempr = XJTech.XJTech_GetTemprByPoint(
                    self.iUserId, self.PointX, self.PointY)
                self.get_logger().info("point_tempr = %f " % fpoint_tempr)
                self.publish_point_tempr(
                    self.PointX, self.PointY, fpoint_tempr*10.0)

    def search_and_open_devices(self):

        g_vecDevices.clear()
        print("searching device...")

        # 调用回调函数的示例
        # 假设 XJTech_StartSearch 是一个 C++ 函数，用于搜索设备，其参数为设备类型、回调函数和用户数据
        # 注意：在调用这个函数时，确保 SearchDeviceCallbackFunc 和 g_vecDevices 变量已经在代码中正确定义和初始化
        XJTech.XJTech_StartSearch(
            self.eDeviceType, search_device_callback_func_ptr, None)
        # 打印搜索到的设备数量
        print("searched device : ", len(g_vecDevices))
        # 如果搜索设备失败
        # 分配内存给 pCurDevInfo
        pCurDevInfo = X_DEVICE_INFO()
        if len(g_vecDevices) <= 0:
            print("search device failed.")
            # 返回-1
            exit(-1)
        else:
            print("search device success.")
            # 复制设备信息到 pCurDevInfo
            pCurDevInfo = g_vecDevices[0]
            # memmove(byref(pCurDevInfo), byref(
            #     g_vecDevices[0]), sizeof(X_DEVICE_INFO))

        # 初始化设备
        self.iUserId = XJTech.XJTech_Init(
            self.eDeviceType, byref(self.pDeviceParam))

        if (self.iUserId < 0):
            print("XJTech_Init failed.")
            # 返回-1
            exit(-1)
        else:
            print("XJTech_Init success.")

        # 打开设备
        iRet = XJTech.XJTech_Open(self.iUserId, pointer(
            pCurDevInfo), upload_data_callback_func_ptr, None)
        print("XJTech_Open iRet : ", iRet)
        if (iRet < 0):
            print("XJTech_Open failed.")
            exit(-1)
        else:
            print("XJTech_Open success.")

    def set_devices_parameters(self):
        #  set parameters
        XJTech.XJTech_SetPlatDimmerParam(
            self.iUserId, byref(self.pPlatHistDimmer))
        XJTech.XJTech_GammaCorrection(self.iUserId, 18)
        XJTech.XJTech_ChangePalette(self.iUserId, 1)
        print("set parameters success.")

    def publish_point_tempr(self, x: int,  y: int,  tempr: float):
        point_tempr = PointTemprInfo()
        point_tempr.x = x
        point_tempr.y = y
        point_tempr.tempr = tempr
        point_tempr.header.frame_id = "xjtech"
        point_tempr.header.stamp = self.get_clock().now().to_msg()  # 获取当前时间戳
        self.temprPointPublisher.publish(point_tempr)


def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = XJTechNode("node_xjtech_class")   # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口
