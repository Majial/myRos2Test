# -*- coding: UTF-8 -*-

from ctypes import *
import sys
import time
import os
import threading
import queue

# a = os.system('export LD_LIBRARY_PATH=./:$LD_LIBRARY_PATH')
XJTech = cdll.LoadLibrary("./libxjtech.so")

# struct
# class POINT(Structure):
#     _fields_ = ("x", c_int), ("y", c_int)
# class MyStruct(Structure):
#     _fields_ = [("a", c_int),
#                  ("b", c_float),
#                  ("point_array", POINT * 4)]

# 定义一个Python类来映射C中的枚举类型
# class Color(c_int):
#     _enum_map = {
#         0: 'RED',
#         1: 'GREEN',
#         2: 'BLUE'
#     }

#     def __repr__(self):
#         return self._enum_map[self.value]

# Color._value_map_ = {v: k for k, v in Color._enum_map.items()}

# 回调函数
# @CFUNCTYPE(c_int, POINTER(c_int), POINTER(c_int))
# def py_cmp_func(a, b):
#     print("py_cmp_func", a[0], b[0])
#     return a[0] - b[0]

# qsort(ia, len(ia), sizeof(c_int), py_cmp_func)


class X_TEMPR_DATA_TYPE(c_int):
    _enum_map = {
        0: 'E_TYPE_NONE',  # 类型非法
        1: 'E_TYPE_SHORT',  # short类型
        2: 'E_TYPE_UNSIGNED_SHORT',  # uint16_t 类型
        3: 'E_TYPE_MAX'
    }

    def __repr__(self):
        return self._enum_map[self.value]


X_TEMPR_DATA_TYPE._value_map_ = {v: k for k,
                                 v in X_TEMPR_DATA_TYPE._enum_map.items()}


class X_FLIP_TYPE(c_int):
    _enum_map = {
        0: 'E_FLIP_NONE',  # 不翻转
        1: 'E_FLIP_R90',  # 顺时针翻转90度
        2: 'E_FLIP_L90',  # 逆时针翻转90度
        3: 'E_FLIP_180',  # 翻转180度
        4: 'E_FLIP_MAX'  # 翻转最大值（哨兵）
    }

    def __repr__(self):
        return self._enum_map[self.value]


X_FLIP_TYPE._value_map_ = {v: k for k, v in X_FLIP_TYPE._enum_map.items()}


class X_ALARM_TYPE(c_int):
    _enum_map = {
        0: 'E_HIGH_ALARM',  # 高温报警
        1: 'E_LOW_ALARM',  # 低温报警
        2: 'E_BOTH_ALARM'  # 高低温报警
    }

    def __repr__(self):
        return self._enum_map[self.value]


X_ALARM_TYPE._value_map_ = {v: k for k, v in X_ALARM_TYPE._enum_map.items()}


class X_DEVICE_TYPE(c_int):
    _enum_map = {
        0: 'E_NONE',  # 无
        1: 'E_UDP',  # UDP设备
        2: 'E_TCP_HISI',  # TCP_HISI设备（迅检红外热像仪）
        3: 'E_RTSP',  # RTSP设备
        4: 'E_TCP_SAM',  # TCP_SAM设备
        5: 'CMD_TYPE_GAIN_ADJUST',  # UVC设备
    }

    def __repr__(self):
        return self._enum_map[self.value]


X_DEVICE_TYPE._value_map_ = {v: k for k, v in X_DEVICE_TYPE._enum_map.items()}


class X_DEVICE_PARAM(Structure):
    _fields_ = [
        ("iFrameRate", c_int),  # 图像帧频
        ("iSrcWidth", c_int),  # 原始图像/温度宽度
        ("iSrcHeight", c_int),  # 原始图像/温度高度
        ("iDstWidth", c_int),  # 目标图像/温度宽度
        ("iDstHeight", c_int),  # 目标图像/温度高度
        ("iCutHorOffset", c_int),  # 数据裁剪水平偏移量
        ("iCutVerOffset", c_int),  # 数据裁剪垂直偏移量
        ("bCut", c_int),  # 数据裁剪标志
        ("eAlarmType", X_ALARM_TYPE),  # 报警类型
        ("eFlipType", X_FLIP_TYPE),  # 数据翻转类型
    ]

    # def __init__(self, iFrameRate=0, iSrcWidth=0, iSrcHeight=0, iDstWidth=0, iDstHeight=0, iCutHorOffset=0, iCutVerOffset=0, bCut=0, eAlarmType=None, eFlipType=None):
    #     self.iFrameRate = iFrameRate
    #     self.iSrcWidth = iSrcWidth
    #     self.iSrcHeight = iSrcHeight
    #     self.iDstWidth = iDstWidth
    #     self.iDstHeight = iDstHeight
    #     self.iCutHorOffset = iCutHorOffset
    #     self.iCutVerOffset = iCutVerOffset
    #     self.bCut = bCut
    #     self.eAlarmType = eAlarmType
    #     self.eFlipType = eFlipType


class X_DEVICE_INFO(Structure):
    _fields_ = [
        ("szMac", c_char * 20),   # 设备Mac地址
        ("szIpAddr", c_char * 16),  # 设备IP地址
        ("szSubNet", c_char * 16),  # 设备子网掩码
        ("szGateWay", c_char * 16),  # 设备网关
        ("iPort", c_int)   # 端口号
    ]

    # def __init__(self, szMac=b'0', szIpAddr=b'0', szSubNet=b'0', szGateWay=b'0', iPort=8080):
    #     self.szMac = szMac
    #     self.szIpAddr = szIpAddr
    #     self.szSubNet = szSubNet
    #     self.szGateWay = szGateWay
    #     self.iPort = iPort
# # 使用默认值创建一个X_DEVICE_PARAM实例
# default_device_param = X_DEVICE_PARAM()
# print(default_device_param.szMac)    # 输出: b'default_mac'
# print(default_device_param.szIpAddr)  # 输出: b'default_ip'
# print(default_device_param.szSubNet)  # 输出: b'default_subnet'
# print(default_device_param.szGateWay) # 输出: b'default_gateway'
# print(default_device_param.iPort)    # 输出: 8080


class PlatHistDimmerParam(Structure):
    _fields_ = [
        ("iPlatThresholdValue", c_int),  # 平台阈值（范围1 ~ 200，默认值100）
        ("iMappingMidValue", c_int),  # 映射中间值，调整亮度（范围0 ~ 255，默认值128）
        ("dLowerDiscardRatio", c_double),  # 下抛点率（默认值1%）
        ("dUpperDiscardRatio", c_double),  # 上抛点率（默认值1%）
        ("iDynamicRangeCoef", c_int),  # 动态范围系数
        ("iMappingRange", c_int),  # 映射范围
    ]

    # def __init__(self, iPlatThresholdValue=100, iMappingMidValue=128, dLowerDiscardRatio=0.01, dUpperDiscardRatio=0.01, iDynamicRangeCoef=10, iMappingRange=300):
    #     self.iPlatThresholdValue = iPlatThresholdValue
    #     self.iMappingMidValue = iMappingMidValue
    #     self.dLowerDiscardRatio = dLowerDiscardRatio
    #     self.dUpperDiscardRatio = dUpperDiscardRatio
    #     self.iDynamicRangeCoef = iDynamicRangeCoef
    #     self.iMappingRange = iMappingRange

# 定义 X_TEMPR_INFO 类


class X_TEMPR_INFO(Structure):
    _fields_ = [
        ("iX", c_int),
        ("iY", c_int),
        ("iTempr", c_int)
    ]

    # def __init__(self, iX=0, iY=0, iTempr=0):
    #     super(X_TEMPR_INFO, self).__init__()
    #     self.iX = iX
    #     self.iY = iY
    #     self.iTempr = iTempr

# 定义 X_PT 类


class X_PT(Structure):
    _fields_ = [
        ("iX", c_int),
        ("iY", c_int)
    ]

    def __init__(self, iX=0, iY=0):
        super(X_PT, self).__init__()
        self.iX = iX
        self.iY = iY

    def __add__(self, other):
        pt = X_PT()
        pt.iX = self.iX + other.iX
        pt.iY = self.iY + other.iY
        return pt

    def __eq__(self, other):
        return self.iX == other.iX and self.iY == other.iY

# 定义 X_ALARM_LINDED_AREA 类


class X_ALARM_LINDED_AREA(Structure):
    _fields_ = [
        ("iDstTempr", c_int),
        ("iAvgTemptr", c_int),
        ("shAvgAd", c_short),
        ("pDstTemprPt", X_PT),
        ("pCentroidPt", X_PT),
        ("iAreaPixelCount", c_int)
    ]

    def __init__(self):
        super(X_ALARM_LINDED_AREA, self).__init__()
        self.iDstTempr = 0
        self.iAvgTemptr = 0
        self.shAvgAd = 0
        self.pDstTemprPt = X_PT()
        self.pCentroidPt = X_PT()
        self.iAreaPixelCount = 0


# 回调函数
# @CFUNCTYPE(c_int, POINTER(c_int), POINTER(c_int))
# def py_cmp_func(a, b):
#     print("py_cmp_func", a[0], b[0])
#     return a[0] - b[0]

# qsort(ia, len(ia), sizeof(c_int), py_cmp_func)
g_vecDevices = []

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


iImageWidth = 384  # 假设 iImageWidth 为图片宽度
iImageHeight = 288  # 假设 iImageHeight 为图片高度
iPixels = 3  # 假设 iPixels 为像素数

# g_lstIrData = []

g_lstIrData = queue.Queue()  # 使用队列作为 g_lstIrData
g_lstIrDataLock = threading.Lock()

# 定义 IrData 类


class IrData:
    def __init__(self):
        self.iImageWidth = 384  # 替换成实际值
        self.iImageHeight = 288  # 替换成实际值
        self.iPixels = 3  # 替换成实际值
        self.pImage = [None] * self.iImageWidth * \
            self.iImageHeight * self.iPixels
        # self.pImage = (c_ubyte * (self.iImageWidth *
        #                self.iImageHeight * self.iPixels))()
        self.pMaxTemprInfo = X_TEMPR_INFO()
        self.pMinTemprInfo = X_TEMPR_INFO()

    def __deepcopy__(self, memodict={}):
        new_obj = IrData()
        new_obj.pImage = self.pImage[:]
        new_obj.pMaxTemprInfo = X_TEMPR_INFO()
        new_obj.pMinTemprInfo = X_TEMPR_INFO()
        return new_obj

    def __del__(self):
        del self.pImage
        self.pMaxTemprInfo = None
        self.pMinTemprInfo = None


# 回调函数类型定义
UploadDataCallbackFunc = CFUNCTYPE(None, POINTER(c_ubyte), POINTER(X_TEMPR_INFO), POINTER(X_TEMPR_INFO), POINTER(X_ALARM_LINDED_AREA),
                                   c_int, POINTER(c_ubyte), c_void_p)

# Python 的替代回调函数


def upload_data_callback_func(pImage, pMaxTemprInfo, pMinTemprInfo, pAlarmLinkedArea, iAlarmAreaCount, pAlarmMask, pUserData):

    pData = IrData()

    # 创建一个 ctypes 数组对象，使用指针和数组大小
    c_array = (c_ubyte * (iImageWidth * iImageHeight * iPixels)
               ).from_address(addressof(pImage.contents))
    pData.pImage = list(c_array)

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



def ImageProcessThreadFunc():
    print("start  imageProcessThreadFunc")
    while True:
        if not g_lstIrData.empty():
            g_lstIrDataLock.acquire()
            mData = g_lstIrData.get()  # 从队列中取出数据
            g_lstIrDataLock.release()

            print("max temperature info : x = {} y = {} temperature : {}".format(
                mData.pMaxTemprInfo.iX, mData.pMaxTemprInfo.iY, mData.pMaxTemprInfo.iTempr / 10.0))


if __name__ == '__main__':
    print("xj tech library test app.")
    # 创建值为 E_TCP_HISI 的 X_DEVICE_TYPE 实例
    eDeviceType = X_DEVICE_TYPE(2)
    print(eDeviceType)  # 输出：E_TCP_HISI

    # 创建 X_DEVICE_PARAM 实例并设置字段的值
    pDeviceParam = X_DEVICE_PARAM()
    pDeviceParam.iFrameRate = 25
    pDeviceParam.iSrcWidth = 384
    pDeviceParam.iSrcHeight = 288
    pDeviceParam.iDstWidth = iImageWidth  # 假设 iImageWidth 已定义
    pDeviceParam.iDstHeight = iImageHeight  # 假设 iImageHeight 已定义
    pDeviceParam.iCutHorOffset = 0
    pDeviceParam.iCutVerOffset = 0
    pDeviceParam.bCut = 1
    pDeviceParam.eAlarmType = X_ALARM_TYPE(0)  # 假设 E_HIGH_ALARM 已定义
    pDeviceParam.eFlipType = X_FLIP_TYPE(0)  # 假设 E_FLIP_NONE 已定义
    print(pDeviceParam.eFlipType)

    g_vecDevices.clear()
    print("searching device...")

    # 调用回调函数的示例
    # 假设 XJTech_StartSearch 是一个 C++ 函数，用于搜索设备，其参数为设备类型、回调函数和用户数据
    # 注意：在调用这个函数时，确保 SearchDeviceCallbackFunc 和 g_vecDevices 变量已经在代码中正确定义和初始化
    XJTech.XJTech_StartSearch(
        eDeviceType, search_device_callback_func_ptr, None)
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
    iUserId = XJTech.XJTech_Init(eDeviceType, byref(pDeviceParam))

    if (iUserId < 0):
        print("XJTech_Init failed.")
        # 返回-1
        exit(-1)
    else:
        print("XJTech_Init success.")

    # 打开设备
    iRet = XJTech.XJTech_Open(iUserId, pointer(
        pCurDevInfo), upload_data_callback_func_ptr, None)
    print("XJTech_Open iRet : ", iRet)
    if (iRet < 0):
        print("XJTech_Open failed.")
        exit(-1)
    else:
        print("XJTech_Open success.")

    pPlatHistDimmer = PlatHistDimmerParam()
    pPlatHistDimmer.iPlatThresholdValue = 30
    pPlatHistDimmer.iMappingMidValue = 145
    pPlatHistDimmer.dLowerDiscardRatio = 0.01
    pPlatHistDimmer.dUpperDiscardRatio = 0.01
    pPlatHistDimmer.iDynamicRangeCoef = 10
    pPlatHistDimmer.iMappingRange = 220

    #  set parameters
    XJTech.XJTech_SetPlatDimmerParam(iUserId, byref(pPlatHistDimmer))
    XJTech.XJTech_GammaCorrection(iUserId, 18)
    XJTech.XJTech_ChangePalette(iUserId, 1)
    print("set parameters success.")
    # 创建并启动线程
    image_process_thread = threading.Thread(target=ImageProcessThreadFunc)
    image_process_thread.start()

    time.sleep(2)

    image_process_thread.join()
