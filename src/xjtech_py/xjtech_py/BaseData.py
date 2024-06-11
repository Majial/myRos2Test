# -*- coding: UTF-8 -*-

from ctypes import *


class X_TEMPR_DATA_TYPE(c_int):
    _enum_map = {
        0: 'E_TYPE_NONE',
        1: 'E_TYPE_SHORT',
        2: 'E_TYPE_UNSIGNED_SHORT',
        3: 'E_TYPE_MAX'
    }

    def __repr__(self):
        return self._enum_map[self.value]


X_TEMPR_DATA_TYPE._value_map_ = {v: k for k,
                                 v in X_TEMPR_DATA_TYPE._enum_map.items()}


class X_FLIP_TYPE(c_int):
    _enum_map = {
        0: 'E_FLIP_NONE',
        1: 'E_FLIP_R90',
        2: 'E_FLIP_L90',
        3: 'E_FLIP_180',
        4: 'E_FLIP_MAX'
    }

    def __repr__(self):
        return self._enum_map[self.value]


X_FLIP_TYPE._value_map_ = {v: k for k, v in X_FLIP_TYPE._enum_map.items()}


class X_ALARM_TYPE(c_int):
    _enum_map = {
        0: 'E_HIGH_ALARM',
        1: 'E_LOW_ALARM',
        2: 'E_BOTH_ALARM'
    }

    def __repr__(self):
        return self._enum_map[self.value]


X_ALARM_TYPE._value_map_ = {v: k for k, v in X_ALARM_TYPE._enum_map.items()}


class X_MEASURE_MODE(c_int):
    _enum_map = {
        0: 'E_SURFACE',  # 体表
        1: 'E_ARMPIT'  # 腋下
    }

    def __repr__(self):
        return self._enum_map[self.value]


X_MEASURE_MODE._value_map_ = {v: k for k,
                              v in X_MEASURE_MODE._enum_map.items()}


class X_GAIN_MODE(c_int):
    _enum_map = {
        0: 'E_MANUAL',  # 手动
        1: 'E_SEMI_AUTO',  # 半自动
        2: 'E_AUTO'  # 自动
    }

    def __repr__(self):
        return self._enum_map[self.value]


X_GAIN_MODE._value_map_ = {v: k for k, v in X_GAIN_MODE._enum_map.items()}


class X_SERIAL_CMD_TYPE(c_int):
    _enum_map = {
        0: 'CMD_TYPE_NONE',  # 无
        1: 'CMD_TYPE_SHUTTER',  # 快门补偿
        2: 'CMD_TYPE_SET_SHUTTER_INTERVAL',  # 设置快门间隔时间
        3: 'CMD_TYPE_SAVE_SHUTTER_INTERAVL',  # 保存快门间隔时间配置
        4: 'CMD_TYPE_PALETTE',  # 设置色带
        5: 'CMD_TYPE_GAIN_ADJUST',  # 增益调节
        6: 'CMD_TYPE_USER_DEFINE',  # 自定义
        7: 'CMD_TYPE_MAX',  # 最大值
    }

    def __repr__(self):
        return self._enum_map[self.value]


X_SERIAL_CMD_TYPE._value_map_ = {v: k for k,
                                 v in X_SERIAL_CMD_TYPE._enum_map.items()}


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

    def __init__(self, iX=0, iY=0, iTempr=0):
        super(X_TEMPR_INFO, self).__init__()
        self.iX = iX
        self.iY = iY
        self.iTempr = iTempr

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

    # def __init__(self):
    #     super(X_ALARM_LINDED_AREA, self).__init__()
    #     self.iDstTempr = 0
    #     self.iAvgTemptr = 0
    #     self.shAvgAd = 0
    #     self.pDstTemprPt = X_PT()
    #     self.pCentroidPt = X_PT()
    #     self.iAreaPixelCount = 0

# 矩形区域结构体


class X_RECT(Structure):
    _fields_ = [
        ("iLeft", c_int),
        ("iTop", c_int),
        ("iRight", c_int),
        ("iBottom", c_int),
    ]



# 定义 IrData 类
class ImageSize:
    def __init__(self, iImageWidth=640, iImageHeight=480, iPixels=3):
        self.iImageWidth = iImageWidth
        self.iImageHeight = iImageHeight
        self.iPixels = iPixels


class IrData:
    def __init__(self, iImageWidth=640, iImageHeight=480, iPixels=3):
        self.iImageWidth = iImageWidth
        self.iImageHeight = iImageHeight
        self.iPixels = iPixels
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
