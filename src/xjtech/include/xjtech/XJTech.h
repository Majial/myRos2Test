#pragma once

#include "XBaseDefine.h"

// 开始搜索设备信息
ExternC XJTech_API int STD_CALL XJTech_StartSearch(X_DEVICE_TYPE eDeviceType, PSEARCH_DEVICE_CALLBACK pSearchDeviceCallback, void* pUserData);

// 结束搜索设备信息
ExternC XJTech_API void STD_CALL XJTech_StopSearch(X_DEVICE_TYPE eDeviceType);

// 修改设备信息
ExternC XJTech_API int STD_CALL XJTech_ModifyDeviceInfo(X_DEVICE_TYPE eDeviceType, X_DEVICE_INFO* pOldDeviceInfo, X_DEVICE_INFO* pNewDeviceInfo, void* pUserData);

// 保存设备信息
ExternC XJTech_API int STD_CALL XJTech_SaveDeviceInfo(X_DEVICE_TYPE eDeviceType, X_DEVICE_INFO * pDeviceInfo, void* pUserData);

// ========================================================================================================================

// 初始化红外设备
ExternC XJTech_API int STD_CALL XJTech_Init(const X_DEVICE_TYPE eDeviceType, const X_DEVICE_PARAM &pDeviceParam);

// 绑定本地网卡
ExternC XJTech_API int STD_CALL XJTech_BindToPhyNetCard(const std::string& strLocalIpAddr);

// 打开红外设备
ExternC XJTech_API int STD_CALL XJTech_Open(int iUserId, X_DEVICE_INFO* pDeviceInfo, PUPLOAD_DATA_CALLBACK pUploadDataCallback, void* pUserData);

// 打开红外设备扩展
ExternC XJTech_API int STD_CALL XJTech_OpenEx(int iUserId, X_DEVICE_INFO* pDeviceInfo, PUPLOAD_DATA_CALLBACK_EX pUploadDataCallbackEx, void* pUserData);

// 重连红外设备
ExternC XJTech_API int STD_CALL XJTech_Reconnect(int iUserId, X_DEVICE_INFO* pDeviceInfo);

// 获取设备运行状态
ExternC XJTech_API bool STD_CALL XJTech_IsRunning(int iUserId);

// 关闭设备
ExternC XJTech_API void STD_CALL XJTech_Close(int iUserId);

// 卸载红外设备
ExternC XJTech_API void STD_CALL XJTech_UnInit(int iUserId);

// 更改红外色带
ExternC XJTech_API void STD_CALL XJTech_ChangePalette(int iUserId, int iPaletteIndex);

// 非均匀性校正
ExternC XJTech_API void STD_CALL XJTech_DoFFC(int iUserId);

// 快门校正
ExternC XJTech_API void STD_CALL XJTech_Shutter(int iUserId);

// 调焦
ExternC XJTech_API void STD_CALL XJTech_Focus(int iUserId);

// 根据坐标点获取温度
ExternC XJTech_API float STD_CALL XJTech_GetTemprByPoint(int iUserId, int iX, int iY);

// 获取矩形区域温度
ExternC XJTech_API void STD_CALL XJTech_GetRectangleTemptr(int iUserId, X_RECT* pRect, X_TEMPR_INFO* pTemprInfo, float fRatio = 1.0F);

// 获取焦平面温度
ExternC XJTech_API int STD_CALL XJTech_GetFPATempr(int iUserId);

// 获取黑体基准温度
ExternC XJTech_API float STD_CALL XJTech_GetBlackBodyBaseTempr(int iUserId);

// 设置黑体基准温度
ExternC XJTech_API bool STD_CALL XJTech_SetBlackBodyBaseTempr(int iUserId, float fTempr);

// 获取黑体校正温度
ExternC XJTech_API float STD_CALL XJTech_GetBlackBodyCorrTempr(int iUserId);

// 设置屏蔽区域
ExternC XJTech_API void STD_CALL XJTech_SetShieldArea(int iUserId, uint8_t* pArea, int iSize);

// 设置黑体区域
ExternC XJTech_API void STD_CALL XJTech_SetBlackBodyRegion(int iUserId, uint8_t* pRegion, int iSize);

// 设置温度曲线
ExternC XJTech_API void STD_CALL XJTech_SetTemprCurve(int iUserId, uint8_t* pCurve, int iSize);

// 设置测温模式
ExternC XJTech_API void STD_CALL XJTech_SetMeasureMode(int iUserId, X_MEASURE_MODE eMode);

// 设置手动温度矫正值
ExternC XJTech_API void STD_CALL XJTech_SetManualTemprCorrect(int iUserId, float fCorrect);

// 设置距离温度矫正值
ExternC XJTech_API void STD_CALL XJTech_SetDistanceTemprCorrect(int iUserId, float fCorrect);

// 设置报警最小像素值
ExternC XJTech_API void STD_CALL XJTech_SetAlarmMinPixelsValue(int iUserId, int iAlarmMinPixelsValue);

// 设置报警等级
ExternC XJTech_API void STD_CALL XJTech_SetAlarmLevel(int iUserId, X_ALARM_LEVEL pAlarmLevel);

// 设置亮度
ExternC XJTech_API void STD_CALL XJTech_SetBrightness(int iUserId, int iBrightness);

// 设置对比度
ExternC XJTech_API void STD_CALL XJTech_SetContrast(int iUserId, int iContrast);

// 设置增益模式
ExternC XJTech_API void STD_CALL XJTech_SetGainMode(int iUserId, bool bGainMode);

// 设置平台调光参数
ExternC XJTech_API void STD_CALL XJTech_SetPlatDimmerParam(int iUserId, PlatHistDimmerParam* pParam);

// Gamma矫正
ExternC XJTech_API void STD_CALL XJTech_GammaCorrection(int iUserId, int iGammaTableIndex);

// 开启黑体模式校正模式
ExternC XJTech_API void STD_CALL XJTech_StartBdCorrectMode(int iUserId, bool bBdCorrect);

// 设置屏蔽区域网格尺寸
ExternC XJTech_API void STD_CALL XJTech_SetShieldGridSize(int iUserId, int iHorGridNum, int iVerGridNum);

// 三点匹配校正矩阵
ExternC XJTech_API bool STD_CALL XJTech_GetTransformMatrix(X_PT irPointF[3], X_PT vlPointF[3], double TranformMatrix[3][3]);

// 机芯升级
ExternC XJTech_API int STD_CALL XJTech_UpdateProgram(int iUserId, char* strFileName, uint8_t ucType, uint8_t ucIsReboot);

// FPGA升级
ExternC XJTech_API int STD_CALL XJTech_UpdateFPGA(int iUserId, char *strFileName, uint8_t ucFileType);

// 获取机芯软件版本号
ExternC XJTech_API int STD_CALL XJTech_GetDeviceSoftwareVersion(int iUserId, unsigned int* uiVersion, unsigned int* uiDate);

// 激活本底图像校正功能
ExternC XJTech_API void STD_CALL XJTech_EnableBackgroundCorrect(int iUserId, bool bEnable);

// 采集本底
ExternC XJTech_API void STD_CALL XJTech_CaptureBackground(int iUserId);

// 透明通道
ExternC XJTech_API void STD_CALL XJTech_SerialSend(int iUserId, X_SERIAL_CMD_TYPE emCmdType, char *pData, int iSize);

// 启用/禁用黑体防遮挡算法
ExternC XJTech_API void STD_CALL XJTech_EnableBlackBodyAvoidCover(int iUserId, bool bEnable);

// 获取设备环境数据
ExternC XJTech_API bool STD_CALL XJTech_GetDeviceEnvData(int iUserId, uint8_t* pBuffer, unsigned int* uiLength);

// 启用/禁用环境温度矫正
ExternC XJTech_API void STD_CALL XJTech_EnableEnvTemprCorrect(int iUserId, bool bEnable);

// 获取环境参数
ExternC XJTech_API void STD_CALL XJTech_GetEnvParam(int iUserId, X_ENV_PARAM* pEnvParam);

// 获取原始测温数据
ExternC XJTech_API void STD_CALL XJTech_GetSrcTemperatureData(int iUserId, short* pData);

// 激活/禁用时域滤波算法
ExternC XJTech_API void STD_CALL XJTech_EnableTimeDomainFilteringAlgo(int iUserId, bool bEnable);

// 设置时域滤波算法参数
ExternC XJTech_API void STD_CALL XJTech_SetTimeDomainFilteringAlgoParam(int iUserId, XTimeDomainFilteringParam* pParam);

// 激活/禁用双边滤波算法
ExternC XJTech_API void STD_CALL XJTech_EnableBilateralFilteringAlgo(int iUserId, bool bEnable);

// 设置双边滤波算法参数
ExternC XJTech_API void STD_CALL XJTech_SetBilateralFilteringAlgoParam(int iUserId, XBilateralFilteringParam* pParam);

// 图像线性调光
ExternC XJTech_API void STD_CALL XJTech_ImageLinerProcess(int iUserId, short* pSrcData, uint8_t* pDstImage, int iWidth, int iHeight, LinearDimmerParam* pParam = nullptr, int iGammaIndex = 18);

// 图像平台调光
ExternC XJTech_API void STD_CALL XJTech_ImagePlatHistProcess(int iUserId, short* pSrcData, uint8_t* pDstImage, int iWidth, int iHeight, PlatHistDimmerParam* pParam = nullptr, int iGammaIndex = 18);

// 播放
ExternC XJTech_API void STD_CALL XJTech_Play(int iUserId);

// 暂停
ExternC XJTech_API void STD_CALL XJTech_Pause(int iUserId);

// 获取串口数据
ExternC XJTech_API int STD_CALL XJTech_GetSerialPortData(int iUserId, uint8_t* pBuffer, unsigned int& uiLen);

// 回放 - 初始化
ExternC XJTech_API int STD_CALL XJTech_PlayBack_Init(int iWidth, int iHeight);

// 回放 - 帧转换 
ExternC XJTech_API bool STD_CALL XJTech_PlayBack_ConvertFrame(int iPlayBackId, const short* pSrcData, uint8_t* pDstImage);

// 回放 - 设置色带 
ExternC XJTech_API void STD_CALL XJTech_PlayBack_SetPaletteIndex(int iPlayBackId, int iIndex);

// 回放 - 设置Gamma值 
ExternC XJTech_API void STD_CALL XJTech_PlayBack_SetGammaIndex(int iPlayBackId, int iIndex);

// 回放 - 析构
ExternC XJTech_API void STD_CALL XJTech_PlayBack_UnInit(int iPlayBackId);

// 设置测温参数
ExternC XJTech_API void STD_CALL XJTech_SetMeasureParam(int iUserId, XMeasureParam* pParam);

// ========================================================================================================================

// 开始模拟数据测试
ExternC XJTech_API void STD_CALL XJTech_StartSimDataTest(int iUserId, PUPLOAD_DATA_CALLBACK pUploadDataCallback);

// 停止模拟数据测试
ExternC XJTech_API void STD_CALL XJTech_StopSimDataTest(int iUserId);