#pragma once

#include "XBaseDefine.h"

// ��ʼ�����豸��Ϣ
ExternC XJTech_API int STD_CALL XJTech_StartSearch(X_DEVICE_TYPE eDeviceType, PSEARCH_DEVICE_CALLBACK pSearchDeviceCallback, void* pUserData);

// ���������豸��Ϣ
ExternC XJTech_API void STD_CALL XJTech_StopSearch(X_DEVICE_TYPE eDeviceType);

// �޸��豸��Ϣ
ExternC XJTech_API int STD_CALL XJTech_ModifyDeviceInfo(X_DEVICE_TYPE eDeviceType, X_DEVICE_INFO* pOldDeviceInfo, X_DEVICE_INFO* pNewDeviceInfo, void* pUserData);

// �����豸��Ϣ
ExternC XJTech_API int STD_CALL XJTech_SaveDeviceInfo(X_DEVICE_TYPE eDeviceType, X_DEVICE_INFO * pDeviceInfo, void* pUserData);

// ========================================================================================================================

// ��ʼ�������豸
ExternC XJTech_API int STD_CALL XJTech_Init(const X_DEVICE_TYPE eDeviceType, const X_DEVICE_PARAM &pDeviceParam);

// �󶨱�������
ExternC XJTech_API int STD_CALL XJTech_BindToPhyNetCard(const std::string& strLocalIpAddr);

// �򿪺����豸
ExternC XJTech_API int STD_CALL XJTech_Open(int iUserId, X_DEVICE_INFO* pDeviceInfo, PUPLOAD_DATA_CALLBACK pUploadDataCallback, void* pUserData);

// �򿪺����豸��չ
ExternC XJTech_API int STD_CALL XJTech_OpenEx(int iUserId, X_DEVICE_INFO* pDeviceInfo, PUPLOAD_DATA_CALLBACK_EX pUploadDataCallbackEx, void* pUserData);

// ���������豸
ExternC XJTech_API int STD_CALL XJTech_Reconnect(int iUserId, X_DEVICE_INFO* pDeviceInfo);

// ��ȡ�豸����״̬
ExternC XJTech_API bool STD_CALL XJTech_IsRunning(int iUserId);

// �ر��豸
ExternC XJTech_API void STD_CALL XJTech_Close(int iUserId);

// ж�غ����豸
ExternC XJTech_API void STD_CALL XJTech_UnInit(int iUserId);

// ���ĺ���ɫ��
ExternC XJTech_API void STD_CALL XJTech_ChangePalette(int iUserId, int iPaletteIndex);

// �Ǿ�����У��
ExternC XJTech_API void STD_CALL XJTech_DoFFC(int iUserId);

// ����У��
ExternC XJTech_API void STD_CALL XJTech_Shutter(int iUserId);

// ����
ExternC XJTech_API void STD_CALL XJTech_Focus(int iUserId);

// ����������ȡ�¶�
ExternC XJTech_API float STD_CALL XJTech_GetTemprByPoint(int iUserId, int iX, int iY);

// ��ȡ���������¶�
ExternC XJTech_API void STD_CALL XJTech_GetRectangleTemptr(int iUserId, X_RECT* pRect, X_TEMPR_INFO* pTemprInfo, float fRatio = 1.0F);

// ��ȡ��ƽ���¶�
ExternC XJTech_API int STD_CALL XJTech_GetFPATempr(int iUserId);

// ��ȡ�����׼�¶�
ExternC XJTech_API float STD_CALL XJTech_GetBlackBodyBaseTempr(int iUserId);

// ���ú����׼�¶�
ExternC XJTech_API bool STD_CALL XJTech_SetBlackBodyBaseTempr(int iUserId, float fTempr);

// ��ȡ����У���¶�
ExternC XJTech_API float STD_CALL XJTech_GetBlackBodyCorrTempr(int iUserId);

// ������������
ExternC XJTech_API void STD_CALL XJTech_SetShieldArea(int iUserId, uint8_t* pArea, int iSize);

// ���ú�������
ExternC XJTech_API void STD_CALL XJTech_SetBlackBodyRegion(int iUserId, uint8_t* pRegion, int iSize);

// �����¶�����
ExternC XJTech_API void STD_CALL XJTech_SetTemprCurve(int iUserId, uint8_t* pCurve, int iSize);

// ���ò���ģʽ
ExternC XJTech_API void STD_CALL XJTech_SetMeasureMode(int iUserId, X_MEASURE_MODE eMode);

// �����ֶ��¶Ƚ���ֵ
ExternC XJTech_API void STD_CALL XJTech_SetManualTemprCorrect(int iUserId, float fCorrect);

// ���þ����¶Ƚ���ֵ
ExternC XJTech_API void STD_CALL XJTech_SetDistanceTemprCorrect(int iUserId, float fCorrect);

// ���ñ�����С����ֵ
ExternC XJTech_API void STD_CALL XJTech_SetAlarmMinPixelsValue(int iUserId, int iAlarmMinPixelsValue);

// ���ñ����ȼ�
ExternC XJTech_API void STD_CALL XJTech_SetAlarmLevel(int iUserId, X_ALARM_LEVEL pAlarmLevel);

// ��������
ExternC XJTech_API void STD_CALL XJTech_SetBrightness(int iUserId, int iBrightness);

// ���öԱȶ�
ExternC XJTech_API void STD_CALL XJTech_SetContrast(int iUserId, int iContrast);

// ��������ģʽ
ExternC XJTech_API void STD_CALL XJTech_SetGainMode(int iUserId, bool bGainMode);

// ����ƽ̨�������
ExternC XJTech_API void STD_CALL XJTech_SetPlatDimmerParam(int iUserId, PlatHistDimmerParam* pParam);

// Gamma����
ExternC XJTech_API void STD_CALL XJTech_GammaCorrection(int iUserId, int iGammaTableIndex);

// ��������ģʽУ��ģʽ
ExternC XJTech_API void STD_CALL XJTech_StartBdCorrectMode(int iUserId, bool bBdCorrect);

// ����������������ߴ�
ExternC XJTech_API void STD_CALL XJTech_SetShieldGridSize(int iUserId, int iHorGridNum, int iVerGridNum);

// ����ƥ��У������
ExternC XJTech_API bool STD_CALL XJTech_GetTransformMatrix(X_PT irPointF[3], X_PT vlPointF[3], double TranformMatrix[3][3]);

// ��о����
ExternC XJTech_API int STD_CALL XJTech_UpdateProgram(int iUserId, char* strFileName, uint8_t ucType, uint8_t ucIsReboot);

// FPGA����
ExternC XJTech_API int STD_CALL XJTech_UpdateFPGA(int iUserId, char *strFileName, uint8_t ucFileType);

// ��ȡ��о����汾��
ExternC XJTech_API int STD_CALL XJTech_GetDeviceSoftwareVersion(int iUserId, unsigned int* uiVersion, unsigned int* uiDate);

// �����ͼ��У������
ExternC XJTech_API void STD_CALL XJTech_EnableBackgroundCorrect(int iUserId, bool bEnable);

// �ɼ�����
ExternC XJTech_API void STD_CALL XJTech_CaptureBackground(int iUserId);

// ͸��ͨ��
ExternC XJTech_API void STD_CALL XJTech_SerialSend(int iUserId, X_SERIAL_CMD_TYPE emCmdType, char *pData, int iSize);

// ����/���ú�����ڵ��㷨
ExternC XJTech_API void STD_CALL XJTech_EnableBlackBodyAvoidCover(int iUserId, bool bEnable);

// ��ȡ�豸��������
ExternC XJTech_API bool STD_CALL XJTech_GetDeviceEnvData(int iUserId, uint8_t* pBuffer, unsigned int* uiLength);

// ����/���û����¶Ƚ���
ExternC XJTech_API void STD_CALL XJTech_EnableEnvTemprCorrect(int iUserId, bool bEnable);

// ��ȡ��������
ExternC XJTech_API void STD_CALL XJTech_GetEnvParam(int iUserId, X_ENV_PARAM* pEnvParam);

// ��ȡԭʼ��������
ExternC XJTech_API void STD_CALL XJTech_GetSrcTemperatureData(int iUserId, short* pData);

// ����/����ʱ���˲��㷨
ExternC XJTech_API void STD_CALL XJTech_EnableTimeDomainFilteringAlgo(int iUserId, bool bEnable);

// ����ʱ���˲��㷨����
ExternC XJTech_API void STD_CALL XJTech_SetTimeDomainFilteringAlgoParam(int iUserId, XTimeDomainFilteringParam* pParam);

// ����/����˫���˲��㷨
ExternC XJTech_API void STD_CALL XJTech_EnableBilateralFilteringAlgo(int iUserId, bool bEnable);

// ����˫���˲��㷨����
ExternC XJTech_API void STD_CALL XJTech_SetBilateralFilteringAlgoParam(int iUserId, XBilateralFilteringParam* pParam);

// ͼ�����Ե���
ExternC XJTech_API void STD_CALL XJTech_ImageLinerProcess(int iUserId, short* pSrcData, uint8_t* pDstImage, int iWidth, int iHeight, LinearDimmerParam* pParam = nullptr, int iGammaIndex = 18);

// ͼ��ƽ̨����
ExternC XJTech_API void STD_CALL XJTech_ImagePlatHistProcess(int iUserId, short* pSrcData, uint8_t* pDstImage, int iWidth, int iHeight, PlatHistDimmerParam* pParam = nullptr, int iGammaIndex = 18);

// ����
ExternC XJTech_API void STD_CALL XJTech_Play(int iUserId);

// ��ͣ
ExternC XJTech_API void STD_CALL XJTech_Pause(int iUserId);

// ��ȡ��������
ExternC XJTech_API int STD_CALL XJTech_GetSerialPortData(int iUserId, uint8_t* pBuffer, unsigned int& uiLen);

// �ط� - ��ʼ��
ExternC XJTech_API int STD_CALL XJTech_PlayBack_Init(int iWidth, int iHeight);

// �ط� - ֡ת�� 
ExternC XJTech_API bool STD_CALL XJTech_PlayBack_ConvertFrame(int iPlayBackId, const short* pSrcData, uint8_t* pDstImage);

// �ط� - ����ɫ�� 
ExternC XJTech_API void STD_CALL XJTech_PlayBack_SetPaletteIndex(int iPlayBackId, int iIndex);

// �ط� - ����Gammaֵ 
ExternC XJTech_API void STD_CALL XJTech_PlayBack_SetGammaIndex(int iPlayBackId, int iIndex);

// �ط� - ����
ExternC XJTech_API void STD_CALL XJTech_PlayBack_UnInit(int iPlayBackId);

// ���ò��²���
ExternC XJTech_API void STD_CALL XJTech_SetMeasureParam(int iUserId, XMeasureParam* pParam);

// ========================================================================================================================

// ��ʼģ�����ݲ���
ExternC XJTech_API void STD_CALL XJTech_StartSimDataTest(int iUserId, PUPLOAD_DATA_CALLBACK pUploadDataCallback);

// ֹͣģ�����ݲ���
ExternC XJTech_API void STD_CALL XJTech_StopSimDataTest(int iUserId);