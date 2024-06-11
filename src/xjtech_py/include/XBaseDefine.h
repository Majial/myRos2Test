#pragma once

#include <stdint.h>
#include <string>

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN             //  从 Windows 头文件中排除极少使用的信息
#include <windows.h>
#define XJTech_API	__declspec(dllexport)
#define STD_CALL	__stdcall

#else

#include <cstring>
#define XJTech_API	
#define STD_CALL	//__attribute__((__stdcall__)) 
typedef unsigned int 	DWORD;	
		
#endif

#define ExternC extern "C"

#include <assert.h>

#define  DEVICE_SEARCH_PORT     10725
#define  CONFIG_UDP_PORT		3956		// 监听端端口号：广播获得网络设备的地址，配置IP，配置帧率，机芯控制, 设备复位(0x)等

#define IP_MARK					"."
#define ByteMask				0x00FF

#define MAX_ALRAM_AREA_NUM		100		// 报警区域最大数目

#define FLOAT_EPS				1e-6
#define DOUBLE_EPS				1e-15

#define X_ABS(x)				((x) > 0 ? (x) : -(x))				// 求绝对值
#define X_MAX(x, y)				(((x) > (y)) ? (x) : (y))			// 求最大值
#define X_MIN(x, y)				(((x) < (y)) ? (x) : (y))			// 求最小值
#define X_ARR_SIZE(a)			(sizeof((a)) / sizeof((a[0])))		// 获取数组元素个数

#define X_SATURATION(upper, lower, value) (((value) > (upper)) ? (upper) : (((value) < (lower)) ? (lower) : (value)))	// 饱和运算

#define X_DEL_POINT(p)			{ if (nullptr != p) { delete (p); p = nullptr; } }		// 删除指针
#define X_DEL_ARRAY(p)			{ if (nullptr != p) { delete[] (p); p = nullptr; } }		// 删除数组

#define IS_FLOAT_EQUAL(x, y)	(X_ABS((x) - (y)) <= FLOAT_EPS)
#define IS_DOUBLE_EQUAL(x, y)	(X_ABS((x) - (y)) <= DOUBLE_EPS)

const static int s_iMargeDis = 0;

#define DOMAINH_DIM	4

// 4领域索引
const static int s_pDx[DOMAINH_DIM] = { -1, 0, 0, 1 };
const static int s_pDy[DOMAINH_DIM] = { 0, -1, 1, 0 };

// 温度数据类型
enum X_TEMPR_DATA_TYPE
{
	E_TYPE_NONE = 0,		// 类型非法

	E_TYPE_SHORT,			// short类型

	E_TYPE_UNSIGNED_SHORT,	// uint16_t 类型

	E_TYPE_MAX
};

// 图像翻转类型
enum X_FLIP_TYPE
{
	E_FLIP_NONE = 0,	// 不翻转

	E_FLIP_R90,			// 顺时针翻转90度

	E_FLIP_L90,			// 逆时针翻转90度

	E_FLIP_180,			// 翻转180度

	E_FLIP_MAX			// 翻转最大值（哨兵）
};

// 报警类型
enum X_ALARM_TYPE
{
	E_HIGH_ALARM = 0,	// 高温报警
			
	E_LOW_ALARM,		// 低温报警	

	E_BOTH_ALARM		// 高低温报警
};

// 测温模型
enum X_MEASURE_MODE
{
	E_SURFACE = 0,	// 体表

	E_ARMPIT		// 腋下
};

// 增益模式
enum X_GAIN_MODE
{
	E_MANUAL = 0,	// 手动

	E_SEMI_AUTO,	// 半自动

	E_AUTO			// 自动
};

enum X_SERIAL_CMD_TYPE
{
	// 无
	CMD_TYPE_NONE = 0,

	// 快门补偿
	CMD_TYPE_SHUTTER = 1,

	// 设置快门间隔时间
	CMD_TYPE_SET_SHUTTER_INTERVAL = 2,

	// 保存快门间隔时间配置
	CMD_TYPE_SAVE_SHUTTER_INTERAVL = 3,

	// 设置色带
	CMD_TYPE_PALETTE = 4,
	
	// 增益调节
	CMD_TYPE_GAIN_ADJUST = 5,

	// 自定义
	CMD_TYPE_USER_DEFINE = 6,

	// 最大值
	CMD_TYPE_MAX
};

#define  MAX_CMD_LEN_NEW 16

enum IPCFG_COMMAND_ITEMS
{
	//请求命令字_搜索网络红外设备
	IPCFG_REQUEST_PRESERVE = 1,

	//请求命令字_打开红外设备
	IPCFG_REQUEST_OPENDEVICE = 2,

	//请求命令字_打开红外设备
	IPCFG_REQUEST_GETDATA = 3,

	//请求命令字_关闭红外设备
	IPCFG_REPLY_CLOSEDEVICE = 5,

	//请求命令字_发送心跳包
	IPCFG_REQUEST_HEARTBEAT = 6,

	//请求命令字_修改红外设备网络配置
	IPCFG_REQUEST_SEARCHDEVICE = 7,

	//请求命令字_修改红外设备网络配置（仅修改IP）
	IPCFG_REQUEST_NETWORKCONFIG = 8,

	//请求命令字_保存红外设备网络配置
	IPCFG_REQUEST_SAVENETWORKCONFIG = 9,

	//请求命令字_保存红外设备网络配置
	IPCFG_REQUEST_FRONTTEMPERATURE = 10,

	//请求命令字_非均匀性校正
	IPCFG_REQUEST_DOFFC = 11,

	//请求命令字_取消非均匀性校正
	IPCFG_REQUEST_UnDOFFC = 12,

	//请求命令字_保存红外设备网络配置（MAC和IP同时修改）
	IPCFG_REQUEST_NETWORKCONFIG_BOTH = 13,

	//请求命令字_快门校正
	IPCFG_REQUEST_SHUTTER = 14,

	//请求命令字_调焦
	IPCFG_REQUEST_FOCUS = 15,

	//回复命令字_打开红外设备
	IPCFG_REPLY_OPENDEVICE = 256 + IPCFG_REQUEST_OPENDEVICE,

	//回复命令字_搜索网络红外设备
	IPCFG_REPLY_SEARCHDEVICE = 256 + IPCFG_REQUEST_SEARCHDEVICE,

	//回复命令字_修改红外设备网络配置
	IPCFG_REPLY_NETWORKCONFIG = 256 + IPCFG_REQUEST_NETWORKCONFIG,

	//回复命令字_修改红外设备网络配置
	IPCFG_REPLY_SAVENETWORKCONFIG = 256 + IPCFG_REQUEST_SAVENETWORKCONFIG,

	//回复命令字_修改红外设备网络配置
	IPCFG_REPLY_FRONTTEMPERATURE = 256 + IPCFG_REQUEST_FRONTTEMPERATURE
};

// 收到的指令帧结构
typedef struct tagIpcfgCommand
{
	// 命令头字段
	uint8_t Header;

	//指令，具体命令参考IPCFG_COMMAND_ITEMS枚举
	uint8_t Command1;

	uint8_t Command2;
	// 对于特定指令的特定参数段
	union
	{
		// IPCFG_REQUEST_NETWORK指令
		struct
		{
			uint8_t mac[6];
			uint8_t addr[4];
			uint8_t mask[4];
			uint8_t gate[4];
		}Set;

		// IPCFG_REPLY_NETWORK指令
		struct
		{
			uint8_t mac[6];
			uint8_t addr[4];
			uint8_t mask[4];
			uint8_t gate[4];
		}Reply;

	}Parameters;

	//控制命令缓冲区
	uint8_t ControlCommandBuffer[MAX_CMD_LEN_NEW];

}IPCFGCOMMAND, *LPIPCFGCOMMAND;

// 设备信息结构体
struct X_DEVICE_INFO
{
	char szMac[20];      // 设备Mac地址
	char szIpAddr[16];   // 设备IP地址
	char szSubNet[16];   // 设备子网掩码
	char szGateWay[16];  // 设备网关
	int  iPort;          // 端口号

	X_DEVICE_INFO()
	{
		memset(this, 0, sizeof(X_DEVICE_INFO));
	}
};

// RGB结构体
struct X_RGBQUAD 
{
	uint8_t ucBlue;
	uint8_t ucGreen;
	uint8_t ucRed;
	uint8_t ucReserved;

	X_RGBQUAD()
	{
		memset(this, 0, sizeof(X_RGBQUAD));
	}
};

// 点结构体
struct X_PT
{
	int iX;  // X坐标
	int iY;  // Y坐标

	X_PT()
	{
		memset(this, 0, sizeof(X_PT));
	}

	X_PT(int iX, int iY)
	{
		this->iX = iX;
		this->iY = iY;
	}

	X_PT& operator+(const X_PT& obj)
	{
		X_PT pt;
		pt.iX = this->iX + obj.iX;
		pt.iY = this->iY + obj.iY;

		return *this;
	}

	X_PT& operator=(const X_PT& obj)
	{
		if (this != &obj)
		{
			this->iX = obj.iX;
			this->iY = obj.iY;
		}

		return *this;
	}
};

// 矩形区域结构体
struct X_RECT
{
	int iLeft;    // 左上角X坐标
	int iTop;     // 左上角Y坐标
	int iRight;   // 右下角X坐标
	int iBottom;  // 右下角Y坐标

	X_RECT()
	{
		memset(this, 0, sizeof(X_RECT));
	}
};

// 温度点结构体
struct X_TEMPR_PT
{
	X_PT	pt;     // 点结构体
	float	fTempr; // 温度
	short	shAd;   // AD值

	X_TEMPR_PT()
	{
		memset(this, 0, sizeof(X_TEMPR_PT));
	}

	X_TEMPR_PT(X_PT pt, float fTempr, short shAd)
	{
		this->pt = pt;
		this->fTempr = fTempr;
		this->shAd = shAd;
	}
};

// 温度信息结构体
struct X_TEMPR_INFO
{
	int iX;        // X坐标		
	int iY;        // Y坐标
	int	iTempr;    // 温度（除以10为实际温度）

	X_TEMPR_INFO()
	{
		memset(this, 0, sizeof(X_TEMPR_INFO));
	}
};

// 报警级别结构体
struct X_ALARM_LEVEL
{
	float	fOne;    // 报警等级1温度
	float	fTwo;    // 报警等级2温度
	float	fThree;  // 报警等级3温度
	float	fFour;   // 报警等级4温度

	X_ALARM_LEVEL()
	{
		memset(this, 0, sizeof(X_ALARM_LEVEL));
	}
};

// 连通区域结构体
struct X_ALARM_LINDED_AREA
{
	int		iDstTempr;			// 最高或者最低温度
	int		iAvgTemptr;			// 平均温度
	short	shAvgAd;			// 平均AD值
	X_PT	pDstTemprPt;		// 最高温度对应的图像坐标
	X_PT	pCentroidPt;		// 质心图像坐标
	int		iAreaPixelCount;	// 区域点个数

	X_ALARM_LINDED_AREA()
	{
		memset(this, 0, sizeof(X_ALARM_LINDED_AREA));
	}
};

// 回调数据结构体
struct X_CALLBACK_DATA
{
	uint8_t*				pImage;					// 红外图像数据（RGB）
	float*					pSrcTemperature;		// 原始温度数据（全图）
	float*					pOptTemperature;		// 优化温度数据（全图）
	X_TEMPR_INFO*			pMaxTemprInfo;			// 高温信息
	X_TEMPR_INFO*			pMinTemprInfo;			// 低温信息
	X_ALARM_LINDED_AREA*	pAlarmLinkedArea;		// 连通区域
	int						iAlarmAreaCount;		// 连通区域个数
	uint8_t*				pAlarmMask;				// 报警模板（全图）
	void*					pUserData;				// 用户数据

	X_CALLBACK_DATA()
	{
		memset(this, 0, sizeof(X_CALLBACK_DATA));
	}
};

// 网络设备搜索回调函数
typedef void (STD_CALL *PSEARCH_DEVICE_CALLBACK)(X_DEVICE_INFO* pDeviceInfo, void *pUserData);

// 上传数据回调函数
typedef void (STD_CALL *PUPLOAD_DATA_CALLBACK)(uint8_t* pImage, 
	X_TEMPR_INFO* pMaxTemprInfo, X_TEMPR_INFO* pMinTemprInfo,
	X_ALARM_LINDED_AREA* pAlarmLinkedArea, int iAlarmAreaCount, uint8_t* pAlarmMask, void* pUserData);

// 上传数据回调函数
typedef void (STD_CALL *PUPLOAD_DATA_CALLBACK_EX)(X_CALLBACK_DATA* pCallbackData);

// 设备类型
enum X_DEVICE_TYPE
{
	E_NONE = 0,		// 无

	E_UDP,			// UDP设备

	E_TCP_HISI,		// TCP_HISI设备（迅检红外热像仪）

	E_RTSP,			// RTSP设备

	E_TCP_SAM,		// TCP_SAM设备

	E_UVC,			// UVC设备
};

// 设备参数
struct X_DEVICE_PARAM
{
	int				iFrameRate;			// 图像帧频
	int				iSrcWidth;			// 原始图像/温度宽度
	int				iSrcHeight;			// 原始图像/温度高度
	int				iDstWidth;			// 目标图像/温度宽度
	int				iDstHeight;			// 目标图像/温度高度
	int				iCutHorOffset;		// 数据裁剪水平偏移量
	int				iCutVerOffset;		// 数据裁剪垂直偏移量
	int				bCut;				// 数据裁剪标志
	X_ALARM_TYPE    eAlarmType;			// 报警类型
	X_FLIP_TYPE		eFlipType;			// 数据翻转类型

	X_DEVICE_PARAM()
	{
		memset(this, 0, sizeof(X_DEVICE_PARAM));
	}
};

struct LinearDimmerParam
{
	X_GAIN_MODE eGainMode;				// 增益模式
	double dLowerDiscardRatio;			// 下抛点率（默认值1%）
	double dUpperDiscardRatio;			// 上抛点率（默认值1%）
	double dBrightness;					// 亮度
	double dContrast;					// 对比度

	LinearDimmerParam()
		: eGainMode(E_AUTO)
		, dLowerDiscardRatio(0.01)
		, dUpperDiscardRatio(0.01)
		, dBrightness(128.0)
		, dContrast(50.0)
	{
	}
};

struct PlatHistDimmerParam
{
	int		iPlatThresholdValue;		// 平台阈值（范围1 ~ 200，默认值100）
	int		iMappingMidValue;			// 映射中间值，调整亮度（范围0 ~ 255，默认值128）
	double	dLowerDiscardRatio;			// 下抛点率（默认值1%）
	double	dUpperDiscardRatio;			// 上抛点率（默认值1%）
	int		iDynamicRangeCoef;			// 动态范围系数
	int		iMappingRange;				// 映射范围

	PlatHistDimmerParam()
		: iPlatThresholdValue(100)
		, iMappingMidValue(128)
		, dLowerDiscardRatio(0.01)
		, dUpperDiscardRatio(0.01)
		, iDynamicRangeCoef(10)
		, iMappingRange(300)
	{
	}
};

// 环境参数
struct X_ENV_PARAM
{
	float fCurrEnvTempr;				// 当前环境温度
	float fBaseEnvTempr;				// 基准环境温度
	float fEnvCorreStep;				// 环境温度矫正系数
	uint8_t chEnvCorreSwitch;		// 下位机环温矫正开关（0代表关，1代表开）

	X_ENV_PARAM()
	{
		memset(this, 0, sizeof(X_ENV_PARAM));
	}
};

// 时域滤波参数结构体
struct XTimeDomainFilteringParam
{
	short	nBegThrd;		// 开始阈值
	short	nLowThrd;		// 低阈值
	short	nHigThrd;		// 高阈值
	short	nEndThrd;		// 结束阈值

	XTimeDomainFilteringParam()
		: nBegThrd(0)
		, nLowThrd(4)
		, nHigThrd(12)
		, nEndThrd(16)
	{
	}
};

// 双边滤波参数结构体
struct XBilateralFilteringParam
{
	short	pWeightMatrices[25];
	short	pDiffTable[64];
	short	nDiffThrd;

	XBilateralFilteringParam()
	{
		memset(pWeightMatrices, 0, sizeof(pWeightMatrices));
		memset(pDiffTable, 0, sizeof(pDiffTable));
		nDiffThrd = 64;
	}
};

// 测温参数
struct XMeasureParam
{
	short nSubCoeff;
	float fDivCoeff;

	XMeasureParam() : nSubCoeff(10000), fDivCoeff(100.0F)
	{

	}
};