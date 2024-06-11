#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_interfaces/msg/point_tempr_info.hpp>

#include <queue>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <fstream>
#include <opencv2/opencv.hpp>

#include "XBaseDefine.h"

// const int iImageWidth  = 360;
// const int iImageHeight = 288;
// const int iPixels      = 3;

enum E_PIXEL_FORMAT
{
  PF_INVALID = -1,

  PF_RGB,  // GRB

  PF_GRAY,  // Gray

  PF_COUNT
};

struct XBitmapFileHeader
{
  unsigned int bfSize;        /* Size of file */
  unsigned short bfReserved1; /* Reserved */
  unsigned short bfReserved2; /* ... */
  unsigned int bfOffBits;     /* Offset to bitmap data */

  XBitmapFileHeader()
  {
    memset(this, 0, sizeof(*this));
  }
};

struct XBitmapInfoHeader
{
  unsigned int biSize;         /* Size of info header */
  int biWidth;                 /* Width of image */
  int biHeight;                /* Height of image */
  unsigned short biPlanes;     /* Number of color planes */
  unsigned short biBitCount;   /* Number of bits per pixel */
  unsigned int biCompression;  /* Type of compression to use */
  unsigned int biSizeImage;    /* Size of image data */
  int biXPelsPerMeter;         /* X pixels per meter */
  int biYPelsPerMeter;         /* Y pixels per meter */
  unsigned int biClrUsed;      /* Number of colors used */
  unsigned int biClrImportant; /* Number of important colors */

  XBitmapInfoHeader()
  {
    memset(this, 0, sizeof(*this));
  }
};

struct ImageSize
{
  int iImageWidth;
  int iImageHeight;
  int iPixels;
};

class IrData
{
public:
  IrData(int iImageWidth_, int iImageHeight_, int iPixels_)
    : iImageWidth(iImageWidth_), iImageHeight(iImageHeight_), iPixels(iPixels_)
  {
    pImage = new unsigned char[iImageWidth * iImageHeight * iPixels];
    pMaxTemprInfo = new X_TEMPR_INFO();
    pMinTemprInfo = new X_TEMPR_INFO();
  }

  // IrData()
  // {
  //     pImage = new unsigned char[iImageWidth * iImageHeight * iPixels];
  //     pMaxTemprInfo = new X_TEMPR_INFO();
  //     pMinTemprInfo = new X_TEMPR_INFO();
  // }

  IrData(const IrData& data)
  {
    memcpy(this->pImage, data.pImage, data.iImageWidth * data.iImageHeight * data.iPixels * sizeof(unsigned char));
    memcpy(this->pMaxTemprInfo, data.pMaxTemprInfo, sizeof(X_TEMPR_INFO));
    memcpy(this->pMinTemprInfo, data.pMinTemprInfo, sizeof(X_TEMPR_INFO));
  }

  ~IrData()
  {
    if (NULL != pImage)
    {
      delete[] pImage;
      pImage = NULL;
    }

    if (NULL != pMaxTemprInfo)
    {
      delete pMaxTemprInfo;
      pMaxTemprInfo = NULL;
    }

    if (NULL != pMinTemprInfo)
    {
      delete pMinTemprInfo;
      pMinTemprInfo = NULL;
    }
  }

public:
  unsigned char* pImage;
  X_TEMPR_INFO* pMaxTemprInfo;
  X_TEMPR_INFO* pMinTemprInfo;
  int iImageWidth;
  int iImageHeight;
  int iPixels;
};

class XjtechRos2 : public rclcpp::Node
{
private:
  /* data */
  // ros
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher_;
  rclcpp::Publisher<message_interfaces::msg::PointTemprInfo>::SharedPtr temprPointPublisher_;

  // parameters
  static ImageSize imageSize;
  PlatHistDimmerParam pPlatHistDimmer;
  X_DEVICE_TYPE eDeviceType;
  X_DEVICE_PARAM pDeviceParam;
  int Palette;
  int Gamma;
  bool showImage;
  int TemperatureMeasurementMode;
  double StandardTemperature;
  int PointX;
  int PointY;
  int LeftTopPointX;
  int LeftTopPointY;
  int RightBottomPointX;
  int RightBottomPointY;
  // 图片保存
  std::string m_strSaveDir;
  bool SaveTemprImage;        // 是否保存温度图像
  std::string ImageSavePath;  // 温度图像保存路径
  int ImageSaveInterval;  // 图像保存间隔，0：逐帧保存 1：间隔1帧保存，范围0-24 （0-FrameRate-1）
  int iWaitSaveImageCount = -1;
  // 测温曲线保存
  bool SaveTemprCurve;        // 是否保存温度曲线
  std::string CurveSavePath;  // 温度曲线保存路径
  int iWaitSaveCurveCount = 0;
  bool SaveTemprToFile;       //  是否保存温度到csv文件
  int FileSplitsLines;        // 文件长度达到多少行进行分割
  std::string FilesSavePath;  // 文件保存路径

  // devices
  int iUserId;
  bool is_connect;
  static std::vector<X_DEVICE_INFO*> g_vecDevices;
  static std::queue<IrData*> g_lstIrData;

  // opencv
  cv::Mat image;       // (320, 640, CV_8UC3, Scalar(255, 255, 255));
  cv::Mat image_show;  // (320, 640, CV_8UC3, Scalar(255, 255, 255));

  // file
  std::queue<std::string> messages;  // 消息队列
  std::mutex mtx;                    // 互斥锁
  std::condition_variable cv;        // 条件变量
  bool stop = false;                 // 停止标志
  std::ofstream file;
  long long iCurrentLines = 0;
  int iCounter = 1;  // 切割计数器
  std::thread m_thread;

private:
  void timer_callback();
  void SaveTemprToFileThread();

public:
  XjtechRos2(/* args */);
  ~XjtechRos2();

  void GetCurrentWorkPath();
  void CreateSaveDirectory(const char* sFilePath);
  std::string GetCurrentTimes();

  bool SaveImage(const char* szFileName, unsigned char* pImageData, int iWdith, int iHeight,
                 E_PIXEL_FORMAT emPixelFormat);

  static void SearchDeviceCallbackFunc(X_DEVICE_INFO* pDeviceInfo, void* pUserData);

  static void UploadDataCallbackFunc(unsigned char* pImage, X_TEMPR_INFO* pMaxTemprInfo, X_TEMPR_INFO* pMinTemprInfo,
                                     X_ALARM_LINDED_AREA* pAlarmLinkedArea, int iAlarmAreaCount,
                                     unsigned char* pAlarmMask, void* pUserData);

  void ImageProcessThreadFunc();

  bool SearchDevices();
  bool InitDevice();
  bool OpenDevices();
  void CloseDevices();
  void SetDevicesParameters();

  void DefinedParameters();
  bool UpdateParamers();
  void PrintParameters();

  void PublishPointTempr(int iPointX, int iPointY, float fTempr);
  void PublishImage(unsigned char* pImage);

  void PrintImage(int iPointX, int iPointY, float fTempr, unsigned char* pImage);
  void MakeTemprCurve(float fTempr);
  void PrintTemprCurve(float fTempr);
  void SaveImage(unsigned char* pImage);
  void SaveCurve();
  void SaveLastCurve();
  void SaveTemprToFiles(std::string sTempr);
};
