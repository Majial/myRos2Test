#include "XjtechRos2.hpp"

#include <chrono>
#include <memory>
#include <sys/types.h>
#include <sys/stat.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <XJTech.h>

using namespace std::chrono_literals;
using namespace cv;

std::vector<X_DEVICE_INFO*> XjtechRos2::g_vecDevices;
std::queue<IrData*> XjtechRos2::g_lstIrData;
ImageSize XjtechRos2::imageSize;
// int iImageWidth;
// int iImageHeight;
// int iPixels;

XjtechRos2::XjtechRos2(/* args */) : Node("xjtech_node")
{
  is_connect = false;

  DefinedParameters();
  UpdateParamers();
  PrintParameters();
  imagePublisher_ = this->create_publisher<sensor_msgs::msg::Image>("/xjtech_image", 1);
  temprPointPublisher_ = this->create_publisher<message_interfaces::msg::PointTemprInfo>("/xjtech_tempr", 1);
  timer_ = this->create_wall_timer(10ms, std::bind(&XjtechRos2::timer_callback, this));

  SearchDevices();
  InitDevice();
  if (OpenDevices())
  {
    SetDevicesParameters();
  }
  else
  {
    // std::cout << "connect err" << std::endl;
    // exit(-1);
    // exit(EXIT_FAILURE);
    rclcpp::shutdown();
  }

  image = Mat(320, 640, CV_8UC3, cv::Scalar(255, 255, 255));
  image_show = Mat(320, 640, CV_8UC3, cv::Scalar(255, 255, 255));

  if (SaveTemprImage)
  {
    if (ImageSavePath == "default")
    {
      GetCurrentWorkPath();
      ImageSavePath = m_strSaveDir + "image/";
      CreateSaveDirectory(ImageSavePath.c_str());
    }
    else
    {
      ImageSavePath += "/";
      CreateSaveDirectory(ImageSavePath.c_str());
    }
  }

  if (SaveTemprCurve)
  {
    if (CurveSavePath == "default")
    {
      GetCurrentWorkPath();
      CurveSavePath = m_strSaveDir + "curve/";
      CreateSaveDirectory(CurveSavePath.c_str());
    }
    else
    {
      CurveSavePath += "/";
      CreateSaveDirectory(CurveSavePath.c_str());
    }
  }

  if (SaveTemprToFile)
  {
    if (FilesSavePath == "default")
    {
      GetCurrentWorkPath();
      FilesSavePath = m_strSaveDir + "files/";
      CreateSaveDirectory(FilesSavePath.c_str());
    }
    else
    {
      FilesSavePath += "/";
      CreateSaveDirectory(FilesSavePath.c_str());
    }
    // 保存文件线程
    auto fileName = FilesSavePath + "temprs.csv";
    file.open(fileName);
    m_thread = std::thread(&XjtechRos2::SaveTemprToFileThread, this);
  }
}

XjtechRos2::~XjtechRos2()
{
  CloseDevices();
  if (SaveTemprCurve)
  {
    SaveLastCurve();
  }

  if (SaveTemprToFile)
  {
    {
      std::lock_guard<std::mutex> lock(mtx);
      stop = true;  // 设置停止标志
    }
    cv.notify_all();  // 通知所有等待的接收者
    if (m_thread.joinable())
    {
      /* code */
      m_thread.join();
    }

    file.close();
  }
}

void XjtechRos2::timer_callback()
{
  if (is_connect)
  {
    ImageProcessThreadFunc();
  }
}

void XjtechRos2::SaveTemprToFileThread()
{
  while (true)
  {
    std::unique_lock<std::mutex> lock(mtx);
    cv.wait(lock, [&] { return !messages.empty() || stop; });  // 等待消息或停止信号
    if (stop)
    {
      if (!messages.empty())
      {
        // 保存messages中的所有数据进csv
        while (true)
        {
          if (!messages.empty())
          {
            /* code */
            // auto msg  = std::move(messages.front());
            SaveTemprToFiles(std::move(messages.front()));
            messages.pop();
            break;
          }
        }
      }
      //线程需要退出，中断循环
      std::cout << "线程退出..." << std::endl;
      break;
    }
    auto msg = std::move(messages.front());
    messages.pop();
    lock.unlock();
    // 保存数据进csv
    SaveTemprToFiles(msg);
  }
}

bool XjtechRos2::SearchDevices()
{
  // clear device list
  g_vecDevices.clear();
  // search device
  XJTech_StartSearch(eDeviceType, SearchDeviceCallbackFunc, this);
  // startSearch
  // X_DEVICE_INFO *pCurDevInfo = new X_DEVICE_INFO();
  std::cout << "searched device : " << g_vecDevices.size() << std::endl;
  // search device failed
  if (g_vecDevices.size() <= 0)
  {
    std::cout << "search device failed." << std::endl;
    return false;
  }
  // else
  // {
  // 	memcpy(pCurDevInfo, g_vecDevices[0], sizeof(X_DEVICE_INFO));
  // }

  return true;
}

bool XjtechRos2::InitDevice()
{
  // std::cout << "xj tech library test app." << std::endl;
  // init device
  iUserId = XJTech_Init(eDeviceType, pDeviceParam);
  std::cout << "XJTech_Init iUserId : " << iUserId << std::endl;
  if (iUserId < 0)
  {
    std::cout << "XJTech_Init failed." << std::endl;
    return false;
  }
  return true;
}

bool XjtechRos2::OpenDevices()
{
  // open device
  int iRet = XJTech_Open(iUserId, g_vecDevices[0], UploadDataCallbackFunc, NULL);
  std::cout << "XJTech_Open iRet : " << iRet << std::endl;
  if (iRet < 0)
  {
    std::cout << "XJTech_Open failed." << std::endl;
    return false;
  }
  is_connect = true;
  return true;
}

void XjtechRos2::CloseDevices()
{
  if (is_connect)
  {
    XJTech_Close(iUserId);
    is_connect = false;
  }
}

void XjtechRos2::SetDevicesParameters()
{
  // set parameters
  XJTech_SetPlatDimmerParam(iUserId, &pPlatHistDimmer);  // 设置平台调光参数
  XJTech_GammaCorrection(iUserId, Gamma);                // Gamma 校正
  XJTech_ChangePalette(iUserId, Palette);                // 设置调色板
  // XJTech_SetMeasureMode(iUserId, E_SURFACE);			  // 设置测温模式
}

void XjtechRos2::DefinedParameters()
{
  // 定义参数及其默认值
  this->declare_parameter("Pixels", 3);
  this->declare_parameter("DeviceType", 2);
  this->declare_parameter("FrameRate", 25);
  this->declare_parameter("SrcWidth", 384);
  this->declare_parameter("SrcHeight", 288);
  this->declare_parameter("DstWidth", 360);
  this->declare_parameter("DstHeight", 288);
  this->declare_parameter("CutHorOffset", 0);
  this->declare_parameter("CutVerOffset", 0);
  this->declare_parameter("Cut", 1);
  this->declare_parameter("AlarmType", 0);
  this->declare_parameter("FlipType", 0);
  this->declare_parameter("PlatThresholdValue", 30);
  this->declare_parameter("MappingMidValue", 145);
  this->declare_parameter("LowerDiscardRatio", 0.01);
  this->declare_parameter("UpperDiscardRatio", 0.01);
  this->declare_parameter("DynamicRangeCoef", 10);
  this->declare_parameter("MappingRange", 220);
  this->declare_parameter("Palette", 3);
  this->declare_parameter("Gamma", 18);
  this->declare_parameter("ImageShow", false);
  this->declare_parameter("TemperatureMeasurementMode", 0);
  this->declare_parameter("StandardTemperature", 50.0);
  this->declare_parameter<int>("Point.x", 0);
  this->declare_parameter<int>("Point.y", 0);
  this->declare_parameter<int>("Region.leftTopPoint.x", 0);
  this->declare_parameter<int>("Region.leftTopPoint.y", 0);
  this->declare_parameter<int>("Region.rightBottomPoint.x", 0);
  this->declare_parameter<int>("Region.rightBottomPoint.y", 0);
  this->declare_parameter("SaveTemprImage", false);
  this->declare_parameter<String>("ImageSavePath", "default");
  this->declare_parameter("ImageSaveInterval", 0);
  this->declare_parameter("SaveTemprCurve", false);
  this->declare_parameter<String>("CurveSavePath", "default");
  this->declare_parameter("SaveTemprToFile", false);
  this->declare_parameter("FileSplitsLines", 100);
  this->declare_parameter<String>("FilesSavePath", "default");
}

bool XjtechRos2::UpdateParamers()
{
  // 尝试从参数服务器读取参数
  // 如果参数不存在，则使用默认值 "default_value"
  this->get_parameter_or("Pixels", imageSize.iPixels, 3);
  this->get_parameter_or("DstWidth", imageSize.iImageWidth, 384);
  this->get_parameter_or("DstHeight", imageSize.iImageHeight, 288);
  this->get_parameter_or("ImageShow", showImage, false);
  TemperatureMeasurementMode = this->get_parameter("TemperatureMeasurementMode").as_int();
  StandardTemperature = this->get_parameter("StandardTemperature").as_double();
  this->get_parameter("Point.x", PointX);
  this->get_parameter("Point.y", PointY);
  this->get_parameter("Region.leftTopPoint.x", LeftTopPointX);
  this->get_parameter("Region.leftTopPoint.y", LeftTopPointY);
  this->get_parameter("Region.rightBottomPoint.x", RightBottomPointX);
  this->get_parameter("Region.rightBottomPoint.y", RightBottomPointY);
  int iDeviceType;
  this->get_parameter_or("DeviceType", iDeviceType, 2);
  eDeviceType = static_cast<X_DEVICE_TYPE>(iDeviceType);  // E_TCP_HISI; // TCP_HISI设备（红外热像仪）
  this->get_parameter_or("FrameRate", pDeviceParam.iFrameRate, 25);       // 图像帧频
  this->get_parameter_or("SrcWidth", pDeviceParam.iSrcWidth, 384);        // 原始图像/温度宽度
  this->get_parameter_or("SrcHeight", pDeviceParam.iSrcHeight, 288);      // 原始图像/温度高度
  this->get_parameter_or("DstWidth", pDeviceParam.iDstWidth, 360);        // 目标图像/温度宽度
  this->get_parameter_or("DstHeight", pDeviceParam.iDstHeight, 288);      // 目标图像/温度高度
  this->get_parameter_or("CutHorOffset", pDeviceParam.iCutHorOffset, 0);  // 数据裁剪水平偏移量
  this->get_parameter_or("CutVerOffset", pDeviceParam.iCutVerOffset, 0);  // 数据裁剪垂直偏移量
  this->get_parameter_or("Cut", pDeviceParam.bCut, 1);                    // 数据裁剪标志
  int iAlarmType;
  this->get_parameter_or("AlarmType", iAlarmType, 0);  // 报警类型
  pDeviceParam.eAlarmType = static_cast<X_ALARM_TYPE>(iAlarmType);
  int iFilpType;
  this->get_parameter_or("FlipType", iFilpType, 0);  // 数据翻转类型
  pDeviceParam.eFlipType = static_cast<X_FLIP_TYPE>(iFilpType);
  this->get_parameter_or("PlatThresholdValue", pPlatHistDimmer.iPlatThresholdValue,
                         30);  // 平台阈值（范围1 ~ 200，默认值100）
  this->get_parameter_or("MappingMidValue", pPlatHistDimmer.iMappingMidValue,
                         145);  // 映射中间值，调整亮度（范围0 ~ 255，默认值128）
  this->get_parameter_or("LowerDiscardRatio", pPlatHistDimmer.dLowerDiscardRatio, 0.01);  // 下抛点率（默认值1%）
  this->get_parameter_or("UpperDiscardRatio", pPlatHistDimmer.dUpperDiscardRatio, 0.01);  // 上抛点率（默认值1%）
  this->get_parameter_or("DynamicRangeCoef", pPlatHistDimmer.iDynamicRangeCoef, 10);      // 动态范围系数
  this->get_parameter_or("MappingRange", pPlatHistDimmer.iMappingRange, 220);             // 映射范围
  this->get_parameter("Palette", Palette);
  this->get_parameter("Gamma", Gamma);
  SaveTemprImage = this->get_parameter("SaveTemprImage").as_bool();
  ImageSaveInterval = this->get_parameter("ImageSaveInterval").as_int();
  ImageSavePath = this->get_parameter("ImageSavePath").as_string();
  SaveTemprCurve = this->get_parameter("SaveTemprCurve").as_bool();
  CurveSavePath = this->get_parameter("CurveSavePath").as_string();
  SaveTemprToFile = this->get_parameter("SaveTemprToFile").as_bool();
  FileSplitsLines = this->get_parameter("FileSplitsLines").as_int();
  FilesSavePath = this->get_parameter("FilesSavePath").as_string();
  return true;
}

void XjtechRos2::PrintParameters()
{
  // 打印读取到的参数值
  RCLCPP_INFO(this->get_logger(), "Pixels: %d", imageSize.iPixels);
  RCLCPP_INFO(this->get_logger(), "DstWidth: %d", imageSize.iImageWidth);
  RCLCPP_INFO(this->get_logger(), "DstHeight: %d", imageSize.iImageHeight);
  RCLCPP_INFO(this->get_logger(), "ImageShow: %d", showImage);
  RCLCPP_INFO(this->get_logger(), "DeviceType: %d", eDeviceType);
  RCLCPP_INFO(this->get_logger(), "FrameRate: %d", pDeviceParam.iFrameRate);
  RCLCPP_INFO(this->get_logger(), "SrcWidth: %d", pDeviceParam.iSrcWidth);
  RCLCPP_INFO(this->get_logger(), "SrcHeight: %d", pDeviceParam.iSrcHeight);
  RCLCPP_INFO(this->get_logger(), "DstWidth: %d", pDeviceParam.iDstWidth);
  RCLCPP_INFO(this->get_logger(), "DstHeight: %d", pDeviceParam.iDstHeight);
  RCLCPP_INFO(this->get_logger(), "CutHorOffset: %d", pDeviceParam.iCutHorOffset);
  RCLCPP_INFO(this->get_logger(), "CutVerOffset: %d", pDeviceParam.iCutVerOffset);
  RCLCPP_INFO(this->get_logger(), "Cut: %d", pDeviceParam.bCut);
  RCLCPP_INFO(this->get_logger(), "AlarmType: %d", pDeviceParam.eAlarmType);
  RCLCPP_INFO(this->get_logger(), "FlipType: %d", pDeviceParam.eFlipType);
  RCLCPP_INFO(this->get_logger(), "PlatThresholdValue: %d", pPlatHistDimmer.iPlatThresholdValue);
  RCLCPP_INFO(this->get_logger(), "MappingMidValue: %d", pPlatHistDimmer.iMappingMidValue);
  RCLCPP_INFO(this->get_logger(), "LowerDiscardRatio: %f", pPlatHistDimmer.dLowerDiscardRatio);
  RCLCPP_INFO(this->get_logger(), "UpperDiscardRatio: %f", pPlatHistDimmer.dUpperDiscardRatio);
  RCLCPP_INFO(this->get_logger(), "DynamicRangeCoef: %d", pPlatHistDimmer.iDynamicRangeCoef);
  RCLCPP_INFO(this->get_logger(), "MappingRange: %d", pPlatHistDimmer.iMappingRange);
  RCLCPP_INFO(this->get_logger(), "Palette: %d", Palette);
  RCLCPP_INFO(this->get_logger(), "Gamma: %d", Gamma);
  RCLCPP_INFO(this->get_logger(), "Mode: %d", TemperatureMeasurementMode);
  RCLCPP_INFO(this->get_logger(), "StandardTemperature: %f", StandardTemperature);
  RCLCPP_INFO(this->get_logger(), "Point X: %d", PointX);
  RCLCPP_INFO(this->get_logger(), "Point Y: %d", PointY);
  RCLCPP_INFO(this->get_logger(), "Left Top Point X: %d", LeftTopPointX);
  RCLCPP_INFO(this->get_logger(), "Left Top Point Y: %d", LeftTopPointY);
  RCLCPP_INFO(this->get_logger(), "Right Bottom Point X: %d", RightBottomPointX);
  RCLCPP_INFO(this->get_logger(), "Right Bottom Point Y: %d", RightBottomPointY);
  RCLCPP_INFO(this->get_logger(), "SaveTemprImage: %d", SaveTemprImage);
  RCLCPP_INFO(this->get_logger(), "ImageSavePath: %s", ImageSavePath);
  RCLCPP_INFO(this->get_logger(), "ImageSaveInterval: %d", ImageSaveInterval);
  RCLCPP_INFO(this->get_logger(), "SaveTemprCurve: %d", SaveTemprCurve);
  RCLCPP_INFO(this->get_logger(), "CurveSavePath: %s", CurveSavePath);
  RCLCPP_INFO(this->get_logger(), "SaveTemprToFile: %d", SaveTemprToFile);
  RCLCPP_INFO(this->get_logger(), "FileSplitsLines: %d", FileSplitsLines);
  RCLCPP_INFO(this->get_logger(), "FilesSavePath: %s", FilesSavePath);
}

void XjtechRos2::PublishPointTempr(int iPointX, int iPointY, float fTempr)
{
  if (iPointX < 0 || iPointY < 0 || fTempr <= 0)
  {
    return;
  }

  auto max_tempr_point = std::make_unique<message_interfaces::msg::PointTemprInfo>();
  max_tempr_point->header.frame_id = "xjtech";
  max_tempr_point->header.stamp = this->now();
  max_tempr_point->x = iPointX;
  max_tempr_point->y = iPointY;
  max_tempr_point->tempr = fTempr;
  temprPointPublisher_->publish(*max_tempr_point);
}

void XjtechRos2::PublishImage(unsigned char* pImage)
{
  if (pImage == nullptr)
  {
    return;
  }

  // 发布实时图像消息
  auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
  img_msg->header.stamp = this->now();
  img_msg->header.frame_id = "bgr_image";
  img_msg->height = imageSize.iImageHeight;
  img_msg->width = imageSize.iImageWidth;
  img_msg->encoding = "bgr8";
  img_msg->is_bigendian = 0;
  img_msg->step = imageSize.iImageWidth * imageSize.iPixels;
  img_msg->data.resize(imageSize.iImageWidth * imageSize.iImageHeight * imageSize.iPixels);
  memcpy(img_msg->data.data(), pImage, imageSize.iImageWidth * imageSize.iImageHeight * imageSize.iPixels);
  imagePublisher_->publish(*img_msg);
}

void XjtechRos2::PrintImage(int iPointX, int iPointY, float fTempr, unsigned char* pImage)
{
  if (iPointX < 0 || iPointY < 0 || fTempr < 0 || pImage == nullptr)
  {
    return;
  }
  // if (showImage)
  // {
  // 通过指针转换为Mat格式
  cv::Mat cv_image(imageSize.iImageHeight, imageSize.iImageWidth, CV_8UC3, pImage);
  // 将 BGR 图像转换为 RGB 图像
  // cv::cvtColor(cv_image, cv_image, cv::COLOR_BGR2RGB);
  // // 在图像左上角显示图像的大小
  // cv::Size image_size = cv_image.size();
  // cv::putText(cv_image, "Width: " + std::to_string(image_size.width) + ", Height: " +
  // std::to_string(image_size.height), 			cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,
  // 255, 255), 1); cv::putText(cv_image, "Width: " + std::to_string(imageSize.iImageWidth) + ", Height: " +
  // std::to_string(imageSize.iImageHeight), 			cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5,
  // cv::Scalar(255, 255, 255), 1); 显示最高温度点或测温点温度
  cv::putText(cv_image,
              "(" + std::to_string(iPointX) + ", " + std::to_string(iPointY) + ") tempr : " + std::to_string(fTempr),
              cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
  // 图片，圆心，半径，颜色，线的粗细（-1表示实心，可用于画点）
  cv::circle(cv_image, cv::Point(iPointX, iPointY), 1, cv::Scalar(0, 0, 255), 3);
  // 如果是区域测温，画出测温区域
  if (TemperatureMeasurementMode == 1)
  {
    // 图片，左上点，右下点，颜色
    cv::rectangle(cv_image, cv::Point2f(LeftTopPointX, LeftTopPointY),
                  cv::Point2f(RightBottomPointX, RightBottomPointY), cv::Scalar(0, 0, 255));
  }
  // 在窗口中实时显示 RGB 图像
  cv::imshow("RGB Image", cv_image);
  cv::waitKey(1);
  // }
}

void XjtechRos2::MakeTemprCurve(float fTempr)
{
  if (fTempr <= 0)
  {
    return;
  }
  // 显示实时测温曲线
  Mat imageT = image.clone();
  Point p;
  p.x = 639;
  p.y = 160 - fTempr + StandardTemperature;
  circle(imageT, p, 1, Scalar(0, 255, 0), -1, 8);
  Mat mt1 = imageT(Rect(1, 0, 639, 320));
  Mat roi = image(Rect(0, 0, 639, 320));
  Mat mask(roi.rows, roi.cols, roi.depth(), Scalar(1));
  mt1.copyTo(roi, mask);
}

void XjtechRos2::PrintTemprCurve(float fTempr)
{
  image_show = image.clone();
  char str[100];
  sprintf(str, "Standard: %.1f, Max: %.1f, Differences:%.1f", StandardTemperature, fTempr,
          fTempr - StandardTemperature);
  putText(image_show, str, Point(5, 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 0), 1, 8);
  line(image_show, Point(0, 160), Point(639, 160), Scalar(0, 0, 255), 1, 8);

  imshow("测温曲线", image_show);
  cv::waitKey(1);
}

void XjtechRos2::SaveImage(unsigned char* pImage)
{
  if (pImage == nullptr)
  {
    return;
  }

  // 获取当前时间
  auto now = std::chrono::system_clock::now();
  //通过不同精度获取相差的毫秒数
  uint64_t dis_millseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() -
                             std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count() * 1000;
  time_t tt = std::chrono::system_clock::to_time_t(now);
  auto time_tm = localtime(&tt);
  char strTime[29] = { 0 };
  sprintf(strTime, "%d-%02d-%02d %02d:%02d:%02d %03d.png", time_tm->tm_year + 1900, time_tm->tm_mon + 1,
          time_tm->tm_mday, time_tm->tm_hour, time_tm->tm_min, time_tm->tm_sec, (int)dis_millseconds);
  E_PIXEL_FORMAT ePixelFormat = PF_RGB;
  SaveImage(strTime, pImage, imageSize.iImageWidth, imageSize.iImageHeight, ePixelFormat);
}

void XjtechRos2::SaveCurve()
{
  auto strTime = GetCurrentTimes();
  //保存图片:
  imwrite(CurveSavePath + strTime + ".png", image);
}

void XjtechRos2::SaveLastCurve()
{
  auto strTime = GetCurrentTimes();
  //保存图片:
  imwrite(CurveSavePath + strTime + ".png", image(cv::Rect(639 - iWaitSaveCurveCount, 0, iWaitSaveCurveCount, 320)));
}

void XjtechRos2::SaveTemprToFiles(std::string sTempr)
{
  if (iCurrentLines >= FileSplitsLines)
  {
    file.close();
    std::string fileName = FilesSavePath + "temprs_" + std::to_string(iCounter) + ".csv";  // 创建下一个切割文件的文件名
    file.open(fileName);                                                                   // 创建新的CSV文件
    iCounter++;                                                                            // 更新切割计数器
    iCurrentLines = 0;                                                                     // 重置行计数器
  }
  file << sTempr << std::endl;
  iCurrentLines++;
}

void XjtechRos2::GetCurrentWorkPath()
{
  char* sz_work_path;
  sz_work_path = getcwd(NULL, 0);

  std::string str_work_path = std::string(sz_work_path);

  str_work_path = str_work_path.substr(0, str_work_path.find_last_of("\\", str_work_path.length()));

  m_strSaveDir = str_work_path + "/" + m_strSaveDir + "/";
}

void XjtechRos2::CreateSaveDirectory(const char* sFilePath)
{
  if (0 != access(sFilePath, 0))
  {
    mkdir(sFilePath, S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
  }
}

std::string XjtechRos2::GetCurrentTimes()
{
  // 获取当前时间
  auto now = std::chrono::system_clock::now();
  //通过不同精度获取相差的毫秒数
  uint64_t dis_millseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() -
                             std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count() * 1000;
  time_t tt = std::chrono::system_clock::to_time_t(now);
  auto time_tm = localtime(&tt);
  char strTime[25] = { 0 };
  sprintf(strTime, "%d-%02d-%02d %02d:%02d:%02d %03d", time_tm->tm_year + 1900, time_tm->tm_mon + 1, time_tm->tm_mday,
          time_tm->tm_hour, time_tm->tm_min, time_tm->tm_sec, (int)dis_millseconds);
  return strTime;
}

bool XjtechRos2::SaveImage(const char* szFileName, unsigned char* pImageData, int iWdith, int iHeight,
                           E_PIXEL_FORMAT emPixelFormat)
{
  if (NULL == szFileName || NULL == pImageData || iWdith <= 0 || iHeight <= 0 || emPixelFormat <= PF_INVALID ||
      emPixelFormat >= PF_COUNT)
  {
    return false;
  }

  std::string str_file_path = ImageSavePath + std::string(szFileName);

  std::cout << str_file_path << std::endl;
  int i_pixel_size = 3;
  unsigned short us_bit_count = 24;

  switch (emPixelFormat)
  {
    case PF_RGB:
      i_pixel_size = 3;
      us_bit_count = 24;
      break;
    case PF_GRAY:
      i_pixel_size = 1;
      us_bit_count = 1;
      break;
    default:
      break;
  }

  int i_real_width = iWdith;
  int i_image_size = i_real_width * iHeight * i_pixel_size;
  unsigned char* p_image_data = pImageData;

  if (0 != iWdith % 4)
  {
    i_real_width = (int)ceil((double)iWdith / 4) * 4;
    i_image_size = i_real_width * iWdith * i_pixel_size;
    p_image_data = new unsigned char[i_image_size];

    memset(p_image_data, 0, sizeof(unsigned char) * i_image_size);

    int i_src_image_step = iWdith * i_pixel_size;
    int i_dst_image_step = i_real_width * i_pixel_size;

    for (int i = 0; i < iHeight; i++)
    {
      memcpy(p_image_data + i * i_dst_image_step, pImageData + i * i_src_image_step, i_src_image_step);
    }
  }

  XBitmapFileHeader st_bmp_file_header;
  st_bmp_file_header.bfSize =
      sizeof(unsigned short) + sizeof(XBitmapFileHeader) + sizeof(XBitmapInfoHeader) + i_image_size;
  st_bmp_file_header.bfReserved1 = 0;
  st_bmp_file_header.bfReserved2 = 0;
  st_bmp_file_header.bfOffBits = 0x36;

  XBitmapInfoHeader st_bmp_info_header;
  st_bmp_info_header.biSize = sizeof(XBitmapInfoHeader);
  st_bmp_info_header.biWidth = i_real_width;
  st_bmp_info_header.biHeight = iHeight;
  st_bmp_info_header.biPlanes = 1;
  st_bmp_info_header.biBitCount = us_bit_count;
  st_bmp_info_header.biCompression = 0;
  st_bmp_info_header.biSizeImage = 0;
  st_bmp_info_header.biXPelsPerMeter = 5000;
  st_bmp_info_header.biYPelsPerMeter = 5000;
  st_bmp_info_header.biClrUsed = 0;
  st_bmp_info_header.biClrImportant = 0;

  FILE* p_file = NULL;
  p_file = fopen(str_file_path.c_str(), "wb");

  if (NULL == p_file)
  {
    return false;
  }

  unsigned short bfType = 0x4D42;
  fwrite(&bfType, sizeof(bfType), 1, p_file);
  fwrite(&st_bmp_file_header, sizeof(st_bmp_file_header), 1, p_file);
  fwrite(&st_bmp_info_header, sizeof(st_bmp_info_header), 1, p_file);
  fwrite(p_image_data, i_image_size, 1, p_file);

  fclose(p_file);

  if (NULL != p_image_data && p_image_data != pImageData)
  {
    delete[] p_image_data;
    p_image_data = NULL;
  }

  return true;
}

void XjtechRos2::SearchDeviceCallbackFunc(X_DEVICE_INFO* pDeviceInfo, void* pUserData)
{
  X_DEVICE_INFO* pDevInfo = new X_DEVICE_INFO();

  memcpy(pDevInfo, pDeviceInfo, sizeof(X_DEVICE_INFO));

  std::cout << "search a device : " << pDevInfo->szIpAddr << std::endl;

  g_vecDevices.push_back(pDevInfo);
}

void XjtechRos2::UploadDataCallbackFunc(unsigned char* pImage,                                     // 图片
                                        X_TEMPR_INFO* pMaxTemprInfo, X_TEMPR_INFO* pMinTemprInfo,  // 最大最小温度
                                        X_ALARM_LINDED_AREA* pAlarmLinkedArea, int iAlarmAreaCount,
                                        unsigned char* pAlarmMask, void* pUserData)
{
  IrData* pData = new IrData(imageSize.iImageWidth, imageSize.iImageHeight, imageSize.iPixels);
  // 当前为高温模式，仅传输图像和最高温度点
  memcpy(pData->pImage, pImage,
         imageSize.iImageWidth * imageSize.iImageHeight * imageSize.iPixels * sizeof(unsigned char));
  memcpy(pData->pMaxTemprInfo, pMaxTemprInfo, sizeof(X_TEMPR_INFO));
  // memcpy(pData->pMinTemprInfo, pMinTemprInfo, sizeof(X_TEMPR_INFO));

  g_lstIrData.push(pData);
}

void XjtechRos2::ImageProcessThreadFunc()
{
  if (!g_lstIrData.empty() && is_connect)
  {
    // 点温度结构体
    auto pTemprInfo = std::make_unique<X_TEMPR_INFO>();
    // 一帧温度图像数据
    IrData* pData = g_lstIrData.front();

    // 发布实时图像消息
    PublishImage(pData->pImage);
    // 按时间戳保存测温图像
    if (SaveTemprImage)
    {
      if (ImageSaveInterval != 0)
      {
        iWaitSaveImageCount++;
        if (iWaitSaveImageCount == ImageSaveInterval)
        {
          SaveImage(pData->pImage);
          iWaitSaveImageCount = -1;
        }
      }
      else
      {
        SaveImage(pData->pImage);
      }
    }

    // 获取测温点温度
    if (TemperatureMeasurementMode == 0)  // 全局温度最高点
    {
      pTemprInfo->iX = pData->pMaxTemprInfo->iX;
      pTemprInfo->iY = pData->pMaxTemprInfo->iY;
      pTemprInfo->iTempr = pData->pMaxTemprInfo->iTempr * 1.0F;
    }
    else if (TemperatureMeasurementMode == 1)  // 区域温度最高点 （推荐使用）
    {
      auto pRect = std::make_unique<X_RECT>();
      pRect->iLeft = LeftTopPointX;        // 左上角X坐标
      pRect->iTop = LeftTopPointY;         // 左上角Y坐标
      pRect->iRight = RightBottomPointX;   // 右下角X坐标
      pRect->iBottom = RightBottomPointY;  // 右下角Y坐标
      // 获取固定矩形区域温度
      XJTech_GetRectangleTemptr(iUserId, pRect.get(), pTemprInfo.get());
    }
    else if (TemperatureMeasurementMode == 2)  // 点温度
    {
      // 获取固定点温度
      auto point_tempr = XJTech_GetTemprByPoint(iUserId, PointX, PointY);
      pTemprInfo->iX = PointX;
      pTemprInfo->iY = PointX;
      pTemprInfo->iTempr = point_tempr * 10.0F;
    }
    // 发布测温点温度
    PublishPointTempr(pTemprInfo->iX, pTemprInfo->iY, pTemprInfo->iTempr * 1.0F);

    // 绘制测温曲线
    if (showImage || SaveTemprCurve)
    {
      MakeTemprCurve(pTemprInfo->iTempr * 1.0F);
    }

    // 显示测温图像
    if (showImage)
    {
      PrintImage(pTemprInfo->iX, pTemprInfo->iY, pTemprInfo->iTempr * 1.0F, pData->pImage);
      PrintTemprCurve(pTemprInfo->iTempr * 1.0F);
    }
    // 保存测温曲线
    if (SaveTemprCurve)
    {
      iWaitSaveCurveCount++;
      if (iWaitSaveCurveCount == 640)
      {
        SaveCurve();
        iWaitSaveCurveCount = 0;
      }
    }
    if (SaveTemprToFile)
    // 保存测温数据到csv
    {
      std::string message = GetCurrentTimes() + "\t" + std::to_string(pTemprInfo->iTempr);
      std::unique_lock<std::mutex> lock(mtx);
      messages.push(message);  // 向队列中添加消息
      cv.notify_one();         // 通知等待的接收者
    }

    g_lstIrData.pop();
  }
}
