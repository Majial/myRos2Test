#include "MyLog.h"
#include <stdint.h>
#include <ctime>
#include <chrono>
#include <memory>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

string MyLog::timeToString()
{
  // 获取当前时间
  auto now = std::chrono::system_clock::now();
  //通过不同精度获取相差的毫秒数
  uint64_t dis_millseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() -
                             std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count() * 1000;
  time_t tt = std::chrono::system_clock::to_time_t(now);
  auto time_tm = localtime(&tt);
  char strTime[25] = { 0 };
  sprintf(strTime, "%d-%02d-%02d_%02d:%02d:%02d_%03d", time_tm->tm_year + 1900, time_tm->tm_mon + 1, time_tm->tm_mday,
          time_tm->tm_hour, time_tm->tm_min, time_tm->tm_sec, (int)dis_millseconds);
  return strTime;
}

const string MyLog::logFolderPath = "log2/";
const string MyLog::folderTime = timeToString() + "/";

void MyLog::init(string path, bool flag)
{
  writeLog = flag;
  if (writeLog)
  {
    string folderPath = logFilePath + logFolderPath + folderTime;
    fullPath = folderPath + fileName + ".log";
    int state = access(folderPath.c_str(), R_OK | W_OK);  // 头文件 #include <unistd.h>
    if (state != 0)
    {
      mkdir(folderPath.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    }
    ofs.open(fullPath, std::ofstream::app);
    debugLog(fileName + " model full path: " + fullPath);
  }
}

/*
    debug_log("this is a(n) debug log"); 1
    info_log("this is a(n) info log");2
    warn_log("this is a(n) warn log");3
    err_log("this is a(n) error log");4
    fatal_log("this is a(n) fatal log");5
*/

void MyLog::log(string content)
{
  if (writeLog)
  {
    ofs << content;
  }
}

void MyLog::debugLog(string content)
{
  string temp = "[D " + fileName + timeToString() + "]" + content + "\n";
  if (logLever <= 1)
  {
    printf("%s", temp.c_str());
  }
  log(temp);
}

void MyLog::infoLog(string content)
{
  string temp = "[I " + fileName + timeToString() + "]" + content + "\n";
  if (logLever <= 2)
  {
    printf("%s", temp.c_str());
  }
  log(temp);
}

void MyLog::warnLog(string content)
{
  string temp = "[W " + fileName + timeToString() + "]" + content + "\n";
  if (logLever <= 3)
  {
    printf("%s", temp.c_str());
  }
  log(temp);
}

void MyLog::errLog(string content)
{
  string temp = "[E " + fileName + timeToString() + "]" + content + "\n";
  if (logLever <= 4)
  {
    printf("%s", temp.c_str());
  }
  log(temp);
}

void MyLog::fatalLog(string content)
{
  string temp = "[F " + fileName + timeToString() + "]" + content + "\n";
  if (logLever <= 5)
  {
    printf("%s", temp.c_str());
  }
  log(temp);
}

MyLog::MyLog(string logfileName) : fileName(logfileName)
{
}

MyLog::~MyLog()
{
  if (writeLog)
  {
    ofs.close();
  }
}
