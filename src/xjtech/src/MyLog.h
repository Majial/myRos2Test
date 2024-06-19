#pragma once

#include <iostream>
#include <fstream>

using namespace std;

class MyLog
{
private:
    //default Path
    string logFilePath;
    const static string logFolderPath;
    const static string folderTime;
    string fileName;
    string fullPath;
    bool writeLog=true;
    int logLever = 0;
    std::ofstream ofs;
    void log(string content);

    /* data */
public:
    MyLog(string logfileName);
    ~MyLog();

    static string timeToString();
    void init(string path, bool flag);
    void debugLog(string content);
    void fatalLog(string content);
    void errLog(string content);
    void warnLog(string content);
    void infoLog(string content);
};
