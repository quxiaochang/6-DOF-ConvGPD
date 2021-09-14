#pragma once
/************************************************************************/
/* 功能：LMI相机点云获取                                    */
/* 作者：瞿孝昌                                                    */
/* 版本：v1.0                                                      */
/************************************************************************/

#ifndef PC_COLLECT_H_
#define PC_COLLECT_H_

//USER
#include "pc_process.h"
//API
#include <GoSdk/GoSdk.h>
//SYSTEM
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

class SCAN {
public:
#define RECEIVE_TIMEOUT 20000000      //单位:微秒，10-6秒
#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data. 
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.  
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.
#define SENSOR_IP               "192.168.2.10"      //192.168.1.10

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)

	/*连接相机*/
	bool ConnectCamera();
    pcl::PointCloud<UserPointType> get_cloud_original();
private:
    kAssembly api = kNULL;
    kStatus status;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    GoDataSet dataset = kNULL;
    unsigned int i, j;
    kIpAddress ipAddress;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<UserPointType> cloud_original_;
};

#endif

