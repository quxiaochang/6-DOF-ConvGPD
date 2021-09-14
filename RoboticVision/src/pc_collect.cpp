#pragma warning(disable:4996)
#include "pc_collect.h"

bool SCAN::ConnectCamera(){
    cloud_original_.clear();
    cloud_original_.is_dense = false;

    // construct Gocator API Library
    if ((status = GoSdk_Construct(&api)) != kOK)
    {
        printf("Error: GoSdk_Construct:%d\n", status);
        return 0;
    }

    // construct GoSystem object
    if ((status = GoSystem_Construct(&system, kNULL)) != kOK)
    {
        printf("Error: GoSystem_Construct:%d\n", status);
        return 0;
    }

    // Parse IP address into address data structure
    kIpAddress_Parse(&ipAddress, SENSOR_IP);

    // obtain GoSensor object by sensor IP address
    if ((status = GoSystem_FindSensorByIpAddress(system, &ipAddress, &sensor)) != kOK)
    {
        printf("Error: GoSystem_FindSensor:%d\n", status);
        return 0;
    }

    // create connection to GoSensor object
    if ((status = GoSensor_Connect(sensor)) != kOK)
    {
        printf("Error: GoSensor_Connect:%d\n", status);
        return 0;
    }

    // enable sensor data channel
    if ((status = GoSystem_EnableData(system, kTRUE)) != kOK)
    {
        printf("Error: GoSensor_EnableData:%d\n", status);
        return 0;
    }

    // start Gocator sensor
    if ((status = GoSystem_Start(system)) != kOK)
    {
        printf("Error: GoSystem_Start:%d\n", status);
        return 0;
    }

    printf("Waiting for Whole Part data in LMI...\n\n");

    if (GoSystem_ReceiveData(system, &dataset, RECEIVE_TIMEOUT) == kOK)
    {
        short int* height_map_memory = NULL;
        unsigned char* intensity_image_memory = NULL;
        k32u surfaceBufferHeight = 0;

        printf("Dataset count: %u\n", (k32u)GoDataSet_Count(dataset));

        // each result can have multiple data items
        // loop through all items in result message
        float x(0), y(0), z(0), n(0);
        for (i = 0; i < GoDataSet_Count(dataset); ++i)
        {
            GoDataMsg dataObj = GoDataSet_At(dataset, i);

            switch (GoDataMsg_Type(dataObj))
            {
            case GO_DATA_MESSAGE_TYPE_STAMP:
            {
                GoStampMsg stampMsg = dataObj;
                printf("  Stamp Message batch count: %u\n\n", (k32u)GoStampMsg_Count(stampMsg));

                for (j = 0; j < GoStampMsg_Count(stampMsg); j++)
                {
                    GoStamp* stamp = GoStampMsg_At(stampMsg, j);
                    printf("  Timestamp: %llu\n", stamp->timestamp);
                    printf("  Encoder position at leading edge: %lld\n", stamp->encoder);
                    printf("  Frame index: %llu\n", stamp->frameIndex);
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE:   //均匀点
            {
                GoSurfaceMsg surfaceMsg = dataObj;
                unsigned int rowIdx, colIdx;

                double XResolution = NM_TO_MM(GoSurfaceMsg_XResolution(surfaceMsg));
                double YResolution = NM_TO_MM(GoSurfaceMsg_YResolution(surfaceMsg));
                double ZResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(surfaceMsg));
                double XOffset = UM_TO_MM(GoSurfaceMsg_XOffset(surfaceMsg));
                double YOffset = UM_TO_MM(GoSurfaceMsg_YOffset(surfaceMsg));
                double ZOffset = UM_TO_MM(GoSurfaceMsg_ZOffset(surfaceMsg));

                printf("  Surface data width: %lu\n", (k32u)GoSurfaceMsg_Width(surfaceMsg));
                printf("  Surface data length: %lu\n", (k32u)GoSurfaceMsg_Length(surfaceMsg));

                //allocate memory if needed
                //if (surfaceBuffer == NULL)
                //{
                //    surfaceBuffer = (ProfilePoint**)malloc(GoSurfaceMsg_Length(surfaceMsg) * sizeof(ProfilePoint*));

                //    for (j = 0; j < GoSurfaceMsg_Length(surfaceMsg); j++)
                //    {
                //        surfaceBuffer[j] = (ProfilePoint*)malloc(GoSurfaceMsg_Width(surfaceMsg) * sizeof(ProfilePoint));
                //    }

                //    surfaceBufferHeight = (k32u)GoSurfaceMsg_Length(surfaceMsg);
                //}

                for (rowIdx = 0; rowIdx < GoSurfaceMsg_Length(surfaceMsg); rowIdx++)
                {
                    k16s* data = GoSurfaceMsg_RowAt(surfaceMsg, rowIdx);

                    for (colIdx = 0; colIdx < GoSurfaceMsg_Width(surfaceMsg); colIdx++)
                    {
                        // gocator transmits range data as 16-bit signed integers
                        // to translate 16-bit range data to engineering units, the calculation for each point is: 
                        //          X: XOffset + columnIndex * XResolution 
                        //          Y: YOffset + rowIndex * YResolution
                        //          Z: ZOffset + height_map[rowIndex][columnIndex] * ZResolution

                        //surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * colIdx;
                        //surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * rowIdx;
                        x = XOffset + XResolution * colIdx;
                        y = YOffset + YResolution * rowIdx;

                        if (data[colIdx] != INVALID_RANGE_16BIT)   //有效点
                        {
                            //surfaceBuffer[rowIdx][colIdx].z = ZOffset + ZResolution * data[colIdx];
                            z = ZOffset + ZResolution * data[colIdx];

                            //std::cout << "xyz:" << surfaceBuffer[rowIdx][colIdx].x << " " << surfaceBuffer[rowIdx][colIdx].y << " " << surfaceBuffer[rowIdx][colIdx].z << std::endl;
                            cloud_original_.points.emplace_back(pcl::PointXYZ(x, y, z));

                        }
                        else
                        {
                            //surfaceBuffer[rowIdx][colIdx].z = INVALID_RANGE_DOUBLE;  //无效点
                            continue;
                        }
                    }
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD:  //非均匀点
            {
                GoSurfacePointCloudMsg surfacePointCloudMsg = dataObj;
                unsigned int rowIdx, colIdx;

                double XResolution = NM_TO_MM(GoSurfacePointCloudMsg_XResolution(surfacePointCloudMsg));
                double YResolution = NM_TO_MM(GoSurfacePointCloudMsg_YResolution(surfacePointCloudMsg));
                double ZResolution = NM_TO_MM(GoSurfacePointCloudMsg_ZResolution(surfacePointCloudMsg));
                double XOffset = UM_TO_MM(GoSurfacePointCloudMsg_XOffset(surfacePointCloudMsg));
                double YOffset = UM_TO_MM(GoSurfacePointCloudMsg_YOffset(surfacePointCloudMsg));
                double ZOffset = UM_TO_MM(GoSurfacePointCloudMsg_ZOffset(surfacePointCloudMsg));

                printf("  Surface Point Cloud data width: %lu\n", (k32u)GoSurfacePointCloudMsg_Width(surfacePointCloudMsg));  //实际点云个数
                printf("  Surface Point Cloud data length: %lu\n", (k32u)GoSurfacePointCloudMsg_Length(surfacePointCloudMsg));

                //allocate memory if needed
 /*               if (surfaceBuffer == NULL)
                {
                    surfaceBuffer = (ProfilePoint**)malloc(GoSurfacePointCloudMsg_Length(surfacePointCloudMsg) * sizeof(ProfilePoint*));

                    for (j = 0; j < GoSurfacePointCloudMsg_Length(surfacePointCloudMsg); j++)
                    {
                        surfaceBuffer[j] = (ProfilePoint*)malloc(GoSurfacePointCloudMsg_Width(surfacePointCloudMsg) * sizeof(ProfilePoint));
                    }

                    surfaceBufferHeight = (k32u)GoSurfacePointCloudMsg_Length(surfacePointCloudMsg);
                }*/


                for (rowIdx = 0; rowIdx < GoSurfacePointCloudMsg_Length(surfacePointCloudMsg); rowIdx++)
                {
                    kPoint3d16s* data = GoSurfacePointCloudMsg_RowAt(surfacePointCloudMsg, rowIdx);

                    for (colIdx = 0; colIdx < GoSurfacePointCloudMsg_Width(surfacePointCloudMsg); colIdx++)
                    {
                        //surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * data[colIdx].x;
                        //surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * data[colIdx].y;
                        x = XOffset + XResolution * data[colIdx].x;
                        y = YOffset + YResolution * data[colIdx].y;

                        if (data[colIdx].z != INVALID_RANGE_16BIT)  //有效点
                        {
                            //surfaceBuffer[rowIdx][colIdx].z = ZOffset + ZResolution * data[colIdx].z;
                            z = ZOffset + ZResolution * data[colIdx].z;

                            //std::cout << "xyz:" << surfaceBuffer[rowIdx][colIdx].x << " " << surfaceBuffer[rowIdx][colIdx].y << " " << surfaceBuffer[rowIdx][colIdx].z << std::endl;
                            cloud_original_.points.emplace_back(pcl::PointXYZ(x, y, z));
                        }
                        else
                        {
                            //surfaceBuffer[rowIdx][colIdx].z = INVALID_RANGE_DOUBLE;
                            continue;
                        }
                    }
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY:
            {
                GoSurfaceIntensityMsg surfaceIntMsg = dataObj;
                unsigned int rowIdx, colIdx;
                double XResolution = NM_TO_MM(GoSurfaceIntensityMsg_XResolution(surfaceIntMsg));
                double YResolution = NM_TO_MM(GoSurfaceIntensityMsg_YResolution(surfaceIntMsg));
                double XOffset = UM_TO_MM(GoSurfaceIntensityMsg_XOffset(surfaceIntMsg));
                double YOffset = UM_TO_MM(GoSurfaceIntensityMsg_YOffset(surfaceIntMsg));

                printf("  Surface intensity width: %lu\n", (k32u)GoSurfaceIntensityMsg_Width(surfaceIntMsg));
                printf("  Surface intensity height: %lu\n", (k32u)GoSurfaceIntensityMsg_Length(surfaceIntMsg));

                //allocate memory if needed
                //if (surfaceBuffer == NULL)
                //{
                //    surfaceBuffer = (ProfilePoint**)malloc(GoSurfaceIntensityMsg_Length(surfaceIntMsg) * sizeof(ProfilePoint*));

                //    for (j = 0; j < GoSurfaceIntensityMsg_Length(surfaceIntMsg); j++)
                //    {
                //        surfaceBuffer[j] = (ProfilePoint*)malloc(GoSurfaceIntensityMsg_Width(surfaceIntMsg) * sizeof(ProfilePoint));
                //    }

                //    surfaceBufferHeight = (k32u)GoSurfaceIntensityMsg_Length(surfaceIntMsg);
                //}

                for (rowIdx = 0; rowIdx < GoSurfaceIntensityMsg_Length(surfaceIntMsg); rowIdx++)
                {
                    k8u* data = GoSurfaceIntensityMsg_RowAt(surfaceIntMsg, rowIdx);

                    // gocator transmits intensity data as an 8-bit grayscale image of identical width and height as the corresponding height map
                    for (colIdx = 0; colIdx < GoSurfaceIntensityMsg_Width(surfaceIntMsg); colIdx++)
                    {
                        //surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * colIdx;
                        //surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * rowIdx;
                        //surfaceBuffer[rowIdx][colIdx].intensity = data[colIdx];

                        x = XOffset + XResolution * colIdx;
                        y = YOffset + YResolution * rowIdx;
                        n = data[colIdx];
                    }

                }
            }
            break;
            }
        }
        //free memory arrays
        //if (1)
        //{
        //    unsigned int i;
        //    for (i = 0; i < surfaceBufferHeight; i++)
        //    {
        //        free(surfaceBuffer[i]);
        //    }
        //}
    }
    else
    {
        printf("Error: No data received during the waiting period\n");
    }

    GoDestroy(dataset);
    // stop Gocator sensor
    if ((status = GoSystem_Stop(system)) != kOK)
    {
        printf("Error: GoSystem_Stop:%d\n", status);
        return 0;
    }

    // destroy handles  
    GoDestroy(system);
    GoDestroy(api);

    cloud_original_.height = 1;
    cloud_original_.width = cloud_original_.size();
    cloud_original_.resize(cloud_original_.size());

    return true;
}

pcl::PointCloud<UserPointType> SCAN::get_cloud_original() {
    return cloud_original_;
}


