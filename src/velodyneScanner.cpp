/**************************************************************************
 *
 *	对当前系统中的激光雷达帧(scan)数据进行各种操作
 *
 ***************************************************************************/

#include "velodyneScanner.h"
#include "velodyneThread.h"

#include "string.h"         // for memcpy
#include <stdio.h>


static void TransformFromRaw(VelodyneDataRaw* scanObj, VelodyneDataStruct* calObj){
    calObj->shots.resize(scanObj->shots.size());
    for(unsigned shot = 0; shot < scanObj->shots.size(); shot++){
        for(unsigned laser = 0; laser < VELODYNE_NUM_BEAMS_IN_ONE_BLOCK; laser++){
            /*printf("sizeof(VelodyneDataRaw::xpoint_t)=%d\n", sizeof(VelodyneDataRaw::xpoint_t));
            printf("sizeof(VelodyneDataStruct::xpoint_t)=%d\n", sizeof(VelodyneDataStruct::xpoint_t));
            */
            memcpy(&(calObj->shots[shot].pt[laser]), &(scanObj->shots[shot].pt[laser]),
                   sizeof(VelodyneDataRaw::xpoint_t));
//            calObj->shots[shot].pt[laser].x = scanObj->shots[shot].pt[laser].x;
//            printf("calObj->shots[shot].pt[laser].x=%d\n", calObj->shots[shot].pt[laser].x);
//
//            calObj->shots[shot].pt[laser].y = scanObj->shots[shot].pt[laser].y;
//            calObj->shots[shot].pt[laser].z = scanObj->shots[shot].pt[laser].z;
//
//            calObj->shots[shot].pt[laser].circleID = scanObj->shots[shot].pt[laser].circleID;
//            calObj->shots[shot].pt[laser].shotID = scanObj->shots[shot].pt[laser].shotID;
//
//            calObj->shots[shot].pt[laser].rad = scanObj->shots[shot].pt[laser].rad;
//            calObj->shots[shot].pt[laser].tan_theta = scanObj->shots[shot].pt[laser].tan_theta;
//
//            calObj->shots[shot].pt[laser].r = scanObj->shots[shot].pt[laser].r;
//            calObj->shots[shot].pt[laser].g = scanObj->shots[shot].pt[laser].g;
//            calObj->shots[shot].pt[laser].b = scanObj->shots[shot].pt[laser].b;
//            calObj->shots[shot].pt[laser].i = scanObj->shots[shot].pt[laser].i;
//
//            calObj->shots[shot].pt[laser].scanline_drtZ = scanObj->shots[shot].pt[laser].scanline_drtZ;
//            calObj->shots[shot].pt[laser].scanline_drtRad = scanObj->shots[shot].pt[laser].scanline_drtRad;
//            calObj->shots[shot].pt[laser].scanline_drtI = scanObj->shots[shot].pt[laser].scanline_drtI;
//
//            calObj->shots[shot].pt[laser].scanline_countZ = scanObj->shots[shot].pt[laser].scanline_countZ;
//            calObj->shots[shot].pt[laser].scanline_countD = scanObj->shots[shot].pt[laser].scanline_countD;
//
//            calObj->shots[shot].pt[laser].point_type = scanObj->shots[shot].pt[laser].point_type;
//            calObj->shots[shot].pt[laser].objectID = scanObj->shots[shot].pt[laser].objectID;
        }
    }
}

VelodyneDataStruct* getScanRaw(void)
{
    VelodyneDataStruct* calObj = NULL;
    if (g_scanBufferSize > 0)
    {
        calObj = new VelodyneDataStruct();
        if (g_scanBufferSize > 1)
        {
            // 丢弃帧
            if (g_scanBufferSize > HALF_SCAN_BUFFER_SIZE)
            {
                g_scanBufferReadIdx = (g_scanBufferReadIdx + 2) % SCAN_BUFFER_SIZE;
                g_scanBufferSize -= 2;
                //g_scanDrop++;
            }
            else
            {
                g_scanBufferReadIdx = (g_scanBufferReadIdx + 1) % SCAN_BUFFER_SIZE;
                g_scanBufferSize--;
            }
            //g_scanBuffer[g_scanBufferReadIdx].isTrackerHandled = false;
        }
        TransformFromRaw(&(g_scanBuffer[g_scanBufferReadIdx]), calObj);
    }
    return calObj;
}

VelodyneDataStruct* getScanRawForDraw(void)
{
    VelodyneDataStruct* calObj = NULL;
    if (g_scanBufferSize > 0)
    {
        calObj = new VelodyneDataStruct();
        TransformFromRaw(&(g_scanBuffer[g_scanBufferReadIdx]), calObj);
    }
    return calObj;
}


