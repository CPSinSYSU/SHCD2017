#ifndef __VELODYNE_SCANNER_H__
#define __VELODYNE_SCANNER_H__

/**************************************************************************
 *
 *	对当前系统中的激光雷达帧(scan)数据进行各种操作
 *
 ***************************************************************************/

#include "velodyneDataStruct.h"

#define SCAN_BUFFER_SIZE		4
#define HALF_SCAN_BUFFER_SIZE	2

VelodyneDataStruct* getScanRaw(void);
VelodyneDataStruct* getScanRawForDraw(void);

#endif
