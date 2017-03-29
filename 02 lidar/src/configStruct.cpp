/**************************************************************************
 *
 *	全局配置
 *	配置文件 ./velodyne.ini
 *	配置结构体
 *		CfgVeloView: 总体配置项结构体定义
 *			CfgCircleItem: 每个circle的配置选项定义
 *			CfgPlaneDetect: 平面检测配置选项定义
 *			CfgGlobal: 全局配置项定义
 *			CfgShotOffset:
 *			CfgInterSection: 路口算法参数
 *			CfgIntersection: 路口算法参数(新的路口算法)
 *			CfgIgnoreCircleWhenProjection:
 *			CfgCarShapeParameter:
 *	加载参数/保存参数 Get/WritePrivateProfileInt(Window API)
 *
 ***************************************************************************/

#include "configStruct.h"
#include <stdio.h>
#ifdef WIN32
#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#include <winbase.h>
#else
#include <glib.h>
#endif

#include <stdlib.h>

static CfgGlobal_t g_DefaultGlobalCfg =
{
//	1,			//ColorMode
//	0,			//ShotShowFrom
//	0,			//ShotShowNum
//	0,			//ShotCalcFrom
//	0,			//ShotCalcNum
//	4500,		//MaxRad
//	1500,		//minRad
//	-225,		//GroundZ
//	300,		//CarFrontX
//	5.0,		//ThresholdDeltaI
//	0.01,		//DeltaIColorFactor
//	0.82,		//ThresholdMinRad
//	50,			//ThresholdMinHollow
//	150,		//ThresholdMaxHollow
//	0, 0, 0, 0, 0, 0, 0, 0,
//	0.4,		// 扫描线找路沿点deltaZ的下限
//	1.0,		// 扫描线找路沿点deltaZ的上限
//	0,
//	64,
//	2000,
//	1,			// MinLaneNum
//	210			// MinLaneWidht
};

static CfgPlaneDetect_t g_DefaultCfgPlaneDetect =
{
//	0,
//	1,
//	0,
//	3000,		// maxrad
//	8.0,
//	0.1,
//	50,
//	360,		// ColumnSize
//	30,
//	110,
//	110,
//	0,			// 本车后方单元格的行数
//	4.5,		// GridThresholdGroundDetect
//	5.0,
//	50,			// DeleteSingleRedGridMaxRow
//	1			// DeleteSingleRedGridMaxNum
};

static CfgNegativeObstacle_t g_DefaultCfgNegativeObstacle =
{

};

//static CfgInterSection_t g_DefaultCfgInterSection = {
//	0,
//	0,
//	13,
//	13,
//	0,
//	0,
//	0,
//	{ 1, 3, 5, 8, 10, 12 },
//	{ 0, 0, 0, 0, 0, 0 },
//	{ 0, 0, 0, 0, 0, 0 },
//	20,
//	12,
//	9
//};

//static CfgIntersection_t g_DefaultCfgIntersection = {
//	15.0,
//	15.0,
//	2.8,
//	9.0,
//	1.0,
//	1.0,
//	43,
//
//	2,
//
//	15,
//	37,
//	15,
//
//	165,
//	0.6,
//	15,
//	7,
//	18,
//	15,
//	4,
//	30,				// 射线最大长度
//	2				// 射线宽度
//};

//static CfgShotOffset_t g_DefaultCfgShotOffset = {
//	{
//		-15, 3, 27, 45, -46, -30, -6, 12,
//		36, 54, -54, -36, -12, 4, 30, 45,
//		-45, -30, -6, 12, 33, 50, -51, -36,
//		-12, 2, 27, 44, -45, -28, -6, 9,
//		-9, 2, 18, 27, -27, -20, -3, 6,
//		24, 33, -33, -24, -7, 0, 18, 27,
//		-27, -19, -1, 6, 22, 30, -32, -24,
//		-9, 1, 18, 27, -27, -19, -3, 6
//	}
//};

static CfgIgnoreCircleWhenProjection_t g_DefaultCfgIgnoreCircleWhenProjection =
{
//	{
//		0, 1, 2, 4, 5, 6, 7, 8, 9, 10,
//		12, 14, 16, 18, 23, 27, 29,
//		-1, -1, -1, -1, -1, -1, -1,
//		-1, -1, -1, -1, -1, -1, -1
//	}
};

//static CfgCarShapeParameter_t g_DefaultCfgCarShapeParameter = {
//	180,
//	500,
//	120,
//	330,
//	4
//};

static CfgCircleItem_t g_DefaultCfgCircleItems[CIRCLE_NUM] =
{
//	{ 1, 0, 497.0, 2.5, 0.07, 0.4, 0.07 },		//0
//	{ 1, 0, 509.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 522.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 535.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 548.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 562.0, 2.5, 0.07, 0.4, 0.07 },		//5
//	{ 1, 0, 577.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 592.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 608.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 625.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 643.0, 2.5, 0.07, 0.4, 0.07 },		//10
//	{ 1, 0, 661.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 681.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 701.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 723.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 746.0, 2.5, 0.07, 0.4, 0.07 },		//15
//	{ 1, 0, 770.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 796.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 823.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 853.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 884.0, 2.5, 0.07, 0.4, 0.07 },		//20
//	{ 1, 0, 917.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 953.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 992.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 1034.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 1079.0, 2.5, 0.07, 0.4, 0.07 },		//25
//	{ 1, 0, 1129.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 1183.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 1242.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 1307.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 1379.0, 2.5, 0.07, 0.4, 0.07 },		//30
//	{ 1, 0, 1460.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 1535.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 1600.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 1670.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 1747.0, 2.5, 0.07, 0.4, 0.07 },		//35
//	{ 1, 0, 1831.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 1923.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 2025.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 2138.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 2265.0, 2.5, 0.07, 0.4, 0.07 },		//40
//	{ 1, 0, 2407.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 2567.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 2751.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 2963.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 3209.0, 2.5, 0.07, 0.4, 0.07 },		//45
//	{ 1, 0, 3500.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 3848.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 4274.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 4804.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 5485.0, 2.5, 0.07, 0.4, 0.07 },		//50
//	{ 1, 0, 6389.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 7651.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 9533.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 12642.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 12642.0, 2.5, 0.07, 0.4, 0.07 },	//55
//	{ 1, 0, 12642.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 12642.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 12642.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 12642.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 12642.0, 2.5, 0.07, 0.4, 0.07 },	//60
//	{ 1, 0, 12642.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 12642.0, 2.5, 0.07, 0.4, 0.07 },
//	{ 1, 0, 12642.0, 2.5, 0.07, 0.4, 0.07 }
};

CfgVeloView_t g_CfgVeloView;

/*
 * 从配置文件读入配置到配置结构体
 * @param[out] cfg       要写入的结构体
 * @param[in]  filename  要读入的文件
 * @return     0:成功 其他失败
 */
int LoadCfgVeloView(CfgVeloView_t& cfg, const char* filename)
{
    // 获得Global 配置文件
    char buf[64];
    char defaultVal[64];
#ifdef WIN32
    /*
     * Use Window API: GetPrivateProfileInt; GetPrivateProfileString...
     */
    /******************************************************** 全局配置参数 ********************************************************/
    //cfg.cfgGlobal.ColorMode =
    //	GetPrivateProfileInt("CfgGlobal", "ColorMode", g_DefaultGlobalCfg.ColorMode, filename);
    cfg.cfgGlobal.GroundZ =
        GetPrivateProfileInt("CfgGlobal", "GroundZ", g_DefaultGlobalCfg.GroundZ, filename);

    cfg.cfgGlobal.ThresholdMinHollow =
        GetPrivateProfileInt("CfgGlobal", "ThresholdMinHollow", g_DefaultGlobalCfg.ThresholdMinHollow, filename);
    cfg.cfgGlobal.ThresholdMaxHollow =
        GetPrivateProfileInt("CfgGlobal", "ThresholdMaxHollow", g_DefaultGlobalCfg.ThresholdMaxHollow, filename);

    cfg.cfgGlobal.ShotCalcFrom =
        GetPrivateProfileInt("CfgGlobal", "ShotCalcFrom", g_DefaultGlobalCfg.ShotCalcFrom, filename);
    cfg.cfgGlobal.ShotCalcNum =
        GetPrivateProfileInt("CfgGlobal", "ShotCalcNum", g_DefaultGlobalCfg.ShotCalcNum, filename);
    cfg.cfgGlobal.ShotShowFrom =
        GetPrivateProfileInt("CfgGlobal", "ShotShowFrom", g_DefaultGlobalCfg.ShotShowFrom, filename);
    cfg.cfgGlobal.ShotShowNum =
        GetPrivateProfileInt("CfgGlobal", "ShotShowNum", g_DefaultGlobalCfg.ShotShowNum, filename);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultGlobalCfg.ThresholdMinRad);
    GetPrivateProfileString("CfgGlobal", "ThresholdMinRad", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgGlobal.ThresholdMinRad = (float)atof(buf);

    //cfg.cfgGlobal.MaxCalRad = GetPrivateProfileInt("CfgGlobal", "MaxCalRad", g_DefaultGlobalCfg.MaxCalRad, filename);
    //cfg.cfgGlobal.MinCalRad = GetPrivateProfileInt("CfgGlobal", "MinCalRad", g_DefaultGlobalCfg.MinCalRad, filename);

    cfg.cfgGlobal.ContinusLineMinDistInCircle =
        GetPrivateProfileInt("CfgGlobal", "ContinusLineMinDistInCircle", g_DefaultGlobalCfg.ContinusLineMinDistInCircle, filename);

    //cfg.cfgGlobal.CarFrontX = GetPrivateProfileInt("CfgGlobal", "CarFrontX", g_DefaultGlobalCfg.CarFrontX, filename);

    //cfg.cfgGlobal.arg2 = GetPrivateProfileInt("CfgGlobal", "arg2", g_DefaultGlobalCfg.arg2, filename);
    //cfg.cfgGlobal.arg3 = GetPrivateProfileInt("CfgGlobal", "arg3", g_DefaultGlobalCfg.arg3, filename);
    //cfg.cfgGlobal.arg4 = GetPrivateProfileInt("CfgGlobal", "arg4", g_DefaultGlobalCfg.arg4, filename);
    //cfg.cfgGlobal.arg5 = GetPrivateProfileInt("CfgGlobal", "arg5", g_DefaultGlobalCfg.arg5, filename);
    //cfg.cfgGlobal.arg6 = GetPrivateProfileInt("CfgGlobal", "arg6", g_DefaultGlobalCfg.arg6, filename);
    //cfg.cfgGlobal.arg7 = GetPrivateProfileInt("CfgGlobal", "arg7", g_DefaultGlobalCfg.arg7, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultGlobalCfg.ThresholdRoadEdgeDeltaZLLimit);
    //GetPrivateProfileString("CfgGlobal", "ThresholdRoadEdgeDeltaZLLimit", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgGlobal.ThresholdRoadEdgeDeltaZLLimit = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultGlobalCfg.ThresholdRoadEdgeDeltaZULimit);
    //GetPrivateProfileString("CfgGlobal", "ThresholdRoadEdgeDeltaZULimit", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgGlobal.ThresholdRoadEdgeDeltaZULimit = (float)atof(buf);

    //cfg.cfgGlobal.RoadCurbStartCircle = GetPrivateProfileInt("CfgGlobal", "RoadCurbStartCircle", g_DefaultGlobalCfg.RoadCurbStartCircle, filename);
    //cfg.cfgGlobal.RoadCurbEndCircle = GetPrivateProfileInt("CfgGlobal", "RoadCurbEndCircle", g_DefaultGlobalCfg.RoadCurbEndCircle, filename);
    //cfg.cfgGlobal.RoadCurbMaxDis = GetPrivateProfileInt("CfgGlobal", "RoadCurbMaxDis", g_DefaultGlobalCfg.RoadCurbMaxDis, filename);
    //cfg.cfgGlobal.MinLaneNum = GetPrivateProfileInt("CfgGlobal", "MinLaneNum", g_DefaultGlobalCfg.MinLaneNum, filename);
    //cfg.cfgGlobal.MinLaneWidth = GetPrivateProfileInt("CfgGlobal", "MinLaneWidth", g_DefaultGlobalCfg.MinLaneWidth, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultGlobalCfg.ThresholdDeltaI);
    //GetPrivateProfileString("CfgGlobal", "ThresholdDeltaI", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgGlobal.ThresholdDeltaI = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultGlobalCfg.DeltaIColorFactor);
    //GetPrivateProfileString("CfgGlobal", "DeltaIColorFactor", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgGlobal.DeltaIColorFactor = (float)atof(buf);

    /******************************************************** 车辆模型 ********************************************************/
    cfg.cfgGlobal.CarLength =
        GetPrivateProfileInt("CfgGlobal", "CarLength", g_DefaultGlobalCfg.CarLength, filename);
    cfg.cfgGlobal.CarWidth =
        GetPrivateProfileInt("CfgGlobal", "CarWidth", g_DefaultGlobalCfg.CarWidth, filename);
    cfg.cfgGlobal.CarHeight =
        GetPrivateProfileInt("CfgGlobal", "CarHeight", g_DefaultGlobalCfg.CarHeight, filename);


    /******************************************************** 平面检测配置参数 ********************************************************/
    cfg.cfgPlaneDetect.GridCellSize =
        GetPrivateProfileInt("CfgPlaneDetect", "GridCellSize", g_DefaultCfgPlaneDetect.GridCellSize, filename);
    cfg.cfgPlaneDetect.GridHeadRowNum =
        GetPrivateProfileInt("CfgPlaneDetect", "GridHeadRowNum", g_DefaultCfgPlaneDetect.GridHeadRowNum, filename);
    cfg.cfgPlaneDetect.GridBackRowNum =
        GetPrivateProfileInt("CfgPlaneDetect", "GridBackRowNum", g_DefaultCfgPlaneDetect.GridBackRowNum, filename);
    cfg.cfgPlaneDetect.GridColumnNum =
        GetPrivateProfileInt("CfgPlaneDetect", "GridColumnNum", g_DefaultCfgPlaneDetect.GridColumnNum, filename);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgPlaneDetect.GridThresholdGroundDetect);
    GetPrivateProfileString("CfgPlaneDetect", "GridThresholdGroundDetect", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgPlaneDetect.GridThresholdGroundDetect = (float)atof(buf);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgPlaneDetect.GridThresholdDiffZ);
    GetPrivateProfileString("CfgPlaneDetect", "GridThresholdDiffZ", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgPlaneDetect.GridThresholdDiffZ = (float)atof(buf);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgPlaneDetect.GridColorFactor);
    GetPrivateProfileString("CfgPlaneDetect", "GridColorFactor", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgPlaneDetect.GridColorFactor = (float)atof(buf);

    cfg.cfgPlaneDetect.GridColorMode =
        GetPrivateProfileInt("CfgPlaneDetect", "GridColorMode", g_DefaultCfgPlaneDetect.GridColorMode, filename);


    cfg.cfgPlaneDetect.PieCellSize =
        GetPrivateProfileInt("CfgPlaneDetect", "PieCellSize", g_DefaultCfgPlaneDetect.PieCellSize, filename);
    cfg.cfgPlaneDetect.PieAzimuthNum =
        GetPrivateProfileInt("CfgPlaneDetect", "PieAzimuthNum", g_DefaultCfgPlaneDetect.PieAzimuthNum, filename);
    cfg.cfgPlaneDetect.PieMinRad =
        GetPrivateProfileInt("CfgPlaneDetect", "PieMinRad", g_DefaultCfgPlaneDetect.PieMinRad, filename);
    cfg.cfgPlaneDetect.PieMaxRad =
        GetPrivateProfileInt("CfgPlaneDetect", "PieMaxRad", g_DefaultCfgPlaneDetect.PieMaxRad, filename);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgPlaneDetect.PieThresholdGroundDetect);
    GetPrivateProfileString("CfgPlaneDetect", "PieThresholdGroundDetect", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgPlaneDetect.PieThresholdGroundDetect = (float)atof(buf);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgPlaneDetect.PieThresholdDiffZ);
    GetPrivateProfileString("CfgPlaneDetect", "PieThresholdDiffZ", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgPlaneDetect.PieThresholdDiffZ = (float)atof(buf);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgPlaneDetect.PieColorFactor);
    GetPrivateProfileString("CfgPlaneDetect", "PieColorFactor", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgPlaneDetect.PieColorFactor = (float)atof(buf);

    cfg.cfgPlaneDetect.PieColorMode =
        GetPrivateProfileInt("CfgPlaneDetect", "PieColorMode", g_DefaultCfgPlaneDetect.PieColorMode, filename);

    //cfg.cfgPlaneDetect.Enable = GetPrivateProfileInt("CfgPlaneDetect", "Enable", g_DefaultCfgPlaneDetect.Enable, filename);

    //cfg.cfgPlaneDetect.DeleteSingleRedGridMaxRow = GetPrivateProfileInt("CfgPlaneDetect", "DeleteSingleRedGridMaxRow", g_DefaultCfgPlaneDetect.DeleteSingleRedGridMaxRow, filename);
    //cfg.cfgPlaneDetect.DeleteSingleRedGridMaxNum = GetPrivateProfileInt("CfgPlaneDetect", "DeleteSingleRedGridMaxNum", g_DefaultCfgPlaneDetect.DeleteSingleRedGridMaxNum, filename);

    /******************************************************** 负障碍检测配置参数 ********************************************************/
    cfg.cfgNegativeObstacle.NegativeObstacleCellSize =
        GetPrivateProfileInt("CfgNegativeObstacle", "NegativeObstacleCellSize", g_DefaultCfgNegativeObstacle.NegativeObstacleCellSize, filename);
    cfg.cfgNegativeObstacle.NegativeObstacleGridDim =
        GetPrivateProfileInt("CfgNegativeObstacle", "NegativeObstacleGridDim", g_DefaultCfgNegativeObstacle.NegativeObstacleGridDim, filename);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgNegativeObstacle.NegativeObstacleThresholdStartEdge);
    GetPrivateProfileString("CfgNegativeObstacle", "NegativeObstacleThresholdStartEdge", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgNegativeObstacle.NegativeObstacleThresholdStartEdge = (float)atof(buf);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgNegativeObstacle.NegativeObstacleThresholdEndEdge);
    GetPrivateProfileString("CfgNegativeObstacle", "NegativeObstacleThresholdEndEdge", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgNegativeObstacle.NegativeObstacleThresholdEndEdge = (float)atof(buf);

    cfg.cfgNegativeObstacle.NegativeObstacleThresholdEdgeNumber =
        GetPrivateProfileInt("CfgNegativeObstacle", "NegativeObstacleThresholdEdgeNumber", g_DefaultCfgNegativeObstacle.NegativeObstacleThresholdEdgeNumber, filename);

    /******************************************************** 旧的路口算法的配置 ********************************************************/
    //cfg.cfginterSection.State1mOr5m = GetPrivateProfileInt("CfgInterSection", "State1mOr5m", g_DefaultCfgInterSection.State1mOr5m, filename);
    //cfg.cfginterSection.StartCheckOut = GetPrivateProfileInt("CfgInterSection", "StartCheckOut", g_DefaultCfgInterSection.StartCheckOut, filename);
    //cfg.cfginterSection.OutOfInterSection = GetPrivateProfileInt("CfgInterSection", "OutOfInterSection", g_DefaultCfgInterSection.OutOfInterSection, filename);
    //cfg.cfginterSection.SearchWide = GetPrivateProfileInt("CfgInterSection", "SearchWide", g_DefaultCfgInterSection.SearchWide, filename);
    //cfg.cfginterSection.SatisfyNum = GetPrivateProfileInt("CfgInterSection", "SatisfyNum", g_DefaultCfgInterSection.SatisfyNum, filename);
    //cfg.cfginterSection.SatisfyContiniousNum = GetPrivateProfileInt("CfgInterSection", "SatisfyContiniousNum", g_DefaultCfgInterSection.SatisfyContiniousNum, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgInterSection.DisL);
    //GetPrivateProfileString("CfgInterSection", "DisL", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfginterSection.DisL = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgInterSection.DisR);
    //GetPrivateProfileString("CfgInterSection", "DisR", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfginterSection.DisR = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgInterSection.LastDisLAboveZero);
    //GetPrivateProfileString("CfgInterSection", "LastDisLAboveZero", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfginterSection.LastDisLAboveZero = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgInterSection.LastDisRAboveZero);
    //GetPrivateProfileString("CfgInterSection", "LastDisRAboveZero", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfginterSection.LastDisRAboveZero = (float)atof(buf);

    //for (j = 0; j<6; j++){
    //	_snprintf(section, sizeof(section), "CfginterSection%d", j);
    //	cfg.cfginterSection.CheckDistance[j] = GetPrivateProfileInt(section, "CheckDistance", g_DefaultCfgInterSection.CheckDistance[j], filename);
    //}

    //for (j = 0; j<6; j++){
    //	_snprintf(section, sizeof(section), "CfginterSection%d", j);
    //	cfg.cfginterSection.LeftReach[j] = GetPrivateProfileInt(section, "LeftReach", g_DefaultCfgInterSection.LeftReach[j], filename);
    //}
    //for (j = 0; j<6; j++){
    //	_snprintf(section, sizeof(section), "CfginterSection%d", j);
    //	cfg.cfginterSection.RightReach[j] = GetPrivateProfileInt(section, "RightReach", g_DefaultCfgInterSection.RightReach[j], filename);
    //}


    /************************************************* 新的路口算法的配置 **************************************************/
    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.RoadWide);
    //GetPrivateProfileString("CfgIntersection", "RoadWide", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.RoadWide = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.IntersectionWide);
    //GetPrivateProfileString("CfgIntersection", "IntersectionWide", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.IntersectionWide = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.CheckPassageDis);
    //GetPrivateProfileString("CfgIntersection", "CheckPassageDis", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.CheckPassageDis = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.PositionDecision);
    //GetPrivateProfileString("CfgIntersection", "PositionDecision", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.PositionDecision = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.Multiple);
    //GetPrivateProfileString("CfgIntersection", "Multiple", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.Multiple = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.Power);
    //GetPrivateProfileString("CfgIntersection", "Power", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.Power = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.TestAngle);
    //GetPrivateProfileString("CfgIntersection", "TestAngle", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.TestAngle = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.YPosAlliance);
    //GetPrivateProfileString("CfgIntersection", "YPosAlliance", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.YPosAlliance = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.safeDis);
    //GetPrivateProfileString("CfgIntersection", "safeDis", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.safeDis = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.turnLeftLimitDis);
    //GetPrivateProfileString("CfgIntersection", "turnLeftLimitDis", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.turnLeftLimitDis = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.turnRightDis);
    //GetPrivateProfileString("CfgIntersection", "turnRightDis", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.turnRightDis = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.endPos);
    //GetPrivateProfileString("CfgIntersection", "endPos", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.endPos = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.turnRightScale);
    //GetPrivateProfileString("CfgIntersection", "turnRightScale", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.turnRightScale = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.frontCalDis);
    //GetPrivateProfileString("CfgIntersection", "frontCalDis", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.frontCalDis = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.frontMaxIntervalLWide);
    //GetPrivateProfileString("CfgIntersection", "frontMaxIntervalLWide", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.frontMaxIntervalLWide = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.frontMaxIntervalLendXDis);
    //GetPrivateProfileString("CfgIntersection", "frontMaxIntervalLendXDis", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.frontMaxIntervalLendXDis = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.interSectionAdvanceDis);
    //GetPrivateProfileString("CfgIntersection", "interSectionAdvanceDis", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.interSectionAdvanceDis = (float)atof(buf);


    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.roadShapeLengthFilter);
    //GetPrivateProfileString("CfgIntersection", "roadShapeLengthFilter", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.roadShapeLengthFilter = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.slashLengthLimit);
    //GetPrivateProfileString("CfgIntersection", "slashLengthLimit", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.slashLengthLimit = (float)atof(buf);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgIntersection.slashWidth);
    //GetPrivateProfileString("CfgIntersection", "slashWidth", defaultVal, buf, sizeof(buf), filename);
    //cfg.cfgIntersecion.slashWidth = (float)atof(buf);


    /******************************************************** 每个Circle的配置 ********************************************************/
    char bufCircle[64];
    for (unsigned circle = 0; circle<CIRCLE_NUM; ++circle)
    {
        _snprintf(bufCircle, sizeof(bufCircle), "CfgCircleItems%02d", circle);
        cfg.cfgCircleItems[circle].Enable =
            GetPrivateProfileInt(bufCircle, "Enable", g_DefaultCfgCircleItems[circle].Enable, filename);

        cfg.cfgCircleItems[circle].ThresholdMinDist =
            GetPrivateProfileInt(bufCircle, "ThresholdMinDist", g_DefaultCfgCircleItems[circle].ThresholdMinDist, filename);

        _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgCircleItems[circle].ThresholdRoadEdge);
        GetPrivateProfileString(bufCircle, "ThresholdRoadEdge", defaultVal, buf, sizeof(buf), filename);
        cfg.cfgCircleItems[circle].ThresholdRoadEdge = (float)atof(buf);

        _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgCircleItems[circle].ThresholdRoadEdgeByDeltaZ);
        GetPrivateProfileString(bufCircle, "ThresholdRoadEdgeByDeltaZ", defaultVal, buf, sizeof(buf), filename);
        cfg.cfgCircleItems[circle].ThresholdRoadEdgeByDeltaZ = (float)atof(buf);

        //cfg.cfgCircleItems[circle].ColorMode = GetPrivateProfileInt(section, "ColorMode", g_DefaultCfgCircleItems[circle].ColorMode, filename);

        //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgCircleItems[circle].ColorFactor);
        //GetPrivateProfileString(section, "ColorFactor", defaultVal, buf, sizeof(buf), filename);
        //cfg.cfgCircleItems[circle].ColorFactor = (float)atof(buf);


        //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgCircleItems[circle].ColorFactorByDeltaZ);
        //GetPrivateProfileString(section, "ColorFactorByDeltaZ", defaultVal, buf, sizeof(buf), filename);
        //cfg.cfgCircleItems[circle].ColorFactorByDeltaZ = (float)atof(buf);
    }

    for (unsigned circle = 0; circle<32; circle++)
    {
        _snprintf(bufCircle, sizeof(bufCircle), "IgnoreCircleWhenProjection%02d", circle);
        cfg.cfgIgnoreCircleWhenProjection.IgnoreCircle[circle] =
            GetPrivateProfileInt("cfgIgnoreCircleWhenProjection", bufCircle, g_DefaultCfgIgnoreCircleWhenProjection.IgnoreCircle[circle], filename);
    }

    //for (j = 0; j<CIRCLE_NUM; ++j){
    //	_snprintf(bufCircle, sizeof(bufCircle), "offset%02d", j);
    //	cfg.cfgShotOffset.offset[circle] = GetPrivateProfileInt("cfgShotOffset", bufCircle, g_DefaultCfgShotOffset.offset[circle], filename);
    //}


    /******************************************************** 车模型的配置 ********************************************************/
    //cfg.cfgCarShapeParameter.min_length = GetPrivateProfileInt("CfgCarShapeParameter", "min_length", g_DefaultCfgCarShapeParameter.min_length, filename);
    //cfg.cfgCarShapeParameter.max_length = GetPrivateProfileInt("CfgCarShapeParameter", "max_length", g_DefaultCfgCarShapeParameter.max_length, filename);
    //cfg.cfgCarShapeParameter.min_width = GetPrivateProfileInt("CfgCarShapeParameter", "min_width", g_DefaultCfgCarShapeParameter.min_width, filename);
    //cfg.cfgCarShapeParameter.max_width = GetPrivateProfileInt("CfgCarShapeParameter", "max_width", g_DefaultCfgCarShapeParameter.max_width, filename);
    //cfg.cfgCarShapeParameter.max_length_width_ratio = GetPrivateProfileInt("CfgCarShapeParameter", "max_length_width_ratio", g_DefaultCfgCarShapeParameter.max_length_width_ratio, filename);
#else
    /*
     * Use Glib API: g_key_file...
     */
    // get Gkeyfile
    GKeyFile* glib_keyfile;
    glib_keyfile = g_key_file_new();

    GError **error;
    error = NULL;
    //flags = G_KEY_FILE_KEEP_COMMENTS | G_KEY_FILE_KEEP_TRANSLATIONS;
    //g_key_file_load_from_file(filename,"velodyne.ini",G_KEY_FILE_NONE, error);
    if (!g_key_file_load_from_file(glib_keyfile, filename, G_KEY_FILE_NONE, NULL))
    {
        fprintf (stderr, "Could not read config file\n");
        return EXIT_FAILURE;
    }
    /*
     * edit by sean
     *  glib function:
     *      gint g_key_file_set_integer (GKeyFile *key_file, const gchar *group_name, const gchar *key, gint value);
     * added by durant35:
     *  no default value, may be achieve it by error process.
     */
    /******************************************************** 全局配置参数 ********************************************************/
    cfg.cfgGlobal.GroundZ =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "GroundZ", error);

    cfg.cfgGlobal.ThresholdMinHollow =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "ThresholdMinHollow", error);
    cfg.cfgGlobal.ThresholdMaxHollow =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "ThresholdMaxHollow", error);

    cfg.cfgGlobal.ShotCalcFrom =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "ShotCalcFrom", error);
    cfg.cfgGlobal.ShotCalcNum =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "ShotCalcNum", error);
    cfg.cfgGlobal.ShotShowFrom =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "ShotShowFrom", error);
    cfg.cfgGlobal.ShotShowNum =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "ShotShowNum", error);

    cfg.cfgGlobal.ThresholdMinRad =
        g_key_file_get_double(glib_keyfile, "CfgGlobal", "ThresholdMinRad", error);
    cfg.cfgGlobal.ContinusLineMinDistInCircle =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "ContinusLineMinDistInCircle", error);
    /******************************************************** 车辆模型 ********************************************************/
    cfg.cfgGlobal.CarLength =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "CarLength", error);
    cfg.cfgGlobal.CarWidth =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "CarWidth", error);
    cfg.cfgGlobal.CarHeight =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "CarHeight", error);

    /****** Laser distacne filter ******/
    cfg.cfgGlobal.MinLaserDistance =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "MinLaserDistance", error);
    cfg.cfgGlobal.MaxLaserDistacne =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "MaxLaserDistacne", error);


    /******************************************************** 平面检测配置参数 ********************************************************/
    cfg.cfgPlaneDetect.GridCellSize =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "GridCellSize", error);
    cfg.cfgPlaneDetect.GridHeadRowNum =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "GridHeadRowNum", error);
    cfg.cfgPlaneDetect.GridBackRowNum =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "GridBackRowNum", error);
    cfg.cfgPlaneDetect.GridColumnNum =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "GridColumnNum", error);
    cfg.cfgPlaneDetect.GridThresholdGroundDetect =
        g_key_file_get_double(glib_keyfile, "CfgPlaneDetect", "GridThresholdGroundDetect", error);
    cfg.cfgPlaneDetect.GridThresholdDiffZ =
        g_key_file_get_double(glib_keyfile, "CfgPlaneDetect", "GridThresholdDiffZ", error);
    cfg.cfgPlaneDetect.GridColorFactor =
        g_key_file_get_double(glib_keyfile, "CfgPlaneDetect", "GridColorFactor", error);
    cfg.cfgPlaneDetect.GridColorMode =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "GridColorMode", error);

    cfg.cfgPlaneDetect.PieRadSize =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "PieRadSize", error);
    cfg.cfgPlaneDetect.PieShotFrom =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "PieShotFrom", error);
    cfg.cfgPlaneDetect.PieShotEnd =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "PieShotEnd", error);
    cfg.cfgPlaneDetect.PieAzimuthNum =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "PieAzimuthNum", error);
    cfg.cfgPlaneDetect.PieMinRad =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "PieMinRad", error);
    cfg.cfgPlaneDetect.PieMaxRad =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "PieMaxRad", error);
    cfg.cfgPlaneDetect.PieThresholdGroundDetect =
        g_key_file_get_double(glib_keyfile, "CfgPlaneDetect", "PieThresholdGroundDetect", error);
    cfg.cfgPlaneDetect.PieThresholdDiffZ =
        g_key_file_get_double(glib_keyfile, "CfgPlaneDetect", "PieThresholdDiffZ", error);
    cfg.cfgPlaneDetect.PieColorFactor =
        g_key_file_get_double(glib_keyfile, "CfgPlaneDetect", "PieColorFactor", error);
    cfg.cfgPlaneDetect.PieColorMode =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "PieColorMode", error);

    /******************************************************** 负障碍检测配置参数 ********************************************************/
    cfg.cfgNegativeObstacle.NegativeObstacleCellSize =
        g_key_file_get_integer(glib_keyfile, "CfgNegativeObstacle", "NegativeObstacleCellSize", error);
    cfg.cfgNegativeObstacle.NegativeObstacleGridDim =
        g_key_file_get_integer(glib_keyfile, "CfgNegativeObstacle", "NegativeObstacleGridDim", error);

    cfg.cfgNegativeObstacle.NegativeObstacleThresholdStartEdge =
        g_key_file_get_double(glib_keyfile, "CfgNegativeObstacle", "NegativeObstacleThresholdStartEdge", error);
    cfg.cfgNegativeObstacle.NegativeObstacleThresholdEndEdge =
        g_key_file_get_double(glib_keyfile, "CfgNegativeObstacle", "NegativeObstacleThresholdEndEdge", error);

    cfg.cfgNegativeObstacle.NegativeObstacleThresholdEdgeNumber =
        g_key_file_get_integer(glib_keyfile, "CfgNegativeObstacle", "NegativeObstacleThresholdEdgeNumber", error);


    /******************************************************** 每个Circle的配置 ********************************************************/
    char bufCircle[64];
    for (unsigned circle = 0; circle<CIRCLE_NUM; ++circle)
    {
        snprintf(bufCircle, sizeof(bufCircle), "CfgCircleItems%02d", circle);

        cfg.cfgCircleItems[circle].Enable =
            g_key_file_get_integer(glib_keyfile, bufCircle, "Enable", error);
        cfg.cfgCircleItems[circle].ThresholdMinDist =
            g_key_file_get_integer(glib_keyfile, bufCircle, "ThresholdMinDist", error);

        //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgCircleItems[circle].ThresholdRoadEdge);
        //GetPrivateProfileString(bufCircle, "ThresholdRoadEdge", defaultVal, buf, sizeof(buf), filename);
        cfg.cfgCircleItems[circle].ThresholdRoadEdge =
            g_key_file_get_double(glib_keyfile, bufCircle, "ThresholdRoadEdge", error);
        //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgCircleItems[circle].ThresholdRoadEdgeByDeltaZ);
        //GetPrivateProfileString(bufCircle, "ThresholdRoadEdgeByDeltaZ", defaultVal, buf, sizeof(buf), filename);
        cfg.cfgCircleItems[circle].ThresholdRoadEdgeByDeltaZ =
            g_key_file_get_double(glib_keyfile, bufCircle, "ThresholdRoadEdgeByDeltaZ", error);
    }
    for (unsigned circle = 0; circle<32; circle++)
    {
        snprintf(bufCircle, sizeof(bufCircle), "IgnoreCircleWhenProjection%02d", circle);
        cfg.cfgIgnoreCircleWhenProjection.IgnoreCircle[circle] =
            g_key_file_get_integer(glib_keyfile, "cfgIgnoreCircleWhenProjection", bufCircle, error);
    }

    g_key_file_free (glib_keyfile);
#endif // WIN32

    return 0;
}
/*
 *    把配置结构体写入到配置文件
 *    @param[in]  cfg       要读取的结构体
 *    @param[out] filename  要写入的文件
 *    @return     0:成功 其他失败
 */
int SaveCfgVeloView(CfgVeloView_t& cfg, const char* filename)
{
#ifdef win32
    char defaultVal[64];

    /******************************************************** 全局配置参数 ********************************************************/
    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.ColorMode);
    //WritePrivateProfileString("CfgGlobal", "ColorMode", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.ShotShowFrom);
    //WritePrivateProfileString("CfgGlobal", "ShotShowFrom", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.ShotShowNum);
    //WritePrivateProfileString("CfgGlobal", "ShotShowNum", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.ShotCalcFrom);
    //WritePrivateProfileString("CfgGlobal", "ShotCalcFrom", defaultVal, filename);
    g_key_file_set_double(filename, "CfgGlobal", "ShotCalcFrom", cfg.cfgGlobal.ShotCalcFrom);

    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.ShotCalcNum);
    //WritePrivateProfileString("CfgGlobal", "ShotCalcNum", defaultVal, filename);
    g_key_file_set_string(filename, "CfgGlobal", "ShotCalcNum", defaultVal);
    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.MaxRad);
    //WritePrivateProfileString("CfgGlobal", "MaxRad", defaultVal, filename);
    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.minRad);
    //WritePrivateProfileString("CfgGlobal", "minRad", defaultVal, filename);

    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.GroundZ);
    //WritePrivateProfileString("CfgGlobal", "GroundZ", defaultVal, filename);
    g_key_file_set_string(filename, "CfgGlobal", "GroundZ", defaultVal);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.CarFrontX);
    //WritePrivateProfileString("CfgGlobal", "CarFrontX", defaultVal, filename);

    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.ThresholdMinHollow);
    //WritePrivateProfileString("CfgGlobal", "ThresholdMinHollow", defaultVal, filename);
    g_key_file_set_string(filename, "CfgGlobal", "ThresholdMinHollow", defaultVal);

    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.ThresholdMaxHollow);
    //WritePrivateProfileString("CfgGlobal", "ThresholdMaxHollow", defaultVal, filename);
    g_key_file_set_string(filename, "CfgGlobal", "ThresholdMaxHollow" defaultVal);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgGlobal.ThresholdMinRad);
    //WritePrivateProfileString("CfgGlobal", "ThresholdMinRad", defaultVal, filename);
    g_key_file_set_string(filename, "CfgGlobal", "ThresholdMinRad", defaultVal);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgGlobal.ThresholdDeltaI);
    //WritePrivateProfileString("CfgGlobal", "ThresholdDeltaI", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgGlobal.DeltaIColorFactor);
    //WritePrivateProfileString("CfgGlobal", "DeltaIColorFactor", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.arg0);
    //WritePrivateProfileString("CfgGlobal", "arg0", defaultVal, filename);

    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.ContinusLineMinDistInCircle);
    //WritePrivateProfileString("CfgGlobal", "ContinusLineMinDistInCircle", defaultVal, filename);
    g_key_file_set_string(filename, "CfgGlobal", "ContinusLineMinDistInCircle", defaultVal);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.arg2);
    //WritePrivateProfileString("CfgGlobal", "arg2", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.arg3);
    //WritePrivateProfileString("CfgGlobal", "arg3", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.arg4);
    //WritePrivateProfileString("CfgGlobal", "arg4", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.arg5);
    //WritePrivateProfileString("CfgGlobal", "arg5", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.arg6);
    //WritePrivateProfileString("CfgGlobal", "arg6", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.arg7);
    //WritePrivateProfileString("CfgGlobal", "arg7", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgGlobal.ThresholdRoadEdgeDeltaZLLimit);
    //WritePrivateProfileString("CfgGlobal", "ThresholdRoadEdgeDeltaZLLimit", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgGlobal.ThresholdRoadEdgeDeltaZULimit);
    //WritePrivateProfileString("CfgGlobal", "ThresholdRoadEdgeDeltaZULimit", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.RoadCurbStartCircle);
    //WritePrivateProfileString("CfgGlobal", "RoadCurbStartCircle", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.RoadCurbEndCircle);
    //WritePrivateProfileString("CfgGlobal", "RoadCurbEndCircle", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.RoadCurbMaxDis);
    //WritePrivateProfileString("CfgGlobal", "RoadCurbMaxDis", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.MinLaneNum);
    //WritePrivateProfileString("CfgGlobal", "MinLaneNum", defaultVal, filename);
    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.MinLaneWidth);
    //WritePrivateProfileString("CfgGlobal", "MinLaneWidth", defaultVal, filename);


    /******************************************************** 平面检测配置参数 ********************************************************/
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.PieCellSize);
    //WritePrivateProfileString("CfgPlaneDetect", "PieCellSize", defaultVal, filename);
    g_key_file_set_string(filename, "CfgPlaneDetect", "PieCellSize", defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.PieAzimuthNum);
    //WritePrivateProfileString("CfgPlaneDetect", "PieAzimuthNum", defaultVal, filename);
    g_key_file_set_string(filename, "CfgPlaneDetect", "PieAzimuthNum", defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.PieMinRad);
    //WritePrivateProfileString("CfgPlaneDetect", "PieMinRad", defaultVal, filename);
    g_key_file_set_string(filename, "CfgPlaneDetect", "PieMinRad", defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.PieMaxRad);
    //WritePrivateProfileString("CfgPlaneDetect", "PieMaxRad", defaultVal, filename);
    g_key_file_set_string(filename, "CfgPlaneDetect", "PieMaxRad", defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgPlaneDetect.PieThresholdGroundDetect);
    //WritePrivateProfileString("CfgPlaneDetect", "PieThresholdGroundDetect", defaultVal, filename);
    g_key_file_set_string(filename,"CfgPlaneDetect", "PieThresholdGroundDetect",defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgPlaneDetect.PieColorFactor);
    //WritePrivateProfileString("CfgPlaneDetect", "PieColorFactor", defaultVal, filename);
    g_key_file_set_string(filename,"CfgPlaneDetect", "PieColorFactor",defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgPlaneDetect.PieColorMode);
    //WritePrivateProfileString("CfgPlaneDetect", "PieColorMode", defaultVal, filename);
    g_key_file_set_string(filename,"CfgPlaneDetect", "PieColorMode",defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.GridCellSize);
    WritePrivateProfileString("CfgPlaneDetect", "GridCellSize", defaultVal, filename);
    g_key_file_set_string(filename,"CfgPlaneDetect", "PieColorMode",defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.GridColumnNum);
    WritePrivateProfileString("CfgPlaneDetect", "GridColumnNum", defaultVal, filename);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.GridHeadRowNum);
    WritePrivateProfileString("CfgPlaneDetect", "GridHeadRowNum", defaultVal, filename);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.GridBackRowNum);
    WritePrivateProfileString("CfgPlaneDetect", "GridBackRowNum", defaultVal, filename);
    _snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgPlaneDetect.GridThresholdGroundDetect);
    WritePrivateProfileString("CfgPlaneDetect", "GridThresholdGroundDetect", defaultVal, filename);
    _snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgPlaneDetect.GridColorFactor);
    WritePrivateProfileString("CfgPlaneDetect", "GridColorFactor", defaultVal, filename);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.GridColorMode);
    WritePrivateProfileString("CfgPlaneDetect", "GridColorMode", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.DeleteSingleRedGridMaxRow);
    //WritePrivateProfileString("CfgPlaneDetect", "DeleteSingleRedGridMaxRow", defaultVal, filename);
    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.DeleteSingleRedGridMaxNum);
    //WritePrivateProfileString("CfgPlaneDetect", "DeleteSingleRedGridMaxNum", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.Enable);
    //WritePrivateProfileString("CfgPlaneDetect", "Enable", defaultVal, filename);


    /******************************************************** 路口算法配置参数 ********************************************************/
    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfginterSection.DisL);
    //WritePrivateProfileString("CfgInterSection", "DisL", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfginterSection.DisR);
    //WritePrivateProfileString("CfgInterSection", "DisR", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfginterSection.LastDisLAboveZero);
    //WritePrivateProfileString("CfgInterSection", "LastDisLAboveZero", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfginterSection.LastDisRAboveZero);
    //WritePrivateProfileString("CfgInterSection", "LastDisRAboveZero", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfginterSection.State1mOr5m);
    //WritePrivateProfileString("CfgInterSection", "State1mOr5m", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfginterSection.StartCheckOut);
    //WritePrivateProfileString("CfgInterSection", "StartCheckOut", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfginterSection.OutOfInterSection);
    //WritePrivateProfileString("CfgInterSection", "OutOfInterSection", defaultVal, filename);




    // Intersection
    //for (j = 0; j<6; j++){
    //	_snprintf(section, sizeof(section), "CheckDistance%01d", j);

    //	_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfginterSection.CheckDistance[j]);
    //	WritePrivateProfileString(section, "CheckDistance", defaultVal, filename);
    //}
    //for (j = 0; j<6; j++){
    //	_snprintf(section, sizeof(section), "LeftReach%01d", j);
    //	_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfginterSection.LeftReach[j]);
    //	WritePrivateProfileString(section, "LeftReach", defaultVal, filename);
    //}
    //for (j = 0; j<6; j++){
    //	_snprintf(section, sizeof(section), "RightReach%01d", j);
    //	_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfginterSection.RightReach[j]);
    //	WritePrivateProfileString(section, "RightReach", defaultVal, filename);
    //}

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfginterSection.SearchWide);
    //WritePrivateProfileString("CfgInterSection", "SearchWide", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfginterSection.SatisfyNum);
    //WritePrivateProfileString("CfgInterSection", "SatisfyNum", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfginterSection.SatisfyContiniousNum);
    //WritePrivateProfileString("CfgInterSection", "SatisfyContiniousNum", defaultVal, filename);

    ///**新的路口**/
    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.RoadWide);
    //WritePrivateProfileString("CfgIntersection", "RoadWide", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.IntersectionWide);
    //WritePrivateProfileString("CfgIntersection", "IntersectionWide", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.CheckPassageDis);
    //WritePrivateProfileString("CfgIntersection", "CheckPassageDis", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.PositionDecision);
    //WritePrivateProfileString("CfgIntersection", "PositionDecision", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.Multiple);
    //WritePrivateProfileString("CfgIntersection", "Multiple", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.Power);
    //WritePrivateProfileString("CfgIntersection", "Power", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.TestAngle);
    //WritePrivateProfileString("CfgIntersection", "TestAngle", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.YPosAlliance);
    //WritePrivateProfileString("CfgIntersection", "YPosAlliance", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.safeDis);
    //WritePrivateProfileString("CfgIntersection", "safeDis", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.turnLeftLimitDis);
    //WritePrivateProfileString("CfgIntersection", "turnLeftLimitDis", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.turnRightDis);
    //WritePrivateProfileString("CfgIntersection", "turnRightDis", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.endPos);
    //WritePrivateProfileString("CfgIntersection", "endPos", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.turnRightScale);
    //WritePrivateProfileString("CfgIntersection", "turnRightScale", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.frontCalDis);
    //WritePrivateProfileString("CfgIntersection", "frontCalDis", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.frontMaxIntervalLWide);
    //WritePrivateProfileString("CfgIntersection", "frontMaxIntervalLWide", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.frontMaxIntervalLendXDis);
    //WritePrivateProfileString("CfgIntersection", "frontMaxIntervalLendXDis", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.interSectionAdvanceDis);
    //WritePrivateProfileString("CfgIntersection", "interSectionAdvanceDis", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.roadShapeLengthFilter);
    //WritePrivateProfileString("CfgIntersection", "roadShapeLengthFilter", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.slashLengthLimit);
    //WritePrivateProfileString("CfgIntersection", "slashLengthLimit", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgIntersecion.slashWidth);
    //WritePrivateProfileString("CfgIntersection", "slashWidth", defaultVal, filename);


    /******************************************************** 每个Circle的配置 ********************************************************/
    char bufCircle[64];
    for (unsigned circle = 0; circle<CIRCLE_NUM; ++circle)
    {
        _snprintf(bufCircle, sizeof(bufCircle), "CfgCircleItems%02d", circle);

        //_snprintf(defaultVal, sizeof(defaultVal), "%d", (int)cfg.cfgCircleItems[circle].MaxRad);
        //WritePrivateProfileString(bufCircle, "MaxRad", defaultVal, filename);

        //_snprintf(defaultVal, sizeof(defaultVal), "%d", (int)cfg.cfgCircleItems[circle].ColorMode);
        //WritePrivateProfileString(bufCircle, "ColorMode", defaultVal, filename);

        //_snprintf(defaultVal, sizeof(defaultVal), "%d", (int)cfg.cfgCircleItems[circle].Enable);
        //WritePrivateProfileString(bufCircle, "Enable", defaultVal, filename);

        //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgCircleItems[circle].ColorFactor);
        //WritePrivateProfileString(bufCircle, "ColorFactor", defaultVal, filename);

        //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgCircleItems[circle].ThresholdRoadEdge);
        //WritePrivateProfileString(bufCircle, "ThresholdRoadEdge", defaultVal, filename);

        //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgCircleItems[circle].ColorFactorByDeltaZ);
        //WritePrivateProfileString(bufCircle, "ColorFactorByDeltaZ", defaultVal, filename);

        //_snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgCircleItems[circle].ThresholdRoadEdgeByDeltaZ);
        //WritePrivateProfileString(bufCircle, "ThresholdRoadEdgeByDeltaZ", defaultVal, filename);
    }

    //for (j = 0; j<CIRCLE_NUM; ++j){
    //	_snprintf(bufCircle, sizeof(bufCircle), "offset%02d", j);
    //	_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgShotOffset.offset[circle]);
    //	WritePrivateProfileString("cfgShotOffset", bufCircle, defaultVal, filename);
    //}
    for (unsigned circle = 0; circle<32; ++circle)
    {
        _snprintf(bufCircle, sizeof(bufCircle), "IgnoreCircleWhenProjection%02d", circle);

        _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgIgnoreCircleWhenProjection.IgnoreCircle[circle]);
        WritePrivateProfileString("cfgIgnoreCircleWhenProjection", bufCircle, defaultVal, filename);
    }

    /******************************************************** 车模型的配置 ********************************************************/
    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgCarShapeParameter.min_length);
    //WritePrivateProfileString("CfgCarShapeParameter", "min_length", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgCarShapeParameter.max_length);
    //WritePrivateProfileString("CfgCarShapeParameter", "max_length", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgCarShapeParameter.min_width);
    //WritePrivateProfileString("CfgCarShapeParameter", "min_width", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgCarShapeParameter.max_width);
    //WritePrivateProfileString("CfgCarShapeParameter", "max_width", defaultVal, filename);

    //_snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgCarShapeParameter.max_length_width_ratio);
    //WritePrivateProfileString("CfgCarShapeParameter", "max_length_width_ratio", defaultVal, filename);
#else
    // get Gkeyfile
    GKeyFile* viewer_ini = g_key_file_new();
    GError **error = NULL;
    if (!g_key_file_load_from_file(viewer_ini, "velodyne.ini",
                                G_KEY_FILE_NONE, error)){
        fprintf (stderr, "Could not read config file\n");
        return EXIT_FAILURE;
    }

    g_key_file_set_integer(viewer_ini, "CfgGlobal", "GroundZ", cfg.cfgGlobal.GroundZ);
    g_key_file_set_double(viewer_ini, "CfgGlobal", "ThresholdMinRad", cfg.cfgGlobal.ThresholdMinRad);

#endif // win32
    return 0;
}
