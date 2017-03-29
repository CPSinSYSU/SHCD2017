/***************************************************************************
 *
 *	实现 velodyneAlgo.h 中构造函数
 *	实现计算功能函数
 *		方向角
 *		半径和旋角的tan值
 *		Circle(垂直角度平面)R Z I均值 平均差等特征
 *		地面点
 *
 ***************************************************************************/

#include "velodyneAlgo.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <assert.h>
#include <algorithm>

// 绝对值
static inline float absf(float a)
{
    if (a < 0)
        return -a;
    return a;
}
// 距离
static float dist(VelodyneDataStruct::xpoint_t&a, VelodyneDataStruct::xpoint_t& b)
{
    return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

VelodyneAlgo::VelodyneAlgo()
{
}

VelodyneAlgo::~VelodyneAlgo()
{
}
/*
 *	TODO: 似乎是在计算方向角
 *	Direction Angles and Direction Cosines: http://www.geom.uiuc.edu/docs/reference/CRC-formulas/node52.html
 */
//float VelodyneAlgo::CalDirectionAngle(float XPos1, float YPos1, float XPos2, float YPos2)
//{
//    float delt_x = XPos2 - XPos1;
//    float delt_y = YPos2 - YPos1;
//    float angle;
//    float length = sqrt(delt_x*delt_x + delt_y*delt_y);
//    if (delt_x<0)
//        angle = 360 - acos(delt_y / length) * 180 / M_PI;
//    else
//        angle = acos(delt_y / length) * 180 / M_PI;
//    return angle;
//}
/*
 *  根据配置来计算scan 每个点的投影距离和与X轴夹角的正切值
 *  @param[IN] cfg       	配置信息
 *  @param[IN] scanobj  	要计算的scan对象
 *  @return     0:成功 	其他失败
 */
int VelodyneAlgo::CalcRadAndTheta(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj)
{
    // shot的数量
    int size = scanobj.shots.size();

    // 需要计算的shot的开始编号
    int start = cfg.cfgGlobal.ShotCalcFrom;
    int end;
    // 需要计算的shot的数量
    int CalcNum = cfg.cfgGlobal.ShotCalcNum;
    // 需要计算的shot的结束编号
    if (CalcNum == 0)
        CalcNum = size;
    end = CalcNum + start;
    assert((start<end) && (start>-1) && (end <= size));

    for (unsigned shot = start; shot<end; ++shot)
    {
        for (unsigned circle = 0; circle<CIRCLE_NUM; ++circle)
        {
            // 如果配置文件没有使用这个Circle，就跳过
            if (cfg.cfgCircleItems[circle].Enable == 0)
                continue;
            VelodyneDataStruct::xpoint_t& pt = scanobj.shots[shot].pt[circle];
            // 跳过无效点
            if (pt.point_type & POINT_TYPE_INVALID)
                continue;
            /*
             *	pt.rad: 激光点在xy平面投影到原点距离
             *	pt.tan_theta: 激光点在xy平面投影点与x轴夹角(旋角)正切值
             */
            pt.rad = sqrt(pt.x*pt.x + pt.y*pt.y);
            pt.tan_theta = pt.y / pt.x;
        }
    }
    return 0;
}
/*
 *	计算出地面点
 *	//TODO: 未实现
 */
//int VelodyneAlgo::CalcGroundPoint(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj){
//	typedef vector<xpoint_t*> vex;
//	return 0;
//}
/*
 *  根据配置来计算scan的特征量(以Circle为单位)
 *		Z I R 均值
 *		Z I R 平均差
 *		更新极值
 *			max_rad/min_rad
 *			max_x/min_x;max_y/min_y;max_z/min_z
 *			Z I R 平均差
 *			max_scanline_drtRad/min_scanline_drtRad
 *			max_scanline_drtZ/min_scanline_drtZ
 *  @param[IN] cfg       配置信息
 *  @param[IN] scanobj  要计算的scan对象
 *  @return   0:成功 其他失败
 */
//int VelodyneAlgo::CalcCircleFeature(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj)
//{
//    /*
//     *	配置文件中 ShotCalcFrom = ShotCalcNum = 0
//     *	GridWidth: 0~360 当前帧前后circleGridWidth帧数据
//     *	以自己为中心，临近点的个数(滑动窗口)
//     */
//    int GridWidth = scanobj.circleGridWidth * 2 + 1;
//    // 从配置计算出起始范围
//    int ShotSize = scanobj.shots.size();
//    int start = cfg.cfgGlobal.ShotCalcFrom;
//    int CalcNum = cfg.cfgGlobal.ShotCalcNum;
//    if (CalcNum == 0)
//        CalcNum = ShotSize;
//    int end = CalcNum + start;
//    assert((start<end) && (start>-1) && (end <= ShotSize));
//    // 初始化整个scan的统计变量
//    scanobj.InitStatistics(scanobj.scanStatistics);
//    // 计算出半径和正切值
//    VelodyneAlgo::CalcRadAndTheta(cfg, scanobj);
//
//    int i, j, k;
//    // 记录临近点的 Z I R 均值，和 Z I R 平均差
//    float aveZ, drtZ;
//    float aveRad, drtRad;
//    float aveI, drtI;
//    int count;
//    int pos;
//    int* dummy;
//    /*
//     *	滑动窗口
//     *	以Circle为单位
//     *	ZIR窗口均值 窗口平均差
//     */
//    for (j = 0; j<CIRCLE_NUM; ++j)
//    {
//        if (cfg.cfgCircleItems[j].Enable == 0)
//            continue;
//
//        // 初始化整个scan的统计变量
//        scanobj.InitStatistics(scanobj.circlesStatistics[j]);
//        for (i = start; i<end; ++i)
//        {
//            VelodyneDataStruct::xpoint_t& pt = scanobj.shots[i].pt[j];
//            if (pt.point_type & POINT_TYPE_INVALID)
//                continue;
//            aveZ = 0.0;
//            drtZ = 0.0;
//            aveRad = 0.0;
//            drtRad = 0.0;
//            aveI = 0.0;
//            drtI = 0.0;
//            count = 0;
//            // 计算均值(Z I R 均值)
//            for (k = 0; k < GridWidth; k++)
//            {
//                pos = i - scanobj.circleGridWidth + k;
//                if (pos > -1 && pos < ShotSize)
//                {
//                    VelodyneDataStruct::xpoint_t&  ptSlide = scanobj.shots[pos].pt[j];
//                    if (ptSlide.point_type&POINT_TYPE_INVALID)
//                        continue;
//                    aveZ += ptSlide.z;
//                    aveRad += ptSlide.rad;
//                    aveI += ptSlide.i;
//                    count++;
//                }
//            }
//            if (count <= 0)
//            {
//                continue;
//            }
//            else
//            {
//                aveZ /= (count *1.0);
//                aveRad /= (count *1.0);
//                aveI /= (count *1.0);
//            }
//            // 计算 Z I R 平均差
//            for (k = 0; k < GridWidth; k++)
//            {
//                pos = i - scanobj.circleGridWidth + k;
//                if (pos > -1 && pos < ShotSize)
//                {
//                    VelodyneDataStruct::xpoint_t& ptSlide = scanobj.shots[pos].pt[j];
//                    if (ptSlide.point_type & POINT_TYPE_INVALID)
//                        continue;
//                    drtZ += absf(ptSlide.z - aveZ);
//                    drtI += absf(ptSlide.i - aveI);
//                    drtRad += absf(ptSlide.rad - aveRad);
//                    count++;
//                }
//            }
//
//            drtZ /= (count * 1.0);
//            drtRad /= (count *1.0);
//            drtI /= (count *1.0);
//            pt.scanline_drtZ = drtZ;
//            pt.scanline_drtRad = drtRad;
//            pt.scanline_drtI = drtI;
//
//            /*
//             *	初始化统计量
//             *	计算每个Circle中[start,end]扫描线特征
//             *		x y z极值
//             *		高程平均差 半径平均差极值
//             */
//            dummy = reinterpret_cast<int*>(&(scanobj.circlesStatistics[j].max_rad));
//            if (*dummy == 0)
//            {
//                scanobj.circlesStatistics[j].max_rad = 0.0;
//                scanobj.circlesStatistics[j].min_rad = pt.rad;
//
//                scanobj.circlesStatistics[j].max_x = pt.x;
//                scanobj.circlesStatistics[j].min_x = pt.x;
//
//                scanobj.circlesStatistics[j].max_y = pt.y;
//                scanobj.circlesStatistics[j].min_y = pt.y;
//
//                scanobj.circlesStatistics[j].max_z = pt.z;
//                scanobj.circlesStatistics[j].min_z = pt.z;
//
//                scanobj.circlesStatistics[j].max_scanline_drtRad = pt.scanline_drtRad;
//                scanobj.circlesStatistics[j].min_scanline_drtRad = pt.scanline_drtRad;
//
//                scanobj.circlesStatistics[j].max_scanline_drtZ = pt.scanline_drtZ;
//                scanobj.circlesStatistics[j].min_scanline_drtZ = pt.scanline_drtZ;
//            }
//            // 更新极值
//            if (scanobj.circlesStatistics[j].max_rad < pt.rad)
//                scanobj.circlesStatistics[j].max_rad = pt.rad;
//
//            if (scanobj.circlesStatistics[j].min_rad > pt.rad)
//                scanobj.circlesStatistics[j].min_rad = pt.rad;
//
//            if (scanobj.circlesStatistics[j].max_x < pt.x)
//                scanobj.circlesStatistics[j].max_x = pt.x;
//
//            if (scanobj.circlesStatistics[j].min_x > pt.x)
//                scanobj.circlesStatistics[j].min_x = pt.x;
//
//            if (scanobj.circlesStatistics[j].max_y < pt.y)
//                scanobj.circlesStatistics[j].max_y = pt.y;
//
//            if (scanobj.circlesStatistics[j].min_y > pt.y)
//                scanobj.circlesStatistics[j].min_y = pt.y;
//
//            if (scanobj.circlesStatistics[j].max_z < pt.z)
//                scanobj.circlesStatistics[j].max_z = pt.z;
//
//            if (scanobj.circlesStatistics[j].min_z > pt.z)
//                scanobj.circlesStatistics[j].min_z = pt.z;
//
//            if (scanobj.circlesStatistics[j].max_scanline_drtRad < pt.scanline_drtRad)
//                scanobj.circlesStatistics[j].max_scanline_drtRad = pt.scanline_drtRad;
//
//            if (scanobj.circlesStatistics[j].min_scanline_drtRad > pt.scanline_drtRad)
//                scanobj.circlesStatistics[j].min_scanline_drtRad = pt.scanline_drtRad;
//
//            if (scanobj.circlesStatistics[j].max_scanline_drtZ < pt.scanline_drtZ)
//                scanobj.circlesStatistics[j].max_scanline_drtZ = pt.scanline_drtZ;
//
//            if (scanobj.circlesStatistics[j].min_scanline_drtZ > pt.scanline_drtZ)
//                scanobj.circlesStatistics[j].min_scanline_drtZ = pt.scanline_drtZ;
//        } // end of for (i = start; i<end; ++i){
//    } // end of for (j = 0; j<CIRCLE_NUM; ++j){
//    scanobj.calcCircleFeatured = true;
//    return 0;
//}
///*
// *  在格网中计算Circle的特征
// *  @param[IN] cfg       配置信息
// *  @param[IN] scanobj  要计算的scan对象
// *  @return     0:成功 其他失败
// */
//int VelodyneAlgo::CalcScanCellCircleFeature(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj)
//{
//    // 从配置计算出起始范围
//    int ShotSize = scanobj.shots.size();
//    int start = cfg.cfgGlobal.ShotCalcFrom;
//    int CalcNum = cfg.cfgGlobal.ShotCalcNum;
//    if (CalcNum == 0)
//        CalcNum = ShotSize;
//    int end = CalcNum + start;
//    assert((start<end) && (start>-1) && (end <= ShotSize));
//
//    // 以自己为中心，临近网格的个数
//    // int GridWidth = .circleGridWidth*2+1;
//    int GridWidth = 2 * 2 + 1;
//
//    int i, j, k;
//
//    /** 记录临近点的 Z I R 均值，和Z I R 均值平均差*/
//    float aveZ, drtZ;
//    float aveRad, drtRad;
//    float aveI, drtI;
//    int count;
//
//    int pos;
//    int *dummy;
//    for (j = 0; j<CIRCLE_NUM; ++j)
//    {
//        if (cfg.cfgCircleItems[j].Enable == 0)
//            continue;
//        scanobj.InitStatistics(scanobj.circlesStatistics[j]);
//        for (i = start; i<end; ++i)
//        {
//            VelodyneDataStruct::xpoint_t&  pt = scanobj.shots[i].pt[j];
//            if (pt.point_type & POINT_TYPE_INVALID)
//                continue;
//
//            aveZ = 0.0;
//            drtZ = 0.0;
//            aveRad = 0.0;
//            drtRad = 0.0;
//            aveI = 0.0;
//            drtI = 0.0;
//            count = 0;
//            //计算均值
//            for (k = 0; k< GridWidth; k++)
//            {
//                pos = i - scanobj.circleGridWidth + k;
//                if (pos>-1 && pos<ShotSize)
//                {
//                    VelodyneDataStruct::xpoint_t&  ptSlide = scanobj.shots[pos].pt[j];
//                    if (ptSlide.point_type & POINT_TYPE_INVALID)
//                        continue;
//                    aveZ += ptSlide.z;
//                    aveRad += ptSlide.rad;
//                    aveI += ptSlide.i;
//                    count++;
//                }
//            }
//            if (count <= 0)
//            {
//                continue;
//            }
//            else
//            {
//                aveZ /= (count *1.0);
//                aveRad /= (count *1.0);
//                aveI /= (count *1.0);
//            }
//            // 滑动窗口
//            for (k = 0; k< GridWidth; k++)
//            {
//                pos = i - scanobj.circleGridWidth + k;
//                if (pos>-1 && pos<ShotSize)
//                {
//                    VelodyneDataStruct::xpoint_t& ptSlide = scanobj.shots[pos].pt[j];
//                    if (ptSlide.point_type & POINT_TYPE_INVALID)
//                        continue;
//
//                    drtZ += absf(ptSlide.z - aveZ);
//                    drtI += absf(ptSlide.i - aveI);
//                    drtRad += absf(ptSlide.rad - aveRad);
//                    count++;
//                }
//            }
//            drtZ /= (count *1.0);
//            drtRad /= (count *1.0);
//            drtI /= (count *1.0);
//            pt.scanline_drtZ = drtZ;
//            pt.scanline_drtRad = drtRad;
//            pt.scanline_drtI = drtI;
//
//            // 初始化统计量
//            dummy = reinterpret_cast<int*>(&(scanobj.circlesStatistics[j].max_rad));
//            if (*dummy == 0)
//            {
//                scanobj.circlesStatistics[j].max_rad = 0.0;
//                scanobj.circlesStatistics[j].min_rad = pt.rad;
//
//                scanobj.circlesStatistics[j].max_x = pt.x;
//                scanobj.circlesStatistics[j].min_x = pt.x;
//
//                scanobj.circlesStatistics[j].max_y = pt.y;
//                scanobj.circlesStatistics[j].min_y = pt.y;
//
//                scanobj.circlesStatistics[j].max_z = pt.z;
//                scanobj.circlesStatistics[j].min_z = pt.z;
//
//                scanobj.circlesStatistics[j].max_scanline_drtRad = pt.scanline_drtRad;
//                scanobj.circlesStatistics[j].min_scanline_drtRad = pt.scanline_drtRad;
//
//                scanobj.circlesStatistics[j].max_scanline_drtZ = pt.scanline_drtZ;
//                scanobj.circlesStatistics[j].min_scanline_drtZ = pt.scanline_drtZ;
//            }
//
//            // 更新极值
//            if (scanobj.circlesStatistics[j].max_rad<pt.rad)
//                scanobj.circlesStatistics[j].max_rad = pt.rad;
//
//            if (scanobj.circlesStatistics[j].min_rad>pt.rad)
//                scanobj.circlesStatistics[j].min_rad = pt.rad;
//
//            if (scanobj.circlesStatistics[j].max_x<pt.x)
//                scanobj.circlesStatistics[j].max_x = pt.x;
//
//            if (scanobj.circlesStatistics[j].min_x>pt.x)
//                scanobj.circlesStatistics[j].min_x = pt.x;
//
//            if (scanobj.circlesStatistics[j].max_y<pt.y)
//                scanobj.circlesStatistics[j].max_y = pt.y;
//
//            if (scanobj.circlesStatistics[j].min_y>pt.y)
//                scanobj.circlesStatistics[j].min_y = pt.y;
//
//            if (scanobj.circlesStatistics[j].max_z<pt.z)
//                scanobj.circlesStatistics[j].max_z = pt.z;
//
//            if (scanobj.circlesStatistics[j].min_z>pt.z)
//                scanobj.circlesStatistics[j].min_z = pt.z;
//
//            if (scanobj.circlesStatistics[j].max_scanline_drtRad<pt.scanline_drtRad)
//                scanobj.circlesStatistics[j].max_scanline_drtRad = pt.scanline_drtRad;
//
//            if (scanobj.circlesStatistics[j].min_scanline_drtRad>pt.scanline_drtRad)
//                scanobj.circlesStatistics[j].min_scanline_drtRad = pt.scanline_drtRad;
//
//            if (scanobj.circlesStatistics[j].max_scanline_drtZ<pt.scanline_drtZ)
//                scanobj.circlesStatistics[j].max_scanline_drtZ = pt.scanline_drtZ;
//
//            if (scanobj.circlesStatistics[j].min_scanline_drtZ>pt.scanline_drtZ)
//                scanobj.circlesStatistics[j].min_scanline_drtZ = pt.scanline_drtZ;
//        }
//    }
//
//    scanobj.calcCircleFeatured = true;
//
//    return 0;
//}
///*
// *	计算连续线段
// */
//int VelodyneAlgo::CalcContinuousLines(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj)
//{
//    int GridWidth = scanobj.circleGridWidth * 2 + 1;
//    int ShotSize = scanobj.shots.size();
//    int start = cfg.cfgGlobal.ShotCalcFrom;
//    int CalcNum = cfg.cfgGlobal.ShotCalcNum;
//    if (CalcNum == 0)
//        CalcNum = ShotSize;
//    int end = CalcNum + start;
//    assert((start<end) && (start>-1) && (end <= ShotSize));
//
//    int i, j;
//    int count;
//    VelodyneDataStruct::continuousLine_t line;
//    for (j = 0; j <CIRCLE_NUM; j++)
//    {
//        CfgCircleItem_t& itemcfg = cfg.cfgCircleItems[j];
//        if (itemcfg.Enable == 0)
//            continue;
//        count = 0;
//        for (i = start; i<end; i++)
//        {
//            if (scanobj.shots[i].pt[j].point_type & POINT_TYPE_BELOW_DELTA_R)
//            {
//                if (count == 0)
//                {
//                    line.start = i;
//                }
//                ++count;
//            }
//            else
//            {
//                if (count != 0)
//                {
//                    line.end = i;
//                    VelodyneDataStruct::xpoint_t start = scanobj.shots[line.start].pt[j];
//                    VelodyneDataStruct::xpoint_t end = scanobj.shots[line.end].pt[j];
//                    int d = (int)dist(start, end);
//                    if (d > (float)cfg.cfgGlobal.ContinusLineMinDistInCircle)
//                        scanobj.circlesLines[j].push_back(line);
//                    count = 0;
//                }
//            }
//        }
//    }
//    scanobj.calcCirclesLined = true;
//
//    return 0;
//}
//
///*
// *	根据距离分类
// */
//int VelodyneAlgo::ClassifyByDeltaR(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj)
//{
//    int GridWidth = scanobj.circleGridWidth * 2 + 1;
//    int ShotSize = scanobj.shots.size();
//    int start = cfg.cfgGlobal.ShotCalcFrom;
//    int CalcNum = cfg.cfgGlobal.ShotCalcNum;
//    if (CalcNum == 0)
//        CalcNum = ShotSize;
//    int end = CalcNum + start;
//    assert((start<end) && (start>-1) && (end <= ShotSize));
//
//    int i, j;
//    for (j = 0; j <CIRCLE_NUM; j++)
//    {
//        CfgCircleItem_t& itemcfg = cfg.cfgCircleItems[j];
//        if (itemcfg.Enable == 0)
//            continue;
//        for (i = start; i<end; i++)
//        {
//            if (scanobj.shots[i].pt[j].point_type & POINT_TYPE_INVALID)
//                continue;
//            if (scanobj.shots[i].pt[j].rad < itemcfg.ThresholdMinDist*cfg.cfgGlobal.ThresholdMinRad)
//            {
//                scanobj.shots[i].pt[j].point_type |= POINT_TYPE_BELOW_R;
//            }
//            else
//            {
//                // 小于该值认为是平台路面，否则为路沿等
//                if (scanobj.shots[i].pt[j].scanline_drtRad>itemcfg.ThresholdRoadEdge)
//                    scanobj.shots[i].pt[j].point_type |= POINT_TYPE_ABOVE_DELTA_R;
//                else
//                    scanobj.shots[i].pt[j].point_type |= POINT_TYPE_BELOW_DELTA_R;
//            }
//        }
//    }
//    return 0;
//}
//
///*
// *	根据高程分类
// */
//int VelodyneAlgo::ClassifyByDeltaZ(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj)
//{
//    int GridWidth = scanobj.circleGridWidth * 2 + 1;
//    int ShotSize = scanobj.shots.size();
//
//    int start = cfg.cfgGlobal.ShotCalcFrom;
//    int CalcNum = cfg.cfgGlobal.ShotCalcNum;
//    if (CalcNum == 0)
//        CalcNum = ShotSize;
//    int end = CalcNum + start;
//    assert((start<end) && (start>-1) && (end <= ShotSize));
//
//    int i, j;
//    for (j = 0; j <CIRCLE_NUM; j++)
//    {
//        CfgCircleItem_t& itemcfg = cfg.cfgCircleItems[j];
//        if (itemcfg.Enable == 0)
//            continue;
//        for (i = start; i<end; i++)
//        {
//            if (scanobj.shots[i].pt[j].point_type & POINT_TYPE_INVALID)
//                continue;
//            if (scanobj.shots[i].pt[j].rad<itemcfg.ThresholdMinDist * cfg.cfgGlobal.ThresholdMinRad)
//            {
//                scanobj.shots[i].pt[j].point_type |= POINT_TYPE_BELOW_R;
//            }
//            else
//            {
//                // 小于该值认为是平台路面，否则为路沿等
//                if (scanobj.shots[i].pt[j].scanline_drtZ>itemcfg.ThresholdRoadEdgeByDeltaZ)
//                    scanobj.shots[i].pt[j].point_type |= POINT_TYPE_ABOVE_DELTA_Z;
//                else
//                    scanobj.shots[i].pt[j].point_type |= POINT_TYPE_BELOW_DELTA_Z;
//            }
//        }
//    }
//    return 0;
//}
///*
// *		利用通过连通图计算的scanobj.scanClusterArray集群信息
// *		计算每个垂直角度平面上障碍物扇形段集合(集群线)
// */
//int VelodyneAlgo::CalClusterLineAndFeature(VelodyneDataStruct& scanobj)
//{
//    scanobj.scanClusterLineArray.clear();
//    scanobj.scanClusterLineFeatureArray.clear();
//
//    scanobj.scanClusterLineArray.resize(scanobj.scanPieClusterSet.size());
//    scanobj.scanClusterLineFeatureArray.resize(scanobj.scanPieClusterSet.size());
//    for (int i = 0; i<scanobj.scanPieClusterSet.size(); i++)
//    {
//        /*
//         *	64个circle 代表64种不同的垂直角度
//         *	scanobj.scanClusterLineArray[i][CircleID(可能没有0~63)][0或者以上]: 每个垂直角度平面上障碍物扇形段集合
//         */
//        TransferClusterToLineStructure(scanobj.scanPieClusterSet[i], scanobj.scanClusterLineArray[i], scanobj);
//        //CalClusterLineFeature(scanobj.scanClusterLineArray[i], scanobj.scanClusterLineFeatureArray[i]);
//        //CalClusterLShapeFeature(scanobj.scanClusterLineArray[i],scanobj.scanClusterLineFeatureArray[i]);
//    }
//    return 0;
//}
///*
// *	按照shotID(packet接收的先后顺序)由小到大排序
// */
//bool cmpByShotNum(VelodyneDataStruct::xpoint_t* a, VelodyneDataStruct::xpoint_t* b)
//{
//    return a->shotID < a->shotID;
//}
///*
// *	获取集群(障碍物集群)中每个Circle上的连续线段
// */
//int VelodyneAlgo::TransferClusterToLineStructure(VelodyneDataStruct::pieCluster_t& clu, VelodyneDataStruct::clusterLine_t& cline, VelodyneDataStruct& scanobj)
//{
//    /*
//     *	typedef vector<line_t> circleLine_t;
//     *	typedef vector<xpoint_t*> line_t;
//     *	xpoint_t: 记录一个激光点的所有数据
//     */
//    VelodyneDataStruct::circleLine_t allScanCircleLine;
//    VelodyneDataStruct::circleLine_t validScanCircleLine;
//    allScanCircleLine.resize(CIRCLE_NUM);
//
//    int i, j, k;
//    float threshod = 80;
//
//    for (j = 0; j <clu.size(); j++)
//    {
//        /*
//         *	pCell: 一个格网单元
//         *	circleID: 垂直平面高-->低(0-CIRCLE_NUM)
//         */
//        VelodyneDataStruct::cell_t* pcellObj = clu[j]->pCell;
//        int pointSize = pcellObj->size();
//        for (k = 0; k<pointSize; k++)
//        {
//            int id = (*pcellObj)[k]->circleID;
//            VelodyneDataStruct::xpoint_t& pt = *(*pcellObj)[k];
//            if (pt.point_type & POINT_TYPE_INVALID)
//                continue;
//            allScanCircleLine[id].push_back(&pt);
//        }
//    }
//    /*
//     *	每个circle的点云必须存在集群中，否则为无效的circle(该垂直角度平面不存在连续线段)
//     *	validScanCircleLine[i]构成一个垂直角度平面
//     */
//    for (i = 0; i<allScanCircleLine.size(); i++)
//    {
//        if (allScanCircleLine[i].size()>1)
//            validScanCircleLine.push_back(allScanCircleLine[i]);
//    }
//    // validScanCircleLine[CircleID][ShotID]
//    allScanCircleLine.clear();
//    cline.resize(validScanCircleLine.size());
//
//    // 集群中包含的各个Circle的点云数据 --> 寻找这些Circle平面上的连续线段
//    for (i = 0; i<validScanCircleLine.size(); i++)
//    {
//        // 每个circle里面的点云数据(同一个垂直角度平面)按照shortID(帧号，即点云数据接收的先后顺序)有小到大排序
//        sort(validScanCircleLine[i].begin(), validScanCircleLine[i].end(), cmpByShotNum);
//
//        // 寻找各个Circle中的连续线段
//        VelodyneDataStruct::line_t circleLineSegment;
//        VelodyneDataStruct::circleLine_t scanCircleLine;
//        /*
//         *	插入起始帧
//         *	同一垂直角度平面前后两帧点云数据距离平面原点距离过大(>80cm) 不能算作连续线段
//         *	scanCircleLine: 同一垂直平面可以连在一块的障碍信息
//         *	circleLineSegment[i]: 同一垂直角度平面内连续线段上的点
//         *	scanCircleLine[i]: 同一垂直角度平面内的连续线段集合
//         */
//        circleLineSegment.push_back(validScanCircleLine[i][0]);
//        for (j = 1; j<validScanCircleLine[i].size(); j++)
//        {
//            // 收藏连续线段上的点
//            if (absf(validScanCircleLine[i][j]->rad - validScanCircleLine[i][j - 1]->rad) < threshod)
//                circleLineSegment.push_back(validScanCircleLine[i][j]);
//            // 收藏Circle上的连续线段
//            if (absf(validScanCircleLine[i][j]->rad - validScanCircleLine[i][j - 1]->rad) >= threshod
//                    || j == validScanCircleLine[i].size() - 1)
//            {
//                scanCircleLine.push_back(circleLineSegment);
//                circleLineSegment.clear();
//                circleLineSegment.push_back(validScanCircleLine[i][j]);
//            }
//        }
//        /*
//         *	scanCircleSegNum: 当前Circle中连续线段数目
//         *	lastSegPtNum: 某段线段由?个激光点(几帧)连成
//         */
//        int scanCircleSegNum = scanCircleLine.size();
//        int lastSegPtNum = scanCircleLine[scanCircleSegNum - 1].size();
//        // 首尾两条连续线段能够连在一起!!
//        if (scanCircleLine.size()>1
//                && absf(scanCircleLine[0][0]->rad - scanCircleLine[scanCircleSegNum - 1][lastSegPtNum - 1]->rad) < threshod)
//        {
//            /*
//             *	firstSegPtNum: 最后一个扇形段由?个激光点(几帧)连成
//             *	将首段扇形段按照顺/逆时针拼接在末段扇形段
//             *	拼接后再合并当前垂直角度平面的扇形段
//             */
//            int firstSegPtNum = scanCircleLine[0].size();
//            for (k = 0; k<firstSegPtNum; k++)
//                scanCircleLine[scanCircleSegNum - 1].push_back(scanCircleLine[0][k]);
//            // 首段因为连接在尾段上 故丢弃
//            for (int m = 1; m<scanCircleSegNum; m++)
//            {
//                cline[i].push_back(scanCircleLine[m]);
//            }
//        }
//        // 无需合并 直接收藏当前Circle的连续线段集合
//        else
//        {
//            cline[i] = scanCircleLine;
//        }
//    }
//    return 0;
//}
///*
// *	计算集群中同一Circle上的连续线段的特征
// *		每个Circle的连续线段总长度 最长线段长度
// *		所有Circle上的连续线段总长度 最长Circle线段总长度
// *		平均Circle总长度
// */
//int VelodyneAlgo::CalClusterLineFeature(VelodyneDataStruct::clusterLine_t& clu_line, VelodyneDataStruct::clusterLineFeature_t& clu_lineFeature)
//{
//    int i, j, k, m, n;
//    // 初始化
//    clu_lineFeature.circleLineFeature.resize(clu_line.size());
//    clu_lineFeature.averageCircleLineLength = 0;
//    clu_lineFeature.validCircleNum = 0;
//    clu_lineFeature.totalLength = 0;
//    clu_lineFeature.maxSegLength = 0;
//    clu_lineFeature.maxCircleLineLength = 0;
//    clu_lineFeature.ptNum = 0;
//    clu_lineFeature.aveX = 0;
//    clu_lineFeature.aveZ = 0;
//    clu_lineFeature.aveY = 0;
//    clu_lineFeature.Lshape = 0;
//    clu_lineFeature.averageCircleLineLength = 0;
//    for (i = 0; i<4; i++)
//        for (j = 0; j<2; j++)
//            clu_lineFeature.LshapeBoxVex[i][j] = 0;
//    clu_lineFeature.LshapeDirection = 0;
//    clu_lineFeature.LshapeLength = 0;
//    clu_lineFeature.LshapeWidth = 0;
//    clu_lineFeature.LshapeDeflection = 0;
//
//    for (i = 0; i<clu_line.size(); i++)
//    {
//        clu_lineFeature.validCircleNum++;
//        clu_lineFeature.circleLineFeature[i].segNum = clu_line[i].size();
//        clu_lineFeature.circleLineFeature[i].totalLength = 0;
//        clu_lineFeature.circleLineFeature[i].ptNum = 0;
//        clu_lineFeature.circleLineFeature[i].maxSegLength = 0;
//        clu_lineFeature.circleLineFeature[i].aveZ = 0;
//        clu_lineFeature.circleLineFeature[i].aveX = 0;
//        clu_lineFeature.circleLineFeature[i].aveY = 0;
//        clu_lineFeature.circleLineFeature[i].Lshape = 0;
//        for (m = 0; m<4; m++)
//            for (n = 0; n<2; n++)
//                clu_lineFeature.circleLineFeature[i].LshapeBoxVex[m][n] = 0;
//        clu_lineFeature.circleLineFeature[i].LshapeDirection = 0;
//        clu_lineFeature.circleLineFeature[i].LshapeLength = 0;
//        clu_lineFeature.circleLineFeature[i].LshapeWidth = 0;
//        clu_lineFeature.circleLineFeature[i].LshapeDeflection = 0;
//        // 遍历每个Circle里面的每条连续线段
//        for (j = 0; j<clu_line[i].size(); j++)
//        {
//            VelodyneDataStruct::clusterLineSegFeature_t tempSegFeature;
//            tempSegFeature.ptNum = clu_line[i][j].size();
//            // 线段起点/终点 方向角
//            tempSegFeature.startAngle = CalDirectionAngle(0, 0, clu_line[i][j][0]->x, clu_line[i][j][0]->y);
//            tempSegFeature.endAngle = CalDirectionAngle(0, 0, clu_line[i][j][clu_line[i][j].size() - 1]->x, clu_line[i][j][clu_line[i][j].size() - 1]->y);
//            if (tempSegFeature.endAngle < tempSegFeature.startAngle)
//                tempSegFeature.endAngle += 360;
//            // 路线长度 X/Y/Z均值
//            tempSegFeature.length = 0;
//            tempSegFeature.aveZ = clu_line[i][j][0]->z;
//            tempSegFeature.aveX = clu_line[i][j][0]->x;
//            tempSegFeature.aveY = clu_line[i][j][0]->y;
//            for (k = 1; k<clu_line[i][j].size(); k++)
//            {
//                float xLength = clu_line[i][j][k]->x - clu_line[i][j][k - 1]->x;
//                float yLength = clu_line[i][j][k]->y - clu_line[i][j][k - 1]->y;
//                tempSegFeature.length += sqrt(xLength*xLength + yLength*yLength);
//
//                tempSegFeature.aveZ += clu_line[i][j][k]->z;
//                tempSegFeature.aveX += clu_line[i][j][k]->x;
//                tempSegFeature.aveY += clu_line[i][j][k]->y;
//            }
//            // 计算整个Circle的X/Y/Z均值
//            clu_lineFeature.circleLineFeature[i].aveX += tempSegFeature.aveX;
//            clu_lineFeature.circleLineFeature[i].aveY += tempSegFeature.aveY;
//            clu_lineFeature.circleLineFeature[i].aveZ += tempSegFeature.aveZ;
//            // 计算Circle中每一条连续线段的X/Y/Z均值
//            tempSegFeature.aveX /= tempSegFeature.ptNum;
//            tempSegFeature.aveY /= tempSegFeature.ptNum;
//            tempSegFeature.aveZ /= tempSegFeature.ptNum;
//            // 计算每个Circle中连续线段的激光点总数目
//            clu_lineFeature.circleLineFeature[i].ptNum += tempSegFeature.ptNum;
//            // 计算每个Circle中连续线段组成的总长度
//            clu_lineFeature.circleLineFeature[i].totalLength += tempSegFeature.length;
//            // 寻找每个Circle的最长连续线段
//            if (clu_lineFeature.circleLineFeature[i].maxSegLength < tempSegFeature.length)
//                clu_lineFeature.circleLineFeature[i].maxSegLength = tempSegFeature.length;
//
//            clu_lineFeature.circleLineFeature[i].lineSegFeature.push_back(tempSegFeature);
//        } // end of for (j = 0; j<clu_line[i].size(); j++){
//        // 每个Circle首段起点 尾段结点的方向角
//        clu_lineFeature.circleLineFeature[i].startAngle = clu_lineFeature.circleLineFeature[i].lineSegFeature[0].startAngle;
//        clu_lineFeature.circleLineFeature[i].endAngle = clu_lineFeature.circleLineFeature[i].lineSegFeature[clu_line[i].size() - 1].endAngle;
//        // 所有Circle上所有连续线段所有点的 X/Y/Z均值
//        clu_lineFeature.aveZ += clu_lineFeature.circleLineFeature[i].aveZ;
//        clu_lineFeature.aveY += clu_lineFeature.circleLineFeature[i].aveY;
//        clu_lineFeature.aveX += clu_lineFeature.circleLineFeature[i].aveX;
//
//        clu_lineFeature.circleLineFeature[i].aveZ /= clu_lineFeature.circleLineFeature[i].ptNum;
//        clu_lineFeature.circleLineFeature[i].aveY /= clu_lineFeature.circleLineFeature[i].ptNum;
//        clu_lineFeature.circleLineFeature[i].aveX /= clu_lineFeature.circleLineFeature[i].ptNum;
//        // 所有Circle上所有连续线段总长度
//        clu_lineFeature.totalLength += clu_lineFeature.circleLineFeature[i].totalLength;
//        // 所有Circle上所有线段包含的点的数目
//        clu_lineFeature.ptNum += clu_lineFeature.circleLineFeature[i].ptNum;
//        // 记录最长线段的长度
//        if (clu_lineFeature.maxSegLength<clu_lineFeature.circleLineFeature[i].maxSegLength)
//            clu_lineFeature.maxSegLength = clu_lineFeature.circleLineFeature[i].maxSegLength;
//        // 记录连续线段总长度最大的Circle 其总长度
//        if (clu_lineFeature.maxCircleLineLength<clu_lineFeature.circleLineFeature[i].totalLength)
//            clu_lineFeature.maxCircleLineLength = clu_lineFeature.circleLineFeature[i].totalLength;
//    } // end of for (i = 0; i<clu_line.size(); i++){
//    // 集群中所有连续线段的平均Circle长度
//    clu_lineFeature.averageCircleLineLength = clu_lineFeature.totalLength / clu_lineFeature.validCircleNum;
//
//    clu_lineFeature.aveX /= clu_lineFeature.ptNum;
//    clu_lineFeature.aveY /= clu_lineFeature.ptNum;
//    clu_lineFeature.aveZ /= clu_lineFeature.ptNum;
//    return 0;
//}

/*
 *
 */
//int VelodyneAlgo::CalClusterLShapeFeature(clusterLine_t& clu_line, clusterLineFeature_t& clu_lineFeature){
//	int i;
//	for (i = 0; i<clu_line.size(); i++){
//		circleLine_t &cl = clu_line[i];
//		clusterCircleLineFeature_t &clf = clu_lineFeature.circleLineFeature[i];
//		//LshapeLine scanLshapeLine;
//		//scanLshapeLine.CalBestLMatchBox(clf, cl);
//		//scanLshapeLine.DrawBestLMatchBox(clf);
//	}
//	return 0;
//}

