#ifndef __VELODYNE_ALGO_H__
#define __VELODYNE_ALGO_H__

/***************************************************************************
 *
 *	Alogrithms used to process laser data
 *		Obstacle detection
 *
 ***************************************************************************/

#include "configStruct.h"
#include "velodyneDataStruct.h"
#include <stdio.h>
/*
 *	bit vector
 *	 charv: 一维向量
 *	 charvv: 二维向量
 */
typedef std::vector<bool> bitv;
typedef std::vector<bitv> bitvv;

/*
 *	所有处理激光点数据的算法都放在这个类里面
 */
class VelodyneAlgo
{
private:
    // @brief CTor
    VelodyneAlgo();
    // @brief DTor
    ~VelodyneAlgo();
public:
    static float CalDirectionAngle(float XPos1, float YPos1, float XPos2, float YPos2);
    // @brief 计算半径和旋角的tan值
    static int CalcRadAndTheta(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj);

    // @brief 计算出地面点
    //static int CalcGroundPoint(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj);

    // @brief 计算R Z I等均值/平均差特征(Circle滑动窗口)
    static int CalcCircleFeature(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj);
    // @brief 在网格中的Circle计算特征
    static int CalcScanCellCircleFeature(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj);

    // @brief 计算连续线段
    static int CalcContinuousLines(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj);

    // @brief 根据距离分类
    static int ClassifyByDeltaR(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj);
    // @brief 根据高程分类
    static int ClassifyByDeltaZ(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj);

    // @brief 转换成单线储存结构
    static int CalClusterLineFeature(VelodyneDataStruct::clusterLine_t& clu_line, VelodyneDataStruct::clusterLineFeature_t& clu_lineFeature);
    static int TransferClusterToLineStructure(VelodyneDataStruct::pieCluster_t& clu, VelodyneDataStruct::clusterLine_t& cline, VelodyneDataStruct& scanobj);
    static int CalClusterLineAndFeature(VelodyneDataStruct& scanobj);
    //static int CalClusterLShapeFeature(clusterLine_t& clu_line, clusterLineFeature_t& clu_lineFeature);


    /**************************** 以下为格网形式的处理函数 ****************************/
    // @brief 转换成格网表示，标记出平面和非平面
    static int TransferToPieMatrix(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj);

    // @brief 计算单元网格 R Z I等均值平均差特征
    static int CalcPieMatrixFeature(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj);
    // @brief 网格单个 R Z I等均值平均差特征
    static int CalcSinglePieCellFeature(CfgVeloView_t& cfg, VelodyneDataStruct::cell_t& pieobj, VelodyneDataStruct::pieFeature_t& pie_feature);

    // @brief 在网格中寻找相关联的网格簇
    static int FindAndCalcPieClusterFeature(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj, bool findClosestOnly = false);
    // @brief 网格周围寻找相同类型网格
    static int SearchPieCluster(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj, VelodyneDataStruct::pieCluster_t& pie_clu, bitvv& isSearchedvv, int azimuth_idx, int rad_idx);
    // @brief 计算网格簇特征
    static int CalcSinglePieClusterFeature(VelodyneDataStruct::pieCluster_t& pie_clu, VelodyneDataStruct::pieClusterFeature_t& pie_cluFeature);


    /**************************** 以下为方格形式的处理函数 ****************************/
    // @brief 转换成方格表示，标记出平面和非平面
    //static int TransferToGridMatrix(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj);
    static int TransferToGridMatrix(CfgVeloView_t& cfg, VelodyneDataStruct& calobj);

    // @brief 计算单元方格 R Z I等均值 平均差特征
    static int CalcGridMatrixFeature(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj);
    // @brief 单个方格 R Z I等均值 平均差特征计算
    static int CalcSingleGridFeature(CfgVeloView_t& cfg, VelodyneDataStruct::cell_t& gridobj, VelodyneDataStruct::gridFeature_t& grid_feature);

    // @brief 在方格中寻找障碍方格集群并计算集群特征信息
    static int FindAndCalcGridClusterFeature(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj);
    // @brief 递归寻找障碍方格集群
    static int SearchGridCluster(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj, VelodyneDataStruct::gridCluster_t& grid_clu, bitvv& isSearchedvv, int row, int grid);
    // @brief 计算单个障碍物方形网格集群的特征信息
    static int CalcSingleGridClusterFeature(VelodyneDataStruct::gridCluster_t& grid_clu, VelodyneDataStruct::gridClusterFeature_t& grid_cluFeature);
};

#endif

