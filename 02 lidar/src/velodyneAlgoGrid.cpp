/***************************************************************************
 *
 *	实现 velodyneAlgo.h 中方格形式的处理函数部分
 *
 ***************************************************************************/
#include "velodyneAlgo.h"
#include <stdio.h>
#ifdef WIN32
    #include <set>
#else
    #include <vector>
    #include <algorithm>
#endif

static inline float absf(float a)
{
    if (a < 0)
        return -a;
    return a;
}
/*
 *	根据配置来把点放入方型网格 vs VelodyneAlgo::TransferToCellArray(CfgVeloView& cfg, scan& scanobj)
 *	方型网格构建在车体周边
 *	@param[IN] cfg    	配置信息
 *	@param[IN] scanobj	要计算的scan对象
 *	@return     0:成功 其他失败
 *	网格大小: (GridROW + GridBackROW)*GridNum
 */
int VelodyneAlgo::TransferToGridMatrix(CfgVeloView_t& cfg, VelodyneDataStruct& calobj)
{
    /*
     *	GridROW: 车头前方单元格行数
     *	GridBackROW: 车头后方单元格(//TODO: 从最后方起，沿车前进的方向，第0行、第1行...)
     *	GridNum: 单元格数量(网格为长方形)
     *	配置文件中 GridNum=200
     */
    int RowNumber = cfg.cfgPlaneDetect.GridHeadRowNum + cfg.cfgPlaneDetect.GridBackRowNum;
    if (RowNumber == 0)
    {
        printf("RowNumber == 0...\n");
        return -1;
    }

    int ColumnNumber = cfg.cfgPlaneDetect.GridColumnNum; 									// 每列有100个方格
    int GridCellSize = cfg.cfgPlaneDetect.GridCellSize;										// 每个方格为边长为25cm的正方形
    int Max_Y = cfg.cfgPlaneDetect.GridHeadRowNum * GridCellSize;							// 只考虑0-25米的范围
    int Min_Y = -1 * (cfg.cfgPlaneDetect.GridBackRowNum * GridCellSize);
    int Max_X = ColumnNumber * GridCellSize / 2;
#if 0
    printf("RowNumber=%d ", RowNumber);
    printf("ColumnNumber=%d ", ColumnNumber);
    printf("GridCellSize=%d ", GridCellSize);
    printf("Max_Y=%d ", Max_Y);
    printf("Min_Y=%d ", Min_Y);
    printf("Max_X=%d\n", Max_X);
#endif

    // 根据是否要投影后方点来选择需要投影的点的范围
    int shotSize = calobj.shots.size();
    #if 0
    printf("shotSize=%d\n", shotSize);
    #endif
    int shotStart, shotEnd;
    /*
     *	不需要投影后方点
     *	velodyneDriver.cpp -> isNewScan: 每一帧的边界从180度开始，即车的后面
     *	需要考虑激光雷达顺时针还是逆时针旋转
     */
    if (Min_Y == 0)
    {
        shotStart = shotSize / 4 - 20;
        shotEnd = (shotSize / 4) * 3 + 20;
    }
    // 需要投影后方点
    else
    {
        shotStart = 0;
        shotEnd = shotSize;
    }
    /*
     *	分配二维数组空间
     *	先行后列
     *	行是沿y正方向，列是沿x负方向
     */
    calobj.scanGridArray.resize(RowNumber);
    for (unsigned row = 0; row<RowNumber; ++row)
        calobj.scanGridArray[row].resize(ColumnNumber);

    bool ignorecircle;
    /*
     *	将点映射到 ColumnSize(x) * GridNumber(y)的方格网
     *		Min_X <= x <= Max_X
     *	   -Max_Y <= y <= Max_Y
     */
    int y_row_idx, x_col_idx;
    //for (unsigned shot=shotStart; shot<shotEnd; ++shot)
    for (unsigned shot=shotStart; shot<shotEnd; ++shot)
    {
        // shot中的每个点按照垂直角度平面由低到高
        for (unsigned circle = 0; circle<CIRCLE_NUM; ++circle)
        {
            /*
             *	投影到方型网格时，当前垂直角度平面上的激光点是否忽略
             *	配置文件中 有32个忽略配置项
             */
            ignorecircle = false;
            for (unsigned i = 0; i<32; i++)
            {
                if (circle == cfg.cfgIgnoreCircleWhenProjection.IgnoreCircle[i])
                {
                    ignorecircle = true;
                    break;
                }
            }
            if (ignorecircle)
                continue;
            /*
             *	将满足上述定义的车周边网格以内的激光点投影到方型网格中
             *	y_row_idx: 行号 投影到y轴
             *	x_col_idx: 列号 投影到x轴
             */
            VelodyneDataStruct::xpoint_t& pt = calobj.shots[shot].pt[circle];
            if (pt.point_type & POINT_TYPE_INVALID)
                continue;
            /* move to velodyneDriver
            pt.circleID = circle;
            */
            if (pt.y >= Min_Y && pt.y <= Max_Y)
            {
                if (pt.y <= 0)
                    // 最小为0
                    y_row_idx = (pt.y - Min_Y) / GridCellSize;
                else
                    y_row_idx = pt.y / GridCellSize + cfg.cfgPlaneDetect.GridBackRowNum;

                if (pt.y == Max_Y)
                {
                    y_row_idx = y_row_idx - 1;			// 防止越界
                }
                if (pt.x >= -Max_X && pt.x <= Max_X)
                {
                    x_col_idx = (Max_X - pt.x) / GridCellSize;
                    if (pt.y == -Max_X)
                        x_col_idx = x_col_idx - 1;		// 防止越界

                    calobj.scanGridArray[y_row_idx][x_col_idx].push_back(&pt);
                }
            }
        } // end of for (unsigned circle = 0; circle<CIRCLE_NUM; ++circle){
    } // end of for (unsigned shot = shotStart; shot<shotEnd; ++shot){

#if 0
    for (unsigned row = 0; row < RowNumber; row++)
    {
        for (unsigned grid = 0; grid < ColumnNumber; grid++)
        {
            if (scanobj.scanGridArray[row][grid].size() > 0)
            {
                printf("[%d-%d] ", row, grid);
            }
        }
        printf("\n");
    }
    printf("end of a scan......Grid\n");
#endif

    return 0;
}
/*
 *	计算方型网格矩阵: x/y/z均值、x/y/z极值、高程平均差
 *	调用 CalcGridFeature 计算单个网格
 */
int VelodyneAlgo::CalcGridMatrixFeature(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj)
{
    if (scanobj.scanGridArray.size() == 0)
        return -1;

    int RowSize = scanobj.scanGridArray.size();
    int ColumnSize = scanobj.scanGridArray[0].size();

    if (scanobj.scanGridFeatureArray.size() == 0) 				// 分配空间
    {
        scanobj.scanGridFeatureArray.resize(RowSize);
        for (unsigned row = 0; row<RowSize; ++row)
            scanobj.scanGridFeatureArray[row].resize(ColumnSize);
    }

    for (unsigned row = 0; row<RowSize; row++)
    {
        // 整个一周
        VelodyneDataStruct::gridRow_t& rowObj = scanobj.scanGridArray[row];
        ColumnSize = rowObj.size();
        for (unsigned grid = 0; grid<ColumnSize; grid++)
        {
            VelodyneDataStruct::cell_t& cellObj = scanobj.scanGridArray[row][grid];
            VelodyneDataStruct::gridFeature_t& grid_feature = scanobj.scanGridFeatureArray[row][grid];
            // TODO: 与SICK激光雷达相关
            //if (gfeature.cellType & CUDE_TYPE_CONTAIN_SICK)
            //	continue;
            #if 0
            if(cellObj.size() > 0)
                printf("%d-%d point-size:%d\n", row, grid, cellObj.size());
            #endif
            CalcSingleGridFeature(cfg, cellObj, grid_feature);

            grid_feature.cellType |= CELL_TYPE_IN_OBSTACLE_RANGE;
        }
    }
    return 0;
}
/*
 *	计算单个Grid的x/y/z均值、x/y/z极值、高程平均差总和(delta_z)特征(遍历单个网格中的所有点)
 *	gridobj: 单个Grid，包括网格内所有点集合
 *	gfeature: Grid特征信息集合
 */
int VelodyneAlgo::CalcSingleGridFeature(CfgVeloView_t& cfg, VelodyneDataStruct::cell_t& cellobj, VelodyneDataStruct::gridFeature_t& grid_feature)
{
    // 方型网格中点的个数
    grid_feature.size = cellobj.size();
    grid_feature.cellType = CELL_TYPE_INITIAL;
    // 如果网格中一个点没有，则无效
    if (grid_feature.size == 0)
    {
        grid_feature.cellType |= CELL_TYPE_INVALID;
        return -1;
    }

    // circleSet 用来统计有多少个circle，upperCircleSet用来数有多少个垂直角度向上打(verticalAngle>0)的点
#ifdef WIN32
    std::set<int> circleSet;
    std::set<int> upperCircleSet;
#else
    std::vector<int> circleSet;
    std::vector<int> upperCircleSet;
#endif

    /*
     *	GroundZ: 地面高程(-225cm ---> 雷达基坐标高度)
     *	ThresholdMinHollow: 中空网格判断 最小值(50cm)
     *	ThresholdMaxHollow: 中空网格判断 最大值(150cm)
     */
    float hollowMin = cfg.cfgGlobal.GroundZ + cfg.cfgGlobal.ThresholdMinHollow;
    float hollowMax = cfg.cfgGlobal.GroundZ + cfg.cfgGlobal.ThresholdMaxHollow;
    /*
     *	找到高程最大值，先忽略,用来解决异常点的问题
     *	这个方法比较粗暴，应该用更合适的算法来检测异常点
     */
    int outlier_pt;
    float max_z_except_outlier;
    /*
     *	计算平均x/y/z
     *	计算高程与平均高程的偏差和
     */
    grid_feature.ave_x = grid_feature.ave_y = grid_feature.ave_z = 0.0;
    grid_feature.delta_z = 0;
    // 遍历所有的点，计算特征
    for (unsigned pt = 0; pt<grid_feature.size; ++pt)
    {
        // 求均值
        grid_feature.ave_x += cellobj[pt]->x;
        grid_feature.ave_y += cellobj[pt]->y;
        grid_feature.ave_z += cellobj[pt]->z;
        // 如果有一个点，在距离阈值内，那么整个网格在阈值距离类(方形网格矩阵未做此标记)
        if (cellobj[pt]->point_type & POINT_TYPE_BELOW_R)
            grid_feature.cellType |= CELL_TYPE_BELOW_R;
        // circle计数
        #ifdef WIN32
        circleSet.insert(cellobj[pt]->circleID);
        #else
        if(std::find(circleSet.begin(), circleSet.end(), cellobj[pt]->circleID) == circleSet.end()){
            //printf("circleSet.insert\n");
            circleSet.push_back(cellobj[pt]->circleID);
        }
        #endif
        // 水平向上打的circle计数，用于判断固定障碍物
        #ifdef WIN32
        if (cellobj[pt]->circleID > UPPER_VERT_FROM){
            upperCircleSet.insert(cellobj[pt]->circleID);
        }
        #else
        if (cellobj[pt]->circleID > UPPER_VERT_FROM){
            if(std::find(upperCircleSet.begin(), upperCircleSet.end(),
                         cellobj[pt]->circleID) == upperCircleSet.end()){
                //printf("upperCircleSet.insert\n");
                upperCircleSet.push_back(cellobj[pt]->circleID);
            }
        }
        #endif // WIN32
        /*
         * Positive Obstacle的关键就是除去中空情况，方法:
         *	1. min_z > ...
         *	2.
         * 非中空 在校园里面，树为中空(高度均在<=hollowMin和>=hollowMax为中空)
         *  只要存在一个点在hollowMin-hollowMax之间 则认为该网格非中空
         *	只要认为是中空 就必须检查每个点 直到所有点都通过检测 则可认为是中空的
         */
        if (!(grid_feature.cellType & CELL_TYPE_NOT_HOLLOW))
        {
            if (cellobj[pt]->z > hollowMin && cellobj[pt]->z < hollowMax)
                grid_feature.cellType |= CELL_TYPE_NOT_HOLLOW;
        }
        // 统计最值 同时寻找异常点(暂时认为最大高程为异常点)
        if (pt == 0)
        {
            // 赋初值
            outlier_pt = 0;
            grid_feature.min_x = grid_feature.max_x = cellobj[pt]->x;
            grid_feature.min_y = grid_feature.max_y = cellobj[pt]->y;
            max_z_except_outlier = grid_feature.min_z = grid_feature.max_z = cellobj[pt]->z;
        }
        else
        {
            // 记录最值
            if (grid_feature.max_x < cellobj[pt]->x)
                grid_feature.max_x = cellobj[pt]->x;
            if (grid_feature.min_x > cellobj[pt]->x)
                grid_feature.min_x = cellobj[pt]->x;

            if (grid_feature.max_y < cellobj[pt]->y)
            {
                grid_feature.max_y = cellobj[pt]->y;
                grid_feature.max_y_x = cellobj[pt]->x;
            }
            if (grid_feature.min_y > cellobj[pt]->y)
            {
                grid_feature.min_y = cellobj[pt]->y;
                grid_feature.min_y_x = cellobj[pt]->x;
            }

            // 暂且认为是异常点，然后在后面进一步判决
            if (grid_feature.max_z < cellobj[pt]->z)
            {
                max_z_except_outlier = grid_feature.max_z;
                grid_feature.max_z = cellobj[pt]->z;
                outlier_pt = pt;
            }
            if (grid_feature.min_z > cellobj[pt]->z)
                grid_feature.min_z = cellobj[pt]->z;
        }
    } // end of for (unsigned pt = 0; pt<grid_feature.size; ++pt){

    /*
     *	确定网格里面有1个以上的点，去除outlier点
     *		计算平均高程
     *		计算平均高程
     *		异常点判决，更新max_z
     */
    if (grid_feature.size > 1)
    {
        float tmp_ave_z = grid_feature.ave_z - cellobj[outlier_pt]->z;
        tmp_ave_z /= (grid_feature.size - 1)*1.0;
        // 异常点与其他点平均高程差距超过50cm
        if (cellobj[outlier_pt]->z - tmp_ave_z < 50)
        {
            outlier_pt = -1;
            grid_feature.ave_z /= grid_feature.size*1.0;
        }
        // 排除掉异常点
        else
        {
            grid_feature.max_z = max_z_except_outlier;
            grid_feature.ave_z = tmp_ave_z;
        }
    }
    else
    {
        outlier_pt = -1;
        grid_feature.ave_z /= grid_feature.size*1.0;
    }
    grid_feature.ave_x /= grid_feature.size*1.0;
    grid_feature.ave_y /= grid_feature.size*1.0;

    // 高程与平均高程的偏差和
    for (unsigned pt = 0; pt<grid_feature.size; ++pt)
    {
        if (pt == outlier_pt)
            continue;
        grid_feature.delta_z += absf(cellobj[pt]->z - grid_feature.ave_z);
    }
#define USE_Z_MEAN_DIFF
    //#define USE_Z_DIFF
    /*
     * 使用高程平均差作为障碍物判断依据
     */
#ifdef USE_Z_MEAN_DIFF
    float threshold;
    threshold = grid_feature.delta_z / ((grid_feature.size)*1.0);
    // GridThresholdGroundDetect=5.000000
    if (threshold > cfg.cfgPlaneDetect.GridThresholdGroundDetect)
        grid_feature.cellType |= CELL_TYPE_ABOVE_DELTA_Z;
    else
        grid_feature.cellType |= CELL_TYPE_BELOW_DELTA_Z;
#endif
    /*
     * 使用高程偏差作为障碍物判断依据
     */
#ifdef USE_Z_DIFF
    float threshold;
    threshold = grid_feature.max_z - grid_feature.min_z;
    // GridThresholdGroundDetect=5.000000
    if (threshold > cfg.cfgPlaneDetect.GridThresholdDiffZ)
        grid_feature.cellType |= CELL_TYPE_ABOVE_DELTA_Z;
    else
        grid_feature.cellType |= CELL_TYPE_BELOW_DELTA_Z;
#endif
    grid_feature.circleNum = circleSet.size();
    // 如果有两个以上的向上打的circle，我们认为是障碍物高于lidar
    if (upperCircleSet.size() > 1)
        grid_feature.cellType |= CELL_TYPE_ABOVE_LIDAR;

    return 0;
}
/*
 *	1. 寻找并组建障碍物集群
 *	2. 统计每个障碍物集群特征信息
 *	// TODO: 该函数不健全
 */
int VelodyneAlgo::FindAndCalcGridClusterFeature(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj)
{
    if (scanobj.scanGridArray.size() == 0)
        return -1;

    int RowSize = scanobj.scanGridArray.size();
    int ColumnSize = scanobj.scanGridArray[0].size();

    bitvv isSearched;
    // 初始化 flag 向量
    isSearched.resize(RowSize);
    for (unsigned row = 0; row<RowSize; ++row)
        isSearched[row].resize(ColumnSize, false);

    for (unsigned row = 0; row<RowSize; ++row)
    {
        for (unsigned grid = 0; grid<ColumnSize; ++grid)
        {
            VelodyneDataStruct::gridCluster_t grid_cluster;
            VelodyneAlgo::SearchGridCluster(cfg, scanobj, grid_cluster, isSearched, row, grid);
            if (grid_cluster.size() > 0)								// 本次遍历搜索到集群，保存该集群
                scanobj.scanGridClusterSet.push_back(grid_cluster);
        }
    }
    int clusterNumber = scanobj.scanGridClusterSet.size();				// 总共有clusterSize个障碍物集群
    if (scanobj.scanGridClusterFeatureSet.size() == 0)
        scanobj.scanGridClusterFeatureSet.resize(clusterNumber);
    for (unsigned cluster = 0; cluster<clusterNumber; ++cluster)
    {
        VelodyneAlgo::CalcSingleGridClusterFeature(scanobj.scanGridClusterSet[cluster],
                scanobj.scanGridClusterFeatureSet[cluster]);
    }
    return 0;
}
/*
 *	方格周围寻找障碍物方格，组建障碍物集群
 */
int VelodyneAlgo::SearchGridCluster(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj, VelodyneDataStruct::gridCluster_t& grid_clu,
                                    bitvv& isSearchedvv, int row_idx, int grid_idx)
{
    int RowSize = scanobj.scanGridArray.size();
    int ColumnSize = scanobj.scanGridArray[0].size();

    if (row_idx < 0 || row_idx >= RowSize || grid_idx<0 || grid_idx >= ColumnSize)
        return -1;

    if (isSearchedvv[row_idx][grid_idx])
        return 0;

    if (scanobj.scanGridFeatureArray[row_idx][grid_idx].size == 0)
    {
        isSearchedvv[row_idx][grid_idx] = 1;
        return -1;
    }

    if (scanobj.scanGridFeatureArray[row_idx][grid_idx].cellType & CELL_TYPE_ABOVE_DELTA_Z) 	// 有符合要求的cell
    {
        isSearchedvv[row_idx][grid_idx] = true;
        grid_clu.push_back(&scanobj.scanGridFeatureArray[row_idx][grid_idx]);					// 存进集群里
        // 由下至上 由左至右递归搜索其他当前方格周边的8个格子
        VelodyneAlgo::SearchGridCluster(cfg, scanobj, grid_clu, isSearchedvv, row_idx - 1, grid_idx - 1);
        VelodyneAlgo::SearchGridCluster(cfg, scanobj, grid_clu, isSearchedvv, row_idx - 1, grid_idx);
        VelodyneAlgo::SearchGridCluster(cfg, scanobj, grid_clu, isSearchedvv, row_idx - 1, grid_idx + 1);
        VelodyneAlgo::SearchGridCluster(cfg, scanobj, grid_clu, isSearchedvv, row_idx, grid_idx - 1);
        VelodyneAlgo::SearchGridCluster(cfg, scanobj, grid_clu, isSearchedvv, row_idx, grid_idx + 1);
        VelodyneAlgo::SearchGridCluster(cfg, scanobj, grid_clu, isSearchedvv, row_idx + 1, grid_idx - 1);
        VelodyneAlgo::SearchGridCluster(cfg, scanobj, grid_clu, isSearchedvv, row_idx + 1, grid_idx);
        VelodyneAlgo::SearchGridCluster(cfg, scanobj, grid_clu, isSearchedvv, row_idx + 1, grid_idx + 1);
    }

    return 0;
}
/*
 *	计算单个方形网格障碍物集群特征信息
 *	遍历集群中的每个方格
 */
int VelodyneAlgo::CalcSingleGridClusterFeature(VelodyneDataStruct::gridCluster_t& grid_clu, VelodyneDataStruct::gridClusterFeature_t& grid_cluFeature)
{
    grid_cluFeature.size = grid_clu.size();
    if (grid_cluFeature.size == 0)
    {
        return -1;
    }

    grid_cluFeature.clusterType = CELL_TYPE_INITIAL;

    for (unsigned grid = 0; grid<grid_cluFeature.size; ++grid)
    {
        if (grid_clu[grid]->cellType & CELL_TYPE_ABOVE_LIDAR)
            grid_cluFeature.clusterType |= CUDE_TYPE_ABOVE_LIDAR;

        if (grid_clu[grid]->cellType&CELL_TYPE_IN_OBSTACLE_RANGE)
            grid_cluFeature.clusterType |= CUDE_TYPE_IN_OBSTACLE_RANGE;
        /*
         *	统计集群x/y/z极值
         */
        if (grid == 0)
        {
            grid_cluFeature.min_x = grid_cluFeature.max_x = grid_clu[grid]->min_x;
            grid_cluFeature.min_y = grid_cluFeature.max_y = grid_clu[grid]->min_y;
            grid_cluFeature.min_z = grid_cluFeature.max_z = grid_clu[grid]->min_z;
        }
        else
        {
            if (grid_cluFeature.max_x<grid_clu[grid]->max_x)
                grid_cluFeature.max_x = grid_clu[grid]->max_x;
            if (grid_cluFeature.min_x>grid_clu[grid]->min_x)
                grid_cluFeature.min_x = grid_clu[grid]->min_x;

            if (grid_cluFeature.max_y<grid_clu[grid]->max_y)
            {
                grid_cluFeature.max_y = grid_clu[grid]->max_y;
                grid_cluFeature.max_y_x = grid_clu[grid]->max_y_x;
            }
            if (grid_cluFeature.min_y>grid_clu[grid]->min_y)
            {
                grid_cluFeature.min_y = grid_clu[grid]->min_y;
                grid_cluFeature.min_y_x = grid_clu[grid]->min_y_x;
            }

            if (grid_cluFeature.max_z<grid_clu[grid]->max_z)
                grid_cluFeature.max_z = grid_clu[grid]->max_z;
            if (grid_cluFeature.min_z>grid_clu[grid]->min_z)
                grid_cluFeature.min_z = grid_clu[grid]->min_z;
        }
    }
    // 计算障碍物集群大致尺寸(长 宽 高)
    grid_cluFeature.size_x = grid_cluFeature.max_x - grid_cluFeature.min_x;
    grid_cluFeature.size_y = grid_cluFeature.max_y - grid_cluFeature.min_y;
    grid_cluFeature.size_z = grid_cluFeature.max_z - grid_cluFeature.min_z;

    return 0;
}

