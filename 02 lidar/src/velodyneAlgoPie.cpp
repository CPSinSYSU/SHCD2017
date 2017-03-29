/***************************************************************************
 *
 *	实现 velodyneAlgo.h 中格网(Pie)形式的处理函数部分
 *
 ***************************************************************************/

#include "velodyneAlgo.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#ifdef WIN32
    #include <set>
#else
    #include <vector>
#endif

// 绝对值
static inline float absf(float a)
{
    if (a < 0)
        return -a;
    return a;
}
/*
 *	转换成格网表示，标记出平面和非平面
 *	cfg: 全局配置项
 *	scanobj.shots[?].pt[0~63]: 点云数据 一个shot有64个点
 *	结果: scanobj.scanCellArray[columnSize(默认360)][(MaxRad-MinRad)/CellSize]
 *		计算激光点的offset和k，然后扔进去:scanobj.scanCellArray[offset][k]
 *		scanobj.scanCellArray[offset][k].circleID == 属于哪一线激光雷达
 */
int VelodyneAlgo::TransferToPieMatrix(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj)
{
    printf("TransferToPieMatrix starting...\n");
#define  DefaultColumnSize 360
    /*
     *	把圆周切成多少份
     *	注意激光雷达的旋角分辨率
     */
    int AzimuthNumber = cfg.cfgPlaneDetect.PieAzimuthNum;

    // 格网大小 50cm
    int PieRadSize = cfg.cfgPlaneDetect.PieRadSize;
    // 0-6000cm 半径60米范围内
    int MinRad = cfg.cfgPlaneDetect.PieMinRad;
    int MaxRad = cfg.cfgPlaneDetect.PieMaxRad;

    int shotSize = scanobj.shots.size();
    int shotStart = (cfg.cfgPlaneDetect.PieShotFrom * 1.0 / 360.0) * shotSize;
    if(shotStart < 0)
        shotStart = 0;
    int shotEnd = (cfg.cfgPlaneDetect.PieShotEnd * 1.0 / 360.0) * shotSize;
    if(shotEnd > shotSize)
        shotEnd = scanobj.shots.size();

    if (PieRadSize != 0 && (MaxRad - MinRad) % PieRadSize != 0)
        PieRadSize = 10;
    // 半径上需要分割的格子数目
    int RadNumber = (MaxRad - MinRad) / PieRadSize;

    if (AzimuthNumber == 0)
    {
        printf("AzimuthNumber == 0...\n");
        return -1;
    }
    // AzimuthNumber有效项检测，必须是DefaultColumnSize(360)的倍数
    //if (AzimuthNumber % PKT_NUM_IN_CIRCLE_LOWER_BOUND != 0)
    //	AzimuthNumber = DefaultColumnSize;
    // 将圆周分割八等份
    int sectionSize = AzimuthNumber / 8;
    // 计算每个激光点的距离和旋角
    //VelodyneAlgo::CalcRadAndTheta(cfg, scanobj);
    // 分配内存
    scanobj.scanPieArray.resize(AzimuthNumber);
    for (unsigned azimuth = 0; azimuth<AzimuthNumber; ++azimuth)
        scanobj.scanPieArray[azimuth].resize(RadNumber);

    // 弧度增量
    float inc = (M_PI * 2) / AzimuthNumber;
    // 计算每个section的
    std::vector<float> tanv;
    std::vector<float>::iterator result;
    // 一个八等份的角度正切值
    for (unsigned section = 0; section<sectionSize; ++section)
    {
        tanv.push_back(tan(inc*section));
#if 0
        printf("tanv[%d]: %.4f\n", section, tanv[section]);
#endif
    }
    int azimuth_idx, rad_idx;
    float flag;
    //for (unsigned shot = 0; shot<scanobj.shots.size(); ++shot)
    for (unsigned shot = shotStart; shot<shotEnd; ++shot)
    {
        /*
         *	一帧激光点云数据包含旋转过一周的所有shot
         *	一个shot由16/32/64束激光点数据组成，按照垂直角度由低到高(Circle)
         */
        for (unsigned circle = 0; circle<CIRCLE_NUM; ++circle)
        {
            VelodyneDataStruct::xpoint_t& pt = scanobj.shots[shot].pt[circle];
            // 去除无效点
            if (pt.point_type & POINT_TYPE_INVALID)
                continue;
            // filter by radius[MinRad:MaxRad]
            if (pt.rad < MinRad || pt.rad > MaxRad)
                continue;

            int azimuth_idx;
            /*
             *	四个象限 八个等份(从第一象限的y<x，逆时针等分)
             *	让 upper_bound 在一个等份中查询，比在一整个圆周中快速 高效
             *	 Returns an iterator pointing to the first element in [first,last) which compares greater than val
             *	 If no element in the range compares greater than val, the function returns last.
             *	该查询等份为 0-44度
             *		通过简单的加减乘除实现到其他7个等份的映射
             *	lower_bound函数(可以直接让tanv从inc*1而不是inc*0开始)
             *	 Returns an iterator pointing to the first element in [first,last) which does not compare less than val
             */
            //printf("tanv.end()-tanv.begin() = %d\n", tanv.end() - tanv.begin());
            if (pt.x > 0 && pt.y > 0)
            {
                // 第一等份
                if (pt.x > pt.y)
                {
                    // 逆时针
                    flag = pt.tan_theta;
                    result = upper_bound(tanv.begin(), tanv.end(), flag);
                    // 注释代码包含y=x 但不包含y=0
                    if (result == tanv.end())
                    {
                        azimuth_idx = sectionSize - 1;
                    }
                    else
                    {
                        //azimuth_idx = result - tanv.begin();
                        azimuth_idx = -1 + result - tanv.begin();
                    }
                }
                // 第二等份
                else
                {
                    // 顺时针
                    flag = 1 / pt.tan_theta;
                    result = upper_bound(tanv.begin(), tanv.end(), flag);
                    // 注释代码不包含x=0也不包含y=x
                    if (result == tanv.end())
                    {
                        azimuth_idx = sectionSize;
                    }
                    else
                    {
                        //azimuth_idx = sectionSize * 2 - 1 - (result - tanv.begin());
                        azimuth_idx = sectionSize * 2 - (result - tanv.begin());
                    }
                }
            } // end of if (pt.x > 0 && pt.y > 0){
            else if (pt.x<0 && pt.y>0)
            {
                // 第四等份
                if (-pt.x>pt.y)
                {
                    flag = -pt.tan_theta;
                    result = upper_bound(tanv.begin(), tanv.end(), flag);
                    // 注释代码不包含y=-x也不包含y=0
                    if (result == tanv.end())
                    {
                        azimuth_idx = sectionSize * 3;
                    }
                    else
                    {
                        //azimuth_idx = sectionSize * 4 - 1 - (result - tanv.begin());
                        azimuth_idx = sectionSize * 4 - (result - tanv.begin());
                    }
                }
                // 第三等份
                else
                {
                    flag = 1 / -pt.tan_theta;
                    result = upper_bound(tanv.begin(), tanv.end(), flag);
                    // 注释代码包含y=-x但不包含x=0
                    if (result == tanv.end())
                    {
                        azimuth_idx = sectionSize * 3 - 1;
                    }
                    else
                    {
                        //azimuth_idx = sectionSize * 2 + (result - tanv.begin());
                        azimuth_idx = sectionSize * 2 - 1 + (result - tanv.begin());
                    }
                }
            } // end of else if (pt.x<0 && pt.y>0){
            else if (pt.x<0 && pt.y<0)
            {
                // 第五等份
                if (-pt.x>-pt.y)
                {
                    flag = pt.tan_theta;
                    result = upper_bound(tanv.begin(), tanv.end(), flag);
                    // 注释代码包含y=x但不包含y=0
                    if (result == tanv.end())
                    {
                        azimuth_idx = sectionSize * 5 - 1;
                    }
                    else
                    {
                        //azimuth_idx = sectionSize * 4 + (result - tanv.begin());
                        azimuth_idx = sectionSize * 4 - 1 + (result - tanv.begin());
                    }
                }
                // 第六等份
                else
                {
                    flag = 1 / pt.tan_theta;
                    result = upper_bound(tanv.begin(), tanv.end(), flag);
                    // 注释代码不包含y=x也不包含x=0
                    if (result == tanv.end())
                    {
                        azimuth_idx = sectionSize * 5;
                    }
                    else
                    {
                        //azimuth_idx = sectionSize * 6 -1 - (result - tanv.begin());
                        azimuth_idx = sectionSize * 6 - (result - tanv.begin());
                    }
                }
            } // end of else if (pt.x<0 && pt.y<0)
            else if (pt.x>0 && pt.y<0)
            {
                // 第八等份
                if (pt.x>-pt.y)
                {
                    flag = -pt.tan_theta;
                    result = upper_bound(tanv.begin(), tanv.end(), flag);
                    // 注释代码不包含y=-x也不包含y=0
                    if (result == tanv.end())
                    {
                        azimuth_idx = sectionSize * 7;
                    }
                    else
                    {
                        //azimuth_idx = sectionSize * 8 -1 - (result - tanv.begin());
                        azimuth_idx = sectionSize * 8 - (result - tanv.begin());
                    }
                }
                // 第七等份
                else
                {
                    flag = 1 / -pt.tan_theta;
                    result = upper_bound(tanv.begin(), tanv.end(), flag);
                    // 包含y=-x但不包含x=0
                    if (result == tanv.end())
                    {
                        azimuth_idx = sectionSize * 7 - 1;
                    }
                    else
                    {
                        //azimuth_idx = sectionSize * 6 + (result - tanv.begin());
                        azimuth_idx = sectionSize * 6 - 1 + (result - tanv.begin());
                    }
                }
            } // end of else if (pt.x>0 && pt.y<0)
            else
            {
                continue;
            }
#if 0
            printf("(%.2f, %.2f): tan_theta=%.4f azimuth_idx = %d\n", pt.x, pt.y, pt.tan_theta, azimuth_idx);
#endif
            // 相同旋角的半径上的网格下标
            rad_idx = (int)((pt.rad - MinRad) / (PieRadSize*1.0));
            // circleID
            pt.circleID = circle;

            scanobj.scanPieArray[azimuth_idx][rad_idx].push_back(&pt);
            // 可能雷达基地(车顶高+雷达架子)约2米
            if (scanobj.shots[shot].pt[circle].z > -200.0
                    && scanobj.shots[shot].pt[circle].rad < cfg.cfgCircleItems[circle].ThresholdMinDist*cfg.cfgGlobal.ThresholdMinRad)
            {
                // 小于半径阈值 考虑为路沿 4500x0.82=3690(cm)
                scanobj.shots[shot].pt[circle].point_type |= POINT_TYPE_BELOW_R;
            }
        } // end of for (unsigned circle = 0; circle<CIRCLE_NUM; ++circle){
    } // end of for (unsigned shot = 0; shot<scanobj.shots.size(); ++shot){
#if 0
    for (unsigned azimuth = 0; azimuth < AzimuthNumber; azimuth++)
    {
        bool isEmpty = true;
        for (unsigned rad = 0; rad < RadNumber; rad++)
        {
            if (scanobj.scanPieArray[azimuth][rad].size() > 0)
            {
                isEmpty = false;
                break;
            }
        }
        if (isEmpty)
        {
            printf("azimuth = %d\n", azimuth);
        }
    }
    printf("end of a scan......Pie\n");
#endif

    return 0;
}

/*
 *  根据配置来计算scan 网格的特征量
 *  @param[IN] cfg       配置信息
 *  @param[IN] scanobj  要计算的scan对象
 *  @return     0:成功 其他失败
 *	scanobj.scanCellFeatureArray保存特征量，大小与scanCellArray保持一致
 *	调用 int VelodyneAlgo::CalcCellFeature(CfgVeloView& cfg, cell& pieobj, cellFeature& f)计算特征
 */
int VelodyneAlgo::CalcPieMatrixFeature(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj)
{
    if (scanobj.scanPieArray.size() == 0)
        return -1;
    int AzimuthNumber = scanobj.scanPieArray.size();
    int RadNumber = scanobj.scanPieArray[0].size();

    // 分配空间: 总共有columnSize(360)个方向的column,每一个column分配cellNumber个网格空间。
    if (scanobj.scanPieFeatureArray.size() == 0)
    {
        scanobj.scanPieFeatureArray.resize(AzimuthNumber);
        for (unsigned azimuth = 0; azimuth<AzimuthNumber; ++azimuth)
            scanobj.scanPieFeatureArray[azimuth].resize(RadNumber);
    }
    // 整个一周
    for (unsigned azimuth = 0; azimuth<AzimuthNumber; azimuth++)
    {
        VelodyneDataStruct::pieAzimuth_t& azimuthObj = scanobj.scanPieArray[azimuth];
        RadNumber = azimuthObj.size();
        for (unsigned rad = 0; rad<RadNumber; rad++)
        {
            VelodyneDataStruct::cell_t& cellObj = scanobj.scanPieArray[azimuth][rad];
            VelodyneDataStruct::pieFeature_t& pie_feature = scanobj.scanPieFeatureArray[azimuth][rad];
            pie_feature.azimuthID = azimuth;
            pie_feature.radID = rad;
            pie_feature.pCell = &cellObj;
            CalcSinglePieCellFeature(cfg, cellObj, pie_feature);
        }
    }
    return 0;
}
/*
 *	计算饼型格网中单个Cell中 R Z I等均值平均差特征
 *		min_x/max_x/ave_x;min_y/max_y/ave_y
 *		min_z/max_z/ave_z(平均高程)/delta_z(高程平均差)
 *	通过高程平均值均值检测是否为障碍物
 */
int VelodyneAlgo::CalcSinglePieCellFeature(CfgVeloView_t& cfg, VelodyneDataStruct::cell_t& pieobj, VelodyneDataStruct::pieFeature_t& pie_feature)
{
    pie_feature.size = pieobj.size();
    pie_feature.cellType = CELL_TYPE_INITIAL;

    if (pie_feature.size == 0)
    {
        // 如果网格中一个点没有，则无效
        pie_feature.cellType |= CELL_TYPE_INVALID;
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
     */
    float hollowMin = cfg.cfgGlobal.GroundZ + cfg.cfgGlobal.ThresholdMinHollow;
    float hollowMax = cfg.cfgGlobal.GroundZ + cfg.cfgGlobal.ThresholdMaxHollow;
    /*
     *	找到高程最大值，并忽略,用来解决异常点的问题
     *	这个方法比较粗暴，应该用更合适的算法来检测异常点
     */
    int outlier_pt;
    float max_z_except_outlier;

    pie_feature.ave_x = pie_feature.ave_y = pie_feature.ave_z = 0.0;
    pie_feature.delta_z = 0;

    for (unsigned pt = 0; pt<pie_feature.size; ++pt)
    {
        // 求均值
        pie_feature.ave_x += pieobj[pt]->x;
        pie_feature.ave_y += pieobj[pt]->y;
        pie_feature.ave_z += pieobj[pt]->z;
        // 如果有一个点，在距离阈值内，那么整个网格在阈值距离内
        if (pieobj[pt]->point_type & POINT_TYPE_BELOW_R)
            pie_feature.cellType |= CELL_TYPE_BELOW_R;
        #ifdef WIN32
        // circle计数
        circleSet.insert(pieobj[pt]->circleID);
        #else
        if(std::find(circleSet.begin(), circleSet.end(), pieobj[pt]->circleID) == circleSet.end()){
            //printf("circleSet.insert\n");
            circleSet.push_back(pieobj[pt]->circleID);
        }
        #endif
         #ifdef WIN32
        // 水平向上打的circle计数，用于判断固定障碍物
        if (pieobj[pt]->circleID > UPPER_VERT_FROM)
            upperCircleSet.insert(pieobj[pt]->circleID);
        #else
        if (pieobj[pt]->circleID > UPPER_VERT_FROM){
            if(std::find(upperCircleSet.begin(), upperCircleSet.end(),
                         pieobj[pt]->circleID) == upperCircleSet.end()){
                //printf("upperCircleSet.insert\n");
                upperCircleSet.push_back(pieobj[pt]->circleID);
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
        if (!(pie_feature.cellType & CELL_TYPE_NOT_HOLLOW))
        {
            if (pieobj[pt]->z > hollowMin && pieobj[pt]->z < hollowMax)
                pie_feature.cellType |= CELL_TYPE_NOT_HOLLOW;
        }
        // 求最值
        if (pt == 0)
        {
            // 赋初值
            outlier_pt = 0;
            pie_feature.min_x = pie_feature.max_x = pieobj[pt]->x;
            pie_feature.min_y = pie_feature.max_y = pieobj[pt]->y;
            max_z_except_outlier = pie_feature.min_z = pie_feature.max_z = pieobj[pt]->z;
        }
        else
        {
            // 记录最值
            if (pie_feature.max_x < pieobj[pt]->x)
                pie_feature.max_x = pieobj[pt]->x;

            if (pie_feature.min_x > pieobj[pt]->x)
                pie_feature.min_x = pieobj[pt]->x;

            if (pie_feature.max_y < pieobj[pt]->y)
                pie_feature.max_y = pieobj[pt]->y;

            if (pie_feature.min_y > pieobj[pt]->y)
                pie_feature.min_y = pieobj[pt]->y;
            // 记录下异常点，然后在后面判断是否删除
            if (pie_feature.max_z < pieobj[pt]->z)
            {
                max_z_except_outlier = pie_feature.max_z;
                pie_feature.max_z = pieobj[pt]->z;
                outlier_pt = pt;
            }
            if (pie_feature.min_z > pieobj[pt]->z)
                pie_feature.min_z = pieobj[pt]->z;
        }
    } // end of for (unsigned pt = 0; pt<pie_feature.size; ++pt){

    // 确定网格里面有1个以上的点
    if (pie_feature.size > 1)
    {
        // 除去可能的异常点后计算平均高程
        float tmp_ave_z = pie_feature.ave_z - pieobj[outlier_pt]->z;
        tmp_ave_z /= (pie_feature.size - 1)*1.0;
        /*
         *	异常点
         *	 正常: 更新平均高程(加入异常点的高程)
         *	 异常: 无需更新平均高程 需更新max_z(不使用异常点的z值作为最值)
         */
        if (pieobj[outlier_pt]->z - tmp_ave_z < 50)
        {
            outlier_pt = -1;
            pie_feature.ave_z /= pie_feature.size*1.0;
        }
        else
        {
            pie_feature.max_z = max_z_except_outlier;
            pie_feature.ave_z = tmp_ave_z;
        }
    }
    else
    {
        outlier_pt = -1;
        pie_feature.ave_z /= pie_feature.size * 1.0;
    }
    // 计算ave_x和ave_y
    pie_feature.ave_x /= pie_feature.size*1.0;
    pie_feature.ave_y /= pie_feature.size*1.0;
    // 计算高程与平均高程的偏差和
    for (unsigned pt = 0; pt<pie_feature.size; ++pt)
    {
        if (pt == outlier_pt)
            continue;
        pie_feature.delta_z += absf(pieobj[pt]->z - pie_feature.ave_z);
    }
#define USE_Z_MEAN_DIFF
    //#define USE_Z_DIFF
    /*
    * 使用高程平均差作为障碍物判断依据
    */
#ifdef USE_Z_MEAN_DIFF
    float threshold;
    threshold = pie_feature.delta_z / ((pie_feature.size)*1.0);
    //  进行路面和障碍物的分割(根据平均差)，具体就是修改Cell的属性
    if (threshold > cfg.cfgPlaneDetect.PieThresholdGroundDetect)
        pie_feature.cellType |= CELL_TYPE_ABOVE_DELTA_Z;
    else
        pie_feature.cellType |= CELL_TYPE_BELOW_DELTA_Z;
#endif
    /*
    * 使用高程偏差作为障碍物判断依据
    */
#ifdef USE_Z_DIFF
    float threshold;
    threshold = pie_feature.max_z - pie_feature.min_z;
    // GridThresholdGroundDetect=5.000000
    if (threshold > cfg.cfgPlaneDetect.PieThresholdDiffZ)
        pie_feature.cellType |= CELL_TYPE_ABOVE_DELTA_Z;
    else
        pie_feature.cellType |= CELL_TYPE_BELOW_DELTA_Z;
#endif

    pie_feature.circleNum = circleSet.size();
    // 如果有两个以上的向上打的circle，我们认为是障碍物高于lidar
    if (upperCircleSet.size() > 1)
        pie_feature.cellType |= CELL_TYPE_ABOVE_LIDAR;


    return 0;
}
/*
 *  在网格中寻找相关联的网格簇并计算该集群特征
 *  @param[IN] cfg       配置信息
 *  @param[IN] scanobj  要计算的scan对象
 *	@param[IN] findClosestOnly  只找最近的一圈
 *  @return     0:成功 其他失败
 */
int VelodyneAlgo::FindAndCalcPieClusterFeature(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj, bool findClosestOnly)
{
    if (scanobj.scanPieArray.size() == 0)
        return -1;
    int AzimuthNumber = scanobj.scanPieArray.size();
    int RadNumber = scanobj.scanPieArray[0].size();

    /*
     *	二维的char数组(bit map/vector)
     *	初始化为0表示该cell未被访问(即存进集群中)
     */
    bitvv isSearched;
    // 初始化 flag 数组
    isSearched.resize(AzimuthNumber);
    for (unsigned azimuth = 0; azimuth<AzimuthNumber; ++azimuth)
        isSearched[azimuth].resize(RadNumber, false);

    for (unsigned azimuth = 0; azimuth<AzimuthNumber; ++azimuth)
    {
        for (unsigned rad = 0; rad<RadNumber; ++rad)
        {
            if (findClosestOnly && isSearched[azimuth][rad])
            {
                break;
            }
            VelodyneDataStruct::pieCluster_t clu;
            /*
             *	递归调用，寻找scanPieArray[azimuth][rad]是否为障碍物 (clu.size()>0)
             *	是的话寻找其连通图 集群为障碍物
             *	一个集群里有好几个网格，每个网格特征(CELL_TYPE_ABOVE_DELTA_Z)相似
             *	cluster是scanobj.scanPieFeatureArray[azimuth][rad]特征信息的集群
             */
            VelodyneAlgo::SearchPieCluster(cfg, scanobj, clu, isSearched, azimuth, rad);
            if (clu.size())                                        			// 该集群里有符合要求的cell，则保存该集群
            {
                scanobj.scanPieClusterSet.push_back(clu);
                if (findClosestOnly)
                    break;
            }
        }
    } // end of for (unsigned azimuth = 0; azimuth<AzimuthNumber; ++azimuth){

    int clusterNumber = scanobj.scanPieClusterSet.size();               	// 总共有clusterNumber个集群
    if (scanobj.scanPieClusterFeatureSet.size() == 0)
        scanobj.scanPieClusterFeatureSet.resize(clusterNumber);

    // 计算障碍物集群特征信息
    for (unsigned cluster = 0; cluster<clusterNumber; cluster++)
    {
        CalcSinglePieClusterFeature(scanobj.scanPieClusterSet[cluster], scanobj.scanPieClusterFeatureSet[cluster]);
    }

    return 0;
}
/*
 *	寻找障碍物Cell连通图
 *	pie_clu: 保存的集群
 */
int VelodyneAlgo::SearchPieCluster(CfgVeloView_t& cfg, VelodyneDataStruct& scanobj, VelodyneDataStruct::pieCluster_t& pie_clu,
                                   bitvv& isSearchedvv, int azimuth_idx, int rad_idx)
{
    int azimuthNumber = scanobj.scanPieArray.size();
    int radNumber = scanobj.scanPieArray[0].size();
    // 到了饼型网格边缘，停止搜索
    if (azimuth_idx<0 || azimuth_idx >= azimuthNumber || rad_idx<0 || rad_idx >= radNumber)
        return -1;
    /*
     *	到了根节点
     *	已经被标记并存进集群中递归直接返回
     */
    if (isSearchedvv[azimuth_idx][rad_idx])
        return 0;

    if (scanobj.scanPieFeatureArray[azimuth_idx][rad_idx].size == 0)
    {
        isSearchedvv[azimuth_idx][rad_idx] = true;
        return -1;
    }

    if (scanobj.scanPieFeatureArray[azimuth_idx][rad_idx].cellType & CELL_TYPE_ABOVE_DELTA_Z)  	// 有符合要求的cell
    {
        isSearchedvv[azimuth_idx][rad_idx] = true;
        pie_clu.push_back(&scanobj.scanPieFeatureArray[azimuth_idx][rad_idx]);					// 存进集群里
        /*
         *	左下 左 左上
         *	正下 正上
         *	右下 右 右上
         */
        VelodyneAlgo::SearchPieCluster(cfg, scanobj, pie_clu, isSearchedvv, azimuth_idx - 1, rad_idx - 1);
        VelodyneAlgo::SearchPieCluster(cfg, scanobj, pie_clu, isSearchedvv, azimuth_idx - 1, rad_idx);
        VelodyneAlgo::SearchPieCluster(cfg, scanobj, pie_clu, isSearchedvv, azimuth_idx - 1, rad_idx + 1);

        VelodyneAlgo::SearchPieCluster(cfg, scanobj, pie_clu, isSearchedvv, azimuth_idx, rad_idx - 1);
        VelodyneAlgo::SearchPieCluster(cfg, scanobj, pie_clu, isSearchedvv, azimuth_idx, rad_idx + 1);

        VelodyneAlgo::SearchPieCluster(cfg, scanobj, pie_clu, isSearchedvv, azimuth_idx + 1, rad_idx - 1);
        VelodyneAlgo::SearchPieCluster(cfg, scanobj, pie_clu, isSearchedvv, azimuth_idx + 1, rad_idx);
        VelodyneAlgo::SearchPieCluster(cfg, scanobj, pie_clu, isSearchedvv, azimuth_idx + 1, rad_idx + 1);

        // 上下左右的方向寻找更远
        VelodyneAlgo::SearchPieCluster(cfg, scanobj, pie_clu, isSearchedvv, azimuth_idx, rad_idx + 2);
        VelodyneAlgo::SearchPieCluster(cfg, scanobj, pie_clu, isSearchedvv, azimuth_idx, rad_idx - 2);
        VelodyneAlgo::SearchPieCluster(cfg, scanobj, pie_clu, isSearchedvv, azimuth_idx + 2, rad_idx);
        VelodyneAlgo::SearchPieCluster(cfg, scanobj, pie_clu, isSearchedvv, azimuth_idx - 2, rad_idx);
    }
    return 0;
}
/*
 *	1. 计算障碍物集群特征信息(一个集群)
 *		min_x/max_x/size_x;min_y/max_y/size_y;min_z/max_z/size_z;
 *	2. 表示这个集群 极坐标(theta, radius)
 */
int VelodyneAlgo::CalcSinglePieClusterFeature(VelodyneDataStruct::pieCluster_t& pie_clu, VelodyneDataStruct::pieClusterFeature_t& pie_cluFeature)
{
    pie_cluFeature.size = pie_clu.size();
    if (pie_cluFeature.size == 0)
    {
        return 0;
    }
    pie_cluFeature.clusterType = 0;
    pie_cluFeature.pointNumber = 0;

    // 最值
    for (unsigned cell = 0; cell<pie_cluFeature.size; ++cell)
    {
        if (cell == 0)
        {
            pie_cluFeature.min_x = pie_cluFeature.max_x = pie_clu[cell]->min_x;
            pie_cluFeature.min_y = pie_cluFeature.max_y = pie_clu[cell]->min_y;
            pie_cluFeature.min_z = pie_cluFeature.max_z = pie_clu[cell]->min_z;
        }
        else
        {
            if (pie_cluFeature.max_x < pie_clu[cell]->max_x)
                pie_cluFeature.max_x = pie_clu[cell]->max_x;

            if (pie_cluFeature.min_x > pie_clu[cell]->min_x)
                pie_cluFeature.min_x = pie_clu[cell]->min_x;

            if (pie_cluFeature.max_y < pie_clu[cell]->max_y)
                pie_cluFeature.max_y = pie_clu[cell]->max_y;

            if (pie_cluFeature.min_y > pie_clu[cell]->min_y)
                pie_cluFeature.min_y = pie_clu[cell]->min_y;

            if (pie_cluFeature.max_z < pie_clu[cell]->max_z)
                pie_cluFeature.max_z = pie_clu[cell]->max_z;

            if (pie_cluFeature.min_z > pie_clu[cell]->min_z)
                pie_cluFeature.min_z = pie_clu[cell]->min_z;

        }
        // 一个集群里有好几个网格，每个网格特征(CELL_TYPE_ABOVE_DELTA_Z)相似
        pie_cluFeature.pointNumber += pie_clu[cell]->size;
        /*
         *	根据每个cell中 point的数量加权 计算出的中心，极坐标表示(theta, radius)
         *	每个cell在格网中的位置(columnID, cellID)，在CalcScanCellFeature()函数中设置
         */
        pie_cluFeature.theta += pie_clu[cell]->size*pie_clu[cell]->azimuthID;
        pie_cluFeature.radius += pie_clu[cell]->size*pie_clu[cell]->radID;
    }
    // 计算每个cluster x/y/z范围(长宽高)
    pie_cluFeature.size_x = pie_cluFeature.max_x - pie_cluFeature.min_x;
    pie_cluFeature.size_y = pie_cluFeature.max_y - pie_cluFeature.min_y;
    pie_cluFeature.size_z = pie_cluFeature.max_z - pie_cluFeature.min_z;
    /*
     * 计算出当前集群的中心
     *  theta: 所有点的平均azimuthID，每个cell里面的所有点使用相同的azimuthID
     *	radius: 所有点的平均radID，每个cell里面的所有点使用相同的radID
     */
    pie_cluFeature.theta = pie_cluFeature.theta / (pie_cluFeature.pointNumber*1.0);
    pie_cluFeature.radius = pie_cluFeature.radius / (pie_cluFeature.pointNumber*1.0);

    return 0;
}


