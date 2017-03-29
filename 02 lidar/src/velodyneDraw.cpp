/**************************************************************************
 *
 * 画各种图标进行分析
 *
 ***************************************************************************/
#include "velodyneDraw.h"

#ifdef WIN32
#include <gl/glut.h>
#else
#include <GL/glut.h>
#endif

#include "velodyneScanner.h"
#include "configStruct.h"
#include "velodyneThread.h"
#include <stdint.h>
#include <string.h>             // for memset

#include "velodyneAlgo.h"
#include "./common/color_util.h"

#include "velodyneHeightMap.h"
#include "velodyneNegativeObstacleMap.h"

//#define _USE_MATH_DEFINES
//#include <math.h>

#include "velodyneDataStruct.h"

#include <unistd.h>              // for usleep()

//uint8_t show_state = SHOW_NEGATIVE_OBSTACLE;
//uint8_t show_state = SHOW_OBSTACLES_PIE;
//uint8_t show_state = SHOW_OBSTACLES_GRID;
uint8_t show_state = SHOW_ALL_POINTS_BELOW;

void drawCar(float angle)
{
    float min_x = -1.0 * (g_CfgVeloView.cfgGlobal.CarWidth / 2);
    float max_x = 1.0 * (g_CfgVeloView.cfgGlobal.CarWidth / 2);
    float min_y = -1.0 * (g_CfgVeloView.cfgGlobal.CarLength / 2);
    float max_y = 1.0 * (g_CfgVeloView.cfgGlobal.CarLength / 2);
    float min_z = -1.0 * (g_CfgVeloView.cfgGlobal.CarHeight / 2);
    float max_z = 1.0 * (g_CfgVeloView.cfgGlobal.CarHeight / 2);
    // 极坐标旋角
    glRotatef(angle, 0.0, 0.0, 1.0);
    // 车尾
    drawLineGLRGB(min_x, min_y, min_z, min_x, min_y, max_z, 1.0, 1.0, 1.0);
    drawLineGLRGB(min_x, min_y, max_z, max_x, min_y, max_z, 1.0, 1.0, 1.0);
    drawLineGLRGB(min_x, min_y, min_z, max_x, min_y, min_z, 1.0, 1.0, 1.0);
    drawLineGLRGB(max_x, min_y, min_z, max_x, min_y, max_z, 1.0, 1.0, 1.0);
    // 车体
    drawLineGLRGB(min_x, min_y, max_z, min_x, max_y, max_z, 1.0, 1.0, 1.0);
    drawLineGLRGB(min_x, min_y, min_z, min_x, max_y, min_z, 1.0, 1.0, 1.0);
    drawLineGLRGB(max_x, min_y, max_z, max_x, max_y, max_z, 1.0, 1.0, 1.0);
    drawLineGLRGB(max_x, min_y, min_z, max_x, max_y, min_z, 1.0, 1.0, 1.0);
    // 车头
    drawLineGLRGB(min_x, max_y, max_z, max_x, max_y, max_z, 1.0, 0., 0.);
    drawLineGLRGB(min_x, max_y, min_z, max_x, max_y, min_z, 1.0, 0., 0.);
    drawLineGLRGB(max_x, max_y, min_z, max_x, max_y, max_z, 1.0, 0., 0.);
    drawLineGLRGB(min_x, max_y, min_z, min_x, max_y, max_z, 1.0, 0., 0.);

    glRotatef(-angle, 0.0, 0.0, 1.0);
}

void drawAllPoints(int psize, int mode)
{
    glPointSize(psize);
    glBegin(GL_POINTS);

    int size;
    int circle_start, circle_end;
    switch (mode)
    {
    case 1:
        circle_start = 0;
        circle_end = VELODYNE_NUM_BEAMS_IN_ONE_SHOT;
        break;
    case 2:
        circle_start = UPPER_VERT_FROM;
        circle_end = VELODYNE_NUM_BEAMS_IN_ONE_SHOT;
        break;
    default:
        circle_start = 0;
        circle_end = 0;
        break;
    }

    //pthread_spin_lock(&g_scanBuffer_lock);
    VelodyneDataStruct* pscanobj = getScanRawForDraw();
    if (pscanobj)
    {
        if (mode == 1)
        {
            printf("draw all points below...\n");
        }
        else if (mode == 2)
        {
            printf("draw all points above...\n");
        }
        //printf("draw points\n");
        // 每个Circle "configStruct.h"
        for (unsigned circle = circle_start; circle <circle_end; circle++)
        {
            //  整个一周
            size = pscanobj->shots.size();
            for (unsigned shot = 0; shot<size; shot++)
            {
                if ((*pscanobj).shots[shot].pt[circle].point_type & POINT_TYPE_INVALID)
                    continue;
                // 绿色
                if ((*pscanobj).shots[shot].pt[circle].x > 0 && (*pscanobj).shots[shot].pt[circle].y > 0)
                {
                    drawPointRGB((*pscanobj).shots[shot].pt[circle], 0.1, 1.0, 0.1, psize);
                }
                // 红色
                else if ((*pscanobj).shots[shot].pt[circle].x < 0 && (*pscanobj).shots[shot].pt[circle].y > 0)
                {
                    drawPointRGB((*pscanobj).shots[shot].pt[circle], 1.0, 0.1, 0.1, psize);
                }
                else if ((*pscanobj).shots[shot].pt[circle].x < 0 && (*pscanobj).shots[shot].pt[circle].y < 0)
                {
                    drawPointRGB((*pscanobj).shots[shot].pt[circle], 0.1, 1.0, 1.0, psize);
                }
                // 蓝色
                else if ((*pscanobj).shots[shot].pt[circle].x > 0 && (*pscanobj).shots[shot].pt[circle].y < 0)
                {
                    drawPointRGB((*pscanobj).shots[shot].pt[circle], 0.1, 0.1, 1.0, psize);
                }
                else
                {
                    drawPointRGB((*pscanobj).shots[shot].pt[circle], 1.0, 1.0, 0.1, psize);
                }
                // 画地面
                //if ((*pscanobj).shots[shot].pt[circle].z == g_CfgVeloView.cfgGlobal.GroundZ){
                //	drawPointRGB((*pscanobj).shots[shot].pt[circle], 1.0, 1.0, 1.0, 2);
                //}
                //printf("(%.2f, %.2f, %.2f)\n", (*pscanobj).shots[i].pt[j].x, (*pscanobj).shots[i].pt[j].y, (*pscanobj).shots[i].pt[j].z);
                // 画天花板
                if ((*pscanobj).shots[shot].pt[circle].z > 100)
                {
                    drawPointRGB((*pscanobj).shots[shot].pt[circle], 1.0, 1.0, 1.0, 2);
                }
//#define GET_GROUND_Z
#ifdef GET_GROUND_Z
                if (circle == 0)
                {
                    printf("#%d: (%.2f, %.2f, %.2f)\n", shot,
                           (*pscanobj).shots[shot].pt[circle].x,
                           (*pscanobj).shots[shot].pt[circle].y,
                           (*pscanobj).shots[shot].pt[circle].z);
                }
#endif
            }
        }
    }
    //pthread_spin_unlock(&g_scanBuffer_lock);
    delete pscanobj;

    glEnd();
}

void drawPointsHeightMap(int psize)
{
    glPointSize(psize);
    glBegin(GL_POINTS);

    //pthread_spin_lock(&g_scanBuffer_lock);

    VelodyneDataStruct* pscanobj = getScanRawForDraw();
    if (pscanobj)
    {
        printf("drawPointsHeightMap...\n");

        velodyne_height_map::HeightMap hm;

        hm.processData(*pscanobj);

    }
    //pthread_spin_unlock(&g_scanBuffer_lock);

    delete pscanobj;
    glEnd();
}

void drawPointsNagetiveObstacleMap(int psize)
{
    glPointSize(psize);
    glBegin(GL_POINTS);

    //pthread_spin_lock(&g_scanBuffer_lock);

    VelodyneDataStruct* pscanobj = getScanRawForDraw();
    if (pscanobj)
    {
        printf("drawPointsNagetiveObstacleMap...\n");

        velodyne_negative_obstacle_map::NegativeObstacleMap nom;
        nom.constructNegativeObstacleCloud(*pscanobj, true);
        // 画出所有的negative obstacle
        for (unsigned pt = 0; pt < nom.obs_count; pt++)
        {
            //printf("pt#%d: (%8.2f, %8.2f, %8.2f)\n", obs_count, obstacle_cloud[pt].x, obstacle_cloud[pt].y, obstacle_cloud[pt].z);
            // TODO 改为框框？
            drawPointRGB(nom.negative_obstacle_cloud[pt], 1., 0., 0.);
            //printf("(%4.2f, %4.2f, %4.2f)\n", nom.negative_obstacle_cloud[pt].x, nom.negative_obstacle_cloud[pt].y, nom.negative_obstacle_cloud[pt].z);
        }
    }

    //pthread_spin_unlock(&g_scanBuffer_lock);
    delete pscanobj;

    glEnd();
}

void drawPointsGridCell(float angle, int psize)
{
    // 极坐标旋角
    //glRotatef(angle, 0.0, 0.0, 1.0);
    glPointSize(psize);
    glBegin(GL_POINTS);

    float* pColor;

    //pthread_spin_lock(&g_scanBuffer_lock);
    VelodyneDataStruct* pcalobj = getScanRawForDraw();

    if (pcalobj)
    {
        //printf("process negative obstacle...\n");
        //velodyne_negative_obstacle_map::NegativeObstacleMap nom;
        //nom.constructNegativeObstacleCloud(*pscanobj);
        printf("draw points grid cell...\n");
        if (pcalobj->scanGridArray.size() == 0)
        {
            //printf("TransferToGridMatrix starting\n");
            VelodyneAlgo::TransferToGridMatrix(g_CfgVeloView, *pcalobj);
            //printf("TransferToGridMatrix done\n");
            VelodyneAlgo::CalcGridMatrixFeature(g_CfgVeloView, *pcalobj);
            //printf("CalcGridMatrixFeature done\n");
            VelodyneAlgo::FindAndCalcGridClusterFeature(g_CfgVeloView, *pcalobj);
            //printf("FindAndCalcGridClusterFeature done\n");
        }

        // 显示方格矩阵，外循环是在圆周上的划分，内循环是在半径方向上的划分
        for (unsigned row = 0; row <pcalobj->scanGridArray.size(); row++)
        {
            // 每行方格
            VelodyneDataStruct::gridRow_t& rowObj = pcalobj->scanGridArray[row];
            for (unsigned grid = 0; grid<rowObj.size(); grid++)
            {
                // 获取每个方格特征信息
                VelodyneDataStruct::gridFeature_t& gridobj = pcalobj->scanGridFeatureArray[row][grid];
                float high = 0;
                float low = gridobj.min_z;
                /*
                 * 根据选项来决定网格的高度用哪个特征值来表示
                 *	方差和
                 *	方差
                 *	数量
                 *	最大值
                 *	均值
                 *	均值平面
                 */
                //if (m_cfgVeloShow.showGridType == 0)
                //	high = gridobj.delta_z + gridobj.min_z;
                //if (m_cfgVeloShow.showGridType == 1)
                //	high = gridobj.delta_z / (gridobj.size*1.0) + gridobj.min_z;
                //if (m_cfgVeloShow.showGridType == 2)
                //	high = gridobj.size*1.0 + gridobj.min_z;
                //if (m_cfgVeloShow.showGridType == 3)
                //	high = gridobj.max_z;
                //if (m_cfgVeloShow.showGridType == 4)
                //	high = gridobj.ave_z;
                //if (m_cfgVeloShow.showGridType == 5){
                //	low = high = gridobj.ave_z;
                //}

                // TODO: 直接使用高程平均差
                //high = gridobj.delta_z / (gridobj.size*1.0) + gridobj.min_z;
                // 使用最高高程
                high = gridobj.max_z;

                //if (m_cfgVeloShow.showAllGridUnit)
                if (1)
                {
                    //if (g_CfgVeloView.cfgPlaneDetect.ColorMode == 0){
                    if (0)
                    {
                        // 显示模式0:根据高程方差的jet颜色
                        float delta_z = gridobj.delta_z / (gridobj.size*1.0);
                        pColor = color_util_jet(delta_z * g_CfgVeloView.cfgPlaneDetect.GridColorFactor);
                        drawCubeGLRGB(gridobj.min_x, gridobj.min_y, low,
                                      gridobj.max_x, gridobj.max_y, high,
                                      pColor[0], pColor[1], pColor[2]);
                    }
                    //else if (g_CfgVeloView.cfgPlaneDetect.ColorMode == 1){
                    else if (1)
                    {
                        // 显示模式1: 根据高程平均差的阈值分割两种颜色
                        if (gridobj.cellType & CELL_TYPE_ABOVE_DELTA_Z)
                        {
                            // 除去中空网格
                            if (!(gridobj.cellType & CELL_TYPE_NOT_HOLLOW))
                            {
                                continue;
                            }
#if 1
                            if (gridobj.cellType & CELL_TYPE_ABOVE_LIDAR)
                            {
                                drawCubeGLRGB(gridobj.min_x, gridobj.min_y, low,
                                              gridobj.max_x, gridobj.max_y, high,
                                              1, 0, 0);
                            }
                            else
                                drawCubeGLRGB(gridobj.min_x, gridobj.min_y, low,
                                              gridobj.max_x, gridobj.max_y, high,
                                              0, 1, 0);
#endif
                        }
                        // 坑
                        else if (gridobj.cellType & CELL_TYPE_NEGATIVE_OBSTACLE)
                        {
                            //printf("A negative obstacle grid...\n");
                            drawCubeGLRGB(gridobj.min_x, gridobj.min_y, low,
                                          gridobj.max_x, gridobj.max_y, 0,
                                          1., 1., 1.);
                        }
                        // 非障碍物
                        else
                        {
#if 0
                            drawCubeGLRGB(gridobj.min_x, gridobj.min_y, low,
                                          gridobj.max_x, gridobj.max_y, high,
                                          0, 0, 1);
#endif
                        }
                    }
                    //else if (g_CfgVeloView.cfgPlaneDetect.ColorMode == 2){
                    //	// 显示模式2:根据高程方差的阈值只显示地面颜色
                    //	if (gridobj.cellType&CELL_TYPE_BELOW_DELTA_Z){
                    //		Draw_Cube_GL_RGB(gridobj.min_x,
                    //			gridobj.min_y,
                    //			low,
                    //			gridobj.max_x,
                    //			gridobj.max_y,
                    //			high, 0, 0, 1);
                    //	}
                    //}
                    //else if (g_CfgVeloView.cfgPlaneDetect.ColorMode == 3){
                    //	/// 显示模式3:根据高度差小于30，则为路面，只显示地面颜色*/
                    //	if (gridobj.max_x - gridobj.min_z<30){
                    //		Draw_Cube_GL_RGB(gridobj.min_x,
                    //			gridobj.min_y,
                    //			low,
                    //			gridobj.max_x,
                    //			gridobj.max_y,
                    //			high, 0, 0, 1);
                    //	}
                    //}
                }// end of if(m_cfgVeloShow.showAllGridUnit)
            }// end of for( i=0; i<column.size(); i++)
        }// end of for(j=0; j <pscanobj->scanGridArray.size(); j++)

        /*
         * 方型网格障碍物网格集群
         */
        //if (m_cfgVeloShow.showCube)
        if (false)
        {
            // draw obstacle cluster
            char output[256];
            memset(output, 0, sizeof(output));

            for (unsigned cluster = 0; cluster<pcalobj->scanGridClusterSet.size(); ++cluster)
            {
                VelodyneDataStruct::gridClusterFeature_t& grid_cluFeature = pcalobj->scanGridClusterFeatureSet[cluster];
                if (grid_cluFeature.clusterType & CUDE_TYPE_IN_OBSTACLE_RANGE)
                {
                    /*
                     *	根据障碍物的长宽高特征进行区分
                     */
                    // 过滤掉长度超过15米的
                    if (grid_cluFeature.size_x > 1500)
                    {
                        //drawCubeGLRGB(pscanobj->scanGridClusterFeatureSet[cluster], ColorTableCircle[0][0] / 256.0, ColorTableCircle[0][1] / 256.0, ColorTableCircle[0][2] / 256.0);
                        //continue;
                    }
                    // 过滤掉超过宽度超过3米的
                    if (grid_cluFeature.size_y>300)
                    {
                        //drawCubeGLRGB(pscanobj->scanGridClusterFeatureSet[cluster], ColorTableCircle[1][0] / 256.0, ColorTableCircle[1][1] / 256.0, ColorTableCircle[1][2] / 256.0);
                        //continue;
                    }
                    // 过滤掉长宽高总和小于3米的
                    if ((grid_cluFeature.size_x + grid_cluFeature.size_y + grid_cluFeature.size_z)<300)
                    {
                        //drawCubeGLRGB(pscanobj->scanGridClusterFeatureSet[cluster], ColorTableCircle[2][0] / 256.0, ColorTableCircle[2][1] / 256.0, ColorTableCircle[2][2] / 256.0);
                        //continue;
                    }
                    // 过滤掉1米二一下的
                    if (grid_cluFeature.size_z < 120)
                    {
                        //drawCubeGLRGB(pscanobj->scanGridClusterFeatureSet[cluster], ColorTableCircle[2][0] / 256.0, ColorTableCircle[2][1] / 256.0, ColorTableCircle[2][2] / 256.0);
                        //continue;
                    }
                    // 过滤掉长宽比大于3的
                    if (grid_cluFeature.size_x / grid_cluFeature.size_y>3.0)
                    {
                        //drawCubeGLRGB(pscanobj->scanGridClusterFeatureSet[cluster], ColorTableCircle[2][0] / 256.0, ColorTableCircle[2][1] / 256.0, ColorTableCircle[2][2] / 256.0);
                        //continue;
                    }
                    sprintf(output, "%d-%dx%dx%d", cluster, (int)grid_cluFeature.size_x,
                            (int)grid_cluFeature.size_y, (int)grid_cluFeature.size_z);
                    if (grid_cluFeature.clusterType & CUDE_TYPE_ABOVE_LIDAR)
                    {
                        // 高于雷达的 应该为障碍物，不予显示
#if 1
                        drawCubeGLRGB(grid_cluFeature, 0, 1, 0);
                        //if (m_cfgVeloShow.showCubeID)
                        if (false)
                            drawTextRGB(pcalobj->scanGridClusterFeatureSet[cluster].max_x,
                                        pcalobj->scanGridClusterFeatureSet[cluster].max_y,
                                        pcalobj->scanGridClusterFeatureSet[cluster].max_z, 1, 1, 1, output);
#endif
                    }
                    else
                    {
                        //printf("Obstacle!!!!\n");
                        //drawCubeGLRGB(grid_cluFeature, 0, 1, 0);
                        //if (m_cfgVeloShow.showCubeID)
                        if (false)
                            drawTextRGB(pcalobj->scanGridClusterFeatureSet[cluster].max_x,
                                        pcalobj->scanGridClusterFeatureSet[cluster].max_y,
                                        pcalobj->scanGridClusterFeatureSet[cluster].max_z, 1, 1, 1, output);
                    }
                    //	}
                }//end of  if(glu.clusterType&CUDE_TYPE_IN_OBSTACLE_RANGE)
            }
        }// end of if(m_cfgVeloShow.showCube)
    }// end of if(pscanobj)

    //pthread_spin_unlock(&g_scanBuffer_lock);
    delete pcalobj;

    glEnd();
    //glRotatef(-angle, 0.0, 0.0, 1.0);
}
void drawPointsPieCell(float angle, int psize)
{
    //glRotatef(angle, 0.0, 0.0, 1.0);
    glPointSize(psize);
    glBegin(GL_POINTS);

    float* pColor;
    int size = 0;

    pthread_spin_lock(&g_scanBuffer_lock);
    VelodyneDataStruct* pscanobj = getScanRawForDraw();
    // 有很多行人目标粘连问题...需要解决
    if (pscanobj)
    {
        printf("draw points pie cell...\n");
        for (unsigned circle = 0; circle < CIRCLE_NUM; circle++)
        {
            size = pscanobj->shots.size();
            for (unsigned shot = 0; shot < size; shot++)
            {
                if ((*pscanobj).shots[shot].pt[circle].point_type & POINT_TYPE_INVALID)
                    continue;
                (*pscanobj).shots[shot].pt[circle].shotID = shot;
            }
        }

        // 计算饼型特征的各种数据，避免重复计算
        if (pscanobj->scanPieArray.size() == 0)
        {
            // 放进网格
            VelodyneAlgo::TransferToPieMatrix(g_CfgVeloView, *pscanobj);
            printf("TransferToPieMatrix done\n");
            // 计算网格特征
            VelodyneAlgo::CalcPieMatrixFeature(g_CfgVeloView, *pscanobj);
            printf("CalcPieMatrixFeature done\n");
            // 计算集群特征
            VelodyneAlgo::FindAndCalcPieClusterFeature(g_CfgVeloView, *pscanobj, false);
            printf("CalcPieMatrixFeature done\n");
            // 转为线结构并计算特征
            //VelodyneAlgo::CalClusterLineAndFeature(*pscanobj);
        }
        // 整个一周
        float high, low;
        float delta_z;
        for (unsigned azimuth = 0; azimuth < pscanobj->scanPieArray.size(); azimuth++)
        {
            VelodyneDataStruct::pieAzimuth_t& azimuthObj = pscanobj->scanPieArray[azimuth];
            for (unsigned rad = 0; rad < azimuthObj.size(); rad++)
            {
                VelodyneDataStruct::pieFeature_t& pieobj = pscanobj->scanPieFeatureArray[azimuth][rad];
                // 利用方形格网进行显示。
                //if (m_cfgVeloShow.showAllGridUnit){
                if (true)
                {
                    high = 0;
                    low = pieobj.min_z;

                    //if (m_cfgVeloShow.showGridType == 0)
                    //	high = pieobj.delta_z + pieobj.min_z;
                    //if (m_cfgVeloShow.showGridType == 1)
                    //	high = pieobj.delta_z / (pieobj.size*1.0) + pieobj.min_z;
                    //if (m_cfgVeloShow.showGridType == 2)
                    //	high = pieobj.size*1.0 + pieobj.min_z;
                    //if (m_cfgVeloShow.showGridType == 3)
                    //	high = pieobj.max_z;
                    //if (m_cfgVeloShow.showGridType == 4)
                    //	high = pieobj.ave_z;
                    //if (m_cfgVeloShow.showGridType == 5)
                    //	low = high = pieobj.ave_z;

                    // TODO: 直接使用高程平均差
                    //high = pieobj.delta_z / (pieobj.size*1.0) + pieobj.min_z;
                    // 使用最大高程
                    high = pieobj.max_z;

                    // 显示模式0:根据高程方差的jet颜色
                    if (g_CfgVeloView.cfgPlaneDetect.PieColorMode == 0)
                    {
                        delta_z = pieobj.delta_z / (pieobj.size*1.0);
                        pColor = color_util_jet(delta_z * g_CfgVeloView.cfgPlaneDetect.PieColorFactor);

                        drawCubeGLRGB(	pieobj.min_x, pieobj.min_y, low,
                                        pieobj.max_x, pieobj.max_y, high,
                                        pColor[0], pColor[1], pColor[2]);
                    }
                    // 显示模式1:根据高程方差的阈值分割两种颜色(红色-障碍物 蓝色-非障碍物)
                    else if (g_CfgVeloView.cfgPlaneDetect.PieColorMode == 1)
                    {
                        // 高程平均差
                        delta_z = pieobj.delta_z / (pieobj.size*1.0);
                        if (pieobj.cellType & CELL_TYPE_ABOVE_DELTA_Z)
                        {
                            //if (delta_z > g_CfgVeloView.cfgPlaneDetect.PieThresholdGroundDetect){
                            // 除去中空网格
                            if (!(pieobj.cellType & CELL_TYPE_NOT_HOLLOW))
                            {
                                continue;
                            }
                            // 高于雷达的障碍物 标红
                            if (pieobj.cellType & CELL_TYPE_ABOVE_LIDAR)
                            {
                                drawCubeGLRGB(pieobj.min_x, pieobj.min_y, low,
                                              pieobj.max_x, pieobj.max_y, high,
                                              1, 0, 0);
                            }
                            // 低于雷达的障碍物 标绿
                            else
                                drawCubeGLRGB(pieobj.min_x, pieobj.min_y, low,
                                              pieobj.max_x, pieobj.max_y, high,
                                              0, 1, 0);
                        }
                        else
                        {
#if 0
                            drawCubeGLRGB(	pieobj.min_x, pieobj.min_y, low,
                                            pieobj.max_x, pieobj.max_y, high,
                                            0, 0, 1);
#endif
                        }
                    }
                    // 显示模式2: 根据高程方差的阈值只显示地面颜色 灰色
                    else if (g_CfgVeloView.cfgPlaneDetect.PieColorMode == 2)
                    {
                        delta_z = pieobj.delta_z / (pieobj.size*1.0);
                        if (delta_z < g_CfgVeloView.cfgPlaneDetect.PieThresholdGroundDetect)
                            drawCubeGLRGB(	pieobj.min_x, pieobj.min_y, low,
                                            pieobj.max_x, pieobj.max_y, high,
                                            0.87, 0.87, 0.87);
                    }
                    // 显示模式3: 根据高度差小于30，则为路面，显示地面颜色
                    else if (g_CfgVeloView.cfgPlaneDetect.PieColorMode == 3)
                    {
                        if (pieobj.max_z - pieobj.min_z<30)
                        {
                            drawCubeGLRGB(	pieobj.min_x, pieobj.min_y, low,
                                            pieobj.max_x, pieobj.max_y, high,
                                            0.87, 0.87, 0.87);
                        }
                    }
                }
            } // end of for (unsigned rad = 0; rad<azimuthObj.size(); rad++){
        } // end of for (unsigned azimuth=0; azimuth<pscanobj->scanPieArray.size(); azimuth++){

        // 显示障碍物集群信息
        char output[256];
        memset(output, 0, sizeof(output));
        for (unsigned cluster = 0; cluster < (pscanobj->scanPieClusterSet.size()); ++cluster)
        {
            // 运动目标的特征结构
            VelodyneDataStruct::pieClusterFeature_t& pie_cluFeature = pscanobj->scanPieClusterFeatureSet[cluster];

            // 树、电杆之类
            if (pie_cluFeature.size_z > 200
                    && ((pie_cluFeature.size_x>pie_cluFeature.size_y ? pie_cluFeature.size_x : pie_cluFeature.size_y)) < 360)
            {
                //drawCubeGLRGB(pie_cluFeature, 1, 0, 1);
                //continue;
            }
            // 至少3.5米长，但宽小于1.4m，所以不可能为车
            else if ((pie_cluFeature.size_y > 350 && pie_cluFeature.size_x < 140)
                     || (pie_cluFeature.size_x>350 && pie_cluFeature.size_y < 140))
            {
                //drawCubeGLRGB(pie_cluFeature, 0, 1, 0);
                //continue;
            }
            else if (pie_cluFeature.size_z > 250)
            {
                //drawCubeGLRGB(pie_cluFeature, 0, 0, 1);
                //continue;
            }
            else if ((pie_cluFeature.size_x
                      > pie_cluFeature.size_y ? pie_cluFeature.size_x : pie_cluFeature.size_y) > 420
                     && pie_cluFeature.size_z<130)
            {
                //drawCubeGLRGB(pie_cluFeature, 0, 0, 1);
                //continue;
            }
            else if ((pie_cluFeature.size_x
                      > pie_cluFeature.size_y ? pie_cluFeature.size_x : pie_cluFeature.size_y) > 3.5
                     &&
                     ((pie_cluFeature.size_x > pie_cluFeature.size_y ? pie_cluFeature.size_x : pie_cluFeature.size_y) / (pie_cluFeature.size_x < pie_cluFeature.size_y ? pie_cluFeature.size_x : pie_cluFeature.size_y)>4))
            {
                //drawCubeGLRGB(pie_cluFeature, 0, 0, 1);
                //continue;
            }
            else if (pie_cluFeature.size_x < 700 && pie_cluFeature.size_y < 700 && pie_cluFeature.size_z > 100)
            {
                // 可能是车
            }

            // 过滤掉超过15米的 超级大的物体肯定是建筑物
            if (pie_cluFeature.size_x>1500 || pie_cluFeature.size_y > 1500
                    || pie_cluFeature.size_x*pie_cluFeature.size_y > 600 * 600)
            {
                //drawCubeGLRGB(pie_cluFeature, 0, 1, 0);
                //continue;
            }
            // 过滤掉面积特别大的 特别细长的大物体，可能是栅栏
            if (pie_cluFeature.size_x*pie_cluFeature.size_y > 500 * 500
                    && pie_cluFeature.size_x / pie_cluFeature.size_y < 1.5)
            {
                //drawCubeGLRGB(pie_cluFeature, 0, 1, 0);
                //continue;
            }
            // 过滤掉1米二以下的 小目标
            if (pie_cluFeature.size_z < 100)
            {
                //drawCubeGLRGB(pie_cluFeature, 1, 0.2, 1);
                //continue;
            }
            // 过滤掉长宽高总和小于3米的 //有可能是行人
            if ((pie_cluFeature.size_x + pie_cluFeature.size_y + pie_cluFeature.size_z) < 1.5)
            {
                //drawCubeGLRGB(pie_cluFeature, 0, 1, 1);
                //continue;
            }
            // 过滤掉超过宽度超过3米的
            if (pie_cluFeature.size_y > 300)
            {
                //drawCubeGLRGB(pie_cluFeature, 0, 0, 1);
                //continue;
            }
            // 有可能是行人
            if ((pie_cluFeature.size_x + pie_cluFeature.size_y) < 4)
            {
                //drawCubeGLRGB(pie_cluFeature, 1, 1, 1);
                //continue;
            }
            // 过滤掉长宽比大于3的  注:有很多公汽符合这个条件
            if (pie_cluFeature.size_x / pie_cluFeature.size_y > 3.0)
            {
                //drawCubeGLRGB(pie_cluFeature, 1, 1, 0);
                //continue;
            }

            sprintf(output, "%d.%dx%dx%d", cluster,
                    (int)pie_cluFeature.size_x,
                    (int)pie_cluFeature.size_y,
                    (int)pie_cluFeature.size_z);
            if (pie_cluFeature.clusterType & CUDE_TYPE_ABOVE_LIDAR)
            {
                // 高于雷达的 应该为障碍物，不予显示
#if 1
                drawCubeGLRGB(pie_cluFeature, 1., 1., 1.);
                //if (m_cfgVeloShow.showCubeID)
                if (1)
                    drawTextRGB(pie_cluFeature.max_x,
                                pie_cluFeature.max_y,
                                pie_cluFeature.max_z, 1., 1., 1., output);
#endif
            }
            else
            {
                // 真实的车辆
                //drawCubeGLRGB(pie_cluFeature, 1, 0, 0);
            }
        }
    }
    pthread_spin_unlock(&g_scanBuffer_lock);
    delete pscanobj;

    glEnd();
    //glRotatef(-angle, 0.0, 0.0, 1.0);
}

/*
 *	绘制三维点
 */
void drawPointRGB(VelodyneDataStruct::xpoint_t &pt, float r, float g, float b, int size)
{
    GLdouble dVect1[3];
    glColor3f(r, g, b);
    glPointSize(size);
    dVect1[0] = pt.x/100.;
    dVect1[1] = pt.y/100.;
    dVect1[2] = pt.z/100.;
    glVertex3dv(dVect1);
}
void drawPointRGB(float x, float y, float z, float psize, float r, float g, float b)
{
    glColor3f(r, g, b);
    glPointSize(psize);
    glBegin(GL_POINTS);
    glVertex3f(x, y, z);
    glEnd();
}
/*
 *	绘制立方体
 *	(min_x, min_y, min_z) --> (max_x, max_y, max_z)
 *	12条边
 */
void drawCubeGLRGB(float min_x, float min_y, float min_z, float max_x, float max_y, float max_z,
                   float r, float g, float b)
{
    drawLineGLRGB(min_x, min_y, max_z, max_x, min_y, max_z, r, g, b);
    drawLineGLRGB(min_x, max_y, max_z, max_x, max_y, max_z, r, g, b);
    drawLineGLRGB(min_x, max_y, min_z, max_x, max_y, min_z, r, g, b);
    drawLineGLRGB(min_x, min_y, min_z, max_x, min_y, min_z, r, g, b);

    drawLineGLRGB(min_x, min_y, max_z, min_x, max_y, max_z, r, g, b);
    drawLineGLRGB(max_x, min_y, max_z, max_x, max_y, max_z, r, g, b);
    drawLineGLRGB(min_x, min_y, min_z, min_x, max_y, min_z, r, g, b);
    drawLineGLRGB(max_x, min_y, min_z, max_x, max_y, min_z, r, g, b);

    drawLineGLRGB(min_x, min_y, min_z, min_x, min_y, max_z, r, g, b);
    drawLineGLRGB(max_x, min_y, min_z, max_x, min_y, max_z, r, g, b);
    drawLineGLRGB(max_x, max_y, min_z, max_x, max_y, max_z, r, g, b);
    drawLineGLRGB(min_x, max_y, min_z, min_x, max_y, max_z, r, g, b);
}

/*
 *	标记障碍物cluster
 */
void drawCubeGLRGB(VelodyneDataStruct::gridClusterFeature_t& grid_cluFeature, float r, float g, float b)
{
    drawCubeGLRGB(	grid_cluFeature.min_x,
                    grid_cluFeature.min_y,
                    grid_cluFeature.min_z,
                    grid_cluFeature.max_x,
                    grid_cluFeature.max_y,
                    grid_cluFeature.max_z,
                    r, g, b);
}
void drawCubeGLRGB(VelodyneDataStruct::pieClusterFeature_t& pie_cluFeature, float r, float g, float b)
{
    drawCubeGLRGB(  pie_cluFeature.min_x,
                    pie_cluFeature.min_y,
                    pie_cluFeature.min_z,
                    pie_cluFeature.max_x,
                    pie_cluFeature.max_y,
                    pie_cluFeature.max_z,
                    r, g, b);
}
void drawLineGLRGB(float x1, float y1, float z1, float x2, float y2, float z2,
                   float r, float g, float b, int wide)
{
    GLdouble dVect1[3], dVect2[3];
    glLineWidth(wide);
    glBegin(GL_LINES);
    glColor3d(r, g, b);

    dVect1[0] = x1/100.;
    dVect1[1] = y1/100.;
    dVect1[2] = z1/100.;
    glVertex3dv(dVect1);

    dVect2[0] = x2/100.;
    dVect2[1] = y2/100.;
    dVect2[2] = z2/100.;
    glVertex3dv(dVect2);
    glEnd();
}
/*
 *	绘制文本
 */
void drawTextRGB(float x, float y, float z, float r, float g, float b, char* outputstring)
{
    glPushMatrix();
    glColor3f(r, g, b);
#ifdef WIN32
    // Windows系统中，可以使用wglUseFontBitmaps函数来批量的产生显示字符用的显示列表
    wglUseFontBitmaps(wglGetCurrentDC(), 0, 255, 100);
#else
    //TODO: not-achieved function
    //glutUseFontBitmaps( ,0, 255, 100)
#endif

    glListBase(100);
    glRasterPos3f(x, y, z);
    glCallLists(strlen(outputstring), GL_UNSIGNED_BYTE, outputstring);

    glPopMatrix();
}

void drawCircle(float center_x, float center_y, float radius, float r, float g, float b, float psize)
{
    float angleInterval = 0.5;
    if (radius>2000)
        angleInterval = 0.25;
    for (float angle = 0; angle<360; angle += angleInterval)
    {
        float x = center_x + radius*sin(angle*M_PI / 180);
        float y = center_y + radius*cos(angle*M_PI / 180);
        drawPointRGB(x, y, g_CfgVeloView.cfgGlobal.GroundZ, psize, r, g, b);
    }
}

/***********************************************************************************************
 *		directly used codes in previous draw_velodyne.cpp
 */
int x_lbefore, y_lbefore;
int x_rbefore, y_rbefore;
int z_before1, z_before2;

bool buttonSaveLeft, buttonSaveMiddle, buttonSaveRight;
float x_move, y_move, z_move;
float x_move_save, y_move_save, z_move_save;
float x_rotate, y_rotate, z_rotate;
float x_rotate_save, y_rotate_save, z_rotate_save;
float m_zoom;

float m_aspect;
float m_eyex, m_eyey, m_eyez;
float m_centerx, m_centery, m_centerz;
float m_upx, m_upy, m_upz;

void  MyGLDispIni()
{
    m_eyex = 0, m_eyey = 0, m_eyez = 80;
    m_centerx = 0, m_centery = 0, m_centerz = 0;
    m_upx = 0, m_upy = 1, m_upz = 0;

    buttonSaveLeft = false;
    buttonSaveMiddle = false;
    buttonSaveRight = false;

    x_move = 0.0;
    y_move = 0.0;
    z_move = 0.0;
    x_rotate = 0.0;
    y_rotate = 0.0;
    z_rotate = 0.0;
    m_zoom = 1;

    x_lbefore = 0, y_lbefore = 0;
    x_rbefore = 0, y_rbefore = 0;
    z_before1 = 0, z_before2 = 0;

    GLenum type;
    type = GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE;
    glutInitDisplayMode(type);

    glutInitWindowSize(800, 600);
    glutCreateWindow("VelodyneHDL-Viewer");

    // 各种响应函数影响 Display Function 中的参数
    glutReshapeFunc(Reshape);
    //glutKeyboardFunc(Key);
    glutSpecialFunc(SpecialKey);
    //glutMouseFunc(MouseKey);
    //glutMotionFunc(MouseMove);
    //glutPassiveMotionFunc(PassiveMouseMove);
    //glutSpaceballRotateFunc(MouseRotate);
    // glutDisplayFunc(&myDisplay);  一般不这么写, 去掉&符号...
    glutDisplayFunc(myDisplay);
    glutIdleFunc(idleFunc);
    glutMainLoop();
}

void idleFunc(void){
    usleep(100000);
    glutPostRedisplay();
}

void myDisplay(void)
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0, 0.0, 0.0, 0.0);

    glLoadIdentity();

    gluLookAt(m_eyex, m_eyey, m_eyez,
              m_centerx, m_centery, m_centerz,
              m_upx, m_upy, m_upz);

    glScalef(1, 1, 1);

    glRotatef(x_rotate, 1, 0, 0);
    glRotatef(y_rotate, 0, 1, 0);
    glRotatef(z_rotate, 0, 0, 1);

    glTranslatef(x_move, y_move, z_move);
    glScalef(m_zoom, m_zoom, m_zoom);

    // draw the car model
    //drawCar(0.);
    // for 16-line -135
    drawCar(0.0);

    switch (show_state)
    {
    case SHOW_ALL_POINTS_ABOVE:
        drawAllPoints(1, 2);
        break;
    case SHOW_ALL_POINTS_BELOW:
        drawAllPoints(1, 1);
        break;
    case SHOW_OBSTACLES_GRID:
        drawAllPoints(1, 1);
        drawAllPoints(1, 2);
        drawPointsGridCell(-135, 1);
        break;
    case SHOW_OBSTACLES_PIE:
        drawAllPoints(1, 1);
        drawAllPoints(1, 2);
        drawPointsPieCell(-135, 1);
        break;
    case SHOW_HEIGHT_MAP:
        drawAllPoints(1, 1);
        drawAllPoints(1, 2);
        drawPointsHeightMap(1);
        break;
    case SHOW_NEGATIVE_OBSTACLE:
        drawAllPoints(1, 1);
        drawAllPoints(1, 2);
        drawPointsNagetiveObstacleMap(1);
    default:
        break;
    }

    glFlush();
    glutSwapBuffers();
}

void SpecialKey(int key, int x, int y)
{
    //int mod = 0;
    switch (key)
    {
    case GLUT_KEY_UP:
        //mod = glutGetModifiers();
        //if (mod == GLUT_ACTIVE_ALT){
        //}
        //else if (mod == GLUT_ACTIVE_SHIFT){
        //}
        //else if (mod == GLUT_ACTIVE_CTRL){
        //}
        //else
        y_move++;
        break;
    case GLUT_KEY_DOWN:
        //mod = glutGetModifiers();
        //if (mod == GLUT_ACTIVE_ALT){
        //}
        //else if (mod == GLUT_ACTIVE_SHIFT){
        //}
        //else if (mod == GLUT_ACTIVE_CTRL){
        //}
        //else
        y_move--;
        break;
    case GLUT_KEY_LEFT:
        //mod = glutGetModifiers();
        //if (mod == GLUT_ACTIVE_ALT){
        //}
        //else if (mod == GLUT_ACTIVE_SHIFT){
        //}
        //else if (mod == GLUT_ACTIVE_CTRL){
        //}
        //else
        x_move--;
        break;
    case GLUT_KEY_RIGHT:
        //mod = glutGetModifiers();
        //if (mod == GLUT_ACTIVE_ALT){
        //}
        //else if (mod == GLUT_ACTIVE_SHIFT){
        //}
        //else if (mod == GLUT_ACTIVE_CTRL){
        //}
        //else
        x_move++;
        break;
    // PgUp 放大
    case GLUT_KEY_PAGE_UP:
        m_zoom = 1.1 * m_zoom;
        break;
    // PgDn 缩小
    case GLUT_KEY_PAGE_DOWN:
        m_zoom = m_zoom / 1.1;
        break;
    case GLUT_KEY_HOME:
        m_zoom = 1.5 * m_zoom;
        break;
    case GLUT_KEY_END:
        m_zoom = m_zoom / 1.5;
        break;
    // 旋转操作
    case  GLUT_KEY_F1:
        x_rotate += 3;
        if (x_rotate > 360)
            x_rotate = x_rotate - 360;
        if (x_rotate < -360)
            x_rotate = x_rotate + 360;
        break;
    case  GLUT_KEY_F2:
        x_rotate += -3;
        if (x_rotate > 360)
            x_rotate = x_rotate - 360;
        if (x_rotate < -360)
            x_rotate = x_rotate + 360;
        break;
    case  GLUT_KEY_F3:
        y_rotate += 3;
        if (y_rotate > 360)
            y_rotate = y_rotate - 360;
        if (y_rotate < -360)
            y_rotate = y_rotate + 360;
        break;
    case  GLUT_KEY_F4:
        y_rotate += -3;
        if (y_rotate > 360)
            y_rotate = y_rotate - 360;
        if (y_rotate < -360)
            y_rotate = y_rotate + 360;
        break;
    case  GLUT_KEY_F5:
        z_rotate += atanf(3);
        if (z_rotate > 360)
            z_rotate = z_rotate - 360;
        if (z_rotate < -360)
            z_rotate = z_rotate + 360;
        break;
    case GLUT_KEY_F6:
        z_rotate += atanf(-3);
        if (z_rotate > 360)
            z_rotate = z_rotate - 360;
        if (z_rotate < -360)
            z_rotate = z_rotate + 360;
        break;
    case GLUT_KEY_F7:
        show_state = SHOW_OBSTACLES_PIE;
        break;
    case GLUT_KEY_F8:
        show_state = SHOW_NEGATIVE_OBSTACLE;
        break;
    case GLUT_KEY_F9:
        show_state = SHOW_HEIGHT_MAP;
        break;
    case GLUT_KEY_F10:
        show_state = SHOW_OBSTACLES_GRID;
        break;
    case GLUT_KEY_F11:
        show_state = SHOW_ALL_POINTS_BELOW;
        break;
    case GLUT_KEY_F12:
        show_state = SHOW_ALL_POINTS_ABOVE;
        break;
    default:
        return;
    }

    pthread_spin_lock(&g_glut_lock);
    glutPostRedisplay();
    pthread_spin_unlock(&g_glut_lock);
}

void Reshape(int w, int h)
{
    glViewport(0, 0, (GLint)w, (GLint)h);

    m_aspect = (GLfloat)w / (GLfloat)h;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(45.0, m_aspect, 0.0f, 4000.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

/***********************************************************************************************/
