#ifndef _CONFIG_STRUCT_H_
#define _CONFIG_STRUCT_H_

/* *************************************************************************
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
 *
 ***************************************************************************/

// 几线激光雷达
#ifdef USE_VLP_16_
#define CIRCLE_NUM 16
#elif USE_HDL_64E_
#define CIRCLE_NUM 64
#else
#define CIRCLE_NUM 32
#endif

// 全局配置项定义
typedef struct
{
    int GroundZ;						// 地面高程

    int ThresholdMinHollow;				// 中空网格判断，最小值
    int ThresholdMaxHollow;				// 中空网格判断，最大值

    int ShotCalcFrom;					// 从第ShotCalcFrom个shot开始计算特征
    int ShotCalcNum;					// 一共计算ShotCalcNum个shot
    int ShotShowFrom;					// 从第ShotShowFrom个shot开始显示
    int ShotShowNum;					// 一共显示ShotShowNum个shot

    int ContinusLineMinDistInCircle;	// Circle中连续线段最小距离

    float ThresholdMinRad;				// 距离阈值

    int CarLength;
    int CarWidth;
    int CarHeight;

    // distance filter for raw data
    int MaxLaserDistacne;
    int MinLaserDistance;
} CfgGlobal_t;
// 平面检测配置选项定义
typedef struct
{
    // 方形网格
    int GridCellSize;					// 单元格大小，单位为厘米
    int GridHeadRowNum;					// 单元格的前方的行数（从车头前方起）*/
    int GridBackRowNum;					// 本车后方单元格的行数(从最后方起，沿车前进方向 第0行，第1行...)
    int GridColumnNum;					// 单元格数量，网格为正方形，原点在正中间，中点到4边的网格数量(//TODO: 应该是横排或者竖排方格数量)
    float GridThresholdGroundDetect;	// 平面检测 判断阈值，小于该数为平面
    float GridThresholdDiffZ;
    float GridColorFactor;				// 在jet color 模式下 用来调整检测值颜色分布0-1
    int GridColorMode;					// 该条circle的显示模式
    // 0 为jet color模式(在0-1区间内分布1024种颜色)
    // 1 为阈值模式(根据阈值分割不同颜色)

    // 饼型网格
    int PieRadSize;					    // 平面检测(半径方向上相同旋角)单元格半径方向的大小，单位为厘米*/
    int PieShotFrom;                    // shot range for valid obstacle detection
    int PieShotEnd;
    int PieAzimuthNum;					// 把圆周分成AzimuthSize份
    int PieMinRad;						// 平面检测计算范围 左区间(半径范围)
    int PieMaxRad;						// 平面检测计算范围 右区间
    float PieThresholdGroundDetect;		// 平面检测 判断阈值，小于该数为平面
    float PieThresholdDiffZ;
    float PieColorFactor;				// 在jet color 模式下 用来调整检测值颜色分布0-1
    int PieColorMode;					// 该条circle的显示模式
    // 0 为jet color模式(在0-1区间内分布1024种颜色)
    // 1 为阈值模式(根据阈值分割不同颜色)


} CfgPlaneDetect_t;
// 负障碍检测算法参数
typedef struct
{
    int NegativeObstacleCellSize;				// 网格边长(cm)
    int NegativeObstacleGridDim;				// 网格大小
    float NegativeObstacleThresholdStartEdge;	// 用来判断“坑”靠近车体的那个边缘
    float NegativeObstacleThresholdEndEdge;		// 用来判断“坑”远离车体的另一个边缘
    int NegativeObstacleThresholdEdgeNumber;	// 用于判断该网格是否为坑的边缘个数
} CfgNegativeObstacle_t;
// 方型网格投影时，忽略的垂直角度平面配置项
typedef struct
{
    int IgnoreCircle[32];
} CfgIgnoreCircleWhenProjection_t;

// 每个circle的配置选项定义
typedef struct
{
    int Enable;								// 是否使用该条Circle对应的激光雷达
    float ThresholdMinDist;					// 距离阈值，小于该阈值 表示非平面
    float ThresholdRoadEdge;				// Delta R 阈值，小于该值认为是平台路面，否则为路沿等
    float ThresholdRoadEdgeByDeltaZ;     // Delta Z 阈值，小于该值认为是平台路面，否则为路沿等
} CfgCircleItem_t;

// 总体配置项结构体定义
typedef struct
{
    CfgGlobal_t cfgGlobal;										// 全局配置项
    CfgPlaneDetect_t cfgPlaneDetect;							// 平面检测项
    CfgCircleItem_t cfgCircleItems[CIRCLE_NUM];					// Circle项
    CfgIgnoreCircleWhenProjection_t cfgIgnoreCircleWhenProjection;
    CfgNegativeObstacle_t cfgNegativeObstacle;					// 负障碍配置项
} CfgVeloView_t;

/*
 *  从配置文件读入配置到配置结构体
 *  @param[out] cfg       要写入的结构体
 *  @param[in]  filename  要读入的文件
 *  @return     0:成功 其他失败
 */
int LoadCfgVeloView(CfgVeloView_t& cfg, const char* filechar);
/*
 *    把配置结构体写入到配置文件
 *    @param[in]  cfg       要读取的结构体
 *    @param[out] filename  要写入的文件
 *    @return     0:成功 其他失败
 */
int SaveCfgVeloView(CfgVeloView_t& cfg, const char* filename);

extern CfgVeloView_t g_CfgVeloView;								// 全局配置结构体的声明

#endif

