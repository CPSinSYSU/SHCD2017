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

    //int MaxCalRad;						// 计算范围，超过该值不计算
    //int MinCalRad;						// 计算范围，小于该值不计算

    //int ColorMode;						// 全局显示模式，只有为0时，circle的显示模式为0才有效

    //	int CarFrontX;                    /**<车前端X值*/
    //	float ThresholdDeltaI;            /**< 反射强度阈值*/
    //	float DeltaIColorFactor;          /**< 反射强度颜色显示参数*/
    //	int   arg0;                       /**< reserved*/
    //	int   arg2;                       /**< reserved*/
    //	int   arg3;                       /**< reserved*/
    //	int   arg4;                       /**< reserved*/
    //	int   arg5;                       /**< reserved*/
    //	int   arg6;                       /**< reserved*/
    //	int   arg7;                       /**< reserved*/
    //	float ThresholdRoadEdgeDeltaZLLimit; /** 扫描线找路沿点deltaZ的下限*/
    //	float ThresholdRoadEdgeDeltaZULimit;/** 扫描线找路沿点deltaZ的上限*/
    //	int RoadCurbStartCircle;  /** 扫描线找路沿点:起始圈号*/
    //	int RoadCurbEndCircle;  /** 扫描线找路沿点:结束圈号*/
    //	int RoadCurbMaxDis;    /** 扫描线找路沿点:这个距离内的可用*/
    //	int MinLaneNum;        /** 当前路段最少车道数 */
    //	int MinLaneWidth;      /** 车道最小宽度 */
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


    //	int Enable;                        /**< 是否计算平面检测*/
    //	/** Pie形网格*/
    //	int DeleteSingleRedGridMaxRow;    /**< 删除本车道前方多少行格子范围内的孤立红格子。该值应小于等于发给规划网格的行数ROW */
    //	int DeleteSingleRedGridMaxNum;   /**< 删除本车道前方孤立红格子集群的最大数量，如3，就是说前方红格子的集群，包含三个（含）以下红格子的，都剔除 */
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
//	int ColorMode;                       /**< 该条circle的显示模式，0 为jet color 模式(在0-1区间内分布1024种颜色)，1为阈值模式(根据阈值分割不同颜色)*/
//	float ColorFactor;                   /**< 在jet color 模式下 用来调整Delta R颜色分布0-1*/
//	float ColorFactorByDeltaZ;           /**< 在jet color 模式下 用来调整Delta Z颜色分布0-1*/
} CfgCircleItem_t;

//typedef struct{
//	int offset[CIRCLE_NUM];
//} CfgShotOffset;
//typedef struct{
//	float DisL;/**<车头离左路口的距离*/
//	float DisR;/**<车头离右路口的距离*/
//	float LastDisLAboveZero;/**<上一次车头离左路口距离大于零的距离*/
//	float LastDisRAboveZero;/**<上一次车头离右路口距离大于零的距离*/
//	bool State1mOr5m; /**<为false时提供给控制5m,10m,15m的数据,为true时提供给控制1m,3m,6m数据*/
//	bool StartCheckOut;/**为真时开始检测是否已过路口**/
//	bool OutOfInterSection;/**<是否已过路口*/
//	int  CheckDistance[6];/**<给定检测车头到路口的距离*/
//	bool LeftReach[6];/**<在给定距离处是否到左路口*/
//	bool RightReach[6];/**<在给定距离处是否到右路口*/
//	int  SearchWide;/**<搜索的方格格数**/
//	int  SatisfyNum;/**<认为是路口的最小方格格数**/
//	int  SatisfyContiniousNum;/**<认为是路口必须具有的连续方格数，要保证足够车通过**/
//}  CfgInterSection;
// 新的路口算法参数，新的算法成熟稳定后会把旧的算法清理掉
//typedef struct{
//	float RoadWide;				/**<由GIS告诉的所在行驶道路宽度,单位m**/
//	float IntersectionWide;		/**<由路网和GIS告诉的路口宽度,单位m**/
//	float CheckPassageDis;		/**<预定转弯路径能否通过的检核值,决定是否可以开始转弯**/
//	float PositionDecision;		/**<当检测路口宽度大于此值时，预定车转到路口四分之一位置，小于此值，转到路口中间**/
//	float Multiple;				/**<为探索转弯半径和转角关系临时设置，给定半径求得的理论转角所乘倍数**/
//	float Power;				/**<为探索转弯半径和转角关系临时设置，固定转弯半径求得的内轮和外轮角度权重**/
//	float TestAngle;
//	float YPosAlliance;
//	float safeDis;
//	float turnLeftLimitDis;
//	float turnRightDis;
//	float endPos;
//	float turnRightScale;
//	float frontCalDis;
//	float frontMaxIntervalLWide;
//	float frontMaxIntervalLendXDis;
//	float interSectionAdvanceDis;
//	float roadShapeLengthFilter;
//	float slashLengthLimit;
//	float slashWidth;
//} CfgIntersection;

//typedef struct{
//	int min_length;
//	int max_length;
//	int min_width;
//	int max_width;
//	int max_length_width_ratio;
//}CfgCarShapeParameter;

// 总体配置项结构体定义
typedef struct
{
    CfgGlobal_t cfgGlobal;										// 全局配置项
    CfgPlaneDetect_t cfgPlaneDetect;							// 平面检测项
    CfgCircleItem_t cfgCircleItems[CIRCLE_NUM];					// Circle项
    CfgIgnoreCircleWhenProjection_t cfgIgnoreCircleWhenProjection;
    CfgNegativeObstacle_t cfgNegativeObstacle;					// 负障碍配置项
    //CfgShotOffset_t cfgShotOffset;
    //CfgInterSection_t cfginterSection;						// 旧的路口配置项
    //CfgIntersection_t cfgIntersecion;							// 新的路口配置项
    //CfgCarShapeParameter_t cfgCarShapeParameter;
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

