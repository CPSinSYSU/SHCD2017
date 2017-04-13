#ifndef __VELODYNE_DATA_STRUCT_H__
#define __VELODYNE_DATA_STRUCT_H__

/**************************************************************************
 *
 * Velodyne collecting data structures, including grid map cell...
 *
 * hold all scanned data and some analysis data
 *
 ***************************************************************************/
/*
 * added by durant35
 *	#pragma pack(i)(i = 1,2,4,8,16)来设置对齐字节数目
 *	vs还可以在项目属性-配置属性-c/c++-代码生成-结构成员对齐设置
 */
#pragma pack(1)

#include <stdint.h>
#include <vector>

#define VELODYNE_NUM_BEAMS_IN_ONE_BLOCK			32
#define VELODYNE_NUM_BLOCKS_IN_ONE_PKT			12

#define VELODYNE_DATA_PORT						2368
#define VELODYNE_POSITION_PORT					8308

#define VELODYNE_PACKET_SIZE					1248
#define VELODYNE_DATA_SIZE				        1206
#define VELODYNE_UDP_OFFSET						42
#define VELODYNE_PACKET_WITH_PCAPHDR_SIZE		1264
#define VELODYNE_UDP_PCAP_OFFSET				58

#ifdef USE_VLP_16_
    #define VELODYNE_NUM_BEAMS_IN_ONE_SHOT		16
    #define PKT_NUM_IN_CIRCLE_LOWER_BOUND		75
    // height order: #7 ---> verticalAngle:-1.00
    // height order: #8 ---> verticalAngle:1.00
    #define UPPER_VERT_FROM						8
    #define DOWN_VERT_TO						7
#elif USE_HDL_64E_
    #define VELODYNE_NUM_BEAMS_IN_ONE_SHOT		64
    #define PKT_NUM_IN_CIRCLE_LOWER_BOUND		360
    // 高于Lidar的障碍物 <--- 高于第57线激光(height order)所在垂直角度平面(针对64线激光雷达)
    // #57 ---> verticalAngle:-0.022350(近似水平)
    // #58 ---> 0.317822
    #define DOWN_VERT_TO						7
    #define UPPER_VERT_FROM						57
#else
    #define VELODYNE_NUM_BEAMS_IN_ONE_SHOT		32
    #define PKT_NUM_IN_CIRCLE_LOWER_BOUND		180
    // height order: #22 ---> verticalAngle:-1.33
    // height order: #23 ---> verticalAngle:0.00
    // height order: #24 ---> verticalAngle:1.33
    #define UPPER_VERT_FROM						23
    #define DOWN_VERT_TO						UPPER_VERT_FROM
#endif

/*************************************** 激光点类型 ***************************************/
#define POINT_TYPE_INITIAL       0x00000000
// 无效点，打到距离外，或者在我们定义的距离之外
#define POINT_TYPE_INVALID       0x00000001
// 大于半径均值平均差阈值
#define POINT_TYPE_ABOVE_DELTA_R 0x00000002
// 小于半径均值平均差阈值
#define POINT_TYPE_BELOW_DELTA_R 0x00000004
// 大于高程均值平均差阈值
#define POINT_TYPE_ABOVE_DELTA_Z 0x00000008
// 小于高程均值平均差阈值
#define POINT_TYPE_BELOW_DELTA_Z 0x00000010
// 大于半径阈值
#define POINT_TYPE_ABOVE_R       0x00000020
// 小于半径阈值
#define POINT_TYPE_BELOW_R       0x00000040
// 坑边缘点
#define POINT_TYPE_HOLE_EDGE     0x00000080
// 坑内点
#define POINT_TYPE_HOLE_INSIDE   0x00000100

/**************************************** 格网类型 *****************************************/
#define CELL_TYPE_INITIAL				0x00000000
// 无效Cell，里面没有点
#define CELL_TYPE_INVALID       		0x00000001
////TODO: 大于半径均值平均差阈值
//#define CELL_TYPE_ABOVE_DELTA_R  		0x00000002
////TODO: 小于半径均值平均差阈值
//#define CELL_TYPE_BELOW_DELTA_R  		0x00000004
// 大于高程均值平均差阈值
#define CELL_TYPE_ABOVE_DELTA_Z  		0x00000008
// 小于高程均值平均差阈值
#define CELL_TYPE_BELOW_DELTA_Z  		0x00000010
// 大于半径阈值
#define CELL_TYPE_ABOVE_R       		0x00000020
// 小于半径阈值
#define CELL_TYPE_BELOW_R       		0x00000040
// 比雷达高
#define CELL_TYPE_ABOVE_LIDAR       	0x00000080
// 在障碍物跟踪检测距离范围内
#define CELL_TYPE_IN_OBSTACLE_RANGE		0x00000100
// 非中空，在校园里面如果头顶有树，则为中空
#define CELL_TYPE_NOT_HOLLOW       		0x00000200
// 负障碍物
#define CELL_TYPE_NEGATIVE_OBSTACLE     0x00000400

/**************************************** 集群类型 *****************************************/
// 比雷达高
#define CUDE_TYPE_ABOVE_LIDAR       	0x00000001
// 在障碍物跟踪检测距离范围内
#define CUDE_TYPE_IN_OBSTACLE_RANGE		0x00000002
///TODO: 该格子里有水平 sick 的扫描点，说明该格子是障碍物格子
//#define CUDE_TYPE_CONTAIN_SICK        	0x00000400


class VelodyneDataRaw
{
public:
    static const char magic[4];

    /*
     *	Velodyne data structure(1206 bytes in each packet)
     *		12*(2[start identifier]+2[Rotational]+32*3)
     *	  + 6(status data:4[GPS timestamp]+1[Status Byte]+1[Status Value])
     *	// TODO: 内存对齐 sizeof(vel_laser_t) == 4 <---- #pragma pack(1)
     */
    typedef struct
    {
        uint16_t distance;  										// 2mm increments
        uint8_t intensity;  										// 255 being brightest
    }  velodyne_fire_t;

    /*
     *	对于16线激光雷达 将相同旋角的两次fire归为一个block
     *	对于64线激光雷达 一次fire拆分成两个block
     */
    typedef struct
    {
        uint16_t start_identifier;
        uint16_t rotational_pos;									// 0-35999  divide by 100 for degrees
        velodyne_fire_t lasers[VELODYNE_NUM_BEAMS_IN_ONE_BLOCK];
    }  velodyne_block_t;

    typedef struct
    {
        velodyne_block_t blocks[VELODYNE_NUM_BLOCKS_IN_ONE_PKT];
        uint8_t GPS_timestamp[4];
        uint8_t status_byte;
        uint8_t status_value;
    }  velodyne_packet_t;

        // 记录一个激光点的所有数据
    typedef struct
    {
        // 坐标
        float x, y, z;
        // 属于哪个 circle(垂直角度平面)
        int circleID;
        // 属于哪次 shot
        int shotID;
        // 半径和 y/x的值(三维点投影到x-y平面)
        float rad, tan_theta;
        // 颜色和反射强度
        unsigned char r, g, b, i;
        // 高程、半径、反射强度的平均差
        float scanline_drtZ;
        float scanline_drtRad;
        float scanline_drtI;
        // reservd
        float scanline_countZ;
        float scanline_countD;
        // 点的类型
        int point_type;
        int objectID;
    } xpoint_t;

    // 32线激光雷达一个shot有32个点；16线激光雷达有16个点
    typedef struct
    {
        xpoint_t pt[VELODYNE_NUM_BEAMS_IN_ONE_SHOT];
    } shot_t;

    std::vector<shot_t> shots;

    ~VelodyneDataRaw();
    VelodyneDataRaw();

    int serialize(const char* filename, bool saveToPCD);
    int deserialize(const char* filename);
};

class VelodyneDataStruct
{
public:
    //static const char magic[4];

    /*
    *	Velodyne data structure(1206 bytes in each packet)
    *		12*(2[start identifier]+2[Rotational]+32*3)
    *	  + 6(status data:4[GPS timestamp]+1[Status Byte]+1[Status Value])
    *	// TODO: 内存对齐 sizeof(vel_laser_t) == 4 <---- #pragma pack(1)
    */
    typedef struct
    {
        uint16_t distance;  										// 2mm increments
        uint8_t intensity;  										// 255 being brightest
    }  velodyne_fire_t;
    /*
    *	对于16线激光雷达 将相同旋角的两次fire归为一个block
    *	对于64线激光雷达 一次fire拆分成两个block
    */
    typedef struct
    {
        uint16_t start_identifier;
        uint16_t rotational_pos;									// 0-35999  divide by 100 for degrees
        velodyne_fire_t lasers[VELODYNE_NUM_BEAMS_IN_ONE_BLOCK];
    }  velodyne_block_t;

    typedef struct
    {
        velodyne_block_t blocks[VELODYNE_NUM_BLOCKS_IN_ONE_PKT];
        uint8_t GPS_timestamp[4];
        uint8_t status_byte;
        uint8_t status_value;
    }  velodyne_packet_t;

    // 记录一个激光点的所有数据
    typedef struct
    {
        // 坐标
        float x, y, z;
        // 属于哪个 circle(垂直角度平面)
        int circleID;
        // 属于哪次 shot
        int shotID;
        // 半径和 y/x的值(三维点投影到x-y平面)
        float rad, tan_theta;
        // 颜色和反射强度
        unsigned char r, g, b, i;
        // 高程、半径、反射强度的平均差
        float scanline_drtZ;
        float scanline_drtRad;
        float scanline_drtI;
        // reservd
        float scanline_countZ;
        float scanline_countD;
        // 点的类型
        int point_type;
        int objectID;
    } xpoint_t;

    // 表示一个Circle中的一个连续线段
    typedef struct
    {
        // 开始序号和结束序号
        int start;
        int end;
    } continuousLine_t;
    // 表示一个Circle中的线段集合
    typedef std::vector<continuousLine_t> continuousLines_t;

    // 32线激光雷达一个shot有32个点；16线激光雷达有16个点
    typedef struct
    {
        xpoint_t pt[VELODYNE_NUM_BEAMS_IN_ONE_SHOT];
    } shot_t;

    /** 统计量，现在没怎么用*/
    typedef struct
    {
        float min_x, min_y, min_z;
        float max_x, max_y, max_z;
        float min_rad, max_rad, avg_rad;
        unsigned char min_i, max_i;
        float   min_scanline_drtZ;
        float   min_scanline_drtRad;
        float   min_scanline_countZ;
        float   min_scanlinecountD;
        float   max_scanline_drtZ;
        float   max_scanline_drtRad;
        float   max_scanline_countZ;
        float   max_scanline_countD;
    } pointStatistics_t;

    // 格网单元: 饼型网格单元，里面包含了该单元所含所有点的指针
    typedef std::vector<xpoint_t*> cell_t;
    // 半径方向上的 相同旋角的 一列格网单元
    typedef std::vector<cell_t> pieAzimuth_t;
    // 整个格网，饼型和方格形
    typedef std::vector<pieAzimuth_t> pieArray_t;

    // 格网单元的特征
    typedef struct
    {
        // x,y,z的均值
        float ave_x, ave_y, ave_z;
        // x,y,z的最值
        float min_x, min_y, min_z;
        float max_x, max_y, max_z;
        // 高程平均差
        float delta_z;
        // 有多少个circle的点
        int circleNum;
        // 点的个数
        int size;
        unsigned int cellType;
        // 指向点的数据结构
        cell_t* pCell;
        /*
         *	格网单元在格网中的位置
         *		columnID: 半径方向上相同旋角的一列格网单元下标
         *		cellID: 该列格网单元中由内到外的格网下标
         */
        int azimuthID;
        int radID;
    } pieFeature_t;

    // 相同旋角方向上的网格特征数组
    typedef std::vector<pieFeature_t> pieFeatureAzimuth_t;
    // 整个网格特征数组
    typedef std::vector<pieFeatureAzimuth_t> pieFeatureArray_t;


    // 一个集群里有好几个网格，每个网格特征相似
    typedef std::vector<pieFeature_t*> pieCluster_t;
    // 集群数组
    typedef std::vector<pieCluster_t> pieClusterSet_t;
    // 集群的特征
    typedef struct
    {
        // x,y,z的最值
        float min_x, min_y, min_z;
        float max_x, max_y, max_z;
        float size_x, size_y, size_z;
        // 根据每个cell中 point的数量加权 计算出的中心，极坐标表示
        float theta, radius;
        // 集群中cell的数量
        int size;
        // 集群中point的数量
        int pointNumber;
        unsigned int clusterType;
    } pieClusterFeature_t;
    typedef std::vector<pieClusterFeature_t> pieClusterFeatureSet_t;

    typedef std::vector<xpoint_t*> line_t;
    typedef std::vector<line_t> circleLine_t;
    typedef std::vector<circleLine_t> clusterLine_t;
    typedef std::vector<clusterLine_t> clusterLineArray_t;
    typedef std::vector<clusterLineArray_t> clusterLineFeatureArray_t;

    // 格网单元:方格群
    //typedef std::vector<xpoint_t*> cell_t;
    // 一个竖排的一列方格单元
    typedef std::vector<cell_t> gridRow_t;
    // 整个格网: 方格形
    typedef std::vector<gridRow_t> gridArray_t;
    /* 方格单元特征 */
    typedef struct
    {
        // x,y,z的均值
        float ave_x, ave_y, ave_z;
        // x,y,z的最值
        float min_x, min_y, min_z;
        float max_x, max_y, max_z;
        // 最大最小y 对应的x值
        float min_y_x, max_y_x;
        // 高程与平均高程的偏差和
        float delta_z;
        // 有多少个circle的点
        int circleNum;
        // 点的个数
        int size;
        // 格网类型
        unsigned int cellType;
        unsigned int clusterID;
    } gridFeature_t;
    // 一个竖排网格特征数组
    typedef std::vector<gridFeature_t> gridFeatureRow_t;
    // 整个方格网络特征数组
    typedef std::vector<gridFeatureRow_t> gridFeatureArray_t;

    // 方格集群(方格特征信息集合) 一个集群里有好几个方格，每个方格特征相似
    typedef std::vector<gridFeature_t*> gridCluster_t;
    // 集群数组
    typedef std::vector<gridCluster_t> gridClusterSet_t;

    /*
     *	方格群特征
     *	动目标的特征
     */
    typedef struct
    {
        // x,y,z的最值
        float min_x, min_y, min_z;
        float max_x, max_y, max_z;
        // 最大最小Y 对应的x
        float min_y_x, max_y_x;
        // 目标尺寸
        float size_x, size_y, size_z;
        // 集群大小
        int size;
        unsigned int clusterType;
    } gridClusterFeature_t;

    typedef std::vector<gridClusterFeature_t> gridClusterFeatureSet_t;

    // 集群上每一圈线里小线段特征
    typedef struct
    {
        float length;
        int ptNum;
        float startAngle;
        float endAngle;
        float aveZ;
        float aveX;
        float aveY;
    } clusterLineSegFeature_t;
    typedef struct
    {
        float totalLength;
        float maxSegLength;
        int segNum;
        int ptNum;
        float startAngle;
        float endAngle;
        float aveZ;
        float aveX;
        float aveY;

        bool Lshape;
        float LshapeDirection;
        double LshapeBoxVex[4][2];
        float LshapeDeflection;
        float LshapeLength;
        float LshapeWidth;

        std::vector <clusterLineSegFeature_t> lineSegFeature;
    } clusterCircleLineFeature_t;
    typedef struct
    {
        int ptNum;
        int validCircleNum;
        float totalLength;
        float maxSegLength;
        float maxCircleLineLength;
        float averageCircleLineLength;
        float aveZ;
        float aveX;
        float aveY;
        bool Lshape;
        float LshapeDirection;
        float LshapeBoxVex[4][2];
        float LshapeDeflection;
        float LshapeLength;
        float LshapeWidth;
        std::vector <clusterCircleLineFeature_t> circleLineFeature;
    } clusterLineFeature_t;

    std::vector<shot_t> shots;
    pointStatistics_t scanStatistics;
    std::vector<pointStatistics_t> circlesStatistics;
    //vector<pointStatistics_t> shotsStatistics;
    // 每个Circle由一组连续线段表示
    std::vector<continuousLines_t> circlesLines;

    /****************************** 网格处理 ******************************/
    // scanCellArray包含了所有网格(cell)数据，并能通过cell得到所有的点
    pieArray_t scanPieArray;
    // scanCellFeatureArray和scanCellArray是一一对应的关系，包含所有cell的特征
    pieFeatureArray_t scanPieFeatureArray;


    /****************************** 集群处理 ******************************/
    pieClusterSet_t scanPieClusterSet;
    pieClusterFeatureSet_t scanPieClusterFeatureSet;

    clusterLineArray_t  scanClusterLineArray;
    clusterLineFeatureArray_t scanClusterLineFeatureArray;


    /******************************** 方格 ********************************/
    gridArray_t scanGridArray;
    gridFeatureArray_t scanGridFeatureArray;
    gridClusterSet_t scanGridClusterSet;
    // 聚类的点集合的目标
    gridClusterFeatureSet_t scanGridClusterFeatureSet;

    // 方格边缘点
    typedef  struct
    {
        int current_state;
        float lx1, ly1;
        float lx2, ly2;
        float lx3, ly3;
        float rx1, ry1;
        float rx2, ry2;
        float rx3, ry3;
        char gridArray[100][100];
    } msgLIDAR_t;
    // 给控制的六个边缘点和方格
    msgLIDAR_t scanMsgLIDAR;
    //控制反馈的当前车速和车轮角度
    typedef struct
    {
        float carSpeed;
        int wheelAngel;
    } msgFromCtrl_t;
    msgFromCtrl_t scanMsgFromCtrl;
    // 1m. 3m. 6m的六个边缘点
    typedef  struct
    {
        float lx1, ly1;
        float lx2, ly2;
        float lx3, ly3;
        float rx1, ry1;
        float rx2, ry2;
        float rx3, ry3;
    } SixPoints1m3m6m_t;
    SixPoints1m3m6m_t scanSixPoints1m3m6m;
    // 5m.10m.15m的六个边缘点
    typedef  struct
    {
        float lx1, ly1;
        float lx2, ly2;
        float lx3, ly3;
        float rx1, ry1;
        float rx2, ry2;
        float rx3, ry3;
    } SixPoints5m10m15m_t;
    SixPoints5m10m15m_t scanSixPoints5m10m15m;

    float scanSixPointsAtAnyDisX[3];
    float scanSixPointsAtAnyDisYL[3];
    float scanSixPointsAtAnyDisYR[3];
    // 存放计算车前任意六个距离的12个边缘点X坐标
    float scanTwelvePointsAtAnyDisX[12];
    // 存放计算车前任意六个距离的12个边缘点Y坐标
    float scanTwelvePointsAtAnyDisY[12];

    VelodyneDataStruct();
    ~VelodyneDataStruct();

    //int DumpStatistics(const char* filename);
    void InitStatistics(pointStatistics_t& pStatistics);

    bool sorted;
    bool calcCircleFeatured;
    bool calcCirclesLined;
    bool calcRoadCurb;
    int circleGridWidth;
    bool isTrackerHandled;

};

#endif

