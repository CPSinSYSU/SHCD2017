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
 *	#pragma pack(i)(i = 1,2,4,8,16)�����ö����ֽ���Ŀ
 *	vs����������Ŀ����-��������-c/c++-��������-�ṹ��Ա��������
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
    // ����Lidar���ϰ��� <--- ���ڵ�57�߼���(height order)���ڴ�ֱ�Ƕ�ƽ��(���64�߼����״�)
    // #57 ---> verticalAngle:-0.022350(����ˮƽ)
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

/*************************************** ��������� ***************************************/
#define POINT_TYPE_INITIAL       0x00000000
// ��Ч�㣬�򵽾����⣬���������Ƕ���ľ���֮��
#define POINT_TYPE_INVALID       0x00000001
// ���ڰ뾶��ֵƽ������ֵ
#define POINT_TYPE_ABOVE_DELTA_R 0x00000002
// С�ڰ뾶��ֵƽ������ֵ
#define POINT_TYPE_BELOW_DELTA_R 0x00000004
// ���ڸ߳̾�ֵƽ������ֵ
#define POINT_TYPE_ABOVE_DELTA_Z 0x00000008
// С�ڸ߳̾�ֵƽ������ֵ
#define POINT_TYPE_BELOW_DELTA_Z 0x00000010
// ���ڰ뾶��ֵ
#define POINT_TYPE_ABOVE_R       0x00000020
// С�ڰ뾶��ֵ
#define POINT_TYPE_BELOW_R       0x00000040
// �ӱ�Ե��
#define POINT_TYPE_HOLE_EDGE     0x00000080
// ���ڵ�
#define POINT_TYPE_HOLE_INSIDE   0x00000100

/**************************************** �������� *****************************************/
#define CELL_TYPE_INITIAL				0x00000000
// ��ЧCell������û�е�
#define CELL_TYPE_INVALID       		0x00000001
////TODO: ���ڰ뾶��ֵƽ������ֵ
//#define CELL_TYPE_ABOVE_DELTA_R  		0x00000002
////TODO: С�ڰ뾶��ֵƽ������ֵ
//#define CELL_TYPE_BELOW_DELTA_R  		0x00000004
// ���ڸ߳̾�ֵƽ������ֵ
#define CELL_TYPE_ABOVE_DELTA_Z  		0x00000008
// С�ڸ߳̾�ֵƽ������ֵ
#define CELL_TYPE_BELOW_DELTA_Z  		0x00000010
// ���ڰ뾶��ֵ
#define CELL_TYPE_ABOVE_R       		0x00000020
// С�ڰ뾶��ֵ
#define CELL_TYPE_BELOW_R       		0x00000040
// ���״��
#define CELL_TYPE_ABOVE_LIDAR       	0x00000080
// ���ϰ�����ټ����뷶Χ��
#define CELL_TYPE_IN_OBSTACLE_RANGE		0x00000100
// ���пգ���У԰�������ͷ����������Ϊ�п�
#define CELL_TYPE_NOT_HOLLOW       		0x00000200
// ���ϰ���
#define CELL_TYPE_NEGATIVE_OBSTACLE     0x00000400

/**************************************** ��Ⱥ���� *****************************************/
// ���״��
#define CUDE_TYPE_ABOVE_LIDAR       	0x00000001
// ���ϰ�����ټ����뷶Χ��
#define CUDE_TYPE_IN_OBSTACLE_RANGE		0x00000002
///TODO: �ø�������ˮƽ sick ��ɨ��㣬˵���ø������ϰ������
//#define CUDE_TYPE_CONTAIN_SICK        	0x00000400


class VelodyneDataRaw
{
public:
    static const char magic[4];

    /*
     *	Velodyne data structure(1206 bytes in each packet)
     *		12*(2[start identifier]+2[Rotational]+32*3)
     *	  + 6(status data:4[GPS timestamp]+1[Status Byte]+1[Status Value])
     *	// TODO: �ڴ���� sizeof(vel_laser_t) == 4 <---- #pragma pack(1)
     */
    typedef struct
    {
        uint16_t distance;  										// 2mm increments
        uint8_t intensity;  										// 255 being brightest
    }  velodyne_fire_t;

    /*
     *	����16�߼����״� ����ͬ���ǵ�����fire��Ϊһ��block
     *	����64�߼����״� һ��fire��ֳ�����block
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

        // ��¼һ����������������
    typedef struct
    {
        // ����
        float x, y, z;
        // �����ĸ� circle(��ֱ�Ƕ�ƽ��)
        int circleID;
        // �����Ĵ� shot
        int shotID;
        // �뾶�� y/x��ֵ(��ά��ͶӰ��x-yƽ��)
        float rad, tan_theta;
        // ��ɫ�ͷ���ǿ��
        unsigned char r, g, b, i;
        // �̡߳��뾶������ǿ�ȵ�ƽ����
        float scanline_drtZ;
        float scanline_drtRad;
        float scanline_drtI;
        // reservd
        float scanline_countZ;
        float scanline_countD;
        // �������
        int point_type;
        int objectID;
    } xpoint_t;

    // 32�߼����״�һ��shot��32���㣻16�߼����״���16����
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
    *	// TODO: �ڴ���� sizeof(vel_laser_t) == 4 <---- #pragma pack(1)
    */
    typedef struct
    {
        uint16_t distance;  										// 2mm increments
        uint8_t intensity;  										// 255 being brightest
    }  velodyne_fire_t;
    /*
    *	����16�߼����״� ����ͬ���ǵ�����fire��Ϊһ��block
    *	����64�߼����״� һ��fire��ֳ�����block
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

    // ��¼һ����������������
    typedef struct
    {
        // ����
        float x, y, z;
        // �����ĸ� circle(��ֱ�Ƕ�ƽ��)
        int circleID;
        // �����Ĵ� shot
        int shotID;
        // �뾶�� y/x��ֵ(��ά��ͶӰ��x-yƽ��)
        float rad, tan_theta;
        // ��ɫ�ͷ���ǿ��
        unsigned char r, g, b, i;
        // �̡߳��뾶������ǿ�ȵ�ƽ����
        float scanline_drtZ;
        float scanline_drtRad;
        float scanline_drtI;
        // reservd
        float scanline_countZ;
        float scanline_countD;
        // �������
        int point_type;
        int objectID;
    } xpoint_t;

    // ��ʾһ��Circle�е�һ�������߶�
    typedef struct
    {
        // ��ʼ��źͽ������
        int start;
        int end;
    } continuousLine_t;
    // ��ʾһ��Circle�е��߶μ���
    typedef std::vector<continuousLine_t> continuousLines_t;

    // 32�߼����״�һ��shot��32���㣻16�߼����״���16����
    typedef struct
    {
        xpoint_t pt[VELODYNE_NUM_BEAMS_IN_ONE_SHOT];
    } shot_t;

    /** ͳ����������û��ô��*/
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

    // ������Ԫ: ��������Ԫ����������˸õ�Ԫ�������е��ָ��
    typedef std::vector<xpoint_t*> cell_t;
    // �뾶�����ϵ� ��ͬ���ǵ� һ�и�����Ԫ
    typedef std::vector<cell_t> pieAzimuth_t;
    // �������������ͺͷ�����
    typedef std::vector<pieAzimuth_t> pieArray_t;

    // ������Ԫ������
    typedef struct
    {
        // x,y,z�ľ�ֵ
        float ave_x, ave_y, ave_z;
        // x,y,z����ֵ
        float min_x, min_y, min_z;
        float max_x, max_y, max_z;
        // �߳�ƽ����
        float delta_z;
        // �ж��ٸ�circle�ĵ�
        int circleNum;
        // ��ĸ���
        int size;
        unsigned int cellType;
        // ָ�������ݽṹ
        cell_t* pCell;
        /*
         *	������Ԫ�ڸ����е�λ��
         *		columnID: �뾶��������ͬ���ǵ�һ�и�����Ԫ�±�
         *		cellID: ���и�����Ԫ�����ڵ���ĸ����±�
         */
        int azimuthID;
        int radID;
    } pieFeature_t;

    // ��ͬ���Ƿ����ϵ�������������
    typedef std::vector<pieFeature_t> pieFeatureAzimuth_t;
    // ����������������
    typedef std::vector<pieFeatureAzimuth_t> pieFeatureArray_t;


    // һ����Ⱥ���кü�������ÿ��������������
    typedef std::vector<pieFeature_t*> pieCluster_t;
    // ��Ⱥ����
    typedef std::vector<pieCluster_t> pieClusterSet_t;
    // ��Ⱥ������
    typedef struct
    {
        // x,y,z����ֵ
        float min_x, min_y, min_z;
        float max_x, max_y, max_z;
        float size_x, size_y, size_z;
        // ����ÿ��cell�� point��������Ȩ ����������ģ��������ʾ
        float theta, radius;
        // ��Ⱥ��cell������
        int size;
        // ��Ⱥ��point������
        int pointNumber;
        unsigned int clusterType;
    } pieClusterFeature_t;
    typedef std::vector<pieClusterFeature_t> pieClusterFeatureSet_t;

    typedef std::vector<xpoint_t*> line_t;
    typedef std::vector<line_t> circleLine_t;
    typedef std::vector<circleLine_t> clusterLine_t;
    typedef std::vector<clusterLine_t> clusterLineArray_t;
    typedef std::vector<clusterLineArray_t> clusterLineFeatureArray_t;

    // ������Ԫ:����Ⱥ
    //typedef std::vector<xpoint_t*> cell_t;
    // һ�����ŵ�һ�з���Ԫ
    typedef std::vector<cell_t> gridRow_t;
    // ��������: ������
    typedef std::vector<gridRow_t> gridArray_t;
    /* ����Ԫ���� */
    typedef struct
    {
        // x,y,z�ľ�ֵ
        float ave_x, ave_y, ave_z;
        // x,y,z����ֵ
        float min_x, min_y, min_z;
        float max_x, max_y, max_z;
        // �����Сy ��Ӧ��xֵ
        float min_y_x, max_y_x;
        // �߳���ƽ���̵߳�ƫ���
        float delta_z;
        // �ж��ٸ�circle�ĵ�
        int circleNum;
        // ��ĸ���
        int size;
        // ��������
        unsigned int cellType;
        unsigned int clusterID;
    } gridFeature_t;
    // һ������������������
    typedef std::vector<gridFeature_t> gridFeatureRow_t;
    // ��������������������
    typedef std::vector<gridFeatureRow_t> gridFeatureArray_t;

    // ����Ⱥ(����������Ϣ����) һ����Ⱥ���кü�������ÿ��������������
    typedef std::vector<gridFeature_t*> gridCluster_t;
    // ��Ⱥ����
    typedef std::vector<gridCluster_t> gridClusterSet_t;

    /*
     *	����Ⱥ����
     *	��Ŀ�������
     */
    typedef struct
    {
        // x,y,z����ֵ
        float min_x, min_y, min_z;
        float max_x, max_y, max_z;
        // �����СY ��Ӧ��x
        float min_y_x, max_y_x;
        // Ŀ��ߴ�
        float size_x, size_y, size_z;
        // ��Ⱥ��С
        int size;
        unsigned int clusterType;
    } gridClusterFeature_t;

    typedef std::vector<gridClusterFeature_t> gridClusterFeatureSet_t;

    // ��Ⱥ��ÿһȦ����С�߶�����
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
    // ÿ��Circle��һ�������߶α�ʾ
    std::vector<continuousLines_t> circlesLines;

    /****************************** ������ ******************************/
    // scanCellArray��������������(cell)���ݣ�����ͨ��cell�õ����еĵ�
    pieArray_t scanPieArray;
    // scanCellFeatureArray��scanCellArray��һһ��Ӧ�Ĺ�ϵ����������cell������
    pieFeatureArray_t scanPieFeatureArray;


    /****************************** ��Ⱥ���� ******************************/
    pieClusterSet_t scanPieClusterSet;
    pieClusterFeatureSet_t scanPieClusterFeatureSet;

    clusterLineArray_t  scanClusterLineArray;
    clusterLineFeatureArray_t scanClusterLineFeatureArray;


    /******************************** ���� ********************************/
    gridArray_t scanGridArray;
    gridFeatureArray_t scanGridFeatureArray;
    gridClusterSet_t scanGridClusterSet;
    // ����ĵ㼯�ϵ�Ŀ��
    gridClusterFeatureSet_t scanGridClusterFeatureSet;

    // �����Ե��
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
    // �����Ƶ�������Ե��ͷ���
    msgLIDAR_t scanMsgLIDAR;
    //���Ʒ����ĵ�ǰ���ٺͳ��ֽǶ�
    typedef struct
    {
        float carSpeed;
        int wheelAngel;
    } msgFromCtrl_t;
    msgFromCtrl_t scanMsgFromCtrl;
    // 1m. 3m. 6m��������Ե��
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
    // 5m.10m.15m��������Ե��
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
    // ��ż��㳵ǰ�������������12����Ե��X����
    float scanTwelvePointsAtAnyDisX[12];
    // ��ż��㳵ǰ�������������12����Ե��Y����
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

