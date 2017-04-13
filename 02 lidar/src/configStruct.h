#ifndef _CONFIG_STRUCT_H_
#define _CONFIG_STRUCT_H_

/* *************************************************************************
 *
 *	ȫ������
 *	�����ļ� ./velodyne.ini
 *	���ýṹ��
 *		CfgVeloView: ����������ṹ�嶨��
 *			CfgCircleItem: ÿ��circle������ѡ���
 *			CfgPlaneDetect: ƽ��������ѡ���
 *			CfgGlobal: ȫ���������
 *			CfgShotOffset:
 *			CfgInterSection: ·���㷨����
 *			CfgIntersection: ·���㷨����(�µ�·���㷨)
 *			CfgIgnoreCircleWhenProjection:
 *			CfgCarShapeParameter:
 *
 ***************************************************************************/

// ���߼����״�
#ifdef USE_VLP_16_
#define CIRCLE_NUM 16
#elif USE_HDL_64E_
#define CIRCLE_NUM 64
#else
#define CIRCLE_NUM 32
#endif

// ȫ���������
typedef struct
{
    int GroundZ;						// ����߳�

    int ThresholdMinHollow;				// �п������жϣ���Сֵ
    int ThresholdMaxHollow;				// �п������жϣ����ֵ

    int ShotCalcFrom;					// �ӵ�ShotCalcFrom��shot��ʼ��������
    int ShotCalcNum;					// һ������ShotCalcNum��shot
    int ShotShowFrom;					// �ӵ�ShotShowFrom��shot��ʼ��ʾ
    int ShotShowNum;					// һ����ʾShotShowNum��shot

    int ContinusLineMinDistInCircle;	// Circle�������߶���С����

    float ThresholdMinRad;				// ������ֵ

    int CarLength;
    int CarWidth;
    int CarHeight;

    // distance filter for raw data
    int MaxLaserDistacne;
    int MinLaserDistance;
} CfgGlobal_t;
// ƽ��������ѡ���
typedef struct
{
    // ��������
    int GridCellSize;					// ��Ԫ���С����λΪ����
    int GridHeadRowNum;					// ��Ԫ���ǰ�����������ӳ�ͷǰ����*/
    int GridBackRowNum;					// �����󷽵�Ԫ�������(��������س�ǰ������ ��0�У���1��...)
    int GridColumnNum;					// ��Ԫ������������Ϊ�����Σ�ԭ�������м䣬�е㵽4�ߵ���������(//TODO: Ӧ���Ǻ��Ż������ŷ�������)
    float GridThresholdGroundDetect;	// ƽ���� �ж���ֵ��С�ڸ���Ϊƽ��
    float GridThresholdDiffZ;
    float GridColorFactor;				// ��jet color ģʽ�� �����������ֵ��ɫ�ֲ�0-1
    int GridColorMode;					// ����circle����ʾģʽ
    // 0 Ϊjet colorģʽ(��0-1�����ڷֲ�1024����ɫ)
    // 1 Ϊ��ֵģʽ(������ֵ�ָͬ��ɫ)

    // ��������
    int PieRadSize;					    // ƽ����(�뾶��������ͬ����)��Ԫ��뾶����Ĵ�С����λΪ����*/
    int PieShotFrom;                    // shot range for valid obstacle detection
    int PieShotEnd;
    int PieAzimuthNum;					// ��Բ�ֳܷ�AzimuthSize��
    int PieMinRad;						// ƽ������㷶Χ ������(�뾶��Χ)
    int PieMaxRad;						// ƽ������㷶Χ ������
    float PieThresholdGroundDetect;		// ƽ���� �ж���ֵ��С�ڸ���Ϊƽ��
    float PieThresholdDiffZ;
    float PieColorFactor;				// ��jet color ģʽ�� �����������ֵ��ɫ�ֲ�0-1
    int PieColorMode;					// ����circle����ʾģʽ
    // 0 Ϊjet colorģʽ(��0-1�����ڷֲ�1024����ɫ)
    // 1 Ϊ��ֵģʽ(������ֵ�ָͬ��ɫ)


} CfgPlaneDetect_t;
// ���ϰ�����㷨����
typedef struct
{
    int NegativeObstacleCellSize;				// ����߳�(cm)
    int NegativeObstacleGridDim;				// �����С
    float NegativeObstacleThresholdStartEdge;	// �����жϡ��ӡ�����������Ǹ���Ե
    float NegativeObstacleThresholdEndEdge;		// �����жϡ��ӡ�Զ�복�����һ����Ե
    int NegativeObstacleThresholdEdgeNumber;	// �����жϸ������Ƿ�Ϊ�ӵı�Ե����
} CfgNegativeObstacle_t;
// ��������ͶӰʱ�����ԵĴ�ֱ�Ƕ�ƽ��������
typedef struct
{
    int IgnoreCircle[32];
} CfgIgnoreCircleWhenProjection_t;

// ÿ��circle������ѡ���
typedef struct
{
    int Enable;								// �Ƿ�ʹ�ø���Circle��Ӧ�ļ����״�
    float ThresholdMinDist;					// ������ֵ��С�ڸ���ֵ ��ʾ��ƽ��
    float ThresholdRoadEdge;				// Delta R ��ֵ��С�ڸ�ֵ��Ϊ��ƽ̨·�棬����Ϊ·�ص�
    float ThresholdRoadEdgeByDeltaZ;     // Delta Z ��ֵ��С�ڸ�ֵ��Ϊ��ƽ̨·�棬����Ϊ·�ص�
} CfgCircleItem_t;

// ����������ṹ�嶨��
typedef struct
{
    CfgGlobal_t cfgGlobal;										// ȫ��������
    CfgPlaneDetect_t cfgPlaneDetect;							// ƽ������
    CfgCircleItem_t cfgCircleItems[CIRCLE_NUM];					// Circle��
    CfgIgnoreCircleWhenProjection_t cfgIgnoreCircleWhenProjection;
    CfgNegativeObstacle_t cfgNegativeObstacle;					// ���ϰ�������
} CfgVeloView_t;

/*
 *  �������ļ��������õ����ýṹ��
 *  @param[out] cfg       Ҫд��Ľṹ��
 *  @param[in]  filename  Ҫ������ļ�
 *  @return     0:�ɹ� ����ʧ��
 */
int LoadCfgVeloView(CfgVeloView_t& cfg, const char* filechar);
/*
 *    �����ýṹ��д�뵽�����ļ�
 *    @param[in]  cfg       Ҫ��ȡ�Ľṹ��
 *    @param[out] filename  Ҫд����ļ�
 *    @return     0:�ɹ� ����ʧ��
 */
int SaveCfgVeloView(CfgVeloView_t& cfg, const char* filename);

extern CfgVeloView_t g_CfgVeloView;								// ȫ�����ýṹ�������

#endif

