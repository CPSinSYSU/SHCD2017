/**************************************************************************
 *
 * ʹ��У׼�㷨��У׼���ݽ������ݱ궨
 *
 * This is the header for the velodyne ladar interface drivers.
 *
 ***************************************************************************/

//#define _PRINT_ORDER_
//#define USE_NEW_SOLVER

// define for fopen()
#define _CRT_SECURE_NO_WARNINGS
#include "velodyneDriver.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <assert.h>
#include <stdio.h>
#include <algorithm>
#include <iostream>

#include "configStruct.h"

// degree --> radian(M_PI/180)
#define RADIANS_PER_LSB 0.0174532925
#define METERS_PER_CM 0.01
#define TWOPI_INV (0.5/M_PI)
#define TWOPI (2*M_PI)
//#define METERS_PER_LSB 0.002

static const float DISTANCE_RESOLUTION = 0.002f;

static const uint16_t UPPER_BANK = 0xEEFF;
static const uint16_t LOWER_BANK = 0xDDFF;

// �궨����(64��)
#ifdef USE_HDL_64E_
static double velodyne_calibrated[VELODYNE_NUM_BEAMS_IN_ONE_SHOT][5] =
{
    //vertCorrection  rotCorrection  distCorrection  vertOffsetCorrection  horizOffsetCorrection
    { -7.1581192, -4.5, 102, 21.560343, -2.5999999 },
    { -6.8178215, -3.4000001, 125, 21.516994, 2.5999999 },
    { 0.31782165, 3, 130, 20.617426, -2.5999999 },
    { 0.65811908, 4.5999999, 128, 20.574717, 2.5999999 },
    { -6.4776502, -0.5, 112, 21.473722, -2.5999999 },
    { -6.1375928, 1, 125, 21.430525, 2.5999999 },
    { -8.520812, -1.5, 106, 21.734608, -2.5999999 },
    { -8.1798887, 0.40000001, 127, 21.690901, 2.5999999 },
    { -5.797637, 4, 111, 21.387396, -2.5999999 },
    { -5.4577708, 5.5, 126, 21.34433, 2.5999999 },
    { -7.8391404, 3.0999999, 113, 21.647291, -2.5999999 },
    { -7.4985547, 4.5, 123, 21.603773, 2.5999999 },
    { -3.0802133, -4.5, 105, 21.044245, -2.5999999 },
    { -2.7406337, -3.2, 133, 21.001518, 2.5999999 },
    { -5.1179824, -5.5, 110, 21.301321, -2.5999999 },
    { -4.7782598, -4, 129, 21.258366, 2.5999999 },
    { -2.4010365, -0.2, 111, 20.958813, -2.5999999 },
    { -2.0614092, 1, 130, 20.916126, 2.5999999 },
    { -4.4385905, -1.2, 115, 21.215462, -2.5999999 },
    { -4.0989642, 0, 133, 21.172602, 2.5999999 },
    { -1.7217404, 3.8, 113, 20.873451, -2.5999999 },
    { -1.3820176, 5, 130, 20.830786, 2.5999999 },
    { -3.7593663, 3, 117, 21.129782, -2.5999999 },
    { -3.4197867, 4.5, 129, 21.086998, 2.5999999 },
    { 0.998555, -4.5, 107, 20.531982, -2.5999999 },
    { 1.339141, -3.2, 131, 20.489222, 2.5999999 },
    { -1.0422293, -5.4000001, 128, 20.788124, -2.5999999 },
    { -0.70236301, -4, 134, 20.745461, 2.5999999 },
    { 1.679889, -0.5, 124, 20.446428, -2.5999999 },
    { 2.0208123, 1, 136, 20.403601, 2.5999999 },
    { -0.36240739, -1.5, 131, 20.702793, -2.5999999 },
    { -0.022349782, 0.2, 136, 20.660116, 2.5999999 },
    { -22.737886, -7.8000002, 101, 16.019152, -2.5999999 },
    { -22.226072, -5, 88, 15.954137, 2.5999999 },
    { -11.513928, 4.5, 121, 14.680806, -2.5999999 },
    { -11.002114, 7.4000001, 88, 14.623099, 2.5999999 },
    { -21.714685, -1, 94, 15.889649, -2.5999999 },
    { -21.203688, 2, 88, 15.82566, 2.5999999 },
    { -24.790272, -2.5, 114, 16.284933, -2.5999999 },
    { -24.276321, 0.5, 89, 16.217583, 2.5999999 },
    { -20.693031, 6, 98, 15.762167, -2.5999999 },
    { -20.182682, 9, 92, 15.699132, 2.5999999 },
    { -23.762968, 4.5, 107, 16.15085, -2.5999999 },
    { -23.250172, 7.5, 80, 16.084715, 2.5999999 },
    { -16.615318, -7.5, 121, 15.26925, -2.5999999 },
    { -16.105938, -5, 92, 15.209245, 2.5999999 },
    { -19.672594, -9, 119, 15.63654, -2.5999999 },
    { -19.162729, -6, 89, 15.574372, 2.5999999 },
    { -15.596496, -1, 109, 15.14954, -2.5999999 },
    { -15.086954, 2, 88, 15.090119, 2.5999999 },
    { -18.653046, -2, 117, 15.51261, -2.5999999 },
    { -18.143503, 0.69999999, 88, 15.451235, 2.5999999 },
    { -14.577271, 5.5, 112, 15.030966, -2.5999999 },
    { -14.067405, 8.3999996, 87, 14.972065, 2.5999999 },
    { -17.634062, 5, 119, 15.390228, -6.1999998 },
    { -17.124681, 7.5, 97, 15.329572, 2.5999999 },
    { -10.489829, -7.5, 119, 14.565539, -2.5999999 },
    { -9.9770317, -4.6999998, 95, 14.508112, 2.5999999 },
    { -13.557318, -8.5, 126, 14.913401, -2.5999999 },
    { -13.046968, -6, 92, 14.854958, 2.5999999 },
    { -9.4636793, -1, 112, 14.450804, -2.5999999 },
    { -8.949728, 1.5, 93, 14.3936, 2.5999999 },
    { -12.536313, -2, 121, 14.796721, -2.5999999 },
    { -12.025314, 0.40000001, 96, 14.738676, 2.5999999 },
};
#elif USE_VLP_16_
static double velodyne_calibrated[VELODYNE_NUM_BEAMS_IN_ONE_SHOT][5] =
{
    //vertCorrection  rotCorrection  distCorrection  vertOffsetCorrection  horizOffsetCorrection
    { -15, 0, 0.0, 0.0, 0 }, // laser 0
    { 1, 0, 0.0, 0.0, 0 }, // laser 1
    { -13, 0, 0.0, 0.0, 0 }, // laser 2
    { 3, 0, 0.0, 0.0, 0 }, // laser 3
    { -11, 0, 0.0, 0.0, 0 }, // laser 4
    { 5, 0, 0.0, 0.0, 0 }, // laser 5
    { -9, 0, 0.0, 0.0, 0 }, // laser 6
    { 7, 0, 0.0, 0.0, 0 }, // laser 7
    { -7, 0, 0.0, 0.0, 0 }, // laser 8
    { 9, 0, 0.0, 0.0, 0 }, // laser 9
    { 11, 0, 0.0, 0.0, 0 }, // laser 11
    { -3, 0, 0.0, 0.0, 0 }, // laser 12
    { 13, 0, 0.0, 0.0, 0 }, // laser 13
    { -5, 0, 0.0, 0.0, 0 }, // laser 10
    { -1, 0, 0.0, 0.0, 0 }, // laser 14
    { 15, 0, 0.0, 0.0, 0 }, // laser 15
};
#else
/*
 *	follow 32db.xml and 63-9113 HDL-32E manual_Rev E_NOV2012.pdf, part <LaserFiring Sequence>
 */
#ifdef USE_NEW_SOLVER
static double velodyne_calibrated[VELODYNE_NUM_BEAMS_IN_ONE_SHOT][5] =
{
    //vertCorrection(������)  rotCorrection  distCorrection  vertOffsetCorrection  horizOffsetCorrection
    { -0.5352924815866609, 0, 0, 0, 0 },
    { -0.1628392174657417, 0, 0, 0, 0 },
    { -0.5119050696099369, 0, 0, 0, 0 },
    { -0.13962634015954636, 0, 0, 0, 0 },
    { -0.4886921905584123, 0, 0, 0, 0 },
    { -0.11641346285335104, 0, 0, 0, 0 },
    { -0.4654793115068877, 0, 0, 0, 0 },
    { -0.09302604738596851, 0, 0, 0, 0 },
    { -0.44209189953016365, 0, 0, 0, 0 },
    { -0.06981317007977318, 0, 0, 0, 0 },
    { -0.4188790204786391, 0, 0, 0, 0 },
    { -0.046600292773577856, 0, 0, 0, 0 },
    { -0.39566614142711454, 0, 0, 0, 0 },
    { -0.023212879051524585, 0, 0, 0, 0 },
    { -0.3722787294503905, 0, 0, 0, 0 },
    { 0.00, 0, 0, 0, 0 },
    { -0.3490658503988659, 0, 0, 0, 0 },
    { 0.023212879051524585, 0, 0, 0, 0 },
    { -0.32585297134734137, 0, 0, 0, 0 },
    { 0.046600292773577856, 0, 0, 0, 0 },
    { -0.30246555937061725, 0, 0, 0, 0 },
    { 0.06981317007977318, 0, 0, 0, 0 },
    { -0.2792526803190927, 0, 0, 0, 0 },
    { 0.09302604738596851, 0, 0, 0, 0 },
    { -0.25603980126756815, 0, 0, 0, 0 },
    { 0.11641346285335104, 0, 0, 0, 0 },
    { -0.23265238929084414, 0, 0, 0, 0 },
    { 0.13962634015954636, 0, 0, 0, 0 },
    { -0.20943951023931956, 0, 0, 0, 0 },
    { 0.1628392174657417, 0, 0, 0, 0 },
    { -0.18622663118779495, 0, 0, 0, 0 },
    { 0.18622663118779495, 0, 0, 0, 0 }
};
#else
static double velodyne_calibrated[VELODYNE_NUM_BEAMS_IN_ONE_SHOT][5] =
{
    //vertCorrection(�Ƕ���)  rotCorrection  distCorrection  vertOffsetCorrection  horizOffsetCorrection
    { -30.67, 0, 0, 0, 0 },
    { -9.33, 0, 0, 0, 0 },
    { -29.33, 0, 0, 0, 0 },
    { -8.00, 0, 0, 0, 0 },
    { -28.00, 0, 0, 0, 0 },
    { -6.66, 0, 0, 0, 0 },
    { -26.66, 0, 0, 0, 0 },
    { -5.33, 0, 0, 0, 0 },
    { -25.33, 0, 0, 0, 0 },
    { -4.00, 0, 0, 0, 0 },
    { -24.00, 0, 0, 0, 0 },
    { -2.67, 0, 0, 0, 0 },
    { -22.67, 0, 0, 0, 0 },
    { -1.33, 0, 0, 0, 0 },
    { -21.33, 0, 0, 0, 0 },
    { 0.00, 0, 0, 0, 0 },
    { -20.00, 0, 0, 0, 0 },
    { 1.33, 0, 0, 0, 0 },
    { -18.67, 0, 0, 0, 0 },
    { 2.67, 0, 0, 0, 0 },
    { -17.33, 0, 0, 0, 0 },
    { 4.00, 0, 0, 0, 0 },
    { -16.00, 0, 0, 0, 0 },
    { 5.33, 0, 0, 0, 0 },
    { -14.67, 0, 0, 0, 0 },
    { 6.67, 0, 0, 0, 0 },
    { -13.33, 0, 0, 0, 0 },
    { 8.00, 0, 0, 0, 0 },
    { -12.00, 0, 0, 0, 0 },
    { 9.33, 0, 0, 0, 0 },
    { -10.07, 0, 0, 0, 0 },
    { 10.07, 0, 0, 0, 0 }
};
#endif		// end of #ifdef USE_NEW_SOLVER
#endif		// end of #ifdef USE_HDL_64E_

// У������
// ����ˮƽ��ת��У����(/rad)
double rotCorrection[VELODYNE_NUM_BEAMS_IN_ONE_SHOT];
//
int rotOffset[VELODYNE_NUM_BEAMS_IN_ONE_SHOT];
// ������xoz���ڵ�ͶӰ��x���ļн�(VerticalAngle: omiga)
double vertCorrection[VELODYNE_NUM_BEAMS_IN_ONE_SHOT];
// ���벹�������������ռ���֮���ľ���
double distCorrection[VELODYNE_NUM_BEAMS_IN_ONE_SHOT];
// �����ڵĴ�ֱƫ�ƣ��������ڸ߶�
double vert_offsetCorrection[VELODYNE_NUM_BEAMS_IN_ONE_SHOT];
// �����ڵ�ˮƽƫ�ƣ�����������xoy����ͶӰ�ϵ�yֵ
double horiz_offsetCorrection[VELODYNE_NUM_BEAMS_IN_ONE_SHOT];

double sin_vertCorrection[VELODYNE_NUM_BEAMS_IN_ONE_SHOT];
double cos_vertCorrection[VELODYNE_NUM_BEAMS_IN_ONE_SHOT];
// ��ת��������ֵ
double sinAzimuth[36000];
double cosAzimuth[36000];
// ����������ת��������ֵ
double cor_sinAzimuth[VELODYNE_NUM_BEAMS_IN_ONE_SHOT][36000];
double cor_cosAzimuth[VELODYNE_NUM_BEAMS_IN_ONE_SHOT][36000];


static double absf(double a)
{
    if (a < 0)
        return -a;
    return a;
}
// valid only for v > 0
static inline double mod2pi_positive(double vin)
{
    double q = vin * TWOPI_INV + 0.5;
    int qi = (int)q;

    return vin - qi * TWOPI;
}
// Map v to [-PI, PI]
static inline double mod2pi(double vin)
{
    if (vin < 0)
        return -mod2pi_positive(-vin);
    else
        return mod2pi_positive(vin);
}
// Return vin such that it is within PI degrees of ref
static inline double mod2pi_ref(double ref, double vin)
{
    return ref + mod2pi(vin - ref);
}
/*
 *		logical: packet�м�����˳��0-VELODYNE_NUM_BEAMS_IN_ONE_SHOT
 *		physical: ��ֱƽ����->��
 */
int heightOrder2firingOrder[VELODYNE_NUM_BEAMS_IN_ONE_SHOT];
int firingOrder2heightOrder[VELODYNE_NUM_BEAMS_IN_ONE_SHOT];

int velodyne_firingOrder_to_heightOrder(int firing_id)
{
    return firingOrder2heightOrder[firing_id];
}

int velodyne_heightOrder_to_firingOrder(int height_id)
{
    return heightOrder2firingOrder[height_id];
}
/*
 *	����qsort�ıȽϺ���
 *	�����״�У������(��ֱ�Ƕ�ƽ����->��)��С��������
 *		return value				meaning
 *			<0			The element pointed to by p1 goes before the element pointed to by p2
 *			0			The element pointed to by p1 is equivalent to the element pointed to by p2
 *			>0			The element pointed to by p1 goes after the element pointed to by p2
 */
int laser_phi_compare(const void *_a, const void *_b)
{
    int a = *((int*)_a);
    int b = *((int*)_b);
    if (velodyne_calibrated[a][0] < velodyne_calibrated[b][0])
        return -1;
    return 1;
}
/*
 *		�����״�����
 *		��ʽ: i, ang, tan_ang, 230/tan_ang
 */
int SaveRad()
{
    int i;
    FILE* p = fopen(".\\rad.txt", "w");
    for (i = 0; i < VELODYNE_NUM_BEAMS_IN_ONE_SHOT; i++)
    {
        float ang = -velodyne_calibrated[heightOrder2firingOrder[i]][0];
        float tan_ang = tan((ang / 360) * 2 * M_PI);
        fprintf(p, "[%d]%f %f %f\n", i, ang, tan_ang, 230 / tan_ang);
    }
    fclose(p);
    return 0;
}
/*
 *	У׼����Ԥ����, ������������:
 *	 logical2physical[VELODYNE_NUM_LASERS_BEAM_IN_ONE_SHOT] = 0~VELODYNE_NUM_LASERS_BEAM_IN_ONE_SHOT���ռ���У������velodyne_calibrated[i][0]�Ӵ���С����
 *	 physical2logical[logical2physical[logical]] = 0~VELODYNE_NUM_LASERS_BEAM_IN_ONE_SHOT
 *
 *	 vertCorrection[i] = ( velodyne_calibrated[i][0] ) * RADIANS_PER_LSB;
 *	 rotCorrection[i] = ( velodyne_calibrated[i][1] ) * RADIANS_PER_LSB;
 *	 rotOffset[i] = velodyne_calibrated[logical2physical[i]][1]*6;
 *	 distCorrection[i] = velodyne_calibrated[i][2] * METERS_PER_CM;
 *	 vertoffsetCorrection[i] = velodyne_calibrated[i][3] * METERS_PER_CM;
 *	 horizdffsetCorrection[i] = velodyne_calibrated[i][4] * METERS_PER_CM;
 *
 *	 sintheta[64][36000] = sin(��������λ��)
 *	 costheta[64][36000] = cos(��������λ��)
 *	 sinctheta[36000] = sin(ԭʼ��λ��)
 *	 cosctheta[36000] = cos(ԭʼ��λ��)
 */
int velodyne_calib_precompute()
{
    for (unsigned firingOrder = 0; firingOrder < VELODYNE_NUM_BEAMS_IN_ONE_SHOT; firingOrder++)
        heightOrder2firingOrder[firingOrder] = firingOrder;
    /*
     *	��ֱƽ����->�� ���߼�packet�еļ���������˳����������
     *	logical2physical: ͨ�����±���ȡ��Ӧ��Circle ID(Vertical Angle�ɵ͵���)
     *	heightOrder2firingOrder: ͨ��Circle ID��ȡ��Ӧ�İ��±�
     */
    qsort(heightOrder2firingOrder, VELODYNE_NUM_BEAMS_IN_ONE_SHOT, sizeof(int), laser_phi_compare);
#ifdef _PRINT_ORDER_
    printf("heightOrder2firingOrder: \n");
    for (unsigned laser = 0; laser < VELODYNE_NUM_BEAMS_IN_ONE_SHOT; laser++)
    {
        printf("firingOrder-%d ", heightOrder2firingOrder[laser]);
        printf("%f\n", velodyne_calibrated[heightOrder2firingOrder[laser]][0]);
        if ((laser + 1) % 10 == 0)
        {
            printf("\n");
        }
    }
    printf("\n\n");
    fflush(stdout);
#endif
    for (unsigned heightOrder = 0; heightOrder < VELODYNE_NUM_BEAMS_IN_ONE_SHOT; heightOrder++)
    {
        firingOrder2heightOrder[heightOrder2firingOrder[heightOrder]] = heightOrder;
    }
#ifdef _PRINT_ORDER_
    printf("firingOrder2heightOrder: \n");
    for (unsigned laser = 0; laser < VELODYNE_NUM_BEAMS_IN_ONE_SHOT; laser++)
    {
        printf("heightOrder-%d ", firingOrder2heightOrder[laser]);
        printf("%f\n", velodyne_calibrated[laser][0]);
        if ((laser + 1) % 10 == 0)
        {
            printf("\n");
        }
    }
    printf("\n\n");
    fflush(stdout);
#endif


    // vertCorrection  rotCorrection  distCorrection  vertOffsetCorrection  horizOffsetCorrection
    for (unsigned i = 0; i < VELODYNE_NUM_BEAMS_IN_ONE_SHOT; i++)
    {
        vertCorrection[i] = (velodyne_calibrated[i][0]) * RADIANS_PER_LSB;
        rotCorrection[i] = (velodyne_calibrated[i][1]) * RADIANS_PER_LSB;
        rotOffset[i] = velodyne_calibrated[heightOrder2firingOrder[i]][1] * 6;
        distCorrection[i] = velodyne_calibrated[i][2] * METERS_PER_CM;
        vert_offsetCorrection[i] = velodyne_calibrated[i][3] * METERS_PER_CM;
        horiz_offsetCorrection[i] = velodyne_calibrated[i][4] * METERS_PER_CM;
        sin_vertCorrection[i] = sin(vertCorrection[i]);
        cos_vertCorrection[i] = cos(vertCorrection[i]);
    }

    // SaveRad();

    double ctheta;
    double theta_1;
    double rat;
    /*
     *	�������ǵ�������ֵ
     *	ʹ�ü���ˮƽ���ǽ���������������������������
     *		(�ֲ�����ֱ��ͨ��) rotational_pos[block] - rotCorrection[laser]
     */
    // sintheta[64][36000] = sin(��������λ��)
    for (unsigned laser = 0; laser<VELODYNE_NUM_BEAMS_IN_ONE_SHOT; laser++)
    {
        for (unsigned theta = 0; theta<36000; theta++)
        {
            rat = ((double)theta) / 100.0;
            ctheta = 2 * M_PI - RADIANS_PER_LSB*rat;
            if (ctheta == 2 * M_PI)
                ctheta = 0;
            theta_1 = mod2pi_ref(M_PI, ctheta + rotCorrection[laser]);
            cor_sinAzimuth[laser][theta] = sin(theta_1);						// sin(��������λ��)
        }
    }
    // costheta[64][36000] = cos(��������λ��)
    for (unsigned laser = 0; laser<VELODYNE_NUM_BEAMS_IN_ONE_SHOT; laser++)
    {
        for (unsigned theta = 0; theta<36000; theta++)
        {
            rat = ((double)theta) / 100.0;
            ctheta = 2*M_PI - RADIANS_PER_LSB*rat;
            if (ctheta == 2*M_PI)
                ctheta = 0;
            theta_1 = mod2pi_ref(M_PI, ctheta + rotCorrection[laser]);
            cor_cosAzimuth[laser][theta] = cos(theta_1);						// cos(��������λ��)
        }
    }
    // sinctheta[36000] = sin(ԭʼ��λ��)
    for (unsigned theta = 0; theta<36000; theta++)
    {
        rat = ((double)theta) / 100.0;
        ctheta = 2 * M_PI - RADIANS_PER_LSB*rat;
        if (ctheta == 2 * M_PI)
            ctheta = 0;
        sinAzimuth[theta] = sin(ctheta);										// sin(ԭʼ�ķ�λ��)
    }
    // cosctheta[36000] = sin(ԭʼ��λ��)
    for (unsigned theta = 0; theta<36000; theta++)
    {
        rat = ((double)theta) / 100.0;
        ctheta = 2 * M_PI - RADIANS_PER_LSB*rat;
        if (ctheta == 2 * M_PI)
            ctheta = 0;
        cosAzimuth[theta] = cos(ctheta);										// cos(ԭʼ��λ��)
    }
    return 0;
}

VelodyneDriver::VelodyneDriver()
{
    // Confirm that the UDP message buffer matches the structure size
    //printf("sizeof(vel_laser_t) = %d\n", sizeof(vel_laser_t));
    //printf("sizeof(velodyne_fire_t) = %d\n", sizeof(velodyne_fire_t));
    //printf("sizeof(velodyne_packet_t) = %d\n", sizeof(velodyne_packet_t));
    lastRotation = -1;
    velodyne_calib_precompute();
}

VelodyneDriver::~VelodyneDriver()
{
}
/*
 *	ͨ����ͷ0xEEFF����packet��Ч��
 *	64�߼����״�ͬ�����ǵ�����packet�İ�ͷ�ֱ�Ϊ EEFF �� DDFF
 */
int VelodyneDriver::checkPacket(VelodyneDataRaw::velodyne_packet_t& packet)
{
    // 61183 == 0xEEFF
    if (packet.blocks[0].start_identifier == 0xEEFF)
    {
        return 0;
    }
    else
    {
        printf("check packet error, start identifier: %04X\n", packet.blocks[0].start_identifier);
        return -1;
    }
}
/*
 *	�ж��Ƿ����µ�һ�����ݣ���������һ�ο�ʼ�Ƕ� lastRotation
 *	�����״�ÿɨ����һ�ܣ�packet��Ŀ��������Ҫ�� ����������һ�ܵļ������ݱ������������У����к�����������
 */
bool VelodyneDriver::isNewScan(VelodyneDataRaw::velodyne_packet_t& packet)
{
    // ���ڲ�������
    //#define _TEST_
#ifdef _TEST_
    static int counter = 0;
    counter++;
    printf("PKT#%d\n", counter);
    for (int i = 0; i < VELODYNE_NUM_BLOCKS_IN_ONE_PKT; i++)
    {
        printf("start_identifier: %X\n", packet.blocks[i].start_identifier);
        printf("rotational_pos = %d", packet.blocks[i].rotational_pos);
        if(i==0)
        {
            printf("	lastRotation = %d", lastRotation);
        }
        printf("\n");
    }
    printf("\n\n");
#endif
    // ����packet����Ч��
    if (checkPacket(packet) != 0)
    {
        return false;
    }
    // ʹ�õ�һ��packet����ʼ���ǳ�ʼ��lastRotation����Ϊ�������ݲɼ�����ʼ����
    if (this->lastRotation == -1)
    {
        lastRotation = packet.blocks[0].rotational_pos;
        //printf("lastRotation = %d\n", lastRotation);
        return false;
    }
    // ÿһ֡�ı߽���90�ȿ�ʼ�������ĺ���(������360-90=270)�������ܱ�֤����ǰ������ [Velodyne_16 from Wuhan]
    if (packet.blocks[0].rotational_pos >= 9000 && lastRotation <= 9000){
    //if (packet.blocks[0].rotational_pos >= 18000 && lastRotation <= 18000){
        lastRotation = packet.blocks[0].rotational_pos;
#ifdef _TEST_
        counter = 0;
        printf("A new scan...\n");
#endif

        return true;
    }
    // ������һ�ܼ���������ʼ����
    else
    {
        lastRotation = packet.blocks[0].rotational_pos;
        return false;
    }
}
/*
 *	packet: 1206 byte
 *	ÿ��packet������12��fire��ÿ��fire������32��laser����(32*3(2*distance+1*intensity))
 *		12*(2[start identifier]+2[Rotational]+32*3)
 *		+ 6(status data:4[GPS timestamp]+1[Status Byte]+1[Status Value])
 *	��packet����������(1206 Bytes)������velodyne_data��
 *		velodyne_data��shot���ɣ�ÿ��shot�ɵ㹹��(���մ�ֱ�Ƕȵ�->��)
 */
int VelodyneDriver::recvPacket(VelodyneDataRaw::velodyne_packet_t& packet, VelodyneDataRaw& velodyne_data)
{
    printf("recv packet\n");
    // laser distance filter paras
    int DISTANCE_MAXIMUM = g_CfgVeloView.cfgGlobal.MaxLaserDistacne;
    int DISTANCE_MINIMUM = g_CfgVeloView.cfgGlobal.MinLaserDistance;

    // ÿ��fire����ת��
    int rotational_pos;

    // �����㼫��������
    float distance;
    // У�����ľ���
    float corredistance;

    // �����㷴��ǿ��
    int intensity;

    // �߶�(��->��)���� ����˳��(�Ⱥ�)����
    unsigned short heightNO;
    unsigned short firingNO;

    // ������ --> ֱ������
    float sin_ctheta, cos_ctheta;
    float sin_theta, cos_theta;
    float sin_omiga, cos_omiga;

    VelodyneDataRaw::shot_t shotobj;


    // ÿ��packet����12��blocks
    for (unsigned block = 0; block < VELODYNE_NUM_BLOCKS_IN_ONE_PKT; block++)
    {
        /*
         *	Each data block begins with a two-byte start identifier,
         *		then a two-byte azimuth value (rotational angle),
         *		followed by 32x3-byte data records.
         */
        // Each frame start with 0xEEFF || 0xDDFF(64�ߵĵڶ���block)


        if (packet.blocks[block].start_identifier != UPPER_BANK)
        {
            return -1;
        }
        /*
         * integer between 0 and 35999. Divide this number by 100 to get degrees from 0 to 360
         */
        rotational_pos = packet.blocks[block].rotational_pos;

        for (unsigned laser = 0; laser < VELODYNE_NUM_BEAMS_IN_ONE_BLOCK; laser++)
        {

            // 32�߼����״� һ��block����һ��shot
            firingNO = laser;

            heightNO = velodyne_firingOrder_to_heightOrder(firingNO);
        #if 0
            printf("DISTANCE_MAXIMUM=%d, DISTANCE_MINIMUM=%d\n",
                   DISTANCE_MAXIMUM, DISTANCE_MINIMUM);
        #endif
            /*
             *	1200 * 0.002 = 2.4m(DISTANCE_MINIMUM�����ǳ��뾶��ֵ)
             *	250  * 0.002 = 0.5m
             *	60000 * 0.002 = 120m
             */
            //#define DISTANCE_MINIMUM	250
            if (packet.blocks[block].lasers[laser].distance && packet.blocks[block].lasers[laser].distance<DISTANCE_MAXIMUM
                    && packet.blocks[block].lasers[laser].distance && packet.blocks[block].lasers[laser].distance>DISTANCE_MINIMUM)
            {
                // contains two bytes of distance information in .2 centimeter(0.002m) increments
                distance = absf((packet.blocks[block].lasers[laser].distance) * DISTANCE_RESOLUTION);
                corredistance = distance + distCorrection[firingNO];

                /*
                 *	one byte of intensity information (0 �C 255, with 255 being the most intense return).
                 *	A zero return indicates no return up to 65 meters
                 */
                intensity = packet.blocks[block].lasers[laser].intensity;

                /* START TODO: you need to fix the following code
                 *
                 */
                
                sin_theta = 0;				// ԭʼ��λ��
                cos_theta = 0;
                sin_ctheta = 0;
                cos_ctheta = 0;
                sin_omiga = 0;
                cos_omiga = 0;

                // ������ӳ�䵽ֱ������

                
                shotobj.pt[heightNO].x = 0;
                shotobj.pt[heightNO].y = 0;
                shotobj.pt[heightNO].z = 0;
                shotobj.pt[heightNO].x -= 0;
                shotobj.pt[heightNO].y += 0;


                // m to cm
                shotobj.pt[heightNO].x *= 0;
                shotobj.pt[heightNO].y *= 0;
                shotobj.pt[heightNO].z *= 0;

                shotobj.pt[heightNO].i = 0;
                
                /* END TODO: you need to fix the above code
                 *
                 */


                shotobj.pt[heightNO].point_type = POINT_TYPE_INITIAL;

                // set circle id
                shotobj.pt[heightNO].circleID = laser;
                float pt_x = shotobj.pt[heightNO].x;
                float pt_y = shotobj.pt[heightNO].y;
                shotobj.pt[heightNO].rad = sqrt(pt_x*pt_x + pt_y*pt_y);
                shotobj.pt[heightNO].tan_theta = pt_y / pt_x;
            }
            else
            {
                // like the center of Lidar
                shotobj.pt[heightNO].x = 0;
                shotobj.pt[heightNO].y = 0;
                shotobj.pt[heightNO].z = 0;
                shotobj.pt[heightNO].i = 0;
                shotobj.pt[heightNO].point_type = POINT_TYPE_INITIAL;
                shotobj.pt[heightNO].point_type |= POINT_TYPE_INVALID;
            }


        }


        velodyne_data.shots.push_back(shotobj);
    }
    return 0;
}

/*
 *	���ڵ��Ե���ʾ����
 *		ÿ�� packet 12��fire��ÿ��fire 32��laser(ÿ����fire����һ��64�߼�������)
 */
void VelodyneDriver::printPacket(VelodyneDataRaw::velodyne_packet_t &packet, int seq)
{
    printf("[%d]", seq);

    for (int in = 0; in < VELODYNE_NUM_BLOCKS_IN_ONE_PKT; in++)
    {
        //printf("[%d]Block %d, %d, %d:\n",seq,in, packet.shots[in].lower_upper, packet.shots[in].rotational_pos);
        if (in % 2 == 0)
            printf("%d ", packet.blocks[in].rotational_pos);
#if 0
        printf("Distance: ");
        for (int im = 0; im < 32; im++)
        {
            printf("%d\t", packet.shots[in].lasers[im].distance);
        }
        printf("\nIntensity: ");
        for (int im = 0; im < 32; im++)
        {
            printf("%d\t", packet.shots[in].lasers[im].intensity);
        }
#endif
    }
    printf("\n");
    //printf("Status: ");
    //std::cout<< (double)packet.degrees + (double)packet.frac_degrees / 256.0<<endl ;
    //printf("%02x %02x %02x %02x\n",packet.status_string[0], packet.status_string[1], packet.status_string[2], packet.status_string[3]);
    //printf("\n\n\n\n");
}

