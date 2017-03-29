#pragma once

#define STEERING_R    15.0
#define WHEEL_BASE     2.7
#define CAR_LENGTH    4.68
#define CAR_WIDTH    1.720
#define CAR_HIGH     1.515
#define CAR_WEIGHT    1245  //kg
#define CAR_MIN_H    0.170
#define SAFE_WIDTH   1.0   //安全保护的距离
#define SAFE_LENGTH  1.0   //安全保护的距离

//////////////////Velodyne ///////////////////
#define RADIANS_PER_LSB 0.0174532925
#define METERS_PER_LSB 0.002       //
#define METERS_PER_CM 0.01

//////////////////坐标转换////////////////////
#define PI 3.14159265358979323846264338
#define  _ee_   0.0066943799013
#define  _ee2_  0.00673949674227
#define  _a_    6378137.0000000000

#define LANE_WIDTH  2.9
#define PATCHS_WIDTH  20
#define PATCHS_LENGTH  20
#define PATCHS_WIDTH_PER_M  0.6

#define velodyne_left_side   0.41
#define velodyne_right_side  0.35
#define velodyne_high_side   1.71
#define velodyne_back_side   2.20
#define velodyne_pitch_d     6.73
#define velodyne_pitch_1      5.6
#define velodyne_pitch_11    7.86

#define ins_high_side  0.31
#define ins_back_side  0.94
#define ins_left_side  0.09

#define MOTION_PLANNING_LENGTH 30
#define AFTER_MOTION_PLANNING_LENGTH 500
