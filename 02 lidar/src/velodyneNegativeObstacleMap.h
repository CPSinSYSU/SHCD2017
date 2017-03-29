#ifndef __VELODYNE_NEGATIVE_OBSTACLE_MAP__
#define __VELODYNE_NEGATIVE_OBSTACLE_MAP__

/**************************************************************************
*
* 1. 使用固定大小的grid matrix，大小是grid_dim * grid_dim
* // TODO : add some imformation
* author: sean
* data:2016/8/10
*
***************************************************************************/

#include "velodyneDataStruct.h"
#include <vector>
#include <stdlib.h>     // for size_t

namespace velodyne_negative_obstacle_map
{
// 用于初始化建立negative obstacle map的buffer的大小
#define MAX_BUFFER_SIZE 1024

typedef VelodyneDataStruct::xpoint_t NPoint;
typedef std::vector<NPoint> NPointCloud;

class NegativeObstacleMap
{
public:
    NegativeObstacleMap();
    ~NegativeObstacleMap();

    // 构建Negative Obstacle Map
    void constructNegativeObstacleCloud(VelodyneDataStruct& scanobj, bool flag);
    void constructNegativeObstacleCloud(VelodyneDataStruct& scanobj);

    NPointCloud negative_obstacle_cloud;
    NPointCloud clear_cloud;

    std::size_t obs_count;
    std::size_t empty_count;

private:
    int grid_dim;
    int cell_size;
    // 用来判断“坑”靠近车体的那个边缘
    double edge_start_threshold;
    // 用来判断“坑”远离车体的另一个边缘
    double edge_end_threshold;
    // 用于判断该网格是否为坑的边缘个数
    int edge_number_threshold;
};

}

#endif
