#ifndef __VELODYNE_HEIGHT_MAP_H__
#define __VELODYNE_HEIGHT_MAP_H__

/**************************************************************************
 *
 * transplant algorithm from ROS implemention velodyne_height_map(http://wiki.ros.org/velodyne_height_map)
 *	1. use fixed-size grid matrix, grid_dim * grid_dim
 *	2. use height_threashold(max_z - min_z) to decide whether a grid is an obstacle
 *	3. show points if it's in the obstacle grid
 *
 ***************************************************************************/

#include "velodyneDataStruct.h"
#include <vector>
#include <stddef.h>

namespace velodyne_height_map
{
#define MAX_BUFFER_SIZE 1024

typedef VelodyneDataStruct::xpoint_t VPoint;
typedef std::vector<VPoint> VPointCloud;

class HeightMap
{
public:
    HeightMap();
    ~HeightMap();
    void processData(VelodyneDataStruct& scanobj);
private:
    VPointCloud obstacle_cloud;
    VPointCloud clear_cloud;

    int grid_dim;
    double cell_size;
    double height_diff_threshold;

    //TODO
    void constructGridClouds(const VelodyneDataStruct& scanobj, size_t& obs_count, size_t& empty_count);
};
}

#endif
