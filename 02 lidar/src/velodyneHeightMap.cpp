/**************************************************************************
 *
 * transplant algorithm from ROS implemention velodyne_height_map(http://wiki.ros.org/velodyne_height_map)
 *	1. use fixed-size grid matrix, grid_dim * grid_dim
 *	2. use height_threashold(max_z - min_z) to decide whether a grid is an obstacle
 *	3. show points if it's in the obstacle grid
 *
 ***************************************************************************/

#include "velodyneHeightMap.h"
#include "velodyneDraw.h"
#include <string.h>
#include <stdio.h>
namespace velodyne_height_map
{

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))

HeightMap::HeightMap()
{
    this->cell_size = 50;
    this->grid_dim = 240;
    this->height_diff_threshold = 25;
    printf("HeightMap Constructed...\n");
}
HeightMap::~HeightMap() {}

void HeightMap::constructGridClouds(const VelodyneDataStruct& scanobj, size_t& obs_count, size_t& empty_count)
{
    //printf("constructGridClouds\n");
    /*
     *  added by druant35
     *	m_per_cell_: cell_size
     *	grid_dim_: a map of grid_dim_ x grid_dim_
     *	height_threshold: height_diff_threshold_, max_z - min_z
     */
    // do not use dynamic memory --> waste memery for speed
    static float min[MAX_BUFFER_SIZE][MAX_BUFFER_SIZE];
    static float max[MAX_BUFFER_SIZE][MAX_BUFFER_SIZE];
    static bool init[MAX_BUFFER_SIZE][MAX_BUFFER_SIZE];
    memset(&init, 0, grid_dim * grid_dim);

    unsigned shotNum = scanobj.shots.size();
#if 0
    printf("shotNum = %d\n", shotNum);
#endif
    /*
     *	build height map
     *	calculate min_z and max_z for each grid
     */
    for (unsigned shot = 0; shot < shotNum; shot++)
    {
        for (unsigned laser = 0; laser < VELODYNE_NUM_BEAMS_IN_ONE_SHOT; laser++)
        {
            if (scanobj.shots[shot].pt[laser].point_type & POINT_TYPE_INVALID)
                continue;

            int grid_index_x = ((grid_dim / 2) + scanobj.shots[shot].pt[laser].x / cell_size);
            int grid_index_y = ((grid_dim / 2) + scanobj.shots[shot].pt[laser].y / cell_size);

            if (grid_index_x >= 0 && grid_index_x < grid_dim && grid_index_y >= 0 && grid_index_y < grid_dim)
            {
                if (!init[grid_index_x][grid_index_y])
                {
                    min[grid_index_x][grid_index_y]  = scanobj.shots[shot].pt[laser].z;
                    max[grid_index_x][grid_index_y]  = scanobj.shots[shot].pt[laser].z;
                    init[grid_index_x][grid_index_y] = true;
                }
                else
                {
                    min[grid_index_x][grid_index_y] = MIN(min[grid_index_x][grid_index_y], scanobj.shots[shot].pt[laser].z);
                    max[grid_index_x][grid_index_y] = MAX(max[grid_index_x][grid_index_y], scanobj.shots[shot].pt[laser].z);
                }
            }
        }
    } // end of for (unsigned shot = 0; shot < shotNum; shot++){

    // display points where map has height-difference > threshold
    for (unsigned shot = 0; shot < shotNum; shot++)
    {
        for (unsigned laser = 0; laser < VELODYNE_NUM_BEAMS_IN_ONE_SHOT; laser++)
        {
            if (scanobj.shots[shot].pt[laser].point_type & POINT_TYPE_INVALID)
                continue;

            int grid_index_x = ((grid_dim / 2) + scanobj.shots[shot].pt[laser].x / cell_size);
            int grid_index_y = ((grid_dim / 2) + scanobj.shots[shot].pt[laser].y / cell_size);

            if (grid_index_x >= 0 && grid_index_x < grid_dim && grid_index_y >= 0 && grid_index_y < grid_dim && init[grid_index_x][grid_index_y])
            {
                if ((max[grid_index_x][grid_index_y] - min[grid_index_x][grid_index_y]) > height_diff_threshold)
                {
                    VelodyneDataStruct::xpoint_t obstacle_pt;

                    obstacle_pt.x = scanobj.shots[shot].pt[laser].x;
                    obstacle_pt.y = scanobj.shots[shot].pt[laser].y;
                    obstacle_pt.z = scanobj.shots[shot].pt[laser].z;
                    obstacle_pt.i = scanobj.shots[shot].pt[laser].i;

                    obstacle_cloud.push_back(obstacle_pt);
                    obs_count++;
                }
                else
                {
                    VelodyneDataStruct::xpoint_t clear_pt;

                    clear_pt.x = scanobj.shots[shot].pt[laser].x;
                    clear_pt.y = scanobj.shots[shot].pt[laser].y;
                    clear_pt.z = scanobj.shots[shot].pt[laser].z;
                    clear_pt.i = scanobj.shots[shot].pt[laser].i;

                    clear_cloud.push_back(clear_pt);
                    empty_count++;
                }
            } // end of if (x >= 0 && x < grid_dim && y >= 0 && y < grid_dim && init[x][y]){
        } // end of for (unsigned laser = 0; laser < VELODYNE_NUM_BEAMS_IN_ONE_SHOT; laser++){
    } // end of for (unsigned shot = 0; shot < shotNum; shot++){
} // end of void HeightMap::constructGridClouds...

void HeightMap::processData(VelodyneDataStruct& scanobj)
{
    size_t obs_count = 0;
    size_t empty_count = 0;

    constructGridClouds(scanobj, obs_count, empty_count);
#if 0
    printf("start to draw point...Number: %d\n", obs_count);
#endif
    for (unsigned pt = 0; pt < obs_count; pt++)
    {
        //printf("pt#%d: (%8.2f, %8.2f, %8.2f)\n", obs_count, obstacle_cloud[pt].x, obstacle_cloud[pt].y, obstacle_cloud[pt].z);
        drawPointRGB(obstacle_cloud[pt], 1., 0., 0.);
    }
} // end of void HeightMap::processData(VelodyneDataStruct& scanobj)
}
