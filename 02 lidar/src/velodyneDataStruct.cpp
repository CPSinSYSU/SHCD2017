/* *************************************************************************
 *
 * scan data initializes, from raw data or .pcd data
 *
 * statistics function, including init/calc/dump(keep in file)
 *
 ***************************************************************************/

#include "velodyneDataStruct.h"
#include <assert.h>
#include <fstream>
// need to define _USE_MATH_DEFINES for <math.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>
#include <string.h>

static inline float absf(float a)
{
    if (a < 0)
        return -a;
    return a;
}

VelodyneDataStruct::VelodyneDataStruct()
    : sorted(false)
{
    assert(sizeof(velodyne_packet_t) == 1206);

    circleGridWidth = 4;
    calcCircleFeatured = false;
    calcCirclesLined = false;
    calcRoadCurb = false;
}

VelodyneDataStruct::~VelodyneDataStruct()
{
}

// initialize statistics to 0
void VelodyneDataStruct::InitStatistics(pointStatistics_t& pStatistics)
{
    memset(&pStatistics, 0, sizeof(pStatistics));
}

/*
 * cpp static member variable initialization
 *		magic for raw data!!!
 *		"rw3" == 0x72773300
 */
const char VelodyneDataRaw::magic[4] = "rw3";

VelodyneDataRaw::VelodyneDataRaw()
{
    assert(sizeof(velodyne_packet_t) == 1206);
}

VelodyneDataRaw::~VelodyneDataRaw()
{
}

/*
 *	added by durant35
 *		filename ָ����.raw��ʽ�ļ�ת���� shots ����
 */
int VelodyneDataRaw::deserialize(const char* filename)
{
    char buffer[sizeof(VelodyneDataRaw::magic)];
    int* p = reinterpret_cast<int*> (buffer);
    int len, i;
    shot_t shotobj;
    std::ifstream file;
    file.open(filename, std::ios::in | std::ios::binary);
    if (file.is_open() == false)
    {
        printf("%s does not exist!!\n", filename);
        return -1;
    }

    // read magic bits, should be the same as scan::magic
    file.read(buffer, sizeof(VelodyneDataRaw::magic));
    if (memcmp(buffer, VelodyneDataRaw::magic, sizeof(VelodyneDataRaw::magic)) != 0)
    {
        printf("Not an raw scan file!!\n");
        file.close();
        return -1;
    }
    // magic �����ĸ��ֽ��� size
    file.read(buffer, sizeof(VelodyneDataRaw::magic));

    len = *p;
    shots.clear();
    /*
     *		���ݰ�֡���桢��֡��ȡ
     *		һ֡����64�� xpoint_t
     */
    for (i = 0; i < len; ++i)
    {
        file.read(reinterpret_cast<char*> (&shotobj), sizeof(shot_t));
        //printf("read[%d]  offset[%u] number[%d]\n",i,ii.offset,ii.scanNumber);
        shots.push_back(shotobj);
    }
    file.close();

    printf("deserialize %d done\n", len);

    return 0;
}

/*
 *		saveToPCD == true���� shots ���ݱ����.pcd��ʽ
 *		saveToPCD == false���� shots ���ݱ����.raw��ʽ
 */
int VelodyneDataRaw::serialize(const char* filename, bool saveToPCD)
{
    int len, i;
    // �����raw��ʽ
    if (saveToPCD == false)
    {
        std::ofstream file;
        file.open(filename, std::ios::out | std::ios::binary);
        // write magic bits
        file.write(VelodyneDataRaw::magic, 4);
        // write size
        len = shots.size();
        file.write(reinterpret_cast<char*>(&len), 4);
        for (i = 0; i < len; ++i)
        {
            file.write(reinterpret_cast<char*> (&shots[i]), sizeof(shot_t));
            //printf("write[%d]  offset[%u] number[%d]\n",i,m_index[i].offset,m_index[i].scanNumber);
        }
        file.close();
    }

    //// �����pcd��ʽ, use pcl API
    //else{
    //	pcl::PointCloud<pcl::PointXYZ> cloud;
    //	// Fill in the cloud data
    //	cloud.width = shots.size() * 64;
    //	cloud.height = 1;
    //	cloud.is_dense = false;
    //	cloud.points.resize(cloud.width * cloud.height);
    //	size_t shotID, pointID;
    //	for (size_t i = 0; i < cloud.points.size(); ++i){
    //		shotID = i / 64;
    //		pointID = i % 64;
    //		cloud.points[i].x = shots[shotID].pt[pointID].x;
    //		cloud.points[i].y = shots[shotID].pt[pointID].y;
    //		cloud.points[i].z = shots[shotID].pt[pointID].z;
    //	}
    //	//pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
    //	pcl::io::savePCDFileBinary(filename, cloud);
    //}

    return 0;
}



