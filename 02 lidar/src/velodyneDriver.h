#ifndef __VELODYNE_DRIVER_H__
#define __VELODYNE_DRIVER_H__

/**************************************************************************
 *
 * Velodyne Ladar Header File
 *
 * This is the header for the velodyne ladar interface drivers.
 *
 ***************************************************************************/

#include "velodyneDataStruct.h"

class VelodyneDriver
{
public:
    VelodyneDriver();
    ~VelodyneDriver();

    int checkPacket(VelodyneDataRaw::velodyne_packet_t& packet_reference);
    bool isNewScan(VelodyneDataRaw::velodyne_packet_t& packet_reference);
    int recvPacket(VelodyneDataRaw::velodyne_packet_t& packet_reference, VelodyneDataRaw& velodyne_data);
    // Print the packet to the screen
    void printPacket(VelodyneDataRaw::velodyne_packet_t& packet_reference, int seq = 0);
private:
    // 初始为-1
    int lastRotation;
};

#endif

