/**************************************************************************
 *
 * Velodyne Process Threads Header File
 *	Socket Thread
 *	LCM Thread
 *	Timer Processing Function
 *
 ***************************************************************************/
#ifdef WIN32
    #define _CRT_SECURE_NO_WARNINGS
    #define socklen_t int
#else
    #define _snprintf snprintf
    #define INVALID_SOCKET (-1)
#endif // WIN32
#include "velodyneThread.h"

#include <pthread.h>
#ifdef WIN32
    #pragma comment(lib,"x86/pthreadVC2.lib")
#endif // WIN32

#include "velodyneDataStruct.h"
#include "velodyneDriver.h"
#ifdef WIN32
    #include <gl/glut.h>
#else
    #include <GL/glut.h>
#endif // WIN32

#ifdef WIN32
    #include <windows.h>
    #include <winsock.h>
    #pragma comment(lib, "Wsock32.lib")
#else
    #define closesocket close
    #include <unistd.h>
    #include <netdb.h>
    #include <sys/types.h>
    #include <netinet/in.h>
    #include <sys/socket.h>
    #include <arpa/inet.h>
#endif

#include <lcm/lcm-cpp.hpp>
#ifdef WIN32
    #pragma comment(lib, "lcm.lib")
#endif // WIN32
//// previous obstacle lcm interface
//#include "obu_lcm\obstacle_list.hpp"
#include "nad_lcm/obstacle_info.hpp"

#include "velodyneAlgo.h"
#include "velodyneScanner.h"
#include <sys/time.h>       // for gettimeofday, clock_t
#include <fstream>          // for ofstrem

#include <math.h>           // for M_PI

#include "velodyneDraw.h"   // for OpenGLThread

// project Lidar cordinate to GPS cordinate, cm as unit
#define LIDAR2GPS 35

// pthread id
pthread_t g_socket_th;
pthread_t g_lcm_th;
pthread_t g_opengl_th;

pthread_spinlock_t g_scanBuffer_lock;
pthread_spinlock_t g_glut_lock;

VelodyneDataRaw g_scanBuffer[SCAN_BUFFER_SIZE];
int g_scanBufferSize = 0;
int g_scanBufferReadIdx = 0;
int g_scanBufferWriteIdx = 0;
int g_scanDrop = 0;

/*
 * 构造以时间作为前缀的.pcap文件名
 */
#ifdef WIN32
// self-achieved gettimeofday
static int gettimeofday(struct timeval* tv)
{
    union
    {
        long long ns100;
        FILETIME ft;
    } now;

    GetSystemTimeAsFileTime(&now.ft);
    tv->tv_usec = (long)((now.ns100 / 10LL) % 1000000LL);
    tv->tv_sec = (long)((now.ns100 - 116444736000000000LL) / 10000000LL);
    return 0;
}
#endif

static int getCurrentTimeStr(char* timeStr, int len){
    struct timeval tp;
    gettimeofday(&tp, NULL);

    struct tm *ltime;
    char timeStr_tmp[32];
    time_t local_tv_sec;

    // convert the timestamp to readable format
    local_tv_sec = tp.tv_sec;
    ltime = localtime(&local_tv_sec);

    strftime(timeStr_tmp, sizeof(timeStr_tmp), "%Y%m%d%H%M%S", ltime);
    _snprintf(timeStr, len, "%s%ld", timeStr_tmp, tp.tv_usec);
    return 0;
}

static int getFileName(char* filename, int len)
{
    struct timeval tp;
#ifdef WIN32
    gettimeofday(&tp);
#else
    gettimeofday(&tp, NULL);
#endif // WIN32
    struct tm *ltime;
    char timestr[32];

    time_t local_tv_sec;

    // convert the timestamp to readable format
    local_tv_sec = tp.tv_sec;
    ltime = localtime(&local_tv_sec);

    strftime(timestr, sizeof(timestr), "%Y%m%d%H%M%S", ltime);
    _snprintf(filename, len, "./gridmap/%s.gridmap", timestr);
    return 0;
}

// 缓冲区已满，直接丢弃当前帧
static void ClearBuffer()
{
    g_scanBuffer[g_scanBufferWriteIdx].shots.clear();
    //g_scanBuffer[g_scanBufferWriteIdx].scanPieArray.clear();
    //g_scanBuffer[g_scanBufferWriteIdx].scanPieFeatureArray.clear();
    //g_scanBuffer[g_scanBufferWriteIdx].scanPieClusterSet.clear();
    //g_scanBuffer[g_scanBufferWriteIdx].scanPieClusterFeatureSet.clear();
    //g_scanBuffer[g_scanBufferWriteIdx].calcCircleFeatured = false;
    //g_scanBuffer[g_scanBufferWriteIdx].calcCirclesLined = false;
    //g_scanBuffer[g_scanBufferWriteIdx].calcRoadCurb = false;
    //g_scanBuffer[g_scanBufferWriteIdx].scanGridArray.clear();
    //g_scanBuffer[g_scanBufferWriteIdx].scanGridClusterSet.clear();
    //g_scanBuffer[g_scanBufferWriteIdx].scanGridFeatureArray.clear();
    //g_scanBuffer[g_scanBufferWriteIdx].scanGridClusterFeatureSet.clear();
}
/*
 * 使用socket通过 UDP 获取激光雷达数据 scan g_scanBuffer[8];(每8个scan(一个scan大概360个packet)更新一次)
 * 解析 UDP 数据包中的数据，保存在 g_scanBuffer 中
 */
//#define CONFIG_CLIENT
void* SocketThread(void* arg)
{
    VelodyneDataRaw::velodyne_packet_t pkt;
    VelodyneDriver velodyneDriver;
#ifdef WIN32
    // 版本号: WSAStartup(MAKEWORD(2, 2), &WSADATA)
    WSADATA wsa;
    if (WSAStartup(0x0101, &wsa) != 0)
    {
        goto socket_out;
    }
    SOCKET sockfd;
#else
    int sockfd;
#endif // WIN32

    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd == INVALID_SOCKET)
    {
        printf("fail to create socket(socket thread dumped)\n");
        return NULL;
    }
    // 512k
    int newBufferSize = 512 * 1024;
    setsockopt(sockfd,
               SOL_SOCKET,                              // level = SOL_SOCKET/IPPROTO_TCP/...
               SO_RCVBUF,
               (char *)&newBufferSize, sizeof(newBufferSize));

    struct sockaddr_in server;							// Information about the server
    struct sockaddr_in client;							// Information about the client

    int pkt_count = 0;
    // Clear out server struct
    memset((void *)&server, '\0', sizeof(struct sockaddr_in));
    // Set family and port
    server.sin_family = AF_INET;
    server.sin_port = htons(VELODYNE_DATA_PORT);
    // Set server address
#ifdef WIN32
    server.sin_addr.S_un.S_addr = INADDR_ANY;
#else
    server.sin_addr.s_addr = htonl(INADDR_ANY);
#endif // WIN32

#ifdef CONFIG_CLIENT
    // Clear out client struct
    memset((void *)&client, '\0', sizeof(struct sockaddr_in));
    // Set family and port
    client.sin_family = AF_INET;
    client.sin_port = htons(VELODYNE_DATA_PORT);
    // Set client address
    #ifdef WIN32
    client.sin_addr.S_un.S_addr = inet_addr("");
    #else
    //client.sin_addr.s_addr = inet_addr("192.168.0.101");
    #endif // WIN32
#endif // CONFIG_CLIENT

    socklen_t client_length = sizeof(struct sockaddr_in);					// Length of client address(need to be initialized)
    int bytes_received;
    int pktNumInCircle = 0;

    // Bind address to socket
    if (bind(sockfd, (struct sockaddr *)&server, sizeof(struct sockaddr_in)) == -1)
    {
        printf("Could not bind name to socket(socket thread dumped)\n");
        goto socket_out;
    }

    char* server_addr;
    //server_addr = inet_ntoa(server.sin_addr);
    //printf("server_addr = %s\n", server_addr);

    printf("SocketThread started...\n");
    //while (g_ThreadExitFlag == 0){
    while (true)
    {
        //#define GET_TIME_SOCKET
        #ifdef GET_TIME_SOCKET
            clock_t begin_clock, end_clock;
            double time_spent;
            begin_clock = clock();
        #endif
        bytes_received = recvfrom(sockfd,
                                  (char*)&pkt, sizeof(pkt),
                                  0,
                                  (struct sockaddr *)&client, &client_length);
        if (bytes_received != VELODYNE_DATA_SIZE)
        {
            printf("Socket set up problem(socket thread dumped)");
            goto socket_out;
        }
        pkt_count++;

        //pthread_spin_lock(&g_scanBuffer_lock);

        if (velodyneDriver.isNewScan(pkt))
        {
            // 每一个 scan 的packet必须要360(32线为其一半)个以上，否则认为丢包太多而丢弃
            if (pktNumInCircle > PKT_NUM_IN_CIRCLE_LOWER_BOUND)
            {
                if (g_scanBufferSize != SCAN_BUFFER_SIZE - 1)
                {
                    // 缓冲区写位置 +1
                    g_scanBufferWriteIdx = (g_scanBufferWriteIdx + 1) % SCAN_BUFFER_SIZE;
                    g_scanBufferSize++;

                    // 清空写位置缓冲区为下一帧做准备
                    ClearBuffer();

                    //#define _TEST_
                #ifdef _TEST_
                    printf("g_scanBufferSize[%d] g_scanBufferWriteIdx[%d] g_scanBufferReadIdx[%d]\n", g_scanBufferSize, g_scanBufferWriteIdx, g_scanBufferReadIdx);
                #endif
                }
                // 达到SCAN_BUFFER_SIZE 直接扔掉
                else
                {
                    g_scanBufferReadIdx = (g_scanBufferReadIdx + 2) % SCAN_BUFFER_SIZE;
                    g_scanBufferWriteIdx = (g_scanBufferWriteIdx + 1) % SCAN_BUFFER_SIZE;
                    g_scanBufferSize--;
                    ClearBuffer();
                    //printf("g_scanBufferSize fully updated\n");
                }
            }
            else
            {
                ClearBuffer();
                //printf("ignore this scan %d due to pktNumInCircle less than bound\n", pktNumInCircle);
            }

            pktNumInCircle = 0;
            printf("g_scanBufferWrited...\n");

        } // end of if (velodyneDriver.isNewScan(pkt))

        // 找到下一个scan的开始，把这一个scan记录下来
        if (!velodyneDriver.recvPacket(pkt, g_scanBuffer[g_scanBufferWriteIdx]))
        {
            // 记录旋转一周大概的packet数目
            pktNumInCircle++;
        }

        //pthread_spin_unlock(&g_scanBuffer_lock);
        /**
         * glutPostRedisplay does not refresh anything.
         * It just sets a flag and that's it.
         * The flag set by glutPostRedisplay is tested for in the main loop,
         *  and if there are not further events to be processed and the flag is set,
         *  the main loop calls the display function.
         */
        //glutPostRedisplay();

        #ifndef OFF_LINE
        //	pthread_spin_lock(&g_glut_lock);
            #ifndef NO_DRAW
            #endif // NO_DRAW
        #endif
        //pthread_spin_unlock(&g_scanBuffer_lock);

        #ifdef GET_TIME_SOCKET
            end_clock = clock();
            // CLOCKS_PER_SEC is a constant in time.h header file.
            time_spent = (double)( end_clock - begin_clock) / CLOCKS_PER_SEC;
            printf("Socket time consumed: %.6lf[s]\n", time_spent);
        #endif
    } // while (true)
socket_out:
    closesocket(sockfd);
#ifdef WIN32
    WSACleanup();
#endif // WIN32

    printf("SocketThread stoped...\n");
    return NULL;
}

/* Grid map file saving
#define SAVE_GRID_MAP
#define SAVE_GRID_MAP_MAX_BUFFER_SIZE 480
*/
//#define USE_GRID_ALG
void* LCMThread(void* lparam)
{
    printf("LCMThread Started...\n");

    lcm::LCM lcm_velodyne_lidar("udpm://239.255.76.63:7667?ttl=3");

    if (!lcm_velodyne_lidar.good())
    {
        printf("fail to create lcm(LCM thread dumped)\n");
        return NULL;
    }

#ifdef SAVE_GRID_MAP
    std::ofstream gridmap_file;
    char filename[64];
    int frame_count = 0;
    static int gridmap[SAVE_GRID_MAP_MAX_BUFFER_SIZE][SAVE_GRID_MAP_MAX_BUFFER_SIZE];
    memset(gridmap, 0, sizeof(gridmap));

    getFileName(filename, sizeof(filename));
    gridmap_file.open(filename);
    if (gridmap_file == NULL || !gridmap_file.is_open())
    {
        printf("fail to create grid map file(LCM thread dumped)\n");
        goto lcm_thread_end;
    }
#endif // SAVE_GRID_MAP
    #define GET_TIME_LCM
    while (true)
    {
        #ifdef GET_TIME_LCM
            clock_t begin_clock, end_clock;
            double time_spent;
            begin_clock = clock();
        #endif

        nad_lcm::obstacle_info velodyne_lidar_data;
        // 障碍物网格数目
        velodyne_lidar_data.num_of_obstacle = 0;
        // 障碍物网格列表
        velodyne_lidar_data.obu_obstacle.clear();

    #ifdef USE_GRID_ALG
        int RowNumber = g_CfgVeloView.cfgPlaneDetect.GridHeadRowNum + g_CfgVeloView.cfgPlaneDetect.GridBackRowNum;
        int ColumnNumber = g_CfgVeloView.cfgPlaneDetect.GridColumnNum; 					// 每列有?个方格
        int GridCellSize = g_CfgVeloView.cfgPlaneDetect.GridCellSize;					// 每个方格为边长为?cm的正方形
        int Min_Y = -1 * (g_CfgVeloView.cfgPlaneDetect.GridBackRowNum * GridCellSize);
        int Max_X = ColumnNumber * GridCellSize / 2;
    #else
        int AzimuthNumber = g_CfgVeloView.cfgPlaneDetect.PieAzimuthNum;
        // 格网大小 50cm
        int PieRadSize = g_CfgVeloView.cfgPlaneDetect.PieRadSize;
        // 0-6000cm 半径60米范围内
        int MinRad = g_CfgVeloView.cfgPlaneDetect.PieMinRad;
        int MaxRad = g_CfgVeloView.cfgPlaneDetect.PieMaxRad;

        float azimuth_delta = (2 * M_PI) / AzimuthNumber;
        float radius_delta = (MaxRad - MinRad) / PieRadSize;
    #endif // USE_GRID_ALG

        // 获取点云数据 保护锁
        //pthread_spin_lock(&g_scanBuffer_lock);

        VelodyneDataStruct* pscanobj = getScanRaw();
        if (pscanobj)
        {
        #ifdef USE_GRID_ALG // Grid cell processing...
            if (pscanobj->scanGridArray.size() == 0)
            {
                printf("TransferToGridMatrix starting\n");
                VelodyneAlgo::TransferToGridMatrix(g_CfgVeloView, *pscanobj);
                printf("TransferToGridMatrix done\n");
                VelodyneAlgo::CalcGridMatrixFeature(g_CfgVeloView, *pscanobj);
                printf("CalcGridMatrixFeature done\n");
                VelodyneAlgo::FindAndCalcGridClusterFeature(g_CfgVeloView, *pscanobj);
                printf("FindAndCalcGridClusterFeature done\n");
            }
        #else               // Pie cell processing
            if (pscanobj->scanPieArray.size() == 0)
            {
                // 放进网格
                VelodyneAlgo::TransferToPieMatrix(g_CfgVeloView, *pscanobj);
                //printf("TransferToPieMatrix done\n");
                // 计算网格特征
                VelodyneAlgo::CalcPieMatrixFeature(g_CfgVeloView, *pscanobj);
                //printf("CalcPieMatrixFeature done\n");
                // 计算集群特征
                //VelodyneAlgo::FindAndCalcPieClusterFeature(g_CfgVeloView, *pscanobj, false);
                //printf("FindAndCalcPieClusterFeature done\n");
            }
        #endif // USE_GRID_ALG

            end_clock = clock();
            // CLOCKS_PER_SEC is a constant in time.h header file.
            time_spent = (double)( end_clock - begin_clock)/CLOCKS_PER_SEC;
            printf("Obstacle processed time consumed: %.6lf[s]\n", time_spent);

            std::vector<nad_lcm::nad_obstacle> obs_list;

        #ifdef USE_GRID_ALG
            // 方格矩阵，外循环是在行上(Y轴)，内循环是列上(X轴)划分
            for (int row = 0; row < pscanobj->scanGridArray.size(); row++)
            {
                // 每行方格
                VelodyneDataStruct::gridRow_t& rowObj = pscanobj->scanGridArray[row];
                for (int grid = 0; grid < rowObj.size(); grid++)
                {
                    // 获取每个方格特征信息
                    VelodyneDataStruct::gridFeature_t& gridobj = pscanobj->scanGridFeatureArray[row][grid];

                    if (gridobj.cellType & CELL_TYPE_ABOVE_DELTA_Z)
                    {
                    #ifdef SAVE_GRID_MAP
                        gridmap[row][grid] = 1;
                    #endif
                        // 构建障碍物信息数据
                        nad_lcm::nad_obstacle* obs = new nad_lcm::nad_obstacle();
                        //memset(&obs, 0, sizeof(obs));
                        obs->source = "velodyne-lidar";

                        //TODO: project cell to points
                        // 障碍物网格2D坐标
                        obs->x = Max_X - grid * GridCellSize;
                        if (row > g_CfgVeloView.cfgPlaneDetect.GridBackRowNum)
                        {
                            obs->y = 1.0 * ((row - g_CfgVeloView.cfgPlaneDetect.GridBackRowNum) * GridCellSize);
                        }
                        else
                        {
                            obs->y = 1.0 * (row * GridCellSize + Min_Y);
                        }
                        // 障碍物网格高度
                        obs->height = gridobj.max_z - gridobj.min_z;
                        // 障碍物网格宽度
                        obs->width = 1.0 * g_CfgVeloView.cfgPlaneDetect.GridCellSize;
                        // 经纬度
                        obs->lat = 0;
                        obs->lon = 0;
                        // GPS时间戳
                        obs->gps_time = 0;
                        obs->speed = 0;
                        obs->type = 0;
                        // 航向角
                        obs->yaw = 0;

                        obs_list.push_back(*obs);
                        velodyne_lidar_data.num_of_obstacle++;
                    } // end of if (gridobj.cellType & CELL_TYPE_ABOVE_DELTA_Z){
                    else
                    {
                    #ifdef SAVE_GRID_MAP
                        gridmap[row][grid] = 0;
                    #endif
                    }
                } // end of for (unsigned grid = 0; grid < rowObj.size(); grid++){
            } // end of for (unsigned row = 0; row < pscanobj->scanGridArray.size(); row++){
        #else
            #if 1
            //printf("pscanobj->scanPieArray.size()=%d\n", pscanobj->scanPieArray.size());
            for (unsigned azimuth = 0; azimuth < pscanobj->scanPieArray.size(); azimuth++)
            {
                //TODO: project cell to points
                // 1. Get Azimuth[0~2*PI]
                /**
                 * not send rect of obstacle to enhance lcm publish.
                 */
                //#define SEND_RECT
            #ifdef SEND_RECT
                float current_azimuth = azimuth * azimuth_delta;
                float next_azimuth = ((azimuth + 1) % AzimuthNumber) * azimuth_delta;
            #else
                float current_azimuth = azimuth * azimuth_delta + azimuth_delta / 2.0;
            #endif // SEND_RECT

                VelodyneDataStruct::pieAzimuth_t& azimuthObj = pscanobj->scanPieArray[azimuth];
                for (unsigned rad = 0; rad < azimuthObj.size(); rad++)
                {
                    // 2. Get Radius[PieMinRad~PieMaxRad]
                #ifdef SEND_RECT
                    float current_radius = MinRad + rad * PieRadSize;
                    int next_radius = MinRad + (rad + 1) * PieRadSize;
                #else
                    float current_radius = MinRad + rad * PieRadSize + PieRadSize / 2.0;
                #endif // SEND_RECT

                    VelodyneDataStruct::pieFeature_t& pieobj = pscanobj->scanPieFeatureArray[azimuth][rad];

                    if (pieobj.cellType & CELL_TYPE_ABOVE_DELTA_Z)
                    {
                        // 除去中空网格
                        if (!(pieobj.cellType & CELL_TYPE_NOT_HOLLOW))
                        {
                            continue;
                        }

                        float cos_cur_azi = cos(current_azimuth);
                        float sin_cur_azi = sin(current_azimuth);
                    #ifdef SEND_RECT
                        float cos_nxt_azi = cos(next_azimuth);
                        float sin_nxt_azi = sin(next_azimuth);
                    #endif // SEND_RECT

                        // 构建障碍物信息数据
                        nad_lcm::nad_obstacle* obs = new nad_lcm::nad_obstacle();
                        obs->source = "velodyne-lidar";

                        // 经纬度
                        obs->lat = 0.;
                        obs->lon = 0.;
                        // GPS时间戳
                        obs->gps_time = 0.;
                        obs->speed = 0.;
                        obs->type = 0;
                        // 航向角
                        obs->yaw = 0.;

                        // 障碍物网格高度
                        obs->height = (pieobj.max_z - pieobj.min_z) * 0.01;
                        // 障碍物网格宽度
                        obs->width = g_CfgVeloView.cfgPlaneDetect.PieRadSize * 0.01;

                        // 3. Distance or 2D point?
                        // 障碍物网格2D坐标
                        // Obstacle A
                    #ifdef SEND_RECT
                        obs->x = current_radius * cos_cur_azi * 0.01;
                        obs->y = (current_radius * sin_cur_azi - LIDAR2GPS) * 0.01;
                        obs_list.push_back(*obs);
                        velodyne_lidar_data.num_of_obstacle++;
                    #else
                        obs->x = current_radius * cos_cur_azi * 0.01;
                        obs->y = (current_radius * sin_cur_azi - LIDAR2GPS) * 0.01;
                        obs_list.push_back(*obs);
                        velodyne_lidar_data.num_of_obstacle++;
                    #endif // SEND_RECT

                    #ifdef SEND_RECT
                        // Obstacle B
                        obs->x = next_radius * cos_cur_azi * 0.01;
                        obs->y = (next_radius * sin_cur_azi - LIDAR2GPS) * 0.01;
                        obs_list.push_back(*obs);
                        velodyne_lidar_data.num_of_obstacle++;
                        // Obstacle C
                        obs->x = next_radius * cos(next_azimuth) * 0.01;
                        obs->y = (next_radius * sin(next_azimuth) - LIDAR2GPS) * 0.01;
                        obs_list.push_back(*obs);
                        velodyne_lidar_data.num_of_obstacle++;
                        // Obstacle D
                        obs->x = current_radius * cos(next_azimuth) * 0.01;
                        obs->y = (current_radius * sin(next_azimuth) - LIDAR2GPS) * 0.01;
                        obs_list.push_back(*obs);
                        velodyne_lidar_data.num_of_obstacle++;
                    #endif // SEND_RECT
                    } // end of if (gridobj.cellType & CELL_TYPE_ABOVE_DELTA_Z){
                    else
                    {
                    #ifdef SAVE_GRID_MAP
                        gridmap[row][grid] = 0;
                    #endif
                    }
                } // end of for (unsigned rad = 0; rad < azimuthObj.size(); rad++)
            } //end of for (unsigned azimuth = 0; azimuth < pscanobj->scanPieArray.size(); azimuth++)
            #endif // 0
            #if 0
                // 构建障碍物信息数据
                nad_lcm::nad_obstacle* obs = new nad_lcm::nad_obstacle();
                obs->source = "velodyne-lidar";

                // 经纬度
                obs->lat = 0.;
                obs->lon = 0.;
                // GPS时间戳
                obs->gps_time = 0.;
                obs->speed = 0.;
                obs->type = 0;
                // 航向角
                obs->yaw = 0.;

                // 障碍物网格高度
                obs->height = 0.;
                // 障碍物网格宽度
                obs->width = 0.;

                // 3. Distance or 2D point?
                // 障碍物网格2D坐标
                // Obstacle A
                obs->x = 5.3;
                obs->y = 5.33;
                obs_list.push_back(*obs);
                velodyne_lidar_data.num_of_obstacle++;

                // Obstacle B
                obs->x = 5.4;
                obs->y = 5.33;
                obs_list.push_back(*obs);
                velodyne_lidar_data.num_of_obstacle++;

                // Obstacle C
                obs->x = 5.4;
                obs->y = 7.53;
                obs_list.push_back(*obs);
                velodyne_lidar_data.num_of_obstacle++;

                // Obstacle D
                obs->x = 4.3;
                obs->y = 10.53;
                obs_list.push_back(*obs);
                velodyne_lidar_data.num_of_obstacle++;


                // Obstacle A
                obs->x = 1.3;
                obs->y = 11.33;
                obs_list.push_back(*obs);
                velodyne_lidar_data.num_of_obstacle++;

                // Obstacle B
                obs->x = 1.4;
                obs->y = 1.33;
                obs_list.push_back(*obs);
                velodyne_lidar_data.num_of_obstacle++;

                // Obstacle C
                obs->x = -4.4;
                obs->y = 1.53;
                obs_list.push_back(*obs);
                velodyne_lidar_data.num_of_obstacle++;

                // Obstacle D
                obs->x = -1.3;
                obs->y = 7.53;
                obs_list.push_back(*obs);
                velodyne_lidar_data.num_of_obstacle++;


                // Obstacle A
                obs->x = 5.3;
                obs->y = 5.33;
                obs_list.push_back(*obs);
                velodyne_lidar_data.num_of_obstacle++;

                // Obstacle B
                obs->x = -5.4;
                obs->y = 5.33;
                obs_list.push_back(*obs);
                velodyne_lidar_data.num_of_obstacle++;

                // Obstacle C
                obs->x = 5.4;
                obs->y = 5.53;
                obs_list.push_back(*obs);
                velodyne_lidar_data.num_of_obstacle++;

                // Obstacle D
                obs->x = 5.3;
                obs->y = 5.53;
                obs_list.push_back(*obs);
                velodyne_lidar_data.num_of_obstacle++;
            #endif // 1

            #if 0
                // 构建障碍物信息数据
                nad_lcm::nad_obstacle* obs = new nad_lcm::nad_obstacle();
                obs->source = "velodyne-lidar";

                // 经纬度
                obs->lat = 0.;
                obs->lon = 0.;
                // GPS时间戳
                obs->gps_time = 0.;
                obs->speed = 0.;
                obs->type = 0;
                // 航向角
                obs->yaw = 0.;

                // 障碍物网格高度
                obs->height = 0.;
                // 障碍物网格宽度
                obs->width = 0.;

                // 3. Distance or 2D point?
                // 障碍物网格2D坐标
                // Obstacle A
                obs->x = 0;
                obs->y = 5.33;
                obs_list.push_back(*obs);
                velodyne_lidar_data.num_of_obstacle++;
            #endif // 0
        #endif

            velodyne_lidar_data.obu_obstacle.resize(velodyne_lidar_data.num_of_obstacle);
            for (unsigned obstacle = 0; obstacle < velodyne_lidar_data.num_of_obstacle; obstacle++)
            {
            #if 0
                printf("\t");
                printf("[ ");
                printf("Source:%s ", obs_list[obstacle].source.c_str());
                printf("Position: (%.2lf, %.2lf) ", obs_list[obstacle].x, obs_list[obstacle].y);
                printf("Height: %.2f ", obs_list[obstacle].height);
                printf("Width: %.2f ", obs_list[obstacle].width);
                printf("]");
                printf("\n");
            #endif
                velodyne_lidar_data.obu_obstacle[obstacle] = obs_list[obstacle];
            }
            lcm_velodyne_lidar.publish("obstacle_info", &velodyne_lidar_data);
            printf("num_of_obstacle=%d\n", velodyne_lidar_data.num_of_obstacle);
        #if 0
            char currentTime[64];
            getCurrentTimeStr(currentTime, sizeof(currentTime));
            printf("obstacle_list published at %s...\n", currentTime);
        #endif

        #ifdef GET_TIME_LCM
            end_clock = clock();
            // CLOCKS_PER_SEC is a constant in time.h header file.
            time_spent = (double)( end_clock - begin_clock)/CLOCKS_PER_SEC;
            printf("LCM time consumed: %.6lf[s]\n", time_spent);
        #endif
        } // end of if (pscanobj){
        else
        {
            //printf("empty scan buffer size...\n");
        }
        // 释放锁 将处理后的障碍物信息发出
        delete pscanobj;

#ifdef SAVE_GRID_MAP
        printf("GridCellSize=%d, RowNumber=%d, ColumnNumber=%d\n", GridCellSize, RowNumber, ColumnNumber);
        const char buf[64] = "hello world";
        gridmap_file.write(buf, sizeof(buf));
        gridmap_file << GridCellSize << " " << RowNumber << " " << ColumnNumber << std::endl;
		for (unsigned row = 0; row < rownumber; row++){
			for (unsigned col = 0; col < columnnumber; col++){
				gridmap_file << gridmap[row][col] << " ";
			}
			gridmap_file << std::endl;
		}
		gridmap_file << "// end of a frame" << std::endl;
		frame_count++;
		if (frame_count >= 1000){
			gridmap_file.flush();
			frame_count = 0;
		}
#endif
        usleep(50000);
        // sychronize with timer thread
    #ifdef OFF_LINE
        //usleep(10000);
    #else
        //usleep(10000);
    #endif // OFF_LINE
    } // end of while(true)

lcm_thread_end:
#ifdef SAVE_GRID_MAP
    if(gridmap_file != NULL && gridmap_file.is_open())
    {
        gridmap_file.close();
    }
#endif

    //lcm_destroy(&lcm_velodyne_lidar);
    printf("LCMThread Stoped...\n");
    return NULL;
}

#ifdef WIN32
VOID CALLBACK TimerProc(HWND hwnd, UINT message, UINT iTimerID, DWORD dwTime)
{
    pthread_spin_lock(&g_scanBuffer_lock);
    if (g_scanBufferSize > 1)
    {
        // 丢弃帧
        if (g_scanBufferSize > HALF_SCAN_BUFFER_SIZE)
        {
            g_scanBufferReadIdx = (g_scanBufferReadIdx + 2) % SCAN_BUFFER_SIZE;
            g_scanBufferSize -= 2;
            //g_scanDrop++;
        }
        else
        {
            g_scanBufferReadIdx = (g_scanBufferReadIdx + 1) % SCAN_BUFFER_SIZE;
            g_scanBufferSize--;
        }
        g_scanBuffer[g_scanBufferReadIdx].isTrackerHandled = false;
    }
    pthread_spin_unlock(&g_scanBuffer_lock);
}
#else
//#define TIMEER_TEST
void TimerProc(int num)
{
#ifdef TIMEER_TEST
    struct timeval tp;
#ifdef WIN32
    gettimeofday(&tp);
#else
    gettimeofday(&tp, NULL);
#endif // WIN32
    struct tm *ltime;
    char timestr[32];

    time_t local_tv_sec;

    // convert the timestamp to readable format
    local_tv_sec = tp.tv_sec;
    ltime = localtime(&local_tv_sec);
    strftime(timestr, sizeof(timestr), "%Y%m%d%H%M%S", ltime);
    printf("%s [usec]%d\n", timestr, tp.tv_usec);
#endif
/*
    //pthread_spin_lock(&g_scanBuffer_lock);
    if (g_scanBufferSize > 1)
    {
        // 丢弃帧
        if (g_scanBufferSize > HALF_SCAN_BUFFER_SIZE)
        {
            g_scanBufferReadIdx = (g_scanBufferReadIdx + 2) % SCAN_BUFFER_SIZE;
            g_scanBufferSize -= 2;
            //g_scanDrop++;
        }
        else
        {
            g_scanBufferReadIdx = (g_scanBufferReadIdx + 1) % SCAN_BUFFER_SIZE;
            g_scanBufferSize--;
        }
        //g_scanBuffer[g_scanBufferReadIdx].isTrackerHandled = false;
    }
*/
    glutPostRedisplay();
    //myDisplay();
    char currentTime[64];
    getCurrentTimeStr(currentTime, sizeof(currentTime));
    printf("glutPostRedisplay at %s\n", currentTime);

    //pthread_spin_unlock(&g_scanBuffer_lock);
}
#endif // WIN32

extern int argc_gl;
extern char **argvs_gl;
void* OpenGLThread(void* arg){
    glutInit(&argc_gl, argvs_gl);
    MyGLDispIni();
}
