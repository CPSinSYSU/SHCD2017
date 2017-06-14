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

#ifdef WIN32
    #pragma comment(lib, "lcm.lib")
#endif // WIN32

#include "velodyneScanner.h"
#include <sys/time.h>       // for gettimeofday, clock_t
#include <fstream>          // for ofstrem

#include <math.h>           // for M_PI

#include "velodyneDraw.h"   // for OpenGLThread

#include <string.h>         // for memset

// pthread id
pthread_t g_socket_th;
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

    printf("SocketThread stopped...\n");
    return NULL;
}

extern int argc_gl;
extern char **argvs_gl;
void* OpenGLThread(void* arg) {
    glutInit(&argc_gl, argvs_gl);
    printf("OpenGLThread started...\n");
    MyGLDispIni();
    printf("OpenGLThread stopped...\n");
}
