#include "configStruct.h"
#include <stdio.h>

#include <pthread.h>
#include <sched.h>
#ifdef WIN32
    #pragma comment(lib, "x86/pthreadVC2.lib")
#endif

#include "velodyneThread.h"
#include <unistd.h>   // for pause
#include <signal.h>   // for signal
#include <string.h>   // for memset
#include <sys/time.h>

int argc_gl;
char **argvs_gl;

// 主程序
int main(int argc, char *argv[])
{
    argc_gl = argc;
    argvs_gl = argv;

    char lidar_type[64] = "Velodyne-HDL-32E";
#ifdef USE_HDL_64E_
    strcpy(lidar_type, "Velodyne-HDL-64E");
#endif
#ifdef USE_VLP_16_
    strcpy(lidar_type, "Velodyne-VLP-16");
#endif
    printf("%s\n", lidar_type);

    LoadCfgVeloView(g_CfgVeloView, "./velodyne.ini");

#ifndef NO_DRAW
    //glutInit(&argc, argv);
#endif // NO_DRAW

#if 1
    #if defined(_POSIX_THREAD_PRIORITY_SCHEDULING)
    #else
        printf("_POSIX_THREAD_PRIORITY_SCHEDULING not allowed\n")
    #endif // defined

    pthread_attr_t thread_attr;

    struct sched_param thread_param;
    int status, rr_min_priority, rr_max_priority, rr_mid_priority;
    int policy;
    pthread_t main_th;
//#define SCHED_RR SCHED_FIFO
    status = pthread_attr_setschedpolicy(&thread_attr, SCHED_RR);
    if(status != 0){
        printf("Unable to set SCHED_RR policy.\n");
    }
    // 使优先级起作用
    status = pthread_attr_setinheritsched(&thread_attr, PTHREAD_EXPLICIT_SCHED);
    if(status != 0){
        printf("Unable to set inheritsched policy.\n");
    }
    /**
     * SCHED_OTHER是不支持优先级使用的，而SCHED_FIFO和SCHED_RR支持优先级的使用，他们分别为1和99
     * 数值越大优先级越高
     */
    rr_max_priority = sched_get_priority_max(SCHED_RR);
    rr_mid_priority = (rr_max_priority + rr_min_priority) / 2;
    rr_min_priority = sched_get_priority_min(SCHED_RR);
#if 0
    printf("rr_max_priority = %d\n", rr_max_priority);
    printf("rr_min_priority = %d\n", rr_min_priority);
#endif // 0

    #if 1
    main_th = pthread_self();
    status = pthread_getschedparam(main_th, &policy, &thread_param);
    //printf("main thread policy: %d\n", policy);
    //printf("main thread prority: %d\n", thread_param.__sched_priority);
    if (status != 0) {
        printf("get main sched ERROR, status=%d\n", status);
    }
    thread_param.sched_priority = rr_min_priority;
    status = pthread_setschedparam(main_th, SCHED_RR, &thread_param);
    if (status != 0) {
        printf("set main thread sched ERROR, status=%d\n", status);
    }
    #endif // 0

#endif
    pthread_spin_init(&g_scanBuffer_lock, 0);
    pthread_spin_init(&g_glut_lock, 0);

    // pthread_create 优先级默认与创建的父进程一致 所以需要设置 main_th
    pthread_create(&(g_socket_th), NULL, &(SocketThread), NULL);
#if 1
    status = pthread_getschedparam(g_socket_th, &policy, &thread_param);
    if (status != 0) {
        printf("get socket thread sched ERROR, status=%d\n", status);
    }
    else{
        //printf("socket thread prority: %d\n", thread_param.__sched_priority);
    }
    thread_param.sched_priority = rr_mid_priority;
    status = pthread_setschedparam(g_socket_th, SCHED_RR, &thread_param);
    if (status != 0) {
        printf("set socket thread sched ERROR, status=%d\n", status);
    }
#endif

    pthread_create(&(g_opengl_th), NULL, &(OpenGLThread), NULL);
#if 1
    status = pthread_getschedparam(g_opengl_th, &policy, &thread_param);
    if (status != 0) {
        printf("get opengl thread sched ERROR, status=%d\n", status);
    }
    thread_param.sched_priority = rr_min_priority;
    status = pthread_setschedparam(g_opengl_th, SCHED_RR, &thread_param);
    if (status != 0) {
        printf("set opengl thread sched ERROR, status=%d\n", status);
    }
#endif

#if 0
    // Timer period 80ms
#ifdef WIN32
    UINT iTimerID = SetTimer(NULL, 0, 80, TimerProc);
#else
    // add by sean
    // linux setitimer: 80ms
    // Get system call result to determine successful or failed
    int res = 0;
    // TimerProc to SIGALRM
    signal(SIGALRM, TimerProc);
    struct itimerval tick;
    // Initialize struct
    memset(&tick, 0, sizeof(tick));
    // Timeout to run function first time
    tick.it_value.tv_sec = 1;                   // sec
    tick.it_value.tv_usec = 1;                  // micro sec
    // Interval time to run function
    tick.it_interval.tv_sec = 0;
    tick.it_interval.tv_usec = 80000;
#ifdef OFF_LINE
#else
    tick.it_interval.tv_usec = 80000;
#endif // OFF_LINE

    // Set timer, ITIMER_REAL : real-time to decrease timer,
    //            send SIGALRM when timeout
    res = setitimer(ITIMER_REAL, &tick, NULL);
    if (res)
    {
        printf("setitimer failed!\n");
        return -1;
    }
#endif // WIN32
#endif

    pthread_join(g_opengl_th, NULL);
    pthread_join(g_socket_th, NULL);

#ifdef WIN32
    KillTimer(NULL, iTimerID);
#endif // WIN32

    pthread_attr_destroy(&thread_attr);

    pthread_spin_destroy(&g_scanBuffer_lock);
    pthread_spin_destroy(&g_glut_lock);

    return 0;
}
