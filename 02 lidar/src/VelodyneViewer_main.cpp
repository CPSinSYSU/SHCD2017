#include "configStruct.h"
#include <stdio.h>

#include <pthread.h>
#include <sched.h>
#ifdef WIN32
    #pragma comment(lib, "x86/pthreadVC2.lib")
#endif

#include "velodyneThread.h"
#include <unistd.h>   		/* pause */
#include <signal.h>   		/* signal */
#include <string.h>   		/* for memset */
#include <sys/time.h>

int argc_gl;
char **argvs_gl;

int main(int argc, char *argv[]) {
    argc_gl = argc;
    argvs_gl = argv;

    char lidar_type[64] = "Velodyne-HDL-32E";
#ifdef USE_HDL_64E_
    strcpy(lidar_type, "Velodyne-HDL-64E");
#endif
#ifdef USE_VLP_16_
    strcpy(lidar_type, "Velodyne-VLP-16");
#endif
    printf("LiDAR: %s\n", lidar_type);

    LoadCfgVeloView(g_CfgVeloView, "./velodyne.ini");

    #if defined(_POSIX_THREAD_PRIORITY_SCHEDULING)
    #else
        printf("_POSIX_THREAD_PRIORITY_SCHEDULING not allowed\n")
    #endif // defined

    int status;

    pthread_attr_t thread_attr;
    struct sched_param thread_param;
    int rr_min_priority, rr_max_priority, rr_mid_priority;
    int policy;
    pthread_t main_th;
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
    rr_min_priority = sched_get_priority_min(SCHED_RR);
    rr_mid_priority = (rr_max_priority + rr_min_priority) / 2;

    main_th = pthread_self();
    // pthread_create 优先级默认与创建的父进程一致 所以需要设置 main_th
    status = pthread_getschedparam(main_th, &policy, &thread_param);
    if (status != 0) {
        printf("get main sched ERROR, status=%d\n", status);
    }
    thread_param.sched_priority = rr_min_priority;
    status = pthread_setschedparam(main_th, SCHED_RR, &thread_param);
    if (status != 0) {
        printf("set main thread sched ERROR, status=%d\n", status);
    }

    pthread_create(&(g_socket_th), NULL, &(SocketThread), NULL);
#if 1
    status = pthread_getschedparam(g_socket_th, &policy, &thread_param);
    if (status != 0) {
        printf("get socket thread sched ERROR, status=%d\n", status);
    }
    else {
        // printf("socket thread prority: %d\n", thread_param.__sched_priority);
    }
    thread_param.sched_priority = rr_max_priority;
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
    thread_param.sched_priority = rr_max_priority;
    status = pthread_setschedparam(g_opengl_th, SCHED_RR, &thread_param);
    if (status != 0) {
        printf("set opengl thread sched ERROR, status=%d\n", status);
    }
#endif

    pthread_join(g_opengl_th, NULL);
    pthread_join(g_socket_th, NULL);

#ifdef WIN32
    KillTimer(NULL, iTimerID);
#endif // WIN32

    pthread_attr_destroy(&thread_attr);

    return 0;
}
