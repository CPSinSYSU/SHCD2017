#ifndef __VELODYNE_THREAD_H__
#define __VELODYNE_THREAD_H__

/**************************************************************************
 *
 * Velodyne Process Threads Header File(for pthread)
 *	Socket Thread
 *	LCM Thread
 *	Timer Processing Function
 *
 ***************************************************************************/
#include <stdio.h> //for FILE ,lxs 

#ifdef WIN32
#include <Windows.h>
#endif // WIN32

#include <pthread.h>
#ifdef WIN32
#pragma comment(lib,"x86/pthreadVC2.lib")
#endif // WIN32

#include "velodyneDataStruct.h"
#include "velodyneScanner.h"

void* SocketThread(void* arg);
void* OpenGLThread(void* arg);

#ifdef WIN32
VOID CALLBACK TimerProc(HWND hwnd, UINT message, UINT iTimerID, DWORD dwTime);
#else
void TimerProc(int num);
#endif // WIN32

extern pthread_t g_socket_th;
extern pthread_t g_opengl_th;

extern pthread_spinlock_t g_scanBuffer_lock;
extern pthread_spinlock_t g_glut_lock;

extern VelodyneDataRaw g_scanBuffer[SCAN_BUFFER_SIZE];
extern int g_scanBufferSize;
extern int g_scanBufferReadIdx;
extern int g_scanBufferWriteIdx;
extern int g_scanDrop;

extern FILE* g_fout;//lxs

#endif
