/**************************************************************************
 *
 * 画各种图标进行分析
 *
 ***************************************************************************/
#include "velodyneDraw.h"
// #include <iostream>
// using namespace std;

#ifdef WIN32
#include <gl/glut.h>
#else
#include <GL/glut.h>
#endif

#include <stdio.h>              // for printf

#include "velodyneScanner.h"
#include "configStruct.h"
#include "velodyneThread.h"
#include <stdint.h>
#include <string.h>             // for memset

#include "velodyneDataStruct.h"
#include <unistd.h>             // for usleep()
#include <math.h>               // for sin ...

uint8_t show_state = SHOW_ALL_POINTS_BELOW;
double max_y = 0;
double max_tan = 0;
static double absf(double a)
{
    if (a < 0)
        return -a;
    return a;
}
void drawCar(float angle)
{
    float min_x = -1.0 * (g_CfgVeloView.cfgGlobal.CarWidth / 2);
    float max_x = 1.0 * (g_CfgVeloView.cfgGlobal.CarWidth / 2);
    float min_y = -1.0 * (g_CfgVeloView.cfgGlobal.CarLength / 2);
    float max_y = 1.0 * (g_CfgVeloView.cfgGlobal.CarLength / 2);
    float min_z = -1.0 * (g_CfgVeloView.cfgGlobal.CarHeight / 2);
    float max_z = 1.0 * (g_CfgVeloView.cfgGlobal.CarHeight / 2);
    // 极坐标旋角
    glRotatef(angle, 0.0, 0.0, 1.0);
    // 车尾
    drawLineGLRGB(min_x, min_y, min_z, min_x, min_y, max_z, 1.0, 1.0, 1.0);
    drawLineGLRGB(min_x, min_y, max_z, max_x, min_y, max_z, 1.0, 1.0, 1.0);
    drawLineGLRGB(min_x, min_y, min_z, max_x, min_y, min_z, 1.0, 1.0, 1.0);
    drawLineGLRGB(max_x, min_y, min_z, max_x, min_y, max_z, 1.0, 1.0, 1.0);
    // 车体
    drawLineGLRGB(min_x, min_y, max_z, min_x, max_y, max_z, 1.0, 1.0, 1.0);
    drawLineGLRGB(min_x, min_y, min_z, min_x, max_y, min_z, 1.0, 1.0, 1.0);
    drawLineGLRGB(max_x, min_y, max_z, max_x, max_y, max_z, 1.0, 1.0, 1.0);
    drawLineGLRGB(max_x, min_y, min_z, max_x, max_y, min_z, 1.0, 1.0, 1.0);
    // 车头
    drawLineGLRGB(min_x, max_y, max_z, max_x, max_y, max_z, 1.0, 0., 0.);
    drawLineGLRGB(min_x, max_y, min_z, max_x, max_y, min_z, 1.0, 0., 0.);
    drawLineGLRGB(max_x, max_y, min_z, max_x, max_y, max_z, 1.0, 0., 0.);
    drawLineGLRGB(min_x, max_y, min_z, min_x, max_y, max_z, 1.0, 0., 0.);

    glRotatef(-angle, 0.0, 0.0, 1.0);
}

void drawAllPoints(int psize, int mode)
{
    glPointSize(psize);
    glBegin(GL_POINTS);

    int size;
    int circle_start, circle_end;
    switch (mode)
    {
    case 1:
        circle_start = 0;
        circle_end = VELODYNE_NUM_BEAMS_IN_ONE_SHOT;
        break;
    case 2:
        circle_start = UPPER_VERT_FROM;
        circle_end = VELODYNE_NUM_BEAMS_IN_ONE_SHOT;
        break;
    default:
        circle_start = 0;
        circle_end = 0;
        break;
    }

    //pthread_spin_lock(&g_scanBuffer_lock);
    VelodyneDataStruct* pscanobj = getScanRawForDraw();
    if (pscanobj)
    {
        if (mode == 1)
        {
            printf("draw all points below...\n");
        }
        else if (mode == 2)
        {
            printf("draw all points above...\n");
        }
        //printf("draw points\n");
        // 每个Circle "configStruct.h"
        for (unsigned circle = circle_start; circle <circle_end; circle++)
        {
            //  整个一周
            size = pscanobj->shots.size();
            for (unsigned shot = 0; shot<size; shot++)
            {
                if ((*pscanobj).shots[shot].pt[circle].point_type & POINT_TYPE_INVALID)
                    continue;
		double temp_x,temp_y,temp_z;
		temp_x = (*pscanobj).shots[shot].pt[circle].x;
		temp_y = (*pscanobj).shots[shot].pt[circle].y;
		temp_z = (*pscanobj).shots[shot].pt[circle].z;
		/*if(temp_y > 0 && absf(temp_x) < temp_y/2 && temp_z > -40 && temp_z < 200)
		{
		    drawPointRGB((*pscanobj).shots[shot].pt[circle], 1.0, 1.0, 0.1, psize);
		    if(max_y < temp_y){
			max_y = temp_y;
		    }
		    printf("X:%f,Y:%f,Z:%f,max_Y:%f\n",temp_x,temp_y,temp_z,max_y);
		}*/
		if(temp_y > 0 && absf(temp_x) < temp_y/2 && temp_z < -40)
		{
		    drawPointRGB((*pscanobj).shots[shot].pt[circle], 1.0, 1.0, 1.0, psize);
		    if(absf(temp_z)/temp_y>max_tan){
			max_tan = absf(temp_z)/temp_y;
		    }
		    printf("X:%f,Y:%f,Z:%f,max_tan:%f\n",temp_x,temp_y,temp_z,max_tan);
		}
                // 绿色
                /*if ((*pscanobj).shots[shot].pt[circle].x > 0 && (*pscanobj).shots[shot].pt[circle].y > 0)
                {
                    drawPointRGB((*pscanobj).shots[shot].pt[circle], 0.1, 1.0, 0.1, psize);
                }
                // 红色
                else if ((*pscanobj).shots[shot].pt[circle].x < 0 && (*pscanobj).shots[shot].pt[circle].y > 0)
                {
                    drawPointRGB((*pscanobj).shots[shot].pt[circle], 1.0, 0.1, 0.1, psize);
                }
                else if ((*pscanobj).shots[shot].pt[circle].x < 0 && (*pscanobj).shots[shot].pt[circle].y < 0)
                {
                    drawPointRGB((*pscanobj).shots[shot].pt[circle], 0.1, 1.0, 1.0, psize);
                }
                // 蓝色
                else if ((*pscanobj).shots[shot].pt[circle].x > 0 && (*pscanobj).shots[shot].pt[circle].y < 0)
                {
                    drawPointRGB((*pscanobj).shots[shot].pt[circle], 0.1, 0.1, 1.0, psize);
                }
                else
                {
                    drawPointRGB((*pscanobj).shots[shot].pt[circle], 1.0, 1.0, 0.1, psize);
                }*/
                // 画地面
                //if ((*pscanobj).shots[shot].pt[circle].z == g_CfgVeloView.cfgGlobal.GroundZ){
                //	drawPointRGB((*pscanobj).shots[shot].pt[circle], 1.0, 1.0, 1.0, 2);
                //}
                //printf("(%.2f, %.2f, %.2f)\n", (*pscanobj).shots[i].pt[j].x, (*pscanobj).shots[i].pt[j].y, (*pscanobj).shots[i].pt[j].z);
                // 画天花板
                /*if ((*pscanobj).shots[shot].pt[circle].z > 100)
                {
                    drawPointRGB((*pscanobj).shots[shot].pt[circle], 1.0, 1.0, 1.0, 2);
                }*/
//#define GET_GROUND_Z
#ifdef GET_GROUND_Z
                if (circle == 0)
                {
                    printf("#%d: (%.2f, %.2f, %.2f)\n", shot,
                           (*pscanobj).shots[shot].pt[circle].x,
                           (*pscanobj).shots[shot].pt[circle].y,
                           (*pscanobj).shots[shot].pt[circle].z);
                }
#endif
            }
        }
    }
    /*char s[] = "this is a test";
    drawTextRGB(0.0, 1000.0, 0.0, 1.0, 1.0, 1.0, s);*/
    //pthread_spin_unlock(&g_scanBuffer_lock);
    delete pscanobj;

    glEnd();
}

void drawPointsHeightMap(int psize)
{
    
}

void drawPointsNagetiveObstacleMap(int psize)
{
    
}

void drawPointsGridCell(float angle, int psize)
{
    
}
void drawPointsPieCell(float angle, int psize)
{
    
}

/*
 *	绘制三维点
 */
void drawPointRGB(VelodyneDataStruct::xpoint_t &pt, float r, float g, float b, int size)
{
    GLdouble dVect1[3];
    glColor3f(r, g, b);
    glPointSize(size);
    dVect1[0] = pt.x/100.;
    dVect1[1] = pt.y/100.;
    dVect1[2] = pt.z/100.;
    glVertex3dv(dVect1);
}
void drawPointRGB(float x, float y, float z, float psize, float r, float g, float b)
{
    glColor3f(r, g, b);
    glPointSize(psize);
    glBegin(GL_POINTS);
    glVertex3f(x, y, z);
    glEnd();
}
/*
 *	绘制立方体
 *	(min_x, min_y, min_z) --> (max_x, max_y, max_z)
 *	12条边
 */
void drawCubeGLRGB(float min_x, float min_y, float min_z, float max_x, float max_y, float max_z,
                   float r, float g, float b)
{
    drawLineGLRGB(min_x, min_y, max_z, max_x, min_y, max_z, r, g, b);
    drawLineGLRGB(min_x, max_y, max_z, max_x, max_y, max_z, r, g, b);
    drawLineGLRGB(min_x, max_y, min_z, max_x, max_y, min_z, r, g, b);
    drawLineGLRGB(min_x, min_y, min_z, max_x, min_y, min_z, r, g, b);

    drawLineGLRGB(min_x, min_y, max_z, min_x, max_y, max_z, r, g, b);
    drawLineGLRGB(max_x, min_y, max_z, max_x, max_y, max_z, r, g, b);
    drawLineGLRGB(min_x, min_y, min_z, min_x, max_y, min_z, r, g, b);
    drawLineGLRGB(max_x, min_y, min_z, max_x, max_y, min_z, r, g, b);

    drawLineGLRGB(min_x, min_y, min_z, min_x, min_y, max_z, r, g, b);
    drawLineGLRGB(max_x, min_y, min_z, max_x, min_y, max_z, r, g, b);
    drawLineGLRGB(max_x, max_y, min_z, max_x, max_y, max_z, r, g, b);
    drawLineGLRGB(min_x, max_y, min_z, min_x, max_y, max_z, r, g, b);
}

/*
 *	标记障碍物cluster
 */
void drawCubeGLRGB(VelodyneDataStruct::gridClusterFeature_t& grid_cluFeature, float r, float g, float b)
{
    drawCubeGLRGB(	grid_cluFeature.min_x,
                    grid_cluFeature.min_y,
                    grid_cluFeature.min_z,
                    grid_cluFeature.max_x,
                    grid_cluFeature.max_y,
                    grid_cluFeature.max_z,
                    r, g, b);
}
void drawCubeGLRGB(VelodyneDataStruct::pieClusterFeature_t& pie_cluFeature, float r, float g, float b)
{
    drawCubeGLRGB(  pie_cluFeature.min_x,
                    pie_cluFeature.min_y,
                    pie_cluFeature.min_z,
                    pie_cluFeature.max_x,
                    pie_cluFeature.max_y,
                    pie_cluFeature.max_z,
                    r, g, b);
}
void drawLineGLRGB(float x1, float y1, float z1, float x2, float y2, float z2,
                   float r, float g, float b, int wide)
{
    GLdouble dVect1[3], dVect2[3];
    glLineWidth(wide);
    glBegin(GL_LINES);
    glColor3d(r, g, b);

    dVect1[0] = x1/100.;
    dVect1[1] = y1/100.;
    dVect1[2] = z1/100.;
    glVertex3dv(dVect1);

    dVect2[0] = x2/100.;
    dVect2[1] = y2/100.;
    dVect2[2] = z2/100.;
    glVertex3dv(dVect2);
    glEnd();
}
/*
 *	绘制文本
 */
void drawTextRGB(float x, float y, float z, float r, float g, float b, char* outputstring)
{
    glRasterPos3f(x, y, z);
    glColor3f(r, g, b);

#ifdef WIN32
    glPushMatrix();
    // Windows系统中，可以使用wglUseFontBitmaps函数来批量的产生显示字符用的显示列表
    wglUseFontBitmaps(wglGetCurrentDC(), 0, 255, 100);
    glListBase(100);
    glCallLists(strlen(outputstring), GL_UNSIGNED_BYTE, outputstring);

    glPopMatrix();
#else
    int textIdx = 0;
    while(outputstring[textIdx] != '\0') {
    	textIdx++;
    }
#endif
}

void drawCircle(float center_x, float center_y, float radius, float r, float g, float b, float psize)
{
    float angleInterval = 0.5;
    if (radius>2000)
        angleInterval = 0.25;
    for (float angle = 0; angle<360; angle += angleInterval)
    {
        float x = center_x + radius*sin(angle*M_PI / 180);
        float y = center_y + radius*cos(angle*M_PI / 180);
        drawPointRGB(x, y, g_CfgVeloView.cfgGlobal.GroundZ, psize, r, g, b);
    }
}

/***********************************************************************************************
 *		directly used codes in previous draw_velodyne.cpp
 */
int x_lbefore, y_lbefore;
int x_rbefore, y_rbefore;
int z_before1, z_before2;

bool buttonSaveLeft, buttonSaveMiddle, buttonSaveRight;
float x_move, y_move, z_move;
float x_move_save, y_move_save, z_move_save;
float x_rotate, y_rotate, z_rotate;
float x_rotate_save, y_rotate_save, z_rotate_save;
float m_zoom;

float m_aspect;
float m_eyex, m_eyey, m_eyez;
float m_centerx, m_centery, m_centerz;
float m_upx, m_upy, m_upz;

void  MyGLDispIni()
{
    m_eyex = 0, m_eyey = 0, m_eyez = 80;
    m_centerx = 0, m_centery = 0, m_centerz = 0;
    m_upx = 0, m_upy = 1, m_upz = 0;

    buttonSaveLeft = false;
    buttonSaveMiddle = false;
    buttonSaveRight = false;

    x_move = 0.0;
    y_move = 0.0;
    z_move = 0.0;
    x_rotate = 0.0;
    y_rotate = 0.0;
    z_rotate = 0.0;
    m_zoom = 1.97566;

    x_lbefore = 0, y_lbefore = 0;
    x_rbefore = 0, y_rbefore = 0;
    z_before1 = 0, z_before2 = 0;

    GLenum type;
    type = GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE;
    glutInitDisplayMode(type);

    glutInitWindowSize(800, 600);
    glutCreateWindow("VelodyneHDL-Viewer1");

    // 各种响应函数影响 Display Function 中的参数
    glutReshapeFunc(Reshape);
    //glutKeyboardFunc(Key);
    glutSpecialFunc(SpecialKey);
    //glutMouseFunc(MouseKey);
    //glutMotionFunc(MouseMove);
    //glutPassiveMotionFunc(PassiveMouseMove);
    //glutSpaceballRotateFunc(MouseRotate);
    // glutDisplayFunc(&myDisplay);  一般不这么写, 去掉&符号...
    glutDisplayFunc(myDisplay);
    glutIdleFunc(idleFunc);
    glutMainLoop();
}

void idleFunc(void){
    usleep(100000);
    glutPostRedisplay();
}

void myDisplay(void)
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(0.0, 0.0, 0.0, 0.0);

    glLoadIdentity();

    gluLookAt(m_eyex, m_eyey, m_eyez,
              m_centerx, m_centery, m_centerz,
              m_upx, m_upy, m_upz);

    glScalef(1, 1, 1);

    glRotatef(x_rotate, 1, 0, 0);
    glRotatef(y_rotate, 0, 1, 0);
    glRotatef(z_rotate, 0, 0, 1);

    glTranslatef(x_move, y_move, z_move);
    glScalef(m_zoom, m_zoom, m_zoom);

    // draw the car model
    //drawCar(0.);
    // for 16-line -135
    drawCar(0.0);
    // draw the coordinate
    drawLineGLRGB(0., 0., 0., 500., 0., 0., 1., 0., 0., 3.0);
    drawLineGLRGB(0., 0., 0., 0., 800., 0., 0., 1., 0., 3.0);
    drawLineGLRGB(0., 0., 0., 0., 0., 350., 0., 0., 1., 3.0);
    char str[] = "sssssssssssssssssssssssssssssssssssssssssssss";
    drawTextRGB(0., 10000., 0., 1., 1., 1., str);

    switch (show_state)
    {
    case SHOW_ALL_POINTS_ABOVE:
        drawAllPoints(1, 2);
        break;
    case SHOW_ALL_POINTS_BELOW:
        drawAllPoints(1, 1);
        break;
    case SHOW_OBSTACLES_GRID:
        drawAllPoints(1, 1);
        drawAllPoints(1, 2);
        drawPointsGridCell(-135, 1);
        break;
    case SHOW_OBSTACLES_PIE:
        drawAllPoints(1, 1);
        drawAllPoints(1, 2);
        drawPointsPieCell(-135, 1);
        break;
    case SHOW_HEIGHT_MAP:
        drawAllPoints(1, 1);
        drawAllPoints(1, 2);
        drawPointsHeightMap(1);
        break;
    case SHOW_NEGATIVE_OBSTACLE:
        drawAllPoints(1, 1);
        drawAllPoints(1, 2);
        drawPointsNagetiveObstacleMap(1);
    default:
        break;
    }

    glFlush();
    glutSwapBuffers();
}

void SpecialKey(int key, int x, int y)
{
    //int mod = 0;
    switch (key)
    {
    case GLUT_KEY_UP:
        //mod = glutGetModifiers();
        //if (mod == GLUT_ACTIVE_ALT){
        //}
        //else if (mod == GLUT_ACTIVE_SHIFT){
        //}
        //else if (mod == GLUT_ACTIVE_CTRL){
        //}
        //else
        y_move++;
        break;
    case GLUT_KEY_DOWN:
        //mod = glutGetModifiers();
        //if (mod == GLUT_ACTIVE_ALT){
        //}
        //else if (mod == GLUT_ACTIVE_SHIFT){
        //}
        //else if (mod == GLUT_ACTIVE_CTRL){
        //}
        //else
        y_move--;
        break;
    case GLUT_KEY_LEFT:
        //mod = glutGetModifiers();
        //if (mod == GLUT_ACTIVE_ALT){
        //}
        //else if (mod == GLUT_ACTIVE_SHIFT){
        //}
        //else if (mod == GLUT_ACTIVE_CTRL){
        //}
        //else
        x_move--;
        break;
    case GLUT_KEY_RIGHT:
        //mod = glutGetModifiers();
        //if (mod == GLUT_ACTIVE_ALT){
        //}
        //else if (mod == GLUT_ACTIVE_SHIFT){
        //}
        //else if (mod == GLUT_ACTIVE_CTRL){
        //}
        //else
        x_move++;
        break;
    // PgUp 放大
    case GLUT_KEY_PAGE_UP:
        m_zoom = 1.1 * m_zoom;
        break;
    // PgDn 缩小
    case GLUT_KEY_PAGE_DOWN:
        m_zoom = m_zoom / 1.1;
        break;
    case GLUT_KEY_HOME:
        m_zoom = 1.5 * m_zoom;
        break;
    case GLUT_KEY_END:
        m_zoom = m_zoom / 1.5;
        break;
    // 旋转操作
    case  GLUT_KEY_F1:
        x_rotate += 3;
        if (x_rotate > 360)
            x_rotate = x_rotate - 360;
        if (x_rotate < -360)
            x_rotate = x_rotate + 360;
        break;
    case  GLUT_KEY_F2:
        x_rotate += -3;
        if (x_rotate > 360)
            x_rotate = x_rotate - 360;
        if (x_rotate < -360)
            x_rotate = x_rotate + 360;
        break;
    case  GLUT_KEY_F3:
        y_rotate += 3;
        if (y_rotate > 360)
            y_rotate = y_rotate - 360;
        if (y_rotate < -360)
            y_rotate = y_rotate + 360;
        break;
    case  GLUT_KEY_F4:
        y_rotate += -3;
        if (y_rotate > 360)
            y_rotate = y_rotate - 360;
        if (y_rotate < -360)
            y_rotate = y_rotate + 360;
        break;
    case  GLUT_KEY_F5:
        z_rotate += atanf(3);
        if (z_rotate > 360)
            z_rotate = z_rotate - 360;
        if (z_rotate < -360)
            z_rotate = z_rotate + 360;
        break;
    case GLUT_KEY_F6:
        z_rotate += atanf(-3);
        if (z_rotate > 360)
            z_rotate = z_rotate - 360;
        if (z_rotate < -360)
            z_rotate = z_rotate + 360;
        break;
    case GLUT_KEY_F7:
        show_state = SHOW_OBSTACLES_PIE;
        break;
    case GLUT_KEY_F8:
        show_state = SHOW_NEGATIVE_OBSTACLE;
        break;
    case GLUT_KEY_F9:
        show_state = SHOW_HEIGHT_MAP;
        break;
    case GLUT_KEY_F10:
        show_state = SHOW_OBSTACLES_GRID;
        break;
    case GLUT_KEY_F11:
        show_state = SHOW_ALL_POINTS_BELOW;
        break;
    case GLUT_KEY_F12:
        show_state = SHOW_ALL_POINTS_ABOVE;
        break;
    default:
        return;
    }

    // cout << x_rotate << " " << y_rotate << " " << z_rotate << endl;
    // cout << x_move << " " << y_move << " " << z_move << endl;
    // cout << m_zoom << endl;

    glutPostRedisplay();
}

void Reshape(int w, int h)
{
    glViewport(0, 0, (GLint)w, (GLint)h);

    m_aspect = (GLfloat)w / (GLfloat)h;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(45.0, m_aspect, 0.0f, 4000.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

/***********************************************************************************************/
