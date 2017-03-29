#ifndef __VELODYNE_DRAW_H__
#define __VELODYNE_DRAW_H__

/**************************************************************************
 *
 * 画各种图标进行分析
 *
 ***************************************************************************/

#include "velodyneDataStruct.h"

/***********************************************************************************************
 *		directly used codes in previous draw_velodyne.cpp
 */
extern int x_lbefore, y_lbefore;
extern int x_rbefore, y_rbefore;
extern int z_before1, z_before2;
extern bool buttonSaveLeft, buttonSaveMiddle, buttonSaveRight;
extern float x_move, y_move, z_move;
extern float x_move_save, y_move_save, z_move_save;
extern float x_rotate, y_rotate, z_rotate;
extern float x_rotate_save, y_rotate_save, z_rotate_save;
extern float m_zoom;
extern float m_aspect;
extern float m_eyex, m_eyey, m_eyez;
extern float m_centerx, m_centery, m_centerz;
extern float m_upx, m_upy, m_upz;

void MyGLDispIni();
void myDisplay();
void idleFunc(void);
void SpecialKey(int key, int x, int y);
void Reshape(int w, int h);
void MouseKey(int button, int state, int x, int y);
void MouseRotate(int x, int y, int z);
void PassiveMouseMove(int x, int y);
void MouseMove(int x, int y);

/***********************************************************************************************/

#define SHOW_ALL_POINTS_ABOVE	0
#define SHOW_ALL_POINTS_BELOW	1
#define SHOW_OBSTACLES_GRID		2
#define SHOW_HEIGHT_MAP			3
#define SHOW_OBSTACLES_PIE		4
#define SHOW_NEGATIVE_OBSTACLE	5
void drawCar(float angle);
void drawAllPoints(int psize, int mode);
void drawPointsGridCell(float angle, int psize);
void drawPointsPieCell(float angle, int psize);
void drawPointsHeightMap(int psize);

void drawPointRGB(VelodyneDataStruct::xpoint_t &pt, float r, float g, float b, int size = 1);
void drawPointRGB(float x, float y, float z, float psize, float r, float g, float b);

void drawCubeGLRGB(float min_x, float min_y, float min_z, float max_x, float max_y, float max_z,
                   float r, float g, float b);
void drawCubeGLRGB(VelodyneDataStruct::gridClusterFeature_t& grid_cluFeature, float r, float g, float b);
void drawCubeGLRGB(VelodyneDataStruct::pieClusterFeature_t& pie_cluFeature, float r, float g, float b);
void drawLineGLRGB(float x1, float y1, float z1, float x2, float y2, float z2,
                   float r, float g, float b, int wide=1);
void drawTextRGB(float x, float y, float z, float r, float g, float b, char* outputstring);


#endif
