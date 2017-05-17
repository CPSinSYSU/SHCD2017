/**************************************************************************
 *
 *	全局配置
 *	配置文件 ./velodyne.ini
 *	配置结构体
 *		CfgVeloView: 总体配置项结构体定义
 *			CfgCircleItem: 每个circle的配置选项定义
 *			CfgPlaneDetect: 平面检测配置选项定义
 *			CfgGlobal: 全局配置项定义
 *			CfgShotOffset:
 *			CfgInterSection: 路口算法参数
 *			CfgIntersection: 路口算法参数(新的路口算法)
 *			CfgIgnoreCircleWhenProjection:
 *			CfgCarShapeParameter:
 *	加载参数/保存参数 Get/WritePrivateProfileInt(Window API)
 *
 ***************************************************************************/

#include "configStruct.h"
#include <stdio.h>
#ifdef WIN32
#define _CRT_SECURE_NO_WARNINGS
#include <windows.h>
#include <winbase.h>
#else
#include <glib.h>
#endif

#include <stdlib.h>

static CfgGlobal_t g_DefaultGlobalCfg =
{
};

static CfgPlaneDetect_t g_DefaultCfgPlaneDetect =
{
};

static CfgNegativeObstacle_t g_DefaultCfgNegativeObstacle =
{

};

static CfgIgnoreCircleWhenProjection_t g_DefaultCfgIgnoreCircleWhenProjection =
{
};

static CfgCircleItem_t g_DefaultCfgCircleItems[CIRCLE_NUM] =
{
};

CfgVeloView_t g_CfgVeloView;

/*
 * 从配置文件读入配置到配置结构体
 * @param[out] cfg       要写入的结构体
 * @param[in]  filename  要读入的文件
 * @return     0:成功 其他失败
 */
int LoadCfgVeloView(CfgVeloView_t& cfg, const char* filename)
{
    // 获得Global 配置文件
    char buf[64];
    char defaultVal[64];
#ifdef WIN32
    /*
     * Use Window API: GetPrivateProfileInt; GetPrivateProfileString...
     */
    /******************************************************** 全局配置参数 ********************************************************/
    cfg.cfgGlobal.GroundZ =
        GetPrivateProfileInt("CfgGlobal", "GroundZ", g_DefaultGlobalCfg.GroundZ, filename);

    cfg.cfgGlobal.ThresholdMinHollow =
        GetPrivateProfileInt("CfgGlobal", "ThresholdMinHollow", g_DefaultGlobalCfg.ThresholdMinHollow, filename);
    cfg.cfgGlobal.ThresholdMaxHollow =
        GetPrivateProfileInt("CfgGlobal", "ThresholdMaxHollow", g_DefaultGlobalCfg.ThresholdMaxHollow, filename);

    cfg.cfgGlobal.ShotCalcFrom =
        GetPrivateProfileInt("CfgGlobal", "ShotCalcFrom", g_DefaultGlobalCfg.ShotCalcFrom, filename);
    cfg.cfgGlobal.ShotCalcNum =
        GetPrivateProfileInt("CfgGlobal", "ShotCalcNum", g_DefaultGlobalCfg.ShotCalcNum, filename);
    cfg.cfgGlobal.ShotShowFrom =
        GetPrivateProfileInt("CfgGlobal", "ShotShowFrom", g_DefaultGlobalCfg.ShotShowFrom, filename);
    cfg.cfgGlobal.ShotShowNum =
        GetPrivateProfileInt("CfgGlobal", "ShotShowNum", g_DefaultGlobalCfg.ShotShowNum, filename);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultGlobalCfg.ThresholdMinRad);
    GetPrivateProfileString("CfgGlobal", "ThresholdMinRad", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgGlobal.ThresholdMinRad = (float)atof(buf);

    cfg.cfgGlobal.ContinusLineMinDistInCircle =
        GetPrivateProfileInt("CfgGlobal", "ContinusLineMinDistInCircle", g_DefaultGlobalCfg.ContinusLineMinDistInCircle, filename);

    /******************************************************** 车辆模型 ********************************************************/
    cfg.cfgGlobal.CarLength =
        GetPrivateProfileInt("CfgGlobal", "CarLength", g_DefaultGlobalCfg.CarLength, filename);
    cfg.cfgGlobal.CarWidth =
        GetPrivateProfileInt("CfgGlobal", "CarWidth", g_DefaultGlobalCfg.CarWidth, filename);
    cfg.cfgGlobal.CarHeight =
        GetPrivateProfileInt("CfgGlobal", "CarHeight", g_DefaultGlobalCfg.CarHeight, filename);


    /******************************************************** 平面检测配置参数 ********************************************************/
    cfg.cfgPlaneDetect.GridCellSize =
        GetPrivateProfileInt("CfgPlaneDetect", "GridCellSize", g_DefaultCfgPlaneDetect.GridCellSize, filename);
    cfg.cfgPlaneDetect.GridHeadRowNum =
        GetPrivateProfileInt("CfgPlaneDetect", "GridHeadRowNum", g_DefaultCfgPlaneDetect.GridHeadRowNum, filename);
    cfg.cfgPlaneDetect.GridBackRowNum =
        GetPrivateProfileInt("CfgPlaneDetect", "GridBackRowNum", g_DefaultCfgPlaneDetect.GridBackRowNum, filename);
    cfg.cfgPlaneDetect.GridColumnNum =
        GetPrivateProfileInt("CfgPlaneDetect", "GridColumnNum", g_DefaultCfgPlaneDetect.GridColumnNum, filename);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgPlaneDetect.GridThresholdGroundDetect);
    GetPrivateProfileString("CfgPlaneDetect", "GridThresholdGroundDetect", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgPlaneDetect.GridThresholdGroundDetect = (float)atof(buf);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgPlaneDetect.GridThresholdDiffZ);
    GetPrivateProfileString("CfgPlaneDetect", "GridThresholdDiffZ", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgPlaneDetect.GridThresholdDiffZ = (float)atof(buf);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgPlaneDetect.GridColorFactor);
    GetPrivateProfileString("CfgPlaneDetect", "GridColorFactor", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgPlaneDetect.GridColorFactor = (float)atof(buf);

    cfg.cfgPlaneDetect.GridColorMode =
        GetPrivateProfileInt("CfgPlaneDetect", "GridColorMode", g_DefaultCfgPlaneDetect.GridColorMode, filename);


    cfg.cfgPlaneDetect.PieCellSize =
        GetPrivateProfileInt("CfgPlaneDetect", "PieCellSize", g_DefaultCfgPlaneDetect.PieCellSize, filename);
    cfg.cfgPlaneDetect.PieAzimuthNum =
        GetPrivateProfileInt("CfgPlaneDetect", "PieAzimuthNum", g_DefaultCfgPlaneDetect.PieAzimuthNum, filename);
    cfg.cfgPlaneDetect.PieMinRad =
        GetPrivateProfileInt("CfgPlaneDetect", "PieMinRad", g_DefaultCfgPlaneDetect.PieMinRad, filename);
    cfg.cfgPlaneDetect.PieMaxRad =
        GetPrivateProfileInt("CfgPlaneDetect", "PieMaxRad", g_DefaultCfgPlaneDetect.PieMaxRad, filename);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgPlaneDetect.PieThresholdGroundDetect);
    GetPrivateProfileString("CfgPlaneDetect", "PieThresholdGroundDetect", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgPlaneDetect.PieThresholdGroundDetect = (float)atof(buf);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgPlaneDetect.PieThresholdDiffZ);
    GetPrivateProfileString("CfgPlaneDetect", "PieThresholdDiffZ", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgPlaneDetect.PieThresholdDiffZ = (float)atof(buf);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgPlaneDetect.PieColorFactor);
    GetPrivateProfileString("CfgPlaneDetect", "PieColorFactor", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgPlaneDetect.PieColorFactor = (float)atof(buf);

    cfg.cfgPlaneDetect.PieColorMode =
        GetPrivateProfileInt("CfgPlaneDetect", "PieColorMode", g_DefaultCfgPlaneDetect.PieColorMode, filename);

    /******************************************************** 负障碍检测配置参数 ********************************************************/
    cfg.cfgNegativeObstacle.NegativeObstacleCellSize =
        GetPrivateProfileInt("CfgNegativeObstacle", "NegativeObstacleCellSize", g_DefaultCfgNegativeObstacle.NegativeObstacleCellSize, filename);
    cfg.cfgNegativeObstacle.NegativeObstacleGridDim =
        GetPrivateProfileInt("CfgNegativeObstacle", "NegativeObstacleGridDim", g_DefaultCfgNegativeObstacle.NegativeObstacleGridDim, filename);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgNegativeObstacle.NegativeObstacleThresholdStartEdge);
    GetPrivateProfileString("CfgNegativeObstacle", "NegativeObstacleThresholdStartEdge", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgNegativeObstacle.NegativeObstacleThresholdStartEdge = (float)atof(buf);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgNegativeObstacle.NegativeObstacleThresholdEndEdge);
    GetPrivateProfileString("CfgNegativeObstacle", "NegativeObstacleThresholdEndEdge", defaultVal, buf, sizeof(buf), filename);
    cfg.cfgNegativeObstacle.NegativeObstacleThresholdEndEdge = (float)atof(buf);

    cfg.cfgNegativeObstacle.NegativeObstacleThresholdEdgeNumber =
        GetPrivateProfileInt("CfgNegativeObstacle", "NegativeObstacleThresholdEdgeNumber", g_DefaultCfgNegativeObstacle.NegativeObstacleThresholdEdgeNumber, filename);

    /******************************************************** 每个Circle的配置 ********************************************************/
    char bufCircle[64];
    for (unsigned circle = 0; circle<CIRCLE_NUM; ++circle)
    {
        _snprintf(bufCircle, sizeof(bufCircle), "CfgCircleItems%02d", circle);
        cfg.cfgCircleItems[circle].Enable =
            GetPrivateProfileInt(bufCircle, "Enable", g_DefaultCfgCircleItems[circle].Enable, filename);

        cfg.cfgCircleItems[circle].ThresholdMinDist =
            GetPrivateProfileInt(bufCircle, "ThresholdMinDist", g_DefaultCfgCircleItems[circle].ThresholdMinDist, filename);

        _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgCircleItems[circle].ThresholdRoadEdge);
        GetPrivateProfileString(bufCircle, "ThresholdRoadEdge", defaultVal, buf, sizeof(buf), filename);
        cfg.cfgCircleItems[circle].ThresholdRoadEdge = (float)atof(buf);

        _snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgCircleItems[circle].ThresholdRoadEdgeByDeltaZ);
        GetPrivateProfileString(bufCircle, "ThresholdRoadEdgeByDeltaZ", defaultVal, buf, sizeof(buf), filename);
        cfg.cfgCircleItems[circle].ThresholdRoadEdgeByDeltaZ = (float)atof(buf);

    }

    for (unsigned circle = 0; circle<32; circle++)
    {
        _snprintf(bufCircle, sizeof(bufCircle), "IgnoreCircleWhenProjection%02d", circle);
        cfg.cfgIgnoreCircleWhenProjection.IgnoreCircle[circle] =
            GetPrivateProfileInt("cfgIgnoreCircleWhenProjection", bufCircle, g_DefaultCfgIgnoreCircleWhenProjection.IgnoreCircle[circle], filename);
    }

#else
    /*
     * Use Glib API: g_key_file...
     */
    // get Gkeyfile
    GKeyFile* glib_keyfile;
    glib_keyfile = g_key_file_new();

    GError **error;
    error = NULL;
    if (!g_key_file_load_from_file(glib_keyfile, filename, G_KEY_FILE_NONE, NULL))
    {
        fprintf (stderr, "Could not read config file\n");
        return EXIT_FAILURE;
    }
    /*
     * edit by sean
     *  glib function:
     *      gint g_key_file_set_integer (GKeyFile *key_file, const gchar *group_name, const gchar *key, gint value);
     * added by durant35:
     *  no default value, may be achieve it by error process.
     */
    /******************************************************** 全局配置参数 ********************************************************/
    cfg.cfgGlobal.GroundZ =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "GroundZ", error);

    cfg.cfgGlobal.ThresholdMinHollow =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "ThresholdMinHollow", error);
    cfg.cfgGlobal.ThresholdMaxHollow =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "ThresholdMaxHollow", error);

    cfg.cfgGlobal.ShotCalcFrom =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "ShotCalcFrom", error);
    cfg.cfgGlobal.ShotCalcNum =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "ShotCalcNum", error);
    cfg.cfgGlobal.ShotShowFrom =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "ShotShowFrom", error);
    cfg.cfgGlobal.ShotShowNum =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "ShotShowNum", error);

    cfg.cfgGlobal.ThresholdMinRad =
        g_key_file_get_double(glib_keyfile, "CfgGlobal", "ThresholdMinRad", error);
    cfg.cfgGlobal.ContinusLineMinDistInCircle =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "ContinusLineMinDistInCircle", error);
    /******************************************************** 车辆模型 ********************************************************/
    cfg.cfgGlobal.CarLength =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "CarLength", error);
    cfg.cfgGlobal.CarWidth =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "CarWidth", error);
    cfg.cfgGlobal.CarHeight =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "CarHeight", error);

    /****** Laser distacne filter ******/
    cfg.cfgGlobal.MinLaserDistance =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "MinLaserDistance", error);
    cfg.cfgGlobal.MaxLaserDistacne =
        g_key_file_get_integer(glib_keyfile, "CfgGlobal", "MaxLaserDistacne", error);


    /******************************************************** 平面检测配置参数 ********************************************************/
    cfg.cfgPlaneDetect.GridCellSize =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "GridCellSize", error);
    cfg.cfgPlaneDetect.GridHeadRowNum =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "GridHeadRowNum", error);
    cfg.cfgPlaneDetect.GridBackRowNum =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "GridBackRowNum", error);
    cfg.cfgPlaneDetect.GridColumnNum =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "GridColumnNum", error);
    cfg.cfgPlaneDetect.GridThresholdGroundDetect =
        g_key_file_get_double(glib_keyfile, "CfgPlaneDetect", "GridThresholdGroundDetect", error);
    cfg.cfgPlaneDetect.GridThresholdDiffZ =
        g_key_file_get_double(glib_keyfile, "CfgPlaneDetect", "GridThresholdDiffZ", error);
    cfg.cfgPlaneDetect.GridColorFactor =
        g_key_file_get_double(glib_keyfile, "CfgPlaneDetect", "GridColorFactor", error);
    cfg.cfgPlaneDetect.GridColorMode =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "GridColorMode", error);

    cfg.cfgPlaneDetect.PieRadSize =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "PieRadSize", error);
    cfg.cfgPlaneDetect.PieShotFrom =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "PieShotFrom", error);
    cfg.cfgPlaneDetect.PieShotEnd =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "PieShotEnd", error);
    cfg.cfgPlaneDetect.PieAzimuthNum =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "PieAzimuthNum", error);
    cfg.cfgPlaneDetect.PieMinRad =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "PieMinRad", error);
    cfg.cfgPlaneDetect.PieMaxRad =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "PieMaxRad", error);
    cfg.cfgPlaneDetect.PieThresholdGroundDetect =
        g_key_file_get_double(glib_keyfile, "CfgPlaneDetect", "PieThresholdGroundDetect", error);
    cfg.cfgPlaneDetect.PieThresholdDiffZ =
        g_key_file_get_double(glib_keyfile, "CfgPlaneDetect", "PieThresholdDiffZ", error);
    cfg.cfgPlaneDetect.PieColorFactor =
        g_key_file_get_double(glib_keyfile, "CfgPlaneDetect", "PieColorFactor", error);
    cfg.cfgPlaneDetect.PieColorMode =
        g_key_file_get_integer(glib_keyfile, "CfgPlaneDetect", "PieColorMode", error);

    /******************************************************** 负障碍检测配置参数 ********************************************************/
    cfg.cfgNegativeObstacle.NegativeObstacleCellSize =
        g_key_file_get_integer(glib_keyfile, "CfgNegativeObstacle", "NegativeObstacleCellSize", error);
    cfg.cfgNegativeObstacle.NegativeObstacleGridDim =
        g_key_file_get_integer(glib_keyfile, "CfgNegativeObstacle", "NegativeObstacleGridDim", error);

    cfg.cfgNegativeObstacle.NegativeObstacleThresholdStartEdge =
        g_key_file_get_double(glib_keyfile, "CfgNegativeObstacle", "NegativeObstacleThresholdStartEdge", error);
    cfg.cfgNegativeObstacle.NegativeObstacleThresholdEndEdge =
        g_key_file_get_double(glib_keyfile, "CfgNegativeObstacle", "NegativeObstacleThresholdEndEdge", error);

    cfg.cfgNegativeObstacle.NegativeObstacleThresholdEdgeNumber =
        g_key_file_get_integer(glib_keyfile, "CfgNegativeObstacle", "NegativeObstacleThresholdEdgeNumber", error);


    /******************************************************** 每个Circle的配置 ********************************************************/
    char bufCircle[64];
    for (unsigned circle = 0; circle<CIRCLE_NUM; ++circle)
    {
        snprintf(bufCircle, sizeof(bufCircle), "CfgCircleItems%02d", circle);

        cfg.cfgCircleItems[circle].Enable =
            g_key_file_get_integer(glib_keyfile, bufCircle, "Enable", error);
        cfg.cfgCircleItems[circle].ThresholdMinDist =
            g_key_file_get_integer(glib_keyfile, bufCircle, "ThresholdMinDist", error);

        //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgCircleItems[circle].ThresholdRoadEdge);
        //GetPrivateProfileString(bufCircle, "ThresholdRoadEdge", defaultVal, buf, sizeof(buf), filename);
        cfg.cfgCircleItems[circle].ThresholdRoadEdge =
            g_key_file_get_double(glib_keyfile, bufCircle, "ThresholdRoadEdge", error);
        //_snprintf(defaultVal, sizeof(defaultVal), "%f", g_DefaultCfgCircleItems[circle].ThresholdRoadEdgeByDeltaZ);
        //GetPrivateProfileString(bufCircle, "ThresholdRoadEdgeByDeltaZ", defaultVal, buf, sizeof(buf), filename);
        cfg.cfgCircleItems[circle].ThresholdRoadEdgeByDeltaZ =
            g_key_file_get_double(glib_keyfile, bufCircle, "ThresholdRoadEdgeByDeltaZ", error);
    }
    for (unsigned circle = 0; circle<32; circle++)
    {
        snprintf(bufCircle, sizeof(bufCircle), "IgnoreCircleWhenProjection%02d", circle);
        cfg.cfgIgnoreCircleWhenProjection.IgnoreCircle[circle] =
            g_key_file_get_integer(glib_keyfile, "cfgIgnoreCircleWhenProjection", bufCircle, error);
    }

    g_key_file_free (glib_keyfile);
#endif // WIN32

    return 0;
}
/*
 *    把配置结构体写入到配置文件
 *    @param[in]  cfg       要读取的结构体
 *    @param[out] filename  要写入的文件
 *    @return     0:成功 其他失败
 */
int SaveCfgVeloView(CfgVeloView_t& cfg, const char* filename)
{
#ifdef win32
    char defaultVal[64];

    g_key_file_set_double(filename, "CfgGlobal", "ShotCalcFrom", cfg.cfgGlobal.ShotCalcFrom);

    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.ShotCalcNum);
    g_key_file_set_string(filename, "CfgGlobal", "ShotCalcNum", defaultVal);

    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.GroundZ);
    g_key_file_set_string(filename, "CfgGlobal", "GroundZ", defaultVal);

    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.ThresholdMinHollow);
    //WritePrivateProfileString("CfgGlobal", "ThresholdMinHollow", defaultVal, filename);
    g_key_file_set_string(filename, "CfgGlobal", "ThresholdMinHollow", defaultVal);

    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.ThresholdMaxHollow);
    //WritePrivateProfileString("CfgGlobal", "ThresholdMaxHollow", defaultVal, filename);
    g_key_file_set_string(filename, "CfgGlobal", "ThresholdMaxHollow" defaultVal);

    _snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgGlobal.ThresholdMinRad);
    //WritePrivateProfileString("CfgGlobal", "ThresholdMinRad", defaultVal, filename);
    g_key_file_set_string(filename, "CfgGlobal", "ThresholdMinRad", defaultVal);

    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgGlobal.ContinusLineMinDistInCircle);
    //WritePrivateProfileString("CfgGlobal", "ContinusLineMinDistInCircle", defaultVal, filename);
    g_key_file_set_string(filename, "CfgGlobal", "ContinusLineMinDistInCircle", defaultVal);


    /******************************************************** 平面检测配置参数 ********************************************************/
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.PieCellSize);
    //WritePrivateProfileString("CfgPlaneDetect", "PieCellSize", defaultVal, filename);
    g_key_file_set_string(filename, "CfgPlaneDetect", "PieCellSize", defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.PieAzimuthNum);
    //WritePrivateProfileString("CfgPlaneDetect", "PieAzimuthNum", defaultVal, filename);
    g_key_file_set_string(filename, "CfgPlaneDetect", "PieAzimuthNum", defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.PieMinRad);
    //WritePrivateProfileString("CfgPlaneDetect", "PieMinRad", defaultVal, filename);
    g_key_file_set_string(filename, "CfgPlaneDetect", "PieMinRad", defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.PieMaxRad);
    //WritePrivateProfileString("CfgPlaneDetect", "PieMaxRad", defaultVal, filename);
    g_key_file_set_string(filename, "CfgPlaneDetect", "PieMaxRad", defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgPlaneDetect.PieThresholdGroundDetect);
    //WritePrivateProfileString("CfgPlaneDetect", "PieThresholdGroundDetect", defaultVal, filename);
    g_key_file_set_string(filename,"CfgPlaneDetect", "PieThresholdGroundDetect",defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgPlaneDetect.PieColorFactor);
    //WritePrivateProfileString("CfgPlaneDetect", "PieColorFactor", defaultVal, filename);
    g_key_file_set_string(filename,"CfgPlaneDetect", "PieColorFactor",defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgPlaneDetect.PieColorMode);
    //WritePrivateProfileString("CfgPlaneDetect", "PieColorMode", defaultVal, filename);
    g_key_file_set_string(filename,"CfgPlaneDetect", "PieColorMode",defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.GridCellSize);
    WritePrivateProfileString("CfgPlaneDetect", "GridCellSize", defaultVal, filename);
    g_key_file_set_string(filename,"CfgPlaneDetect", "PieColorMode",defaultVal);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.GridColumnNum);
    WritePrivateProfileString("CfgPlaneDetect", "GridColumnNum", defaultVal, filename);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.GridHeadRowNum);
    WritePrivateProfileString("CfgPlaneDetect", "GridHeadRowNum", defaultVal, filename);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.GridBackRowNum);
    WritePrivateProfileString("CfgPlaneDetect", "GridBackRowNum", defaultVal, filename);
    _snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgPlaneDetect.GridThresholdGroundDetect);
    WritePrivateProfileString("CfgPlaneDetect", "GridThresholdGroundDetect", defaultVal, filename);
    _snprintf(defaultVal, sizeof(defaultVal), "%f", cfg.cfgPlaneDetect.GridColorFactor);
    WritePrivateProfileString("CfgPlaneDetect", "GridColorFactor", defaultVal, filename);
    _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgPlaneDetect.GridColorMode);
    WritePrivateProfileString("CfgPlaneDetect", "GridColorMode", defaultVal, filename);

    /******************************************************** 每个Circle的配置 ********************************************************/
    char bufCircle[64];
    for (unsigned circle = 0; circle<CIRCLE_NUM; ++circle)
    {
        _snprintf(bufCircle, sizeof(bufCircle), "CfgCircleItems%02d", circle);

    }

    for (unsigned circle = 0; circle<32; ++circle)
    {
        _snprintf(bufCircle, sizeof(bufCircle), "IgnoreCircleWhenProjection%02d", circle);

        _snprintf(defaultVal, sizeof(defaultVal), "%d", cfg.cfgIgnoreCircleWhenProjection.IgnoreCircle[circle]);
        WritePrivateProfileString("cfgIgnoreCircleWhenProjection", bufCircle, defaultVal, filename);
    }

#else
    // get Gkeyfile
    GKeyFile* viewer_ini = g_key_file_new();
    GError **error = NULL;
    if (!g_key_file_load_from_file(viewer_ini, "velodyne.ini",
                                G_KEY_FILE_NONE, error)){
        fprintf (stderr, "Could not read config file\n");
        return EXIT_FAILURE;
    }

    g_key_file_set_integer(viewer_ini, "CfgGlobal", "GroundZ", cfg.cfgGlobal.GroundZ);
    g_key_file_set_double(viewer_ini, "CfgGlobal", "ThresholdMinRad", cfg.cfgGlobal.ThresholdMinRad);

#endif // win32
    return 0;
}
