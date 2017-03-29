# ./src/CMakeLists.txt

#########################################################
# FIND GLUT
#########################################################
find_package(GLUT REQUIRED)
#include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
#add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)
#########################################################
# FIND OPENGL
#########################################################
find_package(OpenGL REQUIRED)
#include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
#add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)

add_executable(Velodyne_viewer  VelodyneViewer_main.cpp
    velodyneThread.cpp velodyneScanner.cpp velodyneHeightMap.cpp
    velodyneDriver.cpp velodyneDraw.cpp velodyneDataStruct.cpp
    velodyneAlgoGrid.cpp velodyneAlgoCore.cpp velodyneAlgoPie.cpp  
    configStruct.cpp velodyneNegativeObstacleMap.cpp)

target_link_libraries(Velodyne_viewer ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} pthread lcm glib-2.0)
