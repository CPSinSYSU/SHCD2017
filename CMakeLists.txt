cmake_minimum_required(VERSION 2.8)

project(Velodyne_viewer)

##########################################################
## FIND GLUT
##########################################################
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
#link_directories(${GLUT_LIBRARY_DIRS})
#add_definitions(${GLUT_DEFINITIONS})
if(NOT GLUT_FOUND)
    message(ERROR " GLUT not found!")
endif(NOT GLUT_FOUND)
##########################################################
## FIND OPENGL
##########################################################
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
#link_directories(${OpenGL_LIBRARY_DIRS})
#add_definitions(${OpenGL_DEFINITIONS})
if(NOT OPENGL_FOUND)
    message(ERROR " OPENGL not found!")
endif(NOT OPENGL_FOUND)
#########################################################
# FIND GLIB
#########################################################
find_package(PkgConfig REQUIRED)
pkg_check_modules(GLIB_PKG glib-2.0)
if (NOT GLIB_PKG_FOUND)
    message(ERROR " glib-2.0 not found!")
endif()
include_directories(${GLIB_PKG_INCLUDE_DIRS})


add_subdirectory(src bin)
########################################################
# Linking & stuff
#########################################################
# target_link_libraries(Velodyne_viewer ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )

