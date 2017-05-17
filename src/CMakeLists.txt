# ./src/CMakeLists.txt

add_executable(Velodyne_viewer main.cpp
    velodyneThread.cpp velodyneScanner.cpp velodyneDriver.cpp velodyneDraw.cpp velodyneDataStruct.cpp configStruct.cpp)

target_link_libraries(Velodyne_viewer ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} pthread glib-2.0)

INSTALL(TARGETS Velodyne_viewer
	RUNTIME DESTINATION bin)
