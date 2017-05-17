#!/bin/bash

rm -rf build/
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX="$(pwd)/../" ..
# error happened in cmake
if [ $? -gt 0 ]; then
	echo "dependencies miss"
	echo "------- glut ------ "
	echo "<1> sudo apt-get install build-essential" 
	echo "<2> sudo apt-get install libgl1-mesa-dev" 
	echo "<3> sudo apt-get install libglu1-mesa-dev"
	echo "<4> sudo apt-get install freeglut3-dev"
	echo "------- glib2.0 ------ "
	echo "sudo apt-get install libglib2.0-dev"
	exit
fi

make
# error happened in cmake
if [ $? -gt 0 ]; then
	echo "dependencies miss"
	echo "------- glut ------ "
	echo "<1> sudo apt-get install build-essential" 
	echo "<2> sudo apt-get install libgl1-mesa-dev" 
	echo "<3> sudo apt-get install libglu1-mesa-dev"
	echo "<4> sudo apt-get install freeglut3-dev"
	echo "------- glib2.0 ------ "
	echo "sudo apt-get install libglib2.0-dev"
	exit
fi

make install

cd ../bin
