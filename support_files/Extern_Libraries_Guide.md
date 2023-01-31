# Install and Use Extern Libraries for this CarMaker Project on Ubuntu 20.04:

## 1. [Eigen >= 3.3.7](https://eigen.tuxfamily.org/index.php?title=Main_Page)
### install
```shell script
sudo apt install libeigen3-dev
```
Your Eigen3-Library should be installed now in path: 
```shell script
/usr/include/eigen3 
```
or 
```shell script
/usr/local/include/eigen3 
```

### use in CarMaker:
add the include path of Eigen3 library in the file "src/Makefile"
```shell script
CXXFLAGS+=-O2 -I/usr/include/eigen3     			
```
Eigen3 does not need to be linked, bacause it does not have any source files.

## 2. [YAML-CPP-0.7.0](https://github.com/jbeder/yaml-cpp)
### install
```shell script
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build
cd build
cmake ..
make
sudo make install
```
yaml-cpp builds a static library by default, if you want to build a shared library, please use:
```shell script
cmake -DYAML_BUILD_SHARED_LIBS=ON ..
```
Your YAML-CPP Library should be installed now in path: 
```shell script
/usr/local/include/yaml-cpp
```
### use in CarMaker:
add the include path of YAML-CPP library in the file "src/Makefile"
```shell script
CXXFLAGS += -O2 -I/usr/local/include/yaml-cpp     
```
link the library
```shell script
YAML_CPP_LIB = /usr/local/lib/libyaml-cpp.a        	# Here link the static library, you can also use shared library (.so)

LD_LIBS = $(YAML_CPP_LIB)
```

## 3. [OpenCV 4.6.0](https://github.com/opencv/opencv/tree/4.6.0) + [OpenCV_Contrib 4.6.0](https://github.com/opencv/opencv_contrib/releases/tag/4.6.0)

### install (recommended through the source)
- Tutorial [hier](https://vitux.com/opencv_ubuntu/)

### use in CarMaker
```shell script
CXXFLAGS += -O2 -I/usr/local/include/opencv4	   	
OPENCV4_LIB = /usr/local/lib/libopencv_aruco.so \
	      /usr/local/lib/libopencv_calib3d.so \
	      /usr/local/lib/libopencv_core.so \
	      /usr/local/lib/libopencv_highgui.so \
	      /usr/local/lib/libopencv_imgcodecs.so \
              ... (add all libraries of funktions you need)
LD_LIBS = $(OPENCV4_LIB)
```