# Build CarMaker Project on Ubuntu 20.04 using "GCC"
1. Change the file "src/Makefile"
	- Change the include path of CarMaker files
		```shell script
		include /opt/ipg/carmaker/linux64-11.0/include/MakeDefs.linux64
		Here: "linux64-11.0" should be changed to match your CarMaker version!
		```
	- Change the library flag for cpp source files
		```shell script
		default: $(APP_NAME)
			...
			$(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS) $(LD_LIBS_OS) \
					 	=>
			$(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS) $(LDXX_LIBS_OS) \
		```
	- Add include path of extern libraries (e.g. Eigen3, yaml-cpp, OpenCV, ...)
		```shell script
		For example:
			CXXFLAGS += -O2 -I/usr/include/eigen3          
		```
	- Link the extern libraries:
		```shell script
		For example:
			YAML_CPP_LIB = /usr/local/lib/libyaml-cpp.a 
			LD_LIBS = $(YAML_CPP_LIB)
		```
	- Link all object files, which are located in the "src" folder:
		```shell script
		For example:
			OBJS =  CM_Main.o CM_Vehicle.o User.o rsds-client-camera.o \
		                sensorfusion/vehicle_positioning_flow.o \
		```
	- Clean all object files
  		```shell script
		For example:
			clean:
			  -rm -f *~ *% *.o core
			  -rm -f sensorfusion/*.o
		```

2. Compile:
	```shell script
	open a new terminal
	cd masterarbeit_jindong_carmaker/src
	make
	make clean
	```
	```shell script
	For more information about building the carmaker project on linux systems using GCC, please read "CarMaker Programmer’s Guide".
	```


