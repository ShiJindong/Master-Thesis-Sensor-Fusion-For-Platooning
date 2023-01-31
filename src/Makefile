#******************************************************************************
#**  CarMaker - Version 11.0
#**  Vehicle Dynamics Simulation Toolkit
#**
#**  Copyright (C)   IPG Automotive GmbH
#**                  Bannwaldallee 60             Phone  +49.721.98520.0
#**                  76185 Karlsruhe              Fax    +49.721.98520.99
#**                  Germany                      WWW    www.ipg-automotive.com
#******************************************************************************
#**
#**  Application Makefile
#**  Run with 'make V=1 ...' for a verbose build.
#**
#******************************************************************************

include /opt/ipg/carmaker/linux64-11.0/include/MakeDefs.linux64

APP_VER =		"Car_Generic <insert.your.version.no>"
APP_NAME =		CarMaker.$(ARCH)$(EXE_EXT)

#OPT_CFLAGS =		-g -O1

# select compiler
# CC = gcc

# add include path of extern libraries
CXXFLAGS += -O2 -I/usr/include/eigen3 -I/usr/local/include/yaml-cpp -I/usr/local/include/opencv4
# add include path of headers
# CXXFLAGS += -Isensorfusion/headers

# link the extern libraries (Eigen3 does not need to be linked, bacause it does not have any source files)
YAML_CPP_LIB = /usr/local/lib/libyaml-cpp.a
OPENCV4_LIB = /usr/local/lib/libopencv_aruco.so \
			  /usr/local/lib/libopencv_calib3d.so \
			  /usr/local/lib/libopencv_core.so \
			  /usr/local/lib/libopencv_highgui.so \
			  /usr/local/lib/libopencv_imgcodecs.so \

LD_LIBS =	$(CAR_LIB) \
			$(CARMAKER_LIB) $(DRIVER_LIB) $(ROAD_LIB) $(TIRE_LIB) \
			$(YAML_CPP_LIB) $(OPENCV4_LIB)
			
OBJS =  CM_Main.o CM_Vehicle.o User.o rsds-client-camera.o \
		sensorfusion/vehicle_positioning_flow.o \
		sensorfusion/rel_pose_estimator.o \
		sensorfusion/LidarDetectionAndTracking.o \
		sensorfusion/velocity_estimator.o \
		sensorfusion/sideslip_angle_estimator.o \
		sensorfusion/wheel_slip_detector.o \
		sensorfusion/kalman_filter.o \
		sensorfusion/data_buff.o \
		sensorfusion/data_type.o \
		sensorfusion/CvOperationsWrapper.o \
		sensorfusion/CvMarkerOperationsClass.o


# Prepend local include/library directory to include path:
# PREINC_CFLAGS +=		-I../include -I../lib/$(ARCH) -I../lib

# Append local include/library directory to include path:
# INC_CFLAGS +=		-I../include -I../lib/$(ARCH) -I../lib


### Linking with RTW-built Simulink models

#MATSUPP_MATVER =	R2019a
#LD_LIBS +=		$(MATSUPP_LIB)


# @@PLUGIN-BEGIN-LIBS@@ - Automatically generated code - don't edit!
# @@PLUGIN-END@@

### END (Linking with RTW-built Simulink models)


default:	$(APP_NAME)
$(APP_NAME):	$(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS_MK) app_tmp.o
	$(QECHO) " LD     $@"
	$Q $(CC) $(CFLAGS) $(LDFLAGS) -o $@ \
		$(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS) $(LDXX_LIBS_OS) \
		app_tmp.o
	$(SET_EXE_PERMISSIONS) $@


install: $(APP_NAME)
	$(INSTALL_APP) $(APP_NAME) $(ARCH)

clean:
	-rm -f 	*~ *% *.o core
	-rm -f sensorfusion/*.o

app_tmp.c: Makefile $(OBJS_$(ARCH)) $(OBJS) $(LD_LIBS_MK)
	$(QECHO) " MK     $@"
	$Q $(CREATE_INFO_CMD)

depend .depend: Makefile
	$(QECHO) " MK     $@"
	@echo -n "" >.depend
ifneq ($(wildcard *.c),)
	$Q-$(CC)  $(CFLAGS)   $(DEPCFLAGS)   *.c   >>.depend 2>/dev/null
endif
ifneq ($(wildcard *.cpp),)
	$Q-$(CXX) $(CXXFLAGS) $(DEPCXXFLAGS) *.cpp >>.depend 2>/dev/null
endif
include .depend
