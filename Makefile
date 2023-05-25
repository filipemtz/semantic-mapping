
# Configuration variables
CFLAGS = -O2 -Wall -Wextra -Wno-sign-compare -Wno-unused-result -Wno-format
IFLAGS = -I /usr/local/include/ 

IFLAGS +=  -I $(shell pwd)
LFLAGS = -L $(shell pwd)/libsegmap -lsegmap

#eigen 
IFLAGS += -I /usr/include/eigen3/ 

# opencv
IFLAGS += `pkg-config --cflags opencv4`
LFLAGS += -L /usr/lib/x86_64-linux-gnu/ `pkg-config --libs opencv4`

# g2o
LFLAGS += -L/usr/local/lib -lcholmod -lg2o_core -lg2o_types_slam2d -lg2o_solver_cholmod -lg2o_stuff 

# vtk
IFLAGS +=  -I /usr/include/vtk-9.1
LFLAGS += -lvtkCommonCore-9.1 -lvtkFiltersCore-9.1 -lvtkRenderingCore-9.1 -lvtkCommonDataModel-9.1 -lvtkCommonMath-9.1

# pcl
IFLAGS += -I /usr/local/include/pcl-1.13/
LFLAGS += -L /usr/local/lib/ -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search \
		-lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization 

# boost
LFLAGS += -lboost_system -lboost_filesystem -lboost_program_options

# Export configuration variables to sub-makefiles
export IFLAGS CFLAGS LFLAGS

all:
	make -C libsegmap 
	make -C graphslam_fast 
	make -C gicp 
	make -C programs 
clean: 
	make clean -C libsegmap 
	make clean -C gicp 
	make clean -C graphslam_fast 
	make clean -C programs 

