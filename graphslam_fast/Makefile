
all: graphslam view_path #joint_graphslam

graphslam: graphslam.cpp ../libsegmap/libsegmap.a
	g++ -o graphslam graphslam.cpp $(CFLAGS) $(IFLAGS) $(LFLAGS)
joint_graphslam: joint_graphslam.cpp ../libsegmap/libsegmap.a
	g++ -o joint_graphslam joint_graphslam.cpp graphslam_util.cpp $(CFLAGS) $(IFLAGS) $(LFLAGS)
view_path: view_path.cpp ../libsegmap/libsegmap.a
	g++ -o view_path view_path.cpp graphslam_util.cpp $(CFLAGS) $(IFLAGS) $(LFLAGS)
clean:
	rm -f graphslam joint_graphslam view_path