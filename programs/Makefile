
all: localizer mapper velodyne_calibration_save_data 

localizer: localizer.cpp ../libsegmap/libsegmap.a
	g++ -o localizer localizer.cpp $(CFLAGS) $(IFLAGS) $(LFLAGS)
mapper: mapper.cpp ../libsegmap/libsegmap.a
	g++ -o mapper mapper.cpp $(CFLAGS) $(IFLAGS) $(LFLAGS)
velodyne_calibration_save_data: velodyne_calibration_save_data.cpp ../libsegmap/libsegmap.a
	g++ -o velodyne_calibration_save_data velodyne_calibration_save_data.cpp $(CFLAGS) $(IFLAGS) $(LFLAGS)
clean:
	rm -f localizer mapper velodyne_calibration_save_data