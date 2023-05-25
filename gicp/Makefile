
all: filter_incorrect_gicps generate_loop_closures_between_logs generate_loop_closures gicp_odometry_manual_correction pf_loop_closures

filter_incorrect_gicps: filter_incorrect_gicps.cpp ../libsegmap/libsegmap.a
	g++ -o filter_incorrect_gicps filter_incorrect_gicps.cpp $(CFLAGS) $(IFLAGS) $(LFLAGS)
generate_loop_closures_between_logs: generate_loop_closures_between_logs.cpp segmap_loop_closures.cpp gicp.cpp ../libsegmap/libsegmap.a
	g++ -o generate_loop_closures_between_logs generate_loop_closures_between_logs.cpp segmap_loop_closures.cpp gicp.cpp $(CFLAGS) $(IFLAGS) $(LFLAGS)
generate_loop_closures: generate_loop_closures.cpp segmap_loop_closures.cpp gicp.cpp ../libsegmap/libsegmap.a
	g++ -o generate_loop_closures generate_loop_closures.cpp segmap_loop_closures.cpp gicp.cpp $(CFLAGS) $(IFLAGS) $(LFLAGS)
gicp_odometry_manual_correction: gicp_odometry_manual_correction.cpp ../libsegmap/libsegmap.a
	g++ -o gicp_odometry_manual_correction gicp_odometry_manual_correction.cpp $(CFLAGS) $(IFLAGS) $(LFLAGS)
pf_loop_closures: pf_loop_closures.cpp segmap_loop_closures.cpp gicp.cpp ../libsegmap/libsegmap.a
	g++ -o pf_loop_closures pf_loop_closures.cpp segmap_loop_closures.cpp gicp.cpp $(CFLAGS) $(IFLAGS) $(LFLAGS)

clean:
	rm -f filter_incorrect_gicps generate_loop_closures_between_logs generate_loop_closures gicp_odometry_manual_correction pf_loop_closures