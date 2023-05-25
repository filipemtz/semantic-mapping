
#include <cstdio>
#include <string>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>

#include "libsegmap/datasets/segmap_dataset.h"
#include "libsegmap/readers/carmen_lidar_reader.h"
#include "libsegmap/readers/carmen_image_reader.h"
#include "libsegmap/types/segmap_conversions.h"
#include "libsegmap/visualization/segmap_sensor_viewer.h"
#include "libsegmap/initializations/segmap_constructors.h"
#include "libsegmap/initializations/segmap_args.h"
#include "libsegmap/slam/segmap_preproc.h"
#include "libsegmap/libcarmen_util/command_line.h"

using namespace cv;
using namespace std;
using namespace pcl;
using namespace Eigen;

#define brighten(x) ((unsigned char) ((x * 5 > 255) ? (255) : (x * 5)))


void
print_sample_info(DataSample *data_package)
{
	printf("gps: %lf %lf ", data_package->gps.x, data_package->gps.y);
	printf("odom: %lf %lf ", data_package->v, data_package->phi);
	printf("gps time: %lf ", data_package->gps_time);
	printf("image time: %lf ", data_package->image_time);
	printf("velodyne time: %lf ", data_package->velodyne_time);
	printf("odom time: %lf ", data_package->odom_time);
	printf("xsens time: %lf ", data_package->xsens_time);
	printf("fused odom: %lf %lf %lf\n---\n", data_package->pose.x, data_package->pose.y, data_package->pose.th);
}


CommandLineArguments
parse_command_line_args(int argc, char **argv)
{
	CommandLineArguments args;

	args.add_positional<string>("log", "Path to a log", 1);
	args.add_positional<string>("param_file", "Path to a carmen.ini file", 1);
	add_default_sensor_preproc_args(args);
	args.parse(argc, argv);

	return args;
}


int
main(int argc, char **argv)
{
	Mat img;
	DataSample* data_package;
	PointCloudViewer viewer;
	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);

	CommandLineArguments args = parse_command_line_args(argc, argv);
	string log_path = args.get<string>("log");
	
	NewCarmenDataset *dataset = create_dataset(log_path, args, "graphslam");
	SensorPreproc preproc = create_sensor_preproc(args, dataset, log_path);

	for (int i = 1; i < dataset->size(); i += 1)
	{
		data_package = dataset->at(i);

		// ignore packages when the car is stopped.
		if (fabs(data_package->v) < 1.0)
			continue;

		print_sample_info(data_package);
		preproc.reinitialize(data_package);
		load_as_pointcloud(preproc, cloud, SensorPreproc::WORLD_REFERENCE);
		img = preproc.read_img(data_package);
		viewer.show(img, "img", 640);
		viewer.show(cloud);
		viewer.loop();
	}

	printf("Log is done.\n");
	return 0;
}

