
#include <boost/filesystem.hpp>
#include "libsegmap/datasets/segmap_dataset.h"
#include "libsegmap/slam/segmap_grid_map.h"
#include "libsegmap/visualization/segmap_semantic_segmentation_viewer.h"
#include "libsegmap/slam/segmap_preproc.h"
#include "libsegmap/libcarmen_util/util_math.h"
#include "libsegmap/libcarmen_util/command_line.h"
#include "libsegmap/initializations/segmap_args.h"
#include "libsegmap/initializations/segmap_constructors.h"
#include "libsegmap/slam_high_level/segmap_map_builder.h"

using namespace cv;
using namespace std;
using namespace pcl;


int
main(int argc, char **argv)
{
	string log_path;
	string map_path;

	CommandLineArguments args;

	add_default_slam_args(args);
	add_default_sensor_preproc_args(args);
	add_default_mapper_args(args);

	args.parse(argc, argv);

	log_path = args.get<string>("log_path");

	//map_path = args.get<string>("map_path");
	//if (args.get<int>("clean_map") && boost::filesystem::exists(map_path))
	//boost::filesystem::remove_all(map_path);

	NewCarmenDataset *dataset = create_dataset(log_path, args, "graphslam");
	SensorPreproc preproc = create_sensor_preproc(args, dataset, log_path);

	create_map(dataset, preproc, args);

	printf("Done.\n");
	return 0;
}

