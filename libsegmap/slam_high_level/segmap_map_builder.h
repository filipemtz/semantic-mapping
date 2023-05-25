
#ifndef SEGMAP_LIBSEGMAP_SLAM_HIGH_LEVEL_SEGMAP_MAP_BUILDER_H_
#define SEGMAP_LIBSEGMAP_SLAM_HIGH_LEVEL_SEGMAP_MAP_BUILDER_H_

#include "libsegmap/datasets/segmap_dataset.h"
#include "libsegmap/types/segmap_colormaps.h"
#include "libsegmap/slam/segmap_preproc.h"
#include "libsegmap/slam/segmap_grid_map.h"
#include "libsegmap/initializations/segmap_args.h"
#include "libsegmap/visualization/segmap_sensor_viewer.h"


void
view_one_map(DataSample *sample, PointCloudViewer &viewer, GridMap *map, int img_width, const char *name);

void
view_maps(DataSample *sample, SensorPreproc &preproc,
          PointCloudViewer &viewer, GridMap *visual_map,
          GridMap *reflectivity_map, GridMap *semantic_map,
          GridMap *occupancy_map, int img_width, int view_point_cloud,
          int view_img_with_points);

void
create_map(NewCarmenDataset *dataset,
					 SensorPreproc &preproc,
					 CommandLineArguments &args);


void
create_map(NewCarmenDataset *dataset,
					 SensorPreproc &preproc,
					 CommandLineArguments &args,
					 const std::string &map_path,
					 const std::vector<int> &sample_to_map);

#endif

