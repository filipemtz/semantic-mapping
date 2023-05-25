
#ifndef __SEGMAP_CONSTRUCTORS_H__
#define __SEGMAP_CONSTRUCTORS_H__

#include <string>
#include "libsegmap/libcarmen_util/command_line.h"
#include "libsegmap/slam/segmap_preproc.h"
#include "libsegmap/slam/segmap_grid_map.h"
#include "libsegmap/slam/segmap_particle_filter.h"
#include "libsegmap/datasets/segmap_dataset.h"

std::string poses_path_from_pose_mode(std::string mode, std::string log_path);
GridMap create_grid_map(CommandLineArguments &args, int save_map);
ParticleFilter create_particle_filter(CommandLineArguments &args);
NewCarmenDataset* create_dataset(std::string log_path, CommandLineArguments &args, std::string pose_mode);
SensorPreproc create_sensor_preproc(CommandLineArguments &args,
																		NewCarmenDataset *dataset,
																		std::string log_path,
																		std::string overcome_imode = "");

#endif
