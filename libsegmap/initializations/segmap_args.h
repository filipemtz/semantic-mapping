
#ifndef __SEGMAP_MAPPER_ARGS_H__
#define __SEGMAP_MAPPER_ARGS_H__

#include <string>
#include "libsegmap/slam/segmap_preproc.h"
#include "libsegmap/slam/segmap_grid_map.h"
#include "libsegmap/slam/segmap_particle_filter.h"
#include "libsegmap/libcarmen_util/command_line.h"

SensorPreproc::IntensityMode parse_intensity_mode(std::string intensity_mode);
GridMapTile::MapType parse_map_type(std::string map_type);

std::string default_data_dir();

void add_default_slam_args(CommandLineArguments &args);
void add_default_mapper_args(CommandLineArguments &args);
void add_default_localizer_args(CommandLineArguments &args);
void add_default_sensor_preproc_args(CommandLineArguments &args);

#endif
