
#ifndef __GICP_H__
#define __GICP_H__

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "libsegmap/datasets/segmap_dataset.h"
#include "libsegmap/slam/segmap_preproc.h"
#include "libsegmap/libcarmen_util/command_line.h"


void
run_gicp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr source,
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
         Eigen::Matrix<double, 4, 4> *correction,
         int *converged,
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
         double leaf_size=0.);


void
add_default_gicp_args(CommandLineArguments &args);


#endif

