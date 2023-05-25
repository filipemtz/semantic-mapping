
#ifndef __SEGMAP_OCCUPANCY_CELL_H__
#define __SEGMAP_OCCUPANCY_CELL_H__

#include <cstdio>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include "libsegmap/slam/hash_map/distributions/segmap_bernoulli.h"
#include "libsegmap/slam/hash_map/cells/segmap_cell_interface.h"


class OccupancyCell : public CellInterface
{
public:
	Bernoulli statistics;

	virtual void add(const pcl::PointXYZRGB &point);
	virtual double log_likelihood(const pcl::PointXYZRGB &point);
	virtual cv::Scalar get_color();
	virtual void write(FILE *fptr);
	virtual void read(FILE *fptr);
};


#endif
