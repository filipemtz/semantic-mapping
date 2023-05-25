
#ifndef __SEGMAP_HEIGHT_CELL_H__
#define __SEGMAP_HEIGHT_CELL_H__

#include <cstdio>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include "libsegmap/slam/hash_map/distributions/segmap_gaussian.h"
#include "libsegmap/slam/hash_map/cells/segmap_cell_interface.h"


class HeightCell : public CellInterface
{
public:
	Gaussian statistics;

	virtual void add(const pcl::PointXYZRGB &point);
	virtual double log_likelihood(const pcl::PointXYZRGB &point);
	virtual cv::Scalar get_color();
	virtual void write(FILE *fptr);
	virtual void read(FILE *fptr);
};



#endif
