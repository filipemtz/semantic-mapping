
#ifndef __SEGMAP_COLOR_CELL_H__
#define __SEGMAP_COLOR_CELL_H__

#include <cstdio>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include "libsegmap/slam/hash_map/cells/segmap_grayscale_cell.h"
#include "libsegmap/slam/hash_map/cells/segmap_cell_interface.h"


class ColorCell : public CellInterface
{
public:
	GrayscaleCell red_cell;
	GrayscaleCell green_cell;
	GrayscaleCell blue_cell;

	virtual void add(const pcl::PointXYZRGB &point);
	virtual double log_likelihood(const pcl::PointXYZRGB &point);
	virtual cv::Scalar get_color();
	virtual void write(FILE *fptr);
	virtual void read(FILE *fptr);
};


#endif
