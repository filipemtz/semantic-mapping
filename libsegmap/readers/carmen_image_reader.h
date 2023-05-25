
#ifndef __CARMEN_IMAGE_LOADER_H__
#define __CARMEN_IMAGE_LOADER_H__

#include "libsegmap/types/synchronized_data_package.h"
#include <opencv2/opencv.hpp>

class CarmenImageLoader
{
public:
	enum CamSide
	{
		LEFT_CAM,
		RIGHT_CAM,
	};

	CarmenImageLoader(CamSide = RIGHT_CAM);
	~CarmenImageLoader();
	cv::Mat load(DataSample *sample);

protected:

	int _image_size;
	unsigned char *_raw;
	cv::Mat _img;
	CamSide _side;

	void _recreate_image_if_necessary(DataSample *sample);
};

#endif
