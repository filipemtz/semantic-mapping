
#include <cstdio>
#include <cstdlib>
#include <string>
#include "libsegmap/libcarmen_util/util_io.h"
#include "libsegmap/libcarmen_util/util_math.h"
#include "libsegmap/types/lidar_shot.h"
#include "libsegmap/types/segmap_conversions.h"
#include "libsegmap/readers/carmen_lidar_reader.h"

using namespace std;
using namespace pcl;

static const double _velodyne_vertical_angles[32] =
	{
		-30.6700000, -29.3300000, -28.0000000, -26.6700000, -25.3300000, -24.0000000, -22.6700000, -21.3300000,
		-20.0000000, -18.6700000, -17.3300000, -16.0000000, -14.6700000, -13.3300000, -12.0000000, -10.6700000,
		-9.3299999, -8.0000000, -6.6700001, -5.3299999, -4.0000000, -2.6700001, -1.3300000, 0.0000000, 1.3300000,
		2.6700001, 4.0000000, 5.3299999, 6.6700001, 8.0000000, 9.3299999, 10.6700000};

static const int velodyne_ray_order[32] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31};

CarmenLidarLoader::CarmenLidarLoader()
{
	_shot = new LidarShot(_n_vert);

	// auxiliary vectors for reading data from file.
	_raw_ranges = (unsigned short int *)calloc(_n_vert, sizeof(unsigned short int));
	_raw_intensities = (unsigned char *)calloc(_n_vert, sizeof(unsigned char));

	_n_rays = 0;
	_fptr = NULL;
	_n_readings = 0;

	for (int i = 0; i < _n_vert; i++)
		_shot->v_angles[i] = normalize_theta(degrees_to_radians(_velodyne_vertical_angles[i]));
}

CarmenLidarLoader::~CarmenLidarLoader()
{
	delete (_shot);

	if (_fptr)
		fclose(_fptr);
}

// void
// CarmenLidarLoader::_initialize_calibration_table(string lidar_calib_path)
//{
//	// initialize with default values
//	for (int i = 0; i < _n_vert; i++)
//		for (int r = 0; r < _n_distance_indices; r++)
//			for (int j = 0; j < 256; j++)
//				calibration_table[i][r][j] = (unsigned char) j;
//
//	// if file exists, replace the table values with the ones from the file
//	FILE *f = fopen(lidar_calib_path.c_str(), "r");
//
//	if (f != NULL)
//	{
//		while (!feof(f))
//		{
//			int laser_id, intensity, calib, ray_size_index;
//			double sum;
//			long count;
//
//			//fscanf(f, "%d %d %lf %ld %d", &ray, &intensity, &sum, &count, &calib);
//			fscanf(f, "%d %d %lf %ld %d", &laser_id, &ray_size_index, &intensity, &calib, &sum, &count);
//			calibration_table[laser_id][ray_size_index][intensity] = calib;
//		}
//
//		fclose(f);
//
//		fprintf(stderr, "Intensity calibration successfully loaded from '%s.\n", lidar_calib_path.c_str());
//	}
//	else
//		fprintf(stderr, "Warning: intensity calibration file '%s' not found. Using default values.\n",
//		        lidar_calib_path.c_str());
// }

unsigned char
preprocess_lidar_intensity(unsigned char intensity)
{
	// brighten the intensity values
	float intensity_f = (float)intensity;
	intensity_f = intensity_f * 4.0;

	if (intensity_f > 255)
		intensity_f = 255;

	intensity = (unsigned char)intensity_f;

	return intensity;
}

LidarShot *
CarmenLidarLoader::next()
{
	if (_fptr == NULL)
		exit(printf("Error: In CarmenLidarLoader, you should call 'initialize' before calling 'next'.\n"));

	fread(&(_shot->h_angle), sizeof(double), 1, _fptr);
	fread(_raw_ranges, sizeof(unsigned short), _n_vert, _fptr);
	fread(_raw_intensities, sizeof(unsigned char), _n_vert, _fptr);

	// Note: the vertical angles are fixed and initialized in the constructor.
	_shot->h_angle = -normalize_theta(degrees_to_radians(_shot->h_angle));

	for (int i = 0; i < _n_vert; i++)
	{
		_shot->ranges[i] = ((double)_raw_ranges[velodyne_ray_order[i]]) / (double)500.;
		_shot->intensities[i] = preprocess_lidar_intensity(_raw_intensities[velodyne_ray_order[i]]);
	}

	_n_readings++;
	return _shot;
}

bool CarmenLidarLoader::done()
{
	return (_n_readings >= _n_rays);
}

void CarmenLidarLoader::reset()
{
	_n_readings = 0;
	rewind(_fptr);
}

void CarmenLidarLoader::reinitialize(std::string &cloud_path, int n_rays)
{
	if (_fptr)
		fclose(_fptr);

	_n_rays = n_rays;
	_fptr = safe_fopen(cloud_path.c_str(), "rb");
	_n_readings = 0;
}

// void
// load_as_pointcloud(CarmenLidarLoader *loader, PointCloud<PointXYZRGB>::Ptr cloud)
//{
//	int i;
//	double x, y, z;
//	LidarShot *shot;
//
//	cloud->clear();
//
//	while (!loader->done())
//	{
//		shot = loader->next();
//
//		for (i = 0; i < shot->n; i++)
//		{
//			spherical2cartersian(shot->v_angles[i], shot->h_angle, shot->ranges[i], &x, &y, &z);
//
//			PointXYZRGB point;
//
//			point.x = (float) x;
//			point.y = (float) y;
//			point.z = (float) z;
//			point.r = point.g = point.b = (unsigned char) shot->intensities[i];
//
//			cloud->push_back(point);
//		}
//	}
// }
