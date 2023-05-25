
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "libsegmap/types/segmap_pose2d.h"
#include "libsegmap/readers/carmen_lidar_reader.h"
#include "libsegmap/readers/carmen_image_reader.h"
#include "libsegmap/readers/carmen_semantic_segmentation_reader.h"
#include "libsegmap/types/synchronized_data_package.h"
#include "libsegmap/types/segmap_conversions.h"
#include "libsegmap/types/segmap_definitions.h"
#include "libsegmap/motion_models/ackerman_motion_model.h"
#include "libsegmap/visualization/segmap_semantic_segmentation_viewer.h"

#include "libsegmap/libcarmen_util/util_io.h"
#include "segmap_preproc.h"

using namespace cv;
using namespace pcl;
using namespace std;
using namespace Eigen;


// TODO: include prob_map.h to prevent copies of code
float***
load_calibration_table(const char *calibration_file)
{
	FILE *calibration_file_bin = fopen(calibration_file, "r");
	
	float ***table;

	table = (float ***) calloc(32, sizeof(float **));

	for (int i = 0; i < 32; i++)
	{
		table[i] = (float **) calloc(10, sizeof(float *));

		for (int j = 0; j < 10; j++)
		{
			table[i][j] = (float *) calloc(256, sizeof(float));
		}
	}

	// if the file calibration file does not exist, we initialize the calibration table with default values.
	if (!calibration_file_bin)
	{
		fprintf(stderr, "Calibration file '%s' not found. Returning default values.\n", calibration_file);
		
		for (int i = 0; i < 32; i++)
			for (int j = 0; j < 10; j++)
				for (int k = 0; k < 256; k++)
					table[i][j][k] = ((float) k / (float) 256.);
					
		return table;
	}

	int laser, ray_size, intensity;
	long accumulated_intennsity, count;
	float val, max_val = 0.0, min_val = 255.0;

	while (fscanf(calibration_file_bin, "%d %d %d %f %ld %ld", &laser, &ray_size, &intensity, &val, &accumulated_intennsity, &count) == 6)
	{
		table[laser][ray_size][intensity] = val;
		if (val > max_val)
			max_val = val;
		if (val < min_val)
			min_val = val;
	}

	for (int i = 0; i < 32; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			for (int k = 0; k < 256; k++)
			{
				val = table[i][j][k];
				val = (val - min_val) / (max_val - min_val);
				if (val > 1.0)
					val = 1.0;
				if (val < 0.0)
					val = 0.0;
				table[i][j][k] = val;
			}
		}
	}
	fclose(calibration_file_bin);

	return (table);
}


int
get_distance_index(double distance)
{
	// Gera um indice para cada faixa de distancias. As faixas crescem de ~50% a cada intervalo. Indices de zero a 9 permitem distancias de zero a ~70 metros
	if (distance < 3.5)
		return (0);

	int distance_index = (int) ((log(distance - 3.5 + 1.0) / log(1.45)) +  0.5);

	if (distance_index > 9)
		return (9);

	return (distance_index);
}


void
load_calibration_table_tf(const char *calib_file_path, float calibration_table_tf[32][256])
{
	FILE *fptr = safe_fopen(calib_file_path, "r");

	int nr, nc;

	fscanf(fptr, "%d %d", &nr, &nc);

	for (int i = 0; i < nr; i++)
	{
		for (int j = 0; j < nc; j++)
		{
			fscanf(fptr, "%f", &(calibration_table_tf[i][j]));
			//printf("%.2f ", calibration_table_tf[i][j]);
		}
		//printf("\n");
	}

	fclose(fptr);
}


SensorPreproc::SensorPreproc(CarmenLidarLoader *vloader,
                             CarmenImageLoader *iloader,
                             SemanticSegmentationLoader *sloader,
                             Matrix<double, 4, 4> vel2cam,
                             Matrix<double, 4, 4> vel2car,
                             Matrix<double, 3, 4> projection,
                             Matrix<double, 4, 4> xsens2car,
                             int use_xsens,
                             IntensityMode intensity_mode,
                             std::string intensity_calib_path,
                             double ignore_above_threshold,
                             double ignore_below_threshold)
{
	_vloader = vloader;
	_iloader = iloader;
	_sloader = sloader;
	_vel2cam = vel2cam;
	_vel2car = vel2car;
	_projection = projection;
	_xsens2car = xsens2car;
	_use_xsens = use_xsens;
	_intensity_mode = intensity_mode;
	_n_lidar_shots = 0;
	_vel2car_inverse = _vel2car.inverse();

	_ignore_above_threshold = ignore_above_threshold;
	_ignore_below_threshold = ignore_below_threshold;

	_p_sensor(3, 0) = 1.0;
	_p_car(3, 0) = 1.0;
	_p_world(3, 0) = 1.0;

	//_initialize_calibration_table(lidar_calib_path);
	calibration_table = load_calibration_table(intensity_calib_path.c_str());
	//load_calibration_table_tf("poly_calib_table.txt", calibration_table_tf);

	_lane_mark_detection_active = 0;
	_use_semantic_remapping = 0;

	_load_img = (_intensity_mode == COLOUR);
	_load_semantic_img = (_intensity_mode == SEMANTIC);
}


SensorPreproc::~SensorPreproc()
{
	delete(_vloader);
	delete(_iloader);
	delete(_sloader);

	for (int i = 0; i < 32; i++)
	{
		for (int r = 0; r < 10; r++)
				free(calibration_table[i][r]);

		free(calibration_table[i]);
	}

	free(calibration_table);
}


void
SensorPreproc::reinitialize(DataSample *sample)
{
	_vloader->reinitialize(sample->velodyne_path, sample->n_laser_shots);

	if (_intensity_mode == COLOUR || _load_img)
	{
		_img = read_img(sample);
		_img_with_points = _img.clone();
	}

	if (_intensity_mode == SEMANTIC || _load_semantic_img)
	{
		_semantic_img = read_segmented_img(sample);
		_semantic_img_with_points = segmented_image_view(_semantic_img);
	}

	_compute_transform_car2world(sample);
	_n_lidar_shots = sample->n_laser_shots;

	Pose2d step(0, 0, 0);
	// correction applied for each shot
	ackerman_motion_model(step, sample->v, sample->phi, TIME_SPENT_IN_EACH_SCAN);
	_motion_correction_step = Pose2d::to_matrix(step);
	_motion_correction = Pose2d::to_matrix(Pose2d(0, 0, 0));
}


vector<pcl::PointXYZRGB>
SensorPreproc::_next_points(SensorReference ref)
{
	int visible_by_cam;
	double h, v, r, in;
	LidarShot *shot;
	PointXYZRGB point;
	vector<pcl::PointXYZRGB> points;

	if (_vloader->done())
		return vector<pcl::PointXYZRGB>();

	_corrected_car2world = _car2world * _motion_correction;

	shot = _vloader->next();

	for (int i = 0; i < shot->n; i++)
	{
		h = shot->h_angle;
		v = shot->v_angles[i];
		r = shot->ranges[i];
		in = shot->intensities[i];

		// this test is performed first to prevent additional calculations as soon as possible.
		if (!_spherical_point_is_valid(h, v, r))
			continue;

		_compute_point_in_different_references(h, v, r, &_p_sensor, &_p_car, &_p_world);

		if (!_point3d_is_valid(_p_sensor, _p_car, _p_world, DBL_MAX, -DBL_MAX))
			continue;

		int ray_is_too_high = _p_sensor(2, 0) > _ignore_above_threshold;
		int ray_is_too_low = _p_sensor(2, 0) < _ignore_below_threshold;

		int point_height_is_invalid = (ray_is_too_high || ray_is_too_low);

		point = _create_point_and_intensity(_p_sensor, _p_car, _p_world, in, &visible_by_cam, ref, i);

		if ((_intensity_mode == REFLECTIVITY && !point_height_is_invalid) || visible_by_cam)
			points.push_back(point);
	}

	_motion_correction *= _motion_correction_step;

	return points;
}


std::vector<pcl::PointXYZRGB>
SensorPreproc::next_points_in_sensor()
{
	return _next_points(SENSOR_REFERENCE);
}


std::vector<pcl::PointXYZRGB>
SensorPreproc::next_points_in_car()
{
	return _next_points(CAR_REFERENCE);
}


std::vector<pcl::PointXYZRGB>
SensorPreproc::next_points_in_world()
{
	return _next_points(WORLD_REFERENCE);
}


// how to prevent this method of being a copy of _next_points?
std::vector<SensorPreproc::CompletePointData>
SensorPreproc::next_points_with_full_information()
{
	LidarShot *shot;
	std::vector<SensorPreproc::CompletePointData> points;

	if (_vloader->done())
		return points;

	shot = _vloader->next();
	_corrected_car2world = _car2world * _motion_correction;

	for (int i = 0; i < shot->n; i++)
	{
		CompletePointData p;

		p.laser_id = i;
		p.h_angle = shot->h_angle;
		p.v_angle = shot->v_angles[i];
		p.range = shot->ranges[i];
		p.raw_intensity = shot->intensities[i];
		p.valid = 0;

		_compute_point_in_different_references(p.h_angle, p.v_angle, p.range,
		                                       &_p_sensor, &_p_car, &_p_world);

		_point_coords_from_mat(_p_car, &p.car);
		_point_coords_from_mat(_p_sensor, &p.sensor);
		_point_coords_from_mat(_p_world, &p.world);

		if (_spherical_point_is_valid(p.h_angle, p.v_angle, p.range)
				&& _point3d_is_valid(_p_sensor, _p_car, _p_world, DBL_MAX, -DBL_MAX))
		{
			int ray_is_too_high = _p_sensor(2, 0) > _ignore_above_threshold;
			int ray_is_too_low = _p_sensor(2, 0) < _ignore_below_threshold;

			// GAMBIARRA
			if (ray_is_too_high || ray_is_too_low)
				p.valid = 2;
			else
				p.valid = 1;

			//_adjust_intensity(&p.sensor, _p_sensor, p.raw_intensity, &valid, i);
			_adjust_intensity(_p_sensor, p.raw_intensity, &p.visible_by_cam, i,
			                  &p.calibrated_intensity, &p.colour, &p.semantic_class);
		}

		points.push_back(p);
	}

	_motion_correction *= _motion_correction_step;

	return points;
}


int
SensorPreproc::size()
{
	return _n_lidar_shots;
}


Mat
SensorPreproc::read_segmented_img(DataSample *sample)
{
	Mat seg_img = _sloader->load(sample);

	if (_lane_mark_detection_active)
		_segment_lane_marks(seg_img, sample);

	return seg_img;
}


void
SensorPreproc::_segment_lane_marks(Mat &m, DataSample *sample)
{
	double r, g, b, gray;

	int st = (int) (_img.rows * ((double) 450. / 960.));
	int end = (int) (_img.rows * ((double) 750. / 960.));

	Mat img = read_img(sample);

	for (int i = st; i < end; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			b = img.data[3 * (i * img.cols + j)];
			g = img.data[3 * (i * img.cols + j) + 1];
			r = img.data[3 * (i * img.cols + j) + 2];
			gray = (b + g + r) / 3;

			if (gray > 40 && m.data[3 * (i * m.cols + j)] != 19)
			{
				m.data[3 * (i * m.cols + j)] = 20;
				m.data[3 * (i * m.cols + j) + 1] = 20;
				m.data[3 * (i * m.cols + j) + 2] = 20;
			}
		}
	}
}



void
SensorPreproc::_compute_transform_car2world(DataSample *sample)
{
	double roll, pitch, yaw;

	yaw = 0.0;
	pitch = 0.0;
	roll = 0.0;

	if (_use_xsens)
	{
		// convert xsens data to roll, pitch, yaw
		_r3x3 = sample->xsens.toRotationMatrix();
		_r3x3 = _move_xsens_to_car(_r3x3);
		getEulerYPR(_r3x3, yaw, pitch, roll);
	}

	_car2world = pose6d_to_matrix((sample->pose.x),
	                              (sample->pose.y),
	                              0., roll, pitch,
	                              sample->pose.th);
}


void
SensorPreproc::_compute_point_in_different_references(double h_angle, double v_angle, double range,
                                                      Matrix<double, 4, 1> *p_sensor,
                                                      Matrix<double, 4, 1> *p_car,
                                                      Matrix<double, 4, 1> *p_world)
{
	double x, y, z;

	spherical2cartersian(v_angle, h_angle, range, &x, &y, &z);

	(*p_sensor)(0, 0) = x;
	(*p_sensor)(1, 0) = y;
	(*p_sensor)(2, 0) = z;
	(*p_sensor)(3, 0) = 1;

	(*p_car) = _vel2car * (*p_sensor);
	(*p_world) = _corrected_car2world * (*p_car);

	(*p_car)(0, 0) /= (*p_car)(3, 0);
	(*p_car)(1, 0) /= (*p_car)(3, 0);
	(*p_car)(2, 0) /= (*p_car)(3, 0);
	(*p_car)(3, 0) = 1.0;

	(*p_world)(0, 0) /= (*p_world)(3, 0);
	(*p_world)(1, 0) /= (*p_world)(3, 0);
	(*p_world)(2, 0) /= (*p_world)(3, 0);
	(*p_world)(3, 0) = 1.0;
}


int
SensorPreproc::_spherical_point_is_valid(double h_angle, double v_angle, double range)
{
	// the angles are not used now, but they can be used in the future.
	(void) h_angle;
	(void) v_angle;

	int ray_is_range_max = (range >= 70.0) || (range < 1.0);

	if (ray_is_range_max)
		return 0;

	return 1;
}


Matrix<double, 3, 3>
SensorPreproc::_move_xsens_to_car(Matrix<double, 3, 3> xsens)
{
	// create a 4x4 transformation matrix from xsens 3x3 rotation matrix
	_r4x4(0, 0) = xsens(0, 0);
	_r4x4(0, 1) = xsens(0, 1);
	_r4x4(0, 2) = xsens(0, 2);
	_r4x4(0, 3) = 0;
	_r4x4(1, 0) = xsens(1, 0);
	_r4x4(1, 1) = xsens(1, 1);
	_r4x4(1, 2) = xsens(1, 2);
	_r4x4(1, 3) = 0;
	_r4x4(2, 0) = xsens(2, 0);
	_r4x4(2, 1) = xsens(2, 1);
	_r4x4(2, 2) = xsens(2, 2);
	_r4x4(2, 3) = 0;
	_r4x4(3, 0) = 0;
	_r4x4(3, 1) = 0;
	_r4x4(3, 2) = 0;
	_r4x4(3, 3) = 1;

	// move xsens to car
	_r4x4 = _xsens2car * _r4x4;

	// extract the 3x3 rotation from back the 4x4 transformation matrix.
	_r3x3(0, 0) = _r4x4(0, 0);
	_r3x3(0, 1) = _r4x4(0, 1);
	_r3x3(0, 2) = _r4x4(0, 2);
	_r3x3(1, 0) = _r4x4(1, 0);
	_r3x3(1, 1) = _r4x4(1, 1);
	_r3x3(1, 2) = _r4x4(1, 2);
	_r3x3(2, 0) = _r4x4(2, 0);
	_r3x3(2, 1) = _r4x4(2, 1);
	_r3x3(2, 2) = _r4x4(2, 2);

	return _r3x3;
}


int
SensorPreproc::_point3d_is_valid(Matrix<double, 4, 1> &p_sensor,
                                 Matrix<double, 4, 1> &p_car,
                                 Matrix<double, 4, 1> &p_world,
                                 double ignore_above_threshold,
                                 double ignore_below_threshold)
{
	int ray_hit_car = 0;
	double safe_border_x = 1.0;
	double safe_border_y = 1.0;

	// test if the ray is inside the car area in x-axis direction.
	// the conditions evaluate if the ray is between rear and rear axis, and between rear axis and front.
	int x_test = (p_car(0, 0) > -(distance_between_rear_car_and_rear_wheels + safe_border_x))
								&& (p_car(0, 0) < (car_length - distance_between_rear_car_and_rear_wheels + safe_border_x));

	// test if the ray is inside the car area in y-axis direction.
	int y_test = (p_car(1, 0) > -(car_width / 2. + safe_border_y))
								&& (p_car(1, 0) < (car_width / 2. + safe_border_y));

	if (x_test && y_test)
		ray_hit_car = 1;

	int ray_contains_nan = std::isnan(p_world(0, 0)) || std::isnan(p_world(1, 0)) || std::isnan(p_world(2, 0));
	int ray_contains_inf = std::isinf(p_world(0, 0)) || std::isinf(p_world(1, 0)) || std::isinf(p_world(2, 0));
	int ray_is_too_high = p_sensor(2, 0) > ignore_above_threshold;
	int ray_is_too_low = p_sensor(2, 0) < ignore_below_threshold;

	if (ray_hit_car
			|| ray_contains_nan
			|| ray_contains_inf
			|| ray_is_too_high
			|| ray_is_too_low
	)
		return 0;

	return 1;
}


void
SensorPreproc::_point_coords_from_mat(Eigen::Matrix<double, 4, 1> &mat, PointXYZRGB *point)
{
	point->x = mat(0, 0) / mat(3, 0);
	point->y = mat(1, 0) / mat(3, 0);
	point->z = mat(2, 0) / mat(3, 0);
}


unsigned char
SensorPreproc::_get_calibrated_intensity(unsigned char raw_intensity, Matrix<double, 4, 1> &p_sensor, int laser_id)
{
	unsigned char intensity = raw_intensity;

	double ray_size_in_the_floor = sqrt(pow(p_sensor(0, 0), 2) + pow(p_sensor(1, 0), 2));
	int ray_distance_index = get_distance_index(ray_size_in_the_floor);
	intensity = (unsigned char) (255 * calibration_table[laser_id][ray_distance_index][intensity]);

	return intensity;
}


unsigned char
SensorPreproc::_get_calibrated_intensity_tf(unsigned char raw_intensity, Matrix<double, 4, 1> &p_sensor, int laser_id)
{
	float c = (255.0 * calibration_table_tf[laser_id][raw_intensity]);

	if (c < 0) c = 0;
	if (c > 255) c = 255;

	return (unsigned char) c;
}


void
SensorPreproc::_adjust_intensity(Matrix<double, 4, 1> &p_sensor, unsigned char raw_intensity,
                                 int *visible_by_cam, int laser_id, unsigned char *calibrated_intensity,
                                 Scalar *colour, int *point_class)
{
	// in INTENSITY mode, the point color is given by the intensity observed by the lidar.
	//if (_intensity_mode == REFLECTIVITY)
	//{
	*calibrated_intensity = _get_calibrated_intensity(raw_intensity, p_sensor, laser_id);

		//unsigned char intensity = _get_calibrated_intensity_tf(raw_intensity, p_sensor, laser_id);
		//point->r = point->g = point->b = intensity;
		//*visible_by_cam = 1;
	//}

	// In the SEMANTIC and VISUAL modes, the point color
	// is obtained by projecting the points in the image,
	// and returning the respective pixel color.
	//else
	*visible_by_cam = 0;
	colour->val[0] = 0;
	colour->val[1] = 0;
	colour->val[2] = 0;
	*point_class = 19; // invalid class (add an enum)

	cv::Point pos_pixel;
	int p = -1;

	if (_intensity_mode == COLOUR || _load_img)
	{
		// just to prevent dying when the image is not present in the computer.
		if (_img.rows > 0)
		{
			_get_pixel_position(p_sensor, _img.rows, _img.cols, &pos_pixel, visible_by_cam);

			if (*visible_by_cam)
			{
				p = 3 * (pos_pixel.y * _img.cols + pos_pixel.x);

				colour->val[0] = _img.data[p];
				colour->val[1] = _img.data[p + 1];
				colour->val[2] = _img.data[p + 2];

				circle(_img_with_points, Point(pos_pixel.x, pos_pixel.y), 2, Scalar(0, 0, 255), -1);
			}
		}
	}

	if (_intensity_mode == SEMANTIC || _load_semantic_img)
	{
		// just to prevent dying when the image is not present in the computer.
		if (_semantic_img.rows > 0)
		{
			_get_pixel_position(p_sensor, _semantic_img.rows, _semantic_img.cols, &pos_pixel, visible_by_cam);

			if (*visible_by_cam)
			{
				p = 3 * (pos_pixel.y * _semantic_img.cols + pos_pixel.x);

				*point_class = CityscapesObjectClassMapper::transform_object_class(_semantic_img.data[p]);

				circle(_semantic_img_with_points, Point(pos_pixel.x, pos_pixel.y), 2, Scalar(0, 0, 255), -1);
			}
		}
	}
}


PointXYZRGB
SensorPreproc::_create_point_and_intensity(Matrix<double, 4, 1> &p_sensor,
                                           Matrix<double, 4, 1> &p_car,
                                           Matrix<double, 4, 1> &p_world,
                                           unsigned char intensity,
                                           int *visible_by_cam,
                                           SensorReference ref,
                                           int laser_id)
{
	unsigned char calibrated_intensity;
	Scalar colour;
	int point_class;

	PointXYZRGB point;

	if (ref == SENSOR_REFERENCE)
		_point_coords_from_mat(p_sensor, &point);
	else if (ref == CAR_REFERENCE)
		_point_coords_from_mat(p_car, &point);
	else
		_point_coords_from_mat(p_world, &point);

	//_adjust_intensity(&point, p_sensor, intensity, valid, laser_id);
	_adjust_intensity(p_sensor, intensity, visible_by_cam, laser_id,
	                  &calibrated_intensity, &colour, &point_class);

	if (_intensity_mode == REFLECTIVITY)
		point.r = point.g = point.b = calibrated_intensity;
	else if (_intensity_mode == SEMANTIC)
		point.r = point.g = point.b = point_class;
	else
	{
		point.r = colour[2];
		point.g = colour[1];
		point.b = colour[0];
	}

	return point;
}


unsigned char
SensorPreproc::_brighten(unsigned char val, unsigned int multiplier)
{
	unsigned int brightened = val * multiplier;

	if (brightened > 255)
		return 255;
	else
		return brightened;
}


///*
void
SensorPreproc::_get_pixel_position(Matrix<double, 4, 1> &p_sensor,
                                   int img_rows, int img_cols, cv::Point *ppixel,
                                   int *is_valid)
{
	*is_valid = 0;

	_p_cam = _vel2cam * p_sensor;

	// as the camera coordinate system is rotated, the z coordinate points forward.
	double point_position_forward = _p_cam(2, 0) / _p_cam(3, 0);
	double point_position_rightwards = _p_cam(0, 0) / _p_cam(3, 0);

	// test to check if the point is in front of the camera.
	// points behind the camera can also be projected into the image plan.
	if (point_position_forward > 0)
	{
		_p_pixel_homogeneous = _projection * _p_cam;

		ppixel->y = (_p_pixel_homogeneous(1, 0) / _p_pixel_homogeneous(2, 0)) * img_rows;
		ppixel->x = (_p_pixel_homogeneous(0, 0) / _p_pixel_homogeneous(2, 0)) * img_cols;

		// Because of the way the sensors are mounted, the first 5 velodyne lasers hit the
		// floor, but when projected to the image plane, they intercept the car. The car
		// occludes the region the lasers hit when looking from the camera position.
		int point_hit_the_car = (point_position_forward < 5.5) && (point_position_rightwards < 3.0 && point_position_rightwards > -2.5);

		// Check if the point is visible by the camera and do not intercept the car.
		if (ppixel->x >= 0 && ppixel->x < img_cols && ppixel->y >= 0 && ppixel->y < img_rows && !point_hit_the_car)
			*is_valid = 1;
	}
}
//*/

/*
void
SensorPreproc::_get_pixel_position(Matrix<double, 4, 1> &p_sensor,
                                   int img_rows, int img_cols, cv::Point *ppixel,
                                   int *is_valid)
{
	Matrix<double, 3, 4> projection;
	Matrix<double, 4, 4> cam_wrt_velodyne, vel2cam;
	Matrix<double, 3, 1> hpixel;
	Matrix<double, 4, 1> point, pcam;

	projection <<
			489.439, 0, 323.471, 0,
			0, 489.437, 237.031, 0,
			0, 0, 1, 0
			;

	double x = 0.04;
	double y = 0.115;
	double z = -0.27;

	double roll = -0.052360;
	double pitch = -0.034907;
	double yaw = 0.008727;

	cam_wrt_velodyne = pose6d_to_matrix(x, y, z,
	                                    -M_PI/2 + roll,
	                                    0 + pitch,
	                                    -M_PI/2 + yaw);

	vel2cam = cam_wrt_velodyne.inverse();
	point = p_sensor;

	pcam = vel2cam * point;
	hpixel = projection * pcam;
	cv::Point pixel((int) hpixel(0, 0) / hpixel(2, 0), (int) hpixel(1, 0) / hpixel(2, 0));

	*ppixel = pixel;

	// check if the point is visible by the camera.
	if (ppixel->x >= 0 && ppixel->x < img_cols && ppixel->y >= 0 && ppixel->y < img_rows)
		*is_valid = 1;
	else
		*is_valid = 0;
}
*/

void
load_as_pointcloud(SensorPreproc &preproc,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                   SensorPreproc::SensorReference ref)
{
	vector<PointXYZRGB> points;
	cloud->clear();

	for (int i = 0; i < preproc.size(); i++)
	{
		if (ref == SensorPreproc::SENSOR_REFERENCE)
			points = preproc.next_points_in_sensor();
		else if (ref == SensorPreproc::CAR_REFERENCE)
			points = preproc.next_points_in_car();
		else
			points = preproc.next_points_in_world();

		for (int j = 0; j < points.size(); j++)
			cloud->push_back(points[j]);
	}
}


pcl::PointXYZRGB
transform_point(Eigen::Matrix<double, 4, 4> &t, pcl::PointXYZRGB &p_in)
{
	Eigen::Matrix<double, 4, 1> p_in_mat, p_out_mat;
	pcl::PointXYZRGB p_out;

	p_in_mat(0, 0) = p_in.x;
	p_in_mat(1, 0) = p_in.y;
	p_in_mat(2, 0) = p_in.z;
	p_in_mat(3, 0) = 1.0;

	p_out_mat = t * p_in_mat;

	p_out.x = p_out_mat(0, 0) / p_out_mat(3, 0);
	p_out.y = p_out_mat(1, 0) / p_out_mat(3, 0);
	p_out.z = p_out_mat(2, 0) / p_out_mat(3, 0);

	p_out.r = p_in.r;
	p_out.g = p_in.g;
	p_out.b = p_in.b;

	return p_out;
}


pcl::PointXYZRGB
transform_point(Pose2d &t, pcl::PointXYZRGB &p_in)
{
	pcl::PointXYZRGB p_out;

	double c = cos(t.th);
	double s = sin(t.th);

	p_out.x = p_in.x * c + p_in.y * (-s) + t.x;
	p_out.y = p_in.x * s + p_in.y * c + t.y;

	p_out.z = p_in.z;
	p_out.r = p_in.r;
	p_out.g = p_in.g;
	p_out.b = p_in.b;

	return p_out;
}


