
#include <cstdlib>
#include <random>
#include <vector>

#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <pcl/common/transforms.h>

#include "libsegmap/libcarmen_util/util_math.h"
#include "libsegmap/types/segmap_definitions.h"
#include "libsegmap/slam/segmap_particle_filter.h"
#include "libsegmap/slam/segmap_preproc.h"
#include <cassert>
#include "libsegmap/libcarmen_util/util_io.h"
#include "libsegmap/libcarmen_util/util_strings.h"
#include "libsegmap/libcarmen_util/util_time.h"
#include <boost/filesystem.hpp>

using namespace Eigen;
using namespace pcl;
using namespace std;


double
ParticleFilter::_gauss()
{
	return _std_normal_distribution(_random_generator);
}


// public:
ParticleFilter::ParticleFilter(int n_particles, 
								double x_std, double y_std, double th_std,
								double v_std, double phi_std, double pred_x_std, double pred_y_std, double pred_th_std,
								double color_std_r, double color_std_g, double color_std_b,
								double reflectivity_var,
								int ecc_n_bins)
{
	_n = n_particles;

	_p = (Pose2d *) calloc (_n, sizeof(Pose2d));
	_w = (double *) calloc (_n, sizeof(double));

	_p_bef = (Pose2d *) calloc (_n, sizeof(Pose2d));
	_w_bef = (double *) calloc (_n, sizeof(double));

	_x_std = x_std;
	_y_std = y_std;
	_th_std = th_std;
	_v_std = v_std;
	_phi_std = phi_std;

	_pred_x_std = pred_x_std;
	_pred_y_std = pred_y_std;
	_pred_th_std = pred_th_std;

	_color_std_r = color_std_r / 255.0;
	_color_std_g = color_std_g / 255.0;
	_color_std_b = color_std_b / 255.0;

	_reflectivity_std = reflectivity_var / 255.0;
	_color_histogram = (double*) calloc (ecc_n_bins, sizeof(double));
	_map_histogram = (double*) calloc (ecc_n_bins, sizeof(double));
	_joint_histogram = (double*) calloc (ecc_n_bins * ecc_n_bins, sizeof(double));
	_ecc_n_bins = ecc_n_bins;
	_ecc_pixel_step = (int) ((double) 256 / (double) _ecc_n_bins);

	assert(_ecc_pixel_step > 0);

	use_gps_weight = 0;
	use_map_weight = 0;
	use_ecc_weight = 0;

	_rejection_thresh = 1.0;
}


ParticleFilter::~ParticleFilter()
{
	free(_p);
	free(_w);
	free(_p_bef);
	free(_w_bef);
	free(_color_histogram);
	free(_map_histogram);
	free(_joint_histogram);
}


void
ParticleFilter::seed(int val)
{
	srand(val);
	_random_generator.seed(val);
}


void
ParticleFilter::set_use_gps_weight(int use_or_not)
{
	use_gps_weight = use_or_not;
}

void
ParticleFilter::set_use_map_weight(int use_or_not)
{
	use_map_weight = use_or_not;
}


void
ParticleFilter::set_use_ecc_weight(int use_or_not)
{
	use_ecc_weight = use_or_not;
}


void
ParticleFilter::reset(double x, double y, double th)
{
	for (int i = 0; i < _n; i++)
	{
		_p[i].x = x + _gauss() * _x_std;
		_p[i].y = y + _gauss() * _y_std;
		_p[i].th = th + _gauss() * _th_std;

		_w[i] = _w_bef[i] = 1. / (double) _n;
		_p_bef[i] = _p[i];
	}

	best = _p[0];
}


void
ParticleFilter::reset(Pose2d &pose)
{
	reset(pose.x, pose.y, pose.th);
}


void
ParticleFilter::predict(double v, double phi, double dt)
{
	double noisy_v, noisy_phi;

	for (int i = 0; i < _n; i++)
	{
		noisy_v = v;
		noisy_phi = phi;

		noisy_v += _gauss() * _v_std;
		noisy_phi = normalize_theta(noisy_phi + _gauss() * _phi_std);

		_p[i].x += noisy_v * dt * cos(_p[i].th);
		_p[i].y += noisy_v * dt * sin(_p[i].th);
		_p[i].th += noisy_v * dt * tan(noisy_phi) / distance_between_front_and_rear_axles;

		_p[i].x += _gauss() * _pred_x_std;
		_p[i].y += _gauss() * _pred_y_std;
		_p[i].th += _gauss() * _pred_th_std;
		_p[i].th = normalize_theta(_p[i].th);

		_p_bef[i] = _p[i];
		_w_bef[i] = _w[i];
	}

	best = _p[0];
}


double
ParticleFilter::_semantic_point_weight(int class_id, vector<double> &cell)
{
	double count;

	// cell observed at least once.
	// TODO: what to do when the cell was never observed?
	count = cell[cell.size() - 2];
	assert(count != 0);

	// log probability of the observed class.
	return log(cell[class_id] / count); 
}


double
ParticleFilter::_semantic_point_weight(int class_id, double *cell, int n_classes)
{
	double count;

	// cell observed at least once.
	// TODO: what to do when the cell was never observed?
	count = cell[n_classes];
	assert(count != 0);

	// log probability of the observed class.
	return log(cell[class_id] / count); 
}


double
ParticleFilter::_image_point_weight(double r, double g, double b, vector<double> &cell)
{
	return (-pow(((r - cell[2]) / 255.) / _color_std_r, 2))
				 + (-pow(((g - cell[1]) / 255.) / _color_std_g, 2))
				 + (-pow(((b - cell[0]) / 255.) / _color_std_b, 2));
}


double
ParticleFilter::_reflectivity_point_weight(double reflectivity, vector<double> &cell)
{
	return -pow(((reflectivity - cell[0]) / 255.) / _reflectivity_std, 2);
}


double
ParticleFilter::_occupancy_point_weight(double prob_obstacle, vector<double> &cell)
{
	return _occupancy_point_weight(prob_obstacle, cell.data());
}


double
ParticleFilter::_reflectivity_point_weight(double reflectivity, double *cell)
{
	return -pow(((reflectivity - cell[0]) / 255.) / _reflectivity_std, 2);
}


double
ParticleFilter::_image_point_weight(double r, double g, double b, double *cell)
{
	return (-pow(((r - cell[2]) / 255.) / _color_std_r, 2))
				 + (-pow(((g - cell[1]) / 255.) / _color_std_g, 2))
				 + (-pow(((b - cell[0]) / 255.) / _color_std_b, 2));
}


double
ParticleFilter::_occupancy_point_weight(double prob_obstacle, double *cell)
{
	double p = 0.5; 

	if (cell[1] > 0)
	{
		// prob of measuring an obstacle
		p = (cell[0] / cell[1]);
		
		// if the measurement is not an obstacle, we return the opposite prob.
		if (prob_obstacle < 0.5)
			p = 1 - p;
	}

	// just to prevent numerical instabilities.
	if (p == 0) 
		p = 1e-6;

	return log(p);
}


double
ParticleFilter::_semantic_weight(PointCloud<PointXYZRGB>::Ptr transformed_cloud, GridMap &map)
{
	double unnorm_log_prob = 0.;
	PointXYZRGB point;

	assert(transformed_cloud->size() > 0);
	//double den = 1. / (double) transformed_cloud->size();

	for (int i = 0; i < transformed_cloud->size(); i += 1)
	{
		point = transformed_cloud->at(i);
		vector<double> cell = map.read_cell(point);
		unnorm_log_prob += _semantic_point_weight(point.r, cell);
	}

	if (transformed_cloud->size() > 0)
		unnorm_log_prob /= (double) transformed_cloud->size();

	return unnorm_log_prob;
}


double
ParticleFilter::_image_weight(PointCloud<PointXYZRGB>::Ptr transformed_cloud, GridMap &map)
{
	double unnorm_log_prob = 0.;
	PointXYZRGB point;

	for (int i = 0; i < transformed_cloud->size(); i++)
	{
		point = transformed_cloud->at(i);
		vector<double> cell = map.read_cell(point);
		unnorm_log_prob += _image_point_weight(point.r, point.g, point.b, cell);
	}

	if (transformed_cloud->size() > 0)
		unnorm_log_prob /= (double) transformed_cloud->size();

	return unnorm_log_prob;
}


double
ParticleFilter::_reflectivity_weight(PointCloud<PointXYZRGB>::Ptr transformed_cloud, GridMap &map)
{
	double unnorm_log_prob = 0.;
	PointXYZRGB point;

	for (int i = 0; i < transformed_cloud->size(); i++)
	{
		point = transformed_cloud->at(i);
		vector<double> cell = map.read_cell(point);
		unnorm_log_prob += _reflectivity_point_weight(point.r, cell);
	}

	if (transformed_cloud->size() > 0)
		unnorm_log_prob /= (double) transformed_cloud->size();

	return unnorm_log_prob;
}


double
ParticleFilter::_gps_weight(Pose2d &pose, Pose2d &gps)
{
	return exp(-0.5 * (pow((pose.x - gps.x) / _x_std, 2) + pow((pose.y - gps.y) / _y_std, 2)));
}


double
ParticleFilter::_ecc_weight(PointCloud<PointXYZRGB>::Ptr transformed_cloud, GridMap &map)
{
	PointXYZRGB point;
	vector<double> cell;
	int n_valid = 0;

	memset(_color_histogram, 0, _ecc_n_bins * sizeof(double));
	memset(_map_histogram, 0, _ecc_n_bins * sizeof(double));
	memset(_joint_histogram, 0, _ecc_n_bins * _ecc_n_bins * sizeof(double));

	// compute histograms
	for (int i = 0; i < transformed_cloud->size(); i++)
	{
		point = transformed_cloud->at(i);
		cell = map.read_cell(point);

		//if (cell[3] > 1.0)
		{
			int pc = (point.r + point.g + point.b) / 3.0;
			int mc = (cell[0] + cell[1] + cell[2]) / 3.0;

			pc = pc / _ecc_pixel_step;
			mc = mc / _ecc_pixel_step;

			_color_histogram[pc]++;
			_map_histogram[mc]++;
			_joint_histogram[pc * _ecc_n_bins + mc]++;

			n_valid++;

			//printf("i: %d pc: %d mc: %d n_valid: %d\n", i, pc, mc, n_valid);
		}
	}

	assert(n_valid > 0);

	// compute entropies
	double prob_color, prob_map, prob_joint;
	double img_entropy = 0.0;
	double map_entropy = 0.0;
	double joint_entropy = 0.0;

	for (int i = 0; i < _ecc_n_bins; i++)
	{
		prob_color = _color_histogram[i] / n_valid;
		prob_map = _map_histogram[i] / n_valid;

		if (prob_color > 0)
			img_entropy += prob_color * log(prob_color);

		if (prob_map > 0)
			map_entropy += prob_map * log(prob_map);

		//printf("i: %d prob_color: %lf prob_map: %lf img_entropy: %lf map_entropy: %lf\n",
		       //i, prob_color, prob_map, img_entropy, map_entropy);

		for (int j = 0; j < _ecc_n_bins; j++)
		{
			prob_joint = _joint_histogram[i * _ecc_n_bins + j] / n_valid;

			if (prob_joint > 0)
				joint_entropy += prob_joint * log(prob_joint);
		}
	}

	img_entropy = -img_entropy;
	map_entropy = -map_entropy;
	joint_entropy = -joint_entropy;

//	printf("joint_entropy: %lf img_entropy: %lf map_entropy: %lf ecc: %lf\n",
//	       joint_entropy, img_entropy, map_entropy,
//	       2.0 - (2 * joint_entropy) / (img_entropy + map_entropy));

	double sum_img_map_entropies = img_entropy + map_entropy;

	if (sum_img_map_entropies == 0)
		sum_img_map_entropies = 2e-5;

	if (joint_entropy == 0)
		joint_entropy = 1e-5;

	// Entropy Correlation Coefficient (ECC) see "http://ijsetr.org/wp-content/uploads/2016/05/IJSETR-VOL-5-ISSUE-5-1700-1703.pdf"
	return log(2.0 - (2 * joint_entropy) / sum_img_map_entropies);

	//return (img_entropy + map_entropy) / (joint_entropy);
}


void
draw_tile(cv::Mat &img, GridMapTile *tile, int i, int j)
{
	int r_offset = i * tile->_h;
	int c_offset = j * tile->_w;

	for (const int& p : tile->_observed_cells)
	{
		int cell_y = (p / tile->_n_fields_by_cell) / tile->_w;
		int cell_x = (p / tile->_n_fields_by_cell) % tile->_w;

		int y = cell_y + r_offset;
		int x = cell_x + c_offset;

		img.data[y * img.cols + x] = 255;
	}
}


void
view_observed_cells(GridMap &map)
{
	int h = map.height_meters * map.pixels_by_m;
	int w = map.width_meters * map.pixels_by_m;

	cv::Mat img = cv::Mat::zeros(h, w, CV_8UC1);

	for (int i = 0; i < map._N_TILES; i++)
		for (int j = 0; j < map._N_TILES; j++)
			draw_tile(img, map._tiles[i][j], i, j);

	cv::imshow("observed", img);
	cv::waitKey(1);
}


double *
ParticleFilter::_get_cell_value_in_offline_map(int cell_linearized_position_in_inst_map, GridMapTile *tile, GridMap &map,
                                               double sin_particle_th, double cos_particle_th, double particle_x, double particle_y)
{
	// cells' x and y position in the instantaneous map
	int cell_y_tile = (cell_linearized_position_in_inst_map / tile->_n_fields_by_cell) / tile->_w;
	int cell_x_tile = (cell_linearized_position_in_inst_map / tile->_n_fields_by_cell) % tile->_w;

	// cell position in meters assuming the origin is in the car
	// we sum half the cell under the assumption that the cell position is in its center.
	double half_cell = map.m_by_pixels / 2.0;
	double dy = cell_y_tile * tile->_m_by_pixel + tile->_yo + half_cell;
	double dx = cell_x_tile * tile->_m_by_pixel + tile->_xo + half_cell;

	// point position in the world
	double wx = dx * cos_particle_th - dy * sin_particle_th + particle_x;
	double wy = dx * sin_particle_th + dy * cos_particle_th + particle_y;

	//return map.read_cell(wx, wy);
	return map.read_cell_ref(wx, wy);
}


double
ParticleFilter::_weight_between_cells(double *inst_cell, double *off_cell, GridMapTile::MapType map_type, int n_classes)
{
	if (map_type == GridMapTile::TYPE_SEMANTIC)
	{
		return _semantic_point_weight(argmax(inst_cell, n_classes), off_cell, n_classes);
	}
	else if (map_type == GridMapTile::TYPE_VISUAL)
	{
		return _image_point_weight(inst_cell[2], inst_cell[1], inst_cell[0], off_cell);
	}
	else if (map_type == GridMapTile::TYPE_REFLECTIVITY)
	{
		return _reflectivity_point_weight(inst_cell[0], off_cell);
	}
	else if (map_type == GridMapTile::TYPE_OCCUPANCY)
	{
		double prob_obstacle = inst_cell[0] / inst_cell[1];
		return _occupancy_point_weight(prob_obstacle, off_cell);
	}

//	else if (_weight_type == WEIGHT_GPS)
//		_w[i] = _gps_weight(_p[i], gps);
//	else if (_weight_type == WEIGHT_ECC)
//		_w[i] = _ecc_weight(transformed_cloud, map);
	else
		exit(printf("Error: unknown type of particle weighting.\n"));
}


void
ParticleFilter::_reset_histograms()
{
	memset(_color_histogram, 0, _ecc_n_bins * sizeof(double));
	memset(_map_histogram, 0, _ecc_n_bins * sizeof(double));
	memset(_joint_histogram, 0, _ecc_n_bins * _ecc_n_bins * sizeof(double));
	_n_histogram_points = 0;
}


void
ParticleFilter::_update_histogram(double *inst_cell, double *off_cell, GridMapTile::MapType map_type)
{
	int pc, mc;

	if (map_type == GridMapTile::TYPE_VISUAL)
	{
		pc = (inst_cell[0] + inst_cell[1] + inst_cell[2]) / 3.0;
		mc = (off_cell[0] + off_cell[1] + off_cell[2]) / 3.0;
	}
	else if (map_type == GridMapTile::TYPE_REFLECTIVITY)
	{
		pc = inst_cell[0];
		mc = off_cell[0];
	}
	else 
		exit(printf("Error: weighting particles with ECC is only implemented for reflectivity and colour maps.\n"));

	pc = pc / _ecc_pixel_step;
	mc = mc / _ecc_pixel_step;

	_color_histogram[pc]++;
	_map_histogram[mc]++;
	_joint_histogram[pc * _ecc_n_bins + mc]++;

	_n_histogram_points++;
}


double
ParticleFilter::_compute_log_ecc_from_histograms()
{
	// compute entropies
	double prob_color, prob_map, prob_joint;
	double img_entropy = 0.0;
	double map_entropy = 0.0;
	double joint_entropy = 0.0;

	assert(_n_histogram_points > 0);

	for (int i = 0; i < _ecc_n_bins; i++)
	{
		prob_color = _color_histogram[i] / _n_histogram_points;
		prob_map = _map_histogram[i] / _n_histogram_points;

		assert(prob_color >= 0 && prob_color <= 1);
		assert(prob_map >= 0 && prob_map <= 1);

		if (prob_color > 0)
			img_entropy += prob_color * log(prob_color);

		if (prob_map > 0)
			map_entropy += prob_map * log(prob_map);

		for (int j = 0; j < _ecc_n_bins; j++)
		{
			prob_joint = _joint_histogram[i * _ecc_n_bins + j] / _n_histogram_points;

			assert(prob_joint >= 0 && prob_joint <= 1);

			if (prob_joint > 0)
				joint_entropy += prob_joint * log(prob_joint);
		}
	}

	img_entropy = -img_entropy;
	map_entropy = -map_entropy;
	joint_entropy = -joint_entropy;

	double sum_img_map_entropies = img_entropy + map_entropy;

	if (joint_entropy == 0 && (img_entropy != 0 || map_entropy != 0))
		exit(printf("Error: inconsistent entropies: joint entropy: %lf img_entropy: %lf map_entropy: %lf\n", joint_entropy, img_entropy, map_entropy));

	if (sum_img_map_entropies == 0)
		sum_img_map_entropies = 1e-6;

	if (joint_entropy == 0)
		joint_entropy = 1e-6;

	if (joint_entropy == sum_img_map_entropies)
		return 1e-6;

	// Entropy Correlation Coefficient (ECC) see "http://ijsetr.org/wp-content/uploads/2016/05/IJSETR-VOL-5-ISSUE-5-1700-1703.pdf"
	return 2.0 - (2 * joint_entropy) / sum_img_map_entropies;
}


double
ParticleFilter::_compute_particle_weight(GridMap &instantaneous_map, GridMap &map, Pose2d &gps, Pose2d &particle_pose)
{
	double particle_weight = 0.0;

	if (use_gps_weight)
		particle_weight += _gps_weight(particle_pose, gps);

	if (use_ecc_weight || use_map_weight)
	{
		if (use_ecc_weight)
			_reset_histograms();

		// computed once for efficiency
		double sin_th = sin(particle_pose.th);
		double cos_th = cos(particle_pose.th);

		// for each tile
		for (int i = 0; i < map._N_TILES; i++)
		{
			for (int j = 0; j < map._N_TILES; j++)
			{
				GridMapTile *tile = instantaneous_map._tiles[i][j];

				// for each observed cell in the tile
				for (const int& cell_linearized_position_in_inst_map : tile->_observed_cells)
				{
					// cell in instantaneuos map
					double *inst_cell = &(tile->_map[cell_linearized_position_in_inst_map]);

					// cell in offline map
					double *off_cell = _get_cell_value_in_offline_map(cell_linearized_position_in_inst_map,
																			tile, map, sin_th, cos_th,
																			particle_pose.x, particle_pose.y);

					if (off_cell == 0)
						continue;

					if (use_ecc_weight)
						_update_histogram(inst_cell, off_cell, tile->_map_type);

					if (use_map_weight)
						particle_weight += _weight_between_cells(inst_cell, off_cell, tile->_map_type, tile->_color_map.n_classes);
				}
			}
		}
	}

	if (use_ecc_weight)
		particle_weight += _compute_log_ecc_from_histograms();

	return particle_weight;
}


double
ParticleFilter::_compute_ecc_weight(GridMap &instantaneous_map, GridMap &map, Pose2d &particle_pose)
{
	_reset_histograms();

	double sin_th = sin(particle_pose.th);
	double cos_th = cos(particle_pose.th);

	// for each tile
	for (int i = 0; i < map._N_TILES; i++)
	{
		for (int j = 0; j < map._N_TILES; j++)
		{
			GridMapTile *tile = instantaneous_map._tiles[i][j];

			// for each observed cell in the tile
			for (const int& cell_linearized_position_in_inst_map : tile->_observed_cells)
			{
				// cell in instantaneuos map
				double *inst_cell = &(tile->_map[cell_linearized_position_in_inst_map]);

				// cell in offline map
				double *off_cell = _get_cell_value_in_offline_map(cell_linearized_position_in_inst_map,
				                                                  tile, map, sin_th, cos_th,
				                                                  particle_pose.x, particle_pose.y);

				if (off_cell == 0)
					continue;

				_update_histogram(inst_cell, off_cell, tile->_map_type);
			}
		}
	}

	return _compute_log_ecc_from_histograms();
}


double
ParticleFilter::_low_log_likelihood_threshold_by_map_type(GridMapTile::MapType map_type, int num_classes)
{
	if (map_type == GridMapTile::TYPE_OCCUPANCY)
		return log(1.0 / 10.0);
	else if (map_type == GridMapTile::TYPE_REFLECTIVITY)
		return -pow((30. / 255.) / _reflectivity_std, 2); // we consider an error of 30 as an outlier.
	else if (map_type == GridMapTile::TYPE_SEMANTIC)
		return log(1.0 / (10.0 + num_classes)); // a classe nunca foi observada em 10 leituras
	else if (map_type == GridMapTile::TYPE_VISUAL)
		return -3 * pow((30. / 255.) / _reflectivity_std, 2); // we consider an error of 30 as an outlier.

	return 0.0;
}


void
ParticleFilter::_compute_all_particles_weights_with_outlier_rejection(GridMap &instantaneous_map, GridMap &map, Pose2d &gps, double rejection_thresh)
{
	// initialize the particles weights, compute the gps_weight or the ecc_weight if necessary, and store the previous value of the pose.
	for (int i = 0; i < _n; i++)
	{
		_w[i] = 0.0;

		if (use_gps_weight)
			_w[i] += _gps_weight(_p[i], gps);
		else if (use_ecc_weight)
			_w[i] += _compute_ecc_weight(instantaneous_map, map, _p[i]);

		_p_bef[i] = _p[i];
	}

	double particle_probs[_n];
	int n_rejected_rays = 0, n_total_rays = 0;

	// compute map weight with outlier rejection.
	if (use_map_weight)
	{
		// for each tile
		for (int i = 0; i < map._N_TILES; i++)
		{
			for (int j = 0; j < map._N_TILES; j++)
			{
				GridMapTile *tile = instantaneous_map._tiles[i][j];

				// for each observed cell in the tile
				for (const int& cell_linearized_position_in_inst_map : tile->_observed_cells)
				{
					int n_parts_with_low_prob = 0;

					for (int p_id = 0; p_id < _n; p_id++)
					{
						// cell in instantaneuos map
						double *inst_cell = &(tile->_map[cell_linearized_position_in_inst_map]);

						// cell in offline map
						double *off_cell = _get_cell_value_in_offline_map(cell_linearized_position_in_inst_map,
																				tile, map, sin(_p[p_id].th), cos(_p[p_id].th), _p[p_id].x, _p[p_id].y);

						if (off_cell == 0)
							particle_probs[p_id] = 0;
						else
							particle_probs[p_id] = _weight_between_cells(inst_cell, off_cell, tile->_map_type, tile->_color_map.n_classes);

						if (particle_probs[p_id] < _low_log_likelihood_threshold_by_map_type(tile->_map_type, tile->_color_map.n_classes))
							n_parts_with_low_prob++;
					}

					double ratio_low_prob_parts = (double) n_parts_with_low_prob / (double) _n;

					//printf("%d %d %lf %lf\n", n_parts_with_low_prob, _n, ratio_low_prob_parts, rejection_thresh);

					if (ratio_low_prob_parts <= rejection_thresh)
					{
						for (int p_id = 0; p_id < _n; p_id++)
							_w[p_id] += particle_probs[p_id];
					}
					else
						n_rejected_rays++;

					n_total_rays++;
				}
			}
		}

		printf("#rejected rays: %d of %d (%.3lf)\n", n_rejected_rays, n_total_rays, 100.0 * ((double) n_rejected_rays / (double) n_total_rays));
	}
}


cv::Mat
view_point_cloud_as_image(PointCloud<PointXYZRGB>::Ptr transformed_cloud, GridMap &map)
{
	cv::Mat img = cv::Mat::zeros(map.height_meters * map.pixels_by_m, map.width_meters * map.pixels_by_m, CV_8UC3);

	for (int i = 0; i < transformed_cloud->size(); i++)
	{
		PointXYZRGB point = transformed_cloud->at(i);

		int x = (point.x - map.xo) * map.pixels_by_m;
		int y = (point.y - map.yo) * map.pixels_by_m;

		if (x >= 0 && x < img.cols && y >= 0 && y < img.rows)
			img.data[3 * (y * img.cols + x) + 2] = 255;
	}

	return img;
}


string
map_type_to_string(GridMapTile::MapType map_type)
{
	if (map_type == GridMapTile::TYPE_OCCUPANCY)
		return "occupancy";
	else if (map_type == GridMapTile::TYPE_REFLECTIVITY)
		return "reflectivity";
	else if (map_type == GridMapTile::TYPE_SEMANTIC)
		return "semantic";
	else if (map_type == GridMapTile::TYPE_VISUAL)
		return "visual";

	return "unknown";
}


string
get_tiles_dir(GridMap &map)
{
	string s = "/tmp/" + map_type_to_string(map._map_type) + "_";
	string map_dir;

	do
	{
		char name[128];
		sprintf(name, "%d", rand());
		map_dir = s + string(name);
	} while (boost::filesystem::exists(map_dir));
	
	printf("Creating dir for instantaneous map: %s\n", map_dir.c_str());
	printf("result: %d\n", boost::filesystem::create_directory(map_dir));

	return map_dir;
}


void
ParticleFilter::_compute_weights(PointCloud<PointXYZRGB>::Ptr cloud, GridMap &map, Pose2d &gps, int *max_id, int *min_id)
{
	int i;
	// PointCloud<PointXYZRGB>::Ptr transformed_cloud(new PointCloud<PointXYZRGB>);

	if (!use_gps_weight && !use_map_weight && !use_ecc_weight)
		exit(printf("Error: set at least one way of computing the particles' weights.\n"));

	// todo: avoid creating this structure when using only gps for computing the particle weights.
	string tiles_dir = ""; //get_tiles_dir(map);

	if (cloud->size() == 0)
	{
		for (int i = 0; i < _n; i++)
		{
			_w[i] = 1.0 / (double) _n;
			_p_bef[i] = _p[i];
		}
	}
	else
	{
		GridMap instantaneous_map(tiles_dir, map._tile_height_meters,
															map._tile_width_meters, map.m_by_pixels,
									map._map_type, 0);

		if (use_map_weight || use_ecc_weight)
		{
			instantaneous_map.reload(0, 0);
			for (int i = 0; i < cloud->size(); i++)
				instantaneous_map.add_point(cloud->at(i));
		}

		//cv::Mat map_img = instantaneous_map.to_image();
		//cv::imshow("inst_map", map_img);
		//cv::waitKey(1);

		// view_observed_cells(instantaneous_map);

		// #pragma omp parallel for default(none) private(i) shared(cloud, map, _p)
		_compute_all_particles_weights_with_outlier_rejection(instantaneous_map, map, gps, _rejection_thresh);
		/*
		for (int i = 0; i < _n; i++)
		{
			//transformed_cloud->clear();
			//transformPointCloud(*cloud, *transformed_cloud, Pose2d::to_matrix(_p[i]));
			//_w[i] = _image_weight(transformed_cloud, map);
	
			_w[i] = _compute_particle_weight(instantaneous_map, map, gps, _p[i]);
			_p_bef[i] = _p[i];
		}
		*/
	}

	*max_id = *min_id = 0;

	//fprintf(stderr, "\nDEBUG: Unormalized particle weights: ");
	for (i = 0; i < _n; i++)
	{
		assert(!isinf(_w[i]) && !isnan(_w[i]));
		//fprintf(stderr, "%.4lf ", _w[i]);

		if (_w[i] > _w[*max_id])
			*max_id = i;

		if (_w[i] < _w[*min_id])
			*min_id = i;
	}
	//fprintf(stderr, "\n");

	best = _p[*max_id];
}


void
ParticleFilter::_normalize_weights(int min_id, int max_id)
{
	int i;
	double sum_weights, min_weight;
	//double max_weight;

	(void) max_id;
	//(void) max_weight;

	min_weight = _w[min_id];
	//max_weight = _w[max_id];
	sum_weights = 0.;

	//fprintf(stderr, "\nDEBUG: Weights as positive values: ");
	for (i = 0; i < _n; i++)
	{
		//_w[i] = exp(_w[i] - max_weight) + (1. / (double) (3. * _n));
		//_w[i] = exp(_w[i]);
		_w[i] -= min_weight;
		//_w[i] = pow(_w[i], 3);
		sum_weights += _w[i];

		assert(!isinf(_w[i]) && !isnan(_w[i]));
		//fprintf(stderr, "%.4lf ", _w[i]);
	}
	//fprintf(stderr, "\n");

	//transformPointCloud(*cloud, *transformed_cloud, Pose2d::to_matrix(_p[max_id]));
	//for(int i = 0; i < transformed_cloud->size(); i++)
	//{
	//	transformed_cloud->at(i).r = 255;
	//	transformed_cloud->at(i).g = 0;
	//	transformed_cloud->at(i).b = 0;
	//	map.add_point(transformed_cloud->at(i));
	//}
	//printf("Sum weights: %lf\n", sum_weights);

	// normalize the weights
	//fprintf(stderr, "\nDEBUG: Weights Normalized: ");
	for (i = 0; i < _n; i++)
	{
		if (fabs(sum_weights) < 1e-6)
			_w[i] = 1. / (double) _n;
		else
			_w[i] /= sum_weights;

		assert(_w[i] >= 0 && !isinf(_w[i]) && !isnan(_w[i]));
		_w_bef[i] = _w[i];

		//fprintf(stderr, "%.4lf ", _w[i]);
	}
	//fprintf(stderr, "\n\n---------\n\n");
}


void
ParticleFilter::_resample()
{
	int i;

	//_p[0] = best; // elitism
	int pos = rand() % _n;
	double step = 1. / (double) _n;
	double sum = 0.;

	//fprintf(stderr, "step: %lf pos: %d Particles: \n", step, pos);
	for (i = 0; i < _n; i++)
	{
		sum += _w[pos];
		while (sum < step)
		{
			pos = (pos + 1) % _n;
			sum += _w[pos];
		}

		//fprintf(stderr, "%d ", pos);
		_p[i] = _p_bef[pos];
		sum = 0.;
		pos = (pos + 1) % _n;
	}
	//fprintf(stderr, "\n");

	// make all particles equally likely after resampling
	for (i = 0; i < _n; i++)
		_w[i] = 1. / (double) _n;
}


void
ParticleFilter::_compute_weights(DataSample *sample, GridMap &map, SensorPreproc &preproc, int *max_id, int *min_id)
{
	int i;

	if (!use_gps_weight && !use_map_weight && !use_ecc_weight)
		exit(printf("Error: set at least one way of computing the particles' weights.\n"));

	// todo: avoid creating this structure when using only gps for computing the particle weights.
	string tiles_dir = ""; // get_tiles_dir(map);
	
	GridMap instantaneous_map(tiles_dir, map._tile_height_meters,
	                          map._tile_width_meters, map.m_by_pixels,
	                          map._map_type, 0);

	if (use_map_weight || use_ecc_weight)
	{
		instantaneous_map.reload(0, 0);
		create_instantaneous_map(sample, preproc, &instantaneous_map, 0);
	}

	// cv::Mat map_img = instantaneous_map.to_image();
	// cv::imshow("inst_map", map_img);
	// cv::waitKey(1);

	// view_observed_cells(instantaneous_map);

	int n_visible_cells = 0;

	for (int i = 0; i < instantaneous_map._N_TILES; i++)
		for (int j = 0; j < instantaneous_map._N_TILES; j++)
			n_visible_cells += instantaneous_map._tiles[i][j]->_observed_cells.size();

	if (n_visible_cells < 10)
	{
		for (int i = 0; i < _n; i++)
		{
			_w[i] = 1.0 / (double) _n;
			_p_bef[i] = _p[i];
		}
	}
	else
	{
		// #pragma omp parallel for default(none) private(i) shared(cloud, map, _p)
		_compute_all_particles_weights_with_outlier_rejection(instantaneous_map, map, sample->gps, _rejection_thresh);

		/*
		for (int i = 0; i < _n; i++)
		{
			//transformed_cloud->clear();
			//transformPointCloud(*cloud, *transformed_cloud, Pose2d::to_matrix(_p[i]));
			//_w[i] = _image_weight(transformed_cloud, map);

			_w[i] = _compute_particle_weight(instantaneous_map, map, sample->gps, _p[i]);
			_p_bef[i] = _p[i];
		}
		*/
	}

	*max_id = *min_id = 0;

	//fprintf(stderr, "\nDEBUG: Unormalized particle weights: ");
	for (i = 0; i < _n; i++)
	{
		assert(!isnan(_w[i]) && !isinf(_w[i]));
		//fprintf(stderr, "%.4lf ", _w[i]);

		if (_w[i] > _w[*max_id])
			*max_id = i;

		if (_w[i] < _w[*min_id])
			*min_id = i;
	}
	//fprintf(stderr, "\n");

	best = _p[*max_id];
}


void
ParticleFilter::correct(PointCloud<PointXYZRGB>::Ptr cloud, GridMap &map, Pose2d &gps)
{
	int max_id = 0;
	int min_id = 0;

	_compute_weights(cloud, map, gps, &max_id, &min_id);
	_normalize_weights(min_id, max_id);
	_resample();
}


void
ParticleFilter::correct(DataSample *sample, GridMap &map, SensorPreproc &preproc)
{
	int max_id = 0;
	int min_id = 0;

	_compute_weights(sample, map, preproc, &max_id, &min_id);
	_normalize_weights(min_id, max_id);
	_resample();
}


Pose2d
ParticleFilter::mean()
{
	Pose2d p;

	// th_y and th_x are used to compute the mean of theta.
	// Note: The circular mean is different than the arithmetic mean.
	// For example, the arithmetic mean of the three angles 0°, 0° and 90° is
	// (0+0+90)/3 = 30°, but the vector mean is 26.565°. The difference is bigger
	// when the angles are widely distributed. If the angles are uniformly distributed
	// on the circle, then the resulting radius will be 0, and there is no circular mean.
	// (In fact, it is impossible to define a continuous mean operation on the circle.)
	// See https://en.wikipedia.org/wiki/Mean_of_circular_quantities
	// See https://rosettacode.org/wiki/Averages/Mean_angle
	double th_y = 0., th_x = 0.;

	for (int i = 0; i < _n; i++)
	{
		p.x += (_p[i].x * _w[i]);
		p.y += (_p[i].y * _w[i]);

		th_y += (sin(_p[i].th) * _w[i]);
		th_x += (cos(_p[i].th) * _w[i]);
	}

	p.th = atan2(th_y, th_x);
	return p;
}


Pose2d
ParticleFilter::std()
{
	Pose2d s(0, 0, 0), m(0, 0, 0);
	m = mean();

	double diff_x = 0;
	double diff_y = 0;
	double diff_th = 0;

	for(int i = 0; i < _n; i++)
	{
		diff_x = (_p[i].x - m.x);
		diff_y = (_p[i].y - m.y);
		diff_th = normalize_theta(_p[i].th - m.th);

		s.x += pow(diff_x, 2);
		s.y += pow(diff_y, 2);
		s.th += pow(diff_th, 2);
	}

	s.x = sqrt(s.x / (double) _n);
	s.y = sqrt(s.y / (double) _n);
	s.th = sqrt(s.th / (double) _n);

	return s;
}


Pose2d
ParticleFilter::mode()
{
	return best;
}


