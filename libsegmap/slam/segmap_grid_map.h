
#ifndef _SEGMAP_GRID_MAP_H_
#define _SEGMAP_GRID_MAP_H_


#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "libsegmap/datasets/segmap_dataset.h"
#include "libsegmap/types/segmap_colormaps.h"
#include "libsegmap/slam/segmap_preproc.h"

#include <unordered_set>


class GridMapTile
{
public:
	enum MapType
	{
		TYPE_SEMANTIC = 0,
		TYPE_VISUAL,
		TYPE_OCCUPANCY,
		TYPE_REFLECTIVITY,
	};

	// make these attributes private
	// they are used in the view function
	double _hm, _wm, _xo, _yo;
	double _m_by_pixel;
	double _pixels_by_m;
	int _h, _w;
	double *_map;
	CityScapesColorMap _color_map;
	std::unordered_set<int> _observed_cells;

	std::vector<double> _unknown;
	int _n_fields_by_cell;
	MapType _map_type;
	std::string _tiles_dir;
	int _save_maps;

	void _initialize_map();
	void _initialize_derivated_values();
	cv::Scalar cell2color(double *cell_vals);

	GridMapTile(double point_y, double point_x,
			double height_meters, double width_meters,
			double resolution, MapType map_type, std::string tiles_dir,
			int save_maps=0);

	~GridMapTile();

	static const char* type2str(MapType map_type);

	void save();
	void add_point(pcl::PointXYZRGB &p);
	bool contains(double x, double y);
	std::vector<double> read_cell(pcl::PointXYZRGB &p);
	std::vector<double> read_cell(double x_world, double y_world);

	double *read_cell_ref(double x_world, double y_world);

	cv::Mat to_image();
};


class GridMap
{
public:
	static const int _N_TILES = 3;

	std::string _tiles_dir;
	double _tile_height_meters;
	double _tile_width_meters;
	int _middle_tile;
	GridMapTile::MapType _map_type;
	int _save_maps;
	int _map_initialized;

	double m_by_pixels;
	double pixels_by_m;
	double height_meters;
	double width_meters;
	int xo, yo;

	GridMapTile *_tiles[_N_TILES][_N_TILES];

	GridMap(std::string tiles_dir, double tile_height_meters,
					double tile_width_meters, double resolution,
					GridMapTile::MapType map_type, int save_maps=0);

	~GridMap();
	GridMapTile* _reload_tile(double x, double y);
	void _reload_tiles(double robot_x, double robot_y);
	void reload(double robot_x, double robot_y);

	void add_point(pcl::PointXYZRGB &p);
	void add_occupancy_shot(std::vector<SensorPreproc::CompletePointData> &points,
							int do_raycast = 1, int use_world_ref = 1);

	std::vector<double> read_cell(pcl::PointXYZRGB &p);
	std::vector<double> read_cell(double x_world, double y_world);

	double *read_cell_ref(double x_world, double y_world);

	cv::Mat to_image();
	void save();

	void _check_if_map_was_initialized();
	void _free_tiles();
};


// utility function for updating the map with a point cloud.
void update_maps(DataSample *sample, SensorPreproc &preproc, GridMap *visual_map, GridMap *reflectivity_map, GridMap *semantic_map, GridMap *occupancy_map);
void create_instantaneous_map(DataSample *sample, SensorPreproc &preproc, GridMap *map, int do_raycast);

#endif
