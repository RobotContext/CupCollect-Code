#ifndef _COVERAGE_
#define _COVERAGE_

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/lambda/bind.hpp>
#include "map.hpp"
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include "Image.hpp"
#include "PPMLoader.hpp"

typedef boost::geometry::model::d2::point_xy<int> point_t;
typedef boost::geometry::model::linestring<point_t> line_t;
enum {UPWARDS,DOWNWARDS};

#define PATH_COLOR 200
using namespace rw::loaders;
using namespace rw::sensor;

void remove_duplicates( std::vector< point_t > * list );
void cover_area( pathPlanner::map::cell * cell , int radius , std::vector< point_t > * route );
int get_cell_width( pathPlanner::map::cell * cell );
void draw_edge( Image * img , point_t end , point_t begin );
void visualize_list( Image * img , std::vector< point_t > * list );

#endif /*_COVERAGE_*/
