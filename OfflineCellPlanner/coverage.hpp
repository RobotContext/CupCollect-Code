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

typedef boost::geometry::model::d2::point_xy<int> point_t;
typedef boost::geometry::model::linestring<point_t> line_t;
enum {UPWARDS,DOWNWARDS};

void remove_duplicates( std::vector< point_t > * list );
int cover_area( pathPlanner::map::cell * cell , int radius );


#endif /*_COVERAGE_*/
