
 /*
 * roadmap.h
 *
 *  Created on: Oct 7, 2012
 *      Author: boerresen
 */
//============================================================================
// Name        : samle_kopper_roadmap.cpp
// Author      : Lasse B��rresen
// Version     :
// Copyright   :
// Description :
//============================================================================


#include <iostream>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <cmath>
#include <vector>
#include "map.hpp"
#include "Image.hpp"
#include "PPMLoader.hpp"

#define CELLS_PER_SOFT_EDGE 2
#define TOP_LEFT                        0
#define TOP_RIGHT                       1
#define BOTTOM_RIGHT            2
#define BOTTOM_LEFT             3

namespace bg = boost::geometry;
using namespace rw::sensor;
using namespace rw::loaders;

typedef bg::model::d2::point_xy<int> point_t;

class rm_node;
class rm_cell;

class rm_node
{
public:
        pathPlanner::map::edge* this_edge;
        std::vector<rm_node*> neigh_rm_nodes;
        std::vector<rm_cell*> neigh_rm_cells;
        bool visited;
};

class rm_cell
{
public:
        pathPlanner::map::cell* this_cell;
        std::vector<rm_node*> neigh_rm_nodes;
        bool visited;
};

class route
{
public:
        //list of nodes in route, chonologically sorted.
        std::vector<rm_node*> trail;
        double length;
};



class road_map
{

        //Prototypes



public:

//attributes
        std::vector<rm_cell> cell_list;
        std::vector<rm_node> node_list;
        Image * imggg;



//member functions
        std::vector<rm_cell*> find_master_route();
        std::vector<rm_cell*> search_rm_cell(rm_cell* this_cell);

        void print_cells_info();

        void draw_route(route the_route );

        void DistanceFromLine(double cx, double cy, double ax, double ay ,
                                                  double bx, double by, double &distanceSegment,
                                                  double &distanceLine);

        void draw_line_pgm( point_t first_point, point_t second_point );

        point_t edge_pos( pathPlanner::map::edge* the_edge);

        point_t cell_pos( pathPlanner::map::cell* the_cell);

        std::vector<point_t*> find_cell_corners( pathPlanner::map::cell* the_cell );

        double find_route_to_rm_cell(point_t starting_point, rm_cell* starting_cell, rm_cell* destination_cell, point_t & destination_point );

        //first connection to the rm is made from the special node of the trays.
        void map( std::vector<pathPlanner::map::edge>* edges, std::vector<pathPlanner::map::cell>* cells );

        double distance_between_point_ts( point_t first_point, point_t second_point );

        bool compare_point_ts( point_t first_point, point_t second_point );
};
