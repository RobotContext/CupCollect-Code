#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/lambda/bind.hpp>
#include "OfflineCellPlanner.hpp"
#include "OfflineReachableMap.hpp"
#include "map.hpp"
#include "coverage.hpp"
#include "cupScanner.hpp"
#include "Image.hpp"
#include "PPMLoader.hpp"
#include "road_map.hpp"
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <algorithm>

#define MAX_CUPS_IN_TRAY 20
#define SWEEP_RADIUS 20
#define CUP_RADIUS 20
#define MINIMUM_CELL_WIDTH 5
#define STARTPOS 2409,1331
#define PREVPOS 2410,1330

using namespace rw::sensor;
using namespace rw::loaders;
using namespace boost::geometry::model::d2;
using namespace pathPlanner;

const std::string mapfile = "Output/complete_map_project.pgm"; // Original map
const std::string reachableMapFile = "Output/complete_map_reachable.pgm"; // Map only with reachable space
const std::string workspaceMapFile = "Output/complete_map_workspace.pgm"; // Workspace map


enum {IDLE,TRANSIT,SWEEPING};

void init_map( map * newMap , std::vector< point_t > * cup_list)
{
	Image * workspaceMap = PPMLoader::load(workspaceMapFile);

	//  // Create reachable map file and workspace map
	//  std::cout << "Loading map" << std::endl;
	//  Image * map = PPMLoader::load(mapfile);
	//  Image * reachableMap;
	//  reachableMap = new Image(map->getWidth(), map->getHeight(), Image::GRAY, Image::Depth8U);
	//  workspaceMap = new Image(map->getWidth(), map->getHeight(), Image::GRAY, Image::Depth8U);
	//  OfflineReachableMap::makeReachableMap(map, reachableMap, point_xy<int>(3000,1400));
	//  OfflineReachableMap::makeWorkspaceMap(reachableMap, workspaceMap);
	//  reachableMap->saveAsPGM(reachableMapFile);
	//  workspaceMap->saveAsPGM(workspaceMapFile);

	// Output
	newMap->vertices.reserve(100000);
	newMap->edges.reserve(10000);
	newMap->cells.reserve(1500);

	OfflineCellPlanner c = OfflineCellPlanner(workspaceMap,
			newMap,
			point_t(10,10),
			point_t((workspaceMap->getWidth()-10),(workspaceMap->getHeight()-10))
	);

	// Draw map for debugging purpose
	Image * vectormap = new Image(workspaceMap->getWidth(), workspaceMap->getHeight(), Image::GRAY, Image::Depth8U);
	newMap->drawMap(vectormap);
	vectormap->saveAsPGM("test.pgm");
}

int getNextCell( map * mp , int max )
{
	static int count;
	if( ++count < max )
	{
//		if(mp->cells.at(count).visited)
//			getNextCell( mp , max);
//		else
			return count;
	}
	else
		return -1;
}

point_t getPosition( map * mp , int cell )
{
	return  *(mp->cells.at( cell ).edges[0]->vertices[0]);
}

void goTo( std::vector<point_t> * route , point_t from , point_t to )
{
	route->push_back( to );
}

int returnCupsDistance( point_t from )
{
	return 200;
}

std::vector<pathPlanner::map::edge> merge( pathPlanner::map::cell * cell )
		{
	std::cout << " 1 ";
	static int cells_merged;
	std::vector<pathPlanner::map::edge> edges;
	for(std::vector< pathPlanner::map::edge * >::iterator edge_itr = cell->edges.begin(); edge_itr != cell->edges.end(); ++edge_itr)
	{
		std::cout << " 2 ";
		if( (*edge_itr)->edgetype == pathPlanner::map::edge::EDGESOFT && cells_merged < 5)
		{
			std::cout << " 3 ";
			for(std::vector< pathPlanner::map::cell * >::iterator cell_itr = (*edge_itr)->cells.begin();
					cell_itr != (*edge_itr)->cells.end(); ++cell_itr)
			{
				std::cout << " 4 ";
				if( (*cell_itr)->cellId != cell->cellId &&
						get_cell_width( *cell_itr ) < MINIMUM_CELL_WIDTH && ! ( (*cell_itr)->visited) )
				{
					std::cout << " cells merged:" << cells_merged;
					std::vector<pathPlanner::map::edge> temp = merge( *cell_itr );
					edges.insert( edges.end() , temp.begin() , temp.end() );
					(*cell_itr)->visited = true;
				}
				std::cout << " 6 ";
			}
			std::cout << " 7 ";
		}
		std::cout << " 8 ";
	}
	std::cout << " 9 ";
	cells_merged++;
	return edges;
		}

void merge_with_neighbours( pathPlanner::map::cell * cell , std::vector<point_t> * route )
{
	pathPlanner::map::cell new_cell;
	std::vector< point_t > vertices;

	std::vector<pathPlanner::map::edge> edges = merge( cell );
	for(std::vector< pathPlanner::map::edge * >::iterator edge_itr = cell->edges.begin(); edge_itr != cell->edges.end(); ++edge_itr)
	{
		new_cell.edges.push_back( *edge_itr );
	}
	cover_area( &new_cell , SWEEP_RADIUS , route );
}

int main()
{
	//declarations
	Image * img = PPMLoader::load("img/complete_map_project.pgm");
	map 	newMap;			//map object
	road_map rm;

	int
	dist = 0 , 				//holds travelled distance
	state = IDLE , 			//state variable following states enum
	tray = 0 , 				//number of cups currently in tray
	cups_found = 0,			//number of cups collected
	cell_width = 0,			//width of the current cell
	cell_id = 0;			//id of the current cell

	bool
	returning = false , 	//indicates if the robot is on its way bact to cup table
	finished = false;		//indicates if all cells have been visited

	point_t
	current_pos (STARTPOS),	//holds the current position of the robot
	prev_pos (PREVPOS),		//holds the previous position of the robot
	saved_pos , 			//placeholder for a temporary saved position
	destination;			//destination of the robot

	std::vector< point_t >
	visited , 				//list of the points visited since start
	route , 				//the current route
	cup_list;				//list of cups in sight

	std::vector< point_t >::iterator
	itr;					//random access iterator for point lists

	//init map and positions
	init_map( & newMap , & cup_list );
	visited.push_back(prev_pos);
	visited.push_back(current_pos);
//	rm.cell_list.reserve(20000000);
//	rm.node_list.reserve(20000000);
//	rm.map( &newMap.edges, &newMap.cells );


	while( ! finished )
	{
		//state transition check
		switch( state )
		{
		case IDLE:
			//get the id of next cell to sweep
			cell_id = getNextCell( &newMap , newMap.cells.size() );

			if( cell_id == -1 )
				//search finished
				finished = true;
			else
			{
				cell_width = get_cell_width( & newMap.cells.at(cell_id) );

				//go to next cell
				destination = getPosition( &newMap , cell_id );
				state = TRANSIT;
			}
			break;

		case TRANSIT:
//			dist += rm.find_route_to_rm_cell(current_pos, &rm.cell_list.at(0), &rm.cell_list.at(19), destination);
//			if(returning)
//			{
//				dist += rm.find_route_to_rm_cell(current_pos, &rm.cell_list.at(0), &rm.cell_list.at(19), destination);
//				returning = false;
//			}
//			current_pos = destination;
//			state = SWEEPING;

			if( bg::get<0>(current_pos) == bg::get<0>(destination) && bg::get<1>(current_pos) == bg::get<1>(destination) )
			{
				//destination has been reached
				if( returning )
				{
					//robot is at cup table
					destination = saved_pos;
					tray = 0;
					returning = false;
					goTo( &route , current_pos , destination );
				}
				else
					//robot is at a room to sweep
					state = SWEEPING;
			}
			else
				//get route to destination
				goTo( &route , current_pos , destination );
			break;

		case SWEEPING:
			//get sweeping route
			if(cell_width >= MINIMUM_CELL_WIDTH)
				cover_area( & ( newMap.cells.at( cell_id ) ) , SWEEP_RADIUS , &route );
			else
			{
			//	merge_with_neighbours( & (newMap.cells.at( cell_id ) ) , &route );
			}
			state = IDLE;
			break;

		}

		//step through selected route
		if( ! route.empty() )
		{
			//step through route
			for(itr = route.begin() ; itr < route.end() ; ++itr)
			{
				//update current point
				bg::set<0>(current_pos , bg::get<0>(*itr));
				bg::set<1>(current_pos , bg::get<1>(*itr));

				//check for cups between previous point and this point
				//cup_list = cupsBetweenPoints(img , prev_pos , current_pos , CUP_RADIUS);

				if( ! cup_list.empty() )
				{
					//if cups are present save current position
					saved_pos = current_pos;

					//iterate through list of cups
					while( ! cup_list.empty() )
					{
						//add distance from current position to cup
						dist += boost::geometry::distance( current_pos , *(cup_list.begin()) );//was begin

						//collect the cup and empty try if necessary
						if( ++tray >= MAX_CUPS_IN_TRAY )
						{
							dist += 2 * returnCupsDistance( *(cup_list.begin()) );//was begin
							tray = 0;
						}

						//update current position
						current_pos = *(cup_list.begin());
						visited.push_back( current_pos );

						//remove the collected cup from list
						cup_list.erase( cup_list.begin() );
						cups_found++;
					}

					//add distance from the position of last cup to saved position
					dist += boost::geometry::distance( current_pos , saved_pos );
					current_pos = saved_pos;
				}

				//add distance from previous to current point
				dist += boost::geometry::distance( prev_pos , *itr);

				//update previous point
				bg::set<0>(prev_pos , bg::get<0>(current_pos));
				bg::set<1>(prev_pos , bg::get<1>(current_pos));
			}
		}
		visited.insert(visited.end() , route.begin() , route.end() );
		route.clear();
	}
	//print entire route
	//	for(itr = visited.begin() ; itr < visited.end() ; ++itr)
	//		std::cout << "("<< bg::get<0>(*itr) << "," << bg::get<1>(*itr) << ") ";
	std::cout << cups_found << " cups found while covering " << dist * 0.1 << "m" << std::endl;

	//visualize covered area
	std::cout << "visualizing...";
	visualize_list( img , &visited );

	// save image
	newMap.saveImage( img , "img/output.pgm");

	return 0;
}
