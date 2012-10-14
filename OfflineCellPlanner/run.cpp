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
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <algorithm>

#define MAX_CUPS_IN_TRAY 20
#define SWEEP_RADIUS 20
#define CUP_RADIUS 20

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

int getNextCell( int max )
{
	static int count;
	if( ++count < max )
		return count;
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

int main()
{
	//declarations
	map 	newMap;					//map object
	int
			dist = 0 , 				//holds travelled distance
			state = 0 , 			//state variable following states enum
			tray = 0 , 				//number of cups currently in tray
			cups_found = 0,			//number of cups collected
			cell_id;				//id of the current cell
	bool
			returning = false , 	//indicates if the robot is on its way bact to cup table
			finished = false;		//indicates if all cells have been visited
	point_t
			current_pos (10,10), 	//holds the current position of the robot
			prev_pos (1,1) ,		//holds the previous position of the robot
			saved_pos , 			//placeholder for a temporary saved position
			destination;			//destination of the robot
	std::vector< point_t >
			visited , 				//list of the points visited since start
			route , 				//the current route
			cup_list;				//list of cups in sight
	std::vector< point_t >::iterator
			itr;					//random access iterator for point lists
	Image * img = PPMLoader::load("img/complete_map_project.pgm");



	//init map and positions
	init_map( & newMap , & cup_list );
	visited.push_back(prev_pos);
	visited.push_back(current_pos);


	while( ! finished )
	{
		//state transition check
		switch( state )
		{
		case IDLE:
			//get the id of next cell to sweep
			cell_id = getNextCell( newMap.cells.size() );
//			std::cout << std::endl << "cell " << cell_id << ": ";
//			cup_list.push_back( point_t(10,10) ); //TODO: temporary hack - push test cup
			if( cell_id == -1 )
				//search finished
				finished = true;
			else
			{
				//go to next cell
				destination = getPosition( &newMap , cell_id );
				state = TRANSIT;
			}
			break;

		case TRANSIT:
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
			cover_area( & ( newMap.cells.at( cell_id ) ) , SWEEP_RADIUS , &route );
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
//				cup_list = cupsBetweenPoints(img , prev_pos , current_pos , CUP_RADIUS);
//				std::cout << cup_list.size() << " cups found between "
//						<< "("<< bg::get<0>(prev_pos) << "," << bg::get<1>(prev_pos) << ") and ("
//						<< bg::get<0>(current_pos) << "," << bg::get<1>(current_pos) << ") " << std::endl;;

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
						//std::cout << "collected cup at ("<< bg::get<0>(current_pos) << "," << bg::get<1>(current_pos) << ") ";

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

	return 0;
}
