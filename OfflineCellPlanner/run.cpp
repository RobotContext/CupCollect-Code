#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/lambda/bind.hpp>
#include "OfflineCellPlanner.hpp"
#include "OfflineReachableMap.hpp"
#include "map.hpp"
#include "coverage.hpp"
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <algorithm>

#define MAX_CUPS_IN_TRAY 20
#define SWEEP_RADIUS 20
#define CUP_RADIUS 100

using namespace rw::sensor;
using namespace rw::loaders;
using namespace boost::geometry::model::d2;
using namespace pathPlanner;

const std::string mapfile = "Output/complete_map_project.pgm"; // Original map
const std::string reachableMapFile = "Output/complete_map_reachable.pgm"; // Map only with reachable space
const std::string workspaceMapFile = "Output/complete_map_workspace.pgm"; // Workspace map

void init_map( map * newMap )
{
	Image * workspaceMap;

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

	/* Load existing workspace map file */
	workspaceMap = PPMLoader::load(workspaceMapFile);

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

enum {IDLE,TRANSIT,SWEEPING} states;

int main()
{
	//declarations
	map 	newMap;					//map object
	int
			dist = 0 , 				//holds travelled distance
			state = 0 , 			//state variable following states enum
			tray = 0 , 				//number of cups currently in tray
			cell_id;				//id of the current cell
	bool
			returning = false , 	//indicates if the robot is on its way bact to cup table
			finished = false;		//indicates if all cells have been visited
	point_t
			current_pos , 			//holds the current position of the robot
			prev_pos , 				//holds the previous position of the robot
			saved_pos , 			//placeholder for a temporary saved position
			destination;			//destination of the robot
	std::vector< point_t >
			visited , 				//list of the points visited since start
			route , 				//the current route
			cup_list;				//list of cups in sight
	std::vector< point_t >::iterator
			itr;					//random access iterator for point lists

	//init map and positions
	init_map( & newMap );
	bg::set<0>(current_pos , 1);
	bg::set<1>(current_pos , 1);
	bg::set<0>(prev_pos , 0);
	bg::set<1>(prev_pos , 0);

	while( ! finished )
	{
		//state transition check
		switch( state )
		{
		case IDLE:
			//get the id of next cell to sweep
			/*TODO: cell_id = getNextCell(); */
			if( cell_id == -1 )
				//search finished
				finished = true;
			else
			{
				//go to next cell
				/* TODO: destination = getPosition( cell_id ); */
				state = TRANSIT;
			}
			break;

		case TRANSIT:
			if( current_pos == destination )
			{
				//destination has been reached
				if( returning )
				{
					//robot is at cup table
					destination = saved_pos;
					tray = 0;
					returning = false;
					/* TODO: goTo( &route , current_pos , destination ); */
				}
				else
					//robot is at a room to sweep
					state = SWEEPING;
			}
			else
				//get route to destination
				/* TODO: goTo( &route , current_pos , destination ); */
				break;

		case SWEEPING:
			//get sweeping route
			cover_area( & ( newMap.cells.at( cell_id ) ) , SWEEP_RADIUS , &route );
			state = IDLE;
			break;

		}

		//step through selected route
		if( ! route.isEmpty() )
		{
			//step through route
			for(itr = route.begin() ; itr < route.end() ; ++itr)
			{
				//update current point
				bg::set<0>(current_point , bg::get<0>(*itr));
				bg::set<1>(current_point , bg::get<1>(*itr));

				//check for cups between previous point and this point
				/* TODO: cupsBetweenPoints ( &cup_list , prev_pos , current_point , CUP_RADIUS ); */
				if( ! cup_list.isEmpty() )
				{
					//if cups are present save current position
					save_pos = current_pos;

					//iterate through list of cups
					while( ! cup_list.isEmpty() )
					{
						//add distance from current position to cup
						distance += boost::geometry::distance( current_point , *(cup_list.begin()) );

						//collect the cup and empty try if necessary
						if( ++tray >= MAX_CUPS_IN_TRAY )
						{
							/* TODO: distance += 2 * returnCupsDistance( *(cup_list.begin()) ); */
							tray = 0;
						}

						//update current position
						current_pos = *(cup_list.begin());

						//remove the collected cup from list
						cup_list.pop_front();
					}

					//add distance from the position of last cup to saved position
					distance += boost::geometry::distance( current_point , saved_pos );
					current_point = saved_pos;
				}

				//add distance from previous to current point
				distance += boost::geometry::distance( prev_point , *itr);

				//update previous point
				bg::set<0>(prev_point , bg::get<0>(current_point));
				bg::set<1>(prev_point , bg::get<1>(current_point));
			}
		}
	}

	std::cout << "covered distance:" << dist * 0.1 << "m" << std::endl;

	return 0;
}
