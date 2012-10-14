#include "coverage.hpp"

namespace bg = boost::geometry;
namespace bl=boost::lambda;
using namespace rw::loaders;
using namespace rw::sensor;

void remove_duplicates( std::vector< point_t > * list )
{
	std::vector< point_t >::iterator itr_a , itr_b;

	for (itr_a = list->begin() ; itr_a != list->end(); itr_a++ )
	{
		itr_b = itr_a;
		itr_b++;
		for ( ; itr_b != list->end() ; itr_b++ )
		{
			if(bg::get<0>(*itr_a) == bg::get<0>( *itr_b ) && bg::get<1>(*itr_a) == bg::get<1>( *itr_b ) )
			{
				//				std::cout << "removed one" << std::endl;
				itr_b = list->erase( itr_b );
				itr_b--;
			}
		}
	}
}

int get_cell_width( pathPlanner::map::cell * cell )
{
	std::vector< pathPlanner::map::edge * >::iterator edge_itr;
	int x_min , x_max , temp;

	x_min = x_max = temp = bg::get<0>(*( (*cell->edges.begin())->vertices.at(0) ));
	for(edge_itr = cell->edges.begin(); edge_itr != cell->edges.end(); ++edge_itr)
	{
		temp = bg::get<0>(*( (*edge_itr)->vertices.at(0) ));
		if( temp < x_min )
			x_min = temp;
		if( temp > x_max )
			x_max = temp;
		temp = bg::get<0>(*( (*edge_itr)->vertices.at(1) ));
		if( temp < x_min )
			x_min = temp;
		if( temp > x_max )
			x_max = temp;
	}
	return x_max - x_min;
}

void draw_edge( Image * img , point_t end , point_t begin )
{
	int move = 0;
	int moves[9][2];
	double dist_to_line , dist_to_target, least, score;
	point_t test_point , current;
	line_t edge;

	//STOP
	moves [0][0] = 0;  	moves [0][1] = 0;
	//NORTH
	moves [1][0] = 0; 	moves [1][1] = -1;
	//NORTH EAST
	moves [2][0] = 1; 	moves [2][1] = -1;
	//EAST
	moves [3][0] = 1; 	moves [3][1] = 0;
	//SOUTH EAST
	moves [4][0] = 1; 	moves [4][1] = 1;
	//SOUTH
	moves [5][0] = 0; 	moves [5][1] = 1;
	//SOUTH WEST
	moves [6][0] = -1; 	moves [6][1] = 1;
	//WEST
	moves [7][0] = -1; 	moves [7][1] = 0;
	//NORTH WEST
	moves [8][0] = -1; 	moves [8][1] = -1;

	bg::set<0>( current , bg::get<0>(begin));
	bg::set<1>( current , bg::get<1>(begin));

	//generate line(vector) from start and goal points
	edge.push_back(begin);
	edge.push_back(end);

	while( bg::get<0>(current) != bg::get<0>(end) || bg::get<1>(current) != bg::get<1>(end) )
	{
		//start guess
		least = 10000;

		//iterate through the possible directions
		for (int i = 0 ; i < 9 ; i++)
		{
			//set temporary point variable to the coordinate being evaluated
			bg::set<0>( test_point , bg::get<0>(current)+ moves[i][0]);
			bg::set<1>( test_point , bg::get<1>(current)+ moves[i][1]);

			//evaluate distance to the line being followed
			dist_to_line = boost::geometry::distance(test_point, edge);

			//evaluate distance to target
			dist_to_target = boost::geometry::distance(test_point, end);

			//calculate reguation variable
			score = dist_to_target + abs(dist_to_line);

			//if this point has lower score than former best point
			if( (score < least) )
			{
				//set as new best point
				least = score;
				move = i;
			}
		}

		//set the color of the current pixel before moving
		img->setPixel8U( bg::get<0>(current) , bg::get<1>(current) , PATH_COLOR );

		bg::set<0>( current , bg::get<0>(current) + moves[move][0] );
		bg::set<1>( current , bg::get<1>(current) + moves[move][1] );
	}
}

void visualize_list( Image * img , std::vector< point_t > * list )
{
	for( std::vector< point_t >::iterator itr = list->begin() ; itr != list->end() - 1 ; )
		draw_edge( img , *itr , *itr++ );
}
void cover_area( pathPlanner::map::cell * cell , int radius , std::vector< point_t > * route )
{
	int x_min , x_max , y_min , y_max , sweep = 1 , direction , distance ;
	std::vector< pathPlanner::map::edge * >::iterator edge_itr;
	std::vector< point_t > * list;					//pointer to list of vertices
	std::vector< point_t > vertices;				//copy of vertices
	std::vector< point_t > intersections_list;  	//temporary list holding itersection points
	line_t bounding_lines;							//multipoint line surrounding the covered area
	line_t test_line;								//temporary line for finding intersection points
	std::vector< point_t >::iterator itr;			//general purpose iterator for point lists
	std::vector< point_t >::iterator list_itr;		//iterator for insertion into turning point list
	point_t upper_test_point , lower_test_point , temp_point;

	//construct bounding lines
	for(edge_itr = cell->edges.begin(); edge_itr != cell->edges.end(); ++edge_itr)
	{
		bounding_lines.push_back( * ( (*edge_itr)->vertices.at(0) ) );
		bounding_lines.push_back( * ( (*edge_itr)->vertices.at(1) ) );
		vertices.push_back( * ( (*edge_itr)->vertices.at(0) ) );
		vertices.push_back( * ( (*edge_itr)->vertices.at(1) ) );
	}
	bounding_lines.push_back( * ( (*(cell->edges.begin()))->vertices.at(0) )  );

	list = &vertices;

	//find boundaries
	itr = list->begin();
	x_min = x_max = bg::get<0>(*itr);
	y_min = y_max = bg::get<1>(*itr);
	for( itr = list->begin() ; itr != list->end() ; ++itr )
	{
		if( bg::get<0>(*itr) < x_min )
			x_min = bg::get<0>(*itr);

		if( bg::get<0>(*itr) > x_max )
			x_max = bg::get<0>(*itr);

		if( bg::get<1>(*itr) < y_min )
			y_min = bg::get<1>(*itr);

		if( bg::get<1>(*itr) > y_max )
			y_max = bg::get<1>(*itr);
	}

	//clear the list
	list->clear();

	//construct turning points for sweep
	while( sweep < x_max )
	{
		//construct new test line
		test_line.clear();
		test_line.push_back(  point_t(x_min + sweep , y_max + radius) );
		test_line.push_back(  point_t(x_min + sweep , y_min - radius) );

		//find intersections
		intersections_list.clear();
		bg::intersection( test_line , bounding_lines , intersections_list );
		remove_duplicates( &intersections_list );

		//insert in list of turning points
		list_itr = list->end();
		list->insert( list_itr , intersections_list.begin() , intersections_list.end() );

		//upkeep
		sweep += radius;
	}

	//sort list
	direction = UPWARDS;
	for ( itr = list->begin() ; itr < list->end() ; itr++ )
	{
		if( bg::get<1>(*itr) < bg::get<1>( *(itr+1) ) && direction == UPWARDS )
			std::iter_swap( itr , itr+1);

		if( bg::get<1>(*itr) > bg::get<1>( *(itr+1) ) && direction == DOWNWARDS )
			std::iter_swap( itr , itr+1);

		if( direction == UPWARDS )
			direction = DOWNWARDS;
		else
			direction = UPWARDS;

		itr++;
	}

	//remove endpoint from list
	if( bg::get<0>( list->back() ) == 0 && bg::get<1>( list->back() ) == 0)
		list->pop_back();

	route->insert( route->end() , vertices.begin() , vertices.end() );
}
