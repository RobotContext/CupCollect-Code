#include "coverage.hpp"

namespace bg = boost::geometry;
namespace bl=boost::lambda;

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
