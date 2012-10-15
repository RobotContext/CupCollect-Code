
#include "road_map.hpp"

point_t road_map::edge_pos( pathPlanner::map::edge* the_edge)
{
	point_t res_point;
	res_point.x();
	//average of x values
	res_point.x( ( the_edge->vertices.at( 0 )->x() + the_edge->vertices.at( 1 )->x() ) / 2 );
	//average of y values
	res_point.y( ( the_edge->vertices.at( 0 )->y() + the_edge->vertices.at( 1 )->y() ) / 2 );
	return res_point;
}

point_t road_map::cell_pos( pathPlanner::map::cell* the_cell)
{
	//      std::cout << "test:11" << std::endl;
	point_t res_point;
	//      std::cout << "test:12" << std::endl;
	//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//OBS:" SHOULD BE MORE GENERIC"
	//average of x_values
	//      std::cout << "test:13" << std::endl;
	std::vector<point_t*> cell_corners = find_cell_corners(the_cell);
	//std::cout << "number of corners: " << cell_corners.size() << std::endl;
	/*
        int top_left_corner_x = cell_corners.at( 0 )->x();
        int top_right_corner_x = cell_corners.at( 1 )->x();
        int bottom_right_corner_x = cell_corners.at( 2 )->x();
        int bottom_left_corner_x = cell_corners.at( 3 )->x();
	 */
	res_point.x( ( find_cell_corners(the_cell).at( 0 )->x() + find_cell_corners(the_cell).at( 1 )->x() + find_cell_corners(the_cell).at( 2 )->x() + find_cell_corners(the_cell).at( 3 )->x() ) / 4 );
	//average of y values
	//      std::cout << "test:14" << std::endl;
	res_point.y( ( find_cell_corners(the_cell).at( 0 )->y() + find_cell_corners(the_cell).at( 1 )->y() + find_cell_corners(the_cell).at( 2 )->y() + find_cell_corners(the_cell).at( 3 )->y() ) / 4 );
	//      std::cout << "test:15" << std::endl;
	return res_point;
}


std::vector<point_t*> road_map::find_cell_corners( pathPlanner::map::cell* the_cell )
{
	//std::cout << "test:16" << std::endl;
	std::vector<point_t*> res;
	//std::cout << "test:17" << std::endl;
	//go though all edges of cell
	int the_size = (int) the_cell->edges.size();
	//std::cout << "fucking cellID" << the_cell->cellId << std::endl;

	for ( int i = 0; i < the_size; i++ )
	{
		//std::cout << "test:18" << std::endl;
		//go through all vertices of edge
		for( int j = 0; j < (int) the_cell->edges.at( i )->vertices.size(); j++ )
		{
			//                              std::cout << "test:19" << std::endl;
			if( res.empty() )
			{
				//                                      std::cout << "test:20" << std::endl;
				res.push_back( the_cell->edges.at( i )->vertices.at( j ) );
				res.push_back( the_cell->edges.at( i )->vertices.at( j ) );
				res.push_back( the_cell->edges.at( i )->vertices.at( j ) );
				res.push_back( the_cell->edges.at( i )->vertices.at( j ) );
			}
			//if more top_left(0)
			else if(
					(
							the_cell->edges.at( i )->vertices.at( j )->x() < res.at(TOP_LEFT)->x() &&
							the_cell->edges.at( i )->vertices.at( j )->y() < res.at(TOP_LEFT)->y()
					) ||
					(
							the_cell->edges.at( i )->vertices.at( j )->x() < res.at(TOP_LEFT)->x() &&
							the_cell->edges.at( i )->vertices.at( j )->y() == res.at(TOP_LEFT)->y()
					) ||
					(
							the_cell->edges.at( i )->vertices.at( j )->x() == res.at(TOP_LEFT)->x() &&
							the_cell->edges.at( i )->vertices.at( j )->y() < res.at(TOP_LEFT)->y()
					)
			)
			{
				//                                       std::cout << "test:21" << std::endl;
				res.erase( res.begin() );
				res.insert( res.begin(), the_cell->edges.at( i )->vertices.at( j ) );
			}
			//if more top_right
			else if(
					(
							the_cell->edges.at( i )->vertices.at( j )->x() > res.at(TOP_RIGHT)->x() &&
							the_cell->edges.at( i )->vertices.at( j )->y() < res.at(TOP_RIGHT)->y()
					) ||
					(
							the_cell->edges.at( i )->vertices.at( j )->x() > res.at(TOP_RIGHT)->x() &&
							the_cell->edges.at( i )->vertices.at( j )->y() == res.at(TOP_RIGHT)->y()
					) ||
					(
							the_cell->edges.at( i )->vertices.at( j )->x() == res.at(TOP_RIGHT)->x() &&
							the_cell->edges.at( i )->vertices.at( j )->y() < res.at(TOP_RIGHT)->y()
					)
			)
			{
				//                              std::cout << "test:22" << std::endl;
				res.erase( res.begin() + TOP_RIGHT );
				res.insert( res.begin() + TOP_RIGHT, the_cell->edges.at( i )->vertices.at( j ) );
			}
			//if more bottom_right
			else if(
					(
							the_cell->edges.at( i )->vertices.at( j )->x() > res.at(BOTTOM_RIGHT)->x() &&
							the_cell->edges.at( i )->vertices.at( j )->y() > res.at(BOTTOM_RIGHT)->y()
					) ||
					(
							the_cell->edges.at( i )->vertices.at( j )->x() > res.at(BOTTOM_RIGHT)->x() &&
							the_cell->edges.at( i )->vertices.at( j )->y() == res.at(BOTTOM_RIGHT)->y()
					) ||
					(
							the_cell->edges.at( i )->vertices.at( j )->x() == res.at(BOTTOM_RIGHT)->x() &&
							the_cell->edges.at( i )->vertices.at( j )->y() > res.at(BOTTOM_RIGHT)->y()
					)
			)
			{
				//                                      std::cout << "test:23" << std::endl;
				res.erase( res.begin() + BOTTOM_RIGHT );
				res.insert( res.begin() + BOTTOM_RIGHT, the_cell->edges.at( i )->vertices.at( j ) );
			}
			//if more bottom_left
			else if(
					(
							the_cell->edges.at( i )->vertices.at( j )->x() < res.at(BOTTOM_LEFT)->x() &&
							the_cell->edges.at( i )->vertices.at( j )->y() > res.at(BOTTOM_LEFT)->y()
					) ||
					(
							the_cell->edges.at( i )->vertices.at( j )->x() < res.at(BOTTOM_LEFT)->x() &&
							the_cell->edges.at( i )->vertices.at( j )->y() == res.at(BOTTOM_LEFT)->y()
					) ||
					(
							the_cell->edges.at( i )->vertices.at( j )->x() == res.at(BOTTOM_LEFT)->x() &&
							the_cell->edges.at( i )->vertices.at( j )->y() > res.at(BOTTOM_LEFT)->y()
					)
			)
			{
				//                                      std::cout << "test:24" << std::endl;
				res.erase( res.begin() + BOTTOM_LEFT );
				res.insert( res.begin() + BOTTOM_LEFT, the_cell->edges.at( i )->vertices.at( j ) );
			}
		}
	}
	//std::cout << "test:30" << std::endl;
	return res;
}

double road_map::distance_between_point_ts( point_t first_point, point_t second_point )
{
	return (double) sqrt( pow( first_point.x() - second_point.x(), 2 ) + pow( first_point.x() - second_point.x(), 2 )  );
}

bool road_map::compare_point_ts( point_t first_point, point_t second_point )
{
	if( first_point.x() == second_point.x() && first_point.y() == second_point.y() )
		return true;
	else
		return false;
}



void road_map::print_cells_info()
{
	//        std::cout << "cell_list size:" << cell_list.size() << std::endl;
	for ( int i = 0; i < (int) cell_list.size(); i++ )
	{

		//        std::cout << "cellID:" << cell_list.at(i).this_cell->cellId << std::endl;
		//        std::cout << "edges number:" << cell_list.at(i).this_cell->edges.size() << std::endl;
	}
}


//could just be function that creates a list of rm_cells to visit, it could be done offline.
//input: nothing
//output: next rm_cell to reach.
//could it be smarter that this algorithm just finds the closes cell not visited??
/*
void road_map::search_rm_cell(rm_cell* this_cell)
{
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //every cell must be initialized to having not been visited
        //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        this_cell->this_cell->visited = pathPlanner::map::cell::VISITED;

        //add this cell to master_route here.

        //check if neigh_cell of this_cell is not it self and NOT_VISITED. if so, search the cell
        for( int i = 0; i < this_cell->neigh_rm_nodes.size(); i++ )
        {
                if(!compare_point_ts(cell_pos(this_cell->neigh_rm_nodes.at( i )->neigh_rm_cells.at( 0 )->this_cell), cell_pos( this_cell->this_cell ) ) &&
                                this_cell->neigh_rm_nodes.at( i )->neigh_rm_cells.at( 1 )->this_cell->visited == pathPlanner::map::cell::NOT_VISITED
                        )
                {
                        search_rm_cell( this_cell->neigh_rm_nodes.at( i )->neigh_rm_cells.at( 0 ) );
                }
                if(!compare_point_ts(cell_pos(this_cell->neigh_rm_nodes.at( i )->neigh_rm_cells.at( 1 )->this_cell), cell_pos( this_cell->this_cell ) ) &&
                                this_cell->neigh_rm_nodes.at( i )->neigh_rm_cells.at( 1 )->this_cell->visited == pathPlanner::map::cell::NOT_VISITED
                        )
                {
                        search_rm_cell( this_cell->neigh_rm_nodes.at( i )->neigh_rm_cells.at( 1 ) );
                }
        }

        this_cell->this_cell->childs_visited = pathPlanner::map::cell::CHILDS_VISITED;
}
 */


double road_map::find_route_to_rm_cell( point_t starting_point, rm_cell* starting_cell, rm_cell* destination_cell, point_t & destination_point)
{
	bool at_destination = false;
	double final_length;
	//routes sorted shortest to longest
	std::vector<route> sorted_routes;
	//std::cout << "test:0" << std::endl;
	//initiate sorted_routes
	//        std::cout << "starting_cell neigh_rm_node size: " << starting_cell->neigh_rm_nodes.size() << std::endl;
	//        std::cout << "starting_cell edges: " << starting_cell->this_cell->edges.size() << std::endl;
	for( int i = 0; i < (int) starting_cell->neigh_rm_nodes.size(); i++ )
	{
		//create route object
		route dummy_route;
		dummy_route.trail.push_back( starting_cell->neigh_rm_nodes.at( i ) );
		dummy_route.length = distance_between_point_ts(
				edge_pos( starting_cell->neigh_rm_nodes.at( i )->this_edge ),
				starting_point
		);
		//add as first element
		if(sorted_routes.empty())
		{
			sorted_routes.push_back( dummy_route );
		}
		//insert at proper position
		else
		{
			int j = 0;
			while( dummy_route.length > sorted_routes.at( j ).length )
				j++;
			sorted_routes.insert( sorted_routes.begin() + j, dummy_route );
		}

		//final destination. lets call it a day.
		//ALWAYS 2 cells?!?! what to do instead?
		for( int j = 0; j < (int) dummy_route.trail.at( dummy_route.trail.size() - 1 )->neigh_rm_cells.size(); j++ )
		{
			//comparison could be a function called "same_cells"
			//                        std::cout << "sidste cell p�� route cellID: " << dummy_route.trail.at( dummy_route.trail.size() - 1 )->neigh_rm_cells.at(j)->this_cell->cellId << std::endl;
			//                        std::cout << "sidste cell p�� route edges.size(): " << dummy_route.trail.at( dummy_route.trail.size() - 1 )->neigh_rm_cells.at(j)->this_cell->edges.size() << std::endl;

			point_t last_node_cell_j_pos = cell_pos( dummy_route.trail.at( dummy_route.trail.size() - 1 )->neigh_rm_cells.at( j )->this_cell );
			point_t dest_cell_pos = cell_pos( destination_cell->this_cell );
			if( compare_point_ts( last_node_cell_j_pos, dest_cell_pos ) )
			{
				//return entire function by returning shortest route..
				at_destination = true;
				destination_point = edge_pos( dummy_route.trail.at(dummy_route.trail.size() - 1)->this_edge );
				final_length = dummy_route.length;
			}
		}
	}


	//std::cout << "finished initiating sorted routes." << std::endl;
	//std::cout << "number of routes." << sorted_routes.size() << std::endl;
	//std::cout << "f��rste routes umiddelbare l��ngede" << sorted_routes.at(0).length << std::endl;
	while(!at_destination)
	{

		//take latest rm_node on shortest route
		int num_options = 0;
		int shortest;
		double shortest_dist;
		for (int i = 0; i < (int) sorted_routes.at(0).trail.at(sorted_routes.at(0).trail.size() - 1)->neigh_rm_nodes.size(); ++i)
		{
			//find closets neigh_node that is not visited.
			//initialize variable:"shortest"
			if( num_options == 0 && !sorted_routes.at(0).trail.at(sorted_routes.at(0).trail.size() - 1)->neigh_rm_nodes.at(i)->visited )
			{
				//distance from latest node on shortest route on sorted list to neigh_node at variable "i".
				shortest_dist =
						distance_between_point_ts(
								edge_pos( sorted_routes.at(0).trail.at(sorted_routes.at(0).trail.size() - 1)->neigh_rm_nodes.at(i)->this_edge ),
								edge_pos( sorted_routes.at(0).trail.at( sorted_routes.at(0).trail.size() - 1 )->this_edge )
						);
				shortest = i;
				num_options++;
				sorted_routes.at(0).trail.at(sorted_routes.at(0).trail.size() - 1)->neigh_rm_nodes.at(i)->visited = true;
			}
			//test for shorter distance
			else if( shortest_dist <
					distance_between_point_ts(
							edge_pos( sorted_routes.at(0).trail.at(sorted_routes.at(0).trail.size() - 1)->neigh_rm_nodes.at(i)->this_edge ),
							edge_pos( sorted_routes.at(0).trail.at( sorted_routes.at(0).trail.size() - 1 )->this_edge )
					) &&
					!sorted_routes.at(0).trail.at(sorted_routes.at(0).trail.size() - 1)->neigh_rm_nodes.at(i)->visited )
			{
				//save as new shortest
				shortest_dist = distance_between_point_ts(
						edge_pos( sorted_routes.at(0).trail.at(sorted_routes.at(0).trail.size() - 1)->neigh_rm_nodes.at(i)->this_edge ),
						edge_pos( sorted_routes.at(0).trail.at( sorted_routes.at(0).trail.size() - 1 )->this_edge )
				);
				shortest = i;
				num_options++;
				sorted_routes.at(0).trail.at(sorted_routes.at(0).trail.size() - 1)->neigh_rm_nodes.at(i)->visited = true;
			}
		}

		//delete shortest route. it is a dead end.
		if( num_options == 0 )
		{
			//                        std::cout << std::endl << "number of sorted routes: " << sorted_routes.size() << std::endl;
			//                        std::cout << "terminate route " << std::endl;
			//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			//OBS:"CAN GIVE PROBLEMS BECAUSE OF 'ITERATOR' "
			//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			sorted_routes.erase( sorted_routes.begin() );
			//                        std::cout << "route just terminated, now: " << sorted_routes.size() << std::endl;

		}
		//push new node to end of shortest and reposition the route in the sorted list of routes.
		if( num_options < 2 )
		{
			//                        std::cout << std::endl << "number of sorted routes: " << sorted_routes.size() << std::endl;
			//                        std::cout << "add to and reposition route " << std::endl;

			//update route
			sorted_routes.at(0).trail.push_back( sorted_routes.at(0).trail.at( sorted_routes.at(0).trail.size() - 1 )->neigh_rm_nodes.at( shortest ) );
			sorted_routes.at( 0 ).length += shortest_dist;

			//final destination. lets call it a day.
			//always 2 runs?!?! what to do instead?
			for( int i = 0; i < (int) sorted_routes.at( 0 ).trail.at( sorted_routes.at( 0 ).trail.size() - 1 )->neigh_rm_cells.size(); i++ )
			{
				if( compare_point_ts(
						cell_pos( sorted_routes.at( 0 ).trail.at( sorted_routes.at( 0 ).trail.size() - 1 )->neigh_rm_cells.at( i )->this_cell ),
						cell_pos( destination_cell->this_cell )
				)
				)
				{
					//return entire function by returning shortest route..
					at_destination = true;
					destination_point = edge_pos( sorted_routes.at( 0 ).trail.at(sorted_routes.at( 0 ).trail.size() - 1)->this_edge );
					final_length = sorted_routes.at( 0 ).length;
				}
			}

			//not final destination, move updated route to correct position in the ordered list of routes.
			//save updated route temporarily, remove it from sorted_routes.
			route moved_route = sorted_routes.at( 0 );
			sorted_routes.erase(sorted_routes.begin());
			//                        std::cout << "route just erased, now: " << sorted_routes.size() << std::endl;

			if(sorted_routes.size() == 0)
			{

				sorted_routes.push_back(moved_route);
				//                                std::cout << "route just pushed to only spot, now: " << sorted_routes.size() << std::endl;
			}
			else
			{

				int i = 0;
				moved_route.length;
				sorted_routes.at(i).length;
				if(moved_route.length > sorted_routes.at( sorted_routes.size() - 1 ).length)
				{
					sorted_routes.push_back(moved_route);
					//                                        std::cout << "route just pushed to last, now: " << sorted_routes.size() << std::endl;
				}
				else
				{
					while(moved_route.length > sorted_routes.at(i).length )
						i++;
					sorted_routes.insert( sorted_routes.begin() + i, moved_route );
					//                                        std::cout << "route just inserted, now: " << sorted_routes.size() << std::endl;
				}
			}
		}
		//fork: initiate new route based on parent and add newly found node
		else
		{
			//                        std::cout << std::endl << "number of sorted routes: " << sorted_routes.size() << std::endl;
			//                        std::cout << "fork route" << std::endl;

			route moved_route = sorted_routes.at( 0 );
			moved_route.trail.push_back( moved_route.trail.at( moved_route.trail.size() - 1 )->neigh_rm_nodes.at( shortest ) );
			moved_route.length += shortest_dist;

			//final destination. lets call it a day.
			//ALWAYS 2 cells?!?! what to do instead?
			for( int i = 0; i < (int) moved_route.trail.at( moved_route.trail.size() - 1 )->neigh_rm_cells.size(); i++ )
			{
				if( compare_point_ts(
						cell_pos( moved_route.trail.at( moved_route.trail.size() - 1 )->neigh_rm_cells.at( i )->this_cell ),
						cell_pos( destination_cell->this_cell )
				)
				)
				{
					//return entire function by returning shortest route..
					//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					// OBS:"MAYBE ALL VISITED FIELDS SHOULD BE RESET"
					//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
					at_destination = true;
					destination_point = edge_pos( moved_route.trail.at(moved_route.trail.size() - 1)->this_edge );
					final_length = moved_route.length;
				}
			}

			if(sorted_routes.size() == 0)
			{
				sorted_routes.push_back(moved_route);
				//                                std::cout << "route just pushed to only spot, now: " << sorted_routes.size() << std::endl;

			}
			else
			{
				int i = 0;
				int hejba1 = moved_route.length;
				int hejba2 = sorted_routes.at(i).length;
				if(moved_route.length > sorted_routes.at(sorted_routes.size() - 1).length )
				{
					sorted_routes.push_back(moved_route);
					//                                        std::cout << "route just pushed to last, now: " << sorted_routes.size() << std::endl;
				}
				else
				{
					while(moved_route.length > sorted_routes.at(i).length )
					{
						i++;
					}
					sorted_routes.insert( sorted_routes.begin() + i, moved_route );
					//                                        std::cout << "route just inserted, now: " << sorted_routes.size() << std::endl;
				}

			}
		}
	}
	//        std::cout << "length of final route:" << final_length << std::endl;
	return final_length;
}









//first connection to the rm is made from the special node of the trays.
void road_map::map( std::vector<pathPlanner::map::edge>* edges, std::vector<pathPlanner::map::cell>* cells )
{
	//std::cout << "lav liste af rm_noder/edges" << std::endl;
	//run through all edges
	for( int i = 0; i < (int) edges->size(); i++ )
	{
		//if soft edge, create node
		//std::cout << "edge i:" << i << std::endl;
		if( edges->at( i ).edgetype ==  pathPlanner::map::edge::EDGESOFT )
		{
			//std::cout << "        edge " << i << " er soft:" << std::endl;
			//create node
			rm_node this_node;
			this_node.this_edge = &edges->at( i );
			//add to list of nodes.
			node_list.push_back(this_node);
		}
	}
	//std::cout << "lav liste af rm_cell" << std::endl;
	//run through list of cells to create list of rm_cells.
	for( int i = 0; i < (int) cells->size(); i++ )
	{
		//create node
		rm_cell this_cell;
		this_cell.this_cell = &cells->at( i );
		//add soft_edges of this cell.
		//run through edges of cell.
		for( int j = 0; j < (int) this_cell.this_cell->edges.size(); j++ )
		{
			//if edge is soft
			if( this_cell.this_cell->edges.at( j )->edgetype == pathPlanner::map::edge::EDGESOFT )
			{
				//run through node_list to find corresponding rm_node
				for( int k = 0; k < (int) node_list.size(); k++ )
				{
					//save node
					if( compare_point_ts(
							edge_pos( node_list.at( k ).this_edge ),
							edge_pos( this_cell.this_cell->edges.at( j ) )
					)
					)
					{
						//save pointer to edge
						this_cell.neigh_rm_nodes.push_back( &node_list.at( k ) );
						//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
						//OBS:"MAYBE THIS BREAK DOES NOT BREAK THE RIGHT LOOP. iT SHOULD ONLY BREAK THE 'k' LOOP"
						break;
					}
				}
			}
		}
		//add to list of rm_cells
		cell_list.push_back(this_cell);
	}




	//run through all nodes, making all the necessary connections by comparing relatives to the list of existing nodes.
	//        std::cout << "node_list.size():" << node_list.size() << std::endl;
	//        std::cout << "cell_list.size():" << cell_list.size() << std::endl;
	for( int i = 0; i < (int) node_list.size(); i++ )
	{
		//                std::cout << "going through this Nodes cells.: " << node_list.at( i ).this_edge->cells.at(0)->cellId << std::endl;
		//                std::cout << " and " << node_list.at( i ).this_edge->cells.at(1)->cellId  << std::endl;
		//std::cout << "test:3" << std::endl;
		//run trough cells of node
		for( int j = 0; j < CELLS_PER_SOFT_EDGE; j++ )
		{
			//std::cout << "test:4" << std::endl;
			//                        std::cout << " going through this cell:cell_ID" << node_list.at( i ).this_edge->cells[j]->cellId << std::endl;
			//run through all edges of cell to locate soft edges.
			for( int k = 0; k < (int) node_list.at( i ).this_edge->cells[j]->edges.size(); k++ )
			{
				//                                std::cout << "locate soft edges." << std::endl;
				//if soft_edge
				if( node_list.at( i ).this_edge->cells[j]->edges.at( k )->edgetype == pathPlanner::map::edge::EDGESOFT &&
						!compare_point_ts( edge_pos( node_list.at( i ).this_edge->cells[j]->edges.at( k ) ), edge_pos( node_list.at( i ).this_edge ) )
				)
				{
					//                                        std::cout << "  edge=soft" << std::endl;
					//search node_list for the corresponding rm_node
					for( int l = 0; l < (int) node_list.size(); l++ )
					{
						//std::cout << "                find tilsvarende rm_node til softedge" << std::endl;
						if( compare_point_ts(
								edge_pos( node_list.at( l ).this_edge ),
								edge_pos( node_list.at( i ).this_edge->cells[j]->edges.at( k ) )
						)
						)
						{
							node_list.at( i ).neigh_rm_nodes.push_back( &node_list.at( l ) );
							//                                                        std::cout << "                  rm_node fundet og tilf��jet" << std::endl;
							//draw_line_pgm(edge_pos(node_list.at(i).this_edge), edge_pos(node_list.at( l ).this_edge));
							break;
						}
					}
				}
			}
			//                        std::cout << "tilf��j cells til node" << std::endl;
			//add neigh cells to node.
			for( int k = 0; k < (int) cell_list.size(); k++ )
			{
				//std::cout << "        celle #:" << k << "med cellID: " << cell_list.at( k ).this_cell->cellId << std::endl;
				if( compare_point_ts( cell_pos( node_list.at( i ).this_edge->cells[j] ), cell_pos( cell_list.at( k ).this_cell ) ) )
				{
					//                                        std::cout << "          forbind denne celle" << std::endl;
					if (cell_list.at(k).this_cell->cellId == 447)
					{
						point_t hej, kej;
						draw_line_pgm(hej, kej);
					}
					node_list.at( i ).neigh_rm_cells.push_back( &cell_list.at( k ) );
					//draw_line_pgm(edge_pos(node_list.at(i).this_edge), cell_pos(cell_list.at( k ).this_cell));
					break;
				}
			}
		}
	}
	//        std::cout << "cell_list.at(12).this_cell cellID" << cell_list.at(12).this_cell->cellId << std::endl;
	//        std::cout << "cell_list.at(12).this_cell cellID:" << cell_list.at(12).this_cell->cellId << std::endl;
	//        std::cout << "mapping done" << std::endl;
}








void road_map::draw_route(route the_route )
{
	for(int i = 0; i < the_route.trail.size() - 1; i++ )
	{
		draw_line_pgm( edge_pos( the_route.trail.at(i)->this_edge ), edge_pos( the_route.trail.at(i+1)->this_edge ) );
	}

}







void road_map::draw_line_pgm( point_t first_point, point_t second_point  )
{
	double distance_segment, distance;
	for( int i = 0; i < imggg->getWidth(); i++ )
	{
		for(int j = 0; j < imggg->getHeight(); j++ )
		{
			DistanceFromLine( i, j, first_point.x(), first_point.y(), second_point.x(), second_point.y(), distance_segment, distance );
			if( abs(distance_segment) < 0.10 )
			{
				imggg->setPixel8U( i, j, 200 );
			}
		}
	}

}

void road_map::DistanceFromLine(double cx, double cy, double ax, double ay ,
		double bx, double by, double &distanceSegment,
		double &distanceLine)
{

	//
	// find the distance from the point (cx,cy) to the line
	// determined by the points (ax,ay) and (bx,by)
	//
	// distanceSegment = distance from the point to the line segment
	// distanceLine = distance from the point to the line (assuming
	//                                      infinite extent in both directions
	//

	/*

Subject 1.02: How do I find the distance from a point to a line?


    Let the point be C (Cx,Cy) and the line be AB (Ax,Ay) to (Bx,By).
    Let P be the point of perpendicular projection of C on AB.  The parameter
    r, which indicates P's position along AB, is computed by the dot product
    of AC and AB divided by the square of the length of AB:

    (1)     AC dot AB
        r = ---------
            ||AB||^2

    r has the following meaning:

        r=0      P = A
        r=1      P = B
        r<0      P is on the backward extension of AB
        r>1      P is on the forward extension of AB
        0<r<1    P is interior to AB

    The length of a line segment in d dimensions, AB is computed by:

        L = sqrt( (Bx-Ax)^2 + (By-Ay)^2 + ... + (Bd-Ad)^2)

    so in 2D:

        L = sqrt( (Bx-Ax)^2 + (By-Ay)^2 )

    and the dot product of two vectors in d dimensions, U dot V is computed:

        D = (Ux * Vx) + (Uy * Vy) + ... + (Ud * Vd)

    so in 2D:

        D = (Ux * Vx) + (Uy * Vy)

    So (1) expands to:

            (Cx-Ax)(Bx-Ax) + (Cy-Ay)(By-Ay)
        r = -------------------------------
                          L^2

    The point P can then be found:

        Px = Ax + r(Bx-Ax)
        Py = Ay + r(By-Ay)

    And the distance from A to P = r*L.

    Use another parameter s to indicate the location along PC, with the
    following meaning:
           s<0      C is left of AB
           s>0      C is right of AB
           s=0      C is on AB

    Compute s as follows:

            (Ay-Cy)(Bx-Ax)-(Ax-Cx)(By-Ay)
        s = -----------------------------
                        L^2


    Then the distance from C to P = |s|*L.

	 */


	double r_numerator = (cx-ax)*(bx-ax) + (cy-ay)*(by-ay);
	double r_denomenator = (bx-ax)*(bx-ax) + (by-ay)*(by-ay);
	double r = r_numerator / r_denomenator;
	//
	double px = ax + r*(bx-ax);
	double py = ay + r*(by-ay);
	//
	double s =  ((ay-cy)*(bx-ax)-(ax-cx)*(by-ay) ) / r_denomenator;

	distanceLine = fabs(s)*sqrt(r_denomenator);

	//
	// (xx,yy) is the point on the lineSegment closest to (cx,cy)
	//
	double xx = px;
	double yy = py;

	if ( (r >= 0) && (r <= 1) )
	{
		distanceSegment = distanceLine;
	}
	else
	{

		double dist1 = (cx-ax)*(cx-ax) + (cy-ay)*(cy-ay);
		double dist2 = (cx-bx)*(cx-bx) + (cy-by)*(cy-by);
		if (dist1 < dist2)
		{
			xx = ax;
			yy = ay;
			distanceSegment = sqrt(dist1);
		}
		else
		{
			xx = bx;
			yy = by;
			distanceSegment = sqrt(dist2);
		}


	}

	return;
}
