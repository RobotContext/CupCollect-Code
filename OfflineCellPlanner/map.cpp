#include "map.hpp"

namespace pathPlanner
{
  point_t* map::verticeAdd(point_t point)
  {
    point_t * out;
    edge * myEdge;
    // Test if it already exist
    out = verticeExists(point);

    // If its a null pointer, add the point
    if (((point_t *)0) == out)
      {
	vertices.push_back(point);
	out = &(vertices.back());
      }
    /*
    else
      {
	// If we are placing this vertice on an existing edge, add it to the edge
	myEdge = edgePoint(point);
	if ((edge*)0 != myEdge)
	  {
	    
	  }
      }
    */

    // Return element pointer
    return out;
  }
  point_t* map::verticeExists(point_t point)
  {
    //std::vector<point_t>::iterator it = std::find(vertices.begin(), vertices.end(), point);
    std::vector<point_t>::iterator it;
    for (it = vertices.begin(); it < vertices.end(); it++)
      {
	if ((it->x() == point.x()) && (it->y() == point.y())) // ....
	  return &(*it);
      }
      return (point_t *) 0;
  }

  map::edge* map::edgePoint(point_t testPoint)
  {
    point_t start;
    point_t next;
    std::vector<edge>::iterator eit = edges.begin();
    std::vector<point_t *>::iterator pit;
    for (; eit < edges.end(); eit++)
      {
	pit = eit->vertices.begin();
	for (; (pit+1) < eit->vertices.end(); pit++)
	  {
	    start = *(*pit);
	    next = *(*(pit+1));

	    // Assuming all lines are straight
	    if (
		(start.x() == next.x()) && (testPoint.x() == start.x())
		&& (
		    ((start.y() >= testPoint.y()) && (next.y() <= testPoint.y())) ||
		    ((start.y() <= testPoint.y()) && (next.y() >= testPoint.y()))
		    )
		)
	      return &(*eit);
	    
	    if (
		(start.y() == next.y()) && (testPoint.y() == start.y())
		&& (
		    ((start.x() >= testPoint.x()) && (next.x() <= testPoint.x())) ||
		    ((start.x() <= testPoint.x()) && (next.x() >= testPoint.x()))
		    )
		)
	      return &(*eit);
	  }
      }
    return (edge *)0;
  }

  map::edge* map::edgeAdd(point_t * startVertice, point_t * stopVertice, map::edge::edgetypes edgetype)
  {
    map::edge * out;
    map::edge e;
    // Test for null length
    if (startVertice == stopVertice)
      {
	//std::cout << "Null edge detected" << std::endl;
	return (map::edge *)0;
      }

    // test if the edge already exists
    out = edgeExists(startVertice, stopVertice);
    if (out != (map::edge *)0)
      {
	//std::cout << "Found existing edge" << std::endl;
	return out; // Return existing edge
      }
      

    e.vertices.push_back(startVertice);
    e.vertices.push_back(stopVertice);
    //    std::cout << "Start: " << startVertice->x() << " x " << startVertice->y() << std::endl;
    //    std::cout << "Stop: " << stopVertice->x() << " x " << stopVertice->y() << std::endl;
    e.edgetype = edgetype;
    edges.push_back(e);
    //out = &(*(edges.end())); // hate writing this..
    out = &(edges.back());
    return out;
  }
  map::edge* map::edgeExists(point_t * startVertice, point_t * stopVertice)
  {
    int hit;
    std::vector<edge>::iterator eit = edges.begin();
    std::vector<point_t *>::iterator pit;
    // Loop edges
    for (; eit < edges.end(); eit++)
      {
	hit = 0;
	// Loop vertices
	for (
	     pit = eit->vertices.begin(); 
	     pit < eit->vertices.end(); 
	     pit++)
	  {
	    if ((*pit) == startVertice)
	      hit++;
	    if ((*pit) == stopVertice)
	      hit++;
	    if (2 == hit)
	      return &(*eit);
	  }
      }
    return (map::edge *)0;
  }
  map::cell * map::cellAdd()
  {
    static int cellcount = 0;
    map::cell c;
    c.cellId = cellcount++;
    cells.push_back(c);
    return &(cells.back());
  }
  void map::cellAddEdge(cell * c, edge * e)
  {
    // Add edge to cell
    c->edges.push_back(e);

    // Add cell to edge
    e->cells.push_back(c);
  }
  void map::cellAddEdge(cell * c, point_t * startVertice, point_t * stopVertice, map::edge::edgetypes edgetype)
  {
    edge * e;
    e = edgeAdd(startVertice, stopVertice, edgetype);
    if (e != (edge *)0)
      cellAddEdge(c, e);
  }
  void map::drawMap(Img * map)
  {
    //drawVertices(map, 128);
    drawEdges(map, 160, 70);
    drawVertices(map, 128);
  }
  void map::drawVertices(Img * bitmap, int color)
  {
    // Run through all vertices and plot on the supplied map
    std::vector<point_t>::iterator it = vertices.begin();
    for (; it < vertices.end(); it++)
      {
	bitmap->setPixel8U(it->x(), it->y(), color);
      }
  }
  void map::drawEdges(Img * bitmap, int colorHard, int colorSoft)
  {
    std::vector<edge>::iterator it = edges.begin();
    for (; it < edges.end(); it++)
      {
	if (edge::EDGEHARD == it->edgetype)
	  drawEdge(bitmap, &(*it), colorHard);
	else
	  drawEdge(bitmap, &(*it), colorSoft);
      }
  }
  void map::drawEdge(Img * bitmap, edge * e, int color)
  {
    point_t pen;
    point_t next;
    std::vector<point_t *>::iterator it = e->vertices.begin();
    for (; (it+1) < e->vertices.end(); it++)
      {
	pen = *(*it);
	next = *(*(it+1));

	//continue; // HERE!!

	// Test for null length
	if ((std::abs(pen.x() - next.x()) > 0) && (std::abs(pen.y() - next.y()) > 0))
	  {
	    std::cout << "Drawing a rotated line" << std::endl;
	    std::cout << "Pen: " << pen.x() << " x " << pen.y() << std::endl;
	    std::cout << "Next: " << next.x() << " x " << next.y() << std::endl;
	  }
	//std::cout << "Pen: " << pen.x() << " x " << pen.y() << " to ";
	//std::cout << "Next: " << next.x() << " x " << next.y() << std::endl;

	for (int c = 0; c < 10000; c++) // Magic number!
	  {
	    if (pen.x() < next.x())
	      pen.x(pen.x()+1);
	    else if (pen.x() > next.x())
	      pen.x(pen.x()-1);

	    
	    if (pen.y() > next.y())
	      pen.y(pen.y()-1);
	    else if (pen.y() < next.y())
	      pen.y(pen.y()+1);      

	    if ((pen.x() == next.x()) && (pen.y() == next.y()))
	      break;

	    bitmap->setPixel8U(pen.x(), pen.y(), color);
	  }
	
      }
    
  }
  void map::printCellInfo(cell * c)
  {
    std::vector<edge *>::iterator it;
    std::cout << "Printing all known info about cell " << c->cellId << std::endl;
    std::cout << "Cell eges:" << std::endl;
    for (it = c->edges.begin(); it < c->edges.end(); it++)
      printEdgeInfo((*it));
  }
  void map::printEdgeInfo(edge * e)
  {
    std::vector<point_t *>::iterator it;
    std::vector<cell *>::iterator cit;
    int first = 1;
    std::cout << "\t";
    if (e->edgetype == edge::EDGEHARD)
      std::cout << "Hardedge: \t";
    else
      std::cout << "Softedge: \t";

    std::cout << "(";
    for (cit = e->cells.begin(); cit < e->cells.end(); cit++)
      {
	if (first)
	  first = 0;
	else
	  std::cout << ", ";

	std::cout << (*cit)->cellId;
      }
    std::cout << ")\t";

    for (it = e->vertices.begin(); it < e->vertices.end(); it++)
      std::cout << (*it)->x() << "x" << (*it)->y() << "\t";
    std::cout << std::endl;
  }

  void map::saveImage( Img * img , std::basic_string<char, std::char_traits<char>, std::allocator<char> > path )
  {
  	// save image
  	img->saveAsPGM(path);
  	std::cout << "File saved" << std::endl;
  }
}
