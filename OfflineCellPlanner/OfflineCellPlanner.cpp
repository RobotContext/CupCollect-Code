#include "OfflineCellPlanner.hpp"

namespace pathPlanner
{
  OfflineCellPlanner::OfflineCellPlanner(Img * map, pathPlanner::map * vectormap, point_t topLeft, point_t bottomRight)
  {
    // Test limits


    // Object proberties
    this->themap = vectormap;

    // Internal variables
    point_t curPoint;
    point_t * curVertice;
    cornerType corner;

    findCells(map);
    /*
    // Find hard egdes
    // Start looping through points
    for (int curY = topLeft.y(); curY < bottomRight.y(); curY++)
      {
	curPoint.y(curY);
	for (int curX = topLeft.x(); curX < bottomRight.x(); curX++)
	  {
	    curPoint.x(curX);

	    // Is this a corner?
	    corner = isCorner(map, curPoint);

	    if (corner != NoCorner)
	      {
		curVertice = themap->verticeAdd(curPoint);
		
		findEdgeHorisontal(corner, curVertice);
		findEdgeVertical(corner, curVertice);
	      }
	  }
      }
    */

    // Find soft edges
    /*std::vector<point_t>::iterator it;
    for (it = themap->vertices.begin(); it < themap->vertices.end(); it++)
      {
	// corner type?
	corner = isCorner(map, *it);
	if (corner != NoCorner)
	  {
	    findEdgeSoft(corner, &(*it));
	  }
	  }*/
    
  }

  int OfflineCellPlanner::isICornerUpperLeft(Img * map, point_t point)
  {
    // Situation we are looking for
    // X #
    // # .
    if (
	(isWall(map, point)) && (isWall(map, point_t(point.x()+1, point.y()))) &&
	(isWall(map, point_t(point.x(), point.y()+1))) && (!isWall(map, point_t(point.x()+1, point.y()+1)))
	)
      return true;
    else
      return false;
  }
  int OfflineCellPlanner::isICornerUpperRight(Img * map, point_t point)
  {
    // Situation we are looking for
    // # X
    // . #
    if (
	(isWall(map, point_t(point.x()-1, point.y()))) && (isWall(map, point_t(point.x(), point.y()))) &&
	(!isWall(map, point_t(point.x()-1, point.y()+1))) && (isWall(map, point_t(point.x(), point.y()+1)))
	)
      return true;
    else
      return false;

  }
  int OfflineCellPlanner::isICornerLowerLeft(Img * map, point_t point)
  {
    // Situation we are looking for
    // # .
    // X #
    if (
	(isWall(map, point_t(point.x(), point.y()-1))) && (isFloor(map, point_t(point.x()+1, point.y()-1))) &&
	(isWall(map, point_t(point.x(), point.y()))) && (isWall(map, point_t(point.x()+1, point.y())))
	)
      return true;
    else
      return false;
  }
  int OfflineCellPlanner::isICornerLowerRight(Img * map, point_t point)
  {
    // Situation we are looking for
    // . #
    // # X
    if (
	(isFloor(map, point_t(point.x()-1, point.y()-1))) && (isWall(map, point_t(point.x(), point.y()-1))) &&
	(isWall(map, point_t(point.x()-1, point.y()))) && (isWall(map, point_t(point.x(), point.y())))
	)
      return true;
    else
      return false;
  }
  int OfflineCellPlanner::isOCornerUpperLeft(Img * map, point_t point)
  {
    // Situation we are looking for
    // X .
    // . .
    if (
	(isWall(map, point_t(point.x(), point.y()))) && (isFloor(map, point_t(point.x()+1, point.y()))) &&
	(isFloor(map, point_t(point.x(), point.y()+1))) && (isFloor(map, point_t(point.x()+1, point.y()+1)))
	)
      return true;
    else
      return false;
  }
  int OfflineCellPlanner::isOCornerUpperRight(Img * map, point_t point)
  {
    // Situation we are looking for
    // . X
    // . .
    if (
	(isFloor(map, point_t(point.x()-1, point.y()))) && (isWall(map, point_t(point.x(), point.y()))) &&
	(isFloor(map, point_t(point.x()-1, point.y()+1))) && (isFloor(map, point_t(point.x(), point.y()+1)))
	)
      return true;
    else
      return false;
  }
  int OfflineCellPlanner::isOCornerLowerLeft(Img * map, point_t point)
  {
    // Situation we are looking for
    // . .
    // X .
    if (
	(isFloor(map, point_t(point.x(), point.y()-1))) && (isFloor(map, point_t(point.x()+1, point.y()-1))) &&
	(isWall(map, point_t(point.x(), point.y()))) && (isFloor(map, point_t(point.x()+1, point.y())))
	)
      return true;
    else
      return false;
  }
  int OfflineCellPlanner::isOCornerLowerRight(Img * map, point_t point)
  {
    // Situation we are looking for
    // . .
    // . X
    if (
	(isFloor(map, point_t(point.x()-1, point.y()-1))) && (isFloor(map, point_t(point.x(), point.y()-1))) &&
	(isFloor(map, point_t(point.x()-1, point.y()))) && (isWall(map, point_t(point.x(), point.y())))
	)
      return true;
    else
      return false;
  }
  enum OfflineCellPlanner::cornerType OfflineCellPlanner::isCorner(Img * map, point_t point)
  {
    // Can be optimized a lot ...
    if (isICornerUpperLeft(map, point))
      return ICornerUpperLeft;
    else if (isICornerUpperRight(map, point))
      return ICornerUpperRight;
    else if (isICornerLowerLeft(map, point))
      return ICornerLowerLeft;
    else if (isICornerLowerRight(map, point))
      return ICornerLowerRight;
    else if (isOCornerUpperLeft(map, point))
      return OCornerUpperLeft;
    else if (isOCornerUpperRight(map, point))
      return OCornerUpperRight;
    else if (isOCornerLowerLeft(map, point))
      return OCornerLowerLeft;
    else if (isOCornerLowerRight(map, point))
      return OCornerLowerRight;
    else
      return NoCorner;
  }


  void OfflineCellPlanner::findCells(Img * map)
  {
    // Find first floor pixel on the map.
    int maxX = map->getWidth();
    int maxY = map->getHeight();
    int cellMinX, cellMinY, cellMaxX, cellMaxY;
    int searchX, searchY;
    int cellCount = 0;
    point_t * topLeft, * topRight;
    point_t * bottomLeft, * bottomRight;
    point_t * startVertice, * curVertice, * nextVertice;

    map::cell * workcell;

    for (int x = 0; x < maxX; x++)
      {
	for (int y = 0; y < maxY; y++)
	  {
	    if (isFloor(map, point_t(x,y)))
	      {
		cellMinX = x;
		cellMinY = y;
		enum {CELLEXPANDTOP, CELLEXPANDBOTTOM, CELLWALL} cellLimit;

		workcell = themap->cellAdd();

		// Find max y
		for (searchY = cellMinY; searchY < maxY; searchY++)
		  {
		    if (!isFloor(map, point_t(x, searchY)))
		      {
			// Wall found
			cellMaxY = searchY-1;
			break;
		      }
		  }
		//std::cout << "Found callMaxY: " << cellMaxY << std::endl;
		
		// Try to extend in x direction	
		for (searchX = cellMinX+1; searchX < maxX; searchX++)
		  {
		    cellMaxX = 0;
	
		    // If there is floor here, area is expanding upwards.. Abort!
		    if (isFloor(map, point_t(searchX, cellMinY-1)))
		      {
			cellMaxX = searchX-1;
			cellLimit = CELLEXPANDTOP;
			//std::cout << "Cell expanding top" << std::endl;
			break;
		      }
		    // Run the whole y direction and test
		    for (int searchY = cellMinY; searchY <= cellMaxY; searchY++)
		      {
			if (!isFloor(map, point_t(searchX, searchY)))
			  {
			    cellMaxX = searchX -1;
			    cellLimit = CELLWALL;
			    //std::cout << "Cell wall" << std::endl;
			    break;
			  }
		      }
		    // Test if the area is expanding downwards
		    if (isFloor(map, point_t(searchX, cellMaxY+1)))
		      {
			cellMaxX = searchX -1;
			cellLimit = CELLEXPANDBOTTOM;
			//std::cout << "Cell expanding bottom" << std::endl;
			break;
		      }
		    if (cellMaxX != 0)
		      break;
		  }
		// Cell is defined.
		// Color it on the map
		colorMap(map, point_t(cellMinX, cellMinY), point_t(cellMaxX, cellMaxY), 180);

		// Expand to vertices and edges
		//if (isICornerUpperLeft(map,point_t(cellMinX-1, cellMinY-1)))
		if (
		    (isWall(map, point_t(cellMinX-1, cellMinY-1))) && 
		    (themap->verticeExists(point_t(cellMinX, cellMinY-1)) == (point_t *)0)
		    )
		  {
		    cellMinX--;
		  }
		cellMinY--;


		startVertice = curVertice = themap->verticeAdd(point_t(cellMinX, cellMinY));

		// run arround along the edges
		enum map::edge::edgetypes edgestate = map::edge::EDGEHARD;
		// Run right --------------------------------------------------------------------------------
		for (searchX = cellMinX; searchX <= cellMaxX; searchX++)
		  {
		    if ((map::edge::EDGEHARD == edgestate) && (!isWall(map, point_t(searchX, cellMinY))))
		      {
			// End of hardedge
			nextVertice = themap->verticeAdd(point_t(searchX-1, cellMinY));
			themap->cellAddEdge(workcell, curVertice, nextVertice, map::edge::EDGEHARD);
			curVertice = nextVertice;
			edgestate = map::edge::EDGESOFT;
		      }
		    else if ((map::edge::EDGESOFT == edgestate) && (isWall(map, point_t(searchX, cellMinY))))
		      {
			// End of softedge
			nextVertice = themap->verticeAdd(point_t(searchX, cellMinY));
			themap->cellAddEdge(workcell, curVertice, nextVertice, map::edge::EDGESOFT);
			curVertice = nextVertice;
			edgestate = map::edge::EDGEHARD;
		      }
		  }

		// If the next pixel is a corner, then move edge in to it.
		//if ((isICornerUpperRight(map, point_t(cellMaxX+1, cellMinY))))
		if (cellLimit == CELLWALL)
		  {
		    cellMaxX = cellMaxX+1;
		    //std::cout << "Found corner at: " << cellMaxX << " x " << cellMinY << std::endl;
		  }

		// And upper right corner
		nextVertice = themap->verticeAdd(point_t(cellMaxX, cellMinY));
		//std::cout << "Upper right corner: " << cellMaxX << " x " << cellMinY << std::endl;

		themap->cellAddEdge(workcell, curVertice, nextVertice, edgestate);
		curVertice = nextVertice;


		// Run down ----------------------------------------------------------------------------
		for (searchY = cellMinY; searchY <= cellMaxY; searchY++)
		  {

		    if ((map::edge::EDGEHARD == edgestate) && (!isWall(map, point_t(cellMaxX, searchY))))
		      {
			//std::cout << "No wall at: " << cellMaxX << " x " << searchY << " ending hardwall" << std::endl;

			// End of hardedge
			nextVertice = themap->verticeAdd(point_t(cellMaxX, searchY-1));
			themap->cellAddEdge(workcell, curVertice, nextVertice, map::edge::EDGEHARD);
			curVertice = nextVertice;
			edgestate = map::edge::EDGESOFT;
		      }
		    else if ((map::edge::EDGESOFT == edgestate) && (isWall(map, point_t(cellMaxX, searchY))))
		      {
			// End of softedge
			nextVertice = themap->verticeAdd(point_t(cellMaxX, searchY));
			themap->cellAddEdge(workcell, curVertice, nextVertice, map::edge::EDGESOFT);
			curVertice = nextVertice;
			edgestate = map::edge::EDGEHARD;;
		      }
		  }
		// We will always hit a wall in this situation
		// Go into it

		
		cellMaxY++; // = searchY +1;
		nextVertice = themap->verticeAdd(point_t(cellMaxX, cellMaxY));
		themap->cellAddEdge(workcell, curVertice, nextVertice, edgestate);
		curVertice = nextVertice;


		// Run left
		for (searchX = cellMaxX; searchX >= cellMinX; searchX--)
		  {
		    if ((map::edge::EDGEHARD == edgestate) && (!isWall(map, point_t(searchX, cellMaxY))))
		      {
			// End of hardedge
			nextVertice = themap->verticeAdd(point_t(searchX+1, cellMaxY));
			themap->cellAddEdge(workcell, curVertice, nextVertice, map::edge::EDGEHARD);
			curVertice = nextVertice;
			edgestate = map::edge::EDGESOFT;
		      }
		    else if ((map::edge::EDGESOFT == edgestate) && (isWall(map, point_t(searchX, cellMaxY))))
		      {
			// End of softedge
			nextVertice = themap->verticeAdd(point_t(searchX, cellMaxY));
			themap->cellAddEdge(workcell, curVertice, nextVertice, map::edge::EDGESOFT);
			curVertice = nextVertice;
			edgestate = map::edge::EDGEHARD;
		      }
		  }

		// Add corner
		nextVertice = themap->verticeAdd(point_t(cellMinX, cellMaxY));
		themap->cellAddEdge(workcell, curVertice, nextVertice, edgestate);
		curVertice = nextVertice;
		
		
		// Run up
		for (searchY = cellMaxY; searchY >= cellMinY; searchY--)
		  {

		    if ((map::edge::EDGEHARD == edgestate) && (!isWall(map, point_t(cellMinX, searchY))))
		      {
			// End of hardedge
			nextVertice = themap->verticeAdd(point_t(cellMinX, searchY+1));
			themap->cellAddEdge(workcell, curVertice, nextVertice, map::edge::EDGEHARD);
			curVertice = nextVertice;
			edgestate = map::edge::EDGESOFT;
		      }
		    else if ((map::edge::EDGESOFT == edgestate) && (isWall(map, point_t(cellMinX, searchY))))
		      {
			// End of softedge
			nextVertice = themap->verticeAdd(point_t(cellMinX, searchY));
			themap->cellAddEdge(workcell, curVertice, nextVertice, map::edge::EDGESOFT);
			curVertice = nextVertice;
			edgestate = map::edge::EDGEHARD;
		      }
		  }
		
		// Final link in cell
		themap->cellAddEdge(workcell, curVertice, startVertice, edgestate);

		/*
		if (isWall(map, point_t(cellMaxX+1, cellMaxY)))
		  cellMaxX += 1;
		if (isWall(map, point_t(cellMaxY, cellMaxY+1)))
		  cellMaxY += 1;
		*/
		/*topLeft = themap->verticeAdd(point_t(cellMinX, cellMinY));
		topRight = themap->verticeAdd(point_t(cellMaxX, cellMinY));
		bottomLeft = themap->verticeAdd(point_t(cellMinX, cellMaxY));
		bottomRight = themap->verticeAdd(point_t(cellMaxX, cellMaxY));

		themap->edgeAdd(topLeft, topRight, map::edge::EDGEHARD);
		themap->edgeAdd(topLeft, bottomLeft, map::edge::EDGEHARD);
		themap->edgeAdd(topRight, bottomRight, map::edge::EDGEHARD);
		themap->edgeAdd(bottomLeft, bottomRight, map::edge::EDGEHARD);
		*/

		cellCount++;
		if (cellCount > 1500)
		  return; // debug
	      }
	  }
      }
    std::cout << cellCount << " cells found" << std::endl;

  }





  int OfflineCellPlanner::isWall(Img * map, point_t point)
  {
    // Test for out of bounds
    if ((point.x() >= map->getWidth()) || (point.y() >= map->getHeight()))
      {
	std::cout << "Out of bounds call to isWall()" << std::endl;
	return true;
      }
    //std::cout << "isWall: " << point.x() << " x " << point.y() << std::endl;
    if (colorWall == map->getPixelValuei(point.x(), point.y(), 0))
      return true;
    else
      return false;
  }
  int OfflineCellPlanner::isFloor(Img * map, point_t point)
  {
    // Test for out of bounds
    if ((point.x() >= map->getWidth()) || (point.y() >= map->getHeight()))
      {
	std::cout << "Out of bounds call to isFloor()" << std::endl;
	return false;
      }
    //std::cout << "isFloor: " << point.x() << " x " << point.y() << std::endl;
    if (colorFloor == map->getPixelValuei(point.x(), point.y(), 0))
      return true;
    else
      return false;
  }
  int OfflineCellPlanner::isSoftwall(Img * outmap, point_t point)
  {
    if (colorMarkerSoftwall == outmap->getPixelValuei(point.x(), point.y(), 0))
      return true;
    else
      return false;
  }
  void OfflineCellPlanner::colorMap(Img * map, point_t min, point_t max, int color)
  {
    //std::cout << "ColorMap: " << min.x() << " x " << min.y() << " - " << max.x() << " x " << max.y() << std::endl;
    for (int x = min.x(); x <= max.x(); x++)
      {
	for (int y = min.y(); y <= max.y(); y++)
	  {
	    map->setPixel8U(x, y, color);
	  }
      }
  }
  void OfflineCellPlanner::findEdgeVertical(enum cornerType corner, point_t * vertice)
  {
    static std::list<int> verticalVertexStartPoints;
    static std::list<int>::iterator findVerticalVertexStartPoint;
    int curX, curY;
    curX = vertice->x();
    curY = vertice->y();
    point_t * endVertice;

    findVerticalVertexStartPoint = std::find(verticalVertexStartPoints.begin(), verticalVertexStartPoints.end(), curX);
    
    if (findVerticalVertexStartPoint == verticalVertexStartPoints.end())
      {
	// This is a vertical start point!
	verticalVertexStartPoints.push_back(curX);
      }
    else
      {
	// This is a vertical wall endpoint
	// Remove startpoint from list
	verticalVertexStartPoints.erase(findVerticalVertexStartPoint);

	// Draw a line to the start point
	for (int i = 1; i < 1000 && ((curY - i) >= 0); i++)
	  {
	    if (themap->edgePoint(point_t(curX, curY-i)) != (map::edge *)0)
	      {
		// We found it!
		endVertice = themap->verticeAdd(point_t(curX, curY-i));
		themap->edgeAdd(vertice, endVertice, map::edge::EDGEHARD);
		break;
	      }
	  }
      }

  }

  void OfflineCellPlanner::findEdgeHorisontal(enum cornerType corner, point_t * vertice)
  {
    static int lastY;
    static int isHorisontalEdge = 0;
    static point_t * startVertice;

    if (lastY != vertice->y())
      isHorisontalEdge = 0;

    if (0 == isHorisontalEdge)
      {
	isHorisontalEdge = 1;
	startVertice = vertice;
      }
    else
      {
	// We are ending a vertex
	isHorisontalEdge = 0;
	// Add edge
	themap->edgeAdd(startVertice, vertice, map::edge::EDGEHARD);
      }
    lastY = vertice->y();
  }
  void OfflineCellPlanner::findEdgeSoft(enum cornerType corner, point_t * vertice)
  {
    int curX, curY;
    point_t * nextVertice;
    curX = vertice->x();
    curY = vertice->y();

    /* ----------------- SOFTWALL DETECTION ------------------- */
    if ((corner == OCornerLowerLeft) | (corner == OCornerLowerRight))
      {
	// This a lower outer corner.
	// Make a softwall
	for (int i = 1; (i < 1000) && ((curY -i) >= 0); i++)
	  {
	    // Run until we hit an edge
	    if (themap->edgePoint(point_t(curX, curY-i)) != (map::edge *)0)
	      {
		nextVertice = themap->verticeAdd(point_t(curX, curY-i));
		themap->edgeAdd(vertice, nextVertice, map::edge::EDGESOFT);
		break;
	      }
	  }
      } 
    else if ((corner == OCornerUpperLeft) | (corner == OCornerUpperRight))
      {
	//if (!isSoftwall(buffermap, point_t(curX, curY+1)))
	if (themap->edgePoint(point_t(curX, curY+1)) == (map::edge *)0)
	  {
	    std::cout << "At " << curX << " x " << curY << std::endl;
	    // Test if this softwall already exist
	    for (int i = 1; (i < 1000); i++) // Remember to put limiter test here
	      {
		// OLD
		/*
		if ((curY +5) > buffermap->getHeight())
		  {
		    std::cout << "Hit the image wall at: " << curX << " x " << curY << std::endl;
		    break;
		  }
		*/
		// Run until we hit an edge
		if (themap->edgePoint(point_t(curX, curY+i)) != (map::edge *)0)
		  {
		    nextVertice = themap->verticeAdd(point_t(curX, curY-i));
		    themap->edgeAdd(vertice, nextVertice, map::edge::EDGESOFT);
		    break;
		  }
	      }
	  }
      }
    
  }
}
