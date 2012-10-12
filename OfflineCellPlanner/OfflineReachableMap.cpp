#include "OfflineReachableMap.hpp"

void OfflineReachableMap::makeReachableMap(Img * map, Img * reachableMap, point_t startPoint)
{
  // Front wave
  int count = 0;
  point_t currentPoint;
  std::stack<point_t> front;

  // Add current point to front
  front.push(startPoint);

  while ((!front.empty()) && (count++ < 5000000))
    {
      // Pop point from stack
      currentPoint = front.top();
      front.pop();

      // Mark point as reachable
      setReachable(reachableMap, currentPoint);

      // Add new valid points to stack
      for (int x = -1; x <= 1; x++)
	{
	  for (int y = -1; y <= 1; y++)
	    {
	      if (!((0 == x) && (0 == y)))
		{
		  if (doSearchField(map, reachableMap, point_t(currentPoint.x()+x, currentPoint.y()+y)))
		    {
		      // This is not the wall
		      front.push(point_t(currentPoint.x()+x, currentPoint.y()+y));
		      setOnStack(reachableMap, point_t(currentPoint.x()+x, currentPoint.y()+y));
		    }
		}
	    }
	}
    }

  // Extend the reachable by following the edge of the reachable map and walls on the map.
  

  // Find wall
  //int max_tries = 1000;
  //for (int ctry = 0; (ctry < max_tries) && (map->getPixelValue(searchPoint.x(), searchPoint.y(), 0) > 0); ctry++) {}
}


int OfflineReachableMap::doSearchField(Img * map, Img * reachableMap, point_t point)
{
  // Is this a wall?
  if (colorWall == map->getPixelValuei(point.x(), point.y(), 0))
    return false;
  else if (colorMarkerReachable == reachableMap->getPixelValuei(point.x(), point.y(), 0))
    return false;
  else if (colorMarkerOnstack == reachableMap->getPixelValuei(point.x(), point.y(), 0)) // Already marked for seaching
    return false;
  else
    return true;
}
void OfflineReachableMap::setReachable(Img * map, point_t point)
{
  map->setPixel8U(point.x(), point.y(), colorMarkerReachable);
}
void OfflineReachableMap::setOnStack(Img * reachableMap, point_t point)
{
  reachableMap->setPixel8U(point.x(), point.y(), colorMarkerOnstack);
}
void OfflineReachableMap::makeWorkspaceMap(Img * reachableMap, Img * workspaceMap)
{
  const int radius = 4;
  // Paint the workspace map white (Reachable)
  int mapW = workspaceMap->getWidth();
  int mapH = workspaceMap->getHeight();
  for (int x = 0; x < mapW; x++)
    {
      for (int y = 0; y < mapH; y++)
	{
	  setReachable(workspaceMap, point_t(x,y));
	}
    }

  for (int x = radius; x < (mapW - radius); x++)
    {
      for (int y = radius; y < (mapH - radius); y++)
	{
	  if (colorMarkerReachable != reachableMap->getPixelValuei(x,y,0))
	    {
	      for (int cx = (-1*radius); cx <= radius; cx++)
		for (int cy = (-1*radius); cy <= radius; cy++)
		  workspaceMap->setPixel8U(x+cx, y+cy, colorWall);
	    }
	}
    }
  
}
