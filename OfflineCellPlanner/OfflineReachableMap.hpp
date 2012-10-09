#ifndef OFFLINEREACHABLEMAP_HPP
#define OFFLINEREACHABLEMAP_HPP

#include <stack>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include "PPMLoader.hpp"

typedef boost::geometry::model::d2::point_xy<int> point_t;
typedef rw::sensor::Image Img;


class OfflineReachableMap
{
public:
  static void makeReachableMap(Img * map, Img * reachableMap, point_t startPoint);
  static int doSearchField(Img * map, Img * reachableMap, point_t point); // Does it make sense to search this field?
  static void setReachable(Img * map, point_t point);
  static void setOnStack(Img * reachableMap, point_t point);
  static void makeWorkspaceMap(Img * reachableMap, Img * workspaceMap);

private:
  const static int colorWall = 0;
  const static int colorMarkerReachable = 255;
  const static int colorMarkerOnstack = 200;

};


#endif
