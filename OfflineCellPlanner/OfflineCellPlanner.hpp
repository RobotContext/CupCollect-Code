#ifndef OFFLINECELLPLANNER_HPP
#define OFFLINECELLPLANNER_HPP
#include <list>
#include <stack>
#include <algorithm>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include "map.hpp"
#include "Image.hpp"
#include "PPMLoader.hpp"
#include "map.hpp"

namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<int> point_t;
typedef rw::sensor::Image Img;

/*
 * Move corner stuff to edge finder class
 * Make linked list of possible corner points * that will be populated when we find a new corner, that extends the area beond the cell.
 
 */
namespace pathPlanner
{
  class OfflineCellPlanner
  {
  public:
    enum cornerType {NoCorner = 0, ICornerUpperLeft = 1, ICornerUpperRight = 2, ICornerLowerLeft = 3, ICornerLowerRight = 4, OCornerUpperLeft = 5, OCornerUpperRight = 5, OCornerLowerLeft = 6, OCornerLowerRight = 7};

    OfflineCellPlanner(Img * map, pathPlanner::map * themap, point_t topLeft, point_t bottomRight);
    static int isICornerUpperLeft(Img * map, point_t point);
    static int isICornerUpperRight(Img * map, point_t point);
    static int isICornerLowerLeft(Img * map, point_t point);
    static int isICornerLowerRight(Img * map, point_t point);
    static int isOCornerUpperLeft(Img * map, point_t point);
    static int isOCornerUpperRight(Img * map, point_t point);
    static int isOCornerLowerLeft(Img * map, point_t point);
    static int isOCornerLowerRight(Img * map, point_t point);
    static enum cornerType isCorner(Img * map, point_t point);

    void findCells(rw::sensor::Image * map);
  
  private:
    pathPlanner::map * themap;

    // consts
    const static int colorWall = 0;
    const static int colorFloor = 255;
    const static int colorMarkerCorner = 128;
    const static int colorMarkerWall = 255;
    const static int colorMarkerSoftwall = 60;

    // probe map
    static int isWall(Img * map, point_t point);
    static int isFloor(Img * map, point_t point);
    static int isSoftwall(Img * outmap, point_t point);
    
    // Tools
    static void colorMap(Img * map, point_t min, point_t max, int color);

    // actual finding stuff
    void findEdgeVertical(enum cornerType corner, point_t * pos);
    void findEdgeHorisontal(enum cornerType corner, point_t * pos);
    void findEdgeSoft(enum cornerType corner, point_t * pos);
  };
}
#endif
