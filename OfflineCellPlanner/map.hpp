#ifndef MAP_H
#define MAP_H

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <vector>
#include "Image.hpp"

typedef boost::geometry::model::d2::point_xy<int> point_t;
typedef rw::sensor::Image Img;

namespace pathPlanner
{
  class map
  {
  public:
    class cell;
    class edge
    {
    public:
      std::vector<point_t *> vertices;
      std::vector<cell *> cells; // Double link
      enum edgetypes {EDGESOFT, EDGEHARD} edgetype;
    };
    class cell
    {
    public:
      std::vector<edge *> edges;
      int cellId;
    };

    std::vector<point_t> vertices;
    std::vector<edge> edges;
    std::vector<cell> cells;

    point_t* verticeAdd(point_t); // Returns the pointer to the vertice
    point_t* verticeExists(point_t); // Returns a (point_t *) null if the vertice doesnt exists
    edge* edgePoint(point_t); // test if this point is on any edge. Returns null pointer if not
    edge* edgeAdd(point_t *, point_t *, edge::edgetypes);
    edge* edgeExists(point_t *, point_t *);
    cell* cellAdd();
    void cellAddEdge(cell *, edge *);
    void cellAddEdge(cell *, point_t *, point_t *, edge::edgetypes);
    void drawMap(Img *); // Fails if the map isnt big enogh (No test for size)
    void drawVertices(Img *, int);
    void drawEdges(Img *, int, int);
    void drawEdge(Img *, edge *, int);
    void printCellInfo(cell *);
    void printEdgeInfo(edge *);
  };
}
#endif
