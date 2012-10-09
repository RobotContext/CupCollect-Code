#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include "OfflineCellPlanner.hpp"
#include "OfflineReachableMap.hpp"
#include "map.hpp"

using namespace rw::sensor;
using namespace rw::loaders;
using namespace boost::geometry::model::d2;
using namespace pathPlanner;

const std::string mapfile = "ramdisk/complete_map_project.pgm"; // Original map
const std::string reachableMapFile = "ramdisk/complete_map_reachable.pgm"; // Map only with reachable space
const std::string workspaceMapFile = "ramdisk/complete_map_workspace.pgm"; // Workspace map

int main()
{
  Image * workspaceMap;

  /** /
  // Create reachable map file and workspace map
  std::cout << "Loading map" << std::endl;
  Image * map = PPMLoader::load(mapfile);
  Image * reachableMap;
  reachableMap = new Image(map->getWidth(), map->getHeight(), Image::GRAY, Image::Depth8U);
  workspaceMap = new Image(map->getWidth(), map->getHeight(), Image::GRAY, Image::Depth8U);
  OfflineReachableMap::makeReachableMap(map, reachableMap, point_xy<int>(3000,1400));
  OfflineReachableMap::makeWorkspaceMap(reachableMap, workspaceMap);
  reachableMap->saveAsPGM(reachableMapFile);
  workspaceMap->saveAsPGM(workspaceMapFile);
  /**/
  
  /* Load existing workspace map file */
  workspaceMap = PPMLoader::load(workspaceMapFile);


  // Output
  map newMap;
  newMap.vertices.reserve(100000);
  newMap.edges.reserve(10000);
  newMap.cells.reserve(1500);

  OfflineCellPlanner c = OfflineCellPlanner(workspaceMap, 
					    &newMap, 
					    point_t(10,10), 
					    point_t((workspaceMap->getWidth()-10),(workspaceMap->getHeight()-10))
					    );

  // Draw map for debugging purpose
  Image * vectormap = new Image(workspaceMap->getWidth(), workspaceMap->getHeight(), Image::GRAY, Image::Depth8U);
  newMap.drawMap(vectormap);
  vectormap->saveAsPGM("test.pgm");

  // Test a cell
  newMap.printCellInfo(&(newMap.cells.at(0)));
  newMap.printCellInfo(&(newMap.cells.at(2)));
  newMap.printCellInfo(&(newMap.cells.at(3)));
  newMap.printCellInfo(&(newMap.cells.at(6)));
  return 0;
}
