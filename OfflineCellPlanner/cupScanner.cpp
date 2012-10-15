//#include <iostream>
#include "cupScanner.hpp"

using namespace rw::loaders;
using namespace rw::sensor;

std::vector<point_t> cupsBetweenPoints (Image* map, point_t from, point_t to, unsigned int radius, bool showCoverage)
{
	std::vector<point_t> result = std::vector<point_t>();

	//	Define boundary box
	point_t pointA, pointB, pointC, pointD, pointE, pointF, pointAx, pointBx, pointCx, pointDx;
	Vector2D diffVector(to.x() - from.x(), to.y() - from.y());					//	Vector between points given
	diffVector = diffVector * ((double)radius / diffVector.length());  			// 	Normalize vector an set length to radius
	pointF = point_t(to.x() + diffVector.x, to.y() + diffVector.y);
	diffVector = diffVector * -1;												//	Rotate vector in oppesite direction
	pointE = point_t(from.x() + diffVector.x, from.y() + diffVector.y);
	diffVector.rotation(-M_PI_2);												//	Rotate negative pi/2
	pointA 	= point_t(from.x() + diffVector.x, from.y() + diffVector.y);
	pointAx = point_t(pointE.x() + diffVector.x, pointE.y() + diffVector.y);
	pointB 	= point_t(to.x() + diffVector.x, to.y() + diffVector.y);
	pointBx = point_t(pointF.x() + diffVector.x, pointF.y() + diffVector.y);
	diffVector = diffVector * -1;												//	Rotate vector in oppesite direction
	pointC = point_t(from.x() + diffVector.x, from.y() + diffVector.y);
	pointCx = point_t(pointE.x() + diffVector.x, pointE.y() + diffVector.y);
	pointD = point_t(to.x() + diffVector.x, to.y() + diffVector.y);
	pointDx = point_t(pointF.x() + diffVector.x, pointF.y() + diffVector.y);

	//	minX, maxX, minY and maxY of bounding box
	int minx(pointA.x());
	int maxx(pointA.x());
	int miny(pointA.y());
	int maxy(pointA.y());

	(pointB.x() < minx) ? minx = pointB.x() : minx;
	(pointC.x() < minx) ? minx = pointC.x() : minx;
	(pointD.x() < minx) ? minx = pointD.x() : minx;
	(pointE.x() < minx) ? minx = pointE.x() : minx;
	(pointF.x() < minx) ? minx = pointF.x() : minx;

	(pointB.x() > maxx) ? maxx = pointB.x() : maxx;
	(pointC.x() > maxx) ? maxx = pointC.x() : maxx;
	(pointD.x() > maxx) ? maxx = pointD.x() : maxx;
	(pointE.x() > maxx) ? maxx = pointE.x() : maxx;
	(pointF.x() > maxx) ? maxx = pointF.x() : maxx;

	(pointB.y() < miny) ? miny = pointB.y() : miny;
	(pointC.y() < miny) ? miny = pointC.y() : miny;
	(pointD.y() < miny) ? miny = pointD.y() : miny;
	(pointE.y() < miny) ? miny = pointE.y() : miny;
	(pointF.y() < miny) ? miny = pointF.y() : miny;

	(pointB.y() > maxy) ? maxy = pointB.y() : maxy;
	(pointC.y() > maxy) ? maxy = pointC.y() : maxy;
	(pointD.y() > maxy) ? maxy = pointD.y() : maxy;
	(pointE.y() > maxy) ? maxy = pointE.y() : maxy;
	(pointF.y() > maxy) ? maxy = pointF.y() : maxy;

	//	Make guidelines
	std::vector<point_t> intersections;
	boost::geometry::model::linestring<point_t> lineA, lineB, travelLine, scanLine;
	lineA.push_back(pointAx);
	lineA.push_back(pointBx);
	lineB.push_back(pointCx);
	lineB.push_back(pointDx);
	travelLine.push_back(pointE);
	travelLine.push_back(pointF);

	//	Scan for cups
	for (int y = miny; y <= maxy; y++)
	{
		intersections.clear();

		//	Create new scan line
		scanLine.clear();
		scanLine.push_back(point_t(minx, y));
		scanLine.push_back(point_t(maxx, y));

		boost::geometry::intersection(travelLine, scanLine, intersections);
		boost::geometry::intersection(lineA, scanLine, intersections);
		boost::geometry::intersection(lineB, scanLine, intersections);

		//	Set up x range for scanning
		int x = minx, endX = maxx, inter = intersections.size();
		point_t (inter,inter);
		if (intersections.size() == 3)
		{
			point_t tempA(intersections.at(1)), tempB(intersections.at(2));
			if (tempA.x() < tempB.x())
			{
				x = tempA.x();
				endX = tempB.x();
			}
			else
			{
				x = tempB.x();
				endX = tempA.x();
			}
		}
		else if (intersections.size() == 2)
		{
			if (intersections.at(0).x() < intersections.at(1).x())
				endX = intersections.at(1).x();
			else
				x = intersections.at(1).x();
		}

		for (; x <= endX; x++)
		{
			if (!showCoverage && (map->getPixelValuei( x, y, 0 ) == 150))
				if ((y >= 0) && (y < map->getWidth()) && (x >= 0) && (x < map->getHeight()))
					result.push_back(point_t(x, y));				//	Cup detected, add to result

			if (showCoverage)
				if ((y >= 0) && (y < map->getWidth()) && (x >= 0) && (x < map->getHeight()))
					result.push_back(point_t(x, y));				//	Coverage point, add to result
		}
	}

	return result;
}

std::vector<point_t> cupsBetweenPoints (Image* map, point_t from, point_t to, unsigned int radius)
{
	return cupsBetweenPoints(map, from, to, radius, false);
}

std::vector<point_t> findCups (Image* map)
{
	const static int channel = 0;

	std::vector<point_t> cups = std::vector<point_t>();

	for (unsigned int x = 0; x < map->getWidth(); x++)
	{
		for (unsigned int y = 0; y < map->getHeight(); y++)
		{
			int val = map->getPixelValuei( x, y, channel );

			if (val == 150) cups.push_back(point_t(x,y));
		}
	}

	return cups;
}

//	Test of unit
//int main ( void )
//{
//	std::cout << " Loading map..." << std::endl;
//	Image* img = PPMLoader::load("img/complete_map_project1.pgm");
//	std::cout << " Width: " << img->getWidth() << "    Height: " << img->getHeight() << std::endl << std::endl;
//
//	std::cout << " Line sweeping map..." << std::endl;
//	std::vector<point_t> cupsPosition = findCups(img);
//	std::cout << std::endl << " Number of cups: " << cupsPosition.size() << std::endl << std::endl;
//
//	std::cout << " Testing cupsBetweenPoints function..." << std::endl;
//
//	//	Testcases
//	//std::vector<point_t> pointsToPlot = cupsBetweenPoints(img, point_t(625,1078), point_t(775,1078), 4, true);
//	//std::vector<point_t> pointsToPlot = cupsBetweenPoints(img, point_t(675,175), point_t(700,225), 4, true);
//	//std::vector<point_t> pointsToPlot = cupsBetweenPoints(img, point_t(675,175), point_t(700,125), 4, true);
//	//std::vector<point_t> pointsToPlot = cupsBetweenPoints(img, point_t(3615,775), point_t(3650,740), 4, true);
//	//std::vector<point_t> pointsToPlot = cupsBetweenPoints(img, point_t(2755,280), point_t(2735,250), 4, true);
//	//std::vector<point_t> pointsToPlot = cupsBetweenPoints(img, point_t(1350,1300), point_t(1310,1255), 4, true);
//	//std::vector<point_t> pointsToPlot = cupsBetweenPoints(img, point_t(1310,1380), point_t(1280,1415), 4, true);
//	//std::vector<point_t> pointsToPlot = cupsBetweenPoints(img, point_t(1475,1270), point_t(1075,1270), 4, true);
//	//std::vector<point_t> pointsToPlot = cupsBetweenPoints(img, point_t(1075,1270), point_t(1475,1270), 4, true);
//	//std::vector<point_t> pointsToPlot = cupsBetweenPoints(img, point_t(1275,1150), point_t(1275,1475), 4, true);
//	//std::vector<point_t> pointsToPlot = cupsBetweenPoints(img, point_t(1275,1475), point_t(1275,1150), 4, true);
//	//std::vector<point_t> pointsToPlot = cupsBetweenPoints(img, point_t(1275,1475), point_t(3950,200), 4, true);
//	//std::vector<point_t> pointsToPlot = cupsBetweenPoints(img, point_t(1275,1475), point_t(3950,200), 4, false);
//
//	/*
//	This is the function to use
//	*/
//	std::vector<point_t> pointsToPlot = cupsBetweenPoints(img, point_t(1310,1380), point_t(1280,1415), 4);
//
//
//	//	Handle output from cupsBetweenPoints
//	std::cout << std::endl << " Number of points to plot: " << pointsToPlot.size() << std::endl << std::endl;
//	for (unsigned int i = 0; i < pointsToPlot.size(); i++)
//		img->setPixel8U(pointsToPlot.at(i).x(), pointsToPlot.at(i).y(), 0);
//
//	std::cout << " Saving map..." << std::endl;
//	img->saveAsPGM("output.pgm");
//
//	// cleanup
//	delete img;
//
//	std::cout << std::endl << " Exiting... " << std::endl;
//
//	return 0;
//}
//
