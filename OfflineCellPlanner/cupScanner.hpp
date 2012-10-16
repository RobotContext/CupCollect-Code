#ifndef _CUP_SCANNER_
#define _CUP_SCANNER_

#include <iostream>
#include <vector>
#include <math.h>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include "Image.hpp"
#include "PPMLoader.hpp"

typedef boost::geometry::model::d2::point_xy<int> point_t;
using namespace rw::loaders;
using namespace rw::sensor;

struct Vector2D
{
	double x;
	double y;

	Vector2D () : x(0), y(0) {}
	Vector2D (const point_t& p) : x((double)p.x()), y((double)p.y()) {}
	Vector2D (double first, double second) : x(first), y(second) {}
	~Vector2D() {}

	Vector2D operator+ (const Vector2D& rhs)
	{
		return Vector2D(this->x + rhs.x, this->y + rhs.y);
	}

	Vector2D operator- (const Vector2D& rhs)
	{
		return Vector2D(this->x - rhs.x, this->y - rhs.y);
	}

	Vector2D operator* (const double& rhs)
	{
		return Vector2D(this->x * rhs, this->y * rhs);
	}


	double length (void)
	{
		return sqrt(pow(this->x, 2) + pow(this->y, 2));
	}

	double rotation (void)
	{
		return atan2(this->y, this->x);
	}
	void rotation (double angle)
	{
		double x(this->x), y(this->y);

		this->x = cos(angle) * x - sin(angle) * y;
		this->y = sin(angle) * x + cos(angle) * y;
	}
};

std::vector<point_t> cupsBetweenPoints (Image* map, point_t from, point_t to, unsigned int radius, bool showCoverage);
std::vector<point_t> cupsBetweenPoints (Image* map, point_t from, point_t to, unsigned int radius);
std::vector<point_t> findCups (Image* map);

#endif /*_CUP_SCANNER_*/
