#include "point.h"
#include <cstdlib>
#include <cmath>

const double point::epsilon = 0.001;

bool operator==(const point &p1, const point &p2)
{
    return abs(p1.x-p2.x)<point::epsilon && abs(p1.y-p2.y)<point::epsilon && abs(p1.z-p2.z)<point::epsilon;
}

point::point(double px, double py, double pz)
{
    x=px;
    y=py;
    z=pz;
}

double point::distance(const point& p) const
{
    return sqrt(pow(p.x-x,2)+pow(p.y-y,2)+pow(p.z-z,2));
}
