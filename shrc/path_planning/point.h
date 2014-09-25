#ifndef point_h
#define point_h

class point
{
public:
    const static double epsilon;
    double x;
    double y;
    double z;
    point(double px=0, double py=0, double pz=0);
    double distance(const point& p) const;
};
bool operator==(const point &p1, const point &p2);

#endif
