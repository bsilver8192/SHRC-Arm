#ifndef fringeData_h
#define fringeData_h

#include "problem.h"
#include <vector>

class fringeData
{
public:
    problem::state* position;
    unsigned long long costToDate;
    std::vector<problem::state*> stepsSoFar;
    double heuristicVal;
    fringeData(problem::state* p, unsigned long long cost, double heuristic, std::vector<problem::state*> steps);
    void deleteSelf();
    std::vector<problem::state*> copySteps();
};
bool operator<(const fringeData& f1, const fringeData& f2);

#endif
