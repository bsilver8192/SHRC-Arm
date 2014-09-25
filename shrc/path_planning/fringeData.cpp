#include "fringeData.h"
#include <vector>

fringeData::fringeData(problem::state* p, unsigned long long cost, double heuristic, std::vector<problem::state*> steps)
{
    position=p;
    costToDate=cost;
    heuristicVal=heuristic;
    stepsSoFar=steps;
}

bool operator<(const fringeData& f1, const fringeData& f2)
{
    return f1.costToDate + f1.heuristicVal < f2.costToDate + f2.heuristicVal;
}

void fringeData::deleteSelf()
{
    delete position;
    for(unsigned int i=0; i<stepsSoFar.size(); i++)
    {
	delete stepsSoFar[i];
    }
    stepsSoFar.clear();
}

std::vector<problem::state*> fringeData::copySteps()
{
    std::vector<problem::state*> ret;
    for(int i=0; i<stepsSoFar.size(); i++)
    {
	ret.push_back(stepsSoFar[i]->create());
    }
    return ret;
}
