#ifndef plan_h
#define plan_h


//need: motorValsToStates //spits out control states: position velocity vectors
//need: statesToXYZ //converts the control states to xyz of end of arm. Stores relationship between full state and xyz
//need: constraints

#include "point.h"
#include "problem.h"
#include <vector>

class AStarSearch
{
private:
    problem *p;
public:
    //std::vector<state> search(const state& start);
    std::vector<problem::state*> search(problem::state* start);
    //AStarSearch(point g, double maxVel, const unsigned int ss);
    AStarSearch(problem *p);
private:
    /*point goal; //where we want to get to
    double maxVelocity; //IMPORTANT: units are distanceUnits/timestep. NOT OVER SECONDS. Over timestep.
    bool atGoal(const state& current);
    double heuristicEuclidean(const state& current); //euclidean distance heuristic for how far it is to the goal, converted to time 
protected:
    const unsigned int sizeSuccessors;
    virtual state* getSuccessors(const state& current) = 0; //will call motorValsToStates and statesToXYZ etc. Should take into account goal
    */
};

#endif
