#ifndef testProblem_h
#define testProblem_h

#include "problem.h"
#include "point.h"

class testProblem : public problem
{
private:
    double maxVelocity;
    point goal;
public:
    testProblem(point g, double maxVel);
    class state : public problem::state
    {
    public:
	point position;
	//whatever the vector of the control loop state looks like. This exists for returning data, and for the successor function.
	//maybe throw in what the motorVals we started with look like, to make it easier to combine these. For returning only.
	state(point p);
	virtual bool equals(problem::state* comp);
	virtual problem::state* create();
    };
    virtual bool atGoal(const problem::state* current);
    virtual double heuristic(const problem::state* current);
    virtual int getSuccessors(const problem::state* current, problem::state** successors);
};

#endif
