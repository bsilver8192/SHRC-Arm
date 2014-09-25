#ifndef problem_h
#define problem_h

class problem
{
public:
    const int maxSuccessors;
    problem(const int mS);
    class state
    {
    public:
	virtual bool equals(problem::state* comp)=0;
	virtual problem::state* create()=0;
    };
    virtual bool atGoal(const state* current)=0;
    virtual double heuristic(const state* current)=0;
    virtual int getSuccessors(const state* current, state** successors)=0;
};
    
#endif
