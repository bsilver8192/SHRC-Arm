#include "testProblem.h"
#include "aStarSearch.h"
#include "gtest/gtest.h"

testProblem::testProblem(point g, double maxVel) : problem(6)
{
    goal=g;
    maxVelocity=maxVel;
}

testProblem::state::state(point p)
{
    position=p;
}

bool testProblem::state::equals(problem::state* comp)
{
    return position==((testProblem::state*)comp)->position;
}

problem::state* testProblem::state::create()
{
    testProblem::state* ret = new testProblem::state(position);
    return (problem::state*)ret;
}

bool testProblem::atGoal(const problem::state* current)
{
    return ((testProblem::state*)current)->position==goal;
}

double testProblem::heuristic(const problem::state* current)
{
    return ((testProblem::state*)current)->position.distance(goal)/maxVelocity;
    //return abs(current.position.x-goal.x)+abs(current.position.y-goal.y)+abs(current.position.z-goal.z);  //Manhattan distance, do not use.
}

int testProblem::getSuccessors(const problem::state* current, problem::state** successors)
{
    int ctr=0;
    const testProblem::state* curr = (testProblem::state*)current;
    for(int i=-1; i<=1; i+=2) //returns successors along each axis, one in positive dir, one in negative
    {
        successors[ctr] = new testProblem::state(point(curr->position.x+i, curr->position.y, curr->position.z));
	successors[ctr+1] = new testProblem::state(point(curr->position.x, curr->position.y+i, curr->position.z));
	successors[ctr+2] = new testProblem::state(point(curr->position.x, curr->position.y, curr->position.z+i));
        ctr+=3;
    }
    return maxSuccessors;
}

/*class googleTestAStar : public ::testing::Test
{
public:
    testProblem tp;
    AStarSearch searcher;
    googleTestAStar() : tp(point(0,0,0),1), searcher(&tp)
    {
    }
    bool comparePaths(const std::vector<problem::state*>& p1, const std::vector<problem::state*>& p2)
    {
	//std::cout << p1.size() << " " << p2.size() << std::endl;
        if(p1.size()!=p2.size())
	    return false;
	for(unsigned int i=0; i<p1.size(); i++)
	{
	    //std::cout << p2[i].position.x << ", " << p2[i].position.y << ", " << p2[i].position.z << std::endl;
	    if(!((testProblem::state*)(p1[i]))->equals((testProblem::state*)(p2[i])) )
		return false;
	}
	return true;
    }
};

TEST_F(googleTestAStar, straightLinePaths)
{
    std::vector<problem::state*> expected;
    for(int j=1; j<=10; j++)
    {
	for(int i=0; i<=100*j; i++)
	{
	    expected.push_back(new testProblem::state(point(i-100*j, 0, 0)));
	}
	testProblem::state* start = new testProblem::state(point(j*-100, 0, 0));
	EXPECT_TRUE(comparePaths(searcher.search(start), expected));
	while(expected.size()>0)
	{
	    delete expected[expected.size()-1];
	    expected.pop_back();
	}
	delete start;
    }
}*/

int main()
{
    testProblem tp(point(0,0,0), 1);
    AStarSearch searcher(&tp);

    testProblem::state* start = new testProblem::state(point(-100, 0, 0));

    std::vector<problem::state*> ret = searcher.search(start);

    //delete start;
    for(int i=0; i<ret.size(); i++)
	delete ret[i];
    
    return 0;
}
