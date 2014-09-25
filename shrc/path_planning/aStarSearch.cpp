#include "aStarSearch.h"
#include "fringeData.h"
#include <cstdlib>
#include <iostream>
#include <cassert>
#include <utility>
#include <set>

/*AStarSearch::AStarSearch(point g, double maxVel, unsigned int ss) : sizeSuccessors(ss)
{
    goal.x=g.x;
    goal.y=g.y;
    goal.z=g.z;
    maxVelocity=maxVel;
}

bool AStarSearch::atGoal(const state& current)
{
    return current.position == goal;
}

double AStarSearch::heuristicEuclidean(const state& current)
{
    return current.position.distance(goal)/maxVelocity;
    //return abs(current.position.x-goal.x)+abs(current.position.y-goal.y)+abs(current.position.z-goal.z);  //Manhattan distance, do not use.
}*/

AStarSearch::AStarSearch(problem *p_p)
{
    p=p_p;
}

std::vector<problem::state*> AStarSearch::search(problem::state* start)
{
    std::multiset<fringeData> fringe;
    std::vector<problem::state*> empty;
    if(p->atGoal(start)) //you're dumb.
    {
	empty.push_back(start);
	return empty;
    }
    fringe.insert(fringeData(start, 0, p->heuristic(start), empty));
    
    int DELETEcounter=0;
    while(true)
    {
	DELETEcounter++;
	assert(!fringe.empty());
	
	fringeData curr=*fringe.begin(); //temporarily store the element that we're exploring.
	fringe.erase(fringe.begin());
	if(p->atGoal(curr.position)) //we're done, hurray!
	{
	    std::cout << "Nodes expanded: " << DELETEcounter << "\n";
            curr.stepsSoFar.push_back(curr.position); //put in the most recently made move. Don't need to copy it because if I did, I'd have to delete it anyway before returning.
	    fringeData toDelete = *(fringe.begin());
	    while(!fringe.empty())
	    {
		toDelete=*(fringe.begin());
		fringe.erase(fringe.begin());
		toDelete.deleteSelf();
	    }
	    return curr.stepsSoFar;
	}

	problem::state* successors[p->maxSuccessors];
	int size=p->getSuccessors(curr.position, successors);
	
	for(unsigned int i=0; i<size; i++) //add all the successors to the fringe. 
	{
	    fringeData toAdd(successors[i], curr.costToDate+1, p->heuristic(successors[i]), curr.copySteps());
	    toAdd.stepsSoFar.push_back(curr.position->create()); //put in the most recently made move. New memory every time
	    std::pair<std::multiset<fringeData>::iterator, std::multiset<fringeData>::iterator> it = fringe.equal_range(toAdd);
	    if(it.first!=it.second)
	    {
		bool good=true;
		for(std::multiset<fringeData>::iterator j=it.first; j!=it.second; j++)
		{
		    if( ((*j).position)->equals(successors[i])) //the position I want to add to the fringe is already there. Can only happen once, fringe contains unique positions because of this code.
		    {
			good=false;
			if(curr.costToDate+1<(*j).costToDate) //the one I found currently is better than the previous best one
			{
			    fringeData toDelete=*j;
			    fringe.erase(j);
			    toDelete.deleteSelf();
			    fringe.insert(toAdd);
			}
			else //the only case wheretoAdd is not used. Everywhere else I need to put it in the fringe, so do not delete.
			{
			    toAdd.deleteSelf();
			}
			break;
		    }
		}
		if(good) //the point I'm trying to add is not already in the set
		    fringe.insert(toAdd);
	    }
	    else
		fringe.insert(toAdd); //1 is because each exploration is 1 timestep, so constant cost per node
	    //std::push_heap(fringe.begin(), fringe.end());
	}
	curr.deleteSelf(); //don't forget to delete the one I'm pulling out of the fringe.
    }
}
