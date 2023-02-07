#pragma once 
#include "LNS.h"

#include <string>

//temp
#include <string.h>
#include <vector>
#include <algorithm>


//Meta-heuristics from decision making 
enum meta_heuristic {ONEACTIONAHEAD, ALLACTIONS, FIXEDACTIONSTEPS, DYNAMIC};

//Result from init LNS
enum solution_type {FEASIBLE, NONFEASIBLE};


class TimeWrapper
{
public: 
    TLNS_options tlnsOptions;
    Instance& instance; 
    //Just in case the time_per_action can be changed after a meta-heuristic runs
    double time_per_action = 1;
    double rate_of_improvement = 0; 
    int no_of_committed_actions = 1; 

    TimeWrapper(Instance&  instance, const double& timePerAction, const int& noOfCommittedActions,
        const string& metaHeuristic, const string& solutionType, const TLNS_options& options);


    pair<clock_t, vector<AgentPositions>> runCommitmentStrategy();

private: 
    meta_heuristic m_heuristic = ONEACTIONAHEAD; 
    solution_type s_type = FEASIBLE; 

    bool atGoals(vector<AgentPositions> states); 
   
};