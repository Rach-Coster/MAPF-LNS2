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

struct TLNS_measures {

    //Cost of agents after the runthrough of initLNS 
    int initCost;


    vector<pair<int, int>> makesumPerExecution; 

    //Cost of agents with path that may contain collisions 
    vector<pair<int, int>> heuristicCommitmentCost; 

    //Number of executions per execution, per agent
    //Kinda redundant but might be nice for data gathering
    vector<pair<int, vector<int>>> commitmentCostPerAgent;

    //Accumulative number of executions, per agent from start until goal 
    vector<pair<int, vector<int>>> accumulativeCostPerAgent; 

    //Number of executions left until the agent reaches the goal location
    vector<pair<int, vector<int>>> remainingCostPerAgent;

    //Accumulative commitment cost of all agents per iteration 
    vector<pair<int, int>> commitmentCostPerExecution; 

    vector<AgentPositions> states; 

    vector<pair<int, double>> processingPerExecution; 

    //int - iteration, int - agent, int - path
    //the path may need additonal elements added to it, e.g. the agent's original start position 
    vector<tuple<int, int, Path>> improvedPath;
    vector<pair<int, Path>> initPaths; 

};

class TimeWrapper
{
public: 
    TLNS_options tlnsOptions;
    Instance& instance; 
    //Just in case the time_per_action can be changed after a meta-heuristic runs
    double time_per_action = 1;; 
    int no_of_committed_actions = 1; 

    TimeWrapper(Instance&  instance, const double& timePerAction, const int& noOfCommittedActions,
        const string& metaHeuristic, const string& solutionType, const TLNS_options& options);


    pair<double, TLNS_measures> runCommitmentStrategy();

    void writeImprovementsToFile(const string & file_name, TLNS_measures & tlns_measures);
    void writePathsToFile(const string & file_name, TLNS_measures & tlns_measures);
    void writeResultToFile(const string & file_name, TLNS_measures & tlns_Measures);

private: 
    meta_heuristic m_heuristic = ONEACTIONAHEAD; 
    solution_type s_type = FEASIBLE; 

    bool atGoals(vector<AgentPositions> states);
    bool atGoal(TLNS_measures tlns_measures, int agentId);

    high_resolution_clock::time_point start_time;
   
};