#include "TimeWrapper.h"
#include <string>
#include <iostream>
#include <utility>

// temp
#include <string.h>
#include <vector>
#include <algorithm>

using namespace std;
using namespace boost::multiprecision;

TimeWrapper::TimeWrapper(Instance &instance, const double &timePerAction, const int &noOfCommittedActions,
                         const string &metaHeuristic, const string &solutionType, const TLNS_options &options) : instance(instance), time_per_action(timePerAction), no_of_committed_actions(noOfCommittedActions), tlnsOptions(options)
{

    if (metaHeuristic == "OneActionAhead")
    {
        m_heuristic = ONEACTIONAHEAD;
    }
    else if (metaHeuristic == "AllActions")
    {
        m_heuristic = ALLACTIONS;
    }
    else if (metaHeuristic == "FixedActionSteps")
    {
        // Probably should have a value for this
        // Need to figure out if it is determined by user or automatically assigned
        m_heuristic = FIXEDACTIONSTEPS;
    }
    else if (metaHeuristic == "Dynamic")
    {
        m_heuristic == DYNAMIC;
    }
    else
    {
        cerr << "Meta-heuristic " << metaHeuristic << "does not exist." << endl;
    }

    if (solutionType == "Feasible")
    {
        s_type = FEASIBLE;
    }
    else if (solutionType == "NonFeasible")
    {
        s_type = NONFEASIBLE;
    }
    else
    {
        cerr << "Solution type " << solutionType << "does not exist." << endl;
    }
};

pair<double, TLNS_measures> TimeWrapper::runCommitmentStrategy()
{
    pair<double, TLNS_measures> completedPlan; 

    TLNS_measures tlns_measures;

    // The clock is the makespan;
    start_time = Time::now();

    // Note: initLNS does not always provide a feasible solution
    int iterationNo = 0;

    LNS *lns = new LNS(instance, tlnsOptions);

    lns->run();
    assert(lns->validateSolution());

    double wallClockTime = ((fsec)(Time::now() - start_time)).count();

    // a vertex is passed and x,y can be accessed via instance.getRowCoordinate(), instance.getColCoordinate()
    vector<pair<int, vector<int>>> solutionPositions;

    // Combine these later so that state pos is equal to the agent.path[no_of_committedActions]
    vector<int> startLocations = instance.getStarts();
    vector<int> goalLocations = instance.getGoals();

    // Gets start state for each agent
    for (int i = 0; i < instance.getDefaultNumberOfAgents(); i++)
    {
        AgentPositions agentPos;
        agentPos.id = i;
        agentPos.currentX = instance.getRowCoordinate(startLocations[i]);
        agentPos.currentY = instance.getColCoordinate(startLocations[i]);

        int heuristicCost = lns->agents[i].path_planner->my_heuristic[lns->agents[i].path_planner->start_location];

        tlns_measures.states.push_back(agentPos);
        tlns_measures.initCommitmentCost.push_back(make_pair(lns->agents[i].id, lns->agents[i].path.size()));
        
        tlns_measures.heuristicCommitmentCost.push_back(make_pair(lns->agents[i].id, heuristicCost));
    }

    // no_of_committed_actions post start position
    // check if counter is needed or if the path is from no_of_committed
    // actions to goal
    vector<int> accumulativeCost;

    while (!atGoals(tlns_measures.states))
    {
        auto planningTime = no_of_committed_actions * time_per_action;
        iterationNo++;
    
        int costPerExecution = 0;

        for (int i = 0; i < lns->agents.size(); i++)
        {   
            vector<int> movingAgent;

            int j;
            for (j = 0; j < no_of_committed_actions; j++)
            {
                // The output.txt file is in y,x rather than x,y which is dumb
                if (tlns_measures.states[i].currentX == instance.getRowCoordinate(goalLocations[i]) &&
                    tlns_measures.states[i].currentY == instance.getColCoordinate(goalLocations[i]))
                {
                    break;
                }

                if(iterationNo > 1){
                    tlns_measures.states[i].pastPositions.push_back(make_pair(instance.getRowCoordinate(lns->agents[i].path[j].location),
                                                                          instance.getColCoordinate(lns->agents[i].path[j].location)));
                }

                movingAgent.push_back(instance.linearizeCoordinate(tlns_measures.states[i].currentX, tlns_measures.states[i].currentY));
            }
            
            tlns_measures.states[i].currentX = instance.getRowCoordinate(lns->agents[i].path[j].location);
            tlns_measures.states[i].currentY = instance.getColCoordinate(lns->agents[i].path[j].location);
            
            //convert all pairs into doubles 
            if(iterationNo == 1){
                tlns_measures.commitmentCostPerAgent.push_back(make_pair(i, vector<int>()));
                tlns_measures.accumulativeCostPerAgent.push_back(make_pair(i, vector<int>()));
                tlns_measures.remainingCostPerAgent.push_back(make_pair(i, vector<int>()));
                
                tlns_measures.accumulativeCostPerAgent[i].second.push_back(j * time_per_action);
            }
            
            else {

                tlns_measures.accumulativeCostPerAgent[i].second.push_back(tlns_measures.accumulativeCostPerAgent[i].second.back() + (j * time_per_action));
            }

            tlns_measures.commitmentCostPerAgent[i].second.push_back(j * time_per_action);
            tlns_measures.remainingCostPerAgent[i].second.push_back(lns->agents[i].path.size() - (j + 1) * time_per_action);

            costPerExecution += tlns_measures.commitmentCostPerAgent[i].second.back();

            instance.setStartLocation(tlns_measures.states[i]);
            solutionPositions.push_back(make_pair(lns->agents[i].id, movingAgent));
        }

        tlns_measures.commitmentCostPerExecution.push_back(make_pair(iterationNo, costPerExecution));

        // pass solution positions to initLNS too or else it will fail on collision
        delete lns;

        // add iteration limit 100k
        tlnsOptions.maxIterations = 100000;

        lns = new LNS(instance, tlnsOptions);

        high_resolution_clock::time_point agent_processing_time = Time::now(); 

        lns->loadTlnsPath(solutionPositions);

        lns->run();
        
        double processingTime = ((fsec)(Time::now() - agent_processing_time)).count();

        tlns_measures.processingPerExecution.push_back(make_pair(iterationNo, processingTime));

        assert(lns->validateSolution());


        wallClockTime += planningTime;
    }

    // States has all the agent positions (past - pastPositions and current - currentX/currentY)

    completedPlan = make_pair(wallClockTime, tlns_measures); 
    return completedPlan;
};

bool TimeWrapper::atGoals(vector<AgentPositions> states)
{

    vector<int> goalLocations = instance.getGoals();

    for (int i = 0; i < states.size(); i++)
    {
        if (states[i].currentX != instance.getRowCoordinate(goalLocations[i]) &&
            states[i].currentY != instance.getColCoordinate(goalLocations[i]))
        {
            return false;
        }
    }

    return true;
};

void TimeWrapper::writePathsToFile(const string &file_name, vector<AgentPositions> agentPositions)
{
    ofstream output;
    output.open(file_name);

    for (const auto &agent : agentPositions)
    {
        output << "---TLNS Agent Paths ---" << endl;

        output << "Agent " << agent.id << ":";
        for (int i = 0; i < agent.pastPositions.size(); i++)
        {
            output << "(" << agent.pastPositions[i].first << "," << agent.pastPositions[i].second << ")->";
        }

        output << "(" << agent.currentX << "," << agent.currentY << ")->";
        output << endl;
    }

    output.close();
}

void TimeWrapper::writeResultToFile(const string & file_name, TLNS_measures & tlns_measures)
{
    string name = file_name;

    ofstream addHeads(name, ios::out);
    addHeads << "Iteration No, Processing Per Execution, Agent ID,Init Cost,Commitment Cost,Accumulative Cost,"
    << "Remaining Cost,Heuristic Commitment Cost" << endl; 
    addHeads.close();

    ofstream stats(name, ios::app);
    for(int i = 0; i < tlns_measures.commitmentCostPerExecution.size(); i++){
        for(int j = 0; j < tlns_measures.initCommitmentCost.size(); j++){
            stats << i << ","; 
            stats << tlns_measures.processingPerExecution[i].second  << ",";
            stats << tlns_measures.initCommitmentCost[j].first << "," << tlns_measures.initCommitmentCost[j].second << ","; 
            stats << tlns_measures.commitmentCostPerAgent[j].second[i] << ",";
            stats << tlns_measures.accumulativeCostPerAgent[j].second[i] << ",";
            stats << tlns_measures.remainingCostPerAgent[j].second[i];

            if(i == 0){
                stats << "," << tlns_measures.heuristicCommitmentCost[j].second;
            }

            else {
                stats << ",0";
            }

            stats << endl; 
        }
    }
    stats.close(); 
    
    
    // stats << "Runtime: " << tlns_measures.runtime << "\n"; 

    // //cleanup into for each loops
    // for(int i = 0; i < tlns_measures.initCommitmentCost.size(); i++){
    //     stats << "Init Commitment - Agent Id: " << tlns_measures.initCommitmentCost[i].first << ", Init Commitment Cost: " << tlns_measures.initCommitmentCost[i].second << "\n";
    // }

    // for(int i = 0; i < tlns_measures.commitmentCostPerAgent.size(); i++){
    //     stats <<"Commitment Cost Per Agent " << i << ": "; 
    //     for(int j = 0; j < tlns_measures.commitmentCostPerAgent[i].second.size(); j++){
    //        stats << tlns_measures.commitmentCostPerAgent[i].second[j] <<  ",";
    //     }

    //     stats << "\n"; 
    // }

    // for(int i = 0; i < tlns_measures.accumulativeCostPerAgent.size(); i++){
    //     stats << "Accumulative Cost Per Agent " << i << ": ";
    //     for(int j = 0; j < tlns_measures.accumulativeCostPerAgent[i].second.size(); j++){
    //         stats << tlns_measures.accumulativeCostPerAgent[i].second[j] << ",";
    //     }

    //     stats << "\n";
    // }

    // for(int i = 0; i < tlns_measures.remainingCostPerAgent.size(); i++){
    //     stats << "Remaining Cost Per Agent " << i << ": "; 
    //     for(int j = 0; j < tlns_measures.remainingCostPerAgent[i].second.size(); j++){
    //         stats << tlns_measures.remainingCostPerAgent[i].second[j] << ","; 
    //     }

    //     stats << "\n";
    // }

    // for(int i = 0; i < tlns_measures.commitmentCostPerExecution.size(); i++){
    //     stats << "Commitment Cost Per Execution " << i << ": " << tlns_measures.commitmentCostPerExecution[i].first << ",";
    //     stats << tlns_measures.commitmentCostPerExecution[i].second << "\n";
        
    // }


}
    



