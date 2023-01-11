#include "TimeWrapper.h"
#include <string>
#include <iostream>

using namespace std; 

TimeWrapper::TimeWrapper(const Instance& instance, const double& timePerAction, const int& noOfCommitedActions,
    const string& metaHeuristic, const string& solutionType, const TLNS_options& options): 
    instance(instance), tlnsOptions(options){
        
    if(metaHeuristic == "OneActionAhead"){
        m_heuristic = ONEACTIONAHEAD; 
    }
    else if(metaHeuristic == "AllActions"){
        m_heuristic = ALLACTIONS;
    }
    else if(metaHeuristic == "FixedActionSteps"){
        //Probably should have a value for this 
        //Need to figure out if it is determined by user or automatically assigned
        m_heuristic = FIXEDACTIONSTEPS;
    }   
    else if(metaHeuristic == "Dynamic"){
        m_heuristic == DYNAMIC;
    }
    else {
        cerr << "Meta-heuristic " << metaHeuristic << "does not exist." << endl;
    }

    if(solutionType == "Feasible"){
        s_type = FEASIBLE; 
    }
    else if(solutionType == "NonFeasible"){
        s_type = NONFEASIBLE;
    }
    else {
        cerr << "Solution type " << solutionType << "does not exist." << endl;
    }
};



void TimeWrapper::runCommitmentStrategy(){
    cout << "Hello from the commitment strategy" << endl; 

    // Instance instance = lns->getInstance(); 
    // May change into a map with an int and then a pair for the x,y pos 
    // so that is it easier to retrieve and alter values

    clock_t t_start = clock(); 


    //make an LNS constructor that contains the strut and the time 
    //initLNS does not always provide a feasible solution

    LNS lns(instance, tlnsOptions.time_limit, tlnsOptions.neighbourSize, 
        tlnsOptions.screen, tlnsOptions, t_start); 
    lns.run();  

    // PathTable pathTable = lns.getPathTable();


    // clock_t wallClockTime = clock() - t_start; 

    // vector<Agents*> states; 

    // for(int i = 0; i < instance.getDefaultNumberOfAgents(); i++){
     
    //     Agents* agent;

    //     agent->id = i;
    //     agent->currentX = instance.getCoordinate(i).first;
    //     agent->currentY = instance.getCoordinate(i).second;
    //     states.push_back(agent);
    // }

    // for(int i = 0; i < pathTable.table.size(); i++){
    //     for(int j = 0; j < pathTable.table.size(); j++){
    //         cout << "Pathtable: " + pathTable.table.at(i).at(j); 
    //     }
    // }
 
 
    //Complete
    // t_start <- clock()
    // S <- Initial feasible solution
    // wall clock time<- clock() - t_start
    // C <- Commit Strategy
    // STATES (agent positions pair, x y) <- {a1,a2, … …, an} 
    // Executed_actions ={}

    //Needs clarification
    
    // While  not goals :
    // num_actions = C()
    // planning_time = num_actions * time per action
    // STATES , S , Executed_actions = execute(S, num_actions, Executed_actions)
    // S <-LNS.optimise(S, planning_time)
    // wall clock time+=planning_time
    // Return Executed_actions, wall clock time
};

// bool TimeWrapper::atGoals(vector<Agents*> states){
//     return false;
//     // for(int i = 0; i < states.size(); i++){
//     //     if(states.at(i)->currentX != goalPositions.at(i))
//     // }
// };