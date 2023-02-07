#include "TimeWrapper.h"
#include <string>
#include <iostream>
#include <utility> 

//temp
#include <string.h>
#include <vector>
#include <algorithm>

using namespace std; 
using namespace boost::multiprecision;

TimeWrapper::TimeWrapper(Instance& instance, const double& timePerAction, const int& noOfCommittedActions,
    const string& metaHeuristic, const string& solutionType, const TLNS_options& options): 
    instance(instance), time_per_action(timePerAction), no_of_committed_actions(noOfCommittedActions), tlnsOptions(options){
        
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

pair<clock_t, vector<AgentPositions>>TimeWrapper::runCommitmentStrategy(){
    cout << "Hello from the commitment strategy" << endl; 

    pair<clock_t, vector<AgentPositions>> completedPlan;
    //vector<pair<int, vector<int>> sumOfCosts; 
    //vector<pair<int, vector<int>> remainingCost; 
    
    //The clock is the makespan; 
    clock_t t_start = clock(); 
 
    //Note: initLNS does not always provide a feasible solution

    LNS* lns = new LNS(instance, tlnsOptions, t_start);     
    lns->run();
    assert(lns->validateSolution());  

    clock_t wallClockTime = clock() - t_start; 

    //a vertex is passed and x,y can be accessed via instance.getRowCoordinate(), instance.getColCoordinate()   
    vector<std::pair<int, vector<int>>> solutionPositions; 
    
    //Combine these later so that state pos is equal to the agent.path[no_of_committedActions]
    vector<AgentPositions> states;
    vector<int> startLocations = instance.getStarts(); 
    vector<int> goalLocations = instance.getGoals(); 

    //Gets start state for each agent
    for(int i = 0; i < instance.getDefaultNumberOfAgents(); i++){   
        AgentPositions agentPos;
        agentPos.id = i; 
        agentPos.currentX = instance.getRowCoordinate(startLocations[i]); 
        agentPos.currentY = instance.getColCoordinate(startLocations[i]);         
        
        states.push_back(agentPos);

        //delete agentPosition after creation 
    }

    //no_of_committed_actions post start position
    //check if counter is needed or if the path is from no_of_committed
    //actions to goal

    while(!atGoals(states)){
        //should planningTime also be a clock?
        clock_t planningTime = no_of_committed_actions * time_per_action; 

        for(int i = 0; i < lns->agents.size(); i++){
            //Agent agent = lns.agents[i];
         
            std::vector<int> movingAgent; 

            //Correct by the txt file is in y,x rather than x,y which is dumb

            for(int j = 0; j <= no_of_committed_actions; j++){    
                if(j == no_of_committed_actions){
                    states[i].currentX = instance.getRowCoordinate(lns->agents[i].path[j].location);
                    states[i].currentY = instance.getColCoordinate(lns->agents[i].path[j].location);
                }

                else if(states[i].currentX == instance.getRowCoordinate(goalLocations[i]) && 
                    states[i].currentY == instance.getColCoordinate(goalLocations[i])){
                    break;
                }

                else {
                    states[i].pastPositions.push_back(std::make_pair(
                        instance.getRowCoordinate(lns->agents[i].path[j].location),
                        instance.getColCoordinate(lns->agents[i].path[j].location))); 
                }  

                movingAgent.push_back(instance.linearizeCoordinate(states[i].currentX, states[i].currentY)); 
            
            }

            instance.setStartLocation(states[i]);

            

            //Agent.id is temp, may change to i
            solutionPositions.push_back(std::make_pair(lns->agents[i].id, movingAgent)); 
            //movingAgent.clear();   
        }
        //pass solution positions to initLNS too or else it will fail on collision
        delete lns; 

        

        //add iteration limit 100k
        tlnsOptions.maxIterations = 100000;
    
        lns = new LNS(instance, tlnsOptions, planningTime);  
        lns->loadTlnsPath(solutionPositions);
      
        lns->run();
        assert(lns->validateSolution());  

        wallClockTime += planningTime; 
    }

    //States has all the agent positions (past - pastPositions and current - currentX/currentY)
    completedPlan = std::make_pair(wallClockTime, states); 
    return completedPlan; 
 
    //Complete
    // t_start <- clock()
    // S <- Initial feasible solution
    // wall clock time<- clock() - t_start
    // C <- Commit Strategy
    // STATES (agent positions pair, x y) <- {a1,a2, … …, an} 
    // Executed_actions ={}
    // While  not goals :
    // num_actions = C()
    // planning_time = num_actions * time per action
    //STATES , S , Executed_actions = execute(S, num_actions, Executed_actions)
    
    
    // S <-LNS.optimise(S, planning_time)

    //Also done 
    // wall clock time+=planning_time
    // Return Executed_actions, wall clock time
};

bool TimeWrapper::atGoals(vector<AgentPositions> states){

    std::vector<int> goalLocations = instance.getGoals();

    for(int i = 0; i < states.size(); i++){
        if(states[i].currentX != instance.getRowCoordinate(goalLocations[i]) && 
            states[i].currentY != instance.getColCoordinate(goalLocations[i])){
            return false; 
        }
    }
    
    return true;
};
