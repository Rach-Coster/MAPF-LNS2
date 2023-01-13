#include "TimeWrapper.h"
#include <string>
#include <iostream>
#include <utility> 

//temp
#include <string.h>
using namespace std; 

TimeWrapper::TimeWrapper(const Instance& instance, const double& timePerAction, const int& noOfCommittedActions,
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

    pair<clock_t, vector<AgentPositions>>completedPlan;

    clock_t t_start = clock(); 
 
    //initLNS does not always provide a feasible solution

    LNS lns(instance, tlnsOptions, t_start);     
    lns.run();  

    clock_t wallClockTime = clock() - t_start; 

    //a node is passed and x,y can be accessed via instance.getRowCoordinate(), instance.getColCoordinate()   
    vector<std::pair<Agent, vector<int>>> solutionPositions; 
    
    //Combine these later so that state pos is equal to the agent.path[no_of_committedActions]
    vector<AgentPositions> states;
    vector<int> startLocations = instance.getStarts(); 

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

        for(int i = 0; i < instance.getDefaultNumberOfAgents(); i++){
            Agent agent = lns.agents[i]; 
            vector<int> movingAgent; 

            //Correct by the txt file is in y,x rather than x,y which is dumb

            for(int j = 0; j <= no_of_committed_actions; j++){
                movingAgent.push_back(agent.path[j].location); 

                if(j == no_of_committed_actions){
                    states[i].currentX = instance.getRowCoordinate(agent.path[j].location);
                    states[i].currentY = instance.getColCoordinate(agent.path[j].location);
                }

                //Do I add it to the history now or after the new LNS instance?
                else {
                    states[i].pastPositions.push_back(std::make_pair(
                        instance.getRowCoordinate(agent.path[j].location),
                        instance.getColCoordinate(agent.path[j].location))); 
                }              
            }

            solutionPositions.push_back(std::make_pair(agent, movingAgent)); 
            movingAgent.clear(); 
  
        }

        //Need to determine how the plan is inserted

        lns.~LNS();
        LNS lns(instance, tlnsOptions, planningTime);     
        lns.run();

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