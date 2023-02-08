﻿#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include <boost/multiprecision/cpp_dec_float.hpp>
#include "LNS.h"
#include "AnytimeBCBS.h"
#include "AnytimeEECBS.h"
#include "PIBT/pibt.h"
#include "TimeWrapper.h"   

//temp
#include <string.h>
#include <sstream>
#include <iomanip>

using namespace boost::multiprecision;

/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
		("agents,a", po::value<string>()->required(), "input file for agents")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
        ("output,o", po::value<string>(), "output file name (no extension)")
        ("inputPaths", po::value<string>(), "input file for paths")
        ("outputPaths", po::value<string>(), "output file for paths")
        ("cutoffTime,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(0),
		        "screen option (0: none; 1: LNS results; 2:LNS detailed results; 3: MAPF detailed results)")
		("stats", po::value<string>(), "output stats file")

		// solver
		("solver", po::value<string>()->default_value("LNS"), "solver (LNS, A-BCBS, A-EECBS)")
		("sipp", po::value<bool>()->default_value(true), "Use SIPP as the single-agent solver")
		("seed", po::value<int>()->default_value(0), "Random seed")

        // params for LNS
        ("initLNS", po::value<bool>()->default_value(true),
             "use LNS to find initial solutions if the initial sovler fails")
        ("neighborSize", po::value<int>()->default_value(8), "Size of the neighborhood")
        ("maxIterations", po::value<int>()->default_value(0), "maximum number of iterations")
        ("initAlgo", po::value<string>()->default_value("PP"),
                "MAPF algorithm for finding the initial solution (EECBS, PP, PPS, CBS, PIBT, winPIBT)")
        ("replanAlgo", po::value<string>()->default_value("PP"),
                "MAPF algorithm for replanning (EECBS, CBS, PP)")
        ("destoryStrategy", po::value<string>()->default_value("Adaptive"),
                "Heuristics for finding subgroups (Random, RandomWalk, Intersection, Adaptive)")
        ("pibtWindow", po::value<int>()->default_value(5),
             "window size for winPIBT")
        ("winPibtSoftmode", po::value<bool>()->default_value(true),
             "winPIBT soft mode")
        ("truncatePaths", po::value<bool>()->default_value(true),
             "Truncate initial paths to maximize agents at goals")

         // params for initLNS
         ("initDestoryStrategy", po::value<string>()->default_value("Adaptive"),
          "Heuristics for finding subgroups (Target, Collision, Random, Adaptive)")
		
        // params for TLNS wrapper 
        ("timePerAction", po::value<double>()->default_value(1), "Number of seconds per t")
        ("noOfCommittedActions", po::value<int>()->default_value(1), "Number commited actions (t)")
        ("metaHeuristic", po::value<string>()->default_value("OneActionAhead"), "Meta-heuristics to determine the predicition scope (OneActionAhead, AllActions, FixedActionSteps, Dynamic)")
        ("solutionType", po::value<string>()->default_value("Feasible"), "Result from LNS that is the starting point for the meta heuristic (Feasible, NonFeasible)")
		;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help")) {
		cout << desc << endl;
		return 1;
	}

    PIBTPPS_option pipp_option;
    pipp_option.windowSize = vm["pibtWindow"].as<int>();
    pipp_option.winPIBTSoft = vm["winPibtSoftmode"].as<bool>();

    po::notify(vm);

	srand((int)time(0));

	Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
		vm["agentNum"].as<int>());
    double time_limit = vm["cutoffTime"].as<double>();
    int screen = vm["screen"].as<int>();
	srand(vm["seed"].as<int>());

	if (vm["solver"].as<string>() == "LNS")
    {
        LNS lns(instance, time_limit,
                vm["initAlgo"].as<string>(),
                vm["replanAlgo"].as<string>(),
                vm["destoryStrategy"].as<string>(),
                vm["neighborSize"].as<int>(),
                vm["maxIterations"].as<int>(),
                vm["initLNS"].as<bool>(),
                vm["initDestoryStrategy"].as<string>(),
                vm["sipp"].as<bool>(),
                vm["truncatePaths"].as<bool>(),
                screen, pipp_option);

        if (vm.count("inputPaths") and !lns.loadPaths(vm["inputPaths"].as<string>()))
        {
            cerr << "The input path file " << vm["inputPaths"].as<string>() << " does not exist" << endl;
            exit(-1);
        }
        bool succ = lns.run();
        if (succ)
        {
            lns.validateSolution();
            if (vm.count("outputPaths"))
                lns.writePathsToFile(vm["outputPaths"].as<string>());
        }
        if (vm.count("output"))
            lns.writeResultToFile(vm["output"].as<string>());
        if (vm.count("stats"))
            lns.writeIterStatsToFile(vm["stats"].as<string>());
        // lns.writePathsToFile("path.txt");
    }
    else if (vm["solver"].as<string>() == "A-BCBS") // anytime BCBS(w, 1)
    {
        AnytimeBCBS bcbs(instance, time_limit, screen);
        bcbs.run();
        bcbs.validateSolution();
        if (vm.count("output"))
            bcbs.writeResultToFile(vm["output"].as<string>() + ".csv");
        if (vm.count("stats"))
            bcbs.writeIterStatsToFile(vm["stats"].as<string>());
    }
    else if (vm["solver"].as<string>() == "A-EECBS") // anytime EECBS
    {
        AnytimeEECBS eecbs(instance, time_limit, screen);
        eecbs.run();
        eecbs.validateSolution();
        if (vm.count("output"))
            eecbs.writeResultToFile(vm["output"].as<string>() + ".csv");
        if (vm.count("stats"))
            eecbs.writeIterStatsToFile(vm["stats"].as<string>());
    }    
    else if(vm["solver"].as<string>() == "TLNS") //Time wrapper with LNS 
    {
        TLNS_options tlnsOptions; 
        tlnsOptions.time_limit = time_limit; 
        tlnsOptions.initAlgo = vm["initAlgo"].as<string>();
        tlnsOptions.replanAlgo = vm["replanAlgo"].as<string>();
        tlnsOptions.destroyStrategy = vm["destoryStrategy"].as<string>();
        tlnsOptions.neighbourSize = vm["neighborSize"].as<int>();
        //Skips LNS and goes directly to LNS2
        tlnsOptions.maxIterations = 0;
        tlnsOptions.initLNS = vm["initLNS"].as<bool>();
        tlnsOptions.initDestroyStrategy = vm["initDestoryStrategy"].as<string>();
        tlnsOptions.sipp = vm["sipp"].as<bool>(),
        tlnsOptions.truncatePaths = vm["truncatePaths"].as<bool>(),
        tlnsOptions.screen  = screen; 
        tlnsOptions.pipp_option = pipp_option;

       
        // //pass instance of struct to timewrapper
        // //struct is the same as the one in the class re cpp 

        TimeWrapper timeWrapper
        (
            instance,
            vm["timePerAction"].as<double>(),
            vm["noOfCommittedActions"].as<int>(),
            vm["metaHeuristic"].as<string>(),
            vm["solutionType"].as<string>(),
            tlnsOptions
        );  

        clock_t start = clock(); 

        pair<clock_t, vector<AgentPositions>> result = timeWrapper.runCommitmentStrategy();
        result.first = result.first += start; 

        //std::ostringstream stream; 

        __float128 runtime = ((__float128) (result.first - start)) / CLOCKS_PER_SEC;

        // cout << "Total Clock time is: " << std::fixed << std::setprecision(8) 
        //     << (double) runtime << " seconds" << endl;

        if (vm.count("outputPaths"))
                timeWrapper.writePathsToFile(vm["outputPaths"].as<string>(), result.second); 

    }

	else
    {
	    cerr << "Solver " << vm["solver"].as<string>() << " does not exist!" << endl;
	    exit(-1);
    }
	return 0;

}