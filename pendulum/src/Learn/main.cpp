#include <iostream>
#include <numeric>
#include <thread>
#include <atomic>
#include <chrono>
#include <inttypes.h>
#include <getopt.h>
#define _USE_MATH_DEFINES // To get M_PI
#include <math.h>

#include "pendulum.h"
#include "instructions.h"

int main(int argc, char ** argv) {

    char option;
    uint64_t seed = 0;
    char paramFile[150];
	bool velocity = 0;
    strcpy(paramFile, ROOT_DIR "/params.json");
    while((option = getopt(argc, argv, "s:p:v:")) != -1){
        switch (option) {
            case 's': seed= atoi(optarg); break;
            case 'p': strcpy(paramFile, optarg); break;
			case 'v': velocity = atoi(optarg); break;
            default: std::cout << "Unrecognised option. Valid options are \'-s seed\' \'-p paramFile.json\' \'-v velocity\'." << std::endl; exit(1);
        }
    }
    std::cout << "Selected seed : " << seed << std::endl;
    std::cout << "Selected params: " << paramFile << std::endl;

    // Save the index of the parameter file.
    int indexParam = std::stoi(std::regex_replace(paramFile, std::regex(R"(.*params_(\d+)\.json)"), "$1"));

	std::cout << "Start Pendulum application." << std::endl;

	// Create the instruction set for programs
	Instructions::Set set;
	fillInstructionSet(set);

	// Set the parameters for the learning process.
	// (Controls mutations probability, program lengths, and graph size
	// among other things)
	// Loads them from the file params.json
	Learn::LearningParameters params;
	File::ParametersParser::loadParametersFromJson(paramFile, params);
#ifdef NB_GENERATIONS
	params.nbGenerations = NB_GENERATIONS;
#endif

	// Instantiate the LearningEnvironment
	Pendulum pendulumLE({ 0.05, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0 }, velocity);

	std::cout << "Number of threads: " << params.nbThreads << std::endl;

	// Instantiate and init the learning agent
	Learn::ParallelLearningAgent la(pendulumLE, set, params);
	la.init(seed);

	const TPG::TPGVertex* bestRoot = NULL;


	std::atomic<bool> exitProgram = false; // (set to false by other thread) 
	std::atomic<bool> toggleDisplay = false;

	// Basic logger
	Log::LABasicLogger basicLogger(la);

    // Basic Logger
    char logPath[150];
	sprintf(logPath, "out.%d.p%d.std", seed, indexParam);

    std::ofstream logStream;
    logStream.open(logPath);
    Log::LABasicLogger log(la, logStream);

	// Create an exporter for all graphs
    char dotPath[150];
    sprintf(dotPath, "out_0000.%d.p%d.dot", seed, indexParam);
	File::TPGGraphDotExporter dotExporter(dotPath, *la.getTPGGraph());

	// Logging best policy stat.
    char bestPolicyStatsPath[150];
    sprintf(bestPolicyStatsPath, "bestPolicyStats.%d.p%d.md", seed, indexParam);
	std::ofstream stats;
	stats.open(bestPolicyStatsPath);
	Log::LAPolicyStatsLogger policyStatsLogger(la, stats);

	// Export parameters before starting training.
	// These may differ from imported parameters because of LE or machine specific
	// settings such as thread count of number of actions.
	File::ParametersParser::writeParametersToJson("exported_params.json", params);

	// Train for params.nbGenerations generations
	for (int i = 0; i < params.nbGenerations && !exitProgram; i++) {
#define PRINT_ALL_DOT 0
#if PRINT_ALL_DOT
		char buff[13];
		sprintf(buff, "out_%04d.dot", i);
		dotExporter.setNewFilePath(buff);
		dotExporter.print();
#endif


		la.trainOneGeneration(i);
	}

	// Keep best policy
	la.keepBestPolicy();

	// Clear introns instructions
	la.getTPGGraph()->clearProgramIntrons();

    char bestDot[150];
	// Export the graph
    sprintf(bestDot, "out_best.%d.p%d.dot", seed, indexParam);
	dotExporter.setNewFilePath(bestDot);
	dotExporter.print();

	TPG::PolicyStats ps;
	ps.setEnvironment(la.getTPGGraph()->getEnvironment());
	ps.analyzePolicy(la.getBestRoot().first);
	std::ofstream bestStats;
    sprintf(bestPolicyStatsPath, "out_best_stats.%d.p%d.md", seed, indexParam);
	bestStats.open(bestPolicyStatsPath);
	bestStats << ps;
	bestStats.close();
	stats.close();

	// cleanup
	for (unsigned int i = 0; i < set.getNbInstructions(); i++) {
		delete (&set.getInstruction(i));
	}

	return 0;
}