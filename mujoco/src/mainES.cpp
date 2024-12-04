#include <iostream>
#include <numeric>
#include <thread>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <inttypes.h>
#include <getopt.h>
#define _USE_MATH_DEFINES // To get M_PI
#include <math.h>

#include "mujocoEnvironment/mujocoWrappers.h"
#include "instructions.h"

int main(int argc, char ** argv) {

    char option;
    uint64_t seed = 0;
    char paramFile[1500];
	char logsFolder[150];
	char xmlFile[150];
	char usecase[150];
	bool useHealthyReward = 1;
	bool useContactForce = 0;
	char dotPath[150];
    strcpy(dotPath, "logs/out_best.0.p0.dot");
    strcpy(logsFolder, "logs");
    strcpy(paramFile, "params/params_0.json");
	strcpy(usecase, "ant");
    strcpy(xmlFile, "none");
    while((option = getopt(argc, argv, "s:p:l:x:h:c:u:d:")) != -1){
        switch (option) {
            case 's': seed= atoi(optarg); break;
            case 'p': strcpy(paramFile, optarg); break;
            case 'l': strcpy(logsFolder, optarg); break;
			case 'u': strcpy(usecase, optarg); break;
			case 'h': useHealthyReward = atoi(optarg); break;
			case 'c': useContactForce = atoi(optarg); break;
            case 'd': strcpy(dotPath, optarg); break;
            case 'x': strcpy(xmlFile, optarg); break;
            default: std::cout << "Unrecognised option. Valid options are \'-s seed\' \'-d dot path\' \'-p paramFile.json\' \'-u useCase\' \'-logs logs Folder\'  \'-x xmlFile\' \'-h useHealthyReward\' \'-c useContactForce\'." << std::endl; exit(1);
        }
    }
	if(strcmp(xmlFile, "none") == 0){
    	snprintf(xmlFile, sizeof(xmlFile), "mujoco_models/%s.xml", usecase);
	}


    std::cout << "Selected seed : " << seed << std::endl;
    std::cout << "Selected params: " << paramFile << std::endl;

    // Save the index of the parameter file.
    int indexParam = std::stoi(std::regex_replace(paramFile, std::regex(R"(.*params_(\d+)\.json)"), "$1"));

	std::cout << "Start Mujoco ES application." << std::endl;
    // Create the instruction set for programs
	Instructions::Set set;
	fillInstructionSet(set);

	// Set the parameters for the learning process.
	// (Controls mutations probability, program lengths, and graph size
	// among other things)
	// Loads them from the file params.json
	Learn::LearningParameters params;
	File::ParametersParser::loadParametersFromJson(paramFile, params);

	std::cout << "Number of threads: " << params.nbThreads << std::endl;

	// Instantiate the LearningEnvironment
	MujocoWrapper* mujocoLE = nullptr;
	if(strcmp(usecase, "humanoid") == 0){
		mujocoLE = new MujocoHumanoidWrapper(xmlFile, useHealthyReward, useContactForce);
	} else if (strcmp(usecase, "half_cheetah") == 0) {
		mujocoLE = new MujocoHalfCheetahWrapper(xmlFile);
	} else if (strcmp(usecase, "hopper") == 0) {
		mujocoLE = new MujocoHopperWrapper(xmlFile);
	} else if (strcmp(usecase, "walker2D") == 0) {
		mujocoLE = new MujocoWalker2DWrapper(xmlFile);
	} else if (strcmp(usecase, "reacher") == 0) {
		mujocoLE = new MujocoReacherWrapper(xmlFile);
	} else if (strcmp(usecase, "ant") == 0) {
		mujocoLE = new MujocoAntWrapper(xmlFile, useHealthyReward, useContactForce);
	} else {
		throw std::runtime_error("Use case not found");
	}

	// Instantiate and init the learning agent
	Learn::ParallelLearningAgent la(*mujocoLE, set, params);
	la.init(seed);

    auto &tpg = *la.getTPGGraph();
    Environment env(set, params, mujocoLE->getDataSources(), mujocoLE->getNbContinuousAction());


    File::TPGGraphDotImporter dotImporter(dotPath, env, tpg);

    if(tpg.getNbRootVertices() > 1){
        
        std::cout<<"Multiple roots identified. One generation training launched to identified the best root"<<std::endl;
        // Basic logger
        Log::LABasicLogger basicLogger(la);
		auto results = la.evaluateAllRoots(-1, Learn::LearningMode::TRAINING);
        // Save the best score of this generation
        la.updateBestScoreLastGen(results);
        // Update the best
        la.updateEvaluationRecords(results);
        // Keep best policy
        la.keepBestPolicy();

        auto iter = results.begin();
        std::advance(iter, results.size() - 1);
        double max = iter->first->getResult();
        std::cout<<max<<std::endl;


        char bestDot[250];
        // Export the graph    
		std::cout<<"Former file "<<dotPath<<std::endl;
		size_t prefixLength = strstr(dotPath, ".dot") - dotPath;
		strncpy(bestDot, dotPath, prefixLength);
		bestDot[prefixLength] = '\0';  // Terminaison explicite
		strcat(bestDot, ".best.dot");
	    File::TPGGraphDotExporter dotExporter(bestDot, *la.getTPGGraph());
        dotExporter.print();
        std::cout<<"Save best root in "<<bestDot<<std::endl;

    }
	Learn::CMAESLearningAgent esLa(la);

	// Set validation to true because we would need it
	auto currentParams = esLa.getParams();
	currentParams.doValidation = true;
	esLa.setParams(currentParams);
	
	auto resultStart = esLa.evaluateAllRoots(0, Learn::LearningMode::VALIDATION).begin()->first->getResult();
	std::cout<<"\n\nStart Evolution Strategy with initial results "<<resultStart<<std::endl;
	std::cout<<"There is " << esLa.getDimension() << " weights to optimize with "<<esLa.getNbAgents()<<" agents"<<std::endl;

	// Basic Logger
	char logPathES[150];
	sprintf(logPathES, "%s/out_es.%d.p%d.%s.std", logsFolder, seed, indexParam, usecase);

	std::ofstream logStreamES;
	logStreamES.open(logPathES);
	Log::ESBasicLogger logES(esLa, logStreamES);

	Log::ESBasicLogger esBasicLogger(esLa);




	for(auto i = 0; i < params.nbGenerations; i++){
		esLa.trainOneGeneration(i);
	}







	// cleanup
	for (unsigned int i = 0; i < set.getNbInstructions(); i++) {
		delete (&set.getInstruction(i));
	}

	return 0;
}