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

#include "regression.h"
#include "instructions.h"

int main(int argc, char ** argv) {

    char option;
    uint64_t seed = 0;
    char paramFile[1500];
	char logsFolder[150];
	char dotPath[150];
    strcpy(dotPath, "logs/out_best.0.p0.dot");
    strcpy(logsFolder, "logs");
    strcpy(paramFile, "params/params_0.json");
    while((option = getopt(argc, argv, "s:p:l:d:")) != -1){
        switch (option) {
            case 's': seed= atoi(optarg); break;
            case 'p': strcpy(paramFile, optarg); break;
            case 'l': strcpy(logsFolder, optarg); break;
            case 'd': strcpy(dotPath, optarg); break;
            default: std::cout << "Unrecognised option. Valid options are \'-s seed\' \'-d dot path\' \'-p paramFile.json\' \'-u useCase\' \'-logs logs Folder\'  \'-x xmlFile\' \'-h useHealthyReward\' \'-c useContactForce\'." << std::endl; exit(1);
        }
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
	RegressionWrapper regressionLE;

	// Instantiate and init the learning agent
	Learn::ParallelLearningAgent la(regressionLE, set, params);
	la.init(seed);

    auto &tpg = *la.getTPGGraph();
    Environment env(set, params, regressionLE.getDataSources(), regressionLE.getNbContinuousAction());

    //File::TPGGraphDotImporter dotImporter(dotPath, env, tpg);

	std::cout<<"connard"<<std::endl;
    if(tpg.getNbRootVertices() > 1 && false){
        
        std::cout<<"Multiple roots identified. One generation training launched to identified the best root"<<std::endl;
        // Basic logger
        Log::LABasicLogger basicLogger(la);
		auto results = la.evaluateAllRoots(0, Learn::LearningMode::TRAINING);
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
        strncpy(bestDot, dotPath, strstr(dotPath, ".dot") - dotPath);
        strcat(bestDot, ".best.dot");
	    File::TPGGraphDotExporter dotExporter(bestDot, *la.getTPGGraph());
        dotExporter.print();
        std::cout<<"Save best root in "<<bestDot<<std::endl;

    }

	la.getTPGGraph()->clear();
	la.getTPGGraph()->addNewAction(0, 0);
	std::shared_ptr<Program::Program> prog = std::make_shared<Program::Program>(la.getTPGGraph()->getEnvironment(), true);
	// RandomInit the Programs
	

	Program::Line& l0 = prog->addNewLine();
	l0.setInstructionIndex(2); 
	l0.setOperand(0, 1, 0);    
	l0.setOperand(1, 1, 0);   
	l0.setDestinationIndex(0); // R[0] = S[0]*S[0]*w0
	l0.setNbConstants(2);

	Program::Line& l1 = prog->addNewLine();
	l1.setInstructionIndex(1); 
	l1.setOperand(0, 0, 0);    
	l1.setOperand(1, 1, 0);   
	l1.setDestinationIndex(1); // R[1] = R[0]*w1 + S[0]*w2
	l1.setNbConstants(3);

	Program::Line& l2 = prog->addNewLine();
	l2.setInstructionIndex(3); 
	l2.setOperand(0, 0, 1);   
	l2.setDestinationIndex(0); // R[0] = sin(R[1]*w3)
	l2.setNbConstants(2);
	size_t nbConstants = 9;
	
	/*for(size_t i = 0; i< 8; i++){
		Program::Line& l0 = prog->addNewLine();
		l0.setInstructionIndex(4); 
		l0.setOperand(0, 1, 0); 
		l0.setDestinationIndex(i); // R[0] = S[0]*S[0]*w0
		l0.setNbConstants(2);
	}

	for(size_t i = 0; i< 8; i++){
		Program::Line& l0 = prog->addNewLine();
		l0.setInstructionIndex(5); 
		for(size_t j = 0; j < 8; j++){
			l0.setOperand(j, 0, j); 
		}
		l0.setDestinationIndex(i+8); // R[0] = S[0]*S[0]*w0
		l0.setNbConstants(9);
	}

	Program::Line& l0 = prog->addNewLine();
	l0.setInstructionIndex(7); 
	for(size_t j = 0; j < 8; j++){
		l0.setOperand(j, 0, j+8); 
	}
	l0.setDestinationIndex(0); // R[0] = S[0]*S[0]*w0
	l0.setNbConstants(9);
	size_t nbConstants = 225; //9(even if 2)*8 + 8*9 + 8*9 + 1*9*/

	/// Generate False Neural network


	std::vector<double> constants;
	for(size_t idx=0; idx<nbConstants; idx++){
		constants.push_back(la.getRNG().getDouble(0, 1.0));
	}
	prog->setLineConstants(constants);

	la.getTPGGraph()->addNewActionEdge(*la.getTPGGraph()->getVertices().at(0), prog, 0);


	Learn::ParallelEvoStratLearningAgent esLa(la);

	// Set validation to true because we would need it
	auto currentParams = esLa.getParams();
	currentParams.doValidation = true;
	esLa.setParams(currentParams);

std::cout<<std::endl;
	auto constants2 = la.getTPGGraph()->getVertices().at(0)->getOutgoingEdges().front()->getProgram().getLineConstants();
    for(double idx: constants2){
        std::cout<<idx<<",";
    }std::cout<<std::endl;
	
	auto resultStart = esLa.evaluateAllRoots(0, Learn::LearningMode::VALIDATION).begin()->first->getResult();
	std::cout<<"\n\nStart Evolution Strategy with initial results "<<resultStart<<"\n"<<std::endl;
	//return 0;

	// Basic Logger
	char logPathES[150];
	sprintf(logPathES, "%s/out_es.%d.p%d.std", logsFolder, seed, indexParam);

	std::ofstream logStreamES;
	logStreamES.open(logPathES);
	Log::ESBasicLogger logES(esLa, logStreamES);

	Log::ESBasicLogger esBasicLogger(esLa);




	for(auto i = 0; i < 2000; i++){
		esLa.trainOneGeneration(i);
	}







	// cleanup
	for (unsigned int i = 0; i < set.getNbInstructions(); i++) {
		delete (&set.getInstruction(i));
	}

	return 0;
}