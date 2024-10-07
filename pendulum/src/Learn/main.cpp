#include <iostream>
#include <numeric>
#include <thread>
#include <atomic>
#include <chrono>
#include <inttypes.h>
#include <getopt.h>
#include <string>  // Added for std::string
#define _USE_MATH_DEFINES  // To get M_PI
#include <math.h>

#include "pendulum.h"
#include "instructions.h"

int main(int argc, char ** argv) {

    char option;
    uint64_t seed = 0;
    std::string paramFile = "params/params_0.json";
    std::string logsFolder = "logs";
    bool velocity = true;
    bool isContinuous = false;

    while ((option = getopt(argc, argv, "s:p:v:c:l:")) != -1) {
        switch (option) {
            case 's': seed = std::stoull(optarg); break;  // Use stoull for uint64_t
            case 'p': paramFile = optarg; break;
            case 'l': logsFolder = optarg; break;
            case 'v': velocity = std::stoi(optarg); break;  // Use stoi for bool
            case 'c': isContinuous = std::stoi(optarg); break;
            default:
                std::cout << "Unrecognized option. Valid options are '-s seed' '-p paramFile.json' '-l logsFolder' '-v velocity' '-c isContinuous'." << std::endl;
                exit(1);
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
    Learn::LearningParameters params;
    File::ParametersParser::loadParametersFromJson(paramFile.c_str(), params);

#ifdef NB_GENERATIONS
    params.nbGenerations = NB_GENERATIONS;
#endif

    // Instantiate the LearningEnvironment
    Pendulum pendulumLE({ 0.05, 0.1, 0.2, 0.4, 0.6, 0.8, 1.0 }, velocity, isContinuous);

    std::cout << "Number of threads: " << params.nbThreads << std::endl;

    // Instantiate and init the learning agent
    Learn::ParallelLearningAgent la(pendulumLE, set, params);
    la.init(seed);

    const TPG::TPGVertex* bestRoot = nullptr;

    std::atomic<bool> exitProgram = false;  // Set to false by other thread
    std::atomic<bool> toggleDisplay = false;

    // Basic logger
    Log::LABasicLogger basicLogger(la);

    // Construct log file path
    std::string logPath = logsFolder + "/out." + std::to_string(seed) + ".p" + std::to_string(indexParam) +
                          ".v" + std::to_string(velocity) + ".c" + std::to_string(isContinuous) + ".std";
    std::ofstream logStream(logPath);
    Log::LABasicLogger log(la, logStream);

    // Create an exporter for all graphs
    std::string dotPath = logsFolder + "/out_0000." + std::to_string(seed) + ".p" + std::to_string(indexParam) +
                          ".v" + std::to_string(velocity) + ".c" + std::to_string(isContinuous) + ".dot";
    File::TPGGraphDotExporter dotExporter(dotPath, *la.getTPGGraph());

    // Logging best policy stat
    std::string bestPolicyStatsPath = logsFolder + "/bestPolicyStats." + std::to_string(seed) +
                                      ".p" + std::to_string(indexParam) + ".v" + std::to_string(velocity) +
                                      ".c" + std::to_string(isContinuous) + ".md";
    std::ofstream stats(bestPolicyStatsPath);
    Log::LAPolicyStatsLogger policyStatsLogger(la, stats);

    // Export parameters before starting training
    std::string jsonFilePath = logsFolder + "/exported_params.json";
    File::ParametersParser::writeParametersToJson(jsonFilePath.c_str(), params);

    // Train for params.nbGenerations generations
    for (int i = 0; i < params.nbGenerations && !exitProgram; i++) {
#define PRINT_ALL_DOT 0
#if PRINT_ALL_DOT
        std::string buff = logsFolder + "/out_" + std::to_string(i) + "." + std::to_string(seed) + 
                           ".p" + std::to_string(indexParam) + ".v" + std::to_string(velocity) + 
                           ".c" + std::to_string(isContinuous) + ".dot";
        dotExporter.setNewFilePath(buff);
        dotExporter.print();
#endif
        la.trainOneGeneration(i);
    }

    // Keep best policy
    la.keepBestPolicy();

    // Clear intron instructions
    la.getTPGGraph()->clearProgramIntrons();

    // Export the best graph
    std::string bestDot = logsFolder + "/out_best." + std::to_string(seed) + ".p" + std::to_string(indexParam) +
                          ".v" + std::to_string(velocity) + ".c" + std::to_string(isContinuous) + ".dot";
    dotExporter.setNewFilePath(bestDot);
    dotExporter.print();

    // Policy stats
    TPG::PolicyStats ps;
    ps.setEnvironment(la.getTPGGraph()->getEnvironment());
    ps.analyzePolicy(la.getBestRoot().first);
    std::ofstream bestStats;
    std::string bestStatsPath = logsFolder + "/out_best_stats." + std::to_string(seed) + ".p" +
                                std::to_string(indexParam) + ".v" + std::to_string(velocity) +
                                ".c" + std::to_string(isContinuous) + ".md";
    bestStats.open(bestStatsPath);
    bestStats << ps;
    bestStats.close();
    stats.close();

    // Cleanup
    for (unsigned int i = 0; i < set.getNbInstructions(); i++) {
        delete (&set.getInstruction(i));  // Safe deletion of instructions
    }

    return 0;
}
