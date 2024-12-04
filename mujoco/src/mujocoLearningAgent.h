#ifndef MUJOCO_LEARNING_AGENT_H
#define MUJOCO_LEARNING_AGENT_H

#include <gegelati.h>
#include "mujocoEnvironment/mujocoWrappers.h"

class MultiMujocoLearningAgent : public Learn::ParallelLearningAgent
{
    private:
    
        std::multimap<double, const TPG::TPGVertex*, std::less<>> stdGlobalScore;

    public:
        MultiMujocoLearningAgent(
            MultiMujocoWrapper& le,
            const Instructions::Set& iSet, const Learn::LearningParameters& p,
            const TPG::TPGFactory& factory = TPG::TPGFactory())
            : ParallelLearningAgent(le, iSet, p, factory){};

        
        std::shared_ptr<Learn::EvaluationResult> evaluateJob(
            TPG::TPGExecutionEngine& tee, const Learn::Job& job, uint64_t generationNumber,
            Learn::LearningMode mode, Learn::LearningEnvironment& le) const override
        {
            MultiMujocoWrapper* multiLE = dynamic_cast<MultiMujocoWrapper*>(&le);

            // Only consider the first root of jobs as we are not in adversarial mode
            const TPG::TPGVertex* root = job.getRoot();

            // Skip the root evaluation process if enough evaluations were already
            // performed. In the evaluation mode only.
            std::shared_ptr<Learn::EvaluationResult> previousEval;
            if (mode == Learn::LearningMode::TRAINING &&
                this->isRootEvalSkipped(*root, previousEval)) {
                return previousEval;
            }

            // Init results
            double result = 0.0;
            double resultMSE = 0.0;
            double meanNbActionUsed = 0.0;

            std::vector<double> resultWrapper(multiLE->getNbWrapper(), 0);


            // Evaluate  nbIteration times
            for (auto iterationNumber = 0;
                iterationNumber < this->params.nbIterationsPerPolicyEvaluation;
                iterationNumber++) {
                // Compute a Hash
                Data::Hash<uint64_t> hasher;
                uint64_t hash = hasher(generationNumber) ^ hasher(iterationNumber);

                // Reset the learning Environment
                le.reset(hash, mode, iterationNumber, generationNumber);

                // Reset the memory registers.
                tee.resetAllMemoryRegisters();

                double nbActionUsed = 0;


                uint64_t nbActions = 0;
                while (!le.isTerminal() &&
                    nbActions < this->params.maxNbActionsPerEval) {
                    // Get the actions
                    std::vector<double> actionsID =
                        tee.executeFromRoot(*root, le.getInitActions(),
                                            this->params.nbEdgesActivable)
                            .second;

                    for(int i = 0; i<le.getNbContinuousAction(); i++){
                        if(actionsID[i] != 0.0){
                            nbActionUsed++;
                        }
                    }
                    // Do it
                    le.doActions(actionsID);
                    // Count actions
                    nbActions++;


                }
                nbActionUsed = nbActionUsed / nbActions;
                meanNbActionUsed += nbActionUsed;

                // Update results
                result += le.getScore();
                resultMSE +=  (le.getScore() > 0) ? std::pow(le.getScore(), 2) : -std::pow(le.getScore(), 2);

                for(size_t idx = 0; idx < multiLE->getNbWrapper(); idx++){
                    resultWrapper.at(idx) += multiLE->getWrapperAt(idx)->getScore();
                }
            }

            meanNbActionUsed /= (double)params.nbIterationsPerPolicyEvaluation;
            result /= (double)params.nbIterationsPerPolicyEvaluation;
            resultMSE /= (double)params.nbIterationsPerPolicyEvaluation * 1000;
            
            for(size_t idx = 0; idx < multiLE->getNbWrapper(); idx++){
                resultWrapper.at(idx) /= (double)params.nbIterationsPerPolicyEvaluation;
            }

            std::vector<double> info;
            info.push_back(meanNbActionUsed);

            for(auto wrapperScore: resultWrapper){
                info.push_back(wrapperScore);
            }
            // Create the EvaluationResult
            auto evaluationResult =
                std::shared_ptr<Learn::EvaluationResult>(new Learn::EvaluationResult(
                    (this->params.useMSE) ? resultMSE : result, params.nbIterationsPerPolicyEvaluation, info, result));

            // Combine it with previous one if any
            if (previousEval != nullptr) {
                *evaluationResult += *previousEval;
            }
            return evaluationResult;
        } 


        void decimateWorstRoots(
            std::multimap<std::shared_ptr<Learn::EvaluationResult>, const TPG::TPGVertex*>&
                results) override
        {

            std::multimap<double, const TPG::TPGVertex*>
                preservedVertex;

            uint64_t nbEnv = results.begin()->first->getInfoSupp().size() - 1;
            std::set<const TPG::TPGVertex*> bestVertexPerEnv;
            uint64_t nbBestVertexPerEnv = 30;
            stdGlobalScore.clear();


            std::vector<std::multimap<double, const TPG::TPGVertex*, std::greater_equal<>>> stdScorePerEnv;

            double indexAvoidEqualties = 0.0;
            for (size_t i = 0; i < nbEnv; i++) {
                //std::cout<<"Env "<<i<<": ";
                std::multimap<double, const TPG::TPGVertex*, std::greater_equal<>> scoreThisEnv;

                // Remplir le multimap avec les scores et les Vertex
                for (auto& pair : results) {
                    double score = pair.first->getInfoSupp().at(i + 1);
                    scoreThisEnv.emplace(score, pair.second); // Ajouter au multimap
                    //std::cout<<score<<"-";
                }//std::cout<<std::endl;

                // Calcul de la moyenne
                double mean = std::accumulate(
                    scoreThisEnv.begin(), scoreThisEnv.end(), 0.0,
                    [](double acc, const auto& pair) { return acc + pair.first; }) / scoreThisEnv.size();

                // Calcul de l'écart-type
                double sq_sum = std::accumulate(
                    scoreThisEnv.begin(), scoreThisEnv.end(), 0.0,
                    [mean](double acc, const auto& pair) {
                        return acc + (pair.first - mean) * (pair.first - mean);
                    });
                double stddev = std::sqrt(sq_sum / scoreThisEnv.size());

                // Vérification de l'écart-type
                if (stddev == 0.0) {
                    std::cerr << "Error: Standard deviation is 0, cannot standardize." << std::endl;
                    return;
                }

                // Standardiser les scores
                std::multimap<double, const TPG::TPGVertex*, std::greater_equal<>> standardizedScoreThisEnv;
                for (const auto& pair : scoreThisEnv) {
                    double standardizedScore = (pair.first - mean) / stddev;
                    standardizedScore += indexAvoidEqualties / 10000000.0;
                    standardizedScoreThisEnv.emplace(standardizedScore, pair.second);

                    indexAvoidEqualties++;
                    //std::cout<<standardizedScore<<"-";
                }//std::cout<<std::endl;

                // Ajouter au vecteur
                stdScorePerEnv.push_back(std::move(standardizedScoreThisEnv));
            }

            // Vector to store combined standardized scores for environment pairs
            std::vector<std::multimap<double, const TPG::TPGVertex*, std::greater_equal<>>> stdScorePerCombEnv;

            // Iterate over each pair of environments
            for (size_t i = 0; i < nbEnv; ++i) {
                for (size_t j = i + 1; j < nbEnv; ++j) {
                    // Multimap to store average scores for the current combination of environments
                    std::multimap<double, const TPG::TPGVertex*, std::greater_equal<>> scoreThisCombEnv;

                    // Iterate over all vertices in environment i
                    for (const auto& [scoreI, vertex] : stdScorePerEnv[i]) {
                        // Check if the same vertex exists in environment j
                        auto it = std::find_if(stdScorePerEnv[j].begin(), stdScorePerEnv[j].end(),
                                            [vertex](const auto& pair) {
                                                return pair.second == vertex;
                                            });

                        if (it != stdScorePerEnv[j].end()) {
                            double scoreJ = it->first; // Get the score in environment j

                            // Calculate the mean score
                            double meanScore = (scoreI + scoreJ) / 2.0;

                            // Add the mean score as key, and the vertex as value
                            scoreThisCombEnv.emplace(meanScore, vertex);
                        }
                    }

                    // Add the sorted scores for the current combination of environments
                    stdScorePerCombEnv.push_back(std::move(scoreThisCombEnv));
                }
            }

            // Temporary map to keep track of minimum scores
            std::unordered_map<const TPG::TPGVertex*, double> tempMinScores;

            // Find minimum scores across all environments
            for (const auto& envMap : stdScorePerEnv) {
                for (const auto& [score, vertex] : envMap) {
                    if (tempMinScores.find(vertex) == tempMinScores.end()) {
                        tempMinScores[vertex] = score;
                    } else {
                        tempMinScores[vertex] = std::min(tempMinScores[vertex], score);
                    }
                }
            }
            // Insert into stdGlobalScore and maintain sorting
            for (const auto& [vertex, minScore] : tempMinScores) {
                stdGlobalScore.emplace(minScore, vertex);
            }


            for (const auto& envMap : stdScorePerEnv) {
                size_t count = 0;

                // Iterate through the multimap, which is already sorted by score
                for (const auto& [score, vertex] : envMap) {
                    if (count < nbBestVertexPerEnv) {
                        bestVertexPerEnv.insert(vertex); // Add the vertex to the top list
                        count++;
                    } else {
                        break; // Stop after collecting 5 vertices
                    }
                }


            }
            for (const auto& envMap : stdScorePerCombEnv) {
                size_t count = 0;

                // Iterate through the multimap, which is already sorted by score
                for (const auto& [score, vertex] : envMap) {
                    if (count < nbBestVertexPerEnv) {
                        bestVertexPerEnv.insert(vertex); // Add the vertex to the top list
                        count++;
                    } else {
                        break; // Stop after collecting 5 vertices
                    }
                }
            }

            // Determine the numbers of roots to delete
            int nbRootsToDelete =
                std::max((this->tpg->getNbRootVertices() - params.mutation.tpg.nbRoots),
                        (uint64_t)0) +
                (int)floor(this->params.ratioDeletedRoots *
                        (double)params.mutation.tpg.nbRoots);

            auto roots = tpg->getRootVertices();
            uint64_t currentNumberOfActionRoot = std::count_if(roots.begin(), roots.end(),
                [](const TPG::TPGVertex* roots) {
                    return dynamic_cast<const TPG::TPGAction*>(roots) != nullptr;
                });
            uint64_t currentNumberOfTeamRoot = std::count_if(roots.begin(), roots.end(),
                [](const TPG::TPGVertex* roots) {
                    return dynamic_cast<const TPG::TPGTeam*>(roots) != nullptr;
                });


            if(params.mutation.tpg.proportionActionRoots + params.mutation.tpg.proportionTeamRoots > 1){
                throw std::runtime_error("Too many proportion!");
            }

            uint64_t nbActionsMin = params.mutation.tpg.proportionActionRoots * params.mutation.tpg.nbRoots * params.ratioDeletedRoots;
            uint64_t nbTeamMin = params.mutation.tpg.proportionTeamRoots * params.mutation.tpg.nbRoots * params.ratioDeletedRoots;

            auto i = 0;
            while (i < nbRootsToDelete && stdGlobalScore.size() > 0) {

                // If the root is an action, do not remove it!
                const TPG::TPGVertex* root = stdGlobalScore.begin()->second;

                // Action can now be removed when continuous action are used
                if (bestVertexPerEnv.find(root) != bestVertexPerEnv.end()){
                    preservedVertex.insert(*stdGlobalScore.begin());
                    i--; // no vertex was actually removed
                }
                else if (dynamic_cast<const TPG::TPGAction*>(root) != nullptr && currentNumberOfActionRoot > nbActionsMin) {
                    tpg->removeVertex(*stdGlobalScore.begin()->second);
                    // Removed stored result (if any)
                    this->resultsPerRoot.erase(stdGlobalScore.begin()->second);
                }
                else if (dynamic_cast<const TPG::TPGTeam*>(root) != nullptr && currentNumberOfTeamRoot > nbTeamMin){
                    tpg->removeVertex(*stdGlobalScore.begin()->second);
                    // Removed stored result (if any)
                    this->resultsPerRoot.erase(stdGlobalScore.begin()->second);
                } else {
                    preservedVertex.insert(*stdGlobalScore.begin());
                    i--; // no vertex was actually removed
                }
                stdGlobalScore.erase(stdGlobalScore.begin());

                // Increment loop counter
                i++;


                roots = tpg->getRootVertices();
                currentNumberOfActionRoot = std::count_if(roots.begin(), roots.end(),
                    [](const TPG::TPGVertex* roots) {
                        return dynamic_cast<const TPG::TPGAction*>(roots) != nullptr;
                    });
                currentNumberOfTeamRoot = std::count_if(roots.begin(), roots.end(),
                    [](const TPG::TPGVertex* roots) {
                        return dynamic_cast<const TPG::TPGTeam*>(roots) != nullptr;
                    });

            }
            stdGlobalScore.insert(preservedVertex.begin(), preservedVertex.end());

        }

        void updateEvaluationRecords(
            const std::multimap<std::shared_ptr<Learn::EvaluationResult>, const TPG::TPGVertex*>& results) override
        {
            { // Update resultsPerRoot
                for (auto score : stdGlobalScore) {
                    
                    // Finding the equivalent in results using the vertex
                    auto it = std::find_if(
                        results.begin(),
                        results.end(),
                        [&score](const auto& pair) {
                            return pair.second == score.second;
                        });

                    if (it == results.end()) {
                        // If no equivalent found, skip this iteration
                        continue;
                    }
                    // Create the pair for the result
                    std::pair<std::shared_ptr<Learn::EvaluationResult>, const TPG::TPGVertex*> result = *it;

                    auto mapIterator = this->resultsPerRoot.find(result.second);
                    if (mapIterator == this->resultsPerRoot.end()) {
                        // First time this root is evaluated
                        this->resultsPerRoot.emplace(result.second, result.first);
                    }
                    else if (result.first != mapIterator->second) {
                        // This root has already been evaluated.
                        // If the received result pointer is different from the one
                        // stored in the map, update the one in the map by replacing it
                        // with the new one (which was combined with the pre-existing
                        // one in evalRoot)
                        mapIterator->second = result.first;
                        // If the received result is associated to the current bestRoot,
                        // update it.
                        if (result.second == this->bestRoot.first) {
                            this->bestRoot.second = result.first;
                        }
                    }
                }
            }
            { // Update bestRoot
                auto iterator = --stdGlobalScore.end();

                // Finding the equivalent in results using the vertex
                auto it = std::find_if(
                    results.begin(),
                    results.end(),
                    [&iterator](const auto& pair) {
                        return pair.second == iterator->second;
                    });

                if (it == results.end()) {
                    throw std::runtime_error("Best root not found in results");
                }

                const std::shared_ptr<Learn::EvaluationResult> evaluation = it->first;
                const TPG::TPGVertex* candidate = it->second;
                // Test the three replacement cases
                // from the simpler to the most complex to test
                if (this->bestRoot.first == nullptr         // NULL case
                    || *this->bestRoot.second < *evaluation // new high-score case
                    || !this->tpg->hasVertex(
                        *this->bestRoot.first) // bestRoot disappearance
                ) {
                    // Replace the best root
                    this->bestRoot = {candidate, evaluation};
                }

                // Otherwise do nothing
            
            }
        }
    
};
#endif // !MUJOCO_LEARNING_AGENT_H