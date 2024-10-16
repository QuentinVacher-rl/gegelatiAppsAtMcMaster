
#ifndef GRIDWORLD_H
#define GRIDWORLD_H

#include <gegelati.h>

class GridWorld : public Learn::LearningEnvironment{

    private:
        
        /**
         * \brief grid of the GridWorld
         *  - 0 are accessible tiles
         *  - 1 is good output
         *  - 2 is bad output
         *  - 3 are unaccessible tiles
         */ 
        std::vector<std::vector<int>> grid = {{ 0, 0, 0, 2},
                                              { 0, 0, 3, 3},
                                              { 0, 0, 0, 1}};

        /// Size of the grid
        std::vector<int> size = {4, 3};

        /// Position of the agent
        std::vector<int> agentCoord = {0 , 0};

        /// True if the episode is terminated
        bool terminated = false;

        /// Total reward accumulated since the last reset
	    double score = 0.0;

        /// Current State
        Data::PrimitiveTypeArray<double> currentState;

    public:

        GridWorld() : LearningEnvironment((uint64_t) 4), currentState(2) {};

        GridWorld(const GridWorld& other) = default;

	    /// Inherited via LearningEnvironment
	    virtual void reset(size_t seed = 0, Learn::LearningMode mode = Learn::LearningMode::TRAINING,
					   uint16_t iterationNumber = 0, uint64_t generationNumber = 0) override;

        /// @brief Return true if the position indicated is available
        /// @param pos_x Coordonate on axis x
        /// @param pos_y Coordonate on axis y
        /// @return boolean that indicate if the position is available
        bool positionAvailable(int pos_x, int pos_y);

        /// Inherited via LearningEnvironment
        virtual void doAction(uint64_t actionID) override;

        /// Inherited via LearningEnvironment
        virtual double getScore() const override;

        /// Inherited via LearningEnvironment
        virtual bool isTerminal() const override;

        /// Inherited via LearningEnvironment
        virtual std::vector<std::reference_wrapper<const Data::DataHandler>> getDataSources() override;

        /// Inherited via LearningEnvironment
        virtual LearningEnvironment* clone() const;

        /// Inherited via LearningEnvironment
	    virtual bool isCopyable() const override;
};

#endif