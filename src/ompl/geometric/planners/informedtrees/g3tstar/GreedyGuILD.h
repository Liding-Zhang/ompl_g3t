/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Technical University of Munich
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the University of Munich nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Authors: Liding Zhang, Yao Ling

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_TESTSTAR_GREEDYGUILD_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_TESTSTAR_GREEDYGUILD_

#include "ompl/geometric/planners/informedtrees/g3tstar/RandomGeometricGraph.h"
#include "ompl/geometric/planners/informedtrees/G3Tstar.h"
#include "ompl/geometric/planners/informedtrees/g3tstar/Edge.h"


namespace ompl
{
    namespace geometric
    {
        namespace g3tstar
        {
            class GreedyGuILD
            {
            public:
                GreedyGuILD(const ompl::base::Cost &currentSolutionCost,
                            const std::shared_ptr<g3tstar::State> &currentgoal,
                            const double unitNBallMeasure,  //
                            const std::vector<std::shared_ptr<State>> &startStates,
                            const std::vector<std::shared_ptr<State>> &goalStates,
                            const std::shared_ptr<ompl::base::OptimizationObjective> &objective,
                            const unsigned int &dimension);

                ~GreedyGuILD() = default;

                /** \brief Returns the greedy GuILD beacon point in the path that makes the PHSMeasure value the smallest. */
                std::shared_ptr<ompl::geometric::g3tstar::State> getGreedyGuILDBeaconPoint();

                /** \brief Returns to the first greedy point that maximizes the sum of its cost to the start point and the cost to the beacon point. */
                std::shared_ptr<ompl::geometric::g3tstar::State> getFirstGreedyPoint() const;

                /** \brief Return to the second greedy point that maximizes the sum of its cost to the goal point and the cost to the beacon point. */
                std::shared_ptr<ompl::geometric::g3tstar::State> getSecondGreedyPoint() const;
                
                /** \brief Sets the greedy Guild PHS measure. */
                void setGUILDPHSMeasure(double measure);

                /** \brief Gets the greedy Guild PHS measure. */
                double getGUILDPHSMeasure();


            private:
                /** \brief The cost of the current path. */
                ompl::base::Cost currentSolutionCost_;

                /** \brief The current goal state. */
                std::shared_ptr<g3tstar::State> currentgoal_;

                /** \brief Const reference to the measure of a unit ball in n dimensions. */
                const double unitNBallMeasure_;

                /** \brief Const reference to the start states of the problem. */
                const std::vector<std::shared_ptr<State>> &startStates_;
                
                /** \brief Const reference to the goal states of the problem. */
                const std::vector<std::shared_ptr<State>> &goalStates_;

                /** \brief Const reference to the optimization objective this GuILD is supposed to help optimize. */
                std::shared_ptr<ompl::base::OptimizationObjective> objective_;

                /** \brief Const reference to the measure of a unit ball in n dimensions. */
                unsigned int dimension_;

                /** \brief The greedy Guild PHS measure. */
                double currentGuildPHSMeasure_;

                /** \brief The greedy GuILD beacon point in the path that makes the PHSMeasure value the smallest. */
                std::shared_ptr<ompl::geometric::g3tstar::State> greedyGuILDBeaconPoint_;

                /** \brief Returns measure of the given PHS. */
                double caculatePHSMeasure(const std::shared_ptr<g3tstar::State> &fociA, const std::shared_ptr<g3tstar::State> &fociB, double dTransverse);

                /** \brief Returns the lower bound cost to the beacon point. */
                ompl::base::Cost lowerBoundCostToBeacon(const std::shared_ptr<State> &state) const;

                /** \brief Returns true if the given state is one of the starts. */
                bool isStart(const std::shared_ptr<State> &state) const;

                /** \brief Returns true if the given state is one of the goals. */
                bool isGoal(const std::shared_ptr<State> &state) const;


            };

        }  // namespace g3tstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_TESTSTAR_GREEDYGUILD_