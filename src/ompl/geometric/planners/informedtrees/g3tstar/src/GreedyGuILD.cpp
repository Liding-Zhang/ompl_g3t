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

#include "ompl/geometric/planners/informedtrees/g3tstar/GreedyGuILD.h"

namespace ompl
{
    namespace geometric
    {
        namespace g3tstar
        {
            GreedyGuILD::GreedyGuILD(const ompl::base::Cost &currentSolutionCost,
                                     const std::shared_ptr<g3tstar::State> &currentgoal,
                                     const double unitNBallMeasure,  //
                                     const std::vector<std::shared_ptr<State>> &startStates,
                                     const std::vector<std::shared_ptr<State>> &goalStates,
                                     const std::shared_ptr<ompl::base::OptimizationObjective> &objective,
                                     const unsigned int &dimension)
                : currentSolutionCost_(currentSolutionCost)
                , currentgoal_(currentgoal)
                , unitNBallMeasure_(unitNBallMeasure)
                , startStates_(startStates)
                , goalStates_(goalStates)
                , objective_(objective)
                , dimension_(dimension)

            {
            }
    
            std::shared_ptr<ompl::geometric::g3tstar::State> GreedyGuILD::getGreedyGuILDBeaconPoint()
            {
                std::vector<std::shared_ptr<State>> pathstates;
                auto goal = currentgoal_;

                // Get all states on the path of the current solution
                auto current = currentgoal_;
                while (!isStart(current))
                {
                    assert(current->asForwardVertex()->getParent().lock());
                    pathstates.emplace_back(current);
                    current = current->asForwardVertex()->getParent().lock()->getState();


                }

                // Delete the start and end points and the states directly connected to the start 
                // and end points to prevent the greedy guild from becoming a straight line. 
                // If the greedy guild becomes a straight line, new points cannot be added to the greedy guild area.
                pathstates.erase(pathstates.begin()); 

                if (pathstates.size() <= 3)
                {
                    return currentgoal_;
                }
                pathstates.erase(pathstates.begin());
                pathstates.erase(pathstates.end()); 


                // Traverse all points and find the point that makes the PHS Measure value the smallest
                double currentMinPHSMeasure = std::numeric_limits<double>::max();
                for (const auto& pathstate : pathstates) 
                {



                    double firstcost = pathstate->getCurrentCostToCome().value();
                    double secondcost = currentSolutionCost_.value() - pathstate->getCurrentCostToCome().value();


                    double totalPHSMeasure = caculatePHSMeasure(pathstate, goal, secondcost) + 
                                            caculatePHSMeasure(current, pathstate, firstcost);

                    
                    if (totalPHSMeasure < currentMinPHSMeasure && !isStart(pathstate) && !isGoal(pathstate))
                    {
                        currentMinPHSMeasure = totalPHSMeasure;
                        greedyGuILDBeaconPoint_ = pathstate;

                    }
                }

                setGUILDPHSMeasure(currentMinPHSMeasure);
                return greedyGuILDBeaconPoint_;
            }  

            double GreedyGuILD::caculatePHSMeasure(const std::shared_ptr<g3tstar::State> &fociA, const std::shared_ptr<g3tstar::State> &fociB, double dTransverse)
            {
                double dFoci = objective_->motionCost(fociA->raw(), fociB->raw()).value();
                const unsigned int N = dimension_;

                double conjugateDiameter;
                double lmeas;

                // Calculate the conjugate diameter:
                conjugateDiameter = std::sqrt(dTransverse * dTransverse - dFoci * dFoci);

                // Calculate the volume
                // First multiply together the radii, noting that the first radius is the transverse
                // diameter/2.0, and the other N-1 are the conjugate diameter/2.0
                lmeas = dTransverse / 2.0;
                for (unsigned int i = 1u; i < N; ++i)
                {
                    lmeas = lmeas * conjugateDiameter / 2.0;
                }

                // Then multiply by the volume of the unit n-ball.
                lmeas = lmeas * unitNBallMeasure_;

                return lmeas;
            }

            std::shared_ptr<ompl::geometric::g3tstar::State> GreedyGuILD::getFirstGreedyPoint() const
            {
                std::vector<std::shared_ptr<State>> states;
                std::shared_ptr<State> beacon;

                // Get all states on the path form start to the greedy guild beacon point. 
                auto current = greedyGuILDBeaconPoint_;
                while (!isStart(current))
                {
                    assert(current->asForwardVertex()->getParent().lock());
                    states.emplace_back(current);
                    current = current->asForwardVertex()->getParent().lock()->getState();
                }
                states.emplace_back(current);

                if (states.size() <= 2)
                {
                    return greedyGuILDBeaconPoint_;
                }
                
                // Traverse all points and find the point that makes the PHS Measure value the maxest.
                ompl::base::Cost maxCost;
                maxCost.setValue(0);
                for (const auto& state : states) 
                {
                    auto costtostart = state->getLowerBoundCostToCome();
                    auto costtoglobalbeacon = lowerBoundCostToBeacon(state);
                    ompl::base::Cost cost;
                    cost.setValue(costtostart.value() + costtoglobalbeacon.value());
                    if (cost.value() > maxCost.value()) {
                        maxCost = cost;
                        beacon = state;
                    }
                }
                return beacon;
            }

            std::shared_ptr<ompl::geometric::g3tstar::State> GreedyGuILD::getSecondGreedyPoint() const
            {
                std::vector<std::shared_ptr<State>> states;
                std::shared_ptr<State> Secondbeacon;

                // Get all states on the path form the greedy guild beacon point to goal. 
                auto current = currentgoal_;
                while (greedyGuILDBeaconPoint_ != current)
                {
                    assert(current->asForwardVertex()->getParent().lock());
                    states.emplace_back(current);
                    current = current->asForwardVertex()->getParent().lock()->getState();
                }
                states.emplace_back(current);
                if (states.size() <= 2)
                {
                    return greedyGuILDBeaconPoint_;
                }

                // Traverse all points and find the point that makes the PHS Measure value the maxest.
                ompl::base::Cost maxCost;
                maxCost.setValue(0);
                for (const auto& state : states) 
                {
                    auto costtoglobalbeacon = lowerBoundCostToBeacon(state);
                    auto costtogoal = state->getLowerBoundCostToGo();
                    ompl::base::Cost cost;
                    cost.setValue(costtoglobalbeacon.value() + costtogoal.value());
                    if (cost.value() > maxCost.value()) {
                        maxCost = cost;
                        Secondbeacon = state;
                    }
                }
                return Secondbeacon;
            }



            ompl::base::Cost GreedyGuILD::lowerBoundCostToBeacon(const std::shared_ptr<State> &state) const
            {
                ompl::base::Cost toBeaconCost;

                toBeaconCost = objective_->motionCostHeuristic(greedyGuILDBeaconPoint_.get()->raw(), state->raw());

                return toBeaconCost;
            }


            void GreedyGuILD::setGUILDPHSMeasure(double measure)
            {
                currentGuildPHSMeasure_ = measure;
            }

            double GreedyGuILD::getGUILDPHSMeasure()
            {
                return currentGuildPHSMeasure_;
            }

            bool GreedyGuILD::isStart(const std::shared_ptr<State> &state) const
            {
                return std::any_of(startStates_.begin(), startStates_.end(),
                                   [&state](const auto &start) { return state->getId() == start->getId(); });
            }

            bool GreedyGuILD::isGoal(const std::shared_ptr<State> &state) const
            {
                return std::any_of(goalStates_.begin(), goalStates_.end(),
                                   [&state](const auto &goal) { return state->getId() == goal->getId(); });
            }


        }  // namespace g3tstar

    }  // namespace geometric

}  // namespace ompl
