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

#include "ompl/geometric/planners/informedtrees/g3tstar/Grafting.h"

namespace ompl
{
    namespace geometric
    {
        namespace g3tstar
        {
            Grafting::Grafting(G3Tstar &g3tstar,
                               const std::shared_ptr<g3tstar::RandomGeometricGraph> &graph,
                               const std::shared_ptr<ompl::base::OptimizationObjective> &objective,
                               const g3tstar::Edge &edge)
                : g3tstar_(g3tstar)
                , graph_(std::move(graph))
                , objective_(objective)
                , edge_(edge)

            {
            }
    

            void Grafting::getCommonNeighbors(const g3tstar::Edge &edge)
            {
                std::vector<std::weak_ptr<State>> sourceNeighbors;
                std::unordered_set<size_t> targetNeighborsId;
                for (const auto &neighborState : graph_->getNeighbors(edge.source))
                {
                    // Find all neighbors of source
                    sourceNeighbors.emplace_back(neighborState);
                }   
                for (const auto &neighborState : graph_->getNeighbors(edge.target))
                {              
                    // Find all neighbors of target
                    targetNeighborsId.insert(neighborState.lock()->getId() );
                }

                // Find common neighbors
                for (const auto &neighbor : sourceNeighbors)
                {
                    if (targetNeighborsId.count(neighbor.lock()->getId()))
                    {
                        commonNeighbors_.push_back(neighbor);
                    }
                }
            }
            
            void Grafting::getSortedNeighborEdgePairs()
            {   
                // Store all pairs of edges between neighbors and source and target in order of cost
                std::vector<std::tuple<std::pair<g3tstar::Edge, g3tstar::Edge>, double>> edgesWithCosts;
                for (const auto &neighbor : commonNeighbors_) {
                    auto sourceEdgeCost = objective_->motionCost(edge_.source->raw(), neighbor.lock()->raw());
                    auto targetEdgeCost = objective_->motionCost(neighbor.lock()->raw(), edge_.target->raw());
                    auto totalCost = objective_->combineCosts(sourceEdgeCost, targetEdgeCost);

                    edgesWithCosts.emplace_back(
                        std::make_pair(g3tstar::Edge(edge_.source, neighbor.lock()), 
                                    g3tstar::Edge(neighbor.lock(), edge_.target)),
                        totalCost.value()
                    );
                }

                std::sort(edgesWithCosts.begin(), edgesWithCosts.end(),
                        [](const auto &a, const auto &b) {
                            return std::get<1>(a) < std::get<1>(b);
                        });
                // Extract the sorted edge pairs
                std::vector<std::pair<g3tstar::Edge, g3tstar::Edge>> sortedEdges;
                for (const auto &item : edgesWithCosts) {
                    sortedEdges.push_back(std::get<0>(item));
                }

                sortedNeighborEdgePairs_ =  sortedEdges;
            }

            bool Grafting::foundValidNeighborEdgePair()
            {
                getCommonNeighbors(edge_);
                if (commonNeighbors_.empty())
                {
                    return false;
                }
                getSortedNeighborEdgePairs();

                if (sortedNeighborEdgePairs_.empty())
                {
                    return false;
                }
                
                for (const auto &edgePair : sortedNeighborEdgePairs_) 
                {
                    if (g3tstar_.couldBeValidPublic(edgePair.first) && g3tstar_.couldBeValidPublic(edgePair.second)) 
                    {
                        if (g3tstar_.isValidPublic(edgePair.first) && g3tstar_.isValidPublic(edgePair.second)) 
                        {
                            bestEdgePair_ = edgePair;
                            return true;
                        }
                    }
                }
                return false;
            }

            std::pair<g3tstar::Edge, g3tstar::Edge> Grafting::getBestEdgePair() const
            {
                return bestEdgePair_;
            }  

        }  // namespace g3tstar

    }  // namespace geometric

}  // namespace ompl
