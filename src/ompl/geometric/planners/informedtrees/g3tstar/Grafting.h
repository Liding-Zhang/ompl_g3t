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

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_TESTSTAR_GRAFTING_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_TESTSTAR_GRAFTING_

#include "ompl/geometric/planners/informedtrees/g3tstar/RandomGeometricGraph.h"
#include "ompl/geometric/planners/informedtrees/G3Tstar.h"
#include "ompl/geometric/planners/informedtrees/g3tstar/Edge.h"


namespace ompl
{
    namespace geometric
    {
        namespace g3tstar
        {
            class Grafting
            {
            public:
                Grafting(G3Tstar &g3tstar,
                         const std::shared_ptr<g3tstar::RandomGeometricGraph> &graph,
                         const std::shared_ptr<ompl::base::OptimizationObjective> &objective,
                         const g3tstar::Edge &edge);

                ~Grafting() = default;

                /** \brief Returns the best neighbor edge pairs sorted by cost */
                std::pair<g3tstar::Edge, g3tstar::Edge> getBestEdgePair() const;

                /** \brief Returns whether the optimal edge pair can be found through grafting */
                bool foundValidNeighborEdgePair();


            private:
                /** \brief Rely on G3Tstar for search */
                G3Tstar &g3tstar_; 

                /** \brief Current RGG map */
                std::shared_ptr<g3tstar::RandomGeometricGraph> graph_;

                /** \brief Const reference to the optimization objective this GuILD is supposed to help optimize. */
                std::shared_ptr<ompl::base::OptimizationObjective> objective_;

                /** \brief Edges where grafting is applied */
                g3tstar::Edge edge_;

                /** \brief The common neighbors of edge source and target */
                std::vector<std::weak_ptr<State>> commonNeighbors_;

                /** \brief The common neighbor edge pairs sorted by cost */
                std::vector<std::pair<g3tstar::Edge, g3tstar::Edge>> sortedNeighborEdgePairs_;

                /** \brief The best neighbor edge pairs sorted by cost */
                std::pair<g3tstar::Edge, g3tstar::Edge> bestEdgePair_;
                
                /** \brief Returns the common neighbors of edge source and target */
                void getCommonNeighbors(const g3tstar::Edge &edge);

                /** \brief Return common neighbor edge pairs sorted by cost */
                void getSortedNeighborEdgePairs();

            };

        }  // namespace g3tstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_TESTSTAR_GRAFTING_