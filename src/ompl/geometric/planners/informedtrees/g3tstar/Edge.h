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

// Authors: Marlin Strub, Liding Zhang, Xu Liang

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_TESTSTAR_EDGE_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_TESTSTAR_EDGE_

#include <array>
#include <limits>
#include <memory>

#include "ompl/base/Cost.h"

namespace ompl
{
    namespace geometric
    {
        namespace g3tstar
        {
            // Forward declaration of state to break include cycle.
            class State;

            /** \brief A struct for basic edge data. */
            struct Edge
            {
                /** \brief The default constructor. This is necessary because OMPL's heap requires elements to be
                 * default constructible. */
                Edge() = default;

                /** \brief Construct the edge by providing source and target states. */
                Edge(const std::shared_ptr<State> &source, const std::shared_ptr<State> &target);

                /** \brief Destruct the edge. */
                ~Edge() = default;

                /** \brief The parent state of this edge. */
                std::shared_ptr<State> source;

                /** \brief The child state of this edge. */
                std::shared_ptr<State> target;
            };

        }  // namespace g3tstar

    }  // namespace geometric

}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_TESTSTAR_EDGE_
