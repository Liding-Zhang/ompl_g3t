/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, University of Toronto
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
*   * Neither the name of the University of Toronto nor the names of its
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

/* Authors: Jonathan Gammell */

#include "ompl/base/samplers/informed/OBDRejectionInfSampler.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/tools/config/MagicConstants.h"

namespace ompl
{
    namespace base
    {
        // The default rejection-sampling class:
        OBDRejectionInfSampler::OBDRejectionInfSampler(const ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls)
          : OBDSampler(probDefn, maxNumberCalls)
          , stddev_(probDefn->getSpaceInformation()->getMaximumExtent() * magic::STD_DEV_AS_SPACE_EXTENT_FRACTION)
        {
            // Create the basic sampler
            baseSampler_ = OBDSampler::space_->allocDefaultStateSampler();

            // Warn if a cost-to-go heuristic is not defined
            if (!OBDSampler::opt_->hasCostToGoHeuristic())
            {
                OMPL_WARN("RejectionInfSampler: The optimization objective does not have a cost-to-go heuristic "
                          "defined. Informed sampling will likely have little to no effect.");
            }
            // No else
        }

        bool OBDRejectionInfSampler::sampleUniform(State *statePtr, const Cost &maxCost,bool &isBridgeState)
        {
            // Variable
            // The persistent iteration counter:
            unsigned int iter = 0u;
            bool isbridgestate;

            // Call the sampleUniform helper function with my iteration counter:
            return sampleUniform(statePtr, maxCost, &iter , isBridgeState);
        }

        bool OBDRejectionInfSampler::sampleUniform(State *statePtr, const Cost &minCost, const Cost &maxCost,bool &isBridgeState)
        {
            // Variable
            // Whether we were successful in creating an informed sample. Initially not:
            bool foundSample = false;
            // Spend numIters_ iterations trying to find an informed sample:
            for (unsigned int i = 0u; i < OBDSampler::numIters_ && !foundSample; ++i)
            {
                // bool isbridgestate;
                // Call the helper function for the larger cost. It will move our iteration counter:
                foundSample = sampleUniform(statePtr, maxCost, &i, isBridgeState);

                // Did we find a sample?
                if (foundSample)
                {
                    // We did, but it only satisfied the upper bound. Check that it meets the lower bound.

                    // Variables
                    // The cost of the sample we found:
                    Cost sampledCost = OBDSampler::heuristicSolnCost(statePtr);

                    // Check if the sample's cost is greater than or equal to the lower bound
                    foundSample = OBDSampler::opt_->isCostEquivalentTo(minCost, sampledCost) ||
                                  OBDSampler::opt_->isCostBetterThan(minCost, sampledCost);
                }
                // No else, no sample was found.
            }

            // One way or the other, we're done:
            return foundSample;
        }

        bool OBDRejectionInfSampler::hasInformedMeasure() const
        {
            return false;
        }

        double OBDRejectionInfSampler::getInformedMeasure(const Cost & /*currentCost*/) const
        {
            return OBDSampler::space_->getMeasure();
        }

        double OBDRejectionInfSampler::getInformedMeasure(const Cost & /*minCost*/, const Cost & /*maxCost*/) const
        {
            return OBDSampler::space_->getMeasure();
        }

        bool OBDRejectionInfSampler::sampleUniform(State *statePtr, const Cost &maxCost, unsigned int *iterPtr, bool &isBridgeState)
        {
            
            // Whether we were successful in creating an informed sample. Initially not:
            bool validSample = false;
            bool foundSample = false;
            bool foundvalidSample = false;
            State *temp = OBDSampler::getProblemDefn()->getSpaceInformation()->allocState();
            State *midpoint = OBDSampler::getProblemDefn()->getSpaceInformation()->allocState();
            // temporary state for the endpoint and midpoint
            // Make numIters_ attempts at finding a sample whose heuristic estimate of solution cost through the sample
            // is better than maxCost by sampling the entire planning domain
            std::cout << "Rejection !!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

            for (/* Provided iteration counter */; *iterPtr < OBDSampler::numIters_ && !foundvalidSample;
                 ++(*iterPtr))
            {  
                baseSampler_->sampleUniform(statePtr);
                bool v1 = false, v2 = false;
                //randomly sample a state
                v1 = OBDSampler::getProblemDefn()->getSpaceInformation()->isValid(statePtr);
                
                
                if (v1)
                {
                    //If it is valid, output it
                    validSample = true;
                } 
                else
                {
                    //if it is not valid, sample a temporary state that is standard deviation distance away from the current state
                    //if it is valid, output it
                    baseSampler_->sampleGaussian(temp, statePtr, stddev_);
                    v2 = OBDSampler::getProblemDefn()->getSpaceInformation()->isValid(temp);
                    if (v2)
                    {
                        OBDSampler::getProblemDefn()->getSpaceInformation()->copyState(statePtr, temp);
                        validSample = true;
                    }
                    //if not, test the midpoint between these 2 invalid states. If the midpoint is valid, ouput it.
                    else
                    {
                        for (float i = 0.1; i <= 0.9; i = i+0.1)
                        {
                            OBDSampler::getProblemDefn()->getSpaceInformation()->getStateSpace()->interpolate(temp, statePtr, i, midpoint);
                        if (OBDSampler::getProblemDefn()->getSpaceInformation()->isValid(midpoint))
                           {
                               OBDSampler::getProblemDefn()->getSpaceInformation()->copyState(statePtr, midpoint);
                               validSample = true;
                               isBridgeState= true;
                               break;
                           }
                        }
                    }
                }

                                
                // Check if it's found, i.e., if f(state) <= maxCost
                foundSample =
                    OBDSampler::opt_->isCostBetterThan(OBDSampler::heuristicSolnCost(statePtr), maxCost);
                foundvalidSample = foundSample && validSample;
            }
            OBDSampler::getProblemDefn()->getSpaceInformation()->freeState(temp);
            OBDSampler::getProblemDefn()->getSpaceInformation()->freeState(midpoint);
            // All done, one way or the other:
            return foundvalidSample;
        }
    };  // base
};      // ompl
