/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_MOTION_VALIDATOR_
#define OMPL_BASE_MOTION_VALIDATOR_

#include "torch/torch.h"

#include "ompl/util/Exception.h"

#include "ompl/base/State.h"
#include "ompl/util/ClassForward.h"
#include <utility>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(SpaceInformation);
        /// @endcond

        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::MotionValidator */
        OMPL_CLASS_FORWARD(MotionValidator);
        /// @endcond

        /** \class ompl::base::MotionValidatorPtr
            \brief A shared pointer wrapper for ompl::base::MotionValidator */

        /** \brief Abstract definition for a class checking the
            validity of motions -- path segments between states. This
            is often called a local planner. The implementation of
            this class must be thread safe. */
        class MotionValidator
        {
        public:
            /** \brief Constructor */
            MotionValidator(SpaceInformation *si) : si_(si), valid_(0), invalid_(0)
            {
            }

            /** \brief Constructor */
            MotionValidator(const SpaceInformationPtr &si) : si_(si.get()), valid_(0), invalid_(0)
            {
            }

            virtual ~MotionValidator() = default;

            /** \brief Cast this instance to a desired type. */
            template <class T>
            T *as()
            {
                /** \brief Make sure the type we are casting to is indeed a state space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, MotionValidator *>));

                return static_cast<T *>(this);
            }

            /** \brief Cast this instance to a desired type. */
            template <class T>
            const T *as() const
            {
                /** \brief Make sure the type we are casting to is indeed a state space */
                BOOST_CONCEPT_ASSERT((boost::Convertible<T *, MotionValidator *>));

                return static_cast<const T *>(this);
            }

            /** \brief Check if the path between two states (from \e s1 to \e s2) is valid. This function assumes \e s1
               is valid.

                \note This function updates the number of valid and invalid segments. */
            virtual bool checkMotion(const State *s1, const State *s2) const = 0;

            /** \brief Check if the path between two states is valid. Also compute the last state that was
                valid and the time of that state. The time is used to parametrize the motion from \e s1 to \e s2, \e s1
               being at t =
                0 and \e s2 being at t = 1. This function assumes \e s1 is valid.
                \param s1 start state of the motion to be checked (assumed to be valid)
                \param s2 final state of the motion to be checked
                \param lastValid first: storage for the last valid state (may be nullptr, if the user does not care
               about the exact state); this need not be different from \e s1 or \e s2. second: the time (between 0 and
               1) of the last valid state, on the motion from \e s1 to \e s2. If the function returns false, \e
               lastValid.first must be set to a valid state, even if that implies copying \e s1 to \e lastValid.first
               (in case \e lastValid.second = 0). If the function returns true, \e lastValid.first and \e
               lastValid.second should \b not be modified.

                \note This function updates the number of valid and invalid segments. */
            virtual bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const = 0;

            /**
             * Interpolate and check the motion between s1 and s2 in the 'traditional' way.
             *
             * @param s1 The first state of the desired motion
             * @param s2 The final state of the desired motion
             * @param states The list of desired states to filled (i.e. the desired trajectory that will be given to the controller).
             * @return The validity of the motion
            */
            virtual bool checkMotion(const State *s1, const State *s2, std::vector<State*>& states) const
            {
                (void)s1;
                (void)s2;
                (void)states;
                throw ompl::Exception("checkMotion", "not implemented");
            }
            
            /**
             * Interpolate and check the motion between s1 and s2 using uncertainty tubes predicted by the GRU neural network.
             *
             * @param s1 The first state of the desired motion
             * @param s2 The final state of the desired motion
             * @param states The initial robot state used as initial condition to simulate the robot dynamic
             * @param traj_tensor The initial hidden state used as initial condition for the prediction
             * @return The validity of the motion.
            */
            virtual bool checkMotionLearning(const State *s1, const State *s2, std::vector<ompl::base::State*>& states, torch::Tensor &traj_tensor) const
            {
                (void)s1;
                (void)s2;
                (void)states;
                (void)traj_tensor;
                throw ompl::Exception("checkMotionLearning", "not implemented");
            }

            /**
             * Interpolate and check the motion between s1 and s2 using uncertainty tubes computed by solving ODEs.
             *
             * @param s1 The first state of the desired motion
             * @param s2 The final state of the desired motion
             * @param states The list of desired states to filled (i.e. the desired trajectory that will be given to the controller).
             * @return The validity of the motion
            */
            virtual bool checkMotionODE(const State *s1, const State *s2, std::vector<ompl::base::State*>& states) const
            {
                (void)s1;
                (void)s2;
                (void)states;
                throw ompl::Exception("checkMotionODE", "not implemented");
            }

            /**
             * Propagate each RandUp particles between s1 and s2 using a c++ implementation based on the python implementation of https://github.com/StanfordASL/randUP_RRT.
             * The collision checking is done in the same way as the one found in the file "plan_quadrotor.py", "is_safe" function line 86.
             *
             * @param s1 The first state of the desired motion
             * @param s2 The final state of the desired motion
             * @param init_dynamic_states The initial robot state of each particle.
             * @param rand_up_params The uncertain model parameter values of each particle.
             * @param propagated_states The final robot state of each particle after propagation to be reused as futur initial state.
             * @param padding The padding value used for the epsilon-randup
             * @return The validity of the motion
            */
            virtual bool checkMotionRandUp(const State *s1, const State *s2, const std::vector<std::vector<double>> &init_dynamic_states, 
                                        const std::vector<std::vector<double>> &rand_up_params, std::vector<std::vector<double>> &propagated_states, double padding) const
            {
                (void)s1;
                (void)s2;
                (void)init_dynamic_states;
                (void)rand_up_params;
                (void)propagated_states;
                (void)padding;
                throw ompl::Exception("checkMotionRandUp", "not implemented");
            }

            //############################################################################################################
            // Re-check a precomputed motion

            /**
             * Re-check a precomputed motion in the 'traditional' way
             *
             * @param states The precomputed trajectory
             * @return The validity of the motion
            */
            virtual bool reCheckMotion(const std::vector<State*>& states) const
            {
                (void)states;
                throw ompl::Exception("reCheckMotion", "not implemented");
            }

            /**
             * Re-check a precomputed motion using uncertainty tubes prediction
             *
             * @param states The precomputed trajectory
             * @param traj_tensor The precomputed trajectory in torch::Tensor format
             * @param init_nominal_state The initial robot state used as initial condition to solve the dynamic ODEs
             * @param new_final_nom_state The updated final nominal state values (i.e qF) to be reused as futur initial conditions
             * @param new_final_hidden_state The updated final hidden state values (i.e hF) to be reused as futur initial conditions
             * @return The validity of the motion
            */
            virtual bool reCheckMotionLearning(const std::vector<ompl::base::State*>& states, const torch::Tensor& traj_tensor) const
            
            {
                (void)states;
                (void)traj_tensor;
                throw ompl::Exception("reCheckMotionLearning", "not implemented");
            }

            /**
             * Re-check a precomputed motion using ODE tubes computation
             *
             * @param states The precomputed trajectory
             * @return The validity of the motion
            */
            virtual bool reCheckMotionODE(const std::vector<ompl::base::State*>& states) const
            {
                (void)states;
                throw ompl::Exception("reCheckMotionODE", "not implemented");
            }

            //############################################################################################################
            // Getter and setter to the time step used for discrete motion checking

            /**
             * Get the current time step used by the collision checker to interpolate and check the motion
             *
             * @return The current interpolation and checking time step
            */
            virtual double getDt() const
            {
                throw ompl::Exception("getDt", "not implemented");
            }
            
            /**
             * Set the new time step for interpolation and collision checking
             *
             * @param new_dt The new interpolation and checking time step
            */
            virtual void setDt(const double new_dt)
            {
                (void)new_dt;
                throw ompl::Exception("setDt", "not implemented");
            }

            /** \brief Get the number of segments that tested as valid */
            unsigned int getValidMotionCount() const
            {
                return valid_;
            }

            /** \brief Get the number of segments that tested as invalid */
            unsigned int getInvalidMotionCount() const
            {
                return invalid_;
            }

            /** \brief Get the total number of segments tested, regardless of result */
            unsigned int getCheckedMotionCount() const
            {
                return valid_ + invalid_;
            }

            /** \brief Get the fraction of segments that tested as valid */
            double getValidMotionFraction() const
            {
                return valid_ == 0 ? 0.0 : (double)valid_ / (double)(invalid_ + valid_);
            }

            /** \brief Reset the counters for valid and invalid segments */
            void resetMotionCounter()
            {
                valid_ = invalid_ = 0;
            }

        protected:
            /** \brief The instance of space information this state validity checker operates on */
            SpaceInformation *si_;

            /** \brief Number of valid segments */
            mutable unsigned int valid_;

            /** \brief Number of invalid segments */
            mutable unsigned int invalid_;
        };
    }
}

#endif
