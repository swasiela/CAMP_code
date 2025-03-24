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

/* Author: Mark Moll */

#ifndef DUBINS_UNICYCLE_SPACE_
#define DUBINS_UNICYCLE_SPACE_

#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/base/MotionValidator.h"
#include <boost/math/constants/constants.hpp>

// Robots
#include "robots/Quadrotor.h"
#include "robots/Unicycle.h"

// State space
#include "planning/state_spaces/RobustStateSpace.h"

// Utils
#include "utils/transform.h"

// Define a new type of state space: need to be larger than STATE_SPACE_TYPE_COUNT
#define DUBINS_STATE_SPACE (ompl::base::STATE_SPACE_TYPE_COUNT + 3)

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(DubinsUnicycleSpace);

        /** \brief An SE(2) state space where distance is measured by the
            length of Dubins curves.

            Note that this Dubins distance is \b not a proper distance metric,
            so nearest neighbor methods that rely on distance() being a metric
            (such as ompl::NearestNeighborsGNAT) will not always return the
            true nearest neighbors or get stuck in an infinite loop.

            The notation and solutions in the code are taken from:<br>
            A.M. Shkel and V. Lumelsky, “Classification of the Dubins set,”
            Robotics and Autonomous Systems, 34(4):179-202, 2001.
            DOI: <a href="http://dx.doi.org/10.1016/S0921-8890(00)00127-5">10.1016/S0921-8890(00)00127-5</a>
            */
        class DubinsUnicycleSpace : public RobustStateSpace
        {
            public:

                /** \brief A desired dubins state (i.e controller inputs) : X = {x y theta}*/
                class StateType : public RobustStateSpace::StateType
                {
                    public:
                        StateType() : RobustStateSpace::StateType(){}
                        
                        /** \brief Get the X component of the state */
                        double getX() const
                        {
                            return as<RealVectorStateSpace::StateType>(0)->values[0];
                        }

                        /** \brief Get the Y component of the state */
                        double getY() const
                        {
                            return as<RealVectorStateSpace::StateType>(0)->values[1];
                        }

                        /** \brief Get the yaw component of the state. This is
                            the rotation in plane, with respect to the Z
                            axis. */
                        double getYaw() const
                        {
                            return as<SO2StateSpace::StateType>(1)->value;
                        }

                        /** \brief Set the X component of the state */
                        void setX(double x)
                        {
                            as<RealVectorStateSpace::StateType>(0)->values[0] = x;
                        }

                        /** \brief Set the Y component of the state */
                        void setY(double y)
                        {
                            as<RealVectorStateSpace::StateType>(0)->values[1] = y;
                        }

                        /** \brief Set the X and Y components of the state */
                        void setXY(double x, double y)
                        {
                            setX(x);
                            setY(y);
                        }

                        /** \brief Set the yaw component of the state. This is
                            the rotation in plane, with respect to the Z
                            axis. */
                        void setYaw(double yaw)
                        {
                            as<SO2StateSpace::StateType>(1)->value = yaw;
                        }

                        /** \brief Save x component one step before */
                        void setPrevX(double x) { prevX = x; }

                        /** \brief Get x component one step before */
                        double getPrevX() const { return prevX; }

                        /** \brief Save y component one step before */
                        void setPrevY(double x) { prevY = x; }

                        /** \brief Get y component one step before */
                        double getPrevY() const { return prevY; }

                        /** \brief Save dt component one step before */
                        void setPrevTime(double x) { last_dt = x; }

                        /** \brief Get dt component one step before */
                        double getPrevTime() const { return last_dt; }

                    private:

                        /** \brief Previous state used for velocities and accelerations computation */
                        double prevX;    // X from one step before
                        double prevY;    // Y from one step before
                        double last_dt;  // Last dt which is not fixed
                };

                /** \brief The Dubins path segment type */
                enum DubinsPathSegmentType
                {
                    DUBINS_LEFT = 0,
                    DUBINS_STRAIGHT = 1,
                    DUBINS_RIGHT = 2
                };

                /** \brief Dubins path types */
                static const DubinsPathSegmentType dubinsPathType[6][3];
                /** \brief Complete description of a Dubins path */
                class DubinsPath
                {
                    public:
                        DubinsPath(const DubinsPathSegmentType *type = dubinsPathType[0], double t = 0.,
                                double p = std::numeric_limits<double>::max(), double q = 0.)
                        : type_(type)
                        {
                            length_[0] = t;
                            length_[1] = p;
                            length_[2] = q;
                            assert(t >= 0.);
                            assert(p >= 0.);
                            assert(q >= 0.);
                        }
                        double length() const
                        {
                            return length_[0] + length_[1] + length_[2];
                        }

                        /** Path segment types */
                        const DubinsPathSegmentType *type_;
                        /** Path segment lengths */
                        double length_[3];
                        /** Whether the path should be followed "in reverse" */
                        bool reverse_{false};
                };

                DubinsUnicycleSpace(double turningRadius = 1.0, bool isSymmetric = false, std::string opt_tag = "NONE", ompl::RobotPtr robot = nullptr)
                : RobustStateSpace(), qBoundsSet_(false), rho_(turningRadius), isSymmetric_(isSymmetric), opt_tag_(opt_tag), robot_(robot)
                {
                    type_ = DUBINS_STATE_SPACE;
                    addSubspace(std::make_shared<RealVectorStateSpace>(2), 1.0);
                    addSubspace(std::make_shared<SO2StateSpace>(), 0.5);
                    lock();

                    // Bounds of the sampled kino states
                    RealVectorBounds q_bounds_(robot_->getDesiredDim());

                    for(int i=0; i<robot_->getDesiredDim(); i++)
                    {
                        q_bounds_.high[i] = robot_->getQUpperBounds()[i];
                        q_bounds_.low[i] = robot_->getQLowerBounds()[i];
                    }
                    setQBounds(q_bounds_);
                }

                // >>>> q
                /** \copydoc RealVectorStateSpace::setPositionBounds() */
                void setQBounds(const RealVectorBounds &q_bounds){
                    as<RealVectorStateSpace>(0)->setBounds(q_bounds);
                    qBoundsSet_ = true;
                }

                /** \copydoc RealVectorStateSpace::getQBounds() */
                const RealVectorBounds& getQBounds() const{
                    if(qBoundsSet_)
                        return as<RealVectorStateSpace>(0)->getBounds();
                    else{
                        ROS_ERROR_STREAM("Position bounds not set for KinosplineStateSpace");
                    }
                }

                bool isMetricSpace() const override
                {
                    return false;
                }

                double distance(const State *state1, const State *state2) const override;

                // Velocity and acceleration
                // Function to compute velocities given positions and time step
                std::vector<double> computeVelocities(const std::vector<double>& positions, double dt) const
                {
                    std::vector<double> velocities(positions.size());
                    velocities[0] = 0.0; // Assume initial velocity is 0

                    for (size_t i = 1; i < positions.size(); ++i) {
                        velocities[i] = (positions[i] - positions[i - 1]) / dt;
                    }

                    return velocities;
                }

                // Function to compute accelerations given velocities and time step, with saturation and backpropagation
                std::vector<double> computeAccelerations(std::vector<double>& velocities, double dt, double max_acceleration) const
                {
                    std::vector<double> accelerations(velocities.size());
                    accelerations[0] = 0.0; // Assume initial acceleration is 0

                    for (size_t i = 1; i < velocities.size(); ++i) {
                        double raw_acceleration = (velocities[i] - velocities[i - 1]) / dt;

                        // Saturate the acceleration using std::clamp
                        double saturated_acceleration = std::clamp(raw_acceleration, -max_acceleration, max_acceleration);

                        // Adjust the velocity if the acceleration was saturated
                        if (saturated_acceleration != raw_acceleration) {
                            velocities[i] = velocities[i - 1] + saturated_acceleration * dt;
                        }

                        // Store the saturated acceleration
                        accelerations[i] = saturated_acceleration;
                    }

                    return accelerations;
                }

                // Function to update positions based on adjusted velocities
                void updatePositions(std::vector<double>& positions, const std::vector<double>& velocities, double dt) const
                {
                    for (size_t i = 1; i < positions.size(); ++i) {
                        positions[i] = positions[i - 1] + velocities[i] * dt;
                    }
                }

                // INTERPOLATION
                void interpolate(const State *from, const State *to, double t, State *state) const override;
                void interpolate(const State *from, const State *to, const double dt, std::vector<State*>& states) const override;
                void interpolateLearning(const State *from, const State *to, const double dt, std::vector<State*>& states, std::vector<std::vector<float>> &vector_to_tensor) const override;
                virtual void interpolate(const State *from, const State *to, double t, bool &firstTime,
                                        DubinsPath &path, State *state) const;
                
                // PROJECTION AND SIMULATION
                void projectStates(const std::vector<ompl::base::State*> &des_states, std::vector<std::vector<double>> &collision_states) const override;
                bool simulateStates(const std::vector<ompl::base::State*> &des_states, const double dt, std::vector<std::vector<double>> &collision_states) const override;
                bool simulateStatesAndTubes(const std::vector<ompl::base::State*> &des_states, double dt, std::vector<std::vector<double>> &collision_states,
                                                std::vector<std::vector<double>> &control_inputs, std::vector<std::vector<double>> &radii) const override;

                bool hasSymmetricDistance() const override
                {
                    return isSymmetric_;
                }

                bool hasSymmetricInterpolate() const override
                {
                    return isSymmetric_;
                }

                unsigned int validSegmentCount(const State *state1, const State *state2) const override;

                void sanityChecks() const override
                {
                    double zero = std::numeric_limits<double>::epsilon();
                    double eps = std::numeric_limits<float>::epsilon();
                    int flags = ~(STATESPACE_INTERPOLATION | STATESPACE_TRIANGLE_INEQUALITY | STATESPACE_DISTANCE_BOUND);
                    if (!isSymmetric_)
                        flags &= ~STATESPACE_DISTANCE_SYMMETRIC;
                    StateSpace::sanityChecks(zero, eps, flags);
                }

                /** \brief Return a shortest Dubins path from SE(2) state state1 to SE(2) state state2 */
                DubinsPath dubins(const State *state1, const State *state2) const;

                /// \brief Get access to the robot pointer
                ompl::RobotPtr getRobot() const
                {
                    return robot_;
                }

                // State management functions
                State* allocState() const override;
                void freeState(State *state) const override;
                void copyState(State *destination, const State *source) const override;

            protected:
                virtual void interpolate(const State *from, const DubinsPath &path, double t, State *state) const;

                // Bounds
                bool qBoundsSet_;

                /** \brief Turning radius */
                double rho_;

                /** \brief Whether the distance is "symmetrized"

                    If true the distance from state s1 to state s2 is the same as the
                    distance from s2 to s1. This is done by taking the \b minimum
                    length of the Dubins curves that connect s1 to s2 and s2 to s1. If
                    isSymmetric_ is true, then the distance no longer satisfies the
                    triangle inequality. */
                bool isSymmetric_;

                /// \brief Pointer to the Robot object
                ompl::RobotPtr robot_;

                //tag for the optimization objective
                const std::string opt_tag_;
        };
    }
}

#endif