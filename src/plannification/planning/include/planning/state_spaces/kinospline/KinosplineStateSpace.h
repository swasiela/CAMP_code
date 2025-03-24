/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2024, LAAS-CNRS
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

/* Author: Simon WASIELA */

#ifndef KINOSPLINE_STATE_SPACE_
#define KINOSPLINE_STATE_SPACE_

// Robots
#include "robots/Quadrotor.h"
#include "robots/Unicycle.h"

// Kinosplines
#include "kinosplines/KinoDynamicTime.h"
#include "kinosplines/kdtp_init.h"

// State space
#include "planning/state_spaces/RobustStateSpace.h"

// Utils
#include "utils/transform.h"

// Define a new type of state space: need to be larger than STATE_SPACE_TYPE_COUNT
#define KINO_STATE_SPACE (ompl::base::STATE_SPACE_TYPE_COUNT + 2)

namespace ompl
{
  namespace base
  {

    OMPL_CLASS_FORWARD(KinosplineStateSpace);

    /** \brief A state space representing the desired kino state (i.e controller inputs) X = {q, qdot, qddot}, with q dynamically allocated according to the robot initial state */
    class KinosplineStateSpace : public RobustStateSpace
    {
      public:

        /** \brief A desired kino state (i.e controller inputs) : X = {q, qdot, qddot} with dynamic size allocation according to the initial state specified for the robot*/
        class StateType : public RobustStateSpace::StateType
        {
          public:
            StateType(unsigned int dim) : RobustStateSpace::StateType(), dim_(dim){}
            
            /** \brief Set q*/
            void setQValues(std::vector<double> values){
              for(int i = 0; i< values.size(); i++)
                as<RealVectorStateSpace::StateType>(0)->values[i] = values[i];
            }

            /** \brief Return q*/
            std::vector<double> getQValues()const{
              std::vector<double> val;
              for(int i = 0; i<dim_; i++)
                val.push_back(as<RealVectorStateSpace::StateType>(0)->values[i]);
              return val;
            }

            /** \brief Set qdot*/
            void setQdotValues(std::vector<double> values){
              for(int i = 0; i< values.size(); i++)
                as<RealVectorStateSpace::StateType>(1)->values[i] = values[i];
            }

            /** \brief Return qdot*/
            std::vector<double> getQdotValues()const{
              std::vector<double> val;
              for(int i = 0; i<dim_; i++)
                val.push_back(as<RealVectorStateSpace::StateType>(1)->values[i]);
              return val;
            }

            /** \brief Set qddot*/
            void setQddotValues(std::vector<double> values){
              for(int i = 0; i< values.size(); i++)
                as<RealVectorStateSpace::StateType>(2)->values[i] = values[i];
            }

            /** \brief Return qddot*/
            std::vector<double> getQddotValues()const{
              std::vector<double> val;
              for(int i = 0; i<dim_; i++)
                val.push_back(as<RealVectorStateSpace::StateType>(2)->values[i]);
              return val;
            }
            
          private:
            // State dimension
            unsigned int dim_;
        };

        KinosplineStateSpace(unsigned int dim, std::string opt_tag, ompl::RobotPtr robot)
              : RobustStateSpace(), qBoundsSet_(false), qdotBoundsSet_(false), qddotBoundsSet_(false), opt_tag_(opt_tag), robot_(robot)
        {
          type_ = KINO_STATE_SPACE;
          // For q
          addSubspace(StateSpacePtr(new RealVectorStateSpace(robot_->getDesiredDim())), 1.0); 
          // For qdot
          addSubspace(StateSpacePtr(new RealVectorStateSpace(robot_->getDesiredDim())), 1.0);  
          // For qddot
          addSubspace(StateSpacePtr(new RealVectorStateSpace(robot_->getDesiredDim())), 1.0);  
          lock();

          // Bounds of the sampled kino states
          RealVectorBounds q_bounds_(robot_->getDesiredDim());
          RealVectorBounds qdot_bounds_(robot_->getDesiredDim());
          RealVectorBounds qddot_bounds_(robot_->getDesiredDim());

          for(int i=0; i<robot_->getDesiredDim(); i++)
          {
            q_bounds_.high[i] = robot_->getQUpperBounds()[i];
            q_bounds_.low[i] = robot_->getQLowerBounds()[i];
            qdot_bounds_.high[i] = robot_->getKinodynamicLimits()[i*5];
            qdot_bounds_.low[i] = -robot_->getKinodynamicLimits()[i*5];
            qddot_bounds_.high[i] = robot_->getKinodynamicLimits()[i*5+1];
            qddot_bounds_.low[i] = -robot_->getKinodynamicLimits()[i*5+1];
            maximumJerk_.push_back(robot_->getKinodynamicLimits()[i*5+2]);
          }
          setQBounds(q_bounds_);
          setQdotBounds(qdot_bounds_);
          setQddotBounds(qddot_bounds_);
        }

        virtual ~KinosplineStateSpace()
        {}

        /** \brief The distance function associated with the kinospline state space is not a pure metric, it's a quasi-metric */
        bool isMetricSpace() const override
        {
            return false;
        }

        /** \brief Check if the distance function on this state space is symmetric, i.e. distance(s1,s2) =
         * distance(s2,s1). Default implementation returns true.*/
        bool hasSymmetricDistance() const override
        {
          return false;
        }

        /** \brief Check if the interpolation function on this state space is symmetric, i.e. interpolate(from, to,
         * t, state) = interpolate(to, from, 1-t, state). Default implementation returns true.*/
        bool hasSymmetricInterpolate() const override
        {
          return false;
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

        // >>>> qdot
        /** \copydoc RealVectorStateSpace::setVelocityBounds() */
        void setQdotBounds(const RealVectorBounds &qdot_bounds){
            as<RealVectorStateSpace>(1)->setBounds(qdot_bounds);
            qdotBoundsSet_ = true;
        }

        /** \copydoc RealVectorStateSpace::getVelocityBounds() */
        const RealVectorBounds& getQdotBounds() const{
          if(qdotBoundsSet_)
            return as<RealVectorStateSpace>(1)->getBounds();
          else{
            ROS_ERROR_STREAM("Translation velocity bounds not set for KinosplineStateSpace");
          }
        }

        // >>>> qddot
        /** \copydoc RealVectorStateSpace::setAccelerationBounds() */
        void setQddotBounds(const RealVectorBounds &qddot_bounds){
            as<RealVectorStateSpace>(2)->setBounds(qddot_bounds);
            qddotBoundsSet_ = true;
        }

        /** \copydoc RealVectorStateSpace::getAccelerationBounds() */
        const RealVectorBounds& getQddotBounds() const{
          if(qddotBoundsSet_)
            return as<RealVectorStateSpace>(2)->getBounds();
          else{
            ROS_ERROR_STREAM("Acceleration bounds not set for KinosplineStateSpace");
          }
        }

        /// \brief Get access to the robot pointer
        ompl::RobotPtr getRobot() const
        {
          return robot_;
        }

        // State management functions
        State* allocState() const override;
        void freeState(State *state) const override;
        void copyState(State *destination, const State *source) const override;

        // Interpolation functions
        void interpolate(const State *from, const State *to, const double t, State *state) const override;
        void interpolate(const State *from, const State *to, const double dt, std::vector<State*>& states) const override;
        void interpolateLearning(const State *from, const State *to, const double dt, std::vector<State*>& states, std::vector<std::vector<float>> &vector_to_tensor) const override;

        // Distance functions
        double distance(const State *state1, const State *state2) const override;
        virtual double distancePos(const State *state1, const State *state2) const;
        virtual double distanceVel(const State *state1, const State *state2) const;
        virtual double distanceAcc(const State *state1, const State *state2) const;
        virtual double distanceState(const State *state1, const State *state2) const;
        virtual double get_interpolate_time(const State *from, const State *to) const;

        // Compute the kinospline quasi metric (i.e. a metric which is directed)
        double callKinoMetric(const State *from, const State *to) const
        {
          //Get q, qdot and qddot
          std::vector<double> from_q, from_qdot, from_qddot, to_q, to_qdot, to_qddot;

          from_q = from->as<StateType>()->getQValues();
          to_q = to->as<StateType>()->getQValues();
          from_qdot = from->as<StateType>()->getQdotValues();
          to_qdot = to->as<StateType>()->getQdotValues();
          from_qddot = from->as<StateType>()->getQddotValues();
          to_qddot = to->as<StateType>()->getQddotValues();

          std::vector<std::vector<double>> q0, qT;
          for(int i = 0; i<from_qdot.size(); i++)
          {
            std::vector<double> q0_component, qT_component;
            q0_component.push_back(from_q.at(i));
            q0_component.push_back(from_qdot.at(i));
            q0_component.push_back(from_qddot.at(i));

            qT_component.push_back(to_q.at(i));
            qT_component.push_back(to_qdot.at(i));
            qT_component.push_back(to_qddot.at(i));

            q0.push_back(q0_component);
            qT.push_back(qT_component);
          }

          std::vector<double> maximumJerk(maximumJerk_);

          return kinoMetric(q0, qT, maximumJerk);
        }

        /** \brief Get the robot desired state according to the end effector desired state */
        virtual void getPlatformFromEEF(std::vector<double>& q, std::vector<double>& acc) const = 0;

      private:

        /// \brief Pointer to the Robot object
        ompl::RobotPtr robot_;
      
        // Bounds
        bool qBoundsSet_;
        bool qdotBoundsSet_;
        bool qddotBoundsSet_;

        //tag for the optimization objective
        const std::string opt_tag_;

        // Maximum jerk allowed
        std::vector<double> maximumJerk_;
    };
  }
}
#endif // PHYSIC_POINT_STATE_SPACE_H
