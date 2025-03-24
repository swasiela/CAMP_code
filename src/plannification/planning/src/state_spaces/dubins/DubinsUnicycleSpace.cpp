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

#include "planning/state_spaces/dubins/DubinsUnicycleSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <queue>
#include <boost/math/constants/constants.hpp>

double EPSILON = 1e-5; // To avoid bad approximation

using namespace ompl::base;

namespace
{
    const double twopi = 2. * boost::math::constants::pi<double>();
    const double DUBINS_EPS = 1e-6;
    const double DUBINS_ZERO = -1e-7;

    enum DubinsClass
    {
        A11 = 0,
        A12 = 1,
        A13 = 2,
        A14 = 3,
        A21 = 4,
        A22 = 5,
        A23 = 6,
        A24 = 7,
        A31 = 8,
        A32 = 9,
        A33 = 10,
        A34 = 11,
        A41 = 12,
        A42 = 13,
        A43 = 14,
        A44 = 15
    };

    inline double mod2pi(double x)
    {
        if (x < 0 && x > DUBINS_ZERO)
            return 0;
        double xm = x - twopi * floor(x / twopi);
        if (twopi - xm < .5 * DUBINS_EPS)
            xm = 0.;
        return xm;
    }

    inline double t_lsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
        const double p = sqrtf(std::max(tmp, 0.0));
        const double theta = atan2f(-ca - cb, d + sa + sb) - atan2f(-2.0, p);
        return mod2pi(-alpha + theta);  // t
    }

    inline double p_lsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
        return sqrtf(std::max(tmp, 0.0));  // p
    }

    inline double q_lsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = -2.0 + d * d + 2.0 * (ca * cb + sa * sb + d * (sa + sb));
        const double p = sqrtf(std::max(tmp, 0.0));
        const double theta = atan2f(-ca - cb, d + sa + sb) - atan2f(-2.0, p);
        return mod2pi(-beta + theta);  // q
    }

    inline double t_rsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
        const double p = sqrtf(std::max(tmp, 0.0));
        const double theta = atan2f(ca + cb, d - sa - sb) - atan2f(2.0, p);
        return mod2pi(alpha - theta);  // t
    }

    inline double p_rsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
        return sqrtf(std::max(tmp, 0.0));  // p
    }

    inline double q_rsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = d * d - 2.0 + 2.0 * (ca * cb + sa * sb - d * (sa + sb));
        const double p = sqrtf(std::max(tmp, 0.));
        const double theta = atan2f(ca + cb, d - sa - sb) - atan2f(2.0, p);
        return mod2pi(beta - theta);  // q
    }

    inline double t_rsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double theta = atan2f(ca - cb, d - sa + sb);
        return mod2pi(alpha - theta);  // t
    }

    inline double p_rsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sb - sa));
        return sqrtf(std::max(tmp, 0.0));  // p
    }

    inline double q_rsr(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double theta = atan2f(ca - cb, d - sa + sb);
        return mod2pi(-beta + theta);  // q
    }

    inline double t_lsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double theta = atan2f(cb - ca, d + sa - sb);
        return mod2pi(-alpha + theta);  // t
    }

    inline double p_lsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double tmp = 2.0 + d * d - 2.0 * (ca * cb + sa * sb - d * (sa - sb));
        return sqrtf(std::max(tmp, 0.0));  // p
    }

    inline double q_lsl(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        const double theta = atan2f(cb - ca, d + sa - sb);
        return mod2pi(beta - theta);  // q
    }

    inline double s_12(double d, double alpha, double beta)
    {
        return p_rsr(d, alpha, beta) - p_rsl(d, alpha, beta) - 2.0 * (q_rsl(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    inline double s_13(double d, double alpha, double beta)
    {  // t_rsr - pi
        return t_rsr(d, alpha, beta) - boost::math::constants::pi<double>();
    }

    inline double s_14_1(double d, double alpha, double beta)
    {
        return t_rsr(d, alpha, beta) - boost::math::constants::pi<double>();
    }

    inline double s_21(double d, double alpha, double beta)
    {
        return p_lsl(d, alpha, beta) - p_rsl(d, alpha, beta) - 2.0 * (t_rsl(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    inline double s_22_1(double d, double alpha, double beta)
    {
        return p_lsl(d, alpha, beta) - p_rsl(d, alpha, beta) - 2.0 * (t_rsl(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    inline double s_22_2(double d, double alpha, double beta)
    {
        return p_rsr(d, alpha, beta) - p_rsl(d, alpha, beta) - 2.0 * (q_rsl(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    inline double s_24(double d, double alpha, double beta)
    {
        return q_rsr(d, alpha, beta) - boost::math::constants::pi<double>();
    }

    inline double s_31(double d, double alpha, double beta)
    {
        return q_lsl(d, alpha, beta) - boost::math::constants::pi<double>();
    }

    inline double s_33_1(double d, double alpha, double beta)
    {
        return p_rsr(d, alpha, beta) - p_lsr(d, alpha, beta) - 2.0 * (t_lsr(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    inline double s_33_2(double d, double alpha, double beta)
    {
        return p_lsl(d, alpha, beta) - p_lsr(d, alpha, beta) - 2.0 * (q_lsr(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    inline double s_34(double d, double alpha, double beta)
    {
        return p_rsr(d, alpha, beta) - p_lsr(d, alpha, beta) - 2.0 * (t_lsr(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    inline double s_41_1(double d, double alpha, double beta)
    {
        return t_lsl(d, alpha, beta) - boost::math::constants::pi<double>();
    }

    inline double s_41_2(double d, double alpha, double beta)
    {
        return q_lsl(d, alpha, beta) - boost::math::constants::pi<double>();
    }

    inline double s_42(double d, double alpha, double beta)
    {
        return t_lsl(d, alpha, beta) - boost::math::constants::pi<double>();
    }

    inline double s_43(double d, double alpha, double beta)
    {
        return p_lsl(d, alpha, beta) - p_lsr(d, alpha, beta) - 2.0 * (q_lsr(d, alpha, beta) - boost::math::constants::pi<double>());
    }

    DubinsUnicycleSpace::DubinsPath dubinsLSL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sa - sb));
        if (tmp >= DUBINS_ZERO)
        {
            double theta = atan2(cb - ca, d + sa - sb);
            double t = mod2pi(-alpha + theta);
            double p = sqrt(std::max(tmp, 0.));
            double q = mod2pi(beta - theta);
            if(fabs(p * cos(alpha + t) - sa + sb - d) >= 2 * DUBINS_EPS)
                return {};
            if(fabs(p * sin(alpha + t) + ca - cb) >= 2 * DUBINS_EPS)
                return {};
            if(mod2pi(alpha + t + q - beta + .5 * DUBINS_EPS) >= DUBINS_EPS)
                return {};
            // assert(fabs(p * cos(alpha + t) - sa + sb - d) < 2 * DUBINS_EPS);
            // assert(fabs(p * sin(alpha + t) + ca - cb) < 2 * DUBINS_EPS);
            // assert(mod2pi(alpha + t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsUnicycleSpace::DubinsPath(DubinsUnicycleSpace::dubinsPathType[0], t, p, q);
        }
        return {};
    }

    DubinsUnicycleSpace::DubinsPath dubinsRSR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa));
        if (tmp >= DUBINS_ZERO)
        {
            double theta = atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta);
            double p = sqrt(std::max(tmp, 0.));
            double q = mod2pi(-beta + theta);
            if(fabs(p * cos(alpha - t) + sa - sb - d) >= 2 * DUBINS_EPS)
                return {};
            if(fabs(p * sin(alpha - t) - ca + cb) >= 2 * DUBINS_EPS)
                return {};
            if(mod2pi(alpha - t - q - beta + .5 * DUBINS_EPS) >= DUBINS_EPS)
                return {};
            // assert(fabs(p * cos(alpha - t) + sa - sb - d) < 2 * DUBINS_EPS);
            // assert(fabs(p * sin(alpha - t) - ca + cb) < 2 * DUBINS_EPS);
            // assert(mod2pi(alpha - t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsUnicycleSpace::DubinsPath(DubinsUnicycleSpace::dubinsPathType[1], t, p, q);
        }
        return {};
    }

    DubinsUnicycleSpace::DubinsPath dubinsRSL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb));
        if (tmp >= DUBINS_ZERO)
        {
            double p = sqrt(std::max(tmp, 0.));
            double theta = atan2(ca + cb, d - sa - sb) - atan2(2., p);
            double t = mod2pi(alpha - theta);
            double q = mod2pi(beta - theta);
            if(fabs(p * cos(alpha - t) - 2. * sin(alpha - t) + sa + sb - d) >= 2 * DUBINS_EPS)
                return {};
            if(fabs(p * sin(alpha - t) + 2. * cos(alpha - t) - ca - cb) >= 2 * DUBINS_EPS)
                return {};
            if(mod2pi(alpha - t + q - beta + .5 * DUBINS_EPS) >= DUBINS_EPS)
                return {};
            // assert(fabs(p * cos(alpha - t) - 2. * sin(alpha - t) + sa + sb - d) < 2 * DUBINS_EPS);
            // assert(fabs(p * sin(alpha - t) + 2. * cos(alpha - t) - ca - cb) < 2 * DUBINS_EPS);
            // assert(mod2pi(alpha - t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsUnicycleSpace::DubinsPath(DubinsUnicycleSpace::dubinsPathType[2], t, p, q);
        }
        return {};
    }

    DubinsUnicycleSpace::DubinsPath dubinsLSR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb));
        if (tmp >= DUBINS_ZERO)
        {
            double p = sqrt(std::max(tmp, 0.));
            double theta = atan2(-ca - cb, d + sa + sb) - atan2(-2., p);
            double t = mod2pi(-alpha + theta);
            double q = mod2pi(-beta + theta);
            if(fabs(p * cos(alpha + t) + 2. * sin(alpha + t) - sa - sb - d) >= 2 * DUBINS_EPS)
                return {};
            if(fabs(p * sin(alpha + t) - 2. * cos(alpha + t) + ca + cb) >= 2 * DUBINS_EPS)
                return {};
            if(mod2pi(alpha + t - q - beta + .5 * DUBINS_EPS) >= DUBINS_EPS)
                return {};
            // assert(fabs(p * cos(alpha + t) + 2. * sin(alpha + t) - sa - sb - d) < 2 * DUBINS_EPS);
            // assert(fabs(p * sin(alpha + t) - 2. * cos(alpha + t) + ca + cb) < 2 * DUBINS_EPS);
            // assert(mod2pi(alpha + t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsUnicycleSpace::DubinsPath(DubinsUnicycleSpace::dubinsPathType[3], t, p, q);
        }
        return {};
    }

    DubinsUnicycleSpace::DubinsPath dubinsRLR(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)));
        if (fabs(tmp) < 1.)
        {
            double p = twopi - acos(tmp);
            double theta = atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta + .5 * p);
            double q = mod2pi(alpha - beta - t + p);
            if(fabs(2. * sin(alpha - t + p) - 2. * sin(alpha - t) - d + sa - sb) >= 2 * DUBINS_EPS)
                return {};
            if(fabs(-2. * cos(alpha - t + p) + 2. * cos(alpha - t) - ca + cb) >= 2 * DUBINS_EPS)
                return {};
            if(mod2pi(alpha - t + p - q - beta + .5 * DUBINS_EPS) >= DUBINS_EPS)
                return {};
            // assert(fabs(2. * sin(alpha - t + p) - 2. * sin(alpha - t) - d + sa - sb) < 2 * DUBINS_EPS);
            // assert(fabs(-2. * cos(alpha - t + p) + 2. * cos(alpha - t) - ca + cb) < 2 * DUBINS_EPS);
            // assert(mod2pi(alpha - t + p - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsUnicycleSpace::DubinsPath(DubinsUnicycleSpace::dubinsPathType[4], t, p, q);
        }
        return {};
    }

    DubinsUnicycleSpace::DubinsPath dubinsLRL(double d, double alpha, double beta)
    {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb)));
        if (fabs(tmp) < 1.)
        {
            double p = twopi - acos(tmp);
            double theta = atan2(-ca + cb, d + sa - sb);
            double t = mod2pi(-alpha + theta + .5 * p);
            double q = mod2pi(beta - alpha - t + p);
            if(fabs(-2. * sin(alpha + t - p) + 2. * sin(alpha + t) - d - sa + sb) >= 2 * DUBINS_EPS)
                return {};
            if(fabs(2. * cos(alpha + t - p) - 2. * cos(alpha + t) + ca - cb) >= 2 * DUBINS_EPS)
                return {};
            if(mod2pi(alpha + t - p + q - beta + .5 * DUBINS_EPS) >= DUBINS_EPS)
                return {};
            // assert(fabs(-2. * sin(alpha + t - p) + 2. * sin(alpha + t) - d - sa + sb) < 2 * DUBINS_EPS);
            // assert(fabs(2. * cos(alpha + t - p) - 2. * cos(alpha + t) + ca - cb) < 2 * DUBINS_EPS);
            // assert(mod2pi(alpha + t - p + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsUnicycleSpace::DubinsPath(DubinsUnicycleSpace::dubinsPathType[5], t, p, q);
        }
        return {};
    }

    bool is_longpath_case(double d, double alpha, double beta)
    {
        return (std::abs(std::sin(alpha)) + std::abs(std::sin(beta)) +
                std::sqrt(4 - std::pow(std::cos(alpha) + std::cos(beta), 2)) - d) < 0;
    }

    DubinsUnicycleSpace::DubinsPath dubins_exhaustive(const double d, const double alpha, const double beta)
    {
        if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS)
            return {DubinsUnicycleSpace::dubinsPathType[0], 0, d, 0};

        DubinsUnicycleSpace::DubinsPath path(dubinsLSL(d, alpha, beta)), tmp(dubinsRSR(d, alpha, beta));
        double len, minLength = path.length();

        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsRSL(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsLSR(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsRLR(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
        {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsLRL(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
            path = tmp;
        return path;
    }

    DubinsClass getDubinsClass(const double alpha, const double beta)
    {
        int row(0), column(0);
        if (0 <= alpha && alpha <= boost::math::constants::half_pi<double>())
        {
            row = 1;
        }
        else if (boost::math::constants::half_pi<double>() < alpha && alpha <= boost::math::constants::pi<double>())
        {
            row = 2;
        }
        else if (boost::math::constants::pi<double>() < alpha && alpha <= 3 * boost::math::constants::half_pi<double>())
        {
            row = 3;
        }
        else if (3 * boost::math::constants::half_pi<double>() < alpha && alpha <= twopi)
        {
            row = 4;
        }

        if (0 <= beta && beta <= boost::math::constants::half_pi<double>())
        {
            column = 1;
        }
        else if (boost::math::constants::half_pi<double>() < beta && beta <= boost::math::constants::pi<double>())
        {
            column = 2;
        }
        else if (boost::math::constants::pi<double>() < beta && beta <= 3 * boost::math::constants::half_pi<double>())
        {
            column = 3;
        }
        else if (3 * boost::math::constants::half_pi<double>() < beta &&
                 beta <= 2.0 * boost::math::constants::pi<double>())
        {
            column = 4;
        }

        assert(row >= 1 && row <= 4 &&
               "alpha is not in the range of [0,2pi] in classifyPath(double alpha, double beta).");
        assert(column >= 1 && column <= 4 &&
               "beta is not in the range of [0,2pi] in classifyPath(double alpha, double beta).");
        assert((column - 1) + 4 * (row - 1) >= 0 && (column - 1) + 4 * (row - 1) <= 15 &&
               "class is not in range [0,15].");
        return (DubinsClass)((column - 1) + 4 * (row - 1));
    }
}  // namespace

// ###########################################################################################################################################
// STATE MANAGEMENT FUNCTIONS

ompl::base::State* ompl::base::DubinsUnicycleSpace::allocState() const{
    StateType *state = new StateType();
    allocStateComponents(state);
    return state;
}

void ompl::base::DubinsUnicycleSpace::freeState(ompl::base::State *state) const{
    CompoundStateSpace::freeState(state);
}

void ompl::base::DubinsUnicycleSpace::copyState(ompl::base::State *destination, const ompl::base::State *source) const{
    // copy the state data
    destination->as<StateType>()->setX(source->as<StateType>()->getX());
    destination->as<StateType>()->setY(source->as<StateType>()->getY());
    destination->as<StateType>()->setYaw(source->as<StateType>()->getYaw());
    destination->as<StateType>()->setPrevX(source->as<StateType>()->getPrevX());
    destination->as<StateType>()->setPrevY(source->as<StateType>()->getPrevY());
    
    // copy the Sensitivity initial conditions 
    if(source->as<StateType>()->PI_.size() > 0)
        destination->as<StateType>()->PI_ = source->as<StateType>()->PI_;
    if(source->as<StateType>()->PI_xi_.size() > 0)
        destination->as<StateType>()->PI_xi_ = source->as<StateType>()->PI_xi_;
    if(source->as<StateType>()->xi_.size() > 0)
        destination->as<StateType>()->xi_ = source->as<StateType>()->xi_;

    // copy the hidden state  
    if(source->as<StateType>()->h0_.sizes()[0]>0)
        destination->as<StateType>()->h0_ = source->as<StateType>()->h0_;

    // copy the nominal state
    if(source->as<StateType>()->nominal_state_.size()>0)
        destination->as<StateType>()->nominal_state_ = source->as<StateType>()->nominal_state_;
    
    destination->as<StateType>()->st_time_ = source->as<StateType>()->st_time_;
}

DubinsUnicycleSpace::DubinsPath dubins_classification(const double d, const double alpha, const double beta)
{
    if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS)
        return {DubinsUnicycleSpace::dubinsPathType[0], 0, d, 0};
    // Dubins set classification scheme
    // Shkel, Andrei M., and Vladimir Lumelsky. "Classification of the Dubins set."
    //   Robotics and Autonomous Systems 34.4 (2001): 179-202.
    // Lim, Jaeyoung, et al. "Circling Back: Dubins set Classification Revisited."
    //   Workshop on Energy Efficient Aerial Robotic Systems, International Conference on Robotics and Automation 2023.
    //   2023.
    DubinsUnicycleSpace::DubinsPath path;
    auto dubins_class = getDubinsClass(alpha, beta);
    switch (dubins_class)
    {
        case DubinsClass::A11:
        {
            path = dubinsRSL(d, alpha, beta);
            break;
        }
        case DubinsClass::A12:
        {
            if (s_13(d, alpha, beta) < 0.0)
            {
                path = (s_12(d, alpha, beta) < 0.0) ? dubinsRSR(d, alpha, beta) : dubinsRSL(d, alpha, beta);
            }
            else
            {
                path = dubinsLSR(d, alpha, beta);
                DubinsUnicycleSpace::DubinsPath tmp = dubinsRSL(d, alpha, beta);
                if (path.length() > tmp.length())
                {
                    path = tmp;
                }
            }
            break;
        }
        case DubinsClass::A13:
        {
            if (s_13(d, alpha, beta) < 0.0)
            {
                path = dubinsRSR(d, alpha, beta);
            }
            else
            {
                path = dubinsLSR(d, alpha, beta);
            }
            break;
        }
        case DubinsClass::A14:
        {
            if (s_14_1(d, alpha, beta) > 0.0)
            {
                path = dubinsLSR(d, alpha, beta);
            }
            else if (s_24(d, alpha, beta) > 0.0)
            {
                path = dubinsRSL(d, alpha, beta);
            }
            else
            {
                path = dubinsRSR(d, alpha, beta);
            }
            break;
        }
        case DubinsClass::A21:
        {
            if (s_31(d, alpha, beta) < 0.0)
            {
                if (s_21(d, alpha, beta) < 0.0)
                {
                    path = dubinsLSL(d, alpha, beta);
                }
                else
                {
                    path = dubinsRSL(d, alpha, beta);
                }
            }
            else
            {
                path = dubinsLSR(d, alpha, beta);
                DubinsUnicycleSpace::DubinsPath tmp = dubinsRSL(d, alpha, beta);
                if (path.length() > tmp.length())
                {
                    path = tmp;
                }
            }
            break;
        }
        case DubinsClass::A22:
        {
            if (alpha > beta)
            {
                path = (s_22_1(d, alpha, beta) < 0.0) ? dubinsLSL(d, alpha, beta) : dubinsRSL(d, alpha, beta);
            }
            else
            {
                path = (s_22_2(d, alpha, beta) < 0.0) ? dubinsRSR(d, alpha, beta) : dubinsRSL(d, alpha, beta);
            }
            break;
        }
        case DubinsClass::A23:
        {
            path = dubinsRSR(d, alpha, beta);
            break;
        }
        case DubinsClass::A24:
        {
            if (s_24(d, alpha, beta) < 0.0)
            {
                path = dubinsRSR(d, alpha, beta);
            }
            else
            {
                path = dubinsRSL(d, alpha, beta);
            }
            break;
        }
        case DubinsClass::A31:
        {
            if (s_31(d, alpha, beta) < 0.0)
            {
                path = dubinsLSL(d, alpha, beta);
            }
            else
            {
                path = dubinsLSR(d, alpha, beta);
            }
            break;
        }
        case DubinsClass::A32:
        {
            path = dubinsLSL(d, alpha, beta);
            break;
        }
        case DubinsClass::A33:
        {
            if (alpha < beta)
            {
                if (s_33_1(d, alpha, beta) < 0.0)
                {
                    path = dubinsRSR(d, alpha, beta);
                }
                else
                {
                    path = dubinsLSR(d, alpha, beta);
                }
            }
            else
            {
                if (s_33_2(d, alpha, beta) < 0.0)
                {
                    path = dubinsLSL(d, alpha, beta);
                }
                else
                {
                    path = dubinsLSR(d, alpha, beta);
                }
            }
            break;
        }
        case DubinsClass::A34:
        {
            if (s_24(d, alpha, beta) < 0.0)
            {
                if (s_34(d, alpha, beta) < 0.0)
                {
                    path = dubinsRSR(d, alpha, beta);
                }
                else
                {
                    path = dubinsLSR(d, alpha, beta);
                }
            }
            else
            {
                path = dubinsLSR(d, alpha, beta);
                DubinsUnicycleSpace::DubinsPath tmp = dubinsRSL(d, alpha, beta);
                if (path.length() > tmp.length())
                {
                    path = tmp;
                }
            }
            break;
        }
        case DubinsClass::A41:
        {
            if (s_41_1(d, alpha, beta) > 0.0)
            {
                path = dubinsRSL(d, alpha, beta);
            }
            else if (s_41_2(d, alpha, beta) > 0.0)
            {
                path = dubinsLSR(d, alpha, beta);
            }
            else
            {
                path = dubinsLSL(d, alpha, beta);
            }
            break;
        }
        case DubinsClass::A42:
        {
            if (s_42(d, alpha, beta) < 0.0)
            {
                path = dubinsLSL(d, alpha, beta);
            }
            else
            {
                path = dubinsRSL(d, alpha, beta);
            }
            break;
        }
        case DubinsClass::A43:
        {
            if (s_42(d, alpha, beta) < 0.0)
            {
                if (s_43(d, alpha, beta) < 0.0)
                {
                    path = dubinsLSL(d, alpha, beta);
                }
                else
                {
                    path = dubinsLSR(d, alpha, beta);
                }
            }
            else
            {
                path = dubinsLSR(d, alpha, beta);
                DubinsUnicycleSpace::DubinsPath tmp = dubinsRSL(d, alpha, beta);
                if (path.length() > tmp.length())
                {
                    path = tmp;
                }
            }
            break;
        }
        case DubinsClass::A44:
        {
            path = dubinsLSR(d, alpha, beta);
            break;
        }
    }
    return path;
}

DubinsUnicycleSpace::DubinsPath dubins(double d, double alpha, double beta)
{
    if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS)
        return {DubinsUnicycleSpace::dubinsPathType[0], 0, d, 0};
    alpha = mod2pi(alpha);
    beta = mod2pi(beta);
    return is_longpath_case(d, alpha, beta) ? ::dubins_classification(d, alpha, beta) :
                                              ::dubins_exhaustive(d, alpha, beta);
}

const ompl::base::DubinsUnicycleSpace::DubinsPathSegmentType ompl::base::DubinsUnicycleSpace::dubinsPathType[6][3] = {
    {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_LEFT},  {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_RIGHT},
    {DUBINS_RIGHT, DUBINS_STRAIGHT, DUBINS_LEFT}, {DUBINS_LEFT, DUBINS_STRAIGHT, DUBINS_RIGHT},
    {DUBINS_RIGHT, DUBINS_LEFT, DUBINS_RIGHT},    {DUBINS_LEFT, DUBINS_RIGHT, DUBINS_LEFT}};

double ompl::base::DubinsUnicycleSpace::distance(const State *state1, const State *state2) const
{
    if (isSymmetric_)
        return rho_ * std::min(dubins(state1, state2).length(), dubins(state2, state1).length());
    return rho_ * dubins(state1, state2).length();
}

void ompl::base::DubinsUnicycleSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    bool firstTime = true;
    DubinsPath path;
    interpolate(from, to, t, firstTime, path, state);
}

void ompl::base::DubinsUnicycleSpace::interpolate(const State *from, const State *to, const double t, bool &firstTime,
                                               DubinsPath &path, State *state) const
{
    if (firstTime)
    {
        if (t >= 1.)
        {
            if (to != state)
                copyState(state, to);
            return;
        }
        if (t <= 0.)
        {
            if (from != state)
                copyState(state, from);
            return;
        }

        path = dubins(from, to);
        if (isSymmetric_)
        {
            DubinsPath path2(dubins(to, from));
            if (path2.length() < path.length())
            {
                path2.reverse_ = true;
                path = path2;
            }
        }
        firstTime = false;
    }
    interpolate(from, path, t, state);
}

void ompl::base::DubinsUnicycleSpace::interpolate(const State *from, const DubinsPath &path, double t, State *state) const
{
    auto *s = allocState()->as<StateType>();
    double seg = t * path.length(), phi, v;

    s->setXY(0., 0.);
    s->setYaw(from->as<StateType>()->getYaw());
    if (!path.reverse_)
    {
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[i]);
            phi = s->getYaw();
            seg -= v;
            switch (path.type_[i])
            {
                case DUBINS_LEFT:
                    s->setXY(s->getX() + sin(phi + v) - sin(phi), s->getY() - cos(phi + v) + cos(phi));
                    s->setYaw(phi + v);
                    break;
                case DUBINS_RIGHT:
                    s->setXY(s->getX() - sin(phi - v) + sin(phi), s->getY() + cos(phi - v) - cos(phi));
                    s->setYaw(phi - v);
                    break;
                case DUBINS_STRAIGHT:
                    s->setXY(s->getX() + v * cos(phi), s->getY() + v * sin(phi));
                    break;
            }
        }
    }
    else
    {
        for (unsigned int i = 0; i < 3 && seg > 0; ++i)
        {
            v = std::min(seg, path.length_[2 - i]);
            phi = s->getYaw();
            seg -= v;
            switch (path.type_[2 - i])
            {
                case DUBINS_LEFT:
                    s->setXY(s->getX() + sin(phi - v) - sin(phi), s->getY() - cos(phi - v) + cos(phi));
                    s->setYaw(phi - v);
                    break;
                case DUBINS_RIGHT:
                    s->setXY(s->getX() - sin(phi + v) + sin(phi), s->getY() + cos(phi + v) - cos(phi));
                    s->setYaw(phi + v);
                    break;
                case DUBINS_STRAIGHT:
                    s->setXY(s->getX() - v * cos(phi), s->getY() - v * sin(phi));
                    break;
            }
        }
    }
    state->as<StateType>()->setX(s->getX() * rho_ + from->as<StateType>()->getX());
    state->as<StateType>()->setY(s->getY() * rho_ + from->as<StateType>()->getY());
    getSubspace(1)->enforceBounds(s->as<SO2StateSpace::StateType>(1));
    state->as<StateType>()->setYaw(s->getYaw());
    freeState(s);
}

void ompl::base::DubinsUnicycleSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double dt, std::vector<ompl::base::State*>& states) const
{
    bool firstTime = true;
    DubinsUnicycleSpace::DubinsPath path;
    int nd = validSegmentCount(from, to);

    // We compute the path once to access the length, we don't care about the state here
    State *s = allocState();
    interpolate(from, to, 0.5, firstTime, path, s);
    freeState(s);

    // Copy the initial state
    State* from_cpy = allocState();
    copyState(from_cpy, from);
    states.push_back(from_cpy);

    // We compute the total time
    double time = dt;
    double total_time = path.length() / getRobot()->as<ompl::Unicycle>()->getV();

    while(time+EPSILON < total_time)
    {
        /* temporary storage for the checked state */
        State *test = allocState();

        interpolate(from, to, time/total_time, firstTime, path, test);

        total_time = path.length() / getRobot()->as<ompl::Unicycle>()->getV();

        test->as<StateType>()->st_time_ = from->as<StateType>()->st_time_ + time;
        
        states.push_back(test);

        time += dt;
    }

    // Copy the final state
    State* to_cpy = allocState();
    copyState(to_cpy, to);
    to_cpy->as<StateType>()->st_time_ = from->as<StateType>()->st_time_ + total_time;
    states.push_back(to_cpy);

    states.back()->as<StateType>()->setPrevTime(states.at(states.size()-2)->as<StateType>()->st_time_); 
    states.back()->as<StateType>()->setPrevX(states.at(states.size()-2)->as<StateType>()->getX()); 
    states.back()->as<StateType>()->setPrevY(states.at(states.size()-2)->as<StateType>()->getY()); 

    return;
}

void ompl::base::DubinsUnicycleSpace::interpolateLearning(const ompl::base::State *from, const ompl::base::State *to, const double dt, 
    std::vector<ompl::base::State*>& states, std::vector<std::vector<float>> &vector_to_tensor) const
{
    bool firstTime = true;
    DubinsUnicycleSpace::DubinsPath path;
    int nd = validSegmentCount(from, to);

    // We compute the path once to access the length, we don't care about the state here
    State *s = allocState();
    interpolate(from, to, 0.5, firstTime, path, s);
    freeState(s);

    // Copy the initial state
    State* from_cpy = allocState();
    copyState(from_cpy, from);
    states.push_back(from_cpy);

    std::vector<std::vector<double>> des_learning;

    des_learning.push_back({from->as<StateType>()->getX(), from->as<StateType>()->getY(), from->as<StateType>()->getYaw()});
    vector_to_tensor.push_back(robot_->getRobotLearningInput(des_learning));

    // We compute the total time
    double time = dt;
    double total_time = path.length() / getRobot()->as<ompl::Unicycle>()->getV();

    while(time+EPSILON < total_time)
    {
        /* temporary storage for the checked state */
        State *test = allocState();

        des_learning.clear();

        interpolate(from, to, time/total_time, firstTime, path, test);

        total_time = path.length() / getRobot()->as<ompl::Unicycle>()->getV();

        test->as<StateType>()->st_time_ = from->as<StateType>()->st_time_ + time;

        states.push_back(test);

        des_learning.push_back({test->as<StateType>()->getX(), test->as<StateType>()->getY(), test->as<StateType>()->getYaw()});
        vector_to_tensor.push_back(robot_->getRobotLearningInput(des_learning));

        time += dt;
    }

    // Copy the final state
    State* to_cpy = allocState();
    copyState(to_cpy, to);
    to_cpy->as<StateType>()->st_time_ = from->as<StateType>()->st_time_ + total_time;
    states.push_back(to_cpy);

    states.back()->as<StateType>()->setPrevTime(states.at(states.size()-2)->as<StateType>()->st_time_); 
    states.back()->as<StateType>()->setPrevX(states.at(states.size()-2)->as<StateType>()->getX()); 
    states.back()->as<StateType>()->setPrevY(states.at(states.size()-2)->as<StateType>()->getY()); 

  return;
}

void ompl::base::DubinsUnicycleSpace::projectStates(const std::vector<ompl::base::State*> &des_states, std::vector<std::vector<double>> &collision_states) const
{
    /* S03 state for CC: x, y, z, qw, qx, qy, qz */
    std::vector<double> st;
    for (auto &state : des_states)
    {
        st.clear();

        st.push_back(state->as<StateType>()->getX()); // X
        st.push_back(state->as<StateType>()->getY()); // Y
        st.push_back(0.0); // Z

        // Convert the orientations from rpy to quaternion
        Eigen::Quaterniond quat = euler2Quaternion(0.0,0.0,state->as<StateType>()->getYaw()); 
        
        st.push_back(quat.w()); // qw
        st.push_back(quat.x()); // qx
        st.push_back(quat.y()); // qy
        st.push_back(quat.z()); // qz

        collision_states.push_back(st);
    }
}

bool ompl::base::DubinsUnicycleSpace::simulateStates(const std::vector<ompl::base::State*> &des_states, const double dt, std::vector<std::vector<double>> &collision_states) const
{
    /* S03 state for CC: x, y, z, qw, qx, qy, qz */
    /* Unicycle final nominal state: x, y, theta */

    services_msgs::DynamicSrv dynamicSrv; // To comunicate with the dynamic service.

    /* Initialize integrator */
    dynamicSrv.request.integrator = getRobot()->getIntegratorType();

    /* Message initialization */
    dynamicSrv.request.t_init = 0.0; // Always start at 0 for the local path (even if the initial conditions are present).
    dynamicSrv.request.t_final = des_states.size()*dt; // Desired path length, we approximate the last state at a time step of dt.
    dynamicSrv.request.dt = dt; // Integration time step.

    /* Initialize the robot state (i.e. q0) */
    dynamicSrv.request.init_robot_state.x = des_states.at(0)->as<StateType>()->nominal_state_.at(0);
    dynamicSrv.request.init_robot_state.y = des_states.at(0)->as<StateType>()->nominal_state_.at(1);
    dynamicSrv.request.init_robot_state.vx = des_states.at(0)->as<StateType>()->nominal_state_.at(2);
    dynamicSrv.request.init_robot_state.vy = des_states.at(0)->as<StateType>()->nominal_state_.at(3);
    dynamicSrv.request.init_robot_state.vz = 0.0;
    dynamicSrv.request.init_robot_state.qw = des_states.at(0)->as<StateType>()->nominal_state_.at(4);
    dynamicSrv.request.init_robot_state.qx = des_states.at(0)->as<StateType>()->nominal_state_.at(5);
    dynamicSrv.request.init_robot_state.qy = des_states.at(0)->as<StateType>()->nominal_state_.at(6);
    dynamicSrv.request.init_robot_state.qz = des_states.at(0)->as<StateType>()->nominal_state_.at(7);
    dynamicSrv.request.init_robot_state.wx = 0.0;
    dynamicSrv.request.init_robot_state.wy = 0.0;
    dynamicSrv.request.init_robot_state.wz = des_states.at(0)->as<StateType>()->nominal_state_.at(8);

    dynamicSrv.request.initial_xi = des_states.at(0)->as<StateType>()->xi_;

    /* Initialize uncertain model parameters and controler gains */
    dynamicSrv.request.model_params = getRobot()->getUncertainParams();
    
    std::vector<std::vector<double>> robot_gains = getRobot()->getGains();
    for(int i = 0; i<robot_gains.size(); i++)
    {
        services_msgs::GainsValues gains_values;
        gains_values.values = robot_gains.at(i);
        dynamicSrv.request.gains.push_back(gains_values); 
    }

    /* Compute velocities and acceleration by differentation */
    std::vector<double> des_x, des_y, times;
    for(int i = 0; i<des_states.size(); i++)
    {
        des_x.push_back(des_states.at(i)->as<StateType>()->getX());
        des_y.push_back(des_states.at(i)->as<StateType>()->getY());
        times.push_back(des_states.at(i)->as<StateType>()->st_time_);
    }

    std::vector<double> des_vx(des_x.size()), des_vy(des_x.size()), des_ax(des_x.size()), des_ay(des_x.size());

    // Finite difference
    des_vx[0] = (des_x[0] - des_states.at(0)->as<StateType>()->getPrevX()) / (des_states.at(0)->as<StateType>()->st_time_ - des_states.at(0)->as<StateType>()->getPrevTime());
    des_vy[0] = (des_y[0] - des_states.at(0)->as<StateType>()->getPrevY()) / (des_states.at(0)->as<StateType>()->st_time_ - des_states.at(0)->as<StateType>()->getPrevTime());

    // std::cout << "PrevX : " << des_states.at(0)->as<StateType>()->getPrevX() << std::endl;

    // Main loop
    for (size_t i = 1; i < des_x.size(); ++i) {
        des_vx[i] = (des_x[i] - des_x[i - 1]) / (times[i] - times[i - 1]);
        des_vy[i] = (des_y[i] - des_y[i - 1]) / (times[i] - times[i - 1]);
    }

    /* Fill the desired trajectory. The first one has already been used so we ski it. */
    for(int i = 1; i<des_x.size(); i++)
    {
        services_msgs::DesiredState des_state;

        des_state.x = des_x.at(i);
        des_state.y = des_y.at(i);
        des_state.z = 0.0;
        des_state.yaw = des_states.at(i)->as<StateType>()->getYaw();
        des_state.vx = des_vx.at(i);
        des_state.vy = des_vy.at(i);
        des_state.vz = 0.0;
        des_state.wyaw = 0.0;
        des_state.ax = des_ax.at(i);
        des_state.ay = des_ay.at(i);
        des_state.az = 0.0;

        dynamicSrv.request.desired_states.push_back(des_state);
    }

    // To compare the service call time with the ODE resolution time
    auto start = std::chrono::high_resolution_clock::now();
    // Solve the dynamic
    if(!getRobot()->dynamic_.call(dynamicSrv))
    {
        ROS_WARN("Unable to solve tracking.");
        return false;
    }
    else
    {
        // To compare the service call time with the ODE resolution time
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> srv_duration = end - start;
        
        // Get the simulated states
        std::vector<double> trajX = dynamicSrv.response.trajX;
        std::vector<double> trajY = dynamicSrv.response.trajY;
        std::vector<double> trajVX = dynamicSrv.response.trajVX;
        std::vector<double> trajVY = dynamicSrv.response.trajVY;
        std::vector<double> trajQw = dynamicSrv.response.trajQw;
        std::vector<double> trajQx = dynamicSrv.response.trajQx;
        std::vector<double> trajQy = dynamicSrv.response.trajQy;
        std::vector<double> trajQz = dynamicSrv.response.trajQz;
        std::vector<double> trajWz = dynamicSrv.response.trajWz;

        for(int i = 0; i<trajX.size(); i++)
        {
            std::vector<double> st;
            st.push_back(trajX.at(i));
            st.push_back(trajY.at(i));
            st.push_back(0.0);
            st.push_back(trajQw.at(i));
            st.push_back(trajQx.at(i));
            st.push_back(trajQy.at(i));
            st.push_back(trajQz.at(i));
            collision_states.push_back(st);
        }

        // Make sure there is is not already a registered state
        des_states.back()->as<StateType>()->nominal_state_.clear();
        // Register the new nominal state
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajX.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajY.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajVX.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajVY.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQw.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQx.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQy.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQz.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(0.0);

        des_states.back()->as<StateType>()->xi_ = dynamicSrv.response.final_xi;

        // To compare the service call time with the ODE resolution time
        double srv_time_percentage = ((srv_duration.count()-dynamicSrv.response.executionTime)/srv_duration.count())*100;
        // std::cout << "Loss of service call time in percent : " << srv_time_percentage << std::endl;
        return true;
    }
}

bool ompl::base::DubinsUnicycleSpace::simulateStatesAndTubes(const std::vector<ompl::base::State*> &des_states, double dt, std::vector<std::vector<double>> &collision_states,
                                        std::vector<std::vector<double>> &control_inputs, std::vector<std::vector<double>> &radii) const
{
    /* S03 state for CC: x, y, z, qw, qx, qy, qz */
    /* Unicycle final nominal state: x, y, vx, vy, qw, qx, qy, qz, wz */

    services_msgs::SensitivitySrv sensitivitySrv; // To comunicate with the sensitivity service.

    /* Initialize integrator */
    sensitivitySrv.request.integrator = getRobot()->getIntegratorType();

    /* Message initialization */
    sensitivitySrv.request.t_init = 0.0; // Always start at 0 for the local path (even if the initial conditions are present).
    sensitivitySrv.request.t_final = des_states.size()*dt; // Desired path length, we approximate the last state at a time step of dt.
    sensitivitySrv.request.dt = dt; // Integration time step.

    /* Initialize the robot state (i.e. q0) */
    sensitivitySrv.request.init_robot_state.x = des_states.at(0)->as<StateType>()->nominal_state_.at(0);
    sensitivitySrv.request.init_robot_state.y = des_states.at(0)->as<StateType>()->nominal_state_.at(1);
    sensitivitySrv.request.init_robot_state.vx = des_states.at(0)->as<StateType>()->nominal_state_.at(2);
    sensitivitySrv.request.init_robot_state.vy = des_states.at(0)->as<StateType>()->nominal_state_.at(3);
    sensitivitySrv.request.init_robot_state.vz = 0.0;
    sensitivitySrv.request.init_robot_state.qw = des_states.at(0)->as<StateType>()->nominal_state_.at(4);
    sensitivitySrv.request.init_robot_state.qx = des_states.at(0)->as<StateType>()->nominal_state_.at(5);
    sensitivitySrv.request.init_robot_state.qy = des_states.at(0)->as<StateType>()->nominal_state_.at(6);
    sensitivitySrv.request.init_robot_state.qz = des_states.at(0)->as<StateType>()->nominal_state_.at(7);
    sensitivitySrv.request.init_robot_state.wx = 0.0;
    sensitivitySrv.request.init_robot_state.wy = 0.0;
    sensitivitySrv.request.init_robot_state.wz = des_states.at(0)->as<StateType>()->nominal_state_.at(8);

    /* Initialize the sensitivity conditions (i.e. PI0, PI_xi0) */
    sensitivitySrv.request.initial_PI = des_states.at(0)->as<StateType>()->PI_;
    sensitivitySrv.request.initial_PI_xi = des_states.at(0)->as<StateType>()->PI_xi_;
    sensitivitySrv.request.initial_xi = des_states.at(0)->as<StateType>()->xi_;
    
    /* Initialize uncertain model parameters and controler gains */
    sensitivitySrv.request.model_params = getRobot()->getUncertainParams();

    std::vector<std::vector<double>> robot_gains = getRobot()->getGains();
    for(int i = 0; i<robot_gains.size(); i++)
    {
        services_msgs::GainsValues gains_values;
        gains_values.values = robot_gains.at(i);
        sensitivitySrv.request.gains.push_back(gains_values); 
    }

    /* Compute velocities and acceleration by differentation */
    std::vector<double> des_x, des_y, times;
    for(int i = 0; i<des_states.size(); i++)
    {
        des_x.push_back(des_states.at(i)->as<StateType>()->getX());
        des_y.push_back(des_states.at(i)->as<StateType>()->getY());
        times.push_back(des_states.at(i)->as<StateType>()->st_time_);
    }

    std::vector<double> des_vx(des_x.size()), des_vy(des_x.size()), des_ax(des_x.size()), des_ay(des_x.size());

    // Finite difference
    des_vx[0] = (des_x[0] - des_states.at(0)->as<StateType>()->getPrevX()) / (des_states.at(0)->as<StateType>()->st_time_ - des_states.at(0)->as<StateType>()->getPrevTime());
    des_vy[0] = (des_y[0] - des_states.at(0)->as<StateType>()->getPrevY()) / (des_states.at(0)->as<StateType>()->st_time_ - des_states.at(0)->as<StateType>()->getPrevTime());

    // std::cout << "PrevX : " << des_states.at(0)->as<StateType>()->getPrevX() << std::endl;

    // Main loop
    for (size_t i = 1; i < des_x.size(); ++i) {
        des_vx[i] = (des_x[i] - des_x[i - 1]) / (times[i] - times[i - 1]);
        des_vy[i] = (des_y[i] - des_y[i - 1]) / (times[i] - times[i - 1]);
    }

    /* Fill the desired trajectory. The first one has already been used so we skip it. */
    for(int i = 1; i<des_x.size(); i++)
    {
        services_msgs::DesiredState des_state;

        des_state.x = des_x.at(i);
        des_state.y = des_y.at(i);
        des_state.z = 0.0;
        des_state.yaw = des_states.at(i)->as<StateType>()->getYaw();
        des_state.vx = des_vx.at(i);
        des_state.vy = des_vy.at(i);
        des_state.vz = 0.0;
        des_state.wyaw = 0.0;
        des_state.ax = des_ax.at(i);
        des_state.ay = des_ay.at(i);
        des_state.az = 0.0;

        sensitivitySrv.request.desired_states.push_back(des_state);
    }

    // To compare the service call time with the ODE resolution time
    auto start = std::chrono::high_resolution_clock::now();
    // Solve the dynamic
    if(!getRobot()->sensiODE_.call(sensitivitySrv))
    {
        ROS_WARN("Unable to solve tracking.");
        return false;
    }
    else
    {
        // To compare the service call time with the ODE resolution time
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> srv_duration = end - start;
        
        // Get the simulated states
        std::vector<double> trajX = sensitivitySrv.response.trajX;
        std::vector<double> trajY = sensitivitySrv.response.trajY;
        std::vector<double> trajVX = sensitivitySrv.response.trajVX;
        std::vector<double> trajVY = sensitivitySrv.response.trajVY;
        std::vector<double> trajQw = sensitivitySrv.response.trajQw;
        std::vector<double> trajQx = sensitivitySrv.response.trajQx;
        std::vector<double> trajQy = sensitivitySrv.response.trajQy;
        std::vector<double> trajQz = sensitivitySrv.response.trajQz;
        std::vector<double> trajWz = sensitivitySrv.response.trajWz;

        for(int i = 0; i<trajX.size(); i++)
        {
            std::vector<double> st;
            st.push_back(trajX.at(i));
            st.push_back(trajY.at(i));
            st.push_back(0.0);
            st.push_back(trajQw.at(i));
            st.push_back(trajQx.at(i));
            st.push_back(trajQy.at(i));
            st.push_back(trajQz.at(i));
            collision_states.push_back(st);
        }

        // Get the simulated control inputs
        control_inputs.push_back(sensitivitySrv.response.u1);
        control_inputs.push_back(sensitivitySrv.response.u2);

        // Get the uncertainty tubes
        radii.push_back(sensitivitySrv.response.ellipsoid_alongX);
        radii.push_back(sensitivitySrv.response.ellipsoid_alongY);
        radii.push_back(sensitivitySrv.response.ellipsoid_u1);
        radii.push_back(sensitivitySrv.response.ellipsoid_u2);

        // The last element is the pnorm of the radii of interest
        radii.push_back(sensitivitySrv.response.radii_lambda);

        // Make sure there is is not already a registered state
        des_states.back()->as<StateType>()->nominal_state_.clear();
        des_states.back()->as<StateType>()->PI_.clear();
        des_states.back()->as<StateType>()->PI_xi_.clear();
        des_states.back()->as<StateType>()->xi_.clear();
        // Register the new nominal state
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajX.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajY.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajVX.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajVY.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQw.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQx.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQy.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(trajQz.back());
        des_states.back()->as<StateType>()->nominal_state_.push_back(0.0);
        // Register new sensitivity conditions
        des_states.back()->as<StateType>()->PI_ = sensitivitySrv.response.final_PI;
        des_states.back()->as<StateType>()->PI_xi_ = sensitivitySrv.response.final_PI_xi;

        des_states.back()->as<StateType>()->xi_ = sensitivitySrv.response.final_xi;

        // To compare the service call time with the ODE resolution time
        double srv_time_percentage = ((srv_duration.count()-sensitivitySrv.response.executionTime)/srv_duration.count())*100;
        // std::cout << "Loss of service call time in percent : " << srv_time_percentage << std::endl;
        return true;
    }
}

unsigned int ompl::base::DubinsUnicycleSpace::validSegmentCount(const State *state1, const State *state2) const
{
    return StateSpace::validSegmentCount(state1, state2);
}

ompl::base::DubinsUnicycleSpace::DubinsPath ompl::base::DubinsUnicycleSpace::dubins(const State *state1,
                                                                              const State *state2) const
{
    const auto *s1 = static_cast<const StateType *>(state1);
    const auto *s2 = static_cast<const StateType *>(state2);
    double x1 = s1->getX(), y1 = s1->getY(), th1 = s1->getYaw();
    double x2 = s2->getX(), y2 = s2->getY(), th2 = s2->getYaw();
    double dx = x2 - x1, dy = y2 - y1, d = sqrt(dx * dx + dy * dy) / rho_, th = atan2(dy, dx);
    double alpha = mod2pi(th1 - th), beta = mod2pi(th2 - th);
    return ::dubins(d, alpha, beta);
}