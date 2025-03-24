/*BSD 2-Clause License

Copyright (c) 2018, LAAS-CNRS
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Author: Alexandre Boeuf */

#ifndef KINO_DYNAMIC_TIME_H
#define KINO_DYNAMIC_TIME_H

#include "kinosplines/kdtp_utils.h"

#include <iostream>

/** \brief
* Kinodynamic metric in one dimension
*
* Returns the duration of the minimum-time trajectory for a trapezoidal
* acceleration with and a given maximum allowed jerk.
*
* Inputs
*
*    std::vector<double> q0 : the one dimensional initial state
*                q0 = { initial_position, initial_velocity, initial_acceleration }
*
*    std::vector<double> qF : the one dimensional final state
*                qF = { final_position, final_velocity, final_acceleration }
*
*    double maximumJerk : maximum allowed jerk
*
* Outputs
*
*    double tmin : duration of the minimum-time trajectory
*
*/
double kinoMetricOneComp(std::vector<double> &q0, std::vector<double> &qT, double maximumJerk);

/**
* Kinodynamic metric in n dimension
*
* Returns the maximum duration of all the minimum-time one dimensional trajectories
*
* Inputs
*
*    std::vector<std::vector<double> > q0 : the n dimensional initial state
*                q0 = { { initial_position_1, initial_velocity_1, initial_acceleration_1 },
*                       { initial_position_2, initial_velocity_2, initial_acceleration_2 },
*                                                   ...
*                       { initial_position_n, initial_velocity_n, initial_acceleration_n } }
*
*    std::vector<std::vector<double> > qT : the n dimensional final state
*                q0 = { { final_position_1 final_velocity_1 final_acceleration_1 },
*                       { final_position_2 final_velocity_2 final_acceleration_2 },
*                                                   ...
*                       { initial_position_n initial_velocity_n initial_acceleration_n } }
*
*    double maximumJerk : maximum allowed jerk vector for all the component of the state (i.e: x_jmax, y_jmax, z_jmax, roll_jmax, etc.)
*
* Outputs
*
*    double tmax : the metric
*
*/
double kinoMetric(std::vector<std::vector<double>> &q0, std::vector<std::vector<double>> &qT, std::vector<double> &maximumJerk);

#endif
