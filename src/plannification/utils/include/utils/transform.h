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

/* Author: Hermes TELLO */

#ifndef THIRD_PARTIES_TRANSFORM_H
#define THIRD_PARTIES_TRANSFORM_H

#include <geometry_msgs/PoseStamped.h>
#include "maths_definitions.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <eigen3/Eigen/Dense>


Eigen::Quaterniond eulerDeg2Quaternion(const double rollDeg, const double pitchDeg,
                                       const double yawDeg);


Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch,
                                    const double yaw);


Eigen::Vector3d Quat2RPY(Eigen::Quaterniond quat);

// Angles in radian
Eigen::Affine3d setAffine3d(double x, double y, double z, double roll, double pitch, double yaw);

double sign(double x);

#endif // TRANSFORM_H
