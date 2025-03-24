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

#include "utils/transform.h"

Eigen::Quaterniond eulerDeg2Quaternion(const double rollDeg, const double pitchDeg,
                                       const double yawDeg)
{
    tf2::Matrix3x3 m;
    m.setRPY(rollDeg*DEG2RAD, pitchDeg*DEG2RAD, yawDeg*DEG2RAD);

    tf2::Quaternion q2;
    m.getRotation(q2);

    Eigen::Quaterniond q;
    q.x() = q2.x();
    q.y() = q2.y();
    q.z() = q2.z();
    q.w() = q2.w();

    return q;
}


Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch,
                                    const double yaw)
{

    Eigen::Quaterniond q;

    q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())*
           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())*
           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());


    return q;
}

Eigen::Vector3d Quat2RPY(Eigen::Quaterniond quat){
    Eigen::Vector3d ypr = quat.toRotationMatrix().eulerAngles(2, 1, 0);
       return Eigen::Vector3d(ypr(2),ypr(1),ypr(0));
}

// Angles in radian
Eigen::Affine3d setAffine3d(double x, double y, double z, double roll, double pitch, double yaw)
{
    Eigen::Quaterniond quat = euler2Quaternion(roll, pitch, yaw);
    return Eigen::Translation3d(x, y, z)*quat;
}

double sign(double x)
{
    if(x >= 0)
        return 1.0;
    else
        return -1.0;
}

