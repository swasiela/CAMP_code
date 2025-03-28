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

#ifndef KDTP_DOF_H
#define KDTP_DOF_H

#include "kinosplines/kdtp_utils.h"


namespace kdtp {

class Dof {

public:

    Dof(double xmin,
        double xmax,
        double vmax,
        double amax,
        double jmax,
        double smax,
        bool rotation);

    double getPositionMin();
    double getPositionMax();
    double getVelocityMax();
    double getAccelerationMax();
    double getJerkMax();
    double getSnapMax();
    bool isRotation();

    void setPositionMin(double xmin);
    void setPositionMax(double xmax);
    void setVelocityMax(double vmax);
    void setAccelerationMax(double amax);
    void setJerkMax(double jmax);
    void setSnapMax(double smax);

private:

    double _xmin;
    double _xmax;
    double _vmax;
    double _amax;
    double _jmax;
    double _smax;
    bool _rotation;

};

typedef std::tr1::shared_ptr<kdtp::Dof> DofPtr;

}

#endif // KDTP_DOF_H
