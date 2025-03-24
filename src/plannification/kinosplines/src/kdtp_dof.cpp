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


#include "kinosplines/kdtp_dof.h"

namespace kdtp {

Dof::Dof(double xmin,
         double xmax,
         double vmax,
         double amax,
         double jmax,
         double smax,
         bool rotation){
    _rotation=rotation;
    _vmax=fabs(vmax);
    _amax=fabs(amax);
    _jmax=fabs(jmax);
    _smax=fabs(smax);
    _xmin=xmin;
    _xmax=xmax;
    if(_rotation){
        _xmin=mainMeasurement(_xmin);
        _xmax=mainMeasurement(_xmax);
    }
    if(_xmax<_xmin){
        double tmp=_xmin;
        _xmin=_xmax;
        _xmax=tmp;
    }
}

double Dof::getPositionMin(){
    return _xmin;
}

double Dof::getPositionMax(){
    return _xmax;
}

double Dof::getVelocityMax(){
    return _vmax;
}

double Dof::getAccelerationMax(){
    return _amax;
}

double Dof::getJerkMax(){
    return _jmax;
}

double Dof::getSnapMax(){
    return _smax;
}

bool Dof::isRotation(){
    return _rotation;
}

void Dof::setPositionMin(double xmin){
    _xmin=xmin;
    if(_rotation)
        _xmin=mainMeasurement(xmin);
    if(_xmax<_xmin){
        double tmp=_xmin;
        _xmin=_xmax;
        _xmax=tmp;
    }
}

void Dof::setPositionMax(double xmax){
    _xmax=xmax;
    if(_rotation)
        _xmax=mainMeasurement(xmax);
    if(_xmax<_xmin){
        double tmp=_xmin;
        _xmin=_xmax;
        _xmax=tmp;
    }
}

void Dof::setVelocityMax(double vmax){
    _vmax=fabs(vmax);
}

void Dof::setAccelerationMax(double amax){
    _amax=fabs(amax);
}

void Dof::setJerkMax(double jmax){
    _jmax=fabs(jmax);
}

void Dof::setSnapMax(double smax){
    _smax=fabs(smax);
}

}
