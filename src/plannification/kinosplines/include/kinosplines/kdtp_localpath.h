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

#ifndef KDTP_LOCALPATH_H
#define KDTP_LOCALPATH_H

#include "kinosplines/kdtp_state.h"
#include "kinosplines/kdtp_spline.h"

namespace kdtp {

class LocalPath {

public:

    LocalPath(RobotPtr robot,StatePtr init,StatePtr end);

    double getDuration();
    unsigned int getNbSplines();
    std::vector<double> getPositionAt(double time);
    std::vector<double> getVelocityAt(double time);
    std::vector<double> getAccelerationAt(double time);
    std::vector<double> getJerkAt(double time);
    std::vector<double> getSnapAt(double time);
    std::vector<std::vector<double> > getAllAt(double time);
    StatePtr getStateAt(double time);
    void exportToFile(const char *filepath,double frequency,bool overwrite=true);
    void exportToMatlabFile(const char *filepath,double frequency);

private:

    void init();

    RobotPtr _robot;
    StatePtr _init;
    StatePtr _end;
    std::vector<SplinePtr> _splines;
    double _duration;

};

typedef std::tr1::shared_ptr<kdtp::LocalPath> LocalPathPtr;

}

#endif // KDTP_LOCALPATH_H
