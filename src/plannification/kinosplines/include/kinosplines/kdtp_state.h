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

#ifndef KDTP_STATE_H
#define KDTP_STATE_H

#include "kinosplines/kdtp_robot.h"

namespace kdtp {

class State;
typedef std::tr1::shared_ptr<kdtp::State> StatePtr;

class State {

public:

    State(RobotPtr robot);

    std::vector<double> getPosition();
    double getPosition(unsigned int i);
    std::vector<double> getVelocity();
    double getVelocity(unsigned int i);
    std::vector<double> getAcceleration();
    double getAcceleration(unsigned int i);
    vector3 getDofState(unsigned int i);
    RobotPtr getRobot(){return _robot;}

    void setPosition(unsigned int i,double value);
    void setVelocity(unsigned int i,double value);
    void setAcceleration(unsigned int i,double value);

    void uniform_shoot();
    void incremental_shoot();

    double distance(StatePtr state);

    bool isOutOfBounds();

    void print();

private:
    RobotPtr _robot;
    std::vector<double> _position;
    std::vector<double> _velocity;
    std::vector<double> _acceleration;

};

typedef enum {
    A1,
    A2,
    B,
    C1,
    C2
} durations_index;

typedef enum {
    A,
    C
} signes_index;


class Data {

public:
  double v0;
  double a0;
  double amax;
  double jmax;
  double smax;
  double signs[2];
  double durations[5];
  double int_v[8];
  double int_a[8];
  int cases[8];

};

}

#endif // KDTP_STATE_H
