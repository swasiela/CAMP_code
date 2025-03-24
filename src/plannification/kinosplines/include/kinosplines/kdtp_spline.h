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

#ifndef KDTP_SPLINE_H
#define KDTP_SPLINE_H

#include "kinosplines/kdtp_dof.h"

namespace kdtp {

class Spline{

public:
    Spline(DofPtr dof,double init,double end);
    Spline(DofPtr dof,vector3 init,vector3 end);

    struct ParamKinoSpline {
        std::vector<double> times;
        std::vector<double> positions;
        std::vector<double> velocities;
        std::vector<double> accelerations;
        std::vector<double> jerks;
        std::vector<double> snaps;
    };

private:

    int case_abc(double aB);
    int case_egh(double aG);
    void durations_and_signs_ac(double aB);
    void durations_and_signs_eh(double aG);
    double x_c(double aB,double new_tB,bool use_old);
    double x_e(double aG,double new_tG,bool use_old);
    double v_c(double aB,double new_tB,bool use_old);
    double v_e(double aG,double new_tG,bool use_old);
    void intervals_ac();
    void intervals_eh();
    double a_b(double vC);
    double a_g(double vE);
    double d_x(double vD);
    void v_optim();

    void pushNext(double time,std::vector<double> next);
    void clearValues();
    void setValues();

    unsigned int getIndexAt(double time);
    double getPositionAtLocal(unsigned int index,double local);
    double getPositionAt(unsigned int index,double time);
    double getVelocityAtLocal(unsigned int index,double local);
    double getVelocityAt(unsigned int index,double time);
    double getAccelerationAtLocal(unsigned int index,double local);
    double getAccelerationAt(unsigned int index,double time);
    double getJerkAtLocal(unsigned int index,double local);
    double getJerkAt(unsigned int index,double time);
    vector3 getStateAt(unsigned int index,double time);
    std::vector<double> getAllAt(unsigned int index,double time);

public:

    double getDuration();
    void synchronize(double tF_des);

    double getVelocityMax();
    double getAccelerationMax();
    double getJerkMax();
    double getSnapMax();

    double getPositionAt(double time);
    double getVelocityAt(double time);
    double getAccelerationAt(double time);
    double getJerkAt(double time);
    double getSnapAt(double time);
    vector3 getStateAt(double time);
    std::vector<double> getAllAt(double time);
    std::vector<double> getTimeVector();

    vector3 getInit();
    vector3 getEnd();

private:

    typedef enum {
        A1,
        A2,
        B,
        C1,
        C2,
        D,
        E1,
        E2,
        G,
        H1,
        H2,
        F
    } durations_index;

    typedef enum {
        A,
        C,
        E,
        H
    } signs_index;

    DofPtr _dof;
    unsigned int _last_call;

    vector3 _init;
    vector3 _end;

    std::vector<double> _times;
    std::vector<double> _positions;
    std::vector<double> _velocities;
    std::vector<double> _accelerations;
    std::vector<double> _jerks;
    std::vector<double> _snaps;

    double _signs[4];
    double _durations[12];

    std::vector<int> _cases_abc;
    std::vector<double> _int_v_abc;
    std::vector<double> _int_a_abc;

    std::vector<int> _cases_egh;
    std::vector<double> _int_v_egh;
    std::vector<double> _int_a_egh;

    double _v_opt;

    bool _stay_still;

};

typedef std::tr1::shared_ptr<kdtp::Spline> SplinePtr;

}
#endif // KDTP_SPLINE_H
