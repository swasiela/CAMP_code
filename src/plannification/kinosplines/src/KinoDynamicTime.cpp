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

#include "kinosplines/KinoDynamicTime.h"

/** \brief Constante used to filter double approximation around zero */
const double DOUBLE_PRECISION_ = 0.0;

double kinoMetricOneComp(std::vector<double> &q0, std::vector<double> &qT, double maximumJerk)
{
    if(q0.size()!=3)
    {
        std::cout << "ERROR: kinoMetricOneComp: q0 must be of size 3" << std::endl;
        return -1;
    }

    if(qT.size()!=3)
    {
        std::cout << "ERROR: kinoMetricOneComp: qT must be of size 3" << std::endl;
        return -1;
    }

    double umax = fabs(maximumJerk);
    double tmin = HUGE_VAL;

    double x0 = q0[0];
    double v0 = q0[1];
    double a0 = q0[2];

    double xT = qT[0];
    double vT = qT[1];
    double aT = qT[2];

    // ZERO SWITCHING
    double u = umax;
    double T = (aT-a0)/u;
    if(T < 0)
    {
        T=-T;
        u=-u;
    }
    double v = u*T*T/2 + a0*T + v0;
    if(fabs(v-vT)<1e-5)
    {
        double x=u*T*T*T/6+a0*T*T/2+v0*T+x0;
        if(fabs(x-xT)<1e-5 && tmin>T)
            tmin=T;
    }

    // ONE SWITCHING
    for(double u=-umax; u<=umax; u+=2*umax)
    {
        std::vector<double> coeffs = { u, 2*a0, (a0*a0-aT*aT)/(2*u)+(v0-vT) };
        std::vector<double> sol = kdtp::realRoots(coeffs);
        for(unsigned int i=0; i<sol.size(); i++)
        {
            double t1 = sol[i];
            if(t1 > DOUBLE_PRECISION_)
            {
                T = 2*t1 + (a0-aT)/u;
                if(T > t1)
                {
                    double x=-u*T*T*T/6+u*T*T*t1-u*T*t1*t1+u*t1*t1*t1/3+a0*T*T+v0*T+x0;
                    if(fabs(x-xT)<1e-5 && tmin>T)
                        tmin = T;
                }
            }
        }
    }

    // TWO SWITCHINGS
    for(double u=-umax; u<=umax; u+=2*umax)
    {
        std::vector<double> coeffs = { u, 0, 2*(2*(v0+vT)-(a0*a0+aT*aT)/u),
                            4*(x0-xT-(a0*v0-aT*vT)/u+(a0*a0*a0-aT*aT*aT)/(3*u*u))};
        double tmp=aT*aT-a0*a0+2*u*(v0-vT);
        coeffs.push_back(-tmp*tmp/(4*u*u*u));
        std::vector<double> sol = kdtp::realRoots(coeffs);
        for(unsigned int i=0; i<sol.size(); i++)
        {
            double t2 = sol[i];
            if(t2 > DOUBLE_PRECISION_)
            {
                double t1 = (2*u*u*t2*t2-4*u*t2*a0+a0*a0-aT*aT+2*u*(vT-v0))/(4*u*u*t2);
                if(t1 > DOUBLE_PRECISION_)
                {
                    T = 2*t2+(aT-a0)/u;
                    if( T>t1+t2 && tmin>T )
                        tmin=T;
                }
            }
        }
    }

    if( tmin == HUGE_VAL)
    {
        tmin = -1;
        //std::cout << "ERROR: kinoMetricOneComp: no solutions" << std::endl;
    }

    return tmin;
}

double kinoMetric(std::vector<std::vector<double>> &q0, std::vector<std::vector<double>> &qT, std::vector<double> &maximumJerk)
{
    if(q0.size() != qT.size())
    {
        std::cout << "ERROR: kinoMetric: q0 and qT must be of same size" << std::endl;
        return HUGE_VAL;
    }

    double tmax = -0.5;

    for(unsigned int i=0; i<q0.size(); i++)
    {
        if(maximumJerk.at(i)==0.0)
            continue;
        double t = kinoMetricOneComp(q0[i], qT[i], maximumJerk.at(i));
        if(t>tmax)
            tmax=t;
    }

    if(tmax<0)
    {
        tmax = HUGE_VAL;
        // std::cout << "ERROR: kinoMetricOneComp: no solutions (should not happen !!!)" << std::endl;
    }

    return tmax;
}