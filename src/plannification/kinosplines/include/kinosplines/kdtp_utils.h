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

#ifndef KDTP_UTILS_H
#define KDTP_UTILS_H

#include <vector>
#include <stdio.h>
#include <cmath>
#include <tr1/memory>
#include <ctime>
#include <cstdlib>

namespace kdtp {

#define EPSILON 1e-10
#define EPSI4 1e-4
#define NB_IT_MAX 1e3
#define SQR(x) ((x)*(x))

double sign(double d);

extern double gen_rand(double a,double b);

std::vector<double> cardan(double p,double q);
std::vector<double> ferrari(double p,double q,double r);
std::vector<double> poly_root_2(double a,double b,double c);
std::vector<double> poly_root_3(double a,double b,double c,double d);
std::vector<double> poly_root_4(double a,double b,double c,double d,double e);
std::vector<double> realRoots(std::vector<double> coeff);

double mainMeasurement(double angle);

class vector3{
public:
    double x;
    double y;
    double z;
    double error;
    vector3();
    vector3(double _x,double _y,double _z);
    double& operator[](int index);
    vector3& operator=(vector3 u);
    vector3& operator+=(vector3 u);
    vector3& operator-=(vector3 u);
    vector3 operator+(vector3 u);
    vector3 operator-(vector3 u);
    vector3 operator-();
    vector3 operator*(double lambda);
    vector3 operator/(double lambda);
    double operator*(vector3 u);
    vector3 operator^(vector3 u);
    vector3 operator^(double p);
    double min();
    double max();
    double sum();
    vector3 min(vector3 u);
    vector3 max(vector3 u);
    vector3 fabs();
    vector3 sqrt();
    double norm();
    double norm(int n);
    double normInf();
    void normalize();
    void print();
};

}

#endif // KDTP_UTILS_H
