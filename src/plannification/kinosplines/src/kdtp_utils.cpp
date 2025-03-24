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

#include "kinosplines/kdtp_utils.h"

namespace kdtp {

/**
* Basic sign function.
*/
double sign(double d){
    if(std::fabs(d)<EPSILON)
        return 0;
    if(d<0)
        return -1;
    return  1;
}

/**
* Returns random double between a and b
*/
double gen_rand(double a,double b){
    if(fabs(a-b)<EPSILON)
        return a;
    double miN=a;
    double maX=b;
    if(a>b){
        miN=b;
        maX=a;
    }
    int D6p1=1e6+1;
    double ranD=(double)(rand() % D6p1)/1e6;
    return (maX-miN)*ranD+miN;
}

/**
* Computing real roots of a second order polynomial.
*
* Input
*    double a : coefficient of the second order
*    double b : coefficient of the first order
*    double c : constant term
*
* Output
*    std::vector<double> : real solutions a*X^2+b*X+c=0
*/
std::vector<double> poly_root_2(double a,double b,double c){
    std::vector<double> sol;
    if(fabs(a)<EPSILON){
        if(fabs(b)<EPSILON)
            sol.push_back(-c);
        else
            sol.push_back(-c/b);
    }
    else{
        double d=b*b-4*a*c;
        if(fabs(d)<EPSILON)
            sol.push_back(-b/(2*a));
        else if(d>0){
            sol.push_back((-b-sqrt(d))/(2*a));
            sol.push_back((-b+sqrt(d))/(2*a));
        }
    }
    return sol;
}

/**
* Implementation of Cardano's method.
*
* Input :
*    double p : coefficient of the first order
*    double q : constant term
*
* Output :
*    std::vector<double> : real solutions of X^3+p*X+q=0
*/
std::vector<double> cardan(double p,double q){
    std::vector<double> sol;
    double delta=-(4*p*p*p+27*q*q);
    if(fabs(delta)<EPSILON){
        if(fabs(p)<EPSILON || fabs(q)<EPSILON)
            sol.push_back(0);
        else{
            sol.push_back(3*q/p);
            sol.push_back(-3*q/(2*p));
        }
    }
    else if(delta<0){
        double v=sqrt(-delta/27)/2;
        double u=-q/2+v;
        u=sign(u)*pow(fabs(u),0.333333333333);
        v=-q/2-v;
        v=sign(v)*pow(fabs(v),0.333333333333);
        sol.push_back(u+v);
    }
    else{
        double u=0.333333333333*acos((-q/2)*sqrt(27/(-p*p*p)));
        double v=2*sqrt(-p/3);
        for(int k=0;k<3;k++){
            sol.push_back(v*cos(u+2*((double)k)*M_PI/3));
        }
    }
    return sol;
}

/**
* Computing real roots of a third order polynomial using Cardano's method.
*
* Input
*    double a : coefficient of the third order
*    double b : coefficient of the second order
*    double c : coefficient of the first order
*    double d : constant term
*
* Output
*    std::vector<double> : real solutions of a*X^3+b*X^2+c*X+d=0
*/
std::vector<double> poly_root_3(double a,double b,double c,double d){
    if(fabs(a)<EPSILON)
        return poly_root_2(b,c,d);
    double aa=b/a;
    double bb=c/a;
    double cc=d/a;
    double p=bb-aa*aa/3.;
    double q=2*aa*aa*aa/27.-aa*bb/3.+cc;
    std::vector<double> sol=cardan(p,q);
    for(unsigned int k=0;k<sol.size();k++){
        sol[k]=sol[k]-aa/3;
    }
    return sol;
}

/**
* Implementation of Ferrari's method.
*
* Input
*    double p : coefficient of the second order
*    double q : coefficient of the first order
*    double r : constant term
*
* Output
*    std::vector<double> : real solutions of X^4+p*X^2+q*X+r=0
*/
std::vector<double> ferrari(double p,double q,double r){
    std::vector<double> sol;
    if(fabs(q)<EPSILON){
        std::vector<double> soltmp=poly_root_2(1,p,r);
        for(unsigned k=0;k<soltmp.size();k++){
            if(soltmp.at(k)>=0){
                sol.push_back(-sqrt(soltmp.at(k)));
                sol.push_back( sqrt(soltmp.at(k)));
            }
        }
    }
    else{
        std::vector<double> soltmp=poly_root_3(1,-p/2,-r,p*r/2-q*q/8);
        double y0=soltmp.at(0);
        double delta=2*y0-p;
        if(delta<0)
            sol=poly_root_2(1,0,y0);
        else{
            double a0=sqrt(delta);
            double b0;
            if(a0<EPSILON){
                if(y0*y0-r>0)
                    b0=sqrt(y0*y0-r);
                else
                    b0=0;
            }
            else
                b0=-q/(2*a0);
            sol=poly_root_2(1,a0,y0+b0);
            soltmp=poly_root_2(1,-a0,y0-b0);
            for(unsigned int k=0;k<soltmp.size();k++){
                sol.push_back(soltmp.at(k));
            }
        }
    }
    return sol;
}

/**
* Computing real roots of a fourth order polynomial using Ferrari's method.
*
* Input
*    double a : coefficient of the fourth order
*    double b : coefficient of the third order
*    double c : coefficient of the second order
*    double d : coefficient of the first order
*    double e : constant term
*
* Output
*    std::vector<double> : real solutions of a*X^4+b*X^3+c*X^2+d*X+e=0
*/
std::vector<double> poly_root_4(double a,double b,double c,double d,double e){
    if(fabs(a)<EPSILON)
        return poly_root_3(b,c,d,e);
    double aa=b/a;
    double bb=c/a;
    double cc=d/a;
    double dd=e/a;
    double p=bb-3*aa*aa/8;
    double q=cc-aa*bb/2+aa*aa*aa/8;
    double r=dd-aa*cc/4+bb*aa*aa/16-3*aa*aa*aa*aa/256;
    std::vector<double> sol=ferrari(p,q,r);
    for(unsigned int k=0;k<sol.size();k++){
        sol[k]=sol[k]-aa/4;
    }
    return sol;
}

/**
* Computing real roots of a polynomial (up to the fourth order).
*
* Input
*    std::vector<double> coeff : coefficient of the polynomial
*
* Output
*    std::vector<double> : real solutions of sum(coeff.at(k)*X^(n-k))=0
*                          for n the degree of the polynomial and k=1..n
*/
std::vector<double> realRoots(std::vector<double> coeff){
    std::vector<double> sol;
    if(coeff.size()>5){
        printf("error: kdtp::realRoots: degree must be less than or egual to");
        printf("4\n");
    }
    else if(coeff.size()==5)
        sol=poly_root_4(coeff.at(0),
                        coeff.at(1),
                        coeff.at(2),
                        coeff.at(3),
                        coeff.at(4));
    else if(coeff.size()==4)
        sol=poly_root_3(coeff.at(0),
                        coeff.at(1),
                        coeff.at(2),
                        coeff.at(3));
    else if(coeff.size()==3)
        sol=poly_root_2(coeff.at(0),
                        coeff.at(1),
                        coeff.at(2));
    else if(coeff.size()==2)
        sol=poly_root_2(0,coeff.at(0),coeff.at(1));
    else{
        printf("error: kdtp::realRoots: degree must be less than or egual to");
        printf("1\n");
    }
    return sol;
}

/**
* Returns the main measurement of an angle
* (i.e. its value between -Pi and Pi)
*/
double mainMeasurement(double angle){
    int mod=sign(angle)*floor(fabs(angle)/(2*M_PI));
    double MM=angle-2*mod*M_PI;
    if(MM>M_PI)
        MM-=2*M_PI;
    if(MM<-M_PI)
        MM+=2*M_PI;
    return MM;
}

/********************************** VECTOR3 ***********************************/

vector3::vector3(){
    x=0;
    y=0;
    z=0;
    error=0;
}

vector3::vector3(double _x,double _y,double _z){
    x=_x;
    y=_y;
    z=_z;
    error=0;
}

double& vector3::operator[](int index){
    switch(index){
        case 0:
            return x;
            break;
        case 1:
            return y;
            break;
        case 2:
            return z;
            break;
        default:
            printf("error: kdtp::vector3::operator[]: out of range\n");
            return error;
    }
}

vector3& vector3::operator=(vector3 u){
    x=u.x;
    y=u.y;
    z=u.z;
    return *this;
}

vector3& vector3::operator+=(vector3 u){
    x+=u.x;
    y+=u.y;
    z+=u.z;
    return *this;
}

vector3& vector3::operator-=(vector3 u){
    x-=u.x;
    y-=u.y;
    z-=u.z;
    return *this;
}

vector3 vector3::operator+(vector3 u){
    return vector3(x+u.x,y+u.y,z+u.z);
}

vector3 vector3::operator-(vector3 u){
    return vector3(x-u.x,y-u.y,z-u.z);
}

vector3 vector3::operator-(){
    return vector3(-x,-y,-z);
}

vector3 vector3::operator*(double lambda){
    return vector3(lambda*x,lambda*y,lambda*z);
}

vector3 vector3::operator/(double lambda){
    return vector3(x/lambda,y/lambda,z/lambda);
}

double vector3::operator*(vector3 u){
    return x*u.x+y*u.y+z*u.z;
}

vector3 vector3::operator^(vector3 u){
    return vector3(y*u.z-z*u.y,z*u.x-x*u.z,x*u.y-y*u.x);
}

vector3 vector3::operator^(double p){
    return vector3(pow(x,p),pow(y,p),pow(z,p));
}

double vector3::min(){
    return std::min(x,std::min(y,z));
}

double vector3::max(){
    return std::max(x,std::max(y,z));
}

double vector3::sum(){
    return x+y+z;
}

vector3 vector3::min(vector3 u){
    return vector3(std::min(x,u.x),
                   std::min(y,u.y),
                   std::min(z,u.z));
}

vector3 vector3::max(vector3 u){
    return vector3(std::max(x,u.x),
                   std::max(y,u.y),
                   std::max(z,u.z));
}

vector3 vector3::fabs(){
    return vector3(std::fabs(x),
                   std::fabs(y),
                   std::fabs(z));
}

vector3 vector3::sqrt(){
    return vector3(std::sqrt(x),
                   std::sqrt(y),
                   std::sqrt(z));
}

double vector3::norm(){
    return std::sqrt((*this)*(*this));
}

double vector3::norm(int n){
    if(n<=0)
        return 1;
    double p=(double)n;
    double invp=1/p;
    return pow(((this->fabs())^p).sum(),invp);
}

double vector3::normInf(){
    return this->fabs().max();
}

void vector3::normalize(){
    (*this)=(*this)/this->norm();
}

void vector3::print(){
    printf("[ ");
    if(x>=0) printf(" ");
    printf("%f ",x);
    if(y>=0) printf(" ");
    printf("%f ",y);
    if(z>=0) printf(" ");
    printf("%f ]\n",z);
}

}
