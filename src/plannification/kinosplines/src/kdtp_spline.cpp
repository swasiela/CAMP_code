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

#include "kinosplines/kdtp_spline.h"

namespace kdtp{

Spline::Spline(DofPtr dof,double init,double end){

    if(!dof){
        printf("error: kdtp::in Spline constructor: dof parameter is null\n");
        return;
    }

    _dof=dof;
    _last_call=0;
    if(_dof->isRotation()){
        _init=vector3(mainMeasurement(init),0,0);
        _end=vector3(mainMeasurement(end),0,0);
    }
    else{
        _init=vector3(init,0,0);
        _end=vector3(end,0,0);
    }

    if(_init.x<_dof->getPositionMin())
        _init.x=_dof->getPositionMin();
    if(_end.x<_dof->getPositionMin())
        _end.x=_dof->getPositionMin();
    if(_init.x>_dof->getPositionMax())
        _init.x=_dof->getPositionMax();
    if(_end.x>_dof->getPositionMax())
        _end.x=_dof->getPositionMax();

    if(_dof->isRotation() && fabs(_init.x-_end.x)>M_PI)
            _end.x-=2*sign(_end.x)*M_PI;

    _stay_still=(fabs(_init.x-_end.x)<EPSILON);

    if(!_stay_still)
        this->v_optim();
}


Spline::Spline(DofPtr dof,vector3 init,vector3 end){

    if(!dof){
        printf("error: kdtp::Spline::Spline: dof is a null\n");
        return;
    }

    _dof=dof;
    _last_call=0;
    _init=vector3(init.x,init.y,init.z);
    _end=vector3(end.x,end.y,end.z);
    if(_dof->isRotation()){
        _init.x=mainMeasurement(_init.x);
        _end.x=mainMeasurement(_end.x);
    }

    if(_init.x<_dof->getPositionMin())
        _init.x=_dof->getPositionMin();
    if(_end.x<_dof->getPositionMin())
        _end.x=_dof->getPositionMin();
    if(_init.x>_dof->getPositionMax())
        _init.x=_dof->getPositionMax();
    if(_end.x>_dof->getPositionMax())
        _end.x=_dof->getPositionMax();

    if(fabs(_init.x-_end.x)>M_PI && _dof->isRotation())
        _end.x-=2*sign(_end.x)*M_PI;

    if(fabs(_init.y)>_dof->getVelocityMax())
        _init.y=sign(_init.y)*_dof->getVelocityMax();
    if(fabs(_end.y)>_dof->getVelocityMax())
        _end.y=sign(_end.y)*_dof->getVelocityMax();

    if(fabs(_init.z)>_dof->getAccelerationMax())
        _init.z=sign(_init.z)*_dof->getAccelerationMax();
    if(fabs(_end.z)>_dof->getAccelerationMax())
        _end.z=sign(_end.z)*_dof->getAccelerationMax();

    _stay_still=( fabs(_init.x-_end.x)<EPSILON
                  && fabs(_init.y)<EPSILON
                  && fabs(_end.y)<EPSILON
                  && fabs(_init.z)<EPSILON
                  && fabs(_end.z)<EPSILON);

    if(!_stay_still)
        this->v_optim();
}

std::vector<double> Spline::getTimeVector()
{
    return _times;
}

/* PRIVATE */

/**
* Returns the value of case_ABC for a given value of aB.
* Input:
*    double aB : acceleration during phase B
* Output:
*    int : case code
*/
int Spline::case_abc(double aB){
    double a0=_init[2];
    double jmax=this->getJerkMax();
    double smax=this->getSnapMax();
    int sA=(int)sign(aB-a0);
    int sC=-(int)sign(aB);
    double aj=jmax*jmax/smax;
    int cond_A=(fabs(a0-aB)>aj);
    int cond_C=(fabs(aB)>aj);
    int case_ABC=1000*(sA+1)+100*(sC+1)+10*cond_A+cond_C;
    return case_ABC;
}

/**
* Returns the value of case_EGH for a given value of aG.
* Input:
*    double aG : acceleration during phase G
* Output:
*    int : case code
*/
int Spline::case_egh(double aG){
    double aF=_end[2];
    double jmax=this->getJerkMax();
    double smax=this->getSnapMax();
    int sH=(int)sign(aF-aG);
    int sE=(int)sign(aG);
    double aj=jmax*jmax/smax;
    int cond_H=(fabs(aG-aF)>aj);
    int cond_E=(fabs(aG)>aj);
    int case_EGH=1000*(sH+1)+100*(sE+1)+10*cond_H+cond_E;
    return case_EGH;
}

/**
* Sets durations of phases A1, A2, A3, C1, C2, C3 into the _durations attribute
* and signs of the snap during phases A1 and C1 into the _signs attribute.
* Input:
*    double aB : acceleration during phase B
*/
void Spline::durations_and_signs_ac(double aB){
    double a0=_init[2];
    double jmax=this->getJerkMax();
    double smax=this->getSnapMax();
    double aj=jmax*jmax/smax;
    _signs[A]=sign(aB-a0);
    _signs[C]=-sign(aB);
    if(fabs(aB-a0)>aj){
        _durations[A1]=jmax/smax;
        _durations[A2]=fabs(aB-a0)/jmax-jmax/smax;
    }
    else{
        _durations[A1]=sqrt(fabs(a0-aB)/smax);
        _durations[A2]=0;
    }
    if(fabs(aB)>aj){
        _durations[C1]=jmax/smax;
        _durations[C2]=fabs(aB)/jmax-jmax/smax;
    }
    else{
        _durations[C1]=sqrt(fabs(aB)/smax);
        _durations[C2]=0;
    }
}

/**
* Sets durations of phases E1, E2, E3, H1, H2, H3 into the _durations attribute
* and signs of the snap during phases E1 and H1 into the _signs attribute.
* Input:
*    double aG : acceleration during phase G
*/
void Spline::durations_and_signs_eh(double aG){
    double aF=_end[2];
    double jmax=this->getJerkMax();
    double smax=this->getSnapMax();
    double aj=jmax*jmax/smax;
    _signs[H]=sign(aF-aG);
    _signs[E]=sign(aG);
    if(fabs(aF-aG)>aj){
        _durations[H1]=jmax/smax;
        _durations[H2]=fabs(aF-aG)/jmax-jmax/smax;
    }
    else{
        _durations[H1]=sqrt(fabs(aF-aG)/smax);
        _durations[H2]=0;
    }
    if(fabs(aG)>aj){
        _durations[E1]=jmax/smax;
        _durations[E2]=fabs(aG)/jmax-jmax/smax;
    }
    else{
        _durations[E1]=sqrt(fabs(aG)/smax);
        _durations[E2]=0;
    }
}

/**
* Calculates position at the end of phase C.
* Inputs:
*    double aB     : acceleration during phase B
*    double new_tB : an updated value of the duration of phase B
*    bool use_old  : indicates if the given value of tB is to be used
* Output:
*    double : position at the end of phase C
*/
double Spline::x_c(double aB,double new_tB,bool use_old){
    if(!use_old){
        _durations[B]=new_tB;
        durations_and_signs_ac(aB);
    }
    double tA1=_durations[A1];
    double tA2=_durations[A2];
    double tB=_durations[B];
    double tC1=_durations[C1];
    double tC2=_durations[C2];
    double sA=_signs[A];
    double sC=_signs[C];
    double x0=_init[0];
    double v0=_init[1];
    double a0=_init[2];
    double smax=this->getSnapMax();
    double tA1_2=tA1*tA1;
    double tA1_3=tA1_2*tA1;
    double tA1_4=tA1_3*tA1;
    double tA2_2=tA2*tA2;
    double tA2_3=tA2_2*tA2;
    double tB_2=tB*tB;
    double tC1_2=tC1*tC1;
    double tC1_3=tC1_2*tC1;
    double tC1_4=tC1_3*tC1;
    double tC2_2=tC2*tC2;
    double tC2_3=tC2_2*tC2;
    return (7*sA*smax*tA1_4)/12+(7*sA*smax*tA1_3*tA2)/6+sA*smax*tA1_3*tB
           +2*sA*smax*tA1_3*tC1+sA*smax*tA1_3*tC2+(3*sA*smax*tA1_2*tA2_2)/4
           +(3*sA*smax*tA1_2*tA2*tB)/2+3*sA*smax*tA1_2*tA2*tC1
           +(3*sA*smax*tA1_2*tA2*tC2)/2+(sA*smax*tA1_2*tB_2)/2
           +2*sA*smax*tA1_2*tB*tC1+sA*smax*tA1_2*tB*tC2+2*sA*smax*tA1_2*tC1_2
           +2*sA*smax*tA1_2*tC1*tC2+(sA*smax*tA1_2*tC2_2)/2+2*a0*tA1_2
           +(sA*smax*tA1*tA2_3)/6+(sA*smax*tA1*tA2_2*tB)/2
           +sA*smax*tA1*tA2_2*tC1+(sA*smax*tA1*tA2_2*tC2)/2
           +(sA*smax*tA1*tA2*tB_2)/2+2*sA*smax*tA1*tA2*tB*tC1
           +sA*smax*tA1*tA2*tB*tC2+2*sA*smax*tA1*tA2*tC1_2
           +2*sA*smax*tA1*tA2*tC1*tC2+(sA*smax*tA1*tA2*tC2_2)/2
           +2*a0*tA1*tA2+2*a0*tA1*tB+4*a0*tA1*tC1+2*a0*tA1*tC2
           +2*v0*tA1+(a0*tA2_2)/2+a0*tA2*tB+2*a0*tA2*tC1+a0*tA2*tC2+v0*tA2
           +(a0*tB_2)/2+2*a0*tB*tC1+a0*tB*tC2+v0*tB+(7*sC*smax*tC1_4)/12
           +(7*sC*smax*tC1_3*tC2)/6+(3*sC*smax*tC1_2*tC2_2)/4+2*a0*tC1_2
           +(sC*smax*tC1*tC2_3)/6+2*a0*tC1*tC2+2*v0*tC1+(a0*tC2_2)/2+v0*tC2+x0;
}

/**
* Calculates position at the beginning of phase E.
* Inputs:
*    double aG     : acceleration during phase G
*    double new_tG : an updated value of the duration of phase G
*    bool use_old  : indicates if the given value of tG is to be used
* Output:
*    double : position at the beginning of phase E.
*/
double Spline::x_e(double aG,double new_tG,bool use_old){
    if(!use_old){
        _durations[G]=new_tG;
        durations_and_signs_eh(aG);
    }
    double tE1=_durations[E1];
    double tE2=_durations[E2];
    double tG=_durations[G];
    double tH1=_durations[H1];
    double tH2=_durations[H2];
    double sE=_signs[E];
    double sH=_signs[H];
    double xF=_end[0];
    double vF=_end[1];
    double aF=_end[2];
    double smax=this->getSnapMax();
    double tE1_2=tE1*tE1;
    double tE1_3=tE1_2*tE1;
    double tE1_4=tE1_3*tE1;
    double tE2_2=tE2*tE2;
    double tE2_3=tE2_2*tE2;
    double tG_2=tG*tG;
    double tH1_2=tH1*tH1;
    double tH1_3=tH1_2*tH1;
    double tH1_4=tH1_3*tH1;
    double tH2_2=tH2*tH2;
    double tH2_3=tH2_2*tH2;
    return -(7*sE*smax*tE1_4)/12-(7*sE*smax*tE1_3*tE2)/6
           -(3*sE*smax*tE1_2*tE2_2)/4-2*sH*smax*tE1_2*tH1_2
           -2*sH*smax*tE1_2*tH1*tH2+2*aF*tE1_2-(sE*smax*tE1*tE2_3)/6
           -2*sH*smax*tE1*tE2*tH1_2-2*sH*smax*tE1*tE2*tH1*tH2+2*aF*tE1*tE2
           -2*sH*smax*tE1*tG*tH1_2-2*sH*smax*tE1*tG*tH1*tH2+2*aF*tE1*tG
           -2*sH*smax*tE1*tH1_3-3*sH*smax*tE1*tH1_2*tH2-sH*smax*tE1*tH1*tH2_2
           +4*aF*tE1*tH1+2*aF*tE1*tH2-2*vF*tE1-(sH*smax*tE2_2*tH1_2)/2
           -(sH*smax*tE2_2*tH1*tH2)/2+(aF*tE2_2)/2-sH*smax*tE2*tG*tH1_2
           -sH*smax*tE2*tG*tH1*tH2+aF*tE2*tG-sH*smax*tE2*tH1_3
           -(3*sH*smax*tE2*tH1_2*tH2)/2-(sH*smax*tE2*tH1*tH2_2)/2+2*aF*tE2*tH1
           +aF*tE2*tH2-vF*tE2-(sH*smax*tG_2*tH1_2)/2-(sH*smax*tG_2*tH1*tH2)/2
           +(aF*tG_2)/2-sH*smax*tG*tH1_3-(3*sH*smax*tG*tH1_2*tH2)/2
           -(sH*smax*tG*tH1*tH2_2)/2+2*aF*tG*tH1+aF*tG*tH2-vF*tG
           -(7*sH*smax*tH1_4)/12-(7*sH*smax*tH1_3*tH2)/6
           -(3*sH*smax*tH1_2*tH2_2)/4+2*aF*tH1_2-(sH*smax*tH1*tH2_3)/6
           +2*aF*tH1*tH2-2*vF*tH1+(aF*tH2_2)/2-vF*tH2+xF;
}

/**
* Calculates velocity at the end of phase C.
* Inputs:
*    double aB     : acceleration during phase B
*    double new_tB : an updated value of the duration of phase B
*    bool use_old  : indicates if the given value of tB is to be used
* Output:
*    double : velocity at the end of phase C
*/
double Spline::v_c(double aB,double new_tB,bool use_old){
    if(!use_old){
        _durations[B]=new_tB;
        durations_and_signs_ac(aB);
    }
    double tA1=_durations[A1];
    double tA2=_durations[A2];
    double tB=_durations[B];
    double tC1=_durations[C1];
    double tC2=_durations[C2];
    double sA=_signs[A];
    double sC=_signs[C];
    double v0=_init[1];
    double a0=_init[2];
    double smax=this->getSnapMax();
    double tA1_2=tA1*tA1;
    double tA1_3=tA1_2*tA1;
    double tA2_2=tA2*tA2;
    double tC1_2=tC1*tC1;
    double tC1_3=tC1_2*tC1;
    double tC2_2=tC2*tC2;
    return sA*smax*tA1_3+(3*sA*smax*tA1_2*tA2)/2+2*sA*smax*tA1_2*tC1
           +sA*smax*tA1_2*tC2+sA*smax*tB*tA1_2+(sA*smax*tA1*tA2_2)/2
           +2*sA*smax*tA1*tA2*tC1+sA*smax*tA1*tA2*tC2+sA*smax*tB*tA1*tA2
           +2*a0*tA1+a0*tA2+sC*smax*tC1_3+(3*sC*smax*tC1_2*tC2)/2
           +(sC*smax*tC1*tC2_2)/2+2*a0*tC1+a0*tC2+v0+a0*tB;;
}

/**
* Calculates velocity at the beginning of phase E.
* Inputs:
*    double aG     : acceleration during phase G
*    double new_tG : an updated value of the duration of phase G
*    bool use_old  : indicates if the given value of tG is to be used
* Output:
*    double : velocity at the beginning of phase E.
*/
double Spline::v_e(double aG,double new_tG,bool use_old){
    if(!use_old){
        _durations[G]=new_tG;
        durations_and_signs_eh(aG);
    }
    double tE1=_durations[E1];
    double tE2=_durations[E2];
    double tG=_durations[G];
    double tH1=_durations[H1];
    double tH2=_durations[H2];
    double sE=_signs[E];
    double sH=_signs[H];
    double vF=_end[1];
    double aF=_end[2];
    double smax=this->getSnapMax();
    double tE1_2=tE1*tE1;
    double tE1_3=tE1_2*tE1;
    double tE2_2=tE2*tE2;
    double tH1_2=tH1*tH1;
    double tH1_3=tH1_2*tH1;
    double tH2_2=tH2*tH2;
    return sE*smax*tE1_3+(3*sE*smax*tE1_2*tE2)/2+(sE*smax*tE1*tE2_2)/2
           +2*sH*smax*tE1*tH1_2+2*sH*smax*tE1*tH1*tH2-2*aF*tE1
           +sH*smax*tE2*tH1_2+sH*smax*tE2*tH1*tH2-aF*tE2+sH*smax*tH1_3
           +(3*sH*smax*tH1_2*tH2)/2+sH*smax*tG*tH1_2+(sH*smax*tH1*tH2_2)/2
           +sH*smax*tG*tH1*tH2-2*aF*tH1-aF*tH2+vF-aF*tG;
}

/**
* Sets the ordered acceleration intervals for phases A to C into _int_a_abc,
* the corresponding values of velocity into _int_v_abc and the corresponding
* cases codes into _cases_abc.
*/
void Spline::intervals_ac(){
    double amax=this->getAccelerationMax();
    double jmax=this->getJerkMax();
    double smax=this->getSnapMax();
    double aj=jmax*jmax/smax;
    double a0=_init[2];
    double Ac[7]={amax,0,aj,-aj,a0,a0+aj,a0-aj};
    double tmp[8];
    tmp[0]=-amax;
    int n=1;
    for(int k=0;k<7;k++){
        if((Ac[k]*a0<=0 || fabs(Ac[k])>=fabs(a0)) && fabs(Ac[k])<=amax){
            int l=0;
            while(l<n && Ac[k]>tmp[l]){
                l++;
            }
            if(l==n || fabs(Ac[k]-tmp[l])>EPSILON){
                for(int i=n+1;i>l;i--){
                    tmp[i]=tmp[i-1];
                }
                tmp[l]=Ac[k];
                n++;
            }
        }
    }
    n--;
    _int_a_abc.push_back(tmp[0]);
    _int_v_abc.push_back(v_c(tmp[0],0,false));
    for(int k=1;k<=n;k++){
        double a1=tmp[k-1];
        double a2=tmp[k];
        _int_a_abc.push_back(a2);
        _int_v_abc.push_back(v_c(a2,0,false));
        _cases_abc.push_back(case_abc((a1+a2)/2));
    }
}

/**
* Sets the ordered acceleration intervals for phases E to H into _int_a_egh,
* the corresponding values of velocity into _int_v_egh and the corresponding
* cases codes into _cases_egh.
*/
void Spline::intervals_eh(){
    double amax=this->getAccelerationMax();
    double jmax=this->getJerkMax();
    double smax=this->getSnapMax();
    double aj=jmax*jmax/smax;
    double aF=_end[2];
    double Ac[7]={amax,0,aj,-aj,aF,aF+aj,aF-aj};
    double tmp[8];
    tmp[0]=-amax;
    int n=1;
    for(int k=0;k<7;k++){
        if((Ac[k]*aF<=0 || fabs(Ac[k])>=fabs(aF)) && fabs(Ac[k])<=amax){
            int l=0;
            while(l<n && Ac[k]<tmp[l]){
                l++;
            }
            if(l==n || fabs(Ac[k]-tmp[l])>EPSILON){
                for(int i=n+1;i>l;i--){
                    tmp[i]=tmp[i-1];
                }
                tmp[l]=Ac[k];
                n++;
            }
        }
    }
    n--;
    _int_a_egh.push_back(tmp[0]);
    _int_v_egh.push_back(v_e(tmp[0],0,false));
    for(int k=1;k<=n;k++){
        double a1=tmp[k-1];
        double a2=tmp[k];
        _int_a_egh.push_back(a2);
        _int_v_egh.push_back(v_e(a2,0,false));
        _cases_egh.push_back(case_egh((a1+a2)/2));
    }
}

/**
* Returns acceleration during phase B and sets corresponding duration of phase B
* into _durations for a given desired velocity at the end of phase C.
* Input:
*    double vC : desired velocity at the end of phase C
* Output:
*    double : acceleration during phase B
*/
double Spline::a_b(double vC){

    double v0=_init[1];
    double a0=_init[2];
    double a0_2=a0*a0;
    double a0_3=a0_2*a0;
    double amax=this->getAccelerationMax();
    double jmax=this->getJerkMax();
    double jmax_2=jmax*jmax;
    double jmax_4=jmax_2*jmax_2;
    double smax=this->getSnapMax();
    double smax_2=smax*smax;

    if(_cases_abc.empty()){
        printf("error: kdtp::Spline::a_b: _cases_abc is not initialized\n");
        return 0;
    }

    if(_int_v_abc.size()!=_cases_abc.size()+1){
        printf("error: kdtp::Spline::a_b: _int_v_abc is not initialized\n");
        return 0;
    }

    if(_int_a_abc.size()!=_cases_abc.size()+1){
        printf("error: kdtp::Spline::a_b: _int_a_abc is not initialized\n");
        return 0;
    }

    if(vC<_int_v_abc.at(0)){
        _durations[B]=(_int_v_abc.at(0)-vC)/amax;
        return -amax;
    }

    if(vC>=_int_v_abc.at(_cases_abc.size())){
        _durations[B]=(vC-_int_v_abc.at(_cases_abc.size()))/amax;
        return amax;
    }

    _durations[B]=0;
    unsigned int index_int=0;
    while(index_int<_cases_abc.size() && vC>_int_v_abc.at(index_int+1)){
        index_int++;
    }
    if(index_int==_cases_abc.size()){
        printf("error: kdtp::Spline::a_b: vC not found (vC=%f)\n",vC);
        return a0;
    }
    int case_ABC=_cases_abc.at(index_int);
    std::vector<double> sol;
    double a,b,c,d;
    unsigned int k;
    switch(case_ABC){
        case 210:
            return -SQR(jmax-sqrt(4*sqrt(smax*(a0*jmax_2+a0_2*smax
                                       +2*jmax*smax*(v0-vC)))+jmax_2))/(4*smax);
        case 211:
            return (jmax_2-sqrt(jmax_4+2*a0_2*smax_2+4*jmax*smax_2*(v0-vC)
                                                   +2*a0*jmax_2*smax))/(2*smax);
        case 2010:
            return SQR(jmax-sqrt(4*sqrt(smax*(a0_2*smax-a0*jmax_2
                                       +2*jmax*smax*(vC-v0)))+jmax_2))/(4*smax);
        case 2011:
            return (sqrt(jmax_4+2*a0_2*smax_2+4*jmax*smax_2*(vC-v0)
                                            -2*a0*jmax_2*smax)-jmax_2)/(2*smax);
        case 200:
            a=2*sqrt(smax)*(v0-vC)/a0;
            b=-a0;
            c=-4*a0*sqrt(smax)*(v0-vC)/a0;
            d=-(smax*SQR(v0-vC)+a0_3)/a0;
            sol=poly_root_4(1,a,b,c,d);
            for(k=0;k<sol.size();k++){
                sol[k]=-SQR(sol[k])+a0;
            }
            break;
        case 201:
            a=2*jmax/sqrt(smax);
            b=(jmax_2-2*a0*smax)/smax;
            c=-4*a0*jmax/sqrt(smax);
            d=-(a0*jmax_2-a0_2*smax+2*jmax*smax*(v0-vC))/smax;
            sol=poly_root_4(1,a,b,c,d);
            for(k=0;k<sol.size();k++){
                sol[k]=-SQR(sol[k])+a0;
            }
            break;
        case 2000:
            a=2*sqrt(smax)*(vC-v0)/a0;
            b=-a0;
            c=0;
            d=-(smax*SQR(vC-v0)+a0_3)/a0;
            sol=poly_root_4(1,a,b,c,d);
            for(k=0;k<sol.size();k++){
                sol[k]=SQR(sol[k]);
            }
            break;
        case 2001:
            a=2*jmax/sqrt(smax);
            b=(2*a0*smax+jmax_2)/smax;
            c=4*a0*jmax/sqrt(smax);
            d=(a0*jmax_2+a0_2*smax+2*jmax*smax*(v0-vC))/smax;
            sol=poly_root_4(1,a,b,c,d);
            for(k=0;k<sol.size();k++){
                sol[k]=SQR(sol[k])+a0;
            }
            break;
        default:
            //printf("error: kdtp::Spline::a_b: unknown case : ");
            //printf("%d (vC=%f)\n",case_ABC,vC);
            return _int_a_abc.at(_cases_abc.size());
    }
    double dV=EPSI4+1;
    double aB;
    if(sol.size()>0){
        double dVmin=fabs(vC-this->v_c(sol.at(0),0,false));
        aB=sol.at(0);
        for(unsigned int i=1;i<sol.size();i++){
            dV=fabs(vC-this->v_c(sol.at(i),0,false));
            if(dV<dVmin){
                dVmin=dV;
                aB=sol.at(i);
            }
        }
        dV=dVmin;
    }
    if(dV>EPSI4){
        double mina=_int_a_abc.at(index_int);
        double maxa=_int_a_abc.at(index_int+1);
        aB=(mina+maxa)/2;
        dV=vC-this->v_c(aB,0,false);
        while(fabs(dV)>EPSI4 && fabs(mina-maxa)>EPSI4){
            if(dV>0)
                mina=aB;
            else
                maxa=aB;
            aB=(mina+maxa)/2;
            dV=vC-this->v_c(aB,0,false);
        }
    }

    return aB;
}

/**
* Returns acceleration during phase G and sets corresponding duration of phase G
* into _durations for a given desired velocity at the beginning of phase E.
* Input:
*    double vE : desired velocity at the beginning of phase E
* Output:
*    double : acceleration during phase G
*/
double Spline::a_g(double vE){

    double vF=_end[1];
    double aF=_end[2];
    double aF_2=SQR(aF);
    double aF_3=aF_2*aF;
    double amax=this->getAccelerationMax();
    double jmax=this->getJerkMax();
    double jmax_2=SQR(jmax);
    double jmax_4=SQR(jmax_2);
    double smax=this->getSnapMax();
    double smax_2=SQR(smax);

    if(_cases_egh.empty()){
        printf("error: kdtp::Spline::a_g: _cases_egh is not initialized\n");
        return 0;
    }

    if(_int_v_egh.size()!=_cases_egh.size()+1){
        printf("error: kdtp::Spline::a_g: _int_v_egh is not initialized\n");
        return 0;
    }

    if(_int_a_egh.size()!=_cases_egh.size()+1){
        printf("error: kdtp::Spline::a_g: _int_a_egh is not initialized\n");
        return 0;
    }

    if(vE<_int_v_egh.at(0)){
        _durations[G]=(_int_v_egh.at(0)-vE)/amax;
        return amax;
    }

    if(vE>=_int_v_egh.at(_cases_egh.size())){
        _durations[G]=(vE-_int_v_egh.at(_cases_egh.size()))/amax;
        return -amax;
    }

    _durations[G]=0;
    unsigned int index_int=0;
    while(index_int<_cases_egh.size() && vE>_int_v_egh.at(index_int+1)){
        index_int++;
    }
    if(index_int==_cases_egh.size()){
        printf("error: kdtp::Spline::a_g: vE not found (vE=%f)\n",vE);
        return aF;
    }
    int case_EGH=_cases_egh.at(index_int);
    std::vector<double> sol;
    double a,b,c,d;
    unsigned int k;
    switch(case_EGH){
        case 210:
            return SQR(jmax-sqrt(4*sqrt(smax*(aF_2*smax-aF*jmax_2
                                       +2*jmax*smax*(vF-vE)))+jmax_2))/(4*smax);
        case 211:
            return (sqrt(jmax_4+2*aF_2*smax_2-2*aF*jmax_2*smax
                                       +4*jmax*smax_2*(vF-vE))-jmax_2)/(2*smax);
        case 2010:
            return -SQR(jmax-sqrt(4*sqrt(smax*(aF*jmax_2+aF_2*smax
                                       +2*jmax*smax*(vE-vF)))+jmax_2))/(4*smax);
        case 2011:
            return (jmax_2-sqrt(jmax_4+2*aF_2*smax_2+2*aF*jmax_2*smax
                                              +4*jmax*smax_2*(vE-vF)))/(2*smax);
        case 200:
            a=2*sqrt(smax)*(vF-vE)/aF;
            b=-aF;
            c=0;
            d=-(smax*SQR(vE-vF)+aF_3)/aF;
            sol=poly_root_4(1,a,b,c,d);
            for(k=0;k<sol.size();k++){
                sol[k]=SQR(sol[k]);
            }
            break;
        case 201:
            a=2*jmax/sqrt(smax);
            b=(2*aF*smax+jmax_2)/smax;
            c=4*aF*jmax/sqrt(smax);
            d=(aF*jmax_2+aF_2*smax+2*jmax*smax*(vE-vF))/smax;
            sol=poly_root_4(1,a,b,c,d);
            for(k=0;k<sol.size();k++){
                sol[k]=SQR(sol[k])+aF;
            }
            break;
        case 2000:
            a=2*sqrt(smax)*(vE-vF)/aF;
            b=-aF;
            c=4*aF*sqrt(smax)*(vF-vE)/aF;
            d=-(smax*SQR(vE-vF)+aF_3)/aF;
            sol=poly_root_4(1,a,b,c,d);
            for(k=0;k<sol.size();k++){
                sol[k]=-SQR(sol[k])+aF;
            }
            break;
        case 2001:
            a=2*jmax/sqrt(smax);
            b=(jmax_2-2*aF*smax)/smax;
            c=-4*aF*jmax/sqrt(smax);
            d=(aF_2*smax-aF*jmax_2+2*jmax*smax*(vF-vE))/smax;
            sol=poly_root_4(1,a,b,c,d);
            for(k=0;k<sol.size();k++){
                sol[k]=-SQR(sol[k])+aF;
            }
            break;
        default:
            //printf("error: kdtp::Spline::a_g: unknown case :");
            //printf(" %d (vE=%f)\n",case_EGH,vE);
            return _int_a_egh.at(_cases_egh.size());
    }
    double dV=EPSI4+1;
    double aG;
    if(sol.size()>0){
        double dVmin=fabs(vE-this->v_e(sol.at(0),0,false));
        aG=sol.at(0);
        for(unsigned int i=1;i<sol.size();i++){
            dV=fabs(vE-this->v_e(sol.at(i),0,false));
            if(dV<dVmin){
                dVmin=dV;
                aG=sol.at(i);
            }
        }
        dV=dVmin;
    }
    if(dV>EPSI4){
        double mina=_int_a_egh.at(index_int);
        double maxa=_int_a_egh.at(index_int+1);
        aG=(mina+maxa)/2;
        dV=vE-this->v_e(aG,0,false);
        while(fabs(dV)>EPSI4 && fabs(maxa-mina)>EPSI4){
            if(dV>0)
                mina=aG;
            else
                maxa=aG;
            aG=(mina+maxa)/2;
            dV=vE-this->v_e(aG,0,false);
        }
    }

    return aG;
}

/**
* Returns difference between position at the beginning of phase E and position
* at the end of phase C given a desired velocity during phase D.
* Sets the corresponding durations into _durations and snap signs into _signs.
* Input:
*    double vD : desired velocity during phase D
* Output:
*    double : difference between position at the beginning of phase E and
*             position at the end of phase C
*/
double Spline::d_x(double vD){
    double aB=this->a_b(vD);
    this->durations_and_signs_ac(aB);
    double aG=this->a_g(vD);
    this->durations_and_signs_eh(aG);
    double xC=this->x_c(aB,0,true);
    double xE=this->x_e(aG,0,true);
    double dX=xE-xC;
    double tD=dX/vD;
    if(tD<0 || !std::isfinite(tD))
        tD=0;
    _durations[D]=tD;
    _durations[F]=2*_durations[A1]
                 +_durations[A2]
                 +_durations[B]
                 +2*_durations[C1]
                 +_durations[C2]
                 +_durations[D]
                 +2*_durations[E1]
                 +_durations[E2]
                 +_durations[G]
                 +2*_durations[H1]
                 +_durations[H2];
    return dX;
}

/**
* Calculates optimal velocity during phase D and sets it into _v_opt.
* Calculates and sets the corresponding durations and snap signs for all phases.
*/
void Spline::v_optim(){

    // Setting values of cases, velocities intervals and accelerations intervals
    this->intervals_ac();
    this->intervals_eh();

    // Searching a zero of d_x with secant method
    double vmax=this->getVelocityMax();
    double v1=0;
    double dX1=this->d_x(v1);
    double s=sign(dX1);
    double v2=s*vmax;
    double dX2=this->d_x(v2);
    double v=v2-(v2-v1)*dX2/(dX2-dX1);
    double dX=this->d_x(v);
    int nb_it=0;
    while(fabs(dX)>EPSI4 && nb_it<NB_IT_MAX){
        dX1=dX2;
        dX2=dX;
        v1=v2;
        v2=v;
        v=v2-(v2-v1)*dX2/(dX2-dX1);
        dX=this->d_x(v);
        nb_it++;
    }

    // vmax is the upper bound
    if(fabs(v)>vmax || s*v<0){
        v=s*vmax;
        dX=this->d_x(v);
    }

    // if not found or out of bound, dichotomial search
    if(fabs(dX)>EPSI4 && s*dX<0){
        v1=0;
        v2=fabs(v);
        v=v2/2;
        dX=this->d_x(s*v);
        while(fabs(dX)>EPSI4 && fabs(v1-v2)>EPSI4){
            if(s*dX<0)
                v2=v;
            else
                v1=v;
            v=(v1+v2)/2;
            dX=this->d_x(s*v);
        }
        if(s*dX<0){
            v=v1;
            dX=this->d_x(s*v);
        }
        v=s*v;
    }

    _v_opt=v;

    if(fabs(v)>vmax || s*v<0){
        printf("error: kdtp::Spline::v_optim(): v=%f s=%f vmax=%f\n",v,s,vmax);
    }

}

void Spline::pushNext(double time,std::vector<double> next){
    _times.push_back(time);
    if(next.size()>0)
        _positions.push_back(next.at(0));
    if(next.size()>1)
        _velocities.push_back(next.at(1));
    if(next.size()>2)
        _accelerations.push_back(next.at(2));
    if(next.size()>3)
        _jerks.push_back(next.at(3));
    if(next.size()>4)
        _snaps.push_back(next.at(4));
}

void Spline::clearValues(){
    _times.clear();
    _positions.clear();
    _velocities.clear();
    _accelerations.clear();
    _jerks.clear();
    _snaps.clear();
}

void Spline::setValues(){
    this->clearValues();
    int index=0;
    double time=0;
    _times.push_back(0);
    _positions.push_back(_init[0]);
    _velocities.push_back(_init[1]);
    _accelerations.push_back(_init[2]);
    _jerks.push_back(0);
    _snaps.push_back(0);
    durations_index phases[15]={A1,A2,A1,B,C1,C2,C1,D,E1,E2,E1,G,H1,H2,H1};
    double snap_signs[15]={_signs[A],0,-_signs[A],0,
                           _signs[C],0,-_signs[C],0,
                           _signs[E],0,-_signs[E],0,
                           _signs[H],0,-_signs[H]};
    for(int k=0;k<15;k++){
        if(_durations[phases[k]]>EPSILON){
            time+=_durations[phases[k]];
            _snaps.at(index)=snap_signs[k]*this->getSnapMax();
            std::vector<double> next=this->getAllAt(index,time);
            this->pushNext(time,next);
            index++;
        }
    }
}

unsigned int Spline::getIndexAt(double time){
    if(time<=0 || fabs(time)<EPSILON)
        return 0;
    if(time>=_durations[F] || fabs(time-_durations[F])<EPSILON)
        return _times.size()-1;
    unsigned int current=_last_call;
    if(current>=_times.size() || time<_times.at(current))
        current=0;
    while(current<_times.size()-1 && time>_times.at(current+1)){
        current++;
    }
    _last_call=current;
    return current;
}

double Spline::getPositionAtLocal(unsigned int index,double local){
    double position=_positions.at(index)
                   +_velocities.at(index)*local
                   +_accelerations.at(index)*local*local/2
                   +_jerks.at(index)*local*local*local/6
                   +_snaps.at(index)*local*local*local*local/24;
    if(_dof->isRotation())
        position=mainMeasurement(position);
    return position;
}

double Spline::getPositionAt(unsigned int index,double time){
    return this->getPositionAtLocal(index,time-_times.at(index));
}

double Spline::getVelocityAtLocal(unsigned int index,double local){
    return  _velocities.at(index)
           +_accelerations.at(index)*local
           +_jerks.at(index)*local*local/2
           +_snaps.at(index)*local*local*local/6;
}


double Spline::getVelocityAt(unsigned int index,double time){
    return this->getVelocityAtLocal(index,time-_times.at(index));
}

double Spline::getAccelerationAtLocal(unsigned int index,double local){
    return  _accelerations.at(index)
           +_jerks.at(index)*local
           +_snaps.at(index)*local*local/2;
}

double Spline::getAccelerationAt(unsigned int index,double time){
    return this->getAccelerationAtLocal(index,time-_times.at(index));
}

double Spline::getJerkAtLocal(unsigned int index,double local){
    return _jerks.at(index)+_snaps.at(index)*local;
}

double Spline::getJerkAt(unsigned int index,double time){
    return this->getJerkAtLocal(index,time-_times.at(index));
}

vector3 Spline::getStateAt(unsigned int index,double time){
    double local=time-_times.at(index);
    vector3 conf=vector3();
    conf[0]=this->getPositionAtLocal(index,local);
    conf[1]=this->getVelocityAtLocal(index,local);
    conf[2]=this->getAccelerationAtLocal(index,local);
    return conf;
}

std::vector<double> Spline::getAllAt(unsigned int index,double time){
    double local=time-_times.at(index);
    std::vector<double> ret;
    ret.push_back(this->getPositionAtLocal(index,local));
    ret.push_back(this->getVelocityAtLocal(index,local));
    ret.push_back(this->getAccelerationAtLocal(index,local));
    ret.push_back(this->getJerkAtLocal(index,local));
    ret.push_back(_snaps[index]);
    return ret;
}

/* PUBLIC */

double Spline::getDuration(){
    if(_stay_still)
        return 0;
    return _durations[F];
}

/**
* Synchronizes spline with given duration by reducing velocity dusing phase D
* Input:
*    double tF_des : desired total duration
*/
void Spline::synchronize(double tF_des){
    if(!_stay_still){
        if(_durations[F]<tF_des && fabs(_durations[F]-tF_des)>EPSI4){
           double s=sign(_v_opt);
           double minv=0;
           double maxv=fabs(_v_opt);
           double vD=maxv/2;
           this->d_x(s*vD);
           while(fabs(_durations[F]-tF_des)>EPSI4 && fabs(minv-maxv)>EPSI4/100){
              if(_durations[F]<tF_des)
                 maxv=vD;
              else
                 minv=vD;
              vD=(minv+maxv)/2;
              this->d_x(s*vD);
           }
           vD=s*vD;
        }
        this->setValues();
    }
}

double Spline::getVelocityMax(){
    if(!_dof){
        printf("error: kdtp::Spline::getVelocityMax: _dof attribute is null\n");
        return INFINITY;
    }
    return _dof->getVelocityMax();
}

double Spline::getAccelerationMax(){
    if(!_dof){
        printf("error: kdtp::Spline::getAccelerationMax(): _dof attribute is");
        printf(" null\n");
        return INFINITY;
    }
    return _dof->getAccelerationMax();
}

double Spline::getJerkMax(){
    if(!_dof){
        printf("error: kdtp::Spline::getJerkMax(): _dof attribute is null\n");
        return INFINITY;
    }
    return _dof->getJerkMax();
}

double Spline::getSnapMax(){
    if(!_dof){
        printf("error: kdtp::Spline::getSnapMax(): _dof attribute is null\n");
        return INFINITY;
    }
    return _dof->getSnapMax();
}

double Spline::getPositionAt(double time){
    if(_stay_still || time<=0 || fabs(time)<EPSILON){
        return _init[0];
    }
    if(time>=_durations[F] || fabs(time-_durations[F])<EPSILON){
        return _end[0];
    }
    return this->getPositionAt(this->getIndexAt(time),time);
}

double Spline::getVelocityAt(double time){
    if(_stay_still)
        return 0;
    if(time<=0 || fabs(time)<EPSILON)
        return _init[1];
    if(time>=_durations[F] || fabs(time-_durations[F])<EPSILON)
        return _end[1];
    return this->getVelocityAt(this->getIndexAt(time),time);
}

double Spline::getAccelerationAt(double time){
    if(_stay_still)
        return 0;
    if(time<=0 || fabs(time)<EPSILON){
        return _init[2];
    }
    if(time>=_durations[F] || fabs(time-_durations[F])<EPSILON){
        return _end[2];
    }
    return this->getAccelerationAt(this->getIndexAt(time),time);
}

double Spline::getJerkAt(double time){
    if(_stay_still || time<=0 || fabs(time)<EPSILON)
        return 0;
    if(time>=_durations[F] || fabs(time-_durations[F])<EPSILON)
        return 0;
    return this->getJerkAt(this->getIndexAt(time),time);
}

double Spline::getSnapAt(double time){
    if(_stay_still)
        return 0;
    if(time<=0 || fabs(time)<EPSILON)
        return _signs[A]*this->getSnapMax();
    if(time>=_durations[F] || fabs(time-_durations[F])<EPSILON)
        return -_signs[H]*this->getSnapMax();
    return this->getSnapAt(this->getIndexAt(time));
}

vector3 Spline::getStateAt(double time){
    if(_stay_still || time<=0 || fabs(time)<EPSILON)
        return this->getInit();
    if(time>=_durations[F] || fabs(time-_durations[F])<EPSILON)
        return this->getEnd();
    return this->getStateAt(this->getIndexAt(time),time);
}

std::vector<double> Spline::getAllAt(double time){
    if(_stay_still){
        std::vector<double> ret;
        ret.push_back(_init.x);
        for(unsigned int i=0;i<4;i++){
            ret.push_back(0);
        }
        return ret;
    }
    if(time<=0 || fabs(time)<EPSILON){
        std::vector<double> ret;
        for(unsigned int i=0;i<3;i++){
            ret.push_back(_init[i]);
        }
        ret.push_back(0);
        ret.push_back(_signs[A]*this->getSnapMax());
        return ret;
    }
    if(time>=_durations[F] || fabs(time-_durations[F])<EPSILON){
        std::vector<double> ret;
        for(unsigned int i=0;i<3;i++){
            ret.push_back(_end[i]);
        }
        ret.push_back(0);
        ret.push_back(-_signs[H]*this->getSnapMax());
        return ret;
    }
    return this->getAllAt(this->getIndexAt(time),time);
}

vector3 Spline::getInit(){
    return vector3(_init.x,_init.y,_init.z);
}

vector3 Spline::getEnd(){
    return vector3(_end.x,_end.y,_end.z);
}

}
