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

#include "kinosplines/kdtp_state.h"

namespace kdtp {

State::State(RobotPtr robot){
    if(!robot){
        printf("error: kdtp::in State constructor: robot parameter is null\n");
        return;
    }
    _robot=robot;
    unsigned int nbDof=robot->getNbDof();
    for(unsigned int i=0;i<nbDof;i++){
        _position.push_back(0);
        _velocity.push_back(0);
        _acceleration.push_back(0);
    }
}

std::vector<double> State::getPosition(){
    return _position;
}

double State::getPosition(unsigned int i){
    if(_position.empty()){
        printf("error: kdtp::State::getPosition: _position is empty\n");
        return 0;
    }
    if(i>=_position.size()){
        printf("error: kdtp::State::getPosition: index i=%d is out of\n",(int)i);
        printf("range [0 %d], returning i=",(int)_robot->getNbDof()-1);
        printf("%d instead\n",(int)_velocity.size()-1);
        return _position[_position.size()-1];
    }
    return _position[i];
}

std::vector<double> State::getVelocity(){
    return _velocity;
}

double State::getVelocity(unsigned int i){
    if(_velocity.empty()){
        printf("error: kdtp::State::getVelocity: _velocity is empty\n");
        return 0;
    }
    if(i>=_velocity.size()){
        printf("error: kdtp::State::getVelocity: index i=%d is out of\n",(int)i);
        printf("range [0 %d], returning i=",(int)_robot->getNbDof()-1);
        printf("%d instead\n",(int)_velocity.size()-1);
        return _velocity[_velocity.size()-1];
    }
    return _velocity[i];
}

std::vector<double> State::getAcceleration(){
    return _acceleration;
}

double State::getAcceleration(unsigned int i){
    if(_acceleration.empty()){
        printf("error: kdtp::State::getAcceleration: _acceleration is empty\n");
        return 0;
    }
    if(i>=_acceleration.size()){
        printf("error: kdtp::State::getAcceleration: index i=%d is\n",(int)i);
        printf("out of range [0 %d], returning i=",(int)_robot->getNbDof()-1);
        printf("%d instead\n",(int)_acceleration.size()-1);
        return _acceleration[_acceleration.size()-1];
    }
    return _acceleration[i];
}

vector3 State::getDofState(unsigned int i){
    if(_position.empty()){
        printf("error: kdtp::State::getDofState: _position is empty\n");
        return vector3();
    }
    if(_velocity.empty()){
        printf("error: kdtp::State::getDofState: _velocity is empty\n");
        return vector3();
    }
    if(_acceleration.empty()){
        printf("error: kdtp::State::getDofState: _acceleration is empty\n");
        return vector3();
    }
    unsigned int mini=i;
    if(mini>=_position.size())
        mini=_position.size()-1;
    if(mini>=_velocity.size())
        mini=_velocity.size()-1;
    if(mini>=_acceleration.size())
        mini=_acceleration.size()-1;
    if(mini!=i){
        printf("error: kdtp::State::getDofState: index i=%d is out of\n",(int)i);
        printf("range [0 %d], returning i=",(int)_robot->getNbDof()-1);
        printf("%d instead\n",(int)mini);
    }
    return vector3(_position[mini],_velocity[mini],_acceleration[mini]);
}

void State::setPosition(unsigned int i,double value){
    if(!_robot){
        printf("error: kdtp::State::setPosition: _robot is null\n");
        return;
    }
    DofPtr dof=_robot->getDof(i);
    if(!dof){
        printf("error: kdtp::State::setPosition: cannot access dof of index");
        printf("%d\n",(int)i);
        return;
    }
    double val=value;
    if(dof->isRotation())
        val=mainMeasurement(value);
    if(i>=_position.size()){
        printf("error: kdtp::State::setPosition: index i=%d is out of ",(int)i);
        printf("range [0 %d]\n",(int)_position.size()-1);
        return;
    }
    _position[i]=val;
}

void State::setVelocity(unsigned int i,double value){
    if(!_robot){
        printf("error: kdtp::State::setVelocity: _robot is null\n");
        return;
    }
    if(i>=_velocity.size()){
        printf("error: kdtp::State::setVelocity: index i=%d is out of ",(int)i);
        printf("range [0 %d]\n",(int)_velocity.size()-1);
        return;
    }
    _velocity[i]=value;
}

void State::setAcceleration(unsigned int i,double value){
    if(!_robot){
        printf("error: kdtp::State::setAcceleration: _robot is null\n");
        return;
    }
    if(i>=_acceleration.size()){
        printf("error: kdtp::State::setAcceleration: ");
        printf("index i=%d is out ",(int)i);
        printf("of range [0 %d]\n",(int)_velocity.size()-1);
        return;
    }
    _acceleration[i]=value;
}

int get_case(double aB,Data *data){
    double a0=data->a0;
    double jmax=data->jmax;
    double smax=data->smax;
    int sA=(int)sign(aB-a0);
    int sC=-(int)sign(aB);
    int cond_A=(fabs(a0-aB)>SQR(jmax)/smax);
    int cond_C=(fabs(aB)>SQR(jmax)/smax);
    return 1000*(sA+1)+100*(sC+1)+10*cond_A+cond_C;
}

void durations_and_signs(double aB,Data *data){
    double a0=data->a0;
    double jmax=data->jmax;
    double smax=data->smax;
    data->signs[A]=sign(aB-a0);
    data->signs[C]=-sign(aB);
    if(fabs(aB-a0)>SQR(jmax)/smax){
        data->durations[A1]=jmax/smax;
        data->durations[A2]=fabs(aB-a0)/jmax-jmax/smax;
    }
    else{
        data->durations[A1]=sqrt(fabs(a0-aB)/smax);
        data->durations[A2]=0;
    }
    if(fabs(aB)>SQR(jmax)/smax){
        data->durations[C1]=jmax/smax;
        data->durations[C2]=fabs(aB)/jmax-jmax/smax;
    }
    else{
        data->durations[C1]=sqrt(fabs(aB)/smax);
        data->durations[C2]=0;
    }
}

double v_c(double aB,double tB,Data *data){
    double v0=data->v0;
    double a0=data->a0;
    double smax=data->smax;
    data->durations[B]=tB;
    durations_and_signs(aB,data);
    double tA1=data->durations[A1];
    double tA2=data->durations[A2];
    double tC1=data->durations[C1];
    double tC2=data->durations[C2];
    double sA=data->signs[A];
    double sC=data->signs[C];
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
            +(sC*smax*tC1*tC2_2)/2+2*a0*tC1+a0*tC2+v0+a0*tB;
}

Data *intervals(double v0,double a0,double amax,double jmax,double smax){
    Data *data=new Data();
    data->v0=v0;
    data->a0=a0;
    data->amax=amax;
    data->jmax=jmax;
    data->smax=smax;
    if(!data){
        printf("error intervals: cannot allocate data structure\n");
        return NULL;
    }
    int i,k,l,n=1;
    double a1,a2;
    double aj=jmax*jmax/smax;
    double Ac[7]={amax,0,aj,-aj,a0,a0+aj,a0-aj};
    data->int_a[0]=-amax;
    for(int k=0;k<7;k++){
        if((Ac[k]*a0<=0 || fabs(Ac[k])>=fabs(a0)) && fabs(Ac[k])<=amax){
            l=0;
            while(l<n && Ac[k]>data->int_a[l]){
                l++;
            }
            if(l==n || fabs(Ac[k]-data->int_a[l])>EPSI4){
                for(i=n+1;i>l;i--){
                    data->int_a[i]=data->int_a[i-1];
                }
                data->int_a[l]=Ac[k];
                n++;
            }
        }
    }
    n--;
    data->cases[0]=n;
    a1=data->int_a[0];
    data->int_v[0]=v_c(a1,0,data);
    for(k=1;k<=n;k++){
        a1=data->int_a[k-1];
        a2=data->int_a[k];
        data->int_v[k]=v_c(a2,0,data);
        data->cases[k]=get_case((a1+a2)/2,data);
    }
    return data;
}

double a_b(double vC,Data *data){

    double v0=data->v0;
    double a0=data->a0;
    double amax=data->amax;
    double jmax=data->jmax;
    double smax=data->smax;
    double a0_2=a0*a0;
    double a0_3=a0_2*a0;
    int nb_int=data->cases[0];
    double jmax_2=jmax*jmax;
    double jmax_4=jmax_2*jmax_2;
    double smax_2=smax*smax;

    if(vC<data->int_v[0]){
        data->durations[B]=(data->int_v[0]-vC)/amax;
        return -amax;
    }

    if(vC>data->int_v[nb_int]){
        data->durations[B]=(vC-data->int_v[nb_int])/amax;
        return amax;
    }

    data->durations[B]=0;
    int k=1;
    while(k<=nb_int && vC>data->int_v[k]){
        k++;
    }
    int index_int=k;
    if(index_int>nb_int){
        //printf("error: a_b: vC not found\n");
        return a0;
    }
    int case_ABC=data->cases[index_int];
    double a,b,c,d;
    std::vector<double> sol;
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
        for(k=0;k<(int)sol.size();k++){
            sol[k]=-SQR(sol[k])+a0;
        }
        break;
    case 201:
        a=2*jmax/sqrt(smax);
        b=(jmax_2-2*a0*smax)/smax;
        c=-4*a0*jmax/sqrt(smax);
        d=-(a0*jmax_2-a0_2*smax+2*jmax*smax*(v0-vC))/smax;
        sol=poly_root_4(1,a,b,c,d);
        for(k=0;k<(int)sol.size();k++){
            sol[k]=-SQR(sol[k])+a0;
        }
        break;
    case 2000:
        a=2*sqrt(smax)*(vC-v0)/a0;
        b=-a0;
        c=0;
        d=-(smax*SQR(vC-v0)+a0_3)/a0;
        sol=poly_root_4(1,a,b,c,d);
        for(k=0;k<(int)sol.size();k++){
            sol[k]=SQR(sol[k]);
        }
        break;
    case 2001:
        a=2*jmax/sqrt(smax);
        b=(2*a0*smax+jmax_2)/smax;
        c=4*a0*jmax/sqrt(smax);
        d=(a0*jmax_2+a0_2*smax+2*jmax*smax*(v0-vC))/smax;
        sol=poly_root_4(1,a,b,c,d);
        for(k=0;k<(int)sol.size();k++){
            sol[k]=SQR(sol[k])+a0;
        }
        break;
    default:
        return data->int_a[index_int];
    }
    int N=(int)sol.size();
    double dVmin,aB,dV,mina,maxa;
    if(N>0){
        dVmin=fabs(vC-v_c(sol[0],0,data));
        aB=sol[0];
        for(k=1;k<N;k++){
            dV=fabs(vC-v_c(sol[k],0,data));
            if(dV<dVmin){
                dVmin=dV;
                aB=sol[k];
            }
        }
        dV=dVmin;
    }
    else{
        dV=EPSI4+1;
    }
    if(dV>EPSI4){
        mina=data->int_a[index_int-1];
        maxa=data->int_a[index_int];
        aB=(mina+maxa)/2;
        dV=vC-v_c(aB,0,data);
        while(fabs(dV)>EPSI4 && fabs(mina-maxa)>EPSI4){
            if(dV>0){
                mina=aB;
            }
            else{
                maxa=aB;
            }
            aB=(mina+maxa)/2;
            dV=vC-v_c(aB,0,data);
        }
    }

    return aB;
}

double get_v_root(Data *data){
    double Times[8];
    Times[0]=0;
    Times[1]=data->durations[A1];
    Times[2]=Times[1]+data->durations[A2];
    Times[3]=Times[2]+data->durations[A1];
    Times[4]=Times[3]+data->durations[B];
    Times[5]=Times[4]+data->durations[C1];
    Times[6]=Times[5]+data->durations[C2];
    Times[7]=Times[6]+data->durations[C1];
    double S[7]={ data->smax*data->signs[A],0,
                  -data->smax*data->signs[A],0,
                  data->smax*data->signs[C],0,
                  -data->smax*data->signs[C]};
    double j_cur=0;
    double a_cur=data->a0;
    double v_cur=data->v0;
    for(int i=0;i<7;i++){
        double tmin=Times[i];
        double tmax=Times[i+1];
        std::vector<double> sol=poly_root_3(S[i]/6,j_cur/2,a_cur,v_cur);
        for(int j=0;j<(int)sol.size();j++){
            double root=sol[j]+tmin;
            if(tmin<=root && root<=tmax)
                return root;
        }
        double trel=tmax-tmin;
        double trel_2=trel*trel;
        double trel_3=trel*trel_2;
        v_cur=S[i]*trel_3/6+j_cur*trel_2/2+a_cur*trel+v_cur;
        a_cur=S[i]*trel_2/2+j_cur*trel+a_cur;
        j_cur=S[i]*trel+j_cur;
    }

    return -1;
}

double spline_at(double t,double x0,Data *data,double *v){
    if(t<0)
        return x0;
    double Times[8];
    Times[0]=0;
    Times[1]=data->durations[A1];
    Times[2]=Times[1]+data->durations[A2];
    Times[3]=Times[2]+data->durations[A1];
    Times[4]=Times[3]+data->durations[B];
    Times[5]=Times[4]+data->durations[C1];
    Times[6]=Times[5]+data->durations[C2];
    Times[7]=Times[6]+data->durations[C1];
    double S[7]={ data->smax*data->signs[A],0,
                  -data->smax*data->signs[A],0,
                  data->smax*data->signs[C],0,
                  -data->smax*data->signs[C]};
    double j_cur=0;
    double a_cur=data->a0;
    double v_cur=data->v0;
    double x_cur=x0;
    for(int i=0;i<7;i++){
        double tmin=Times[i];
        double tmax=Times[i+1];
        if(tmin<=t && t<=tmax){
            double trel=t-tmin;
            double trel_2=trel*trel;
            double trel_3=trel*trel_2;
            double trel_4=trel*trel_3;
            *v=S[i]*trel_3/6+j_cur*trel_2/2+a_cur*trel+v_cur;
            return S[i]*trel_4/24+j_cur*trel_3/6+a_cur*trel_2/2+v_cur*trel
                    +x_cur;
        }
        else{
            double trel=tmax-tmin;
            double trel_2=trel*trel;
            double trel_3=trel*trel_2;
            double trel_4=trel*trel_3;
            x_cur=S[i]*trel_4/24+j_cur*trel_3/6+a_cur*trel_2/2+v_cur*trel+x_cur;
            v_cur=S[i]*trel_3/6+j_cur*trel_2/2+a_cur*trel+v_cur;
            a_cur=S[i]*trel_2/2+j_cur*trel+a_cur;
            j_cur=S[i]*trel+j_cur;
        }
    }
    *v=v_cur;
    return x_cur;
}

double x_bound(double v0,double a0,double xmin,double xmax,double vmax,
               double amax,double jmax,double smax,bool backward){
    double bound=0;
    Data *data=intervals(v0,a0,amax,jmax,smax);
    double vC=-sign(v0)*vmax;
    double aB=a_b(vC,data);
    durations_and_signs(aB,data);
    double root=get_v_root(data);
    if(root>0){
        double v;
        double x=spline_at(root,0,data,&v);
        if(backward)
            x=-x;
        if(backward){
            if(v0>0)
                bound=xmin-x;
            else
                bound=xmax-x;
        }
        else{
            if(v0<0)
                bound=xmin-x;
            else
                bound=xmax-x;
        }
    }
    else
    {
        //printf("error: x_bound: v_root not found\n");
    }
    delete data;
    return bound;
}

double get_a_root(double a0,double aB,double smax,double t1,double t2){
    double dB0=sign(aB-a0);
    double delta=-2*dB0*smax*a0;
    double root;
    if(delta>=0){
        root=sqrt(delta)/(dB0*smax);
        if(root<0)
            root=-root;
        if(root<=t1)
            return root;
    }
    root=-(-dB0*smax*t1*t1+2*a0)/(2*dB0*smax*t1);
    if(t1<=root && root<=t1+t2)
        return root;
    delta=2*dB0*smax*(dB0*smax*t1*(t1+t2)+a0);
    if(delta>=0){
        double b=dB0*smax*(2*t1+t2);
        root=(-b-sqrt(delta))/(dB0*smax);
        if(t1+t2<=root && root<=2*t1+t2)
            return root;
        root=(-b+sqrt(delta))/(dB0*smax);
        if(t1+t2<=root && root<=2*t1+t2)
            return root;
    }
    //printf("error: get_a_root: cannot find root :(\n");
    return -1;
}

double get_velocity(double t,double v0,double a0,double aB,double smax,
                    double t1,double t2){
    double dB0=sign(aB-a0);
    if(t<t1)
        return dB0*smax*t*t*t/6+a0*t+v0;
    if(t<t1+t2)
        return dB0*smax*t1*(t-t1)*(t-t1)/2+(dB0*smax*t1*t1/2+a0)*(t-t1)
                +dB0*smax*t1*t1*t1/6+a0*t1+v0;
    return -dB0*smax*(t-t1-t2)*(t-t1-t2)*(t-t1-t2)/6+dB0*smax*t1*(t-t1-t2)
            *(t-t1-t2)/2+(dB0*smax*t1*t2+dB0*smax*t1*t1/2+a0)*(t-t1-t2)
            +dB0*smax*t1*t2*t2/2+(dB0*smax*t1*t1/2+a0)*t2+dB0*smax*t1*t1*t1/6
            +a0*t1+v0;
}

void State::uniform_shoot(){
    unsigned int nbDof=_robot->getNbDof();
    for(unsigned int i=0;i<nbDof;i++){
        DofPtr dof=_robot->getDof(i);
        this->setPosition(i,gen_rand(dof->getPositionMin(),
                                     dof->getPositionMax()));
        this->setVelocity(i,gen_rand(-dof->getVelocityMax(),
                                     dof->getVelocityMax()));
        this->setAcceleration(i,gen_rand(-dof->getAccelerationMax(),
                                         dof->getAccelerationMax()));
    }
}

void State::incremental_shoot(){
    unsigned int nbDof=_robot->getNbDof();
    for(unsigned int i=0;i<nbDof;i++){
        DofPtr dof=_robot->getDof(i);
        double xmin=dof->getPositionMin();
        double xmax=dof->getPositionMax();
        double vmax=dof->getVelocityMax();
        double amax=dof->getAccelerationMax();
        double jmax=dof->getJerkMax();
        double smax=dof->getSnapMax();
        int nb_try=0;
        bool found=false;
        while(!found && nb_try<1e2){
            double a0=gen_rand(-amax,amax);
            this->setAcceleration(i,a0);
            double aB=-sign(a0)*amax;
            double t1,t2;
            if(fabs(aB-a0)>jmax*jmax/smax){
                t1=jmax/smax;
                t2=fabs(aB-a0)/jmax-jmax/smax;
            }
            else{
                t1=sqrt(fabs(aB-a0)/smax);
                t2=0;
            }
            double root=get_a_root(a0,aB,smax,t1,t2);
            double v_bound=vmax;
            if(root<0){
                //printf("warning: kdtp::State::incremental_shoot: ");
                //printf("a_root not found for dof #%d\n",(int)i);
            }
            else{
                double v_root=get_velocity(root,0,a0,aB,smax,t1,t2);
                v_bound=fabs(sign(v_root)*vmax-v_root);
            }
            double v0=gen_rand(-v_bound,v_bound);
            this->setVelocity(i,v0);
            double boundF=x_bound(v0, a0,xmin,xmax,vmax,amax,jmax,smax,false);
            double boundB=x_bound(v0,-a0,xmin,xmax,vmax,amax,jmax,smax,true);
            xmin=std::min(boundB,boundF);
            xmax=std::max(boundB,boundF);
            found = (dof->getPositionMin()<=xmin) &&
                    (xmax<=dof->getPositionMax());
            nb_try++;
        }
        if(!found){
            //printf("warning: kdtp::State::incremental_shoot: ");
            //printf("x[%i] bounds not found\n",(int)i);
        }
        else
            this->setPosition(i,gen_rand(dof->getPositionMin(),dof->getPositionMax()));
    }
}

double State::distance(StatePtr state){

    if(!state){
        printf("error: kdtp::State::distance: state parameter is null\n");
        return INFINITY;
    }

    double tmax=-1;
    unsigned int nbDof=_robot->getNbDof();

    for(unsigned int i=0;i<nbDof;i++){

        DofPtr dof=_robot->getDof(i);
        double jmax=dof->getJerkMax();
        double x0=this->getPosition(i);
        double xT=state->getPosition(i);
        double v0=this->getVelocity(i);
        double vT=state->getVelocity(i);
        double a0=this->getAcceleration(i);
        double aT=state->getAcceleration(i);
        double umax=fabs(jmax);
        double tmin=INFINITY;
        int nbSol=0;

        // ZERO SWITCHING
        double u=umax;
        double T=(aT-a0)/u;
        if(T<0){
            T=-T;
            u=-u;
        }
        double v=u*T*T/2+a0*T+v0;
        if(fabs(v-vT)<1e-5){
            double x=u*T*T*T/6+a0*T*T/2+v0*T+x0;
            if(fabs(x-xT)<1e-5 && tmin>T){
                tmin=T;
                nbSol++;
            }
        }

        // ONE SWITCHING
        for(double u=-umax;u<=umax;u+=2*umax){
            double c2=u;
            double c1=2*a0;
            double c0=(a0*a0-aT*aT)/(2*u)+(v0-vT);
            std::vector<double> sol=poly_root_2(c2,c1,c0);
            for(unsigned int i=0;i<sol.size();i++){
                double t1=sol[i];
                if(t1>=0){
                    T=2*t1+(a0-aT)/u;
                    if(T>t1){
                        double x=-u*T*T*T/6+u*T*T*t1-u*T*t1*t1+u*t1*t1*t1/3
                                +a0*T*T+v0*T+x0;
                        if(fabs(x-xT)<1e-5 && tmin>T){
                            tmin=T;
                            nbSol++;
                        }
                    }
                }
            }
        }

        // TWO SWITCHINGS
        for(double u=-umax;u<=umax;u+=2*umax){
            double c4=u;
            double c3=0;
            double c2=2*(2*(v0+vT)-(a0*a0+aT*aT)/u);
            double c1=4*(x0-xT-(a0*v0-aT*vT)/u+(a0*a0*a0-aT*aT*aT)/(3*u*u));
            double tmp=aT*aT-a0*a0+2*u*(v0-vT);
            double c0=-tmp*tmp/(4*u*u*u);
            std::vector<double> sol=poly_root_4(c4,c3,c2,c1,c0);
            for(unsigned int i=0;i<sol.size();i++){
                double t2=sol[i];
                if(t2>0){
                    double t1=(2*u*u*t2*t2-4*u*t2*a0+a0*a0-aT*aT
                               +2*u*(vT-v0))/(4*u*u*t2);
                    if(t1>0){
                        T=2*t2+(aT-a0)/u;
                        if(T>t1+t2 && tmin>T){
                            tmin=T;
                            nbSol++;
                        }
                    }
                }
            }
        }

        if(nbSol>0 && tmin>tmax)
            tmax=tmin;

    }

    if(tmax<0){
        tmax=INFINITY;
        printf("error: kdtp::State::distance: no solutions\n");
    }
    return tmax;
}

bool State::isOutOfBounds(){
    for(unsigned int i=0;i<_robot->getNbDof();i++){
        kdtp::DofPtr dof=_robot->getDof(i);
        if(_position[i]<dof->getPositionMin()) return true;
        if(_position[i]>dof->getPositionMax()) return true;
        if(fabs(_velocity[i])>dof->getVelocityMax()) return true;
        if(fabs(_acceleration[i])>dof->getAccelerationMax()) return true;
    }
    return false;
}

void State::print(){
    printf("P=[ ");
    for(unsigned int i=0;i<_position.size();i++){
        printf("%f ",_position[i]);
    }
    printf("]\nV=[ ");
    for(unsigned int i=0;i<_velocity.size();i++){
        printf("%f ",_velocity[i]);
    }
    printf("]\nA=[ ");
    for(unsigned int i=0;i<_acceleration.size();i++){
        printf("%f ",_acceleration[i]);
    }
    printf("]\n");
}

}
