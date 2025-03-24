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

#include "kinosplines/kdtp_localpath.h"
#include <ros/ros.h>

namespace kdtp {

LocalPath::LocalPath(RobotPtr robot,StatePtr init,StatePtr end){
    if(!robot){
        printf("error: in kdtp::LocalPath constructor: robot parameter is ");
        printf("null\n");
        return;
    }
    _robot=robot;
    _init=init;
    _end=end;
    this->init();
}

void LocalPath::init(){
    double maximum_duration = 0;
    unsigned int nbDof=_robot->getNbDof();
    for(unsigned int i=0;i<nbDof;i++)
    {
        DofPtr dof=_robot->getDof(i);
        vector3 init=_init->getDofState(i);
        vector3 end=_end->getDofState(i);
        SplinePtr spline(new Spline(dof,init,end));
        if(maximum_duration<spline->getDuration())
            maximum_duration=spline->getDuration();
        _splines.push_back(spline);
    }
    _duration=maximum_duration;
    unsigned int nbSplines=_splines.size();
    for(unsigned int i=0;i<nbSplines;i++)
    {
        SplinePtr spline=_splines[i];
        spline->synchronize(_duration);
    }
}

double LocalPath::getDuration(){
    return _duration;
}

unsigned int LocalPath::getNbSplines(){
    return _splines.size();
}

std::vector<double> LocalPath::getPositionAt(double time){
    std::vector<double> position;
    unsigned int nbSplines=_splines.size();
    for(unsigned int i=0;i<nbSplines;i++){
        SplinePtr spline=_splines[i];
        position.push_back(spline->getPositionAt(time));
    }
    return position;
}

std::vector<double> LocalPath::getVelocityAt(double time){
    std::vector<double> velocity;
    unsigned int nbSplines=_splines.size();
    for(unsigned int i=0;i<nbSplines;i++){
        SplinePtr spline=_splines[i];
        velocity.push_back(spline->getVelocityAt(time));
    }
    return velocity;
}

std::vector<double> LocalPath::getAccelerationAt(double time){
    std::vector<double> acceleration;
    unsigned int nbSplines=_splines.size();
    for(unsigned int i=0;i<nbSplines;i++){
        SplinePtr spline=_splines[i];
        acceleration.push_back(spline->getAccelerationAt(time));
    }
    return acceleration;
}

std::vector<double> LocalPath::getJerkAt(double time){
    std::vector<double> jerk;
    unsigned int nbSplines=_splines.size();
    for(unsigned int i=0;i<nbSplines;i++){
        SplinePtr spline=_splines[i];
        jerk.push_back(spline->getJerkAt(time));
    }
    return jerk;
}

std::vector<double> LocalPath::getSnapAt(double time){
    std::vector<double> snap;
    unsigned int nbSplines=_splines.size();
    for(unsigned int i=0;i<nbSplines;i++){
        SplinePtr spline=_splines[i];
        snap.push_back(spline->getSnapAt(time));
    }
    return snap;
}

std::vector<std::vector<double> > LocalPath::getAllAt(double time){
    std::vector<std::vector<double> > all;
    unsigned int nbSplines=_splines.size();
    for(unsigned int i=0;i<nbSplines;i++){
        SplinePtr spline=_splines[i];
        all.push_back(spline->getAllAt(time));
    }
    return all;
}

StatePtr LocalPath::getStateAt(double time){
    StatePtr state(new State(_robot));
    unsigned int nbSplines=_splines.size();
    for(unsigned int i=0;i<nbSplines;i++){
        SplinePtr spline=_splines[i];
        vector3 dofState=spline->getStateAt(time);
        state->setPosition(i,dofState.x);
        state->setVelocity(i,dofState.y);
        state->setAcceleration(i,dofState.z);
    }
    return state;
}

void LocalPath::exportToFile(const char *filepath,double frequency,bool overwrite){
    if(frequency<0 || fabs(frequency)<EPSILON)
        return;
    FILE *fileout;
    if(overwrite)
        fileout=fopen(filepath,"w");
    else
        fileout=fopen(filepath,"a");
    if(!fileout)
        return;
    double dt=1/frequency;
    double tF=this->getDuration();
    for(double t=0;t<tF;t+=dt){
        fprintf(fileout,"%f ",t);
        std::vector<std::vector<double> > all=this->getAllAt(t);
        for(unsigned int i=0;i<all.size();i++){
            std::vector<double> state=all[i];
            for(unsigned int j=0;j<state.size();j++){
                fprintf(fileout,"%f ",state[j]);
            }
        }
        fprintf(fileout,"\n");
    }
    fprintf(fileout,"%f ",tF);
    std::vector<std::vector<double> > all=this->getAllAt(tF);
    for(unsigned int i=0;i<all.size();i++){
        std::vector<double> state=all[i];
        for(unsigned int j=0;j<state.size();j++){
            fprintf(fileout,"%f ",state[j]);
        }
    }
    fprintf(fileout,"\n");
    fclose(fileout);
}

void LocalPath::exportToMatlabFile(const char *filepath,double frequency){
    if(frequency<0 || fabs(frequency)<EPSILON)
        return;
    FILE *fileout=fopen(filepath,"w");

    if(!fileout)
        return;
    fprintf(fileout,"%c automatically generated by kdtp\n\nclear DATA\nDATA=[\n",'%');
    fclose(fileout);
    this->exportToFile(filepath,frequency,false);
    fileout=fopen(filepath,"a");
    if(!fileout)
        return;
    fprintf(fileout,"];\n");
    fclose(fileout);
}

}
