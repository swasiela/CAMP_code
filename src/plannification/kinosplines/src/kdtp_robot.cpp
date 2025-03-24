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

#include "kinosplines/kdtp_robot.h"

namespace kdtp {


Robot::Robot(const char *name){
    _name=std::string(name);
}

Robot::Robot(std::string name){
    _name=name;
}

std::string Robot::getName(){
    return _name;
}

unsigned int Robot::getNbDof(){
    return _dofs.size();
}

DofPtr Robot::getDof(unsigned int i){
    if(_dofs.empty()){
        printf("error: kdtp::Robot::getDof: _dofs is empty\n");
        return DofPtr();
    }
    if(i>=_dofs.size())
        return _dofs[_dofs.size()-1];
    return _dofs[i];
}

void Robot::addDof(DofPtr dof){
    _dofs.push_back(dof);
}

void Robot::setName(char *name){
    _name=std::string(name);
}

void Robot::setName(std::string name){
    _name=name;
}

void Robot::print(){
    printf("---------------");
    for(unsigned int i=0;i<_name.size();i++){
        printf("-");
    }
    printf("--\n");
    printf("| kdtp::Robot: %s |\n",_name.c_str());
    printf("-------------------------------------------------------------------\n");
    printf("| dof |  pmin  |  pmax  |  vmax  |  amax  |  jmax  |  smax  | R/T |\n");
    printf("-------------------------------------------------------------------\n");
    unsigned int nbDof=_dofs.size();
    for(unsigned int i=0;i<nbDof;i++){
        DofPtr dof=_dofs[i];
        printf("| ");
        if(i<100){
            if(nbDof>99)
                printf("0");
            else
                printf(" ");
        }
        if(i<10){
            if(nbDof>9)
                printf("0");
            else
                printf(" ");
        }
        printf("%d |",(int)i);
        double values[6]={dof->getPositionMin(),
                          dof->getPositionMax(),
                          dof->getVelocityMax(),
                          dof->getAccelerationMax(),
                          dof->getJerkMax(),
                          dof->getSnapMax()};
        for(int j=0;j<6;j++){
            double val=values[j];
            if(val>=0)
                printf(" ");
            if(fabs(val)>=100)
                printf("%0.2f |",val);
            else if(fabs(val)>=10)
                printf("%0.3f |",val);
            else
                printf("%0.4f |",val);
        }
        char c='T';
        if(dof->isRotation())
            c='R';
        printf("  %c  |\n",c);
    }
    printf("-------------------------------------------------------------------\n");
}

}
