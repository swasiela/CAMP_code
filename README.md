# CAMP

**WARNING**: We are aware of the unstable behavior of the quadrotor robot, leading to undesirable oscillatory behavior on the control inputs and tubes, making the learned model not super accurate during planning. We are trying to correct it.

## Description

This repository contains all the code needed to reproduce the results presented in the three references cited below. 

This project is based on **[OMPL](https://github.com/ompl/ompl)**.

## Installation

### Minimum requirement

**WARNING**: Ubuntu 20.04 is required as this project uses ROS1.
> If you're using a **virtual machine**, please choose one that allows **GPU pass-through**. Otherwise, please turn off the visualization option when planning, and perform your learning operations on the CPU.

### Install

First clone the project : ```git clone https://github.com/swasiela/CAMP_code.git``` and rename the *code* folder into *CAMP*. This step is not optional, as most paths in the code are relative to the string *CAMP*.

Then:

- Install all required system dependencies from scratch. See **[install](INSTALL.md)**.

- **OR** use **[apptainer](https://github.com/swasiela/CAMP_apptainer)**. 

### Building the project

You can build the project either in *release* or in *debug* mode. The following instructions are designed to build the project in *release* mode. To build in *debug* mode, replace the *Release* tag with *Debug*.

- Once you have followed the installation steps open a new terminal and *cd* to the CAMP folder.

- Start by building the *ompl* package by running:
```
catkin build ompl -j6 -DCMAKE_BUILD_TYPE=Release
```
> The *-j6* arguments limit the number of jobs that can be run at the same time, but you can increase this to suit your computer's capacity.
> Note that this step may take several minutes.

- Build the other packages by running:
```
catkin build -j6 -DCMAKE_BUILD_TYPE=Release
```

- Compile the robot models you will use before planning by running:
```
python3 src/robots/scripts/models/quadrotor_model_CoM.py 
python3 src/robots/scripts/models/quadrotor_model_CoM_dynamic.py 
```
> Note that this step will no longer be necessary after the first model compilation, as models can be overwritten.

## Getting started

At this stage, everything should be installed without error. If this is not the case, you will not be able to perform the following actions.

### Repository organization

This section explains how packages are organized and what you can find in them.

- blender_model

This folder contains environment models, robots models and 3D animated results plotters. 

- libtorch

This folder contains the sources and executables for **[libtorch](https://pytorch.org/cppdocs/installing.html)**. 

> **IMPORTANT** We're providing the sources and executables we've used directly because the current C++ distribution is conflicting with ROS. Furthermore, this allows a better reproducibility of the learning results.

- learning_sensitivity

This package contains neural network models, as well as various training, processing and data analysis scripts.

- ompl

This package contains the modified **[OMPL](https://github.com/ompl/ompl)** sources on which our algorithms are based.

- plannification

This package is the core of the planning process and is composed of the following sub-packages:

    - kinosplines: The kinospline library which is one of the steering methods used in planning.
    - planning: The core of the software. It contains the state spaces, planners, collision checking, ect.
    - utils: Contains usefull function like ellipsoid projection.

- plots

This folder contains all the python plotters. These are simple 2D plotters, consider using blender to access animated 3D plotters.

- results

The results folder.

- robots

Package implementing robots configuration, models and urdf. 

- services_msgs

Package for ROS messages between planning and robot model.

- simulation_genom

Package to run simulation using the genom3 architecture that was used for experimental validation. Running such simulation is optional as a simulation step is already available at the end of the planning process.

### Run minimal example

To run the quadrotor example in the window environment open a terminal and *cd* to the CAMP workspace. Then source it by running:

```
source devel/setup.bash
```

Then launch the planning by running:

```
roslaunch planning plan_quadrotor.launch
```

Follow the instructions on the screen.

Once the planning is done, you can display the results either using a simple python plotter by running:

```
python3 src/plots/plot_simu_tube.py Quadrotor
```

>**(optional)** If you have installed blender and want a 3D animation, open a new terminal and run blender. 
>Then open the LAAS_indoor_window_video.blend file.
>Head to the scripting section and set your filename according to your path.
>Run the scripting section.

## Usage

This section is dedicated to an in-depth understanding of the different packages and how to plan beyond the minimal example.

### Learning sensitivity

>You can run the learning either on CPU or GPU. Due to the model's small size, no notable difference in inference time has been observed between the two.

>You can display the training curves by using Tensorboard using the following command: 
>
>```
>tensorboard --logdir models/"YOUR MODEL FOLDER LOGS"
>```

### Planning

### (**optional**) Simulation genom3

## Roadmap

This repository is no longer maintained.

## LICENSE

BSD-2-Clause

## REFERENCES

[1] Simon Wasiela, Paolo Robuffo Giordano, Juan Cortés and Thierry Siméon, "A Sensitivity-Aware Motion Planner (SAMP) to Generate Intrinsically-Robust Trajectories", in ICRA 2023.

[2] Simon Wasiela, Smail Ait Bouhsain, Marco Cognetti, Juan Cortés and Thierry Siméon, "Learning Uncertainty Tubes via Recurrent Neural Networks for Planning Robust Robot Motions", in ECAI 2024.

[3] Simon Wasiela, Marco Cognetti, Paolo Robuffo Giordano, Juan Cortés and Thierry Siméon, "Robust Motion Planning with Accuracy Optimization based on Learned Sensitivity Metrics", in RA-L 2024.

## ACKNOWLEDGEMENT
This work was supported by the project ANR-20-CE33-0003 ``CAMP'' and by the Chaire de Professeur Junior grant no. ANR-22-CPJ1-0064-01.
