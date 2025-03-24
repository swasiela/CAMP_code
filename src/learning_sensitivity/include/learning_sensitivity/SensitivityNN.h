/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2024, LAAS-CNRS
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Simon WASIELA */

#ifndef SENSITIVITY_NN_
#define SENSITIVITY_NN_

#include <torch/script.h> // One-stop header.
#include <torch/torch.h>
#include <iostream>
#include <filesystem>  // C++17 and later
#include "ros/ros.h"

class SensitivityNN
{
  public:

    SensitivityNN(const ros::NodeHandle& node_handler);
    ~SensitivityNN();

    c10::intrusive_ptr<c10::ivalue::Tuple> predict(std::vector<torch::jit::IValue> &inputs);
    
    // Get the standardization coeffs
    double getStdInTubes(int i) const
    {
      return std_in_tubes_.at(i);
    }
    double getMeanInTubes(int i) const
    {
      return mean_in_tubes_.at(i);
    }
    double getStdOutTubes(int i)
    {
      return std_out_tubes_.at(i);
    }
    double getMeanOutTubes(int i)
    {
      return mean_out_tubes_.at(i);
    }
    double getQuartileTubes(int i)
    {
      return quartile_out_tubes_.at(i);
    }

    // Get the min max scale values
    double getVmax()
    {
      return vmax_;
    }
    double getVmin()
    {
      return vmin_;
    }
    double getWmax()
    {
      return wmax_;
    }
    double getWmin()
    {
      return wmin_;
    }
    double getAmax()
    {
      return amax_;
    }
    double getAmin()
    {
      return amin_;
    }

    double getMinInTubes(int i) const
    {
      return min_in_tubes_.at(i);
    }
    double getMaxInTubes(int i) const
    {
      return max_in_tubes_.at(i);
    }
    
    double getMinOutTubes(int i)
    {
      return min_out_tubes_.at(i);
    }
    double getMaxOutTubes(int i)
    {
      return max_out_tubes_.at(i);
    }

    // Get the model parameters (hidden size, input size, etc.)
    int getHiddenSize()
    {
      return hidden_size_;
    }
    int getInputSize()
    {
      return inputsize_;
    }
    int getRqSize()
    {
      return rqsize_;
    }
    int getUSize()
    {
      return usize_;
    }
    int getRuSize()
    {
      return rusize_;
    }
    int getOutSize()
    {
      return outsize_;
    }
    double getDtTrain()
    {
      return dt_train_;
    }

  private:

    /// @brief Path of the CAMP repository
    std::filesystem::path campDir_;

    /// @brief GRU model file path
    std::string model_path_;

    /// @brief The devic to run the model, cpu or gpu
    std::string device_;

    /// @brief Loaded network
    torch::jit::script::Module network_;

    /// @brief Maximum and minimum values used for the min max scaling
    double vmax_, amax_, wmax_, vmin_, amin_, wmin_;

    /// @brief Mean and Std of the output and input vector components used for the standardization during training
    std::vector<double> std_in_tubes_, mean_in_tubes_, std_out_tubes_, mean_out_tubes_, quartile_out_tubes_, min_out_tubes_, max_out_tubes_, min_in_tubes_, max_in_tubes_;

    /// @brief Number af radii along the state space (rqsize_), the control input space (rusize_), and the number of control inputs (usize_) to be predicted. Number of NN inputs. Hidden size.
    int hidden_size_, inputsize_, rqsize_, rusize_, usize_, outsize_;

    /// @brief The time step used for training the recurrent neural network. Use to make sure you're using the NN at the same time step as the one use for trainning.
    double dt_train_;

    // Node handler
    ros::NodeHandle nh_;
};
typedef std::shared_ptr<SensitivityNN> SensitivityNNPtr;

class ModelNNException : public std::exception 
{
  public:
    std::string not_loaded () 
    {
        return "Model network not loaded !";
    }
};

#endif


