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

#include <learning_sensitivity/SensitivityNN.h>

SensitivityNN::SensitivityNN(const ros::NodeHandle& node_handler) : nh_(node_handler)
{
  // Get the current CAMP path 
  std::filesystem::path campDir_;
  std::filesystem::path sourceFilePath = __FILE__; 
  std::filesystem::path currentDir = sourceFilePath.parent_path();
  // Traverse upwards until the root of the filesystem
  while (currentDir.has_parent_path()) {
      if (currentDir.filename() == "CAMP") {  // Adjust this condition if "CAMP" is not the directory name
          campDir_ = currentDir;
          break;
      }
      currentDir = currentDir.parent_path();
  }

  if (campDir_.empty()) {
      ROS_ERROR("CAMP directory not found.");
  }

  if(!nh_.getParam("model/model_file", model_path_))
  {
    ROS_WARN("No model found !");
  }
  else
  {
    // Combine the CAMP directory with the relative file path
    std::filesystem::path fullPath = std::filesystem::absolute(campDir_ / model_path_);

    // Normalize the path to remove redundant ".." and "."
    fullPath = fullPath.lexically_normal();

    // Convert to string and use the full path
    model_path_ = fullPath.string();
  }
  
  // Load model params
  if(!nh_.getParam("model/device", device_))
    ROS_WARN("No device found !");

  if(!nh_.getParam("model/hidden_size", hidden_size_))
    ROS_WARN("No hidden_size found !");
  if(!nh_.getParam("model/nb_input", inputsize_))
    ROS_WARN("No nb_input found !");
  if(!nh_.getParam("model/nb_rq", rqsize_))
    ROS_WARN("No nb_rq found !");
  if(!nh_.getParam("model/nb_u", usize_))
    ROS_WARN("No nb_u found !");
  if(!nh_.getParam("model/nb_ru", rusize_))
    ROS_WARN("No nb_ru found !");
  if(!nh_.getParam("model/dt_train", dt_train_))
    ROS_WARN("No dt_train found !");

  outsize_ = rqsize_ + rusize_ + usize_;

  // Load outputs standardization coeffs
  if(!nh_.getParam("std_out_coeff_tubes",std_out_tubes_))
    ROS_WARN("std_out_coeff not found !");
  if(!nh_.getParam("mean_out_coeff_tubes",mean_out_tubes_))
    ROS_WARN("mean_out_coeff not found !");
  if(!nh_.getParam("quartile_out_coeff_tubes",quartile_out_tubes_))
  {
    ROS_WARN("quartile_out_coeff_tubes not found !");
    for(int i = 0; i<mean_out_tubes_.size(); i++)
      quartile_out_tubes_.push_back(0.0);
  }
  if(!nh_.getParam("min_out_coeff_tubes",min_out_tubes_))
  {
    ROS_WARN("min_out_coeff_tubes not found !");
    for(int i = 0; i<mean_out_tubes_.size(); i++)
      min_out_tubes_.push_back(0.0);
  }
  if(!nh_.getParam("max_out_coeff_tubes",max_out_tubes_))
  {
    ROS_WARN("max_out_coeff_tubes not found !");
    for(int i = 0; i<mean_out_tubes_.size(); i++)
      max_out_tubes_.push_back(0.0);
  }

  // Load inputs standardization coeffs
  if(!nh_.getParam("std_in_coeff_tubes",std_in_tubes_))
    ROS_WARN("std_in_coeff_tubes not found !");
  if(!nh_.getParam("mean_in_coeff_tubes",mean_in_tubes_))
    ROS_WARN("mean_in_coeff_tubes not found !");
  if(!nh_.getParam("min_in_coeff_tubes",min_in_tubes_))
    ROS_WARN("min_in_coeff_tubes not found !");
  if(!nh_.getParam("max_in_coeff_tubes",max_in_tubes_))
    ROS_WARN("max_in_coeff_tubes not found !");
  if(!nh_.getParam("Vmax",vmax_))
    ROS_WARN("Vmax not found !");
  vmin_ = -vmax_;
  if(!nh_.getParam("Wmax",wmax_))
    ROS_WARN("Wmax not found !");
  wmin_ = -wmax_;
  if(!nh_.getParam("Amax",amax_))
    ROS_WARN("Amax not found !");
  amin_ = -amax_;
  
  try 
  {
    torch::NoGradGuard no_grad;
    c10::intrusive_ptr<c10::ivalue::Tuple> out_tuple;
    std::vector<torch::jit::IValue> inputs;
    torch::Tensor x, h0, c0;

    // Load the network for tubes & actuators predictions.
    // #######################################################################################
    network_ = torch::jit::load(model_path_);
    network_.to(at::kFloat);
    network_.eval();
    if(device_ == "gpu")
      network_.to(torch::kCUDA);
    ROS_INFO("Sensitivity network succesfully loaded !");
    
    //Check the shape of the inputs
    x = torch::zeros({1,100,inputsize_});
    h0 = torch::zeros({1,1,hidden_size_});
    inputs.push_back(x);
    inputs.push_back(h0);
    
    if(device_ == "gpu")
    {
      // Ensure the tensors are on the GPU (CUDA)
      for (auto& input : inputs) {
        input = input.toTensor().to(torch::kCUDA);
      }
    }
    
    //warmup
    for (int i = 0; i < 300; ++i) 
    {
      out_tuple = network_.forward(inputs).toTuple();
    }
    // #######################################################################################

    ROS_INFO("Model ready !");
  }
  catch(ModelNNException mne){std::cout << mne.not_loaded() << std::endl;}
}

SensitivityNN::~SensitivityNN()
{
}

c10::intrusive_ptr<c10::ivalue::Tuple> SensitivityNN::predict(std::vector<torch::jit::IValue> &inputs)
{
  if(device_ == "gpu")
  {
    // Ensure the tensors are on the GPU (CUDA)
    for (auto& input : inputs) {
      input = input.toTensor().to(torch::kCUDA);
    }
  }
  return network_.forward(inputs).toTuple();
}