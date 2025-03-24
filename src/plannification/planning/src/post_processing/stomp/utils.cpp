/**
 * @file utils.cpp
 * @brief This is a utility class for stomp
 *
 * @author Jorge Nicho
 * @date March 7, 2016
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2016, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "planning/post_processing/stomp/utils.h"
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

namespace stomp
{
void generateFiniteDifferenceMatrix(int num_time_steps,
                                    DerivativeOrders::DerivativeOrder order,
                                    double dt,
                                    Eigen::MatrixXd& diff_matrix)
{
  diff_matrix = Eigen::MatrixXd::Zero(num_time_steps, num_time_steps);
  double multiplier = 1.0 / pow(dt, (int)order);
  for (int i = 0; i < num_time_steps; ++i)
  {
    for (int j = -FINITE_DIFF_RULE_LENGTH / 2; j <= FINITE_DIFF_RULE_LENGTH / 2; ++j)
    {
      int index = i + j;
      if (index < 0)
      {
        index = 0;
        continue;
      }
      if (index >= num_time_steps)
      {
        index = num_time_steps - 1;
        continue;
      }

      diff_matrix(i, index) = multiplier * FINITE_CENTRAL_DIFF_COEFFS[order][j + FINITE_DIFF_RULE_LENGTH / 2];
    }
  }
}

void generateSmoothingMatrix(int num_timesteps, double dt, Eigen::MatrixXd& projection_matrix_M)
{
  using namespace Eigen;

  // generate augmented finite differencing matrix
  int start_index_padded = FINITE_DIFF_RULE_LENGTH - 1;
  int num_timesteps_padded = num_timesteps + 2 * (FINITE_DIFF_RULE_LENGTH - 1);
  MatrixXd finite_diff_matrix_A_padded;
  generateFiniteDifferenceMatrix(
      num_timesteps_padded, DerivativeOrders::STOMP_ACCELERATION, dt, finite_diff_matrix_A_padded);

  /* computing control cost matrix (R = A_transpose * A):
   * Note: Original code multiplies the A product by the time interval.  However this is not
   * what was described in the literature
   */
  MatrixXd control_cost_matrix_R_padded = dt * finite_diff_matrix_A_padded.transpose() * finite_diff_matrix_A_padded;
  MatrixXd control_cost_matrix_R =
      control_cost_matrix_R_padded.block(start_index_padded, start_index_padded, num_timesteps, num_timesteps);
  MatrixXd inv_control_cost_matrix_R = control_cost_matrix_R.fullPivLu().inverse();

  // computing projection matrix M
  projection_matrix_M = inv_control_cost_matrix_R;
  double max = 0;
  for (auto t = 0u; t < num_timesteps; t++)
  {
    max = projection_matrix_M(t, t);
    projection_matrix_M.col(t) *=
        (1.0 / (num_timesteps * max));  // scaling such that the maximum value is 1/num_timesteps
  }
}

void differentiate(const Eigen::VectorXd& parameters,
                   DerivativeOrders::DerivativeOrder order,
                   double dt,
                   Eigen::VectorXd& derivatives)
{
  using namespace Eigen;

  derivatives = Eigen::VectorXd::Zero(parameters.size());

  // coefficient arrays
  VectorXd central_coeffs = VectorXd::Map(&FINITE_CENTRAL_DIFF_COEFFS[order][0], FINITE_DIFF_RULE_LENGTH);
  VectorXd forward_coeffs = VectorXd::Map(&FINITE_FORWARD_DIFF_COEFFS[order][0], FINITE_DIFF_RULE_LENGTH);
  VectorXd backward_coeffs = forward_coeffs.reverse();

  // check order eveness
  if (order % 2 != 0)
  {
    backward_coeffs *= -1.0;
  }

  // creating difference matrix
  int rule_length = FINITE_DIFF_RULE_LENGTH;
  int size = parameters.size();
  int skip = FINITE_DIFF_RULE_LENGTH / 2;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(size, size);
  int start_ind, end_ind;
  for (auto i = 0; i < size; i++) 
  {
    if (i < skip)
    {
      start_ind = i;
      A.row(i).segment(start_ind, rule_length) = forward_coeffs;
    }
    else if (i >= skip && i < size - skip)
    {
      start_ind = i - skip;
      A.row(i).segment(start_ind, rule_length) = central_coeffs;
    }
    else
    {
      start_ind = i - rule_length + 1;
      A.row(i).segment(start_ind, rule_length) = backward_coeffs;
    }
  }

  derivatives = A * parameters / std::pow(dt, 2);
}

// Function to compute derivatives using finite differences
void custom_differentiate(const Eigen::VectorXd& parameters,
                   const Eigen::MatrixXd& times,
                   DerivativeOrders::DerivativeOrder order,
                   double dt,
                   Eigen::VectorXd& derivatives) 
{
    using namespace Eigen;

    int size = parameters.size();
    derivatives = VectorXd::Zero(size);  // Initialize the derivatives vector with zeros

    if (order == DerivativeOrders::STOMP_VELOCITY) 
    {
        // Compute velocity (1st derivative) with variable time steps
        for (int i = 1; i < size - 1; ++i) 
        {
            double dt_prev = times(i) - times(i - 1);
            double dt_next = times(i + 1) - times(i);
            double dt_total = dt_next + dt_prev;

            derivatives(i) = (parameters(i + 1) - parameters(i - 1)) / dt_total;
        }
    } 
    else if (order == DerivativeOrders::STOMP_ACCELERATION) 
    {
      Eigen::VectorXd vel = VectorXd::Zero(size);  // Initialize the derivatives vector with zeros

      // Compute velocity using central difference for interior points
      for (int i = 1; i < size - 1; ++i) 
      {
        double dt1 = times(i) - times(i - 1);
        double dt2 = times(i + 1) - times(i);
        vel(i) = (parameters(i + 1) - parameters(i - 1)) / (times(i + 1) - times(i - 1));
      }

      // Compute acceleration using central difference for interior points
      for (int i = 1; i < size - 1; ++i) 
      {
          derivatives(i) = (vel(i + 1) - vel(i - 1)) / (times(i + 1) - times(i - 1));
          // std::cout << "Acc stomp : " << derivatives(i) << std::endl;
          // Detect large jumps in acceleration
          // if (abs(derivatives(i) - derivatives(i - 1)) > 1.0) 
          // {
          //     std::cout << "Large difference in acceleration detected." << std::endl;
          //     std::cout << "Times: " << times(i - 1) << " " << times(i) << " " << times(i + 1) << std::endl;
          //     std::cout << "Parameters: " << parameters(i - 1) << " " << parameters(i) << " " << parameters(i + 1) << std::endl;
          // }
      }
      
      // // Compute acceleration (2nd derivative) with variable time steps
      // for (int i = 1; i < size - 1; ++i) 
      // {
      //     double dt_prev = times(i) - times(i - 1);
      //     double dt_next = times(i + 1) - times(i);

      //     // Handle very small time steps to avoid division by zero or instability
      //     if (dt_prev < 1e-6 || dt_next < 1e-6) {
      //         std::cout << "Warning: Very small time step detected at index " << i << std::endl;
      //         dt_prev = std::max(dt_prev, 1e-6);
      //         dt_next = std::max(dt_next, 1e-6);
      //     }

      //     // Compute velocity differences and average time step
      //     double vel_next = (parameters(i + 1) - parameters(i)) / dt_next;
      //     double vel_prev = (parameters(i) - parameters(i - 1)) / dt_prev;

      //     derivatives(i) = 2 * (vel_next - vel_prev) / (dt_next + dt_prev);

      //     std::cout << "Acc stomp : " << derivatives(i) << std::endl;
      //     // Detect large jumps in acceleration
      //     if (abs(derivatives(i) - derivatives(i - 1)) > 1.0) 
      //     {
      //         std::cout << "Large difference in acceleration detected." << std::endl;
      //         std::cout << "Times: " << times(i - 1) << " " << times(i) << " " << times(i + 1) << std::endl;
      //         std::cout << "Parameters: " << parameters(i - 1) << " " << parameters(i) << " " << parameters(i + 1) << std::endl;
      //     }
      // }
      
      // std::cout << "____________________" << std::endl;
    }
}

void toVector(const Eigen::MatrixXd& m, std::vector<Eigen::VectorXd>& v)
{
  v.resize(m.rows(), Eigen::VectorXd::Zero(m.cols()));
  for (auto d = 0u; d < m.rows(); d++)
  {
    v[d] = m.row(d);
  }
}

std::string toString(const std::vector<Eigen::VectorXd>& data)
{
  Eigen::IOFormat clean_format(4, 0, ", ", "\n", "[", "]");
  Eigen::MatrixXd m = Eigen::MatrixXd::Zero(data.size(), data.front().size());
  std::stringstream ss;
  for (auto d = 0u; d < data.size(); d++)
  {
    m.row(d) = data[d].transpose();
  }

  ss << m.format(clean_format);
  return ss.str();
}

std::string toString(const Eigen::MatrixXd& data)
{
  Eigen::IOFormat clean_format(4, 0, ", ", "\n", "[", "]");
  std::stringstream ss;
  ss << data.format(clean_format);
  return ss.str();
}

std::string toString(const Eigen::VectorXd& data)
{
  Eigen::IOFormat clean_format(4, 0, ", ", "\n", "[", "]");
  std::stringstream ss;
  ss << data.transpose().format(clean_format);
  return ss.str();
}

}  // namespace stomp
