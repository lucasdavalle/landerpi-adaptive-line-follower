// Copyright 2026 ldavalle
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "landerpi_adaptive_line_follower/pid.hpp"

#include <algorithm>

namespace landerpi_adaptive_line_follower
{

Pid::Pid(PidConfig config)
: config_(config)
{
}

double Pid::Update(double error, double dt_s)
{
  if (dt_s <= 0.0) {
    return std::min(std::max(config_.kp * error, config_.output_min), config_.output_max);
  }

  integral_ += error * dt_s;
  integral_ = std::min(std::max(integral_, config_.integral_min), config_.integral_max);

  const double derivative = has_previous_error_ ? (error - previous_error_) / dt_s : 0.0;
  previous_error_ = error;
  has_previous_error_ = true;

  const double output = config_.kp * error + config_.ki * integral_ + config_.kd * derivative;
  return std::min(std::max(output, config_.output_min), config_.output_max);
}

void Pid::Reset()
{
  integral_ = 0.0;
  previous_error_ = 0.0;
  has_previous_error_ = false;
}

}  // namespace landerpi_adaptive_line_follower
