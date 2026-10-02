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

#pragma once

namespace landerpi_adaptive_line_follower
{

struct PidConfig
{
  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double output_min = -1.0;
  double output_max = 1.0;
  double integral_min = -1.0;
  double integral_max = 1.0;
};

// Minimal PID controller with output and integral clamping (anti-windup).
// Pure logic, no ROS dependency, so it is gtest-able on its own.
class Pid
{
public:
  explicit Pid(PidConfig config);

  // error: current error signal. dt_s: seconds since the previous Update()
  // call (<=0 disables the integral/derivative terms for that call, so a
  // single call after Reset() or a bad timestamp can't produce a spike).
  double Update(double error, double dt_s);
  void Reset();

private:
  PidConfig config_;
  double integral_ = 0.0;
  double previous_error_ = 0.0;
  bool has_previous_error_ = false;
};

}  // namespace landerpi_adaptive_line_follower
