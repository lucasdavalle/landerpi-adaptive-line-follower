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

#include <gtest/gtest.h>

#include "landerpi_adaptive_line_follower/pid.hpp"

namespace landerpi_adaptive_line_follower
{
namespace
{

TEST(PidTest, ProportionalOnlyScalesError)
{
  PidConfig config;
  config.kp = 2.0;
  config.output_min = -100.0;
  config.output_max = 100.0;
  Pid pid(config);

  EXPECT_DOUBLE_EQ(pid.Update(3.0, 0.1), 6.0);
}

TEST(PidTest, OutputIsClamped)
{
  PidConfig config;
  config.kp = 10.0;
  config.output_min = -1.0;
  config.output_max = 1.0;
  Pid pid(config);

  EXPECT_DOUBLE_EQ(pid.Update(5.0, 0.1), 1.0);
  EXPECT_DOUBLE_EQ(pid.Update(-5.0, 0.1), -1.0);
}

TEST(PidTest, IntegralAccumulatesAndClamps)
{
  PidConfig config;
  config.ki = 1.0;
  config.integral_min = -5.0;
  config.integral_max = 5.0;
  config.output_min = -100.0;
  config.output_max = 100.0;
  Pid pid(config);

  EXPECT_DOUBLE_EQ(pid.Update(2.0, 1.0), 2.0);  // integral = 2
  EXPECT_DOUBLE_EQ(pid.Update(2.0, 1.0), 4.0);  // integral = 4
  EXPECT_DOUBLE_EQ(pid.Update(2.0, 1.0), 5.0);  // integral clamped to 5
}

TEST(PidTest, DerivativeRespondsToChangeButNotOnFirstCall)
{
  PidConfig config;
  config.kd = 1.0;
  config.output_min = -100.0;
  config.output_max = 100.0;
  Pid pid(config);

  EXPECT_DOUBLE_EQ(pid.Update(0.0, 1.0), 0.0);  // no previous error yet
  EXPECT_DOUBLE_EQ(pid.Update(5.0, 1.0), 5.0);  // (5 - 0) / 1.0
}

TEST(PidTest, ResetClearsIntegralAndDerivativeHistory)
{
  PidConfig config;
  config.kd = 1.0;
  config.ki = 1.0;
  config.output_min = -100.0;
  config.output_max = 100.0;
  Pid pid(config);

  pid.Update(5.0, 1.0);
  pid.Update(5.0, 1.0);
  pid.Reset();

  // Behaves like a fresh instance: no derivative kick, integral back at 0.
  EXPECT_DOUBLE_EQ(pid.Update(0.0, 1.0), 0.0);
}

TEST(PidTest, NonPositiveDtSkipsIntegralAndDerivative)
{
  PidConfig config;
  config.kp = 1.0;
  config.ki = 1.0;
  config.kd = 1.0;
  config.output_min = -100.0;
  config.output_max = 100.0;
  Pid pid(config);

  EXPECT_DOUBLE_EQ(pid.Update(3.0, 0.0), 3.0);  // only the kp term applies
}

}  // namespace
}  // namespace landerpi_adaptive_line_follower
