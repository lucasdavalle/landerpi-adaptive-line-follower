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

#include <cmath>
#include <vector>

#include "landerpi_adaptive_line_follower/zone_monitor.hpp"

using landerpi_adaptive_line_follower::GuardResult;
using landerpi_adaptive_line_follower::ZoneConfig;
using landerpi_adaptive_line_follower::ZoneMonitor;

namespace
{

// -90 deg .. +90 deg sweep, 1 deg per sample (181 samples), matching a
// typical LaserScan layout.
constexpr float kAngleMin = static_cast<float>(-M_PI / 2.0);
constexpr float kAngleIncrement = static_cast<float>(M_PI / 180.0);
constexpr float kRangeMin = 0.1f;
constexpr float kRangeMax = 8.0f;

std::vector<float> makeScan(float default_range, int hit_index = -1, float hit_range = 0.0f)
{
  std::vector<float> ranges(181, default_range);
  if (hit_index >= 0) {
    ranges[static_cast<std::size_t>(hit_index)] = hit_range;
  }
  return ranges;
}

ZoneConfig makeConfig(std::size_t window)
{
  ZoneConfig config;
  config.zone_min_angle_deg = -15.0;
  config.zone_max_angle_deg = 15.0;
  config.zone_radius_m = 1.0;
  config.noise_filter_window = window;
  return config;
}

GuardResult runUpdate(ZoneMonitor & monitor, const std::vector<float> & scan)
{
  return monitor.update(scan, kAngleMin, kAngleIncrement, kRangeMin, kRangeMax);
}

}  // namespace

TEST(ZoneMonitor, EmptyZoneNeverTriggers)
{
  ZoneMonitor monitor(makeConfig(1));
  const auto scan = makeScan(kRangeMax);
  for (int i = 0; i < 5; ++i) {
    EXPECT_FALSE(runUpdate(monitor, scan).intrusion_detected);
  }
}

TEST(ZoneMonitor, ClearIntrusionTriggers)
{
  ZoneMonitor monitor(makeConfig(1));
  const auto scan = makeScan(kRangeMax, 90, 0.4f);
  const GuardResult result = runUpdate(monitor, scan);
  EXPECT_TRUE(result.intrusion_detected);
  EXPECT_NEAR(result.distance_m, 0.4, 1e-6);
  EXPECT_NEAR(result.angle_deg, 0.0, 1e-3);
}

TEST(ZoneMonitor, SingleFrameOutlierDoesNotTrigger)
{
  ZoneMonitor monitor(makeConfig(5));
  const auto clear_scan = makeScan(kRangeMax);
  const auto noisy_scan = makeScan(kRangeMax, 90, 0.05f);

  for (int i = 0; i < 4; ++i) {
    runUpdate(monitor, clear_scan);
  }
  EXPECT_FALSE(runUpdate(monitor, noisy_scan).intrusion_detected);
}

TEST(ZoneMonitor, SustainedIntrusionEventuallyTriggers)
{
  ZoneMonitor monitor(makeConfig(3));
  const auto intruding_scan = makeScan(kRangeMax, 90, 0.4f);

  GuardResult result;
  for (int i = 0; i < 3; ++i) {
    result = runUpdate(monitor, intruding_scan);
  }
  EXPECT_TRUE(result.intrusion_detected);
}

TEST(ZoneMonitor, OutsideZoneIgnored)
{
  ZoneMonitor monitor(makeConfig(1));
  const auto scan = makeScan(kRangeMax, 170, 0.2f);
  EXPECT_FALSE(runUpdate(monitor, scan).intrusion_detected);
}

TEST(ZoneMonitor, HandlesZeroTo360AngleConvention)
{
  // This robot's real lidar (LD19/MS200) publishes angle_min=0 and sweeps
  // up to ~2*pi instead of -pi..pi, so a point physically 60 deg to one
  // side shows up as a *raw* angle near 300 deg. ZoneMonitor must
  // normalize that back to -60 deg so a symmetric zone still catches it.
  ZoneConfig config;
  config.zone_min_angle_deg = -90.0;
  config.zone_max_angle_deg = 90.0;
  config.zone_radius_m = 1.0;
  config.noise_filter_window = 1;
  ZoneMonitor monitor(config);

  const float angle_min = 0.0f;
  const float angle_increment = static_cast<float>(M_PI / 180.0);
  std::vector<float> ranges(360, kRangeMax);
  ranges[300] = 0.4f;

  const GuardResult result =
    monitor.update(ranges, angle_min, angle_increment, kRangeMin, kRangeMax);
  EXPECT_TRUE(result.intrusion_detected);
  EXPECT_NEAR(result.angle_deg, -60.0, 1e-3);
}

TEST(ZoneMonitor, SetConfigResetsHistory)
{
  ZoneMonitor monitor(makeConfig(3));
  const auto intruding_scan = makeScan(kRangeMax, 90, 0.4f);
  runUpdate(monitor, intruding_scan);
  runUpdate(monitor, intruding_scan);

  monitor.setConfig(makeConfig(3));
  EXPECT_FALSE(runUpdate(monitor, intruding_scan).intrusion_detected);
}
