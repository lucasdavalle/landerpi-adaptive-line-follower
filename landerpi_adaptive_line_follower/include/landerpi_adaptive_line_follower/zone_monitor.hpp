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
//
// Same class as landerpi_perimeter_guard's ZoneMonitor (B1) — copied
// rather than shared across repos since there's no common library package
// between the two portfolio projects yet. Used here for the lidar
// obstacle-stop cutoff (STOPPED_OBSTACLE), not perimeter watching, but the
// underlying problem (angular sector + distance threshold + noise
// filtering + LD19/MS200's 0-360 angle convention) is identical.

#pragma once

#include <cstddef>
#include <deque>
#include <vector>

namespace landerpi_adaptive_line_follower
{

struct ZoneConfig
{
  double zone_min_angle_deg = -15.0;
  double zone_max_angle_deg = 15.0;
  double zone_radius_m = 1.0;
  std::size_t noise_filter_window = 3;
};

struct GuardResult
{
  bool intrusion_detected = false;
  double distance_m = 0.0;
  double angle_deg = 0.0;
};

// Pure logic, no rclcpp dependency: watches a configurable angular sector of
// a laser scan and reports an intrusion once the median distance to the
// closest point in that sector, over the last noise_filter_window scans,
// drops below zone_radius_m.
class ZoneMonitor
{
public:
  explicit ZoneMonitor(const ZoneConfig & config);

  void setConfig(const ZoneConfig & config);
  const ZoneConfig & config() const;

  // ranges: LaserScan-style range array (meters). Readings that are
  // non-finite, non-positive, or outside [range_min, range_max] are ignored.
  GuardResult update(
    const std::vector<float> & ranges,
    float angle_min,
    float angle_increment,
    float range_min,
    float range_max);

  // Clears the temporal filtering history (call on (re)activation so a stale
  // reading from a previous run can't influence the first live detection).
  void reset();

private:
  struct Sample
  {
    double distance_m = 0.0;
    double angle_deg = 0.0;
  };

  bool angleInZone(double angle_deg) const;
  static Sample median(std::deque<Sample> samples);
  static double normalizeDegrees(double deg);

  ZoneConfig config_;
  std::deque<Sample> history_;
};

}  // namespace landerpi_adaptive_line_follower
