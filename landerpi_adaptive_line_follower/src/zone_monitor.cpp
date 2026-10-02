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

#include "landerpi_adaptive_line_follower/zone_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace landerpi_adaptive_line_follower
{

ZoneMonitor::ZoneMonitor(const ZoneConfig & config)
: config_(config)
{
}

void ZoneMonitor::setConfig(const ZoneConfig & config)
{
  config_ = config;
  history_.clear();
}

const ZoneConfig & ZoneMonitor::config() const
{
  return config_;
}

void ZoneMonitor::reset()
{
  history_.clear();
}

bool ZoneMonitor::angleInZone(double angle_deg) const
{
  if (config_.zone_min_angle_deg <= config_.zone_max_angle_deg) {
    return angle_deg >= config_.zone_min_angle_deg && angle_deg <= config_.zone_max_angle_deg;
  }
  // Sector wraps across the +/-180 deg boundary.
  return angle_deg >= config_.zone_min_angle_deg || angle_deg <= config_.zone_max_angle_deg;
}

double ZoneMonitor::normalizeDegrees(double deg)
{
  double result = std::fmod(deg + 180.0, 360.0);
  if (result < 0.0) {
    result += 360.0;
  }
  return result - 180.0;
}

ZoneMonitor::Sample ZoneMonitor::median(std::deque<Sample> samples)
{
  if (samples.empty()) {
    return Sample{std::numeric_limits<double>::infinity(), 0.0};
  }
  std::sort(
    samples.begin(), samples.end(),
    [](const Sample & a, const Sample & b) {return a.distance_m < b.distance_m;});
  const std::size_t n = samples.size();
  if (n % 2 == 1) {
    return samples[n / 2];
  }
  const Sample & lo = samples[n / 2 - 1];
  const Sample & hi = samples[n / 2];
  Sample result;
  result.distance_m = (lo.distance_m + hi.distance_m) / 2.0;
  result.angle_deg = lo.distance_m <= hi.distance_m ? lo.angle_deg : hi.angle_deg;
  return result;
}

GuardResult ZoneMonitor::update(
  const std::vector<float> & ranges,
  float angle_min,
  float angle_increment,
  float range_min,
  float range_max)
{
  double closest_range = std::numeric_limits<double>::infinity();
  double closest_angle_deg = 0.0;

  for (std::size_t i = 0; i < ranges.size(); ++i) {
    const float r = ranges[i];
    if (!std::isfinite(r) || r <= 0.0f || r < range_min || r > range_max) {
      continue;
    }
    const double angle_rad = static_cast<double>(angle_min) +
      static_cast<double>(i) * static_cast<double>(angle_increment);
    const double angle_deg = normalizeDegrees(angle_rad * 180.0 / M_PI);
    if (!angleInZone(angle_deg)) {
      continue;
    }
    if (r < closest_range) {
      closest_range = r;
      closest_angle_deg = angle_deg;
    }
  }

  const bool had_valid_point = std::isfinite(closest_range);
  Sample sample;
  sample.distance_m = had_valid_point ? closest_range : static_cast<double>(range_max);
  sample.angle_deg = closest_angle_deg;

  history_.push_back(sample);
  const std::size_t window = std::max<std::size_t>(config_.noise_filter_window, 1);
  while (history_.size() > window) {
    history_.pop_front();
  }

  const Sample filtered = median(history_);
  GuardResult result;
  result.distance_m = filtered.distance_m;
  result.angle_deg = filtered.angle_deg;
  result.intrusion_detected = history_.size() == window &&
    result.distance_m <= config_.zone_radius_m;
  return result;
}

}  // namespace landerpi_adaptive_line_follower
