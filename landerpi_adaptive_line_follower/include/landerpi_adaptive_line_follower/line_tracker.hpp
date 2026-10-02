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

#include <opencv2/opencv.hpp>

namespace landerpi_adaptive_line_follower
{

// A horizontal strip of the image, expressed as a fraction of image height
// (0.0 = top row, 1.0 = bottom row).
struct RoiBand
{
  double y_start_frac;
  double y_end_frac;
};

struct BandDetection
{
  bool found = false;
  double x_px = 0.0;
};

struct LineTrackerResult
{
  bool line_found = false;
  // Pixels, positive = line is to the right of image center. Taken from
  // the closest-to-robot band that detected the line (near, then mid,
  // then far).
  double offset_px = 0.0;
  // Radians. Difference between the near->mid and mid->far segment
  // headings: ~0 for a straight line, larger magnitude for a sharper
  // curve. Only computed when all three bands detect the line; 0.0
  // otherwise (not enough data to tell a curve from noise).
  double curvature_rad = 0.0;
  BandDetection near_band;
  BandDetection mid_band;
  BandDetection far_band;
};

struct LineTrackerConfig
{
  // Target line color in Lab space (as returned by cv::cvtColor with
  // COLOR_BGR2Lab): L in [0, 255], a/b in [0, 255] (128 = neutral).
  cv::Vec3d target_lab{0.0, 0.0, 0.0};
  double l_tolerance = 40.0;
  double ab_tolerance = 20.0;
  double min_contour_area = 30.0;
  // Ordered near (closest to the robot, largest y fraction) to far
  // (smallest y fraction) so segment headings in Track() point "ahead".
  RoiBand near_roi{0.85, 0.95};
  RoiBand mid_roi{0.55, 0.65};
  RoiBand far_roi{0.25, 0.35};
};

// Pure OpenCV line detector: no rclcpp dependency, so it can be gtest-ed
// directly on synthetic/saved frames without bringing up a ROS node.
//
// Samples the target color at three fixed-height horizontal bands and
// derives a steering offset (from the nearest band that saw the line) plus
// a curvature estimate (from how the line's x position changes across the
// three bands) — the two signals the node maps to angular and linear
// velocity respectively.
class LineTracker
{
public:
  explicit LineTracker(LineTrackerConfig config);

  LineTrackerResult Track(const cv::Mat & bgr_frame) const;

  void SetTargetColor(const cv::Vec3d & target_lab);
  void SetTolerance(double l_tolerance, double ab_tolerance);

private:
  BandDetection DetectBand(const cv::Mat & bgr_frame, const RoiBand & roi) const;

  LineTrackerConfig config_;
};

}  // namespace landerpi_adaptive_line_follower
