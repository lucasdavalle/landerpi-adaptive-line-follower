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

#include "landerpi_adaptive_line_follower/line_tracker.hpp"

#include <cmath>

namespace landerpi_adaptive_line_follower
{

LineTracker::LineTracker(LineTrackerConfig config)
: config_(std::move(config))
{
}

void LineTracker::SetTargetColor(const cv::Vec3d & target_lab)
{
  config_.target_lab = target_lab;
}

void LineTracker::SetTolerance(double l_tolerance, double ab_tolerance)
{
  config_.l_tolerance = l_tolerance;
  config_.ab_tolerance = ab_tolerance;
}

BandDetection LineTracker::DetectBand(const cv::Mat & bgr_frame, const RoiBand & roi) const
{
  BandDetection detection;

  const int h = bgr_frame.rows;
  const int w = bgr_frame.cols;
  const int y0 = static_cast<int>(roi.y_start_frac * h);
  const int y1 = static_cast<int>(roi.y_end_frac * h);
  if (y0 < 0 || y1 > h || y1 <= y0 || w <= 0) {
    return detection;
  }

  const cv::Mat band = bgr_frame(cv::Range(y0, y1), cv::Range(0, w));
  cv::Mat lab;
  cv::cvtColor(band, lab, cv::COLOR_BGR2Lab);
  cv::GaussianBlur(lab, lab, cv::Size(3, 3), 3);

  const cv::Scalar lower(
    config_.target_lab[0] - config_.l_tolerance,
    config_.target_lab[1] - config_.ab_tolerance,
    config_.target_lab[2] - config_.ab_tolerance);
  const cv::Scalar upper(
    config_.target_lab[0] + config_.l_tolerance,
    config_.target_lab[1] + config_.ab_tolerance,
    config_.target_lab[2] + config_.ab_tolerance);

  cv::Mat mask;
  cv::inRange(lab, lower, upper, mask);

  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::erode(mask, mask, kernel);
  cv::dilate(mask, mask, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  double best_area = config_.min_contour_area;
  const std::vector<cv::Point> * best_contour = nullptr;
  for (const auto & contour : contours) {
    const double area = std::fabs(cv::contourArea(contour));
    if (area > best_area) {
      best_area = area;
      best_contour = &contour;
    }
  }

  if (best_contour == nullptr) {
    return detection;
  }

  const cv::RotatedRect rect = cv::minAreaRect(*best_contour);
  detection.found = true;
  detection.x_px = rect.center.x;
  return detection;
}

LineTrackerResult LineTracker::Track(const cv::Mat & bgr_frame) const
{
  LineTrackerResult result;
  result.near_band = DetectBand(bgr_frame, config_.near_roi);
  result.mid_band = DetectBand(bgr_frame, config_.mid_roi);
  result.far_band = DetectBand(bgr_frame, config_.far_roi);

  const double image_center_x = bgr_frame.cols / 2.0;

  if (result.near_band.found) {
    result.line_found = true;
    result.offset_px = result.near_band.x_px - image_center_x;
  } else if (result.mid_band.found) {
    result.line_found = true;
    result.offset_px = result.mid_band.x_px - image_center_x;
  } else if (result.far_band.found) {
    result.line_found = true;
    result.offset_px = result.far_band.x_px - image_center_x;
  } else {
    return result;
  }

  if (result.near_band.found && result.mid_band.found && result.far_band.found) {
    const int h = bgr_frame.rows;
    const double near_y = 0.5 * (config_.near_roi.y_start_frac + config_.near_roi.y_end_frac) * h;
    const double mid_y = 0.5 * (config_.mid_roi.y_start_frac + config_.mid_roi.y_end_frac) * h;
    const double far_y = 0.5 * (config_.far_roi.y_start_frac + config_.far_roi.y_end_frac) * h;

    const double near_to_mid_angle = std::atan2(
      result.mid_band.x_px - result.near_band.x_px, near_y - mid_y);
    const double mid_to_far_angle = std::atan2(
      result.far_band.x_px - result.mid_band.x_px, mid_y - far_y);

    result.curvature_rad = mid_to_far_angle - near_to_mid_angle;
  }

  return result;
}

}  // namespace landerpi_adaptive_line_follower
