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

#include "landerpi_adaptive_line_follower/line_tracker.hpp"

namespace landerpi_adaptive_line_follower
{
namespace
{

constexpr int kWidth = 320;
constexpr int kHeight = 240;

cv::Vec3d BgrToLab(const cv::Scalar & bgr)
{
  cv::Mat sample(1, 1, CV_8UC3, bgr);
  cv::Mat lab;
  cv::cvtColor(sample, lab, cv::COLOR_BGR2Lab);
  const cv::Vec3b px = lab.at<cv::Vec3b>(0, 0);
  return cv::Vec3d(px[0], px[1], px[2]);
}

LineTrackerConfig MakeConfig()
{
  LineTrackerConfig config;
  config.target_lab = BgrToLab(cv::Scalar(0, 0, 255));  // pure red line on white floor
  config.l_tolerance = 30.0;
  config.ab_tolerance = 20.0;
  config.min_contour_area = 30.0;
  return config;
}

// Draws a vertical strip of the target color spanning [y0, y1), centered at
// center_x, `width` pixels wide, so its centroid lands exactly at center_x.
void DrawLineSegment(cv::Mat & frame, int center_x, int y0, int y1, int width = 20)
{
  cv::rectangle(
    frame,
    cv::Point(center_x - width / 2, y0),
    cv::Point(center_x + width / 2, y1),
    cv::Scalar(0, 0, 255), cv::FILLED);
}

int BandPixelY0(const RoiBand & roi) {return static_cast<int>(roi.y_start_frac * kHeight);}
int BandPixelY1(const RoiBand & roi) {return static_cast<int>(roi.y_end_frac * kHeight);}

TEST(LineTrackerTest, StraightCenteredLineHasZeroOffsetAndZeroCurvature)
{
  const LineTrackerConfig config = MakeConfig();
  cv::Mat frame(kHeight, kWidth, CV_8UC3, cv::Scalar(255, 255, 255));
  DrawLineSegment(frame, kWidth / 2, BandPixelY0(config.far_roi), BandPixelY1(config.near_roi));

  const LineTracker tracker(config);
  const LineTrackerResult result = tracker.Track(frame);

  ASSERT_TRUE(result.line_found);
  ASSERT_TRUE(result.near_band.found);
  ASSERT_TRUE(result.mid_band.found);
  ASSERT_TRUE(result.far_band.found);
  EXPECT_NEAR(result.offset_px, 0.0, 2.0);
  EXPECT_NEAR(result.curvature_rad, 0.0, 1e-6);
}

TEST(LineTrackerTest, CurveAheadIsDetectedAsNonZeroCurvature)
{
  const LineTrackerConfig config = MakeConfig();
  cv::Mat frame(kHeight, kWidth, CV_8UC3, cv::Scalar(255, 255, 255));
  // Robot is currently aligned (near/mid both centered), but the line
  // curves to the right further ahead (far band shifted).
  const int near_x = kWidth / 2;
  const int mid_x = kWidth / 2;
  const int far_x = kWidth / 2 + 60;
  DrawLineSegment(frame, near_x, BandPixelY0(config.near_roi), BandPixelY1(config.near_roi));
  DrawLineSegment(frame, mid_x, BandPixelY0(config.mid_roi), BandPixelY1(config.mid_roi));
  DrawLineSegment(frame, far_x, BandPixelY0(config.far_roi), BandPixelY1(config.far_roi));

  const LineTracker tracker(config);
  const LineTrackerResult result = tracker.Track(frame);

  ASSERT_TRUE(result.line_found);
  ASSERT_TRUE(result.near_band.found);
  ASSERT_TRUE(result.mid_band.found);
  ASSERT_TRUE(result.far_band.found);
  EXPECT_NEAR(result.offset_px, 0.0, 2.0);
  // near->mid heading is ~0 (aligned); mid->far heading points right, so
  // the curvature (their difference) should be a clearly positive angle.
  EXPECT_GT(result.curvature_rad, 0.3);
}

TEST(LineTrackerTest, NoMatchingColorMeansLineNotFound)
{
  const LineTrackerConfig config = MakeConfig();
  cv::Mat frame(kHeight, kWidth, CV_8UC3, cv::Scalar(255, 255, 255));  // plain white floor

  const LineTracker tracker(config);
  const LineTrackerResult result = tracker.Track(frame);

  EXPECT_FALSE(result.line_found);
  EXPECT_FALSE(result.near_band.found);
  EXPECT_FALSE(result.mid_band.found);
  EXPECT_FALSE(result.far_band.found);
  EXPECT_DOUBLE_EQ(result.offset_px, 0.0);
  EXPECT_DOUBLE_EQ(result.curvature_rad, 0.0);
}

TEST(LineTrackerTest, OnlyNearBandFoundStillReportsOffsetButNoCurvature)
{
  const LineTrackerConfig config = MakeConfig();
  cv::Mat frame(kHeight, kWidth, CV_8UC3, cv::Scalar(255, 255, 255));
  // Sharp curve / occlusion: only the near band sees the line, mid and far
  // don't have enough valid data to tell a curve from noise.
  DrawLineSegment(
    frame, kWidth / 2 + 15, BandPixelY0(config.near_roi), BandPixelY1(config.near_roi));

  const LineTracker tracker(config);
  const LineTrackerResult result = tracker.Track(frame);

  ASSERT_TRUE(result.line_found);
  EXPECT_TRUE(result.near_band.found);
  EXPECT_FALSE(result.mid_band.found);
  EXPECT_FALSE(result.far_band.found);
  EXPECT_NEAR(result.offset_px, 15.0, 2.0);
  EXPECT_DOUBLE_EQ(result.curvature_rad, 0.0);
}

}  // namespace
}  // namespace landerpi_adaptive_line_follower
