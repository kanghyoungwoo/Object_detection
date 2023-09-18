/**
 * @file hough_transform_lane_detector.cpp
 * @author Jiho Han (hanjiho97@naver.com)
 * @brief hough transform lane detector class source file
 * @version 0.3
 * @date 2023-01-20
 */
#include "lane_keeping_system/hough_transform_lane_detector.h"

namespace xycar
{
const double HoughTransformLaneDetector::kHoughRho = 1.0;
const double HoughTransformLaneDetector::kHoughTheta = CV_PI / 180.0;

HoughTransformLaneDetector::HoughTransformLaneDetector(
  const YAML::Node &config)
{
  set(config);
}

void HoughTransformLaneDetector::set(const YAML::Node &config)
{
  image_width_ = config["IMAGE"]["WIDTH"].as<int>();
  image_height_ = config["IMAGE"]["HEIGHT"].as<int>();
  roi_start_height_ = config["IMAGE"]["ROI_START_HEIGHT"].as<int>();
  roi_height_ = config["IMAGE"]["ROI_HEIGHT"].as<int>();
  canny_edge_low_threshold_ = config["CANNY"]["LOW_THRESHOLD"].as<int>();
  canny_edge_high_threshold_ = config["CANNY"]["HIGH_THRESHOLD"].as<int>();
  hough_line_slope_range_ = config["HOUGH"]["ABS_SLOPE_RANGE"].as<float>();
  hough_threshold_ = config["HOUGH"]["THRESHOLD"].as<int>();
  hough_min_line_length_ = config["HOUGH"]["MIN_LINE_LENGTH"].as<int>();
  hough_max_line_gap_ = config["HOUGH"]["MAX_LINE_GAP"].as<int>();
  debug_ = config["DEBUG"].as<bool>();

  l_weight_.reserve(lSampleSize_);
  for (uint8_t i = 1U; i <= lSampleSize_; ++i)
  {
    l_weight_.push_back(i*i);
  }

  r_weight_.reserve(rSampleSize_);
  for (uint8_t i = 1U; i <= rSampleSize_; ++i)
  {
    r_weight_.push_back(i*i);
  }
}

cv::Mat *HoughTransformLaneDetector::getDebugFrame() { return &debug_frame_; }

std::pair<float, float> HoughTransformLaneDetector::get_line_params(
  const std::vector<cv::Vec4i> &lines, const std::vector<int> &line_index)
{
  float x_sum = 0.0F;
  float y_sum = 0.0F;
  float slope_sum = 0.0F;
  float slope = 0.0F;
  float y_intercept = 0.0F;
  int16_t x1 = -1;
  int16_t y1 = -1;
  int16_t x2 = -1;
  int16_t y2 = -1;
  const auto lines_size = line_index.size();
  if (lines_size != 0U)
  {
    for (uint8_t i = 0; i < lines_size; ++i)
    {
      x1 = lines[line_index[i]][kHoughIndex::x1],
      y1 = lines[line_index[i]][kHoughIndex::y1];
      x2 = lines[line_index[i]][kHoughIndex::x2],
      y2 = lines[line_index[i]][kHoughIndex::y2];
      x_sum += static_cast<float>(x1 + x2);
      y_sum += static_cast<float>(y1 + y2);
      if (x2 == x1)
      {
        if (y1 > y2)
        {
          slope_sum += -30.0F;
        }
        else
        {
          slope_sum += 30.0F;
        }
      }
      else
      {
        slope_sum += static_cast<float>(y2 - y1) / static_cast<float>(x2 - x1);
      }
    }
    float x_avg;
    float y_avg;
    x_avg = x_sum / static_cast<float>(lines_size * 2);
    y_avg = y_sum / static_cast<float>(lines_size * 2);
    slope = slope_sum / static_cast<float>(lines_size);
    y_intercept = y_avg - slope * x_avg;
  }
  std::pair<float, float> slope_and_y_intercept(slope, y_intercept);
  return slope_and_y_intercept;
}

int HoughTransformLaneDetector::get_line_pos(
  const std::vector<cv::Vec4i> &lines,
  const std::vector<int> &line_index,
  const bool direction) {
  float m, b;
  float y, pos;
  std::tie(m, b) = get_line_params(lines, line_index);

  if (m == 0.0 && b == 0.0) {
    if (direction == kLeftLane)
    {
      if (invaild_path_flag)
      {
        pos = left_mean;
      }
      else
      {
        pos = 0.0F;
      }
    }
    else
    {
      if (invaild_path_flag)
      {
        pos = right_mean;
      }
      else
      {
        pos = 640.0f;
      }
    }
  }
  else
  {
    y = static_cast<float>(roi_height_) * 0.5;
    pos = (y - b) / m;
  }
  return std::round(pos);
}

std::pair<std::vector<int>, std::vector<int>>
HoughTransformLaneDetector::divideLines(const std::vector<cv::Vec4i> &lines) {
  int lines_size = lines.size();
  std::vector<int> left_line_index;
  std::vector<int> right_line_index;
  left_line_index.reserve(lines_size);
  right_line_index.reserve(lines_size);
  int x1, y1, x2, y2;
  float slope;
  float left_line_x_sum = 0.0f;
  float right_line_x_sum = 0.0f;
  float left_x_avg, right_x_avg;

  int max_left_x1;
  int max_left_x2 = 0;
  int max_left_index = -1;
  int min_right_x2;
  int min_right_x1 = 640;
  int min_right_index = -1;

  for (int i = 0; i < lines_size; ++i) {
    x1 = lines[i][kHoughIndex::x1], y1 = lines[i][kHoughIndex::y1];
    x2 = lines[i][kHoughIndex::x2], y2 = lines[i][kHoughIndex::y2];
    if (x2 - x1 == 0) {
      if (y1 > y2)
      {
        slope = -30.0f;
      }
      else
      {
        slope = 30.0f;
      }
    } else {
      slope = static_cast<float>(y2 - y1) / static_cast<float>(x2 - x1);
    }

    int x_mean = (x1+x2)/2;

    if ((slope < 0) && (x_mean <= image_width_ / 2) &&
      ((lpos_flag == false) || ((left_mean != -1) && (std::abs(left_mean - x_mean) < 50)) ||
      ((left_mean == -1))))
    {
      left_line_x_sum += static_cast<float>(x1 + x2) * 0.5;
      left_line_index.push_back(i);
    }
    else if ((0 < slope) && (x_mean >= image_width_ / 2) &&
      ((rpos_flag == false) || ((right_mean != -1) && (std::abs(right_mean - x_mean) < 50)) ||
      ((right_mean == -1))))
    {
      right_line_x_sum += static_cast<float>(x1 + x2) * 0.5;
      right_line_index.push_back(i);
    }
  }

  int left_lines_size = left_line_index.size();
  int right_lines_size = right_line_index.size();
  invaild_path_flag = false;
  if (left_lines_size != 0 && right_lines_size != 0)
  {
    left_x_avg = left_line_x_sum / left_lines_size;
    right_x_avg = right_line_x_sum / right_lines_size;
    if (left_x_avg > right_x_avg)
    {
      left_line_index.clear();
      right_line_index.clear();
      invaild_path_flag = true;
    }
  }

  return std::pair<std::vector<int>, std::vector<int>>(
    std::move(left_line_index), std::move(right_line_index));
}

std::pair<int, int> HoughTransformLaneDetector::getLanePosition(
  const cv::Mat &image) {
  cv::Mat gray_image;
  cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  cv::Mat blur_image;
  cv::GaussianBlur(gray_image, blur_image, cv::Size(5, 5), 1.5);
  cv::Mat canny_image;
  cv::Canny(blur_image,
            canny_image,
            canny_edge_low_threshold_,
            canny_edge_high_threshold_);
  roi_start_height_ = 360;
  cv::Mat roi =
    canny_image(cv::Rect(0, roi_start_height_, image_width_, roi_height_));
  cv::dilate(roi, roi, cv::Mat());
  cv::Mat mask_img = cv::imread("mask.png", cv::IMREAD_GRAYSCALE);
  cv::bitwise_and(roi, mask_img, roi);
  std::vector<cv::Vec4i> all_lines;
  cv::HoughLinesP(roi,
                  all_lines,
                  kHoughRho,
                  kHoughTheta,
                  hough_threshold_,
                  hough_min_line_length_,
                  hough_max_line_gap_);
  if (all_lines.size() == 0)
  {
    return std::pair<int, int>(0,image_width_);
  }

  std::vector<int> left_line_index, right_line_index;
  std::tie(left_line_index, right_line_index) =
    std::move(divideLines(all_lines));

  int lpos = get_line_pos(all_lines, left_line_index, kLeftLane);
  int rpos = get_line_pos(all_lines, right_line_index, kRightLane);

  if (debug_)
  {
    image.copyTo(debug_frame_);
    draw_lines(all_lines, left_line_index, right_line_index);
  }
  return HoughTransformLaneDetector::refine_LanePosition(lpos, rpos);
}

std::pair<int, int> HoughTransformLaneDetector::refine_LanePosition(int lpos, int rpos)
{
  if ((lpos <= 0) && (rpos < image_width_))
  {
    lpos = rpos - 380;
    l_samples_.clear();
    addRSample(rpos);
    right_mean = getRWeightedMovingAverage();
    lpos_flag = false;
    rpos_flag = true;
  }
  else if ((lpos > 0) && (rpos == image_width_))
  {
    rpos = lpos + 380;
    r_samples_.clear();
    addLSample(lpos);
    left_mean = getLWeightedMovingAverage();
    lpos_flag = true;
    rpos_flag = false;
  }
  else if (lpos > rpos)
  {
    lpos = left_mean;
    rpos = right_mean;
    addLSample(lpos);
    addRSample(rpos);
    left_mean = getLWeightedMovingAverage();
    right_mean =getRWeightedMovingAverage();
  }
  else if ((lpos <= 0) && (rpos >= image_width_)) {
    l_samples_.clear();
    r_samples_.clear();
    lpos = left_mean;
    rpos = right_mean;
    lpos_flag = false;
    rpos_flag = false;
  }
  else
  {
    addLSample(lpos);
    addRSample(rpos);
    left_mean = getLWeightedMovingAverage();
    right_mean =getRWeightedMovingAverage();
    lpos_flag = true;
    rpos_flag = true;
  }
  return std::pair<int, int>(lpos, rpos);
}

void HoughTransformLaneDetector::draw_lines(
  const std::vector<cv::Vec4i> &lines,
  const std::vector<int> &left_line_index,
  const std::vector<int> &right_line_index) {
  cv::Point2i pt1, pt2;
  cv::Scalar color;
  for (int i = 0; i < left_line_index.size(); ++i)
  {
    pt1 = cv::Point2i(
      lines[left_line_index[i]][kHoughIndex::x1],
      lines[left_line_index[i]][kHoughIndex::y1] + roi_start_height_);
    pt2 = cv::Point2i(
      lines[left_line_index[i]][kHoughIndex::x2],
      lines[left_line_index[i]][kHoughIndex::y2] + roi_start_height_);
    int r, g, b;
    r = static_cast<float>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
    g = static_cast<float>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
    b = static_cast<float>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
    color = std::move(cv::Scalar(b, g, r));
    cv::line(debug_frame_, pt1, pt2, color, kDebgLineWidth);
  }
  for (int i = 0; i < right_line_index.size(); ++i)
  {
    pt1 = cv::Point2i(
      lines[right_line_index[i]][kHoughIndex::x1],
      lines[right_line_index[i]][kHoughIndex::y1] + roi_start_height_);
    pt2 = cv::Point2i(
      lines[right_line_index[i]][kHoughIndex::x2],
      lines[right_line_index[i]][kHoughIndex::y2] + roi_start_height_);
    int r, g, b;
    r = static_cast<float>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
    g = static_cast<float>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
    b = static_cast<float>(std::rand()) / RAND_MAX * std::numeric_limits<uint8_t>::max();
    color = std::move(cv::Scalar(b, g, r));
    cv::line(debug_frame_, pt1, pt2, color, kDebgLineWidth);
  }
}

void HoughTransformLaneDetector::draw_rectangles(int lpos,
                                                 int rpos,
                                                 int ma_pos) {
  static cv::Scalar kCVRed(0, 0, 255);
  static cv::Scalar kCVGreen(0, 255, 0);
  static cv::Scalar kCVBlue(255, 0, 0);
  cv::rectangle(debug_frame_,
                cv::Point(lpos - kDebugRectangleHalfWidth,
                          kDebugRectangleStartHeight + roi_start_height_),
                cv::Point(lpos + kDebugRectangleHalfWidth,
                          kDebugRectangleEndHeight + roi_start_height_),
                kCVGreen,
                kDebgLineWidth);
  cv::rectangle(debug_frame_,
                cv::Point(rpos - kDebugRectangleHalfWidth,
                          kDebugRectangleStartHeight + roi_start_height_),
                cv::Point(rpos + kDebugRectangleHalfWidth,
                          kDebugRectangleEndHeight + roi_start_height_),
                kCVGreen,
                kDebgLineWidth);
  cv::rectangle(debug_frame_,
                cv::Point(ma_pos - kDebugRectangleHalfWidth,
                          kDebugRectangleStartHeight + roi_start_height_),
                cv::Point(ma_pos + kDebugRectangleHalfWidth,
                          kDebugRectangleEndHeight + roi_start_height_),
                kCVRed,
                kDebgLineWidth);
  cv::rectangle(debug_frame_,
                cv::Point(image_width_ / 2 - kDebugRectangleHalfWidth,
                          kDebugRectangleStartHeight + roi_start_height_),
                cv::Point(image_width_ / 2 + kDebugRectangleHalfWidth,
                          kDebugRectangleEndHeight + roi_start_height_),
                kCVBlue,
                kDebgLineWidth);
}

void HoughTransformLaneDetector::addLSample(int new_sample)
{
  l_samples_.push_back(new_sample);
  if (l_samples_.size() > lSampleSize_)
  {
    l_samples_.pop_front();
  }
}

void HoughTransformLaneDetector::addRSample(int new_sample)
{
  r_samples_.push_back(new_sample);
  if (r_samples_.size() > rSampleSize_)
  {
    r_samples_.pop_front();
  }
}

float HoughTransformLaneDetector::getLWeightedMovingAverage() {
  int sum = 0;
  int weight_sum = 0;
  for (uint8_t i = 0U; i < l_samples_.size(); ++i)
  {
    sum += l_samples_[i] * l_weight_[i];
    weight_sum += l_weight_[i];
  }
  if (weight_sum == 0)
  {
    return 0.0F;
  }
  return static_cast<float>(sum) / static_cast<float>(weight_sum);
}

float HoughTransformLaneDetector::getRWeightedMovingAverage() {
  int sum = 0;
  int weight_sum = 0;
  for (uint8_t i = 0U; i < r_samples_.size(); ++i)
  {
    sum += r_samples_[i] * r_weight_[i];
    weight_sum += r_weight_[i];
  }
  if (weight_sum == 0)
  {
   throw std::runtime_error("RWeight sum is zero");
  }
  return static_cast<float>(sum) / static_cast<float>(weight_sum);
}
}  // namespace xycar
