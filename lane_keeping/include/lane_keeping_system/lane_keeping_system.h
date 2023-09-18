/**
 * @file lane_keeping_system.h
 * @author Jongrok Lee (lrrghdrh@naver.com)
 * @brief Lane Keeping System Class header file
 * @version 0.2
 * @date 2022-11-27
 */
#ifndef LANE_KEEPING_SYSTEM_H_
#define LANE_KEEPING_SYSTEM_H_

#include <algorithm>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <xycar_msgs/xycar_motor.h>
#include <yolov3_trt_ros/BoundingBox.h>
#include <yolov3_trt_ros/BoundingBoxes.h>
#include <yaml-cpp/yaml.h>

#include "lane_keeping_system/hough_transform_lane_detector.h"
#include "lane_keeping_system/moving_average_filter.h"
#include "lane_keeping_system/pid_controller.h"

#include <unistd.h>


namespace xycar {

class LaneKeepingSystem {
public:
  // Construct a new Lane Keeping System object
  LaneKeepingSystem();
  // Destroy the Lane Keeping System object
  virtual ~LaneKeepingSystem();
  // Running Lane Keeping System Algorithm
  void run();
  // moving by detected object
  void drive_normal(std::string direction, float time);
  void drive_stop(std::string direction, float time, float skip_time);
  // void drive_stop(float time);
  // void drive_traffic(float time);
  // Set parameters from config file
  void setParams(const YAML::Node& config);

private:
  // Puslish Xycar Moter Message
  void drive(float steering_angle);

  // Subcribe Image Topic
  void imageCallback(const sensor_msgs::Image& msg);

  // Subcribe bbox Topic
  void bboxCallback(const yolov3_trt_ros::BoundingBoxes& msg);

  std::fstream outfile;

private:
  // Xycar Steering Angle Limit
  static const int kXycarSteeringAngleLimit = 50;
  // PID Class for Control
  PID *pid_ptr_;
  // Moving Average Filter Class for Noise filtering
  MovingAverageFilter *ma_filter_ptr_;
  // Hough Transform Lane Detector Class for Lane Detection
  HoughTransformLaneDetector *hough_transform_lane_detector_ptr_;

  // ROS Variables
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Subscriber bbox_sub_;
  std::string pub_topic_name_;
  std::string sub_topic_name_;
  std::string bbox_sub_topic_name_;
  int queue_size_;
  xycar_msgs::xycar_motor msg_;

  // OpenCV Image processing Variables
  cv::Mat frame_;

  // Xycar Device variables
  float xycar_speed_;
  float xycar_max_speed_;
  float xycar_min_speed_;
  float xycar_speed_control_threshold_;
  float acceleration_step_;
  float deceleration_step_;

  // object detection
  int object_id_ = -1;
  int xmin_;
  int xmax_;
  int ymin_;
  int ymax_;
  int traffic_sign_space_;
  bool bbox_flag_ = false;
  bool traffic_light_ = false;
  int red_count_ = 0;
  int green_count_ = 0;
  int stop_count_ = 0;
  //int traffic_sign_detect(cv::Mat frame, int xmin, int xmax, int ymin, int ymax);
  bool traffic_sign_detect(cv::Mat frame, int xmin, int xmax, int ymin, int ymax);

  // Debug Flag
  bool debug_;

};
}  // namespace xycar

#endif  // LANE_KEEPING_SYSTEM_H_
