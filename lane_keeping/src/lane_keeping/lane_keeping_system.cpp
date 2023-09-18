/**
 * @file lane_keeping_system.cpp
 * @author Jiho Han (hanjiho97@naver.com)
 * @brief Lane Keeping System Class source file
 * @version 0.3
 * @date 2022-01-20
 */
#include "lane_keeping_system/lane_keeping_system.h"

namespace xycar {
LaneKeepingSystem::LaneKeepingSystem()
{
  std::string config_path;
  nh_.getParam("config_path", config_path);
  // std::cout << config_path << std::endl;
  YAML::Node config = YAML::LoadFile(config_path);
  // std::cout << config << std::endl;

  float p_gain, i_gain, d_gain;
  int sample_size;
  pid_ptr_ = new PID(config["PID"]["P_GAIN"].as<float>(),
                     config["PID"]["I_GAIN"].as<float>(),
                     config["PID"]["D_GAIN"].as<float>());
  ma_filter_ptr_ = new MovingAverageFilter(
    config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<int>());

  hough_transform_lane_detector_ptr_ = new HoughTransformLaneDetector(config);
  setParams(config);

  pub_ = nh_.advertise<xycar_msgs::xycar_motor>(pub_topic_name_, queue_size_);
  sub_ = nh_.subscribe(
    sub_topic_name_, queue_size_, &LaneKeepingSystem::imageCallback, this);
  bbox_sub_ = nh_.subscribe(
    bbox_sub_topic_name_, queue_size_, &LaneKeepingSystem::bboxCallback, this);
}

void LaneKeepingSystem::setParams(const YAML::Node &config)
{
  pub_topic_name_ = config["TOPIC"]["PUB_NAME"].as<std::string>();
  sub_topic_name_ = config["TOPIC"]["SUB_NAME"].as<std::string>();
  bbox_sub_topic_name_ = config["TOPIC"]["BBOX_SUB_NAME"].as<std::string>();
  queue_size_ = config["TOPIC"]["QUEUE_SIZE"].as<int>();

  xycar_speed_ = config["XYCAR"]["START_SPEED"].as<float>();
  xycar_max_speed_ = config["XYCAR"]["MAX_SPEED"].as<float>();
  xycar_min_speed_ = config["XYCAR"]["MIN_SPEED"].as<float>();
  xycar_speed_control_threshold_ =
    config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<float>();
  acceleration_step_ = config["XYCAR"]["ACCELERATION_STEP"].as<float>();
  deceleration_step_ = config["XYCAR"]["DECELERATION_STEP"].as<float>();
  debug_ = config["DEBUG"].as<bool>();
}

LaneKeepingSystem::~LaneKeepingSystem()
{
  delete ma_filter_ptr_;
  delete pid_ptr_;
  delete hough_transform_lane_detector_ptr_;
}

void LaneKeepingSystem::run()
{
  int lpos, rpos, error, ma_mpos;
  float steering_angle;
  while (ros::ok()) {
    ros::spinOnce();
    if (frame_.empty() || (bbox_flag_ == false))
    {
      continue;
    }

    if (object_id_ == 0)
    {
      drive_normal("left", 15.0);
      stop_count_ = 0;
    }
    else if (object_id_ == 1)
    {
      drive_normal("right", 15.0);
      stop_count_ = 0;
    }
    else if ((object_id_ == 2) || (object_id_ == 3))
    {
      drive_stop("straight", 6.0, 2.0);
    }
    else if (object_id_ == 4)
    {
      traffic_light_ = traffic_sign_detect(frame_, xmin_, xmax_, ymin_, ymax_);
      if (traffic_light_ == false)
      {
        ++red_count_;
        if (red_count_ > 3)
        {
          drive_stop("straight", 2.0, 0.0);
          green_count_ = 0;
        }
      }
      else
      {
        ++green_count_;
        if (green_count_ > 6)
        {
          drive_normal("go", 0.0);
          red_count_ = 0;
        }
      }
    }
    else
    {
      drive_normal("go", 0.0);
    }
    object_id_ = -1;
  }
}

void LaneKeepingSystem::drive_normal(std::string direction, float time) {
  int lpos, rpos, error, ma_mpos;
  float steering_angle = 0.0F;
    // Set the rate variable
  ros::Rate rate(30);

  float max_cnt;
  int cnt = 0;

  if (direction == "go")
  {  // "go" means straight drive.
    max_cnt = 1;
  }
  else
  {
    // Set the maximun number of iteration in while loop.
    max_cnt = static_cast<float>(30) * time;
  }

  while (static_cast<float>(cnt) < max_cnt)
  {
    ros::spinOnce();

    if ((object_id_ == 2) || (object_id_ == 3))
    {
      ++stop_count_;
      if (stop_count_ == 1)
      {
        if (direction == "left")
        {
          drive_stop("left", 6.0, 2.0);
        }
        else if (direction == "right")
        {
          drive_stop("right", 6.0, 2.0);
        }
        max_cnt -= 60;
      }
    }

    std::tie(lpos, rpos) =
      hough_transform_lane_detector_ptr_->getLanePosition(frame_);
    // Left or Right
    if (direction == "left")
    {
      rpos = lpos + 380;
    }
    else if (direction == "right")
    {
      lpos = rpos - 380;
    }

    ma_filter_ptr_->addSample((lpos + rpos) / 2);
    ma_mpos = ma_filter_ptr_->getWeightedMovingAverage();
    error = ma_mpos - frame_.cols / 2;
    steering_angle = std::max(-(float)kXycarSteeringAngleLimit,
                              std::min(pid_ptr_->getControlOutput(error),
                                       (float)kXycarSteeringAngleLimit));
    pid_ptr_->getAngle(steering_angle);
    drive(steering_angle);

    cnt++;
    if (debug_) {
      std::cout << "lpos: " << lpos << ", rpos: " << rpos << ", mpos: " << ma_mpos << std::endl;
      hough_transform_lane_detector_ptr_->draw_rectangles(lpos, rpos, ma_mpos);
      cv::imshow("debug",
                 *(hough_transform_lane_detector_ptr_->getDebugFrame()));
      cv::waitKey(1);
    }
    rate.sleep();
  }
}

void LaneKeepingSystem::drive_stop(std::string direction, float time, float skip_time)
{
  int lpos, rpos, error, ma_mpos;
  float steering_angle = 0.0F;
  int cnt = 0;
  float max_cnt;
  ros::Rate rate(30);
  max_cnt = static_cast<float>(30) * time;
  xycar_speed_ = 0;
  while (static_cast<float>(cnt) < max_cnt)
  {
    drive(steering_angle);
    cnt++;
    rate.sleep();
  }
  xycar_speed_ = -8;
  cnt = 0;
  max_cnt = static_cast<float>(30) * skip_time;
  while (static_cast<float>(cnt) < max_cnt)
  {
    ros::spinOnce();
    std::tie(lpos, rpos) =
      hough_transform_lane_detector_ptr_->getLanePosition(frame_);
    if (direction == "left")
    {
      rpos = lpos + 380;
    }
    else if (direction == "right")
    {
      lpos = rpos - 380;
    }

    ma_filter_ptr_->addSample((lpos + rpos) / 2);
    ma_mpos = ma_filter_ptr_->getWeightedMovingAverage();
    error = ma_mpos - frame_.cols / 2;
    steering_angle = std::max(-(float)kXycarSteeringAngleLimit,
                              std::min(pid_ptr_->getControlOutput(error),
                                       (float)kXycarSteeringAngleLimit));
    pid_ptr_->getAngle(steering_angle);
    drive(steering_angle);

    cnt++;
    if (debug_)
    {
      std::cout << "lpos: " << lpos << ", rpos: " << rpos << ", mpos: " << ma_mpos << std::endl;
      hough_transform_lane_detector_ptr_->draw_rectangles(lpos, rpos, ma_mpos);
      cv::imshow("debug",
                 *(hough_transform_lane_detector_ptr_->getDebugFrame()));
      cv::waitKey(1);
    }
    rate.sleep();
  }
}

void LaneKeepingSystem::imageCallback(const sensor_msgs::Image& msg)
{
  cv::Mat src = cv::Mat(msg.height,
                        msg.width,
                        CV_8UC3,
                        const_cast<uint8_t *>(&msg.data[0]),
                        msg.step);
  cv::cvtColor(src, frame_, cv::COLOR_RGB2BGR);
}

void LaneKeepingSystem::bboxCallback(const yolov3_trt_ros::BoundingBoxes& msg)
{
  float traffic_slope = 0.0F;
  bbox_flag_ = true;
  for (auto& bbox : msg.bounding_boxes)
  {
    xmax_ = bbox.xmax;
    xmin_ = bbox.xmin;
    ymax_ = bbox.ymax;
    ymin_ = bbox.ymin;
    traffic_sign_space_ = (xmax_ - xmin_) * (ymax_ - ymin_);
    traffic_slope = static_cast<float>(ymax_ - ymin_) / static_cast<float>(xmax_ - xmin_);
    if (((bbox.id == 0) || (bbox.id == 1)) && (traffic_sign_space_ >= 1500))
    {
      object_id_ = bbox.id;
    }
    else if (((bbox.id == 2) || (bbox.id == 3)) && (traffic_sign_space_ >= 3500))
    {
      object_id_ = bbox.id;
    }
    else if ((bbox.id == 4) && (traffic_sign_space_ >= 10000) && (traffic_sign_space_ < 25000) &&
    (traffic_slope < 2.5))
    {
      object_id_ = bbox.id;
    }
  }
}

bool LaneKeepingSystem::traffic_sign_detect(cv::Mat frame, int xmin, int xmax, int ymin, int ymax)
{
  xmin = std::min(416, std::max(xmin, 0));
  xmax = std::min(416, std::max(xmax, 0));
  ymin = std::min(416, std::max(ymin, 0));
  ymax = std::min(416, std::max(ymax, 0));
  double height_resize = 480.0 / 416.0;
  double width_resize = 640.0 / 416.0;
  xmin = static_cast<int>(xmin * width_resize);
  xmax = static_cast<int>(xmax * width_resize);
  ymin = static_cast<int>(ymin * height_resize);
  ymax = static_cast<int>(ymax * height_resize);
  cv::Rect rect(xmin, ymin, xmax - xmin, ymax - ymin);
  cv::Mat cropped_img = frame(rect);
  cv::Mat hsv_img;
	cv::cvtColor(cropped_img, hsv_img, cv::COLOR_BGR2HSV);

	cv::Mat red_mask, red_image;
	cv::Mat green_mask, green_image;

	cv::Scalar lower_red = cv::Scalar(160, 80, 80);//160
	cv::Scalar upper_red = cv::Scalar(180, 255, 255);

	cv::Scalar lower_green = cv::Scalar(60, 80, 100);
	cv::Scalar upper_green = cv::Scalar(100, 255, 255);

	cv::inRange(hsv_img, lower_red, upper_red, red_mask);
	cv::inRange(hsv_img, lower_green, upper_green,green_mask);

	cv::Mat dst;
	cv::bitwise_and(hsv_img, hsv_img, red_image, red_mask);
	cv::bitwise_and(hsv_img, hsv_img, green_image, green_mask);

	int red_result = 0;
	int green_result = 0;
	int result = 0;

	for (int y = 0; y < ymax - ymin; ++y)
	{
		for (int x = 0; x < xmax - xmin; ++x)
		{
			red_result += red_mask.at<uchar>(y, x);
			green_result += green_mask.at<uchar>(y, x);
		}
	}
	if (green_result > red_result)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void LaneKeepingSystem::drive(float steering_angle)
{
  xycar_msgs::xycar_motor motor_msg;
  if (abs(steering_angle) < 5)
  {
    steering_angle = 0;
  }
  motor_msg.angle = std::round(steering_angle);
  motor_msg.speed = std::round(xycar_speed_);
  pub_.publish(motor_msg);
}
}  // namespace xycar
