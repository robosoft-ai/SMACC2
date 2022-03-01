// Copyright 2021 RobosoftAI Inc.
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

/*****************************************************************************************************************
 *
 * 	 Authors: Pablo Inigo Blasco, Brett Aldrich
 *
 ******************************************************************************************************************/

// #include <cv_bridge/cv_bridge.h>
#include <image_tools/cv_mat_sensor_msgs_image_type_adapter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "image_tools/cv_mat_sensor_msgs_image_type_adapter.hpp"

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  image_tools::ROSCvMatContainer,
  sensor_msgs::msg::Image);

rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr detectionPub;
rclcpp::Publisher<image_tools::ROSCvMatContainer>::SharedPtr debugImagePub;
rclcpp::Subscription<image_tools::ROSCvMatContainer>::SharedPtr imageSub;

void segmentColor(const cv::Mat& inputRGB, int hueMean, int hueWindow, cv::Mat& out)
{
  cv::Mat hsvInput;
  cv::cvtColor(inputRGB, hsvInput, cv::COLOR_BGR2HSV);
  cv::inRange(hsvInput, cv::Scalar(hueMean - hueWindow, 20, 20), cv::Scalar(hueMean + hueWindow, 255, 255), out);
  cv::threshold(out, out, 100, 255, cv::THRESH_BINARY);
}

int testImage(cv::Mat& input, cv::Mat& debugImage, std::string colorName, int hueMean, int hueWindow)
{
  cv::Mat segmented;

  segmentColor(input, hueMean, hueWindow, segmented);

  cv::SimpleBlobDetector::Params params;
  // params.filterByArea= true;
  // params.minArea = 10;
  params.filterByArea = true;
  params.minArea = 100;
  params.maxArea = 100000;
  params.filterByColor = true;
  params.blobColor = 255;

  auto blobDetector = cv::SimpleBlobDetector::create(params);
  std::vector<cv::KeyPoint> blobs;
  blobDetector->detect(segmented, blobs);

  std::cout << "-------" << std::endl;
  for (auto& b : blobs)
  {
    std::cout << "blob " << colorName << " detected: " << b.size << std::endl;
  }

  if(!blobs.empty())
  {
    for(auto& b: blobs)
    {
      cv::Rect r;
      float diameter = b.size;
      auto radius = diameter*0.5;
      r.x = b.pt.x - radius;
      r.y = b.pt.y - radius;
      r.width = diameter;
      r.height = diameter;
      cv::rectangle(debugImage,r, cv::Scalar(255,0,0),1 );
    }
  }

  // cv::imshow(colorName + " filter - "+ path, segmented);
  // cv::waitKey();

  return blobs.size();
}

int testRed(cv::Mat& input, cv::Mat& debugImage)
{
  return testImage(input, debugImage, "red", 130, 20);
}

int testBlue(cv::Mat& input, cv::Mat& debugImage)
{
  return testImage(input, debugImage, "blue", 10, 10);
}

int testGreen(cv::Mat& input, cv::Mat& debugImage)
{
  return testImage(input, debugImage, "green", 50, 10);
}

int testRed(std::string path, cv::Mat& debugImage)
{
  cv::Mat input = cv::imread(path);
  return testImage(input, debugImage, "red", 130, 20);
}

int testBlue(std::string path, cv::Mat& debugImage)
{
  cv::Mat input = cv::imread(path);
  return testImage(input, debugImage, "blue", 10, 10);
}

int testGreen(std::string path, cv::Mat& debugImage)
{
  cv::Mat input = cv::imread(path);
  return testImage(input, debugImage, "green", 50, 10);
}

void update()
{
}

void callback(const image_tools::ROSCvMatContainer& img)
{
  cv::Mat image = img.cv_mat();

  image_tools::ROSCvMatContainer outimg;
  auto outmat = outimg.cv_mat();

  int detectedColor = 0;
  if (testRed(image, outmat) > 0)
  {
    detectedColor = 1;
  }

  if (testGreen(image, outmat) > 0)
  {
    detectedColor = 2;
  }

  if (testBlue(image, outmat) > 0)
  {
    detectedColor = 3;
  }

  std_msgs::msg::Int32 detectedColorMsg;
  detectedColorMsg.data = detectedColor;
  detectionPub->publish(detectedColorMsg);


  outimg.header() = img.header();
  debugImagePub->publish(outimg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh;

  detectionPub = nh->create_publisher<std_msgs::msg::Int32>("detected_color", 1);
  imageSub = nh->create_subscription<image_tools::ROSCvMatContainer>("/image_raw", 1, callback);
  debugImagePub = nh->create_publisher<image_tools::ROSCvMatContainer>("/opencv_debug_image", 1);

  rclcpp::Rate r(10);

  while (!rclcpp::shutdown())
  {
    update();
    rclcpp::spin_some(nh);
    r.sleep();
  }

  /*
  testRed("../../red1.png");
  testRed("../../green1.png");
  testRed("../../blue1.png");
  testRed("../../red2.png");

  testGreen("../../red1.png");
  testGreen("../../green1.png");
  testGreen("../../blue1.png");
  testGreen("../../red2.png");

  testBlue("../../red1.png");
  testBlue("../../green1.png");
  testBlue("../../blue1.png");
  testBlue("../../red2.png");
  */
}
