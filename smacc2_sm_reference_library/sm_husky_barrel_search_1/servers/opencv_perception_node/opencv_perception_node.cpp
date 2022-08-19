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

#ifdef ROS_ROLLING
#include <image_tools/cv_mat_sensor_msgs_image_type_adapter.hpp>
#else
#include "cv_mat_sensor_msgs_image_type_adapter.hpp"

#endif
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sm_husky_barrel_search_1/msg/detected_objects.hpp>

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "cv_mat_sensor_msgs_image_type_adapter.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

// RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
//   image_tools::sensor_msgs::msg::Image,
//   sensor_msgs::msg::Image);

rclcpp::Publisher<sm_husky_barrel_search_1::msg::DetectedObjects>::SharedPtr detectionPub;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePub;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub;

void segmentColor(const cv::Mat& inputRGB, int hueMean, int hueWindow, cv::Mat& out, int minSaturation = 20,
                  int minValue = 20, bool imgShow = false)
{
  cv::Mat hsvInput;
  cv::cvtColor(inputRGB, hsvInput, cv::COLOR_RGB2HSV);
  cv::inRange(hsvInput, cv::Scalar(hueMean - hueWindow, minSaturation, minValue),
              cv::Scalar(hueMean + hueWindow, 255, 255), out);
  cv::threshold(out, out, 100, 255, cv::THRESH_BINARY);

  if (imgShow)
  {
    cv::imshow("segment", out);
    cv::waitKey();
  }
}

#define DEFAULT_MIN_SATURATION 20
#define DEFAULT_MIN_VALUE 20
#define DEFAULT_MIN_BLOB_AREA 20

int testImage(cv::Mat& input, cv::Mat& debugImage, std::vector<sm_husky_barrel_search_1::msg::DetectedObject>& detectedObjects, std::string colorName,
              int hueMean, int hueWindow, std::string message = "", int minSaturation = DEFAULT_MIN_SATURATION,
              int minValue = DEFAULT_MIN_VALUE, int minBlobArea = DEFAULT_MIN_BLOB_AREA, bool imgShow = false)
{
  cv::Mat segmented;

  segmentColor(input, hueMean, hueWindow, segmented, minSaturation, minValue, imgShow);

  std::vector<std::vector<cv::Point> > contours;

  std::vector<cv::Vec4i> hierarchy;

  cv::Mat contourOutput = segmented.clone();
  cv::findContours(contourOutput, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

  int foundCount = 0;

  cv::Mat contourImage(segmented.size(), CV_8UC3, cv::Scalar(0, 0, 0));
  for (size_t idx = 0; idx < contours.size(); idx++)
  {
    auto currentCountour = contours[idx];
    auto area = cv::contourArea(currentCountour);
    auto currentHierarchy = hierarchy[idx];

    if (area > minBlobArea && currentHierarchy[3] < 0)
    {
      foundCount++;
      auto r = cv::boundingRect(currentCountour);
      cv::drawContours(contourImage, contours, idx, cv::Scalar(255, 0, 0), 1, cv::LINE_8, hierarchy, 0);

      cv::drawContours(debugImage, contours, idx, cv::Scalar(255, 0, 0), 1, cv::LINE_8, hierarchy, 0);
      cv::rectangle(debugImage, r, cv::Scalar(255, 0, 0), 1);
      cv::putText(debugImage, message, cv::Point(r.x, r.y), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Vec3b(255, 255, 255));

      sm_husky_barrel_search_1::msg::DetectedObject detectedObject;
      detectedObject.name = message;
      detectedObject.blob_area = area;
      detectedObjects.push_back(detectedObject);
    }
  }

  if (imgShow)
  {
    cv::imshow("contours", contourImage);
    cv::imshow("debugImage", debugImage);
  }

  return foundCount;

  // cv::SimpleBlobDetector::Params params;
  // // params.filterByArea= true;
  // // params.minArea = 10;
  // params.filterByArea = true;
  // params.minArea = minBlobArea;
  // params.maxArea = 100000;
  // params.filterByColor = false;
  // params.blobColor = 255;

  // auto blobDetector = cv::SimpleBlobDetector::create(params);
  // std::vector<cv::KeyPoint> blobs;
  // blobDetector->detect(segmented, blobs);

  // // std::cout << "-------" << std::endl;
  // for (auto& b : blobs)
  // {
  //   std::cout << "-blob " << colorName << " detected: " << b.size << std::endl;
  // }

  // if (!blobs.empty())
  // {
  //   for (auto& b : blobs)
  //   {
  //     cv::Rect r;
  //     float diameter = b.size;
  //     auto radius = diameter * 0.5;
  //     r.x = b.pt.x - radius;
  //     r.y = b.pt.y - radius;
  //     r.width = diameter;
  //     r.height = diameter;
  //     cv::rectangle(debugImage, r, cv::Scalar(255, 0, 0), 1);
  //     cv::putText(debugImage, message, cv::Point(r.x, r.y), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Vec3b(255, 255, 255));
  //   }
  // }

  // return blobs.size();
}

int testRed(cv::Mat& input, cv::Mat& debugImage, std::vector<sm_husky_barrel_search_1::msg::DetectedObject>& detectedObjects, bool imShow = false)
{
  return testImage(input, debugImage, detectedObjects, "red", 130, 20, "enemy", DEFAULT_MIN_SATURATION,
                   DEFAULT_MIN_VALUE, DEFAULT_MIN_BLOB_AREA, imShow);

  // RGB
  // return testImage(input, debugImage, "red", 10, 10, "enemy", DEFAULT_MIN_SATURATION, DEFAULT_MIN_VALUE,
  // DEFAULT_MIN_BLOB_AREA, imShow);
}

int testBlue(cv::Mat& input, cv::Mat& debugImage, std::vector<sm_husky_barrel_search_1::msg::DetectedObject>& detectedObjects, bool imShow = false)
{
  return testImage(input, debugImage, detectedObjects, "blue", 10, 10, "blue-barrel", DEFAULT_MIN_SATURATION,
                   DEFAULT_MIN_VALUE, DEFAULT_MIN_BLOB_AREA, imShow);
}

int testGreen(cv::Mat& input, cv::Mat& debugImage, std::vector<sm_husky_barrel_search_1::msg::DetectedObject>& detectedObjects, bool imShow = false)
{
  return testImage(input, debugImage, detectedObjects, "green", 50, 10, "ally", DEFAULT_MIN_SATURATION,
                   DEFAULT_MIN_VALUE, DEFAULT_MIN_BLOB_AREA, imShow);
}

int testYellow(cv::Mat& input, cv::Mat& debugImage, std::vector<sm_husky_barrel_search_1::msg::DetectedObject>& detectedObjects, bool imShow = false)
{
  // hue: 31 , minvalue: 200, minsat: 40, in rgb
  // return testImage(input, debugImage, "yellow", 31, 10, "mine", 40, 200, 100, imShow);

  // return testImage(input, debugImage, "yellow", 195, 40, "mine", 40, 20, 100, imShow);
  return testImage(input, debugImage, detectedObjects, "yellow", 90, 10, "mine", 40, 200, 100, imShow);
}

void testYellowFile(std::string path)
{
  cv::Mat input;
  cv::cvtColor(cv::imread(path), input, cv::COLOR_RGB2BGR);
  std::vector<sm_husky_barrel_search_1::msg::DetectedObject> detectedObjects;

  cv::Mat debugImage = input.clone();

  testYellow(input, debugImage, detectedObjects, true);

  cv::imshow("yellow filter - " + path, debugImage);
  cv::waitKey();
}

void testRedFile(std::string path)
{
  cv::Mat input = cv::imread(path);
  std::vector<sm_husky_barrel_search_1::msg::DetectedObject> detectedObjects;
  cv::Mat debugImage = input.clone();
  bool imShow = true;
  testRed(input, debugImage, detectedObjects, imShow);

  cv::imshow("red filter - " + path, debugImage);

  cv::waitKey();
}

void testRedFileBench(std::string path)
{
  cv::Mat input = cv::imread(path);
  std::vector<sm_husky_barrel_search_1::msg::DetectedObject> detectedObjects;
  // cv::cvtColor(input, input, cv::COLOR_RGB2BGR);

  cv::Mat debugImage = input.clone();
  bool imShow = true;

  int size = 60;
  int step = 5;
  for (int i = 0; i < 255; i++)
  {
    auto huevalue = i;
    std::cout << "huevalue: " << huevalue << std::endl;
    testImage(input, debugImage, detectedObjects, "red", huevalue, 20, "red", DEFAULT_MIN_SATURATION, DEFAULT_MIN_VALUE,
              DEFAULT_MIN_BLOB_AREA, imShow);
    cv::imshow("red filter - " + path, debugImage);
    cv::waitKey();
  }
}

void testBlueFile(std::string path, cv::Mat& debugImage)
{
  cv::Mat input = cv::imread(path);
  std::vector<sm_husky_barrel_search_1::msg::DetectedObject> detectedObjects;
  testImage(input, debugImage, detectedObjects, "blue", 10, 10, "");
}

void testGreenFile(std::string path, cv::Mat& debugImage)
{
  cv::Mat input = cv::imread(path);
  std::vector<sm_husky_barrel_search_1::msg::DetectedObject> detectedObjects;
  testImage(input, debugImage, detectedObjects, "green", 50, 10, "");
}

int encoding2mat_type(const std::string& encoding)
{
  if (encoding == "mono8")
  {
    return CV_8UC1;
  }
  else if (encoding == "bgr8")
  {
    return CV_8UC3;
  }
  else if (encoding == "mono16")
  {
    return CV_16SC1;
  }
  else if (encoding == "rgba8")
  {
    return CV_8UC4;
  }
  else if (encoding == "bgra8")
  {
    return CV_8UC4;
  }
  else if (encoding == "32FC1")
  {
    return CV_32FC1;
  }
  else if (encoding == "rgb8")
  {
    return CV_8UC3;
  }
  else if (encoding == "yuv422")
  {
    return CV_8UC2;
  }
  else
  {
    throw std::runtime_error("Unsupported encoding type");
  }
}

cv::Mat to_cv_mat(const sensor_msgs::msg::Image& img)
{
  return cv::Mat(cv::Size(img.width, img.height), encoding2mat_type(img.encoding),
                 const_cast<unsigned char*>(img.data.data()));
}

void callback(const sensor_msgs::msg::Image& imgMsg)
{
  cv::Mat image = to_cv_mat(imgMsg);
  // BUG: for some reason we get an image message from gazebo with as RGB but in practice we see it is BGR
  cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

  sensor_msgs::msg::Image outimg = imgMsg;

  // cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
  auto outmat = to_cv_mat(outimg);

  // USED TO WORK IN FILE MODE
  // cv::imwrite("/tmp/out.png", outmat);

  bool testGreenAlly = false;

  std::vector<sm_husky_barrel_search_1::msg::DetectedObject> detectedObjects;

  int detectedColor = 0;
  if (testRed(image, outmat, detectedObjects) > 0)
  {
    detectedColor = 1;
  }

  if (testGreenAlly && testGreen(image, outmat, detectedObjects) > 0)
  {
    detectedColor = 2;
  }

  if (testBlue(image, outmat, detectedObjects) > 0)
  {
    detectedColor = 3;
  }

  if (testYellow(image, outmat, detectedObjects) > 0)
  {
    detectedColor = 4;
  }

  sm_husky_barrel_search_1::msg::DetectedObjects detectedColorMsg;
  detectedColorMsg.detected_objects = detectedObjects;

  detectionPub->publish(detectedColorMsg);

  cv::cvtColor(outmat, outmat, cv::COLOR_RGB2BGR);

  outimg.header = imgMsg.header;
  debugImagePub->publish(outimg);
}

void main_ros_loop(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("opencv_perception_node");

  RCLCPP_INFO(nh->get_logger(), "opencv perception node started");
  detectionPub = nh->create_publisher<sm_husky_barrel_search_1::msg::DetectedObjects>("detected_objects", 1);
  imageSub = nh->create_subscription<sensor_msgs::msg::Image>("/image_raw", rclcpp::QoS(1).best_effort(), callback);
  debugImagePub = nh->create_publisher<sensor_msgs::msg::Image>("/opencv_debug_image", rclcpp::QoS(1).best_effort());

  rclcpp::Rate r(10);

  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh);
    r.sleep();
  }
}

int main(int argc, char** argv)
{
  main_ros_loop(argc, argv);

  auto testImagePath =
      ament_index_cpp::get_package_share_directory("sm_husky_barrel_search_1") + "/test_images/red4.png";

  // testRedFileBench(testImagePath);
  // testRedFile(testImagePath);

  /*
  testRedFile("../../red1.png");
  testRedFile("../../green1.png");
  testRedFile("../../blue1.png");


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
