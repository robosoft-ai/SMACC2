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
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "cv_mat_sensor_msgs_image_type_adapter.hpp"

// RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
//   image_tools::sensor_msgs::msg::Image,
//   sensor_msgs::msg::Image);

rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr detectionPub;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debugImagePub;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imageSub;

void segmentColor(const cv::Mat& inputRGB, int hueMean, int hueWindow, cv::Mat& out, int minSaturation=20, int minValue=20, bool imgShow = false)
{
  cv::Mat hsvInput;
  cv::cvtColor(inputRGB, hsvInput, cv::COLOR_BGR2HSV);
  cv::inRange(hsvInput, cv::Scalar(hueMean - hueWindow, minSaturation, minValue), cv::Scalar(hueMean + hueWindow, 255, 255), out);
  cv::threshold(out, out, 100, 255, cv::THRESH_BINARY);

  if(imgShow)
    cv::imshow("segment", out);
}

int testImage(cv::Mat& input, cv::Mat& debugImage, std::string colorName, int hueMean, int hueWindow,  std::string message ="", int minSaturation=20, int minValue=20, int minBlobArea=100, bool imgShow= false)
{
  cv::Mat segmented;

  segmentColor(input, hueMean, hueWindow, segmented,minSaturation, minValue, imgShow);

  cv::SimpleBlobDetector::Params params;
  // params.filterByArea= true;
  // params.minArea = 10;
  params.filterByArea = true;
  params.minArea = minBlobArea;
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
      cv::putText(debugImage, message, cv::Point(r.x,r.y),cv::FONT_HERSHEY_SIMPLEX,0.3,cv::Vec3b(255,255, 255));

    }
  }

  // cv::imshow(colorName + " filter - "+ path, segmented);
  // cv::waitKey();

  return blobs.size();
}

int testRed(cv::Mat& input, cv::Mat& debugImage)
{
  return testImage(input, debugImage, "red", 130, 20, "enemy");
}

int testBlue(cv::Mat& input, cv::Mat& debugImage)
{
  return testImage(input, debugImage, "blue", 10, 10, "blue-barrel");
}

int testGreen(cv::Mat& input, cv::Mat& debugImage)
{
  return testImage(input, debugImage, "green", 50, 10, "ally");
}

int testYellow(cv::Mat& input, cv::Mat& debugImage, bool imShow = false)
{
  // hue: 31 , minvalue: 200, minsat: 40, in rgb
  //return testImage(input, debugImage, "yellow", 31, 10, "mine", 40, 200, 100, imShow);

  //return testImage(input, debugImage, "yellow", 195, 40, "mine", 40, 20, 100, imShow);
  return testImage(input, debugImage, "yellow", 90, 10, "mine", 40, 200, 100, imShow);
}

void testYellowFile(std::string path)
{
  cv::Mat input;
    cv::cvtColor(cv::imread(path), input, cv::COLOR_RGB2BGR);

  cv::Mat debugImage= input.clone();


  testYellow(input, debugImage, true);

  cv::imshow("yellow filter - "+ path, debugImage);
  cv::waitKey();

}

void testRedFile(std::string path)
{
  cv::Mat input = cv::imread(path);
    cv::Mat debugImage = input.clone();

  testRed(input, debugImage);
}

void testBlueFile(std::string path, cv::Mat& debugImage)
{
  cv::Mat input = cv::imread(path);
  testImage(input, debugImage, "blue", 10, 10, "");
}

void testGreenFile(std::string path, cv::Mat& debugImage)
{
  cv::Mat input = cv::imread(path);
  testImage(input, debugImage, "green", 50, 10, "");
}


int
encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  } else if (encoding == "yuv422") {
    return CV_8UC2;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

cv::Mat to_cv_mat(const sensor_msgs::msg::Image& img)
{
    return cv::Mat(cv::Size(img.height, img.width),
                    encoding2mat_type(img.encoding),
                    const_cast<unsigned char *>(img.data.data()));
}

void update()
{
}

void callback(const sensor_msgs::msg::Image& img)
{
  cv::Mat image = to_cv_mat(img);
  sensor_msgs::msg::Image outimg = img;
  auto outmat = to_cv_mat(outimg);

  cv::cvtColor(image, outmat, cv::COLOR_RGB2BGR);


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

  if (testYellow(image, outmat) > 0)
  {
    detectedColor = 4;
  }

  std_msgs::msg::Int32 detectedColorMsg;
  detectedColorMsg.data = detectedColor;
  detectionPub->publish(detectedColorMsg);


  outimg.header = img.header;
  debugImagePub->publish(outimg);
}


void main_ros_loop(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("opencv_perception_node");

  RCLCPP_INFO(nh->get_logger(), "opencv perception node started");
  detectionPub = nh->create_publisher<std_msgs::msg::Int32>("detected_color", 1);
  imageSub = nh->create_subscription<sensor_msgs::msg::Image>("/image_raw", 1, callback);
  debugImagePub = nh->create_publisher<sensor_msgs::msg::Image>("/opencv_debug_image", 1);

  rclcpp::Rate r(10);

  while (rclcpp::ok())
  {
    update();
    rclcpp::spin_some(nh);
    r.sleep();
  }

}

int main(int argc, char** argv)
{
  main_ros_loop(argc, argv);

  //testYellowFile("/home/geus/Desktop/smacc_ws/src/SMACC2/smacc2_sm_reference_library/sm_husky_barrel_search_1/servers/yellow.png");

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
