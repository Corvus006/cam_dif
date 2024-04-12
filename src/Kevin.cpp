// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <string>

#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <cam_dif/Motioncapture.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

// Kevin is the guy for communication ;)
class Kevin : public rclcpp::Node {
public:
  Kevin() : Node("Kevin") {}

  void init(image_transport::ImageTransport &it) {
    sub = it.subscribe(
        "/cam_pub", 10,
        std::bind(&Kevin::topic_callback, this, std::placeholders::_1));
    pub = it.advertise("/kevin", 10);
  }

private:
  void topic_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    auto message = std::make_shared<sensor_msgs::msg::Image>();

    mc.detectionF(message, msg);

    pub.publish(message);
  }

  image_transport::Subscriber sub;
  image_transport::Publisher pub;
  MotionCapture mc;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto kevin = std::make_shared<Kevin>();
  image_transport::ImageTransport it(kevin);
  kevin->init(it);
  rclcpp::spin(kevin);
  rclcpp::shutdown();
  return 0;
}
