// Copyright 2020 Ericsson AB
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
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher_options.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisherWithUniqueNetworkFlow : public rclcpp::Node
{
public:
  MinimalPublisherWithUniqueNetworkFlow()
  : Node("minimal_publisher_with_unique_network_flow"), count_1_(0), count_2_(0)
  {
    // Create publisher with unique network flow
    // Enable unique network flow via options
    auto options_1 = rclcpp::PublisherOptions();
    options_1.require_unique_network_flow = RMW_UNIQUE_NETWORK_FLOW_OPTIONALLY_REQUIRED;
    publisher_1_ = this->create_publisher<std_msgs::msg::String>("topic_1", 10, options_1);
    timer_1_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisherWithUniqueNetworkFlow::timer_1_callback, this));

    // Create publisher without unique network flow
    // Unique network flow is disabled in default options
    auto options_2 = rclcpp::PublisherOptions();
    publisher_2_ = this->create_publisher<std_msgs::msg::String>("topic_2", 10);
    timer_2_ = this->create_wall_timer(
      1000ms, std::bind(&MinimalPublisherWithUniqueNetworkFlow::timer_2_callback, this));

    // Get network flows
    auto flows_1 = publisher_1_->get_network_flow();
    auto flows_2 = publisher_2_->get_network_flow();

    // Check if network flow is unique
    for (auto flow_1 : flows_1) {
      for (auto flow_2 : flows_2) {
        if (flow_1 == flow_2) {
          RCLCPP_WARN(this->get_logger(), "Network flows across publishers are not unique");
          break;
        }
      }
    }

    // Print network flows
    print_network_flows(flows_1);
    print_network_flows(flows_2);
  }

private:
  void timer_1_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_1_++);

    RCLCPP_INFO(
      this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_1_->publish(message);
  }
  void timer_2_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hej, världen! " + std::to_string(count_2_++);

    RCLCPP_INFO(
      this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_2_->publish(message);
  }
  /// Print network flows in JSON-like format
  void print_network_flows(const std::vector<std::map<std::string, std::string>> & network_flows) const
  {
    std::ostringstream stream;
    stream << "{\"networkFlows\": [";
    bool skip_1 = true;
    for (auto flow : network_flows) {
      if (skip_1) {
        skip_1 = false;
      } else {
        stream << ",";
      }
      stream << "{" << std::endl;
      bool skip_2 = true;
      for (auto key_val : flow) {
        if (skip_2) {
          skip_2 = false;
        } else {
          stream << "," << std::endl;
        }
        stream << "\"" << key_val.first << "\": \"" << key_val.second << "\"";
      }
      stream << "}";
    }
    stream << "]}";
    RCLCPP_INFO(
      this->get_logger(), "%s",
      stream.str().c_str());
  }
  rclcpp::TimerBase::SharedPtr timer_1_;
  rclcpp::TimerBase::SharedPtr timer_2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_2_;
  size_t count_1_;
  size_t count_2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisherWithUniqueNetworkFlow>());
  rclcpp::shutdown();
  return 0;
}
