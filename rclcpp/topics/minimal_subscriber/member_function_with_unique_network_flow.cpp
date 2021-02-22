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


#include <functional>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_options.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriberWithUniqueNetworkFlow : public rclcpp::Node
{
public:
  MinimalSubscriberWithUniqueNetworkFlow()
  : Node("minimal_subscriber_with_unique_network_flow")
  {
    // Create subscription with unique network flow
    // Enable unique network flow via options
    auto options_1 = rclcpp::SubscriptionOptions();
    options_1.require_unique_network_flow = RMW_UNIQUE_NETWORK_FLOW_STRICTLY_REQUIRED;

    subscription_1_ = this->create_subscription<std_msgs::msg::String>(
      "topic_1", 10, std::bind(
        &MinimalSubscriberWithUniqueNetworkFlow::topic_1_callback, this,
        _1), options_1);

    // Create subscription without unique network flow
    // Unique network flow is disabled in default options
    auto options_2 = rclcpp::SubscriptionOptions();
    subscription_2_ = this->create_subscription<std_msgs::msg::String>(
      "topic_2", 10, std::bind(
        &MinimalSubscriberWithUniqueNetworkFlow::topic_2_callback, this,
        _1), options_2);

    // Get network flows
    auto flows_1 = subscription_1_->get_network_flow();
    auto flows_2 = subscription_2_->get_network_flow();

    // Check if network flow is unique
    for (auto flow_1 : flows_1) {
      for (auto flow_2 : flows_2) {
        if (flow_1 == flow_2) {
          RCLCPP_ERROR(this->get_logger(), "Network flows across subscriptions are not unique");
          break;
        }
      }
    }

    // Print network flows
    print_network_flows(flows_1);
    print_network_flows(flows_2);
  }

private:
  void topic_1_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Topic 1 news: '%s'", msg->data.c_str());
  }
  void topic_2_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Topic 2 news: '%s'", msg->data.c_str());
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
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_1_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriberWithUniqueNetworkFlow>());
  rclcpp::shutdown();
  return 0;
}
