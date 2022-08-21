#ifndef _ROS2_PACKAGES_PARAMS_H_
#define _ROS2_PACKAGES_PARAMS_H_

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

const std::string get_param_srv(const std::shared_ptr<rclcpp::Node> node,
                                const std::string &remote_node,
                                const std::string &request_param)
{
  //const std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ljy");
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, remote_node);
  while (!parameters_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(node->get_logger(), remote_node + " parameters service not available, waiting again...");
    return std::string{""};
  }
  auto parameters = parameters_client->get_parameters({request_param});
  //parameters_client->set_parameters({rclcpp::Parameter("CTI_MAP_PATH", "111111111111111111111111")});
  return parameters[0].value_to_string();
}

void set_param_srv(const std::shared_ptr<rclcpp::Node> node,
                   const std::string &remote_node,
                   const std::string &request_param,
                   const std::string &changed_value)
{
  //const std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("ljy");
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node, remote_node);
  while (!parameters_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(node->get_logger(), remote_node + " parameters service not available, waiting again...");
  }
  parameters_client->set_parameters({rclcpp::Parameter(request_param, changed_value)});
}



#endif