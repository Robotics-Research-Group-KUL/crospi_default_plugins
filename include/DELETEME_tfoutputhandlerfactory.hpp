#pragma once

#include "etasl_task_utils/registry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


namespace etasl {

void registerTFOutputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node);


} // namespace