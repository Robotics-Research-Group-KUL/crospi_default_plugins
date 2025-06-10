#pragma once

#include "etasl_task_utils/registry.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace etasl {

void registerJointStateOutputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node);


} // namespace