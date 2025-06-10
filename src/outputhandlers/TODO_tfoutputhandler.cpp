#include "outputhandlers/tfoutputhandler.hpp"
#include "expressiongraph/context.hpp"

namespace etasl {

TFOutputHandler::TFOutputHandler(
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const std::vector<TFSpec>& _tfspecs)
    : node(_node)
    , ctx(nullptr)
    , tfspecs(_tfspecs)
{
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    name = fmt::format("TFOutputHandler()");
}

const std::string& TFOutputHandler:: getName() const {
    return name;
}

bool TFOutputHandler::initialize(
    Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames)
{
    RCLCPP_INFO(node->get_logger(), "Initializing TFOutputHandler");
    RCLCPP_INFO(node->get_logger(), "Outputexpressions:");
    for (auto v: ctx->output_vars) {
        RCLCPP_INFO(node->get_logger(), "   - %s", v.first.c_str());
    }
    size_t ndx = 0;
    for (auto tfspec : tfspecs) {
        if (tfspec.variablenames.size() != 0) {
            for (auto v : tfspec.variablenames) {
                auto ptr = ctx->getOutputExpression<KDL::Frame>(v);
                RCLCPP_INFO(node->get_logger(), "Looking up variable %s", v.c_str());
                if (ptr != 0) {
                    geometry_msgs::msg::TransformStamped msg;
                    msg.header.frame_id = tfspec.frame_id;
                    msg.child_frame_id = v;
                    msgs.push_back(msg);
                    outp.push_back(ptr);
                } else {
                    RCLCPP_INFO(node->get_logger(), "variable %s not found", v.c_str());
                }
            }
        } else {
            for (auto v : ctx->output_vars) {
                RCLCPP_INFO(node->get_logger(), "Examing variable %s", v.first.c_str());
                auto ptr = ctx->getOutputExpression<KDL::Frame>(v.first);
                if (ptr != 0) {
                    geometry_msgs::msg::TransformStamped msg;
                    msg.header.frame_id = tfspec.frame_id;
                    msg.child_frame_id = v.first;
                    msgs.push_back(msg);
                    outp.push_back(ptr);
                } else {
                    RCLCPP_INFO(node->get_logger(), "Variable %s not a frame expression", v.first.c_str());
                }
            }
        }
    }
    RCLCPP_INFO(node->get_logger(), "Initializing TF2-outputhandler for %ld frames", msgs.size());

    return true; //TODO: return false when the method fails
}

void TFOutputHandler::update(
    const std::vector<std::string>& jnames,
    const Eigen::VectorXd& jpos,
    const Eigen::VectorXd& jvel,
    const std::vector<std::string>& fnames,
    const Eigen::VectorXd& fvel,
    const Eigen::VectorXd& fpos)
{
    rclcpp::Time now = node->get_clock()->now();

    for (size_t ndx = 0; ndx < msgs.size(); ++ndx) {
        msgs[ndx].header.stamp = now;
        KDL::Frame F = outp[ndx]->value();
        msgs[ndx].transform.translation.x = F.p.x();
        msgs[ndx].transform.translation.y = F.p.y();
        msgs[ndx].transform.translation.z = F.p.z();
        KDL::Quaternion q = KDL::toQuat(F.M);
        msgs[ndx].transform.rotation.w = q.w;
        msgs[ndx].transform.rotation.x = q.vec.x();
        msgs[ndx].transform.rotation.y = q.vec.y();
        msgs[ndx].transform.rotation.z = q.vec.z();
    }
    tf_broadcaster->sendTransform(msgs);
}

} // namespace etasl

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::TFOutputHandler, etasl::OutputHandler)