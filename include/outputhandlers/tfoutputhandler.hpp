//  Copyright (c) 2025 KU Leuven, Belgium
//
//  Authors: Santiago Iregui and Erwin Aertbeliën
//  emails: <santiago.iregui@kuleuven.be> and <erwin.aertbelien@kuleuven.be>
//
//  GNU Lesser General Public License Usage
//  Alternatively, this file may be used under the terms of the GNU Lesser
//  General Public License version 3 as published by the Free Software
//  Foundation and appearing in the file LICENSE.LGPLv3 included in the
//  packaging of this file. Please review the following information to
//  ensure the GNU Lesser General Public License version 3 requirements
//  will be met: https://www.gnu.org/licenses/lgpl.html.
// 
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.


#ifndef TFOUTPUTHANDLER_HPP_34DF23
#define TFOUTPUTHANDLER_HPP_34DF23
#include "expressiongraph/context.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "crospi_utils/outputhandler.hpp"
#include "crospi_utils/etasl_error.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace etasl {
using namespace KDL;
struct TFSpec {
    std::string frame_id;
    std::vector<std::string> variablenames;
};
class TFOutputHandler : public OutputHandler {
    typedef sensor_msgs::msg::JointState MsgType;

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::vector<geometry_msgs::msg::TransformStamped> msgs;
    std::vector<Expression<KDL::Frame>::Ptr> outp;
    Context::Ptr ctx;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    bool initialized;
    std::vector<TFSpec> tfspecs;
    std::string name;

public:
    /**
     * Gets all position and velocity values for the robot variables
     * in the specification.
     */
    TFOutputHandler();

    virtual const std::string& getName() const override;

    virtual bool construct(
        std::string _name, 
        rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
        const Json::Value& _parameters,
        std::shared_ptr<etasl::JsonChecker> _jsonchecker) override;

    virtual bool initialize(
        Context::Ptr ctx,
        const std::vector<std::string>& jnames,
        const std::vector<std::string>& fnames) override;

    /**
     * updates the values for the output and prints them in a tab-delimited format
     */
    virtual void update(
        const std::vector<std::string>& jnames,
        const Eigen::VectorXd& jpos,
        const Eigen::VectorXd& jvel,
        const std::vector<std::string>& fnames,
        const Eigen::VectorXd& fvel,
        const Eigen::VectorXd& fpos) override;

}; // TFOutputHandler
} // namespace KDL

#endif
