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


#pragma once

#include "crospi_interfaces/msg/input.hpp"
#include "crospi_utils/inputhandler.hpp"
#include <expressiongraph/context.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include "rclcpp_lifecycle/lifecycle_node.hpp"


namespace etasl {
using namespace KDL;

class TopicInputHandler : public InputHandler {
    struct Buffer {
        double most_recent_value; // most recent value obtained during the current sample period
        VariableType<double>::Ptr inputChannel; // the inputChannel that will get the most recent value when update() is called
        bool adapted; // the inputChannel is only updated when a value was received.
    };


    Context::Ptr ctx;
    std::string topicname;
    std::unordered_map<std::string, size_t> inputs; // from name to index in buffer, for ALL input channels
    std::vector<Buffer> buffer;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    rclcpp::Subscription<crospi_interfaces::msg::Input>::SharedPtr sub;
    crospi_interfaces::msg::Input msg;
    std::string name;

public:
    typedef std::shared_ptr<TopicInputHandler> SharedPtr;
    
    TopicInputHandler();

    virtual bool construct(
        std::string name,    
        rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
        const Json::Value& parameters,
        std::shared_ptr<etasl::JsonChecker> jsonchecker) override;

    /**
     * @brief Initialize the input handler
     * @param ctx etasl context
     * @param jnames vector with joint names
     * @param fnames vector with feature variable names
     * @param jpos Eigen vector with joint positions
     * @param fpos Eigen vector with feature variable positions
     *
     * @details can be used to initialize the handler, or can be used to only change jpos/fpos
     *          in the initialization phase of eTaSL (e.g. specifying the initial value of the
     *           feature variables)
     */
    virtual bool initialize(
        Context::Ptr ctx,
        const std::vector<std::string>& jnames,
        const std::vector<std::string>& fnames,
        Eigen::VectorXd& jpos,
        Eigen::VectorXd& fpos) override;

    virtual void on_new_message(const crospi_interfaces::msg::Input& msg);

    /**
     * gets new values for this input and use them to fill in the input channels defined
     * within eTaSL.
     * time is an argument that can be used to lookup an appropriate value for the input
     * (e.g. when the inputhandler stores a whole trajectory)
     * jnames,jpos, fnames, fpos can be used to update the state.  This handler however doesn't use
     * these, but changes the input channels in the eTaSL context.
     */
    virtual void update(
        double time, 
        const std::vector<std::string>& jnames,
        Eigen::VectorXd& jpos, 
        const std::vector<std::string>& fnames,
        Eigen::VectorXd& fpos) override;

    virtual const std::string& getName() const override;

    virtual ~TopicInputHandler() {};
};

} // namespace etasls
