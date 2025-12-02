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

#include "crospi_utils/outputhandler.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "crospi_interfaces/msg/output_derivatives.hpp"

namespace etasl {
using namespace KDL;

class TopicOutputDerivativesHandler : public OutputHandler {
private:
    std::vector<Expression<double>::Ptr> outp;
    Context::Ptr ctx;
    std::vector<std::string> not_found;
    std::string topicname;
    std::vector<std::string> varnames;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
    rclcpp_lifecycle::LifecyclePublisher<crospi_interfaces::msg::OutputDerivatives>::SharedPtr pub;
    crospi_interfaces::msg::OutputDerivatives msg;
    bool initialized;
    bool activated;
    std::string name;

    //Stuff to compute derivatives:
    int time_ndx;
    std::vector<int>          fnames_ctx_ndx;
    std::vector<int>          jnames_ctx_ndx;
    Eigen::VectorXd           fvelocities;
    Eigen::VectorXd           jvelocities;
    boost::shared_ptr<solver> slv_;

public:
    /**
     */
    TopicOutputDerivativesHandler();


    /**
     * @return get the name of this instance of TopicOutputDerivativesHandler
    */
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

    // virtual void finalize();

    virtual void on_activate(Context::Ptr ctx,    
                            const std::vector<std::string>& jnames,
                            const std::vector<std::string>& fnames,
                            boost::shared_ptr<solver> slv) override;


    virtual void on_deactivate(Context::Ptr ctx) override;

}; // TopicOutputDerivativesHandler
} // namespace KDL
