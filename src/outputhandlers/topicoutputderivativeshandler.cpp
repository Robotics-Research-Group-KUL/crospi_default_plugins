//  Copyright (c) 2025 KU Leuven, Belgium
//
//  Author: Santiago Iregui
//  email: <santiago.iregui@kuleuven.be>
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

#include "outputhandlers/topicoutputderivativeshandler.hpp"

namespace etasl {

TopicOutputDerivativesHandler::TopicOutputDerivativesHandler()
    : initialized(false)
    , activated(false)
    , time_ndx(0)
{

}


bool TopicOutputDerivativesHandler::construct(
    std::string _name, 
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const Json::Value& parameters,
    std::shared_ptr<etasl::JsonChecker> jsonchecker)
{
    node = _node;
    topicname = jsonchecker->asString(parameters, "topic-name");
    name = fmt::format("{}({})",_name, topicname);
    for (auto n : parameters["variable-names"]) {
    // for (auto n : jsonchecker->asArray(parameters, "variable-names")) {
        varnames.push_back(n.asString());
        // varnames.push_back(jsonchecker->asString(n, ""));
    }

    return true;
}



bool TopicOutputDerivativesHandler::initialize(
    Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames)
{
    // std::cout << "entering on initialize topicoutputDerivativeshandler =======================" << std::endl;

    if(!initialized){
        pub = node->create_publisher<crospi_interfaces::msg::OutputDerivatives>(topicname, 10);
        pub->on_deactivate();
        initialized = true;
        // std::cout << "initialized topicoutputDerivativeshandler=======================" << std::endl;
    }
    else{
        RCLCPP_WARN(node->get_logger(), "Ignoring request: the topicoutputDerivativeshandler was already initialized, so it cannot be initialized again.");
        return false;
    }
    return true;
}

void TopicOutputDerivativesHandler::update(
    const std::vector<std::string>& jnames,
    const Eigen::VectorXd& jpos,
    const Eigen::VectorXd& jvel,
    const std::vector<std::string>& fnames,
    const Eigen::VectorXd& fvel,
    const Eigen::VectorXd& fpos)
{
    if(!activated){
        // RCLCPP_WARN(node->get_logger(), "The topicoutputDerivativeshandler cannot be updated since it has not been initialized yet");
        return;
    }
    assert( msg.data.size() == outp.size() /* size of msg.data and outp vector containing expressions is not the same */); 

    slv_->getJointVelocities(jvelocities);
    slv_->getFeatureVelocities(fvelocities);

    unsigned int L = outp.size();
    for (unsigned int i = 0; i < L; ++i) {
        outp[i]->value();  // always call value() before derivative() !
        msg.data[i] = outp[i]->derivative(time_ndx);
        for (unsigned int j=0;j< jnames_ctx_ndx.size();++j) {
            msg.data[i] += outp[i]->derivative(jnames_ctx_ndx[j]) * jvelocities[j];
        }
        for (unsigned int j=0;j< fnames_ctx_ndx.size();++j) {
            msg.data[i] += outp[i]->derivative(fnames_ctx_ndx[j]) * fvelocities[j];
        }
    }
    pub->publish(msg);
}

void TopicOutputDerivativesHandler::on_activate(Context::Ptr ctx, 
    const std::vector<std::string>& jnames, 
    const std::vector<std::string>& fnames, 
    boost::shared_ptr<solver> slv) 
{
    // std::cout << "entering on activate =======================" << std::endl;
    // if (outp.size()>0){
    //     throw std::runtime_error("TopicoutputDerivativeshandler: Before calling on_activate again, on_deactivate must be called ");
    //     return;
    // }
    

    if(!initialized){
        RCLCPP_WARN(node->get_logger(), "The topicoutputDerivativeshandler cannot be activated since it has not been initialized yet");
        return;
    }
    if(activated){
        RCLCPP_WARN(node->get_logger(), "The topicoutputDerivativeshandler cannot be activated since it has already been activated. Call on_deactivate first.");
        return;
    }

    if (varnames.size() == 0) {
        // std::cout << "topic output hello1............................" << std::endl;
        for (auto ov : ctx->output_vars) {
            auto ptr = ctx->getOutputExpression<double>(ov.first);
            // std::cout << "topic output hello4444............................" << ptr->value() << std::endl;
            if (ptr != 0) {
                msg.names.push_back(cut_global(ov.first));
                outp.push_back(ptr);
                msg.data.push_back(0.0);
                msg.is_declared.push_back(true);
            }
        }
    } else {
        // msg.names = varnames;
        // msg.data.resize(varnames.size());
        for (auto i = 0; i < varnames.size(); ++i) {
            auto ptr = ctx->getOutputExpression<double>(varnames[i]);
            if (ptr != 0) {
            // std::cout << "topic output hello1222............................" << varnames[i] << std::endl;
                // std::cout << varnames[i] << "value:" << ptr->value() << std::endl;

                msg.names.push_back(cut_global(varnames[i]));
                outp.push_back(ptr);
                msg.data.push_back(0.0);
                msg.is_declared.push_back(true);
            }
            else{
                // std::cout << "topic output hello3333............................" << varnames[i] << std::endl;
                not_found.push_back(varnames[i]);
                msg.names.push_back(cut_global(varnames[i]));
                outp.push_back(Constant(0.0));
                msg.data.push_back(0.0);
                msg.is_declared.push_back(false);
            }
        }
    }
    pub->on_activate(); //This works because we check for initialized above

    //Stuff to compute derivatives:
    time_ndx      = ctx->getScalarNdx("time");

    std::vector< std::string> jnames_complete; //jnames coming as argument could be a subset of this
    slv->getJointNameVector(jnames_complete);
    jnames_ctx_ndx.resize(jnames_complete.size());
    for (unsigned int i=0;i<jnames_complete.size();++i) {
        int r = ctx->getScalarNdx( jnames_complete[i] );
        assert( (r!= -1) && "BUG: solver cannot contain variables that are outside the context");
        jnames_ctx_ndx[i] = r; 
    }

    // int fnames; //already available in the argument of the function
    // slv->getFeatureNameVector(fnames);  
    fnames_ctx_ndx.resize(fnames.size(), 0.0);
    for (unsigned int i=0;i<fnames.size();++i) {
        int r = ctx->getScalarNdx( fnames[i] );
        assert( (r!= -1) && "BUG: solver cannot contain variables that are outside the context");
        fnames_ctx_ndx[i] = r; 
    }

    jvelocities.resize( jnames_complete.size() );
    jvelocities = Eigen::VectorXd::Zero( jnames_complete.size() );

    fvelocities.resize( fnames.size() ); 
    fvelocities = Eigen::VectorXd::Zero( fnames.size() );



    slv_ = slv;

    activated = true;
}

void TopicOutputDerivativesHandler::on_deactivate(Context::Ptr ctx) {
    outp.clear();
    msg.data.clear();
    msg.names.clear();
    msg.is_declared.clear();
    if(initialized){
        pub->on_deactivate();
    }
    activated = false;
}

const std::string& TopicOutputDerivativesHandler::getName() const {
    return name;
}

// void TopicOutputDerivativesHandler::finalize()
// {
//     pub.reset();
// }

} // namespace etasl

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::TopicOutputDerivativesHandler, etasl::OutputHandler)
