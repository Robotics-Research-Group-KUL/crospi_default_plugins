#include "inputhandlers/vectorinputhandler.hpp"
// #include "rclcpp/wait_for_message.hpp"
#include <fmt/format.h>

#include <kdl/frames.hpp>

namespace etasl {


VectorInputHandler::VectorInputHandler()
    : kdl_vector(KDL::Vector::Zero()) // zero position
    , counter(1)
    , counter_deriv(1)
    , initialized(false)
    , activated(false)
    , enable_feedforward(false)
{
    

}

bool VectorInputHandler::construct(
    std::string _name,
    rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
    const Json::Value& parameters,
    std::shared_ptr<etasl::JsonChecker> jsonchecker)
{
    
    node = _node;
    
    topicname = jsonchecker->asString(parameters, "topic-name");
    name = fmt::format("{}({})",_name, topicname);
    depth = jsonchecker->asInt(parameters, "depth");

    nroftries = jsonchecker->asInt(parameters, "number_of_tries");

    when_unpublished = jsonchecker->asString(parameters, "when_unpublished");

    msg_type = jsonchecker->asString(parameters, "msg_type");

    varname = jsonchecker->asString(parameters, "varname");

    if(jsonchecker->is_member(parameters, "feedforward_input_topic"))
    {
        enable_feedforward = true;
        feedforward_input_topic = jsonchecker->asString(parameters, "feedforward_input_topic");
    }
    


    default_msg.x = jsonchecker->asDouble(parameters, "default_vector/x");
    default_msg.y = jsonchecker->asDouble(parameters, "default_vector/y");
    default_msg.z = jsonchecker->asDouble(parameters, "default_vector/z");


    input_msg.fs = NoData;
    input_msg.data = default_msg;
    // transform_stamped = default_msg;

    //initialize input_msg_deriv with a zero kdl_velocity msg:
    default_msg_deriv.x = 0.0;
    default_msg_deriv.y = 0.0;
    default_msg_deriv.z = 0.0;

    input_msg_deriv.data = default_msg_deriv;
    input_msg_deriv.fs = NoData;

    return true;
}

bool VectorInputHandler::initialize(
    Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames,
    Eigen::VectorXd& jpos,
    Eigen::VectorXd& fpos)
{

    if(input_msg.fs == NoData && when_unpublished=="use_default"){
        input_msg.fs = NewData; //Fakes new Data 
        input_msg.data = default_msg;
    }
    else if(input_msg.fs == NoData && when_unpublished=="throw_error"){
        input_msg.data = default_msg;
    }

    if(input_msg_deriv.fs == NoData && when_unpublished=="use_default"){
        input_msg_deriv.fs = NewData; //Fakes new Data 
        input_msg_deriv.data = default_msg_deriv;

    }
    else if(input_msg_deriv.fs == NoData && when_unpublished=="throw_error"){
        input_msg_deriv.data = default_msg_deriv;
    }

    using namespace std::chrono_literals;
    using namespace std::placeholders;

    auto qos = rclcpp::SensorDataQoS().keep_last(depth).lifespan(100ms);

    if(msg_type == "geometry_msgs/Vector3")
    {
        auto cb = std::bind(&VectorInputHandler::on_new_message_vector, this, _1);
        sub_vector = node->create_subscription<MsgType_vector>(topicname, qos, cb);
    
        if (!sub_vector) {
            RCLCPP_FATAL(node->get_logger(), "Could not create subscriber for input handler '%s'.", name.c_str());
            return false;
        }
    }
    else if(msg_type == "geometry_msgs/Point")
    {
        auto cb = std::bind(&VectorInputHandler::on_new_message_point, this, _1);
        sub_point = node->create_subscription<MsgType_point>(topicname, qos, cb);
    
        if (!sub_point) {
            RCLCPP_FATAL(node->get_logger(), "Could not create subscriber for input handler '%s'.", name.c_str());
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "msg_type %s is not supported. Supported types are 'geometry_msgs/Vector3' and 'geometry_msgs/Point'.", msg_type.c_str());
        return false;
    }


      
    if(enable_feedforward){
        using namespace std::chrono_literals;
        using namespace std::placeholders;
        auto cb = std::bind(&VectorInputHandler::on_new_message_deriv, this, _1);
        auto qos = rclcpp::SensorDataQoS().keep_last(1).lifespan(100ms);
        deriv_sub = node->create_subscription<MsgTypeDeriv>(feedforward_input_topic, qos, cb);
    
        if(!deriv_sub)
        {
            RCLCPP_FATAL(node->get_logger(), "Could not create subscriber associated to feedforward_input_topic: %s.", feedforward_input_topic.c_str());
            return false;
        }
    }

    initialized = true;
    RCLCPP_INFO(node->get_logger(), "Vectorinputhandler has been initialized correctly ________________________");

    return true;
}


void VectorInputHandler::update(
    double time,
    const std::vector<std::string>& jnames,
    Eigen::VectorXd& jpos,
    const std::vector<std::string>& fnames,
    Eigen::VectorXd& fpos)
{
    // if(activated){
    //     std::cout << "it is activated ////////////////////////////////" << std::endl;
    //     std::cout << "fs: " << input_msg.fs << std::endl;
    //     std::cout << "when_unpublished: " << when_unpublished  << std::endl;
    // }
    // input_msg.data = msg;



    if(activated && input_msg.fs == NoData && when_unpublished=="throw_error"){

        if (counter >= nroftries){
            RCLCPP_ERROR(node->get_logger(), "Vector could be not found after %i tries and therefore the node is shutting down. Consider changing when_unpublished policy of inputhandler %s to use_default if desired. ", nroftries, getName().c_str());
            rclcpp::shutdown();
        }
        else{
            counter++;
            // std::cout << "Counter +++++: " << counter<< std::endl;
        }
        return;
    }
    else if(activated && input_msg.fs == NewData){
        consume_data(true); 
    }

    if(enable_feedforward && activated && input_msg_deriv.fs == NoData && when_unpublished=="throw_error"){

        if (counter_deriv >= nroftries){
            RCLCPP_ERROR(node->get_logger(), "Velocity vector could not be read from topic %s and used as feedforward for vector after %i tries and therefore the node is shutting down. Consider changing when_unpublished policy of inputhandler %s to use_default to use the default value (zero velocity!) insted. ", feedforward_input_topic.c_str(), nroftries, getName().c_str());
            rclcpp::shutdown();
        }
        else{
            counter_deriv++;
        }
        return;
    }
    else if(activated && input_msg_deriv.fs == NewData){
        consume_data_deriv(true); 
    }
    // RCLCPP_INFO(node->get_logger(), "JointStateInputHander::update()");
}

void VectorInputHandler::on_activate(Context::Ptr ctx,
    const std::vector<std::string>& jnames,
    const std::vector<std::string>& fnames,
    boost::shared_ptr<solver> slv) 
{
    // std::cout << "entering on activate =======================" << std::endl;
    if(!initialized){
        RCLCPP_WARN(node->get_logger(), "The VectorInputHandler cannot be activated since it has not been initialized yet");
        return;
    }
    if(activated){
        RCLCPP_WARN(node->get_logger(), "The VectorInputHandler cannot be activated since it has already been activated. Call on_deactivate first.");
        return;
    }


    inp = ctx->getInputChannel<KDL::Vector>(varname); 
    if (!inp) {
        RCLCPP_INFO(node->get_logger(), fmt::format("No Vector input channel with the name {} was created in the LUA specification, and thus the vector will not be requested. If desired, this can be created with with createInputChannelVector function.",varname).c_str());
        activated = false;
    } 
    else {
        // inps->setJacobian(time_ndx, default_value_kdl );
        time_ndx = ctx->getScalarNdx("time");
        consume_data(false); 
        consume_data_deriv(false); 
        activated = true;
    }
}

void VectorInputHandler::on_deactivate(Context::Ptr ctx) {
    // std::cout << "entering on deactivate =======================" << std::endl;

    inp = nullptr;
    activated=false;
}

const std::string& VectorInputHandler::getName() const
{
    return name;
}

void VectorInputHandler::consume_data(const bool& make_old_data)
{
    if (inp) {
        kdl_vector[0] = input_msg.data.x;
        kdl_vector[1] = input_msg.data.y;
        kdl_vector[2] = input_msg.data.z;


        inp->setValue(kdl_vector);
        if (make_old_data){
            input_msg.fs = OldData;
        }
    }
}

void VectorInputHandler::consume_data_deriv(const bool& make_old_data)
{
    if (inp) {
        kdl_velocity[0] = input_msg_deriv.data.x;
        kdl_velocity[1] = input_msg_deriv.data.y;
        kdl_velocity[2] = input_msg_deriv.data.z;

        inp->setJacobian(time_ndx, kdl_velocity );
        if (make_old_data){
            input_msg_deriv.fs = OldData;
        }
    }
}


void VectorInputHandler::on_new_message_vector(const VectorInputHandler::MsgType_vector& msg)
{
    input_msg.fs = NewData;
    input_msg.data = msg; //Message type is both vector3
}

void VectorInputHandler::on_new_message_point(const VectorInputHandler::MsgType_point& msg)
{
    input_msg.fs = NewData;
    input_msg.data.x = msg.x; //input_msg is vector3 and msg is point
    input_msg.data.y = msg.y;
    input_msg.data.z = msg.z;
}

void VectorInputHandler::on_new_message_deriv(const VectorInputHandler::MsgTypeDeriv& msg)
{
    input_msg_deriv.fs = NewData;
    input_msg_deriv.data = msg;
}

VectorInputHandler::~VectorInputHandler() {

};



} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::VectorInputHandler, etasl::InputHandler)
