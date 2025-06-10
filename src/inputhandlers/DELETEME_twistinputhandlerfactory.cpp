
#include "etasl_task_utils/etasl_error.hpp"
#include "etasl_task_utils/inputhandlerfactory.hpp"
#include "etasl_node_utils/twistinputhandler.hpp"
#include "etasl_task_utils/registry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <jsoncpp/json/json.h>

namespace etasl {

/**
 * This is a factory that can create TwistInputHandlers
 * The platform specific part is given with the constructor of this factory
 * Afterwards, everything is generic and independent of the platform
 */
class TwistInputHandlerFactory : public InputHandlerFactory {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;

public:
    typedef std::shared_ptr<InputHandlerFactory> SharedPtr;

    TwistInputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
        : node(_node)
    {
    }

    /**
     * @brief gets the schema for the parameters of this factory
     * @return JSON schema
     */
    virtual Json::Value getSchema()
    {
        std::string schema_src = R"(
                    {
                        "$schema": "http://json-schema.org/draft-06/schema",
                        "$id":"twistinputhandler",
                        "type":"object",
                            "properties":{
                                "is-twistinputhandler" : {
                                    "description":"To indicate that this describes a twist inputhandler which reads from a geometry_msgs/Twist Message topic and converts it into a twist etasl expression",
                                    "type":"boolean",
                                    "default":true
                                },
                                "topic-name" : {
                                    "description":"name of the topic to subscribe to",
                                    "type":"string"
                                },
                                "number_of_tries" : {
                                    "description":"the number of times unsuccesfully initialize() can be called before throwing an error",
                                    "type":"number"
                                },
                                "depth" : {
                                    "description":"Depth of the topic queue.  In case of multiple sources for the joint values, it can be useful to set a depth larger than 1",
                                    "type":"number",
                                    "default": 1
                                },
                                "default_twist" : { 
                                    "description": "Default twist used in case that the topic defined in topic-name is not yet publishing",    
                                    "$ref":"twist.json"
                                },
                                "when_unpublished" : {
                                    "description": "Defines the behavior of the node when nothing has been published to the topic defined in topic-name during activation time of the node",
                                    "enum": ["use_default", "throw_error"],
                                    "oneOf": [
                                        {
                                            "type": "string", 
                                            "const": "use_default",
                                            "title": "Use default value",
                                            "description": "Uses the defined default value in the default_twist field while nothing has been published to the subscribed topic defined in topic-name. An INFO message will be logged."
                                        },
                                        {
                                            "type": "string", 
                                            "const": "throw_error",
                                            "title": "Throw error",
                                            "description": "Throws an error if nothing has been published to the subscribed topic defined in topic-name at the time of activation of the node, preventing its activation. "
                                        }
                                    ] 
                                },
                                "varname" : {            
                                    "description":"Name of the expression variable that was created in the task specification with createInputChannelTwist. The data coming from the topic will be available in the task specification through this variable",
                                    "type":"string"
                                }
                        }
                    }
                )";
        Json::Value schema;
        Json::Reader reader;
        reader.parse(schema_src, schema);
        return schema;
    }

    /**
     * @brief gets the name of this solver
     */
    virtual const char* getName()
    {
        return "twistinputhandler";
    }

    /**
     * @brief create the solver with the given parameters
     *
     */
    virtual InputHandler::SharedPtr create(const Json::Value& parameters, boost::shared_ptr<etasl::JsonChecker> jsonchecker)
    {
        std::string topic_name = jsonchecker->asString(parameters, "topic-name");
        
        int depth = jsonchecker->asInt(parameters, "depth");
        int nroftries = jsonchecker->asInt(parameters, "number_of_tries");

        std::string when_unpublished = jsonchecker->asString(parameters, "when_unpublished");

        std::string varname = jsonchecker->asString(parameters, "varname");


        geometry_msgs::msg::Twist def_msg;
        def_msg.linear.x = jsonchecker->asDouble(parameters, "default_twist/linear/x");
        def_msg.linear.y = jsonchecker->asDouble(parameters, "default_twist/linear/y");
        def_msg.linear.z = jsonchecker->asDouble(parameters, "default_twist/linear/z");
        def_msg.angular.x = jsonchecker->asDouble(parameters, "default_twist/angular/x");
        def_msg.angular.y = jsonchecker->asDouble(parameters, "default_twist/angular/y");
        def_msg.angular.z = jsonchecker->asDouble(parameters, "default_twist/angular/z");

        // for (auto n : parameters["variable-names"]) {
        //     varnames.push_back(n.asString());
        // }

        return std::make_shared<TwistInputHandler>(
            node,
            topic_name,
            nroftries,
            depth,
            def_msg,
            when_unpublished,
            varname);
    }

    virtual ~TwistInputHandlerFactory() { }
};

void registerTwistInputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<InputHandlerFactory>::registerFactory(std::make_shared<TwistInputHandlerFactory>(_node));
}

} // namespace