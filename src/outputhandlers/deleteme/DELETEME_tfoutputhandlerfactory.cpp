#include "etasl_task_utils/outputhandlerfactory.hpp"
#include "etasl_node_utils/tfoutputhandler.hpp"
#include "etasl_task_utils/etasl_error.hpp"
#include "etasl_task_utils/registry.hpp"
#include "rclcpp/rclcpp.hpp"
#include <jsoncpp/json/json.h>

namespace etasl {


/**
 * This is a factory that can create TopicOutputHandlers
 * The platform specific part is given with the constructor of this factory
 * Afterwards, everything is generic and independent of the platform
*/
class TFOutputHandlerFactory : public OutputHandlerFactory {
    rclcpp_lifecycle::LifecycleNode::SharedPtr node;

public:
    typedef std::shared_ptr<OutputHandlerFactory> SharedPtr;

    TFOutputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
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
    "$id": "tfoutputhandler.json",
    "type": "object",
    "title": "Parameters for TFOutputHandler",
    "description": "A TFOutputHandler publishes eTaSL frames in TF2",
    "properties": {
        "is-tfoutputhandler": {
            "description": "A key value to flag that this describes a FileOutputHandler",
            "type": "boolean"
        },
        "tf": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "frame_id": {
                        "title": "frame_id",
                        "description": "frame_id that will be send to TF2.  The frames are expressed w.r.t. this frame in TF2.",
                        "type": "string"
                    },
                    "variable-names": {
                        "title": "variable-names",
                        "description": "list of names for Frame output expressions. Name will also be used as child_frame_id for TF2. When list is empty, all Frame output expressions will be used.",
                        "type": "array",
                        "items": {
                            "type": "string"
                        },
                        "default": []
                    }
                },
                "required": [
                    "frame_id",
                    "variable-names"
                ],
                "additionalProperties": false
            }
        }
    },
    "required": [
        "is-tfoutputhandler",
        "tf"
    ],
    "additionalProperties": false
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
        return "tfoutputhandler";
    }

    /**
     * @brief create the solver with the given parameters
     *
     */
    virtual TFOutputHandler::SharedPtr create(const Json::Value& parameters, boost::shared_ptr<etasl::JsonChecker> jsonchecker)
    {
        std::vector<TFSpec> tfspecs;
        for (auto p : jsonchecker->asArray(parameters, "tf")) {
            TFSpec tfspec;
            tfspec.frame_id = jsonchecker->asString(p, "frame_id");
            for (auto v: jsonchecker->asArray(p, "variable-names")) {
                tfspec.variablenames.push_back(jsonchecker->asString(v, ""));
            }
            tfspecs.emplace_back(tfspec);
        }
        return std::make_shared<TFOutputHandler>(
            node,
            tfspecs);
    }

    virtual ~TFOutputHandlerFactory() { }
};

void registerTFOutputHandlerFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr _node)
{
    // be sure to use the BASE CLASS as template parameter for the Registry!
    Registry<OutputHandlerFactory>::registerFactory(std::make_shared<TFOutputHandlerFactory>(_node));
}

} // namespace