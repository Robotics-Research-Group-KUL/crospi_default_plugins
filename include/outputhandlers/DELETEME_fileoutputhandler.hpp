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

#include <fstream>

namespace etasl {
using namespace KDL;
/**
 * Example of a class that handles output suitable for simple, unbuffered, csv output.
 * It gets the output values from the context and outputs the results in a
 * tab-delimited file with a header containing the names of the signals
 *
 * Only handles double-expressions !
 *
 * This class is just an example, you can choose the structure of the class and its methods
 * yourself.
 */
class FileOutputHandler : public OutputHandler {
    struct Buffer {
        Expression<double>::Ptr outp;
        std::string name;
    };

    std::vector<Buffer> lst;
    std::vector<std::string> varnames;
    std::string filename;
    bool header;
    std::ofstream out;
    size_t linenr;
    size_t maxlines;
    size_t maxcounter;
    size_t counter;
    std::vector<double> data;
    std::string name;

public:
    /**
     * @brief
     * @param filename
     * @param header
     * @param _varnames
     */
    FileOutputHandler();

    /**
     * @return a name for the instance of this object 
     */
    const std::string& getName() const override;

    virtual bool construct(
        std::string name, 
        rclcpp_lifecycle::LifecycleNode::SharedPtr _node,
        const Json::Value& _parameters,
        std::shared_ptr<etasl::JsonChecker> _jsonchecker) override;

    /**
     * @brief
     * @param ctx
     * @param jnames
     * @param fnames
     */
    virtual bool initialize(
        Context::Ptr ctx,
        const std::vector<std::string>& jnames,
        const std::vector<std::string>& fnames) override;
    /**
     * @brief
     * @param jnames
     * @param jpos
     * @param jvel
     * @param fnames
     * @param fvel
     * @param fpos
     */
    virtual void update(
        const std::vector<std::string>& jnames,
        const Eigen::VectorXd& jpos, const Eigen::VectorXd& jvel,
        const std::vector<std::string>& fnames,
        const Eigen::VectorXd& fvel,
        const Eigen::VectorXd& fpos) override;

    /**
     * @brief
     */
    virtual void finalize();

}; // TopicOutputHandler
} // namespace KDL
