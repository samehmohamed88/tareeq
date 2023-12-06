/**
 * @file CANInitException.hpp
 * @author Simon Cahill (simonc@online.de)
 * @brief Contains the implementation of an exception that is thrown when a CAN socket couldn't be inintialised.
 * @version 0.1
 * @date 2020-07-02
 * 
 * @copyright Copyright (c) 2020 Simon Cahill
 *
 *  Copyright 2020 Simon Cahill
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#pragma once

#include <exception>
#include <string>

namespace nav {
namespace can {
namespace exception {
/// @brief An exception that may be thrown when an error occurred while initialising a CAN socket.
class CANInitException : public std::exception {
public:
    CANInitException(std::string message) : _message(message) {}

    virtual ~CANInitException() {}

public:
    const char *what() { return _message.c_str(); }

private:
    std::string _message;
};
} // namespace exceptions
} // namespace can
} // namespace nav
