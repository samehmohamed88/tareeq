/**
 * @file CANCloseException.hpp
 * @author Simon Cahill (simonc@online.de)
 * @brief Contains the implementation of an exception that may be thrown when an error occurs while closing a CAN socket.
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

/// @brief An exception that may be thrown when an error occurs while closing a CAN socket.
class CANCloseException: public std::exception {
public:
    CANCloseException(std::string message): _message(message) {}
    ~CANCloseException() {}

public:
    const char* what() { return _message.c_str(); }

private:
    std::string _message;
};

} // namespace can
} // namespace nav
