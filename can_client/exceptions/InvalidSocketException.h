/**
 * @file InvalidSocketException.hpp
 * @author Simon Cahill (simonc@online.de)
 * @brief Contains the implementation of an exception that may be thrown when an invalid CAN socket is detected.
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
/**
  * @brief An exception that may be thrown when an error occurs while closing a CAN socket.
  */
class InvalidSocketException: public std::exception {
public:
    InvalidSocketException(std::string message, int32_t socket): _socket(socket), _message(message) {}

    ~InvalidSocketException() {}
public:
    const char* what() { return _message.c_str(); }
    const int32_t getSocket() const { return _socket; }
private:
    int32_t _socket;
    std::string _message;
};
} // namespace exception
} // namespace can
} // namespace nav
