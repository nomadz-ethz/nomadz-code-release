// Copyright 2021 Kenji Brameld
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nao_lola_client/connection.hpp"

#include <chrono>
#include <string>

using resolver = boost::asio::ip::tcp::resolver;
namespace nao_lola_client {

  Connection::Connection() : socket_(io_service_), logger_(rclcpp::get_logger("lola connection")) {}

  void Connection::connect(Connection::endpoint_t& endpoint) {
    boost::system::error_code ec;

    do {
      socket_.connect(endpoint, ec);

      if (ec) {
        RCLCPP_ERROR(logger_, "Could not connect to LoLA: %s. Retrying in 2s.", ec.message().c_str());
        rclcpp::sleep_for(std::chrono::seconds(2));
      }
    } while (ec);
  }

  std::array<char, Connection::LOLA_PKT_SIZE> Connection::receive() {
    boost::system::error_code ec;
    std::array<char, Connection::LOLA_PKT_SIZE> data;
    socket_.receive(boost::asio::buffer(data), 0, ec);
    if (ec) {
      RCLCPP_ERROR(logger_, "Could not read from LoLA: %s", ec.message().c_str());
    }
    return data;
  }

  void Connection::send(std::string data) {
    socket_.send(boost::asio::buffer(data));
  }
} // namespace nao_lola_client
