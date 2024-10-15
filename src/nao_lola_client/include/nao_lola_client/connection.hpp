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

#pragma once

#include <string>

#include <boost/asio.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nao_lola_client {

  // Connection handler for the socket.
  class Connection {
  public:
    static constexpr size_t LOLA_PKT_SIZE{896};
    using socket_t = boost::asio::generic::stream_protocol::socket;
    using endpoint_t = boost::asio::generic::stream_protocol::endpoint;

    Connection();
    void connect(Connection::endpoint_t& endpoint);
    std::array<char, LOLA_PKT_SIZE> receive();
    void send(std::string data);

  private:
    boost::asio::io_service io_service_;
    socket_t socket_;
    rclcpp::Logger logger_;
  };
} // namespace nao_lola_client
