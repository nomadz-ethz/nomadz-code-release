/**
 * @file xboxJoystick.cpp
 *
 * Compile command: g++ xboxJoystick.cpp -o xboxJoystick -std=c++11 XBoxJoystick.cpp
 *
 * This file is subject to the terms of NomadZ 2022 License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

#include "XBoxJoystick.h"

bool openSocket(unsigned int port, int& robotSocket) {
  int server_fd;
  struct sockaddr_in address;
  int opt = 1;
  int addrlen = sizeof(address);

  // Creating socket file descriptor
  if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
    perror("socket failed");
    exit(EXIT_FAILURE);
    return false;
  }

  // Forcefully attaching socket to the port 8080
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
    perror("setsockopt");
    exit(EXIT_FAILURE);
    return false;
  }
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(port);

  // Forcefully attaching socket to the port 8080
  if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) {
    perror("bind failed");
    exit(EXIT_FAILURE);
    return false;
  }
  if (listen(server_fd, 3) < 0) {
    perror("listen");
    exit(EXIT_FAILURE);
    return false;
  }
  if ((robotSocket = accept(server_fd, (struct sockaddr*)&address, (socklen_t*)&addrlen)) < 0) {
    perror("accept");
    exit(EXIT_FAILURE);
    return false;
  }
  return true;
}

int main(int argc, char const* argv[]) {

  // Define parameters
  const unsigned int PORT = 8080;

  bool numCorrectlyEntered = false;
  std::string joystickNumStr = "";
  int joystickNum = 5;
  std::cout << "Please enter the number that lights up at the joystick's X button:" << std::endl;

  while (!numCorrectlyEntered) {

    std::cin >> joystickNumStr;
    joystickNum = std::stoi(joystickNumStr);

    if (joystickNum < 1 || joystickNum > 4) {
      std::cout << "Nope, number needs to be between 1 and 4. Try again:" << std::endl;
    } else {
      numCorrectlyEntered = true;
    }
  }

  const unsigned int joystickID = joystickNum - 1;

  while (true) {

    // Connect to xbox controller
    XBoxJoystick xboxJoystick;
    bool connected = xboxJoystick.init(joystickID);
    if (connected) {
      std::cout << "Joystick successfully connected" << std::endl;
    } else {
      std::cerr << "No connection to joystick" << std::endl;
      continue;
      // return 0;
    }

    // Open socket
    int robotSocket;
    std::cout << "Waiting until client for socket is opened..." << std::endl;
    bool socketOpen = openSocket(PORT, robotSocket);
    if (socketOpen) {
      std::cout << "Socket successfully connected" << std::endl;
    } else {
      std::cerr << "Socket could not be opened" << std::endl;
      continue;
      // return 0;
    }

    // Get joystick infos and send it over socket
    while (true) {
      connected = xboxJoystick.checkConnection();
      if (!connected) {
        std::cerr << "Lost connection to joystick" << std::endl;
        break;
        // return 0;
      }

      unsigned int buttonId;
      bool pressed;
      unsigned int axisId;
      double axisVal;
      xboxJoystick.getNextEvent(buttonId, pressed, axisId, axisVal);
      usleep(20 * 1000);

      // Get buttons and axes values
      std::string msg = "";
      msg += std::to_string(xboxJoystick.isButtonPressed(A_BUTTON));
      msg += " ";
      msg += std::to_string(xboxJoystick.isButtonPressed(B_BUTTON));
      msg += " ";
      msg += std::to_string(xboxJoystick.isButtonPressed(X_BUTTON));
      msg += " ";
      msg += std::to_string(xboxJoystick.isButtonPressed(Y_BUTTON));
      msg += " ";
      msg += std::to_string(xboxJoystick.isButtonPressed(LB_BUTTON));
      msg += " ";
      msg += std::to_string(xboxJoystick.isButtonPressed(RB_BUTTON));
      msg += " ";
      msg += std::to_string(xboxJoystick.isButtonPressed(BACK_BUTTON));
      msg += " ";
      msg += std::to_string(xboxJoystick.isButtonPressed(START_BUTTON));
      msg += " ";
      msg += std::to_string(xboxJoystick.isButtonPressed(CENTER_BUTTON));
      msg += " ";
      msg += std::to_string(xboxJoystick.isButtonPressed(LW_BUTTON));
      msg += " ";
      msg += std::to_string(xboxJoystick.isButtonPressed(RW_BUTTON));
      msg += " ";

      msg += std::to_string(xboxJoystick.getAxisState(LWH_AXIS));
      msg += " ";
      msg += std::to_string(xboxJoystick.getAxisState(LWV_AXIS));
      msg += " ";
      msg += std::to_string(xboxJoystick.getAxisState(RWH_AXIS));
      msg += " ";
      msg += std::to_string(xboxJoystick.getAxisState(RWV_AXIS));
      msg += " ";
      msg += std::to_string(xboxJoystick.getAxisState(RT_AXIS));
      msg += " ";
      msg += std::to_string(xboxJoystick.getAxisState(LT_AXIS));
      msg += " ";
      msg += std::to_string(xboxJoystick.getAxisState(CH_AXIS));
      msg += " ";
      msg += std::to_string(xboxJoystick.getAxisState(CV_AXIS));
      msg += " \n";
      // msg += "#";

      // Send message to client
      const char* message = msg.c_str();
      send(robotSocket, message, strlen(message), 0);
    }
  }
}
