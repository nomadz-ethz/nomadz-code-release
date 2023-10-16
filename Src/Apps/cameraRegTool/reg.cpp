/**
 * @file reg.cpp
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <bits/stdc++.h>

#include <fcntl.h>
#include <linux/uvcvideo.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>
#include <linux/usb/video.h>
#include <linux/uvcvideo.h>

int main(int argc, char** argv) {
  // top camera
  int fd = open("/dev/video-top", O_RDWR);
  assert(fd >= 0);

  if (argc == 3) {
    std::stringstream ss;
    uint16_t address, value;
    ss << std::hex << argv[1] << " " << argv[2];
    ss >> address >> value;

    // construct the query struct
    struct uvc_xu_control_query xu_query;
    std::memset(&xu_query, 0, sizeof(xu_query));
    xu_query.unit = 3;
    xu_query.selector = 0x0e;
    xu_query.query = UVC_SET_CUR;
    xu_query.size = 5;

    std::uint8_t data[5];
    std::memset(data, 0, 5);
    data[0] = 1;
    data[1] = address >> 8;
    data[2] = address & 0xff;
    data[3] = value >> 8;
    data[4] = value & 0xff;
    xu_query.data = data;
    if (-1 == ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query)) {
      std::cerr << "(ERROR) : UVC_SET_CUR failed: " << std::strerror(errno);
      assert(false);
    }

  } else if (argc == 2) {
    std::stringstream ss;
    uint16_t address;
    ss << std::hex << argv[1];
    ss >> address;

    // construct the query struct
    uvc_xu_control_query xu_query;
    std::memset(&xu_query, 0, sizeof(xu_query));
    xu_query.unit = 3;
    // selecting register control on the microcontroller?
    xu_query.selector = 0x0e;
    xu_query.query = UVC_SET_CUR;
    xu_query.size = 5;

    // contruct the data block
    std::uint8_t data[5];
    std::memset(data, 0, 5);
    // set flag to "Read"
    data[0] = 0;
    // split 16-bit address into two 8-bit parts
    data[1] = address >> 8;
    data[2] = address & 0xff;
    xu_query.data = data;
    if (-1 == ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query)) {
      std::cerr << "(ERROR) : UVCIOC_CTRL_QUERY failed: " << std::strerror(errno);
      assert(false);
    }

    // wait for the microcontroller to query the register from the camera
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // query the value
    xu_query.query = UVC_GET_CUR;
    if (-1 == ioctl(fd, UVCIOC_CTRL_QUERY, &xu_query)) {
      std::cerr << "(ERROR) : UVCIOC_CTRL_QUERY failed: " << std::strerror(errno);
      assert(false);
    }

    // the resulting concatenated value
    std::cout << std::hex << ((data[3] << 8) | data[4]) << std::endl;
  } else {
    std::cout << "incorrect number of arguments" << std::endl;
  }
}
