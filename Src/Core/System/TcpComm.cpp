/**
 * @file TcpComm.cpp
 *
 * Implementation of class TcpComm.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:Thomas.Roefer@dfki.de">Thomas Röfer</a>
 */

#include "TcpComm.h"
#include "BHAssert.h"

#include <cerrno>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/ioctl.h>

#define ERRNO errno
#define RESET_ERRNO errno = 0
#define NON_BLOCK(socket) fcntl(socket, F_SETFL, O_NONBLOCK)
#define CLOSE(socket) close(socket)

TcpComm::TcpComm(const char* ip, int port, int maxPackageSendSize, int maxPackageReceiveSize)
    : createSocket(0), transferSocket(0), overallBytesSent(0), overallBytesReceived(0),
      maxPackageSendSize(maxPackageSendSize), maxPackageReceiveSize(maxPackageReceiveSize), wasConnected(false) {
  address.sin_family = AF_INET;
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#endif
  address.sin_port = htons(port);
#ifdef __clang__
#pragma clang diagnostic pop
#endif
  if (ip) { // connect as client?
    address.sin_addr.s_addr = inet_addr(ip);
  } else {
    createSocket = socket(AF_INET, SOCK_STREAM, 0);
    ASSERT(createSocket > 0);
    int val = 1;
    setsockopt(createSocket, SOL_SOCKET, SO_REUSEADDR, (char*)&val, sizeof(val));
    address.sin_addr.s_addr = INADDR_ANY;
    VERIFY(bind(createSocket, (sockaddr*)&address, sizeof(sockaddr_in)) == 0);
    VERIFY(listen(createSocket, SOMAXCONN) == 0);
    NON_BLOCK(createSocket);
  }
  checkConnection();
}

TcpComm::~TcpComm() {
  if (connected()) {
    closeTransferSocket();
  }
  if (createSocket > 0) {
    CLOSE(createSocket);
  }
}

bool TcpComm::checkConnection() {
  if (!connected()) {
#ifndef WIN32
    unsigned int addrlen = sizeof(sockaddr_in);
#else
    int addrlen = sizeof(sockaddr_in);
#endif

    if (createSocket) {
      transferSocket = accept(createSocket, (sockaddr*)&address, &addrlen);
    } else if (!wasConnected) {
      transferSocket = socket(AF_INET, SOCK_STREAM, 0);
      ASSERT(connected());
      if (connect(transferSocket, (sockaddr*)&address, sizeof(sockaddr_in)) != 0) {
        CLOSE(transferSocket);
        transferSocket = 0;
      }
    }

    if (connected()) {
      wasConnected = true;
      NON_BLOCK(transferSocket); // switch socket to nonblocking
#ifdef MACOSX
      int yes = 1;
      VERIFY(!setsockopt(transferSocket, SOL_SOCKET, SO_NOSIGPIPE, &yes, sizeof(yes)));
#endif
      if (maxPackageSendSize) {
        VERIFY(!setsockopt(transferSocket, SOL_SOCKET, SO_SNDBUF, (char*)&maxPackageSendSize, sizeof(maxPackageSendSize)));
      }
      if (maxPackageReceiveSize) {
        VERIFY(
          !setsockopt(transferSocket, SOL_SOCKET, SO_RCVBUF, (char*)&maxPackageReceiveSize, sizeof(maxPackageReceiveSize)));
      }
      return true;
    } else {
      return false;
    }
  } else {
    return true;
  }
}

void TcpComm::closeTransferSocket() {
  CLOSE(transferSocket);
  transferSocket = 0;
}

bool TcpComm::receive(unsigned char* buffer, int size, bool wait) {
  if (!checkConnection()) {
    return false;
  }

  if (!wait) {
    RESET_ERRNO;
#ifndef WIN32
    int received = recv(transferSocket, (char*)buffer, size, MSG_PEEK);
    if (received < size) {
      if (!received || (received < 0 && ERRNO != EWOULDBLOCK && ERRNO != EINPROGRESS)) {
        closeTransferSocket();
      }
      return false;
    }
#else
    char c;
    int received = recv(transferSocket, &c, 1, MSG_PEEK);
    if (!received || (received < 0 && ERRNO != EWOULDBLOCK && ERRNO != EINPROGRESS) ||
        ioctlsocket(transferSocket, FIONREAD, (u_long*)&received) != 0) {
      closeTransferSocket();
      return false;
    } else if (received == 0)
      return false;
#endif
  }

  int received = 0;
  while (received < size) {
    RESET_ERRNO;

    int received2 = recv(transferSocket, (char*)buffer + received, size - received, 0);

    if (!received2 || (received2 < 0 && ERRNO != EWOULDBLOCK && ERRNO != EINPROGRESS)) // error during reading of package
    {
      closeTransferSocket();
      return false;
    } else if (ERRNO == EWOULDBLOCK || ERRNO == EINPROGRESS) // wait for the rest
    {
      received2 = 0;
      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;
      fd_set rset;
      FD_ZERO(&rset);
      FD_SET(transferSocket, &rset);
      if (select(transferSocket + 1, &rset, 0, 0, &timeout) == -1) {
        closeTransferSocket();
        return false; // error while waiting
      }
    }
    received += received2;
    overallBytesReceived += received2;
  }
  return true; // ok, data received
}

int TcpComm::receiveSys(void* buffer, unsigned int len, int flags) {
  return recv(transferSocket, (char*)buffer, len, flags);
}

bool TcpComm::send(const unsigned char* buffer, int size) {
  if (!checkConnection()) {
    return false;
  }

  RESET_ERRNO;
  int sent = ::send(transferSocket, (const char*)buffer, size, 0);
  if (sent > 0) {
    overallBytesSent += sent;
    while (sent < size && (ERRNO == EWOULDBLOCK || ERRNO == EINPROGRESS || ERRNO == 0)) {
      timeval timeout;
      timeout.tv_sec = 0;
      timeout.tv_usec = 100000;
      fd_set wset;
      FD_ZERO(&wset);
      FD_SET(transferSocket, &wset);
      RESET_ERRNO;
      if (select(transferSocket + 1, 0, &wset, 0, &timeout) == -1) {
        break;
      }
      RESET_ERRNO;
      int sent2 = ::send(transferSocket, (const char*)buffer + sent, size - sent, 0);
      if (sent2 >= 0) {
        sent += sent2;
        overallBytesSent += sent;
      }
    }
  }

  if (ERRNO == 0 && sent == size) {
    return true;
  } else {
    closeTransferSocket();
    return false;
  }
}
