/**
 * @file UdpComm.h
 *
 * Simple UDP communication handling.
 * Basically a wrapper for a udp socket.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Armin
 */

#pragma once
#include <string>

struct sockaddr;
struct sockaddr_in;

/**
 * @class UdpComm
 */
class UdpComm {
public:
  /**
   * Constructor.
   */
  UdpComm();

  /**
   * Destructor.
   */
  ~UdpComm();

  /**
   * Set default target address.
   * @param ip The ip address of the host system.
   * @param port The port used for the connection.
   * \return Does a connection exist?
   */
  bool setTarget(const char* ip, int port);

  /**
   * Set broadcast mode.
   */
  bool setBroadcast(bool);

  bool setBlocking(bool);

  /**
   * Set multicast mode (please use multicast adresses to avoid confusion).
   */
  bool joinMulticast(const char*);

  /**
   * Set Time-To-Live (router hops).
   */
  bool setTTL(const char);

  /**
   * Set Loopback (receive own multicast packets). Default: enabled.
   */
  bool setLoopback(bool);

  bool setRcvBufSize(unsigned int);

  /**
   * bind to IN_ADDR_ANY to receive packets
   */
  bool bind(const char* addr, int port);

  /**
   * The function tries to read a package from a socket.
   * @return Number of bytes received or -1 in case of an error.
   */
  int read(char* data, int len, sockaddr_in& from);

  /**
   * The function tries to read a package from a socket.
   * @return Number of bytes received or -1 in case of an error.
   */
  int read(char* data, int len, unsigned int& ip);

  /**
   * The function tries to read a package from a socket.
   * @return Number of bytes received or -1 in case of an error.
   */
  int read(char* data, int len);

  /**
   * The function tries to read a package from a socket.
   * It only accepts a package from this host.
   * @return Number of bytes received or -1 in case of an error.
   */
  int readLocal(char* data, int len);

  /**
   * The function writes a package to a socket.
   * @return True if the package was written.
   */
  bool write(const char* data, const int len);

  static std::string getWifiBroadcastAddress();

private:
  struct sockaddr* target;
  int sock;
  bool resolve(const char*, int, struct sockaddr_in*);
};
