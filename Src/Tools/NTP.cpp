/**
 * @file NTP.cpp
 *
 * Representations and functions for time synchronisation inside
 * the team. Implementation of parts of the Network Time Protocol.
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is <a href="mailto:timlaue@informatik.uni-bremen.de">Tim Laue</a>
 */

#include "NTP.h"

NTP::NTP()
    : lastNTPRequestSent(0), numberOfReceivedNTPRequests(0), numberOfReceivedNTPResponses(0), localId(255),
      currentRemoteId(255) {}

NTP::NTP(unsigned char id)
    : lastNTPRequestSent(INT_MIN), numberOfReceivedNTPRequests(0), numberOfReceivedNTPResponses(0), localId(id),
      currentRemoteId(id) {}

bool NTP::doSynchronization(unsigned now, OutMessage& out) {
  // Process incoming NTP responses:
  for (int i = 0; i < numberOfReceivedNTPResponses; ++i) {
    NTPResponse& response = receivedNTPResponses[i];
    auto syncMeas = response.computeSynchronizationMeasurement();
    timeSyncBuffers[response.sender].add(syncMeas);
  }
  numberOfReceivedNTPResponses = 0;

  // Send NTP responses  to teammates?
  bool sendNTPData = numberOfReceivedNTPRequests > 0;

  // Process incoming NTP requests:
  for (int i = 0; i < numberOfReceivedNTPRequests; ++i) {
    NTPRequest& request = receivedNTPRequests[i];
    if (request.sender == request.id) {
      // Send a response to this request:
      NTPResponse response(request);
      out.bin << response;
      out.finishMessage(idNTPResponse);
    } else {
      // Should not happen anymore, IDs are preset.
      ASSERT(false);
    }
  }
  numberOfReceivedNTPRequests = 0;

  // Send NTP requests to teammates?
  if (now - lastNTPRequestSent >= NTP_REQUEST_INTERVAL) {
    out.bin << NTPRequest(localId);
    out.finishMessage(idNTPRequest);
    lastNTPRequestSent = now;
    sendNTPData = true;
  }

  return sendNTPData;
}

unsigned NTP::getRemoteTimeInLocalTime(unsigned remoteTime, unsigned char remoteId) const {
  if (remoteId < MAX_NUM_OF_NTP_CLIENTS) {
    int localTime = int(remoteTime) - timeSyncBuffers[remoteId].bestOffset;
    return (unsigned)localTime;
  }
  return 0;
}

unsigned NTP::getRoundTripLength(unsigned char remoteId) const {
  if (remoteId < MAX_NUM_OF_NTP_CLIENTS) {
    return timeSyncBuffers[remoteId].shortestRoundTrip;
  }
  return 0;
}

bool NTP::handleMessage(InMessage& message) {
  switch (message.getMessageID()) {
  case idNTPHeader: {
    message.bin >> currentRemoteId;
    message.bin >> sendTimeStamp;
    message.bin >> receiveTimeStamp;

    // packets from the future protection
    // if(currentRemoteId < MAX_NUM_OF_NTP_CLIENTS)
    // if(receiveTimeStamp < (sendTimeStamp - timeSyncBuffers[currentRemoteId].bestOffset))
    // timeSyncBuffers[currentRemoteId].bestOffset = sendTimeStamp - receiveTimeStamp;
  }
    return true;

  case idNTPIdentifier: {
    unsigned char receiver;
    message.bin >> receiver;
    if (receiver == localId) {
      message.bin >> localId;
    }
  }
    return true;

  case idNTPRequest:
    if (numberOfReceivedNTPRequests < MAX_NUM_OF_NTP_PACKAGES) {
      NTPRequest& ntpRequest = receivedNTPRequests[numberOfReceivedNTPRequests++];
      message.bin >> ntpRequest;
      ntpRequest.id = currentRemoteId;
      ntpRequest.origination = sendTimeStamp;
      ntpRequest.receipt = receiveTimeStamp;
    }
    return true;

  case idNTPResponse:
    if (numberOfReceivedNTPResponses < MAX_NUM_OF_NTP_PACKAGES) {
      NTPResponse& ntpResponse = receivedNTPResponses[numberOfReceivedNTPResponses];
      message.bin >> ntpResponse;
      // Check, if this response belongs to an own request:
      if (ntpResponse.receiver == localId) {
        ntpResponse.sender = currentRemoteId;
        ntpResponse.responseOrigination = sendTimeStamp;
        ntpResponse.responseReceipt = receiveTimeStamp;
        ++numberOfReceivedNTPResponses;
      }
    }
    return true;

  default:
    return false;
  }
}
