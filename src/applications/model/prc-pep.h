/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Universidad de la Republica - Uruguay
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Matías Richart <mrichart@fing.edu.uy>
 */

#ifndef PRC_PEP_H
#define PRC_PEP_H

#include "ns3/application.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/ipv4-address.h"
#include "ns3/ipv4.h"

namespace ns3 {

class Socket;
class Packet;

struct Command
{
	std::string action;
	uint32_t level;
	std::string station;
};

class PrcPep : public Application
{
public:
  static TypeId
  GetTypeId (void);

  PrcPep ();

  virtual ~PrcPep ();

  /**
   * \brief set the remote address and port
   * \param ip remote IP address
   * \param port remote port
   */
  void SetRemote (Ipv4Address ip, uint16_t port);  

protected:
  virtual void DoDispose (void);

private:

  virtual void StartApplication (void);
  virtual void StopApplication (void);

  void ConnectionSucceeded (Ptr<Socket> socket);
  void ConnectionFailed (Ptr<Socket> socket);
  void SendSub (void);
  void HandleRead (Ptr<Socket> socket);
  
  Command ParseActionPacket(Ptr<Packet> p);
  Ptr<Packet> CreateSubscriptionPacket(std::string filter);

  Ptr<Socket> m_socket;
  bool m_connected;
  EventId m_sendEvent;
  Ipv4Address m_rnrAddress;
  uint16_t m_rnrPort;
  Ipv4Address m_myAddress;
  uint16_t m_myPort;
  uint32_t m_updateTimer;

};

} // namespace ns3

#endif // PRC_PEP_H
