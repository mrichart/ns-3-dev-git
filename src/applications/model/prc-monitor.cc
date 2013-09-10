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

#include "ns3/wifi-remote-station-manager.h"
#include "ns3/rule-based-wifi-manager.h"
#include "ns3/wifi-net-device.h"
#include "ns3/log.h"
#include "ns3/ipv4-address.h"
#include "ns3/nstime.h"
#include "ns3/inet-socket-address.h"
#include "ns3/socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/yans-wifi-phy.h"
#include "prc-monitor.h"
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <fstream>

NS_LOG_COMPONENT_DEFINE ("PrcMonitor");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (PrcMonitor);

TypeId
PrcMonitor::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PrcMonitor")
    .SetParent<Application> ()
    .AddConstructor<PrcMonitor> ()
    .AddAttribute ("Upstream",
                   "The destination Ipv4Address of the outbound packets",
                   Ipv4AddressValue ("127.0.0.1"),
                   MakeIpv4AddressAccessor (&PrcMonitor::m_rnrAddress),
                   MakeIpv4AddressChecker ())
    .AddAttribute ("RnrPort", "The destination port of the outbound packets",
                   UintegerValue (8182),
                   MakeUintegerAccessor (&PrcMonitor::m_rnrPort),
                   MakeUintegerChecker<uint16_t> ())
   .AddAttribute ("MyHost",
				  "The source Ipv4Address of the outbound packets",
				  Ipv4AddressValue ("127.0.0.1"),
				  MakeIpv4AddressAccessor (&PrcMonitor::m_myAddress),
				  MakeIpv4AddressChecker ())
   .AddAttribute ("MyPort", "The source port of the outbound packets",
				  UintegerValue (9591),
				  MakeUintegerAccessor (&PrcMonitor::m_myPort),
				  MakeUintegerChecker<uint16_t> ())
   .AddAttribute ("UpdateTimer", "Time for updating stats in seconds",
				  UintegerValue (10),
				  MakeUintegerAccessor (&PrcMonitor::m_updateTimer),
				  MakeUintegerChecker<uint32_t> ())
    ;
  return tid;
}

PrcMonitor::PrcMonitor ()
  : m_socket (0),
	m_sendEvent (EventId ()),
	m_connected (false)
{
  NS_LOG_FUNCTION_NOARGS ();
}

PrcMonitor::~PrcMonitor ()
{
  NS_LOG_FUNCTION_NOARGS ();
}


void
PrcMonitor::SetUpstream (Ipv4Address ip, uint16_t port)
{
  m_rnrAddress = ip;
  m_rnrPort = port;
}

void
PrcMonitor::DoDispose (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  Application::DoDispose ();
}

void
PrcMonitor::StartApplication (void)
{
  NS_LOG_FUNCTION_NOARGS ();
  
  if (m_socket == 0)
    {
      TypeId tid = TypeId::LookupByName ("ns3::TcpSocketFactory");
      m_socket = Socket::CreateSocket (GetNode (), tid);
      InetSocketAddress local = InetSocketAddress (m_myAddress, m_myPort);
      m_socket->Bind (local);
      InetSocketAddress rnr = InetSocketAddress (m_rnrAddress, m_rnrPort);
      m_socket->Connect(rnr);
      m_socket->SetConnectCallback (
        MakeCallback (&PrcMonitor::ConnectionSucceeded, this),
        MakeCallback (&PrcMonitor::ConnectionFailed, this));
      m_sendEvent = Simulator::Schedule (Seconds (m_updateTimer), &PrcMonitor::UpdateStats, this);
    }
}

void
PrcMonitor::StopApplication ()
{
  NS_LOG_FUNCTION_NOARGS ();
  Simulator::Cancel (m_sendEvent);
}

void
PrcMonitor::SendSub ()
{
  //Create and send a subscription for all notifications to this host
  std::ostringstream filter;
  filter << "target_service=NS3-RMOON\n";
  Ptr<Packet> sub = CreateSubscriptionPacket(filter.str());
  int error = m_socket->Send(sub);
  if (error >= 0)
	  NS_LOG_INFO("Subscription sent, " << error << " bytes");
}

void
PrcMonitor::UpdateStats ()
{
  NS_LOG_FUNCTION_NOARGS ();
  if (m_connected && m_sendEvent.IsExpired ())
    {
      Ptr<Node> node = GetNode();
      Ptr<NetDevice> device = node->GetDevice(1); // It has to be the wifi device
      Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice> (device);
      Ptr<WifiRemoteStationManager> manager = wifiDevice->GetRemoteStationManager();
      Ptr<RuleBasedWifiManager> wifiManager = DynamicCast<RuleBasedWifiManager> (manager);

      std::vector <Mac48Address> stations = manager->GetAssociatedStations();

      for (std::vector<Mac48Address>::const_iterator i = stations.begin (); i != stations.end (); i++)
        {
    	  uint32_t prob = wifiManager->GetStats(*i);

    	  Ptr<Packet> p = CreateTrapPacket("sProb", prob);

    	  if ((m_socket->Send(p)) >= 0)
		    {
    		  NS_LOG_INFO ("Prob event sent to RNR" << " Time: " << (Simulator::Now ()).GetSeconds () << " sProb: " << prob);
		    }
		  else
		    {
			  NS_LOG_INFO ("Error while sending event to the RNR " << m_socket->GetErrno());
		    }
        }
    }
  else
    NS_LOG_INFO ("**Update stats error**");

  m_sendEvent = Simulator::Schedule (Seconds (m_updateTimer), &PrcMonitor::UpdateStats, this);
}

void
PrcMonitor::ConnectionSucceeded (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  NS_LOG_INFO ("PrcMonitor Connection succeeded");
  m_connected = true;
  SendSub();
}

void
PrcMonitor::ConnectionFailed (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
  NS_LOG_INFO ("PrcMonitor, Connection Failed");
}

Ptr<Packet>
PrcMonitor::CreateTrapPacket(std::string mib, uint32_t value)
{
	Ptr<UniformRandomVariable> urv = CreateObject<UniformRandomVariable> ();
	uint32_t nid = urv->GetInteger(0, 999999);
    std::ostringstream msg;
    msg 	<< "NOTIFICATION\n"
    		<< "notification_id=" << nid << "\n"
    		<< "message_type=trap\n"
    		<< "host=127.0.0.1\n"
    		<< "service=NS3-RMOON\n"
    		<< "target_service=AP0/lupa/pdp\n"
    		<< "target_host=127.0.0.1\n"
    		<< "timestamp=" << Simulator::Now () << "\n"
    		<< "mib=" << mib << "\n"
    		<< "value=" << value << "\n"
    		<<"END\n";

    size_t size = strlen(msg.str().c_str());
    uint8_t* data = new uint8_t [size];
    memcpy (data, msg.str().c_str(), size);
    Ptr<Packet> p = Create<Packet> (data, size);
    return p;
}

Ptr<Packet>
PrcMonitor::CreateSubscriptionPacket(std::string filter)
{
	Ptr<UniformRandomVariable> urv = CreateObject<UniformRandomVariable> ();
	uint32_t sid = urv->GetInteger(0, 999999);
	int ttl = 20; //XXX
    std::ostringstream msg;
    msg 	<< "SUBSCRIBE\n"
    		<< "subscription_id=" << sid << "\n"
    		<< "host=127.0.0.1\n"
    		<< "service=NS3-RMOON\n"
    		<< "timestamp=" << Simulator::Now () << "\n"
    		<< "ttl=" << ttl << "\n"
    		<< "FILTER\n"
    		<< filter
    		<<"END\n";

    size_t size = strlen(msg.str().c_str());
    uint8_t* data = new uint8_t [size];
    memcpy (data, msg.str().c_str(), size);
    Ptr<Packet> p = Create<Packet> (data, size);
    return p;
}

} // Namespace ns3
