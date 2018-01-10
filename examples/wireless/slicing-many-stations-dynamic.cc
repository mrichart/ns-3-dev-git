/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Universidad de la República - Uruguay
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

/**
 * This example program is designed to illustrate the behavior of
 * airtime slicing. The idea of airtime slicing is to assign
 * transmission airtime to stations to guarantee bitrate
 * requests.
 *
 * This simulation consist of 6 nodes, one AP and 5 STAs.
 * The AP generates UDP traffic with a CBR of 20 Mbps to each STA.
 * By default, the AP is at coordinate (0,0,0) and the STAs are all located
 * equidistantly from the AP.
 *
 * For testing airtime slicing with different channel capacities between the
 * AP and the STAs the are two options:
 * - To use an adaptation rate control mechanism (like Minstrel) and locate the
 * STAs at different distances from the AP,
 * - To use a constant rate for the transmission to each STA (a modification of
 * existent rate control mechanisms is needed).
 *
 * The output consists of:
 * - A plot of average throughput vs. time.
 *
 * Example usage:
 * ./waf --run "slicing-five-stations --outputFileName=slicing"
 *
 */

#include <sstream>
#include <fstream>
#include <math.h>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/stats-module.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("SlicingManyStationsDynamic");

int main (int argc, char *argv[])
{
  uint32_t rtsThreshold = 65535;
  std::string staManager = "ns3::MinstrelHtWifiManager";
  std::string apManager = "ns3::MinstrelHtWifiManager";
  std::string standard = "802.11n-5GHz";
  std::string outputFileName = "minstrelHT";
  uint32_t BE_MaxAmpduSize = 65535;
  bool shortGuardInterval = false;
  uint32_t chWidth = 20;
  int simuTime = 20;

  CommandLine cmd;
  cmd.AddValue ("staManager", "PRC Manager of the STA", staManager);
  cmd.AddValue ("apManager", "PRC Manager of the AP", apManager);
  cmd.AddValue ("standard", "Wifi Phy Standard", standard);
  cmd.AddValue ("shortGuardInterval", "Enable Short Guard Interval in all stations", shortGuardInterval);
  cmd.AddValue ("channelWidth", "Channel width of all the stations", chWidth);
  cmd.AddValue ("rtsThreshold", "RTS threshold", rtsThreshold);
  cmd.AddValue ("BE_MaxAmpduSize", "BE Mac A-MPDU size", BE_MaxAmpduSize);
  cmd.AddValue ("outputFileName", "Output filename", outputFileName);
  cmd.AddValue ("simuTime", "Simulation time in seconds", simuTime);
  cmd.Parse (argc, argv);

  // Define the APs
  NodeContainer wifiApNodes;
  wifiApNodes.Create (1);

  //Define the STAs
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (10);


  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());

  wifiPhy.Set("ShortGuardEnabled", BooleanValue(shortGuardInterval));

  NetDeviceContainer wifiApDevices;
  NetDeviceContainer wifiStaDevices;
  NetDeviceContainer wifiDevices;

  WifiHelper wifi;
  if (standard == "802.11a" || standard == "802.11b" || standard == "802.11g")
    {
      if (standard == "802.11a")
        {
          wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
        }
      else if (standard == "802.11b")
        {
          wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
        }
      else if (standard == "802.11g")
        {
          wifi.SetStandard (WIFI_PHY_STANDARD_80211g);
        }
      NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();

      //Configure the STA node
      wifi.SetRemoteStationManager (staManager, "RtsCtsThreshold", UintegerValue (rtsThreshold));

      Ssid ssid = Ssid ("AP");
      wifiMac.SetType ("ns3::StaWifiMac",
                       "Ssid", SsidValue (ssid));
      wifiStaDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiStaNodes));

      //Configure the AP node
      wifi.SetRemoteStationManager (apManager, "RtsCtsThreshold", UintegerValue (rtsThreshold));

      ssid = Ssid ("AP");
      wifiMac.SetType ("ns3::ApWifiMac",
                       "Ssid", SsidValue (ssid));
      wifiApDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiApNodes.Get (0)));
    }
  else if (standard == "802.11n-2.4GHz" || standard == "802.11n-5GHz")
    {
      if (standard == "802.11n-2.4GHz")
        {
          wifi.SetStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);
        }
      else if (standard == "802.11n-5GHz")
        {
          wifi.SetStandard (WIFI_PHY_STANDARD_80211n_5GHZ);
        }

      HtWifiMacHelper wifiMac = HtWifiMacHelper::Default ();

      //Configure the STA node
      wifi.SetRemoteStationManager (staManager, "RtsCtsThreshold", UintegerValue (rtsThreshold));

      Ssid ssid = Ssid ("AP");
      wifiMac.SetType ("ns3::StaWifiMac",
                       "Ssid", SsidValue (ssid),
                       "BE_MaxAmpduSize", UintegerValue (BE_MaxAmpduSize));
      wifiStaDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiStaNodes));

      //Configure the AP node
      wifi.SetRemoteStationManager (apManager, "RtsCtsThreshold", UintegerValue (rtsThreshold));

      ssid = Ssid ("AP");
      wifiMac.SetType ("ns3::ApWifiMac",
                       "Ssid", SsidValue (ssid),
                       "BE_MaxAmpduSize", UintegerValue (BE_MaxAmpduSize));
      wifiApDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiApNodes.Get (0)));
    }
  else if (standard == "802.11ac")
    {
      wifi.SetStandard (WIFI_PHY_STANDARD_80211ac);
      VhtWifiMacHelper wifiMac = VhtWifiMacHelper::Default ();

      //Configure the STA node
      wifi.SetRemoteStationManager (staManager, "RtsCtsThreshold", UintegerValue (rtsThreshold));

      Ssid ssid = Ssid ("AP");
      wifiMac.SetType ("ns3::StaWifiMac",
                       "Ssid", SsidValue (ssid),
                       "BE_MaxAmpduSize", UintegerValue (BE_MaxAmpduSize));
      wifiStaDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiStaNodes));

      //Configure the AP node
      wifi.SetRemoteStationManager (apManager, "RtsCtsThreshold", UintegerValue (rtsThreshold));

      ssid = Ssid ("AP");
      wifiMac.SetType ("ns3::ApWifiMac",
                       "Ssid", SsidValue (ssid),
                       "BE_MaxAmpduSize", UintegerValue (BE_MaxAmpduSize));
      wifiApDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiApNodes.Get (0)));
    }

  wifiDevices.Add (wifiStaDevices);
  wifiDevices.Add (wifiApDevices);

  // Set channel width
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (chWidth));

  // Configure the mobility.
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNodes.Get (0));

  mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
                                 "X", StringValue ("ns3::UniformRandomVariable[Min=-20|Max=20]"),
                                 "Y", StringValue ("ns3::UniformRandomVariable[Min=-20|Max=20]"));
  mobility.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (-20, 20, -20, 20)),
                             "Speed", StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=2.0]"),
                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=2.0]"));
  mobility.Install (wifiStaNodes);

  //Configure the IP stack
  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = address.Assign (wifiDevices);
  uint16_t port = 9;
  uint16_t port2 = 10;
  uint16_t port3 = 11;

  //Configure the CBR generator
  PacketSinkHelper sink1_1 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (0), port));
  ApplicationContainer apps_sink = sink1_1.Install (wifiStaNodes.Get (0));

  PacketSinkHelper sink2_1 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (1), port));
  apps_sink.Add(sink2_1.Install (wifiStaNodes.Get (1)));

  PacketSinkHelper sink3_1 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (2), port));
  apps_sink.Add(sink3_1.Install (wifiStaNodes.Get (2)));

  PacketSinkHelper sink4_1 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (3), port));
  apps_sink.Add(sink4_1.Install (wifiStaNodes.Get (3)));

  PacketSinkHelper sink4_2 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (3), port2));
  apps_sink.Add(sink4_2.Install (wifiStaNodes.Get (3)));

  PacketSinkHelper sink5_2 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (4), port2));
  apps_sink.Add(sink5_2.Install (wifiStaNodes.Get (4)));

  PacketSinkHelper sink6_2 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (5), port2));
  apps_sink.Add(sink6_2.Install (wifiStaNodes.Get (5)));

  PacketSinkHelper sink7_2 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (6), port2));
  apps_sink.Add(sink7_2.Install (wifiStaNodes.Get (6)));

  PacketSinkHelper sink7_3 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (6), port3));
  apps_sink.Add(sink7_3.Install (wifiStaNodes.Get (6)));

  PacketSinkHelper sink8_3 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (7), port3));
  apps_sink.Add(sink8_3.Install (wifiStaNodes.Get (7)));

  PacketSinkHelper sink9_3 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (8), port3));
  apps_sink.Add(sink9_3.Install (wifiStaNodes.Get (8)));

  PacketSinkHelper sink10_3 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (9), port3));
  apps_sink.Add(sink10_3.Install (wifiStaNodes.Get (9)));

  ///-------Slice 1----------------------------------------
  InetSocketAddress remote1_1 = InetSocketAddress (i.GetAddress (0), port);
  remote1_1.SetTos(32); //Set the TOS field to select the TID in the wifi queues. TID = TOS >> 5 (done in qos-utils)
  OnOffHelper onoff1_1 ("ns3::UdpSocketFactory", remote1_1);
  onoff1_1.SetConstantRate (DataRate ("7Mb/s"), 1420);
  onoff1_1.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff1_1.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  ApplicationContainer apps_source = onoff1_1.Install (wifiApNodes.Get (0));

  InetSocketAddress remote2_1 = InetSocketAddress (i.GetAddress (1), port);
  remote2_1.SetTos(32);
  OnOffHelper onoff2_1 ("ns3::UdpSocketFactory",  remote2_1);
  onoff2_1.SetConstantRate (DataRate ("7Mb/s"), 1420);
  onoff2_1.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff2_1.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff2_1.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote3_1 = InetSocketAddress (i.GetAddress (2), port);
  remote3_1.SetTos(32);
  OnOffHelper onoff3_1 ("ns3::UdpSocketFactory",  remote3_1);
  onoff3_1.SetConstantRate (DataRate ("7Mb/s"), 1420);
  onoff3_1.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff3_1.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff3_1.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote4_1 = InetSocketAddress (i.GetAddress (3), port);
  remote4_1.SetTos(32);
  OnOffHelper onoff4_1 ("ns3::UdpSocketFactory",  remote4_1);
  onoff4_1.SetConstantRate (DataRate ("7Mb/s"), 1420);
  onoff4_1.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff4_1.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff4_1.Install (wifiApNodes.Get (0)));

  ///-------Slice 2----------------------------------------
  InetSocketAddress remote4_2 = InetSocketAddress (i.GetAddress (3), port2);
  remote4_2.SetTos(64);
  OnOffHelper onoff4_2 ("ns3::UdpSocketFactory",  remote4_2);
  onoff4_2.SetConstantRate (DataRate ("7Mb/s"), 1420);
  onoff4_2.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff4_2.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff4_2.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote5_2 = InetSocketAddress (i.GetAddress (4), port2);
  remote5_2.SetTos(64);
  OnOffHelper onoff5_2 ("ns3::UdpSocketFactory",  remote5_2);
  onoff5_2.SetConstantRate (DataRate ("7Mb/s"), 1420);
  onoff5_2.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff5_2.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff5_2.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote6_2 = InetSocketAddress (i.GetAddress (5), port2);
  remote6_2.SetTos(64);
  OnOffHelper onoff6_2 ("ns3::UdpSocketFactory",  remote6_2);
  onoff6_2.SetConstantRate (DataRate ("7Mb/s"), 1420);
  onoff6_2.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff6_2.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff6_2.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote7_2 = InetSocketAddress (i.GetAddress (6), port2);
  remote7_2.SetTos(64);
  OnOffHelper onoff7_2 ("ns3::UdpSocketFactory",  remote7_2);
  onoff7_2.SetConstantRate (DataRate ("7Mb/s"), 1420);
  onoff7_2.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff7_2.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff7_2.Install (wifiApNodes.Get (0)));

  ///-------Slice 3----------------------------------------
  InetSocketAddress remote7_3 = InetSocketAddress (i.GetAddress (6), port3);
  remote7_3.SetTos(96);
  OnOffHelper onoff7_3 ("ns3::UdpSocketFactory",  remote7_3);
  onoff7_3.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff7_3.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff7_3.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff7_3.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote8_3 = InetSocketAddress (i.GetAddress (7), port3);
  remote8_3.SetTos(96);
  OnOffHelper onoff8_3 ("ns3::UdpSocketFactory",  remote8_3);
  onoff8_3.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff8_3.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff8_3.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff8_3.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote9_3 = InetSocketAddress (i.GetAddress (8), port3);
  remote9_3.SetTos(96);
  OnOffHelper onoff9_3 ("ns3::UdpSocketFactory",  remote9_3);
  onoff9_3.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff9_3.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff9_3.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff9_3.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote10_3 = InetSocketAddress (i.GetAddress (9), port3);
  remote10_3.SetTos(96);
  OnOffHelper onoff10_3 ("ns3::UdpSocketFactory", remote10_3);
  onoff10_3.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff10_3.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff10_3.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff10_3.Install (wifiApNodes.Get (0)));

  apps_sink.Start (Seconds (0.5));
  apps_sink.Stop (Seconds (simuTime));

  // Calculate Throughput using Flowmonitor
  //

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  Simulator::Stop (Seconds (simuTime));
  Simulator::Run ();

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      NS_LOG_UNCOND ("Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n");
      NS_LOG_UNCOND ("  Throughput: " << i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds () - i->second.timeFirstTxPacket.GetSeconds ()) / 1024 / 1024  << " Mbps\n");
    }

  Simulator::Destroy ();

  return 0;
}
