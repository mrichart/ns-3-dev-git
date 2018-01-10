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
  bool enablePcap = false;

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
  cmd.AddValue ("enablePcap", "Enable PCAP traces", enablePcap);
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
//  mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
//                                 "X", StringValue ("0.0"),
//                                 "Y", StringValue ("0.0"),
//                                 "Rho", StringValue ("ns3::UniformRandomVariable[Min=0|Max=5]"));
//  mobility.Install (wifiStaNodes);

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
  PacketSinkHelper sink0_1 ("ns3::TcpSocketFactory", InetSocketAddress (i.GetAddress (0), port));
  ApplicationContainer apps_sink = sink0_1.Install (wifiStaNodes.Get (0));

  PacketSinkHelper sink1_1 ("ns3::TcpSocketFactory", InetSocketAddress (i.GetAddress (1), port));
  apps_sink.Add(sink1_1.Install (wifiStaNodes.Get (1)));

  PacketSinkHelper sink2 ("ns3::TcpSocketFactory", InetSocketAddress (i.GetAddress (2), port));
  apps_sink.Add(sink2.Install (wifiStaNodes.Get (2)));

  PacketSinkHelper sink3_1 ("ns3::TcpSocketFactory", InetSocketAddress (i.GetAddress (3), port));
  apps_sink.Add(sink3_1.Install (wifiStaNodes.Get (3)));

  PacketSinkHelper sink1_3 ("ns3::TcpSocketFactory", InetSocketAddress (i.GetAddress (3), port2));
  apps_sink.Add(sink1_3.Install (wifiStaNodes.Get (3)));

  PacketSinkHelper sink4_2 ("ns3::TcpSocketFactory", InetSocketAddress (i.GetAddress (4), port2));
  apps_sink.Add(sink4_2.Install (wifiStaNodes.Get (4)));

  PacketSinkHelper sink5_2 ("ns3::TcpSocketFactory", InetSocketAddress (i.GetAddress (5), port2));
  apps_sink.Add(sink5_2.Install (wifiStaNodes.Get (5)));

  PacketSinkHelper sink6_2 ("ns3::TcpSocketFactory", InetSocketAddress (i.GetAddress (6), port2));
  apps_sink.Add(sink6_2.Install (wifiStaNodes.Get (6)));

  PacketSinkHelper sink6_3 ("ns3::TcpSocketFactory", InetSocketAddress (i.GetAddress (6), port3));
  apps_sink.Add(sink6_3.Install (wifiStaNodes.Get (6)));

  PacketSinkHelper sink7_3 ("ns3::TcpSocketFactory", InetSocketAddress (i.GetAddress (7), port3));
  apps_sink.Add(sink7_3.Install (wifiStaNodes.Get (7)));

  PacketSinkHelper sink8_3 ("ns3::TcpSocketFactory", InetSocketAddress (i.GetAddress (8), port3));
  apps_sink.Add(sink8_3.Install (wifiStaNodes.Get (8)));

  PacketSinkHelper sink9_3 ("ns3::TcpSocketFactory", InetSocketAddress (i.GetAddress (9), port3));
  apps_sink.Add(sink9_3.Install (wifiStaNodes.Get (9)));

  apps_sink.Start (Seconds (0.0));
  apps_sink.Stop (Seconds (simuTime));

  //-------------- User 1 - Slice 1 -----------------------
  InetSocketAddress remote0 = InetSocketAddress (i.GetAddress (0), port);
  remote0.SetTos(32); //Set the TOS field to select the TID in the wifi queues. TID = TOS >> 5 (done in qos-utils)
//  OnOffHelper onoff0 ("ns3::TcpSocketFactory", remote0);
//  onoff0.SetAttribute ("PacketSize", UintegerValue (1472));
//  onoff0.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//  onoff0.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
//  onoff0.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
//  ApplicationContainer apps_source = onoff0.Install (wifiApNodes.Get (0));

  BulkSendHelper source0 ("ns3::TcpSocketFactory", remote0);
  //Set the amount of data to send in bytes.  Zero is unlimited.
  source0.SetAttribute ("MaxBytes", UintegerValue (0));
  source0.SetAttribute ("SendSize", UintegerValue (1420));
  ApplicationContainer apps_source = source0.Install (wifiApNodes.Get (0));

  //-------------- User 2 - Slice 1 -----------------------
  InetSocketAddress remote1 = InetSocketAddress (i.GetAddress (1), port);
  remote1.SetTos(32);
  BulkSendHelper source1 ("ns3::TcpSocketFactory", remote1);
  //Set the amount of data to send in bytes.  Zero is unlimited.
  source1.SetAttribute ("MaxBytes", UintegerValue (0));
  source1.SetAttribute ("SendSize", UintegerValue (1420));
  apps_source.Add(source1.Install (wifiApNodes.Get (0)));
//  OnOffHelper onoff1 ("ns3::TcpSocketFactory", remote1);
//  onoff1.SetAttribute ("PacketSize", UintegerValue (1472));
//  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
//  onoff1.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
//  apps_source.Add(onoff1.Install (wifiApNodes.Get (0)));

  //-------------- User 3 - Slice 1 -----------------------
  InetSocketAddress remote2 = InetSocketAddress (i.GetAddress (2), port);
  remote2.SetTos(32);
  BulkSendHelper source2 ("ns3::TcpSocketFactory", remote2);
  //Set the amount of data to send in bytes.  Zero is unlimited.
  source2.SetAttribute ("MaxBytes", UintegerValue (0));
  source2.SetAttribute ("SendSize", UintegerValue (1420));
  apps_source.Add(source2.Install (wifiApNodes.Get (0)));
//  OnOffHelper onoff2 ("ns3::TcpSocketFactory", remote2);
//  onoff2.SetAttribute ("PacketSize", UintegerValue (1472));
//  onoff2.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//  onoff2.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
//  onoff2.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
//  apps_source.Add(onoff2.Install (wifiApNodes.Get (0)));

  //-------------- User 4 - Slice 1 -----------------------
  InetSocketAddress remote3 = InetSocketAddress (i.GetAddress (3), port);
  remote3.SetTos(32);
  BulkSendHelper source3 ("ns3::TcpSocketFactory", remote3);
  //Set the amount of data to send in bytes.  Zero is unlimited.
  source3.SetAttribute ("MaxBytes", UintegerValue (0));
  source3.SetAttribute ("SendSize", UintegerValue (1420));
  apps_source.Add(source3.Install (wifiApNodes.Get (0)));
//  OnOffHelper onoff3 ("ns3::TcpSocketFactory", remote3);
//  onoff3.SetAttribute ("PacketSize", UintegerValue (1472));
//  onoff3.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//  onoff3.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
//  onoff3.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
//  apps_source.Add(onoff3.Install (wifiApNodes.Get (0)));

  //-------------- User 4 - Slice 2 -----------------------
  InetSocketAddress remote3_2 = InetSocketAddress (i.GetAddress (3), port2);
  remote3_2.SetTos(64);
  BulkSendHelper source3_2 ("ns3::TcpSocketFactory", remote3_2);
  //Set the amount of data to send in bytes.  Zero is unlimited.
  source3_2.SetAttribute ("MaxBytes", UintegerValue (0));
  source3_2.SetAttribute ("SendSize", UintegerValue (1420));
  apps_source.Add(source3_2.Install (wifiApNodes.Get (0)));
//  OnOffHelper onoff3_2 ("ns3::TcpSocketFactory", remote3_2);
//  onoff3_2.SetAttribute ("PacketSize", UintegerValue (1472));
//  onoff3_2.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//  onoff3_2.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
//  onoff3_2.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
//  apps_source.Add(onoff3_2.Install (wifiApNodes.Get (0)));

  //-------------- User 5 - Slice 2 -----------------------
  InetSocketAddress remote4 = InetSocketAddress (i.GetAddress (4), port2);
  remote4.SetTos(64);
  BulkSendHelper source4 ("ns3::TcpSocketFactory", remote4);
  //Set the amount of data to send in bytes.  Zero is unlimited.
  source4.SetAttribute ("MaxBytes", UintegerValue (0));
  source4.SetAttribute ("SendSize", UintegerValue (1420));
  apps_source.Add(source4.Install (wifiApNodes.Get (0)));
//  OnOffHelper onoff4 ("ns3::TcpSocketFactory", remote4);
//  onoff4.SetAttribute ("PacketSize", UintegerValue (1472));
//  onoff4.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//  onoff4.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
//  onoff4.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
//  apps_source.Add(onoff4.Install (wifiApNodes.Get (0)));

  //-------------- User 6 - Slice 2 -----------------------
  InetSocketAddress remote5 = InetSocketAddress (i.GetAddress (5), port2);
  remote5.SetTos(64);
  BulkSendHelper source5 ("ns3::TcpSocketFactory", remote5);
  //Set the amount of data to send in bytes.  Zero is unlimited.
  source5.SetAttribute ("MaxBytes", UintegerValue (0));
  source5.SetAttribute ("SendSize", UintegerValue (1420));
  apps_source.Add(source5.Install (wifiApNodes.Get (0)));
//  OnOffHelper onoff5 ("ns3::TcpSocketFactory", remote5);
//  onoff5.SetAttribute ("PacketSize", UintegerValue (1472));
//  onoff5.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//  onoff5.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
//  onoff5.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
//  apps_source.Add(onoff5.Install (wifiApNodes.Get (0)));

  //-------------- User 7 - Slice 2 -----------------------
  InetSocketAddress remote6 = InetSocketAddress (i.GetAddress (6), port2);
  remote6.SetTos(64);
  BulkSendHelper source6 ("ns3::TcpSocketFactory", remote6);
  //Set the amount of data to send in bytes.  Zero is unlimited.
  source6.SetAttribute ("MaxBytes", UintegerValue (0));
  source6.SetAttribute ("SendSize", UintegerValue (1420));
  apps_source.Add(source6.Install (wifiApNodes.Get (0)));
//  OnOffHelper onoff6 ("ns3::TcpSocketFactory", remote6);
//  onoff6.SetAttribute ("PacketSize", UintegerValue (1472));
//  onoff6.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//  onoff6.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
//  onoff6.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
//  apps_source.Add(onoff6.Install (wifiApNodes.Get (0)));

  //-------------- User 7 - Slice 3 -----------------------
  InetSocketAddress remote6_2 = InetSocketAddress (i.GetAddress (6), port3);
  remote6_2.SetTos(96);
  BulkSendHelper source6_2 ("ns3::TcpSocketFactory", remote6_2);
  //Set the amount of data to send in bytes.  Zero is unlimited.
  source6_2.SetAttribute ("MaxBytes", UintegerValue (0));
  source6_2.SetAttribute ("SendSize", UintegerValue (1420));
  apps_source.Add(source6_2.Install (wifiApNodes.Get (0)));
//  OnOffHelper onoff6_2 ("ns3::TcpSocketFactory", remote6_2);
//  onoff6_2.SetAttribute ("PacketSize", UintegerValue (1472));
//  onoff6_2.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//  onoff6_2.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
//  onoff6_2.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
//  apps_source.Add(onoff6_2.Install (wifiApNodes.Get (0)));

  //-------------- User 8 - Slice 3 -----------------------
  InetSocketAddress remote7 = InetSocketAddress (i.GetAddress (7), port3);
  remote7.SetTos(96);
  BulkSendHelper source7 ("ns3::TcpSocketFactory", remote7);
  //Set the amount of data to send in bytes.  Zero is unlimited.
  source7.SetAttribute ("MaxBytes", UintegerValue (0));
  source7.SetAttribute ("SendSize", UintegerValue (1420));
  apps_source.Add(source7.Install (wifiApNodes.Get (0)));
//  OnOffHelper onoff7 ("ns3::TcpSocketFactory", remote7);
//  onoff7.SetAttribute ("PacketSize", UintegerValue (1472));
//  onoff7.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//  onoff7.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
//  onoff7.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
//  apps_source.Add(onoff7.Install (wifiApNodes.Get (0)));

  //-------------- User 9 - Slice 3 -----------------------
  InetSocketAddress remote8 = InetSocketAddress (i.GetAddress (8), port3);
  remote8.SetTos(96);
  BulkSendHelper source8 ("ns3::TcpSocketFactory", remote8);
  //Set the amount of data to send in bytes.  Zero is unlimited.
  source8.SetAttribute ("MaxBytes", UintegerValue (0));
  source8.SetAttribute ("SendSize", UintegerValue (1420));
  apps_source.Add(source8.Install (wifiApNodes.Get (0)));
//  OnOffHelper onoff8 ("ns3::TcpSocketFactory", remote8);
//  onoff8.SetAttribute ("PacketSize", UintegerValue (1472));
//  onoff8.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//  onoff8.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
//  onoff8.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
//  apps_source.Add(onoff8.Install (wifiApNodes.Get (0)));

  //-------------- User 10 - Slice 3 -----------------------
  InetSocketAddress remote9 = InetSocketAddress (i.GetAddress (9), port3);
  remote9.SetTos(96);
  BulkSendHelper source9 ("ns3::TcpSocketFactory", remote9);
  //Set the amount of data to send in bytes.  Zero is unlimited.
  source9.SetAttribute ("MaxBytes", UintegerValue (0));
  source9.SetAttribute ("SendSize", UintegerValue (1420));
  apps_source.Add(source9.Install (wifiApNodes.Get (0)));
//    OnOffHelper onoff9 ("ns3::TcpSocketFactory", remote9);
//    onoff9.SetAttribute ("PacketSize", UintegerValue (1472));
//    onoff9.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
//    onoff9.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
//    onoff9.SetAttribute ("DataRate", DataRateValue (DataRate ("100Mbps")));
//    apps_source.Add(onoff9.Install (wifiApNodes.Get (0)));

  apps_source.Start(Seconds(1.0));
  apps_source.Stop(Seconds (simuTime));

  // Calculate Throughput using Flowmonitor
  //

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  if (enablePcap)
    {
      wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
      wifiPhy.EnablePcap ("AccessPoint", wifiApDevices);
      wifiPhy.EnablePcap ("Station", wifiStaDevices);
    }

  Simulator::Stop (Seconds (simuTime));
  Simulator::Run ();

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      NS_LOG_UNCOND ("Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n");
      NS_LOG_UNCOND ("  Throughput:   " << i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds () - i->second.timeFirstTxPacket.GetSeconds ()) / 1000000  << " Mbps");
      NS_LOG_UNCOND ("  Mean delay:   " << i->second.delaySum.GetSeconds () / i->second.rxPackets);
      NS_LOG_UNCOND ("  Mean jitter:   " << i->second.jitterSum.GetSeconds () / (i->second.rxPackets - 1));
    }

  Simulator::Destroy ();

  return 0;
}
