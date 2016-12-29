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

NS_LOG_COMPONENT_DEFINE ("SlicingManyStations");

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
  wifiStaNodes.Create (13);


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
  mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                                 "X", StringValue ("0.0"),
                                 "Y", StringValue ("0.0"),
                                 "Rho", StringValue ("ns3::UniformRandomVariable[Min=0|Max=10]"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNodes.Get (0));
  mobility.Install (wifiStaNodes);

  //Configure the IP stack
  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = address.Assign (wifiDevices);
  uint16_t port = 9;

  //Configure the CBR generator
  PacketSinkHelper sink0 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (0), port));
  ApplicationContainer apps_sink = sink0.Install (wifiStaNodes.Get (0));
  PacketSinkHelper sink1 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (1), port));
  apps_sink.Add(sink1.Install (wifiStaNodes.Get (1)));
  PacketSinkHelper sink2 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (2), port));
  apps_sink.Add(sink2.Install (wifiStaNodes.Get (2)));
  PacketSinkHelper sink3 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (3), port));
  apps_sink.Add(sink3.Install (wifiStaNodes.Get (3)));
  PacketSinkHelper sink4 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (4), port));
  apps_sink.Add(sink4.Install (wifiStaNodes.Get (4)));
  PacketSinkHelper sink5 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (5), port));
  apps_sink.Add(sink5.Install (wifiStaNodes.Get (5)));
  PacketSinkHelper sink6 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (6), port));
  apps_sink.Add(sink6.Install (wifiStaNodes.Get (6)));
  PacketSinkHelper sink7 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (7), port));
  apps_sink.Add(sink7.Install (wifiStaNodes.Get (7)));
  PacketSinkHelper sink8 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (8), port));
  apps_sink.Add(sink8.Install (wifiStaNodes.Get (8)));
  PacketSinkHelper sink9 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (9), port));
  apps_sink.Add(sink9.Install (wifiStaNodes.Get (9)));
  PacketSinkHelper sink10 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (10), port));
  apps_sink.Add(sink10.Install (wifiStaNodes.Get (10)));
  PacketSinkHelper sink11 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (11), port));
  apps_sink.Add(sink11.Install (wifiStaNodes.Get (11)));
  PacketSinkHelper sink12 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (12), port));
  apps_sink.Add(sink12.Install (wifiStaNodes.Get (12)));

  OnOffHelper onoff0 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (0), port));
  onoff0.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff0.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff0.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  ApplicationContainer apps_source = onoff0.Install (wifiApNodes.Get (0));

  OnOffHelper onoff1 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (1), port));
  onoff1.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff1.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff1.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff1.Install (wifiApNodes.Get (0)));

  OnOffHelper onoff2 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (2), port));
  onoff2.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff2.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff2.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff2.Install (wifiApNodes.Get (0)));

  OnOffHelper onoff3 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (3), port));
  onoff3.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff3.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff3.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff3.Install (wifiApNodes.Get (0)));

  OnOffHelper onoff4 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (4), port));
  onoff4.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff4.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff4.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff4.Install (wifiApNodes.Get (0)));

  OnOffHelper onoff5 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (5), port));
  onoff5.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff5.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff5.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff5.Install (wifiApNodes.Get (0)));

  OnOffHelper onoff6 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (6), port));
  onoff6.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff6.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff6.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff6.Install (wifiApNodes.Get (0)));

  OnOffHelper onoff7 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (7), port));
  onoff7.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff7.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff7.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff7.Install (wifiApNodes.Get (0)));

  OnOffHelper onoff8 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (8), port));
  onoff8.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff8.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff8.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff8.Install (wifiApNodes.Get (0)));

  OnOffHelper onoff9 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (9), port));
  onoff9.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff9.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff9.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff9.Install (wifiApNodes.Get (0)));

  OnOffHelper onoff10 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (10), port));
  onoff10.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff10.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff10.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff10.Install (wifiApNodes.Get (0)));

  OnOffHelper onoff11 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (11), port));
  onoff11.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff11.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff11.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff11.Install (wifiApNodes.Get (0)));

  OnOffHelper onoff12 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (12), port));
  onoff12.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff12.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff12.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff12.Install (wifiApNodes.Get (0)));

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
