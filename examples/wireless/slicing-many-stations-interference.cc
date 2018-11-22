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

class NodeStatistics
{
public:
  NodeStatistics (NetDeviceContainer aps, NetDeviceContainer stas);

  void CheckStatistics (double time);

  void RxCallback (std::string path, Ptr<const Packet> packet, const Address &from);
  void StateCallback (std::string path, Time init, Time duration, enum WifiPhy::State state);

  Gnuplot2dDataset GetDatafile ();
  Gnuplot2dDataset GetPowerDatafile ();
  Gnuplot2dDataset GetIdleDatafile ();
  Gnuplot2dDataset GetBusyDatafile ();
  Gnuplot2dDataset GetTxDatafile ();
  Gnuplot2dDataset GetRxDatafile ();

  double GetBusyTime ();

private:
  typedef std::vector<std::pair<Time,WifiMode> > TxTime;
  void SetupPhy (Ptr<WifiPhy> phy);
  Time GetCalcTxTime (WifiMode mode);

  uint32_t m_bytesTotal;

  double totalTime;
  double busyTime;
  double idleTime;
  double txTime;
  double rxTime;

  double totalBusyTime;
  double totalIdleTime;
  double totalTxTime;
  double totalRxTime;

  Ptr<WifiPhy> myPhy;

  Gnuplot2dDataset m_output;
  Gnuplot2dDataset m_output_power;
  Gnuplot2dDataset m_output_idle;
  Gnuplot2dDataset m_output_busy;
  Gnuplot2dDataset m_output_rx;
  Gnuplot2dDataset m_output_tx;
};

NodeStatistics::NodeStatistics (NetDeviceContainer aps, NetDeviceContainer stas)
{
  totalTime = 0;

  busyTime = 0;
  idleTime = 0;
  txTime = 0;
  rxTime = 0;

  totalBusyTime = 0;
  totalIdleTime = 0;
  totalTxTime = 0;
  totalRxTime = 0;

  m_bytesTotal = 0;

  m_output.SetTitle ("Throughput Mbits/s");
  m_output_idle.SetTitle ("Idle Time");
  m_output_busy.SetTitle ("Busy Time");
  m_output_rx.SetTitle ("RX Time");
  m_output_tx.SetTitle ("TX Time");
}

void
NodeStatistics::StateCallback (std::string path, Time init, Time duration, enum WifiPhy::State state)
{
  if (state == WifiPhy::CCA_BUSY)
    {
      busyTime += duration.GetSeconds ();
      totalBusyTime += duration.GetSeconds ();
    }
  else if (state == WifiPhy::IDLE)
    {
      idleTime += duration.GetSeconds ();
      totalIdleTime += duration.GetSeconds ();
    }
  else if (state == WifiPhy::TX)
    {
      txTime += duration.GetSeconds ();
      totalTxTime += duration.GetSeconds ();
    }
  else if (state == WifiPhy::RX)
    {
      rxTime += duration.GetSeconds ();
      totalRxTime += duration.GetSeconds ();
    }
}

void
NodeStatistics::RxCallback (std::string path, Ptr<const Packet> packet, const Address &from)
{
  m_bytesTotal += packet->GetSize ();
}

void
NodeStatistics::CheckStatistics (double time)
{
  double mbs = ((m_bytesTotal * 8.0) / (1000000 * time));
  m_bytesTotal = 0;
  totalTime = 0;
  m_output.Add ((Simulator::Now ()).GetSeconds (), mbs);

  m_output_idle.Add ((Simulator::Now ()).GetSeconds (), idleTime * 100);
  m_output_busy.Add ((Simulator::Now ()).GetSeconds (), busyTime * 100);
  m_output_tx.Add ((Simulator::Now ()).GetSeconds (), txTime * 100);
  m_output_rx.Add ((Simulator::Now ()).GetSeconds (), rxTime * 100);

  busyTime = 0;
  idleTime = 0;
  txTime = 0;
  rxTime = 0;

  Simulator::Schedule (Seconds (time), &NodeStatistics::CheckStatistics, this, time);
}

Gnuplot2dDataset
NodeStatistics::GetDatafile ()
{
  return m_output;
}

Gnuplot2dDataset
NodeStatistics::GetIdleDatafile ()
{
  return m_output_idle;
}

Gnuplot2dDataset
NodeStatistics::GetBusyDatafile ()
{
  return m_output_busy;
}

Gnuplot2dDataset
NodeStatistics::GetRxDatafile ()
{
  return m_output_rx;
}

Gnuplot2dDataset
NodeStatistics::GetTxDatafile ()
{
  return m_output_tx;
}

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
  int simuTime = 1440;
  int interfLowStart = 0.5;
  int interfLowStop = 480;
  int interfMedStart = 480;
  int interfMedStop = 720;
  int interfHiStart = 720;
  int interfHiStop = 1080;

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
  cmd.AddValue ("interfLowStart", "Start time for low interference generator", interfLowStart);
  cmd.AddValue ("interfLowStart", "Start time for low interference generator", interfLowStop);
  cmd.AddValue ("interfMedStart", "Start time for medium interference generator", interfMedStart);
  cmd.AddValue ("interfMedStart", "Start time for medium interference generator", interfMedStop);
  cmd.AddValue ("interfHiStart", "Start time for high interference generator", interfHiStart);
  cmd.AddValue ("interfHiStart", "Start time for high interference generator", interfHiStop);
  cmd.Parse (argc, argv);

  // Define the APs
  NodeContainer wifiApNodes;
  wifiApNodes.Create (1);

  //Define the STAs
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (10);

  // Define the interferer AP
  NodeContainer wifiInterfererAp;
  wifiInterfererAp.Create (1);

  //Define the interferer STA
  NodeContainer wifiInterfererSta;
  wifiInterfererSta.Create (1);


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

      //Configure the interferer STA

      ssid = Ssid ("I");
      wifiMac.SetType ("ns3::StaWifiMac",
		       "Ssid", SsidValue (ssid));
      wifiStaDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiInterfererSta));

      //Configure the interferer AP

      ssid = Ssid ("I");
      wifiMac.SetType ("ns3::ApWifiMac",
		       "Ssid", SsidValue (ssid));
      wifiApDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiInterfererAp));
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
      wifi.SetRemoteStationManager (apManager, "RtsCtsThreshold", UintegerValue (rtsThreshold), "PrintStats", BooleanValue(false));

      ssid = Ssid ("AP");
      wifiMac.SetType ("ns3::ApWifiMac",
                       "Ssid", SsidValue (ssid),
                       "BE_MaxAmpduSize", UintegerValue (BE_MaxAmpduSize));
      wifiApDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiApNodes.Get (0)));

      //Configure the interferer STA

      ssid = Ssid ("I");
      wifiMac.SetType ("ns3::StaWifiMac",
		       "Ssid", SsidValue (ssid));
      wifiStaDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiInterfererSta));

      //Configure the interferer AP

      ssid = Ssid ("I");
      wifiMac.SetType ("ns3::ApWifiMac",
		       "Ssid", SsidValue (ssid),
	               "BeaconInterval", TimeValue (MicroSeconds (103424)));  //for avoiding collisions
      wifiApDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiInterfererAp));
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

      //Configure the interferer STA

      ssid = Ssid ("I");
      wifiMac.SetType ("ns3::StaWifiMac",
		       "Ssid", SsidValue (ssid));
      wifiStaDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiInterfererSta));

      //Configure the interferer AP

      ssid = Ssid ("I");
      wifiMac.SetType ("ns3::ApWifiMac",
		       "Ssid", SsidValue (ssid));
      wifiApDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiInterfererAp));
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

  positionAlloc->Add (Vector (10.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiInterfererAp.Get (0));

  positionAlloc->Add (Vector (10.0, 5.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiInterfererSta.Get (0));

  mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                                 "X", StringValue ("0.0"),
                                 "Y", StringValue ("0.0"),
                                 "Rho", StringValue ("ns3::UniformRandomVariable[Min=0|Max=5]"));
  mobility.Install (wifiStaNodes);

  //Configure the IP stack
  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);
  stack.Install (wifiInterfererAp);
  stack.Install (wifiInterfererSta);
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
  onoff2_1.SetAttribute ("StartTime", TimeValue (Seconds (240)));
  onoff2_1.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff2_1.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote3_1 = InetSocketAddress (i.GetAddress (2), port);
  remote3_1.SetTos(32);
  OnOffHelper onoff3_1 ("ns3::UdpSocketFactory",  remote3_1);
  onoff3_1.SetConstantRate (DataRate ("7Mb/s"), 1420);
  onoff3_1.SetAttribute ("StartTime", TimeValue (Seconds (360)));
  onoff3_1.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff3_1.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote4_1 = InetSocketAddress (i.GetAddress (3), port);
  remote4_1.SetTos(32);
  OnOffHelper onoff4_1 ("ns3::UdpSocketFactory",  remote4_1);
  onoff4_1.SetConstantRate (DataRate ("7Mb/s"), 1420);
  onoff4_1.SetAttribute ("StartTime", TimeValue (Seconds (360)));
  onoff4_1.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff4_1.Install (wifiApNodes.Get (0)));

  ///-------Slice 2----------------------------------------
  InetSocketAddress remote4_2 = InetSocketAddress (i.GetAddress (3), port2);
  remote4_2.SetTos(64);
  OnOffHelper onoff4_2 ("ns3::UdpSocketFactory",  remote4_2);
  onoff4_2.SetConstantRate (DataRate ("7Mb/s"), 1420);
  onoff4_2.SetAttribute ("StartTime", TimeValue (Seconds (540)));
  onoff4_2.SetAttribute ("StopTime", TimeValue (Seconds (1260)));
  apps_source.Add(onoff4_2.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote5_2 = InetSocketAddress (i.GetAddress (4), port2);
  remote5_2.SetTos(64);
  OnOffHelper onoff5_2 ("ns3::UdpSocketFactory",  remote5_2);
  onoff5_2.SetConstantRate (DataRate ("7Mb/s"), 1420);
  onoff5_2.SetAttribute ("StartTime", TimeValue (Seconds (540)));
  onoff5_2.SetAttribute ("StopTime", TimeValue (Seconds (1260)));
  apps_source.Add(onoff5_2.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote6_2 = InetSocketAddress (i.GetAddress (5), port2);
  remote6_2.SetTos(64);
  OnOffHelper onoff6_2 ("ns3::UdpSocketFactory",  remote6_2);
  onoff6_2.SetConstantRate (DataRate ("7Mb/s"), 1420);
  onoff6_2.SetAttribute ("StartTime", TimeValue (Seconds (540)));
  onoff6_2.SetAttribute ("StopTime", TimeValue (Seconds (1260)));
  apps_source.Add(onoff6_2.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote7_2 = InetSocketAddress (i.GetAddress (6), port2);
  remote7_2.SetTos(64);
  OnOffHelper onoff7_2 ("ns3::UdpSocketFactory",  remote7_2);
  onoff7_2.SetConstantRate (DataRate ("7Mb/s"), 1420);
  onoff7_2.SetAttribute ("StartTime", TimeValue (Seconds (540)));
  onoff7_2.SetAttribute ("StopTime", TimeValue (Seconds (1260)));
  apps_source.Add(onoff7_2.Install (wifiApNodes.Get (0)));

  ///-------Slice 3----------------------------------------
  InetSocketAddress remote7_3 = InetSocketAddress (i.GetAddress (6), port3);
  remote7_3.SetTos(96);
  OnOffHelper onoff7_3 ("ns3::UdpSocketFactory",  remote7_3);
  onoff7_3.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff7_3.SetAttribute ("StartTime", TimeValue (Seconds (660)));
  onoff7_3.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff7_3.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote8_3 = InetSocketAddress (i.GetAddress (7), port3);
  remote8_3.SetTos(96);
  OnOffHelper onoff8_3 ("ns3::UdpSocketFactory",  remote8_3);
  onoff8_3.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff8_3.SetAttribute ("StartTime", TimeValue (Seconds (660)));
  onoff8_3.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff8_3.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote9_3 = InetSocketAddress (i.GetAddress (8), port3);
  remote9_3.SetTos(96);
  OnOffHelper onoff9_3 ("ns3::UdpSocketFactory",  remote9_3);
  onoff9_3.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff9_3.SetAttribute ("StartTime", TimeValue (Seconds (900)));
  onoff9_3.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff9_3.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote10_3 = InetSocketAddress (i.GetAddress (9), port3);
  remote10_3.SetTos(96);
  OnOffHelper onoff10_3 ("ns3::UdpSocketFactory", remote10_3);
  onoff10_3.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff10_3.SetAttribute ("StartTime", TimeValue (Seconds (900)));
  onoff10_3.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff10_3.Install (wifiApNodes.Get (0)));

  //Configure interference generator

  PacketSinkHelper sinkInterferer ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (10), port));
  apps_sink = sinkInterferer.Install (wifiInterfererSta.Get (0));

  InetSocketAddress remote = InetSocketAddress (i.GetAddress (10), port);
  OnOffHelper onoffLow ("ns3::UdpSocketFactory", remote);
  onoffLow.SetConstantRate (DataRate ("5Mb/s"), 1420);
  onoffLow.SetAttribute ("StartTime", TimeValue (Seconds (interfLowStart)));
  onoffLow.SetAttribute ("StopTime", TimeValue (Seconds (interfLowStop)));
  onoffLow.Install (wifiInterfererAp.Get (0));

  OnOffHelper onoffMed ("ns3::UdpSocketFactory", remote);
  onoffMed.SetConstantRate (DataRate ("20Mb/s"), 1420);
  onoffMed.SetAttribute ("StartTime", TimeValue (Seconds (interfMedStart)));
  onoffMed.SetAttribute ("StopTime", TimeValue (Seconds (interfMedStop)));
  onoffMed.Install (wifiInterfererAp.Get (0));

  OnOffHelper onoffHi ("ns3::UdpSocketFactory", remote);
  onoffHi.SetConstantRate (DataRate ("40Mb/s"), 1420);
  onoffHi.SetAttribute ("StartTime", TimeValue (Seconds (interfHiStart)));
  onoffHi.SetAttribute ("StopTime", TimeValue (Seconds (interfHiStop)));
  onoffHi.Install (wifiInterfererAp.Get (0));

  OnOffHelper onoffLow2 ("ns3::UdpSocketFactory", remote);
  onoffLow2.SetConstantRate (DataRate ("20Mb/s"), 1420);
  onoffLow2.SetAttribute ("StartTime", TimeValue (Seconds (1080)));
  onoffLow2.SetAttribute ("StopTime", TimeValue (Seconds (1440)));
  onoffLow2.Install (wifiInterfererAp.Get (0));

  apps_sink.Start (Seconds (0.5));
  apps_sink.Stop (Seconds (simuTime));

  //------------------------------------------------------------
  //-- Setup stats and data collection
  //--------------------------------------------

  //Statistics counters
  NodeStatistics statisticsAp0 = NodeStatistics (wifiApDevices, wifiStaDevices);

  //Register packet receptions to calculate throughput
  //Config::Connect ("/NodeList/2/ApplicationList/*/$ns3::PacketSink/Rx",
    //               MakeCallback (&NodeStatistics::RxCallback, &statisticsAp0));

  //Register States
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/State/State",
                   MakeCallback (&NodeStatistics::StateCallback, &statisticsAp0));

  statisticsAp0.CheckStatistics (1);

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

  std::ofstream outfileTh0 (("throughput-" + outputFileName + "-0.plt").c_str ());
  Gnuplot gnuplot = Gnuplot (("throughput-" + outputFileName + "-0.eps").c_str (), "Throughput");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Throughput (Mb/s)");
  gnuplot.SetTitle ("Throughput (AP0 to STA) vs time");
  gnuplot.AddDataset (statisticsAp0.GetDatafile ());
  gnuplot.GenerateOutput (outfileTh0);

  std::ofstream outfileTx0 (("tx-" + outputFileName + "-0.plt").c_str ());
  gnuplot = Gnuplot (("tx-" + outputFileName + "-0.eps").c_str (), "Time in TX State");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Percent");
  gnuplot.SetTitle ("Percentage time AP0 in TX state vs time");
  gnuplot.AddDataset (statisticsAp0.GetTxDatafile ());
  gnuplot.GenerateOutput (outfileTx0);

  std::ofstream outfileRx0 (("rx-" + outputFileName + "-0.plt").c_str ());
  gnuplot = Gnuplot (("rx-" + outputFileName + "-0.eps").c_str (), "Time in RX State");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Percent");
  gnuplot.SetTitle ("Percentage time AP0 in RX state vs time");
  gnuplot.AddDataset (statisticsAp0.GetRxDatafile ());
  gnuplot.GenerateOutput (outfileRx0);

  std::ofstream outfileBusy0 (("busy-" + outputFileName + "-0.plt").c_str ());
  gnuplot = Gnuplot (("busy-" + outputFileName + "-0.eps").c_str (), "Time in Busy State");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Percent");
  gnuplot.SetTitle ("Percentage time AP0 in Busy state vs time");
  gnuplot.AddDataset (statisticsAp0.GetBusyDatafile ());
  gnuplot.GenerateOutput (outfileBusy0);

  std::ofstream outfileIdle0 (("idle-" + outputFileName + "-0.plt").c_str ());
  gnuplot = Gnuplot (("idle-" + outputFileName + "-0.eps").c_str (), "Time in Idle State");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Percent");
  gnuplot.SetTitle ("Percentage time AP0 in Idle state vs time");
  gnuplot.AddDataset (statisticsAp0.GetIdleDatafile ());
  gnuplot.GenerateOutput (outfileIdle0);

  Simulator::Destroy ();

  return 0;
}
