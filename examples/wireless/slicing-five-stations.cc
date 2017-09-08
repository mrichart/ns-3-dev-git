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

NS_LOG_COMPONENT_DEFINE ("SlicingFiveStations");

class NodeStatistics
{
public:
  NodeStatistics (NetDeviceContainer aps, NetDeviceContainer stas);

  void CheckStatistics (double time);

  void RxCallback (std::string path, Ptr<const Packet> packet, const Address &from);
  void PhyRxCallback (std::string path, Ptr<const Packet> packet);
  void SetPosition (Ptr<Node> node, Vector position);
  void AdvancePosition (Ptr<Node> node, int stepsSize, int stepsTime);
  Vector GetPosition (Ptr<Node> node);

  Gnuplot2dDataset GetDatafile ();

private:
  uint32_t m_bytesTotal;
  Gnuplot2dDataset m_output;
};

NodeStatistics::NodeStatistics (NetDeviceContainer aps, NetDeviceContainer stas)
{
  m_bytesTotal = 0;
}

void
NodeStatistics::RxCallback (std::string path, Ptr<const Packet> packet, const Address &from)
{
  m_bytesTotal += packet->GetSize ();
}

void
NodeStatistics::PhyRxCallback (std::string path, Ptr<const Packet> packet)
{
  m_bytesTotal += packet->GetSize ();
}

void
NodeStatistics::CheckStatistics (double time)
{

}

void
NodeStatistics::SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

Vector
NodeStatistics::GetPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

void
NodeStatistics::AdvancePosition (Ptr<Node> node, int stepsSize, int stepsTime)
{
  Vector pos = GetPosition (node);
  double mbs = ((m_bytesTotal * 8.0) / (1000000 * stepsTime));
  m_bytesTotal = 0;
  m_output.Add ((Simulator::Now ()).GetSeconds (), mbs);
  pos.x += stepsSize;
  SetPosition (node, pos);
  Simulator::Schedule (Seconds (stepsTime), &NodeStatistics::AdvancePosition, this, node, stepsSize, stepsTime);
}

Gnuplot2dDataset
NodeStatistics::GetDatafile ()
{
  return m_output;
}


void RateCallback (std::string path, uint64_t rate, Mac48Address dest)
{
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " " << dest << " Rate " <<  rate / 1000000.0);
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
  int ap1_x = 0;
  int ap1_y = 0;
  int sta1_x = 5;
  int sta1_y = 0;
  int sta2_x = -5;
  int sta2_y = 0;
  int sta3_x = 0;
  int sta3_y = 5;
  int sta4_x = 0;
  int sta4_y = -5;
  double sta5_x = 3.5;
  double sta5_y = 3.5;
  int steps = 100;
  int stepsSize = 1;
  int stepsTime = 1;

  CommandLine cmd;
  cmd.AddValue ("staManager", "PRC Manager of the STA", staManager);
  cmd.AddValue ("apManager", "PRC Manager of the AP", apManager);
  cmd.AddValue ("standard", "Wifi Phy Standard", standard);
  cmd.AddValue ("shortGuardInterval", "Enable Short Guard Interval in all stations", shortGuardInterval);
  cmd.AddValue ("channelWidth", "Channel width of all the stations", chWidth);
  cmd.AddValue ("rtsThreshold", "RTS threshold", rtsThreshold);
  cmd.AddValue ("BE_MaxAmpduSize", "BE Mac A-MPDU size", BE_MaxAmpduSize);
  cmd.AddValue ("outputFileName", "Output filename", outputFileName);
  cmd.AddValue ("steps", "How many different distances to try", steps);
  cmd.AddValue ("stepsTime", "Time on each step", stepsTime);
  cmd.AddValue ("stepsSize", "Distance between steps", stepsSize);
  cmd.AddValue ("AP1_x", "Position of AP1 in x coordinate", ap1_x);
  cmd.AddValue ("AP1_y", "Position of AP1 in y coordinate", ap1_y);
  cmd.AddValue ("STA1_x", "Position of STA1 in x coordinate", sta1_x);
  cmd.AddValue ("STA1_y", "Position of STA1 in y coordinate", sta1_y);
  cmd.AddValue ("STA2_x", "Position of STA2 in x coordinate", sta2_x);
  cmd.AddValue ("STA2_y", "Position of STA2 in y coordinate", sta2_y);
  cmd.AddValue ("STA3_x", "Position of STA3 in x coordinate", sta3_x);
  cmd.AddValue ("STA3_y", "Position of STA3 in y coordinate", sta3_y);
  cmd.AddValue ("STA4_x", "Position of STA4 in x coordinate", sta4_x);
  cmd.AddValue ("STA4_y", "Position of STA4 in y coordinate", sta4_y);
  cmd.AddValue ("STA5_x", "Position of STA5 in x coordinate", sta5_x);
  cmd.AddValue ("STA5_y", "Position of STA5 in y coordinate", sta5_y);
  cmd.Parse (argc, argv);

  int simuTime = steps * stepsTime;

  // Define the APs
  NodeContainer wifiApNodes;
  wifiApNodes.Create (1);

  //Define the STAs
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (5);


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
  //Initial position of AP and STA
  positionAlloc->Add (Vector (ap1_x, ap1_y, 0.0));
  positionAlloc->Add (Vector (sta1_x, sta1_y, 0.0));
  positionAlloc->Add (Vector (sta2_x, sta2_y, 0.0));
  positionAlloc->Add (Vector (sta3_x, sta3_y, 0.0));
  positionAlloc->Add (Vector (sta4_x, sta4_y, 0.0));
  positionAlloc->Add (Vector (sta5_x, sta5_y, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNodes.Get (0));
  mobility.Install (wifiStaNodes);

  //Statistics counter
  NodeStatistics atpCounter = NodeStatistics (wifiApDevices, wifiStaDevices);
  NodeStatistics atpCounter2 = NodeStatistics (wifiApDevices, wifiStaDevices);
  NodeStatistics atpCounter3 = NodeStatistics (wifiApDevices, wifiStaDevices);
  NodeStatistics atpCounter4 = NodeStatistics (wifiApDevices, wifiStaDevices);
  NodeStatistics atpCounter5 = NodeStatistics (wifiApDevices, wifiStaDevices);

  //Move the STA by stepsSize meters every stepsTime seconds
  Simulator::Schedule (Seconds (0.5 + stepsTime), &NodeStatistics::AdvancePosition, &atpCounter, wifiStaNodes.Get (0), stepsSize, stepsTime);
  Simulator::Schedule (Seconds (0.5 + stepsTime), &NodeStatistics::AdvancePosition, &atpCounter2, wifiStaNodes.Get (1), stepsSize, stepsTime);
  Simulator::Schedule (Seconds (0.5 + stepsTime), &NodeStatistics::AdvancePosition, &atpCounter3, wifiStaNodes.Get (2), stepsSize, stepsTime);
  Simulator::Schedule (Seconds (0.5 + stepsTime), &NodeStatistics::AdvancePosition, &atpCounter4, wifiStaNodes.Get (3), stepsSize, stepsTime);
  Simulator::Schedule (Seconds (0.5 + stepsTime), &NodeStatistics::AdvancePosition, &atpCounter5, wifiStaNodes.Get (4), stepsSize, stepsTime);

  //Configure the IP stack
  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = address.Assign (wifiDevices);
  Ipv4Address sinkAddress = i.GetAddress (0);
  Ipv4Address sinkAddress2 = i.GetAddress (1);
  Ipv4Address sinkAddress3 = i.GetAddress (2);
  Ipv4Address sinkAddress4 = i.GetAddress (3);
  Ipv4Address sinkAddress5 = i.GetAddress (4);
  uint16_t port = 9;

  //Configure the CBR generator
  PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
  ApplicationContainer apps_sink = sink.Install (wifiStaNodes.Get (0));
  PacketSinkHelper sink2 ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress2, port));
  apps_sink.Add(sink2.Install (wifiStaNodes.Get (1)));
  PacketSinkHelper sink3 ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress3, port));
  apps_sink.Add(sink3.Install (wifiStaNodes.Get (2)));
  PacketSinkHelper sink4 ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress4, port));
  apps_sink.Add(sink4.Install (wifiStaNodes.Get (3)));
  PacketSinkHelper sink5 ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress5, port));
  apps_sink.Add(sink5.Install (wifiStaNodes.Get (4)));

  OnOffHelper onoff ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
  onoff.SetConstantRate (DataRate ("20Mb/s"), 1420);
  onoff.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  ApplicationContainer apps_source = onoff.Install (wifiApNodes.Get (0));

  OnOffHelper onoff2 ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress2, port));
  onoff2.SetConstantRate (DataRate ("20Mb/s"), 1420);
  onoff2.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff2.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff2.Install (wifiApNodes.Get (0)));

  OnOffHelper onoff3 ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress3, port));
  onoff3.SetConstantRate (DataRate ("20Mb/s"), 1420);
  onoff3.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff3.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff3.Install (wifiApNodes.Get (0)));

  OnOffHelper onoff4 ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress4, port));
  onoff4.SetConstantRate (DataRate ("20Mb/s"), 1420);
  onoff4.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff4.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff4.Install (wifiApNodes.Get (0)));

  OnOffHelper onoff5 ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress5, port));
  onoff5.SetConstantRate (DataRate ("20Mb/s"), 1420);
  onoff5.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff5.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff5.Install (wifiApNodes.Get (0)));

  apps_sink.Start (Seconds (0.5));
  apps_sink.Stop (Seconds (simuTime));

  //------------------------------------------------------------
  //-- Setup stats and data collection
  //--------------------------------------------

  //Register packet receptions to calculate throughput
  Config::Connect ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
                   MakeCallback (&NodeStatistics::PhyRxCallback, &atpCounter));
  Config::Connect ("/NodeList/2/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
                   MakeCallback (&NodeStatistics::PhyRxCallback, &atpCounter2));
  Config::Connect ("/NodeList/3/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
                   MakeCallback (&NodeStatistics::PhyRxCallback, &atpCounter3));
  Config::Connect ("/NodeList/4/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
                   MakeCallback (&NodeStatistics::PhyRxCallback, &atpCounter4));
  Config::Connect ("/NodeList/5/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx",
                   MakeCallback (&NodeStatistics::PhyRxCallback, &atpCounter5));

  //Callbacks to print every change of rate
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + apManager + "/RateChange",
                   MakeCallback (RateCallback));

  Simulator::Stop (Seconds (simuTime));
  Simulator::Run ();

  std::ofstream outfile (("throughput-" + outputFileName + ".plt").c_str ());
  Gnuplot gnuplot = Gnuplot (("throughput-" + outputFileName + ".eps").c_str (), "Throughput");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Throughput (Mb/s)");
  gnuplot.SetTitle ("Throughput (AP to STA) vs time");
  gnuplot.AddDataset (atpCounter.GetDatafile ());
  gnuplot.GenerateOutput (outfile);

  std::ofstream outfile2 (("throughput-" + outputFileName + "2.plt").c_str ());
  gnuplot = Gnuplot (("throughput-" + outputFileName + "2.eps").c_str (), "Throughput");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Throughput (Mb/s)");
  gnuplot.SetTitle ("Throughput (AP to STA) vs time");
  gnuplot.AddDataset (atpCounter2.GetDatafile ());
  gnuplot.GenerateOutput (outfile2);

  std::ofstream outfile3 (("throughput-" + outputFileName + "3.plt").c_str ());
  gnuplot = Gnuplot (("throughput-" + outputFileName + "3.eps").c_str (), "Throughput");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Throughput (Mb/s)");
  gnuplot.SetTitle ("Throughput (AP to STA) vs time");
  gnuplot.AddDataset (atpCounter3.GetDatafile ());
  gnuplot.GenerateOutput (outfile3);

  std::ofstream outfile4 (("throughput-" + outputFileName + "4.plt").c_str ());
  gnuplot = Gnuplot (("throughput-" + outputFileName + "4.eps").c_str (), "Throughput");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Throughput (Mb/s)");
  gnuplot.SetTitle ("Throughput (AP to STA) vs time");
  gnuplot.AddDataset (atpCounter4.GetDatafile ());
  gnuplot.GenerateOutput (outfile4);

  std::ofstream outfile5 (("throughput-" + outputFileName + "5.plt").c_str ());
  gnuplot = Gnuplot (("throughput-" + outputFileName + "5.eps").c_str (), "Throughput");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Throughput (Mb/s)");
  gnuplot.SetTitle ("Throughput (AP to STA) vs time");
  gnuplot.AddDataset (atpCounter5.GetDatafile ());
  gnuplot.GenerateOutput (outfile5);

  Simulator::Destroy ();

  return 0;
}