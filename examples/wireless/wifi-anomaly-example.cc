/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2017 Universidad de la República - Uruguay
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
 * This example program is designed to illustrate the WiFi performance anomaly.
 * This anomaly appears in scenarios where devices with different data rates
 * share a common transmission medium.
 * Because of the behavior of the medium access mechanism of WiFi, all stations
 * will transmit at a similar rate, closer to the smallest rateof the scenario.
 *
 * A proposal to deal with this issue is presented in:
 * Høiland-Jørgensen, T., Kazior, M., Täht, D., Hurtig, P., & Brunstrom, A. (2017).
 * Ending the Anomaly: Achieving Low Latency and Airtime Fairness in WiFi.
 * arXiv preprint arXiv:1703.00064.
 *
 * Airtime Fairness consists on assigning equal airtime to all the stations connected
 * to an AP independently of the rate.
 *
 * This simulation consist of 4 nodes, one AP and 3 STAs.
 * The AP generates UDP CBR traffic to each STA.
 * The AP is at coordinate (0,0,0) and the STAs are located
 * such that 2 STAs have the highest possible rate and one STA
 * the lowest rate.
 *
 * The output consists of:
 * - A plot of average throughput vs. time.
 *
 * Example usage:
 * ./waf --run "wifi-anomaly-example --outputFileName=wifi-anomaly"
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

NS_LOG_COMPONENT_DEFINE ("WifiAnomalyExample");

class NodeStatistics
{
public:
  NodeStatistics (NetDeviceContainer stas);

  void RxCallback (std::string path, Ptr<const Packet> packet, const Address &from);
  void PhyRxCallback (std::string path,Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, uint32_t rate, WifiPreamble preamble, WifiTxVector txVector, struct mpduInfo aMpdu, struct signalNoiseDbm signalNoise);
  void PhyTxCallback (std::string path,Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, uint32_t rate, WifiPreamble preamble, WifiTxVector txVector, struct mpduInfo aMpdu);
  void RegisterStats (int interval);

  void RateCallback (std::string path, uint32_t rate, Mac48Address dest);
  void StateCallback (std::string path, Time init, Time duration, enum WifiPhy::State state);

  Gnuplot2dDataset GetThroughputDatafile ();
  Gnuplot2dDataset GetIdleDatafile ();
  Gnuplot2dDataset GetBusyDatafile ();
  Gnuplot2dDataset GetTxDatafile (Mac48Address sta);
  Gnuplot2dDataset GetRxDatafile (Mac48Address sta);

  double GetIdleTime ();
  double GetBusyTime ();
  double GetTxTime (Mac48Address sta);
  double GetRxTime (Mac48Address sta);

private:
  uint32_t m_bytesTotal;
  double busyTime;
  double idleTime;
  std::map<Mac48Address, double> txTime;
  std::map<Mac48Address, double> rxTime;
  double totalTxTime;
  double totalRxTime;
  Gnuplot2dDataset m_output_th;
  Gnuplot2dDataset m_output_idle;
  Gnuplot2dDataset m_output_busy;
  std::map<Mac48Address, Gnuplot2dDataset> m_output_rx;
  std::map<Mac48Address, Gnuplot2dDataset> m_output_tx;
  Mac48Address m_currentTx;
  Mac48Address m_currentRx;
  NetDeviceContainer m_stations;
};

NodeStatistics::NodeStatistics (NetDeviceContainer stas)
{
  m_stations = stas;
  for (uint32_t j = 0; j < stas.GetN (); j++)
    {
      Ptr<NetDevice> staDevice = stas.Get (j);
      Ptr<WifiNetDevice> wifiStaDevice = DynamicCast<WifiNetDevice> (staDevice);
      Mac48Address addr = wifiStaDevice->GetMac ()->GetAddress ();
      txTime[addr] = 0;
      rxTime[addr] = 0;
      m_output_rx[addr].SetTitle ("RX Time");
      m_output_tx[addr].SetTitle ("TX Time");
    }
  m_output_th.SetTitle ("Throughput Mbits/s");
  totalRxTime = 0;
  totalTxTime = 0;
  m_bytesTotal = 0;
  busyTime = 0;
  idleTime = 0;
  m_output_idle.SetTitle ("Idle Time");
  m_output_busy.SetTitle ("Busy Time");
}

void
NodeStatistics::RxCallback (std::string path, Ptr<const Packet> packet, const Address &from)
{
  m_bytesTotal += packet->GetSize ();
}

void
NodeStatistics::PhyTxCallback (std::string path, Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, uint32_t rate, WifiPreamble preamble, WifiTxVector txVector, struct mpduInfo aMpdu)
{
  WifiMacHeader macHead;
  if (aMpdu.type == NORMAL_MPDU)
    {
      packet->PeekHeader (macHead);
    }
  else if (aMpdu.type == MPDU_IN_AGGREGATE)
    {
      Ptr<Packet> newPacket = packet->Copy();
      AmpduSubframeHeader head;
      newPacket->RemoveHeader (head);
      newPacket->PeekHeader (macHead);
    }
  m_currentTx = macHead.GetAddr1 ();
}

void
NodeStatistics::PhyRxCallback (std::string path,Ptr<const Packet> packet, uint16_t channelFreqMhz, uint16_t channelNumber, uint32_t rate, WifiPreamble preamble, WifiTxVector txVector, struct mpduInfo aMpdu, struct signalNoiseDbm signalNoise)
{
  WifiMacHeader macHead;
  if (aMpdu.type == NORMAL_MPDU)
    {
      packet->PeekHeader (macHead);
    }
  else if (aMpdu.type == MPDU_IN_AGGREGATE)
    {
      Ptr<Packet> newPacket = packet->Copy();
      AmpduSubframeHeader head;
      newPacket->RemoveHeader (head);
      newPacket->PeekHeader (macHead);
    }
  m_currentRx = macHead.GetAddr2 ();
}

void
NodeStatistics::StateCallback (std::string path, Time init, Time duration, enum WifiPhy::State state)
{
  if (state == WifiPhy::CCA_BUSY)
    {
      busyTime += duration.GetSeconds ();
    }
  else if (state == WifiPhy::IDLE)
    {
      idleTime += duration.GetSeconds ();
    }
  else if (state == WifiPhy::TX)
    {
      txTime[m_currentTx] += duration.GetSeconds ();
      totalTxTime += duration.GetSeconds ();
    }
  else if (state == WifiPhy::RX)
    {
      rxTime[m_currentRx] += duration.GetSeconds ();
      totalRxTime += duration.GetSeconds ();
    }
}

void
NodeStatistics::RegisterStats (int interval)
{
  double th = ((m_bytesTotal * 8.0) / (1000000 * interval));  //throughput in Mbps
  m_output_th.Add ((Simulator::Now ()).GetSeconds (), th);
  m_bytesTotal = 0;
  double totalTime = totalTxTime + totalRxTime + busyTime + idleTime;
  for (uint32_t j = 0; j < m_stations.GetN (); j++)
    {
      Ptr<NetDevice> staDevice = m_stations.Get (j);
      Ptr<WifiNetDevice> wifiStaDevice = DynamicCast<WifiNetDevice> (staDevice);
      Mac48Address addr = wifiStaDevice->GetMac ()->GetAddress ();

      m_output_tx[addr].Add ((Simulator::Now ()).GetSeconds (), txTime[addr] / totalTime);
      m_output_rx[addr].Add ((Simulator::Now ()).GetSeconds (), rxTime[addr] / totalTime);
      txTime[addr] = 0;
      rxTime[addr] = 0;
    }
  m_output_idle.Add ((Simulator::Now ()).GetSeconds (), idleTime / totalTime);
  m_output_busy.Add ((Simulator::Now ()).GetSeconds (), busyTime / totalTime);
  totalTxTime = 0;
  totalRxTime = 0;
  busyTime = 0;
  idleTime = 0;
  Simulator::Schedule (Seconds (interval), &NodeStatistics::RegisterStats, this, interval);
}

Gnuplot2dDataset
NodeStatistics::GetThroughputDatafile()
{
  return m_output_th;
}

Gnuplot2dDataset
NodeStatistics::GetBusyDatafile()
{
  return m_output_busy;
}

Gnuplot2dDataset
NodeStatistics::GetIdleDatafile()
{
  return m_output_idle;
}

Gnuplot2dDataset
NodeStatistics::GetTxDatafile(Mac48Address sta)
{
  return m_output_tx[sta];
}

Gnuplot2dDataset
NodeStatistics::GetRxDatafile(Mac48Address sta)
{
  return m_output_rx[sta];
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
  uint8_t nStreams = 1;
  uint32_t chWidth = 20;
  int simuTime = 30;
  int interval = 1;

  CommandLine cmd;
  cmd.AddValue ("staManager", "PRC Manager of the STA", staManager);
  cmd.AddValue ("apManager", "PRC Manager of the AP", apManager);
  cmd.AddValue ("standard", "Wifi Phy Standard", standard);
  cmd.AddValue ("shortGuardInterval", "Enable Short Guard Interval in all stations", shortGuardInterval);
  cmd.AddValue ("nStreams", "Set the number of spatial streams (valid only for 802.11n or ac)", nStreams);
  cmd.AddValue ("channelWidth", "Channel width of all the stations", chWidth);
  cmd.AddValue ("rtsThreshold", "RTS threshold", rtsThreshold);
  cmd.AddValue ("BE_MaxAmpduSize", "BE Mac A-MPDU size", BE_MaxAmpduSize);
  cmd.AddValue ("outputFileName", "Output filename", outputFileName);
  cmd.AddValue ("interval", "Interval in secong to register stats", interval);
  cmd.AddValue ("simuTime", "Simulation time in seconds", simuTime);
  cmd.Parse (argc, argv);

  // Define the APs
  NodeContainer wifiApNodes;
  wifiApNodes.Create (1);

  //Define the STAs
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (3);


  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  // Set guard interval
  wifiPhy.Set ("ShortGuardEnabled", BooleanValue (shortGuardInterval));
  // Set MIMO capabilities
  wifiPhy.Set ("TxAntennas", UintegerValue (nStreams));
  wifiPhy.Set ("RxAntennas", UintegerValue (nStreams));

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

  positionAlloc->Add (Vector (0.0, 400.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (wifiStaNodes.Get (0));

  positionAlloc->Add (Vector (3.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (wifiStaNodes.Get(1));
  positionAlloc->Add (Vector (-3.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (wifiStaNodes.Get(2));

  //Configure the IP stack
  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = address.Assign (wifiDevices);
  uint16_t port = 9;

  //Configure the CBR generator
  PacketSinkHelper sink0_1 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (0), port));
  ApplicationContainer apps_sink = sink0_1.Install (wifiStaNodes.Get (0));

  PacketSinkHelper sink1_1 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (1), port));
  apps_sink.Add(sink1_1.Install (wifiStaNodes.Get (1)));

  PacketSinkHelper sink2 ("ns3::UdpSocketFactory", InetSocketAddress (i.GetAddress (2), port));
  apps_sink.Add(sink2.Install (wifiStaNodes.Get (2)));

  InetSocketAddress remote0 = InetSocketAddress (i.GetAddress (0), port);
  remote0.SetTos(0); //Set the TOS field to select the TID in the wifi queues. TID = TOS >> 5 (done in qos-utils)
  OnOffHelper onoff0 ("ns3::UdpSocketFactory", remote0);
  onoff0.SetConstantRate (DataRate ("10Mb/s"), 1420);
  onoff0.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff0.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  ApplicationContainer apps_source = onoff0.Install (wifiApNodes.Get (0));

  InetSocketAddress remote1 = InetSocketAddress (i.GetAddress (1), port);
  remote1.SetTos(0);
  OnOffHelper onoff1 ("ns3::UdpSocketFactory",  remote1);
  onoff1.SetConstantRate (DataRate ("50Mb/s"), 1420);
  onoff1.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff1.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff1.Install (wifiApNodes.Get (0)));

  InetSocketAddress remote2 = InetSocketAddress (i.GetAddress (2), port);
  remote2.SetTos(0);
  OnOffHelper onoff2 ("ns3::UdpSocketFactory",  remote2);
  onoff2.SetConstantRate (DataRate ("50Mb/s"), 1420);
  onoff2.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff2.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  apps_source.Add(onoff2.Install (wifiApNodes.Get (0)));

  apps_sink.Start (Seconds (0.5));
  apps_sink.Stop (Seconds (simuTime));

  //------------------------------------------------------------
  //-- Setup stats and data collection
  //--------------------------------------------

  //Statistics counters
  NodeStatistics statsAp = NodeStatistics (wifiStaDevices);
  NodeStatistics statsAp2 = NodeStatistics (wifiStaDevices);
  NodeStatistics statsAp3 = NodeStatistics (wifiStaDevices);

  //Register packet receptions to calculate throughput
  Config::Connect ("/NodeList/1/ApplicationList/*/$ns3::PacketSink/Rx",
                   MakeCallback (&NodeStatistics::RxCallback, &statsAp));
  Config::Connect ("/NodeList/2/ApplicationList/*/$ns3::PacketSink/Rx",
                   MakeCallback (&NodeStatistics::RxCallback, &statsAp2));
  Config::Connect ("/NodeList/3/ApplicationList/*/$ns3::PacketSink/Rx",
                   MakeCallback (&NodeStatistics::RxCallback, &statsAp3));

  //Register States for calculating tx and rx airtime
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferTx",
                   MakeCallback (&NodeStatistics::PhyTxCallback, &statsAp));
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
                   MakeCallback (&NodeStatistics::PhyRxCallback, &statsAp));
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/State/State",
                   MakeCallback (&NodeStatistics::StateCallback, &statsAp));

  statsAp.RegisterStats (interval);
  statsAp2.RegisterStats (interval);
  statsAp3.RegisterStats (interval);

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
      NS_LOG_UNCOND ("  Throughput: " << i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds () - i->second.timeFirstTxPacket.GetSeconds ()) / 1000 / 1000  << " Mbps\n");
      NS_LOG_UNCOND ("  Mean delay:   " << i->second.delaySum.GetSeconds () * 1000 / i->second.rxPackets << " ms\n");
      NS_LOG_UNCOND ("  Mean jitter:   " << i->second.jitterSum.GetSeconds () * 1000 / (i->second.rxPackets - 1) << " ms\n");
    }

  Gnuplot gnuplot;

  std::ofstream outfileTh (("throughput-" + outputFileName + "-sta0.plt").c_str ());
  gnuplot = Gnuplot (("throughput-" + outputFileName + "-sta0.eps").c_str (), "Throughput");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Throughput (Mb/s)");
  gnuplot.SetTitle ("Throughput (AP to STA) vs time");
  gnuplot.AddDataset (statsAp.GetThroughputDatafile());
  gnuplot.GenerateOutput (outfileTh);

  std::ofstream outfileTh2 (("throughput-" + outputFileName + "-sta1.plt").c_str ());
  gnuplot = Gnuplot (("throughput-" + outputFileName + "-sta1.eps").c_str (), "Throughput");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Throughput (Mb/s)");
  gnuplot.SetTitle ("Throughput (AP to STA) vs time");
  gnuplot.AddDataset (statsAp2.GetThroughputDatafile());
  gnuplot.GenerateOutput (outfileTh2);

  std::ofstream outfileTh3 (("throughput-" + outputFileName + "-sta2.plt").c_str ());
  gnuplot = Gnuplot (("throughput-" + outputFileName + "-sta2.eps").c_str (), "Throughput");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Throughput (Mb/s)");
  gnuplot.SetTitle ("Throughput (AP to STA) vs time");
  gnuplot.AddDataset (statsAp3.GetThroughputDatafile());
  gnuplot.GenerateOutput (outfileTh3);

  for (uint32_t j = 0; j < wifiStaDevices.GetN (); j++)
    {
      Ptr<NetDevice> staDevice = wifiStaDevices.Get (j);
      Ptr<WifiNetDevice> wifiStaDevice = DynamicCast<WifiNetDevice> (staDevice);
      Mac48Address addr = wifiStaDevice->GetMac ()->GetAddress ();
      char str[1];
      sprintf(str, "%d", j);

      std::ofstream outfileTx0 (("tx-" + outputFileName + "-sta" + str + ".plt").c_str ());
      gnuplot = Gnuplot (("tx-" + outputFileName + "-sta" + str + ".eps").c_str (), "Time in TX State");
      gnuplot.SetTerminal ("post eps color enhanced");
      gnuplot.SetLegend ("Time (seconds)", "Percent");
      gnuplot.SetTitle ("Percentage time AP in TX state vs time");
      gnuplot.AddDataset (statsAp.GetTxDatafile (addr));
      gnuplot.GenerateOutput (outfileTx0);

      std::ofstream outfileRx0 (("rx-" + outputFileName + "-sta" + str + ".plt").c_str ());
      gnuplot = Gnuplot (("rx-" + outputFileName + "-sta" + str + ".eps").c_str (), "Time in RX State");
      gnuplot.SetTerminal ("post eps color enhanced");
      gnuplot.SetLegend ("Time (seconds)", "Percent");
      gnuplot.SetTitle ("Percentage time AP in RX state vs time");
      gnuplot.AddDataset (statsAp.GetRxDatafile (addr));
      gnuplot.GenerateOutput (outfileRx0);
    }

  std::ofstream outfileBusy0 (("busy-" + outputFileName + ".plt").c_str ());
  gnuplot = Gnuplot (("busy-" + outputFileName + ".eps").c_str (), "Time in Busy State");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Percent");
  gnuplot.SetTitle ("Percentage time AP in Busy state vs time");
  gnuplot.AddDataset (statsAp.GetBusyDatafile ());
  gnuplot.GenerateOutput (outfileBusy0);

  std::ofstream outfileIdle0 (("idle-" + outputFileName + ".plt").c_str ());
  gnuplot = Gnuplot (("idle-" + outputFileName + ".eps").c_str (), "Time in Idle State");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Percent");
  gnuplot.SetTitle ("Percentage time AP in Idle state vs time");
  gnuplot.AddDataset (statsAp.GetIdleDatafile ());
  gnuplot.GenerateOutput (outfileIdle0);

  Simulator::Destroy ();

  return 0;
}
