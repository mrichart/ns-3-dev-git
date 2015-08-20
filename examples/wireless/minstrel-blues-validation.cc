/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 Universidad de la República - Uruguay
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
 * Author: Matias Richart <mrichart@fing.edu.uy>
 */

/**
 * This example program is designed to validate power adaptation
 * in ns3.
 *
 * The output of this is typically two plot files, named throughput-validation.plt
 * and power-validation.plt If Gnuplot program is available, one can use it to convert
 * the plt file into an eps file, by running:
 * \code{.sh}
 *   gnuplot throughput-validation.plt
 * \endcode
 * Also, to enable logging of rate and power changes and SNR to the terminal, set this
 * environment variable:
 * \code{.sh}
 *   export NS_LOG=PowerAdaptationValidation=level_info
 * \endcode
 *
 * This simulation consist of 2 nodes, one AP and one STA.
 * The AP generates UDP traffic with a CBR of 54 Mbps to the STA.
 * The nodes can use any power and rate control mechanism.
 * By default, the AP is at coordinate (0,0,0) and the STA starts at 
 * coordinate (5,0,0) (meters).
 *
 * The output consists of:
 * - A plot of average throughput vs. time.
 * - A plot of average transmit power vs. time.
 * - (if logging is enabled) the changes of power and rate to standard output.
 * - (if logging is enabled) the rate and SNR (signal to noise ratio) at which the packets where received by the STA.
 *
 * The Average Transmit Power is defined as an average of the power
 * consumed per measurement interval, expressed in milliwatts.  The
 * power level for each frame transmission is reported by the simulator, 
 * and the energy consumed is obtained by multiplying the power by the
 * frame duration.  At every 'time' (defaulting to 1 second), the
 * total energy for the collection period is divided by the step time 
 * and converted from dbm to milliwatt units, and this average is 
 * plotted against time.
 *
 * When neither power control mechanism is selected as the rate control, the
 * generation of the plot of average transmit power vs distance is suppressed
 * since the other Wifi rate controls do not support the necessary callbacks
 * for computing the average power.
 *
 * To display all the possible arguments and their defaults:
 * \code{.sh}
 *   ./waf --run "power-adaptation-validation --help"
 * \endcode
 * 
 * Example usage (fixing power (1dBm) and rate (6Mbps)):
 * \code{.sh}
 *   ./waf --run "power-adaptation-validation --mode=OfdmRate6Mbps --powerLevels=1 --maxPower=1 --minPower=1 --outputFileName=6Mbps1Dbm"
 * \endcode
 *
 * To enable the log of rate and power changes:
 * \code{.sh}
 *   export NS_LOG=PowerAdaptationValidation=level_info
 * \endcode
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

NS_LOG_COMPONENT_DEFINE ("MinstrelBluesValidation");

// packet size generated at the AP
static const uint32_t packetSize = 1420;

class NodeStatistics
{
public:
  NodeStatistics (NetDeviceContainer aps, NetDeviceContainer stas);

  void CheckStatistics (double time);

  void PhyCallback (std::string path, Ptr<const Packet> packet);
  void RxCallback (std::string path, Ptr<const Packet> packet, const Address &from);
  void PowerCallback (std::string path, uint8_t power, Mac48Address dest);
  void BluesPowerCallback (std::string path, std::string type, uint8_t power, Mac48Address dest);
  void RateCallback (std::string path, uint32_t rate, Mac48Address dest);
  void BluesRateCallback (std::string path, std::string type, uint32_t rate, Mac48Address dest);
  void SetPosition (Ptr<Node> node, Vector position);
  void AdvancePosition (Ptr<Node> node, int stepsSize, int stepsTime);
  Vector GetPosition (Ptr<Node> node);

  Gnuplot2dDataset GetDatafile ();
  Gnuplot2dDataset GetPowerDatafile ();

private:
  typedef std::vector<std::pair<Time,WifiMode> > TxTime;
  void SetupPhy (Ptr<WifiPhy> phy);
  Time GetCalcTxTime (WifiMode mode);

  std::map<Mac48Address, uint32_t> actualPower;
  std::map<Mac48Address, WifiMode> actualMode;
  uint32_t m_bytesTotal;
  double totalEnergy;
  double totalTime;
  Ptr<WifiPhy> phyNode0;
  TxTime timeTable;
  Gnuplot2dDataset m_output;
  Gnuplot2dDataset m_output_power;
};

NodeStatistics::NodeStatistics (NetDeviceContainer aps, NetDeviceContainer stas)
{
  Ptr<NetDevice> device = aps.Get (0);
  Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice> (device);
  Ptr<WifiPhy> phy = wifiDevice->GetPhy ();
  phyNode0 = phy;
  SetupPhy (phy);
  for (uint32_t j = 0; j < stas.GetN (); j++)
    {
      Ptr<NetDevice> staDevice = stas.Get (j);
      Ptr<WifiNetDevice> wifiStaDevice = DynamicCast<WifiNetDevice> (staDevice);
      Mac48Address addr = wifiStaDevice->GetMac ()->GetAddress ();
      actualPower[addr] = 17;
      actualMode[addr] = phy->GetMode (0);
    }
  actualMode[Mac48Address ("ff:ff:ff:ff:ff:ff")] = phy->GetMode (0);
  totalEnergy = 0;
  totalTime = 0;
  m_bytesTotal = 0;
  m_output.SetTitle ("Throughput Mbits/s");
  m_output_power.SetTitle ("Average Transmit Power");
}

void
NodeStatistics::SetupPhy (Ptr<WifiPhy> phy)
{
  uint32_t nModes = phy->GetNModes ();
  for (uint32_t i = 0; i < nModes; i++)
    {
      WifiMode mode = phy->GetMode (i);
      WifiTxVector txVector;
      txVector.SetMode (mode);
      timeTable.push_back (std::make_pair (phy->CalculateTxDuration (packetSize, txVector, WIFI_PREAMBLE_LONG, phy->GetFrequency (), 0, 0), mode));
    }
}

Time
NodeStatistics::GetCalcTxTime (WifiMode mode)
{
  for (TxTime::const_iterator i = timeTable.begin (); i != timeTable.end (); i++)
    {
      if (mode == i->second)
        {
          return i->first;
        }
    }
  NS_ASSERT (false);
  return Seconds (0);
}

void
NodeStatistics::PhyCallback (std::string path, Ptr<const Packet> packet)
{
  WifiMacHeader head;
  packet->PeekHeader (head);
  Mac48Address dest = head.GetAddr1 ();

  if (head.GetType() == WIFI_MAC_DATA)
    {
      totalEnergy += pow (10, actualPower[dest] / 10) * GetCalcTxTime (actualMode[dest]).GetSeconds ();
      totalTime += GetCalcTxTime (actualMode[dest]).GetSeconds ();
    }
}

void
NodeStatistics::PowerCallback (std::string path, uint8_t power, Mac48Address dest)
{
  double   txPowerBaseDbm = phyNode0->GetTxPowerStart ();
  double   txPowerEndDbm = phyNode0->GetTxPowerEnd ();
  uint32_t nTxPower = phyNode0->GetNTxPower ();
  double dbm;
  if (nTxPower > 1)
    {
      dbm = txPowerBaseDbm + power * (txPowerEndDbm - txPowerBaseDbm) / (nTxPower - 1);
    }
  else
    {
      NS_ASSERT_MSG (txPowerBaseDbm == txPowerEndDbm, "cannot have TxPowerEnd != TxPowerStart with TxPowerLevels == 1");
      dbm = txPowerBaseDbm;
    }
  actualPower[dest] = dbm;
}

void
NodeStatistics::BluesPowerCallback (std::string path, std::string type, uint8_t power, Mac48Address dest)
{
  actualPower[dest] = power;
}

void
NodeStatistics::RateCallback (std::string path, uint32_t rate, Mac48Address dest)
{
  actualMode[dest] = phyNode0->GetMode (rate);
}

void
NodeStatistics::BluesRateCallback (std::string path, std::string type, uint32_t rate, Mac48Address dest)
{
  actualMode[dest] = phyNode0->GetMode (rate);
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
  double atp = totalEnergy / time;
  totalEnergy = 0;
  totalTime = 0;
  m_output_power.Add ((Simulator::Now ()).GetSeconds (), atp);
  m_output.Add ((Simulator::Now ()).GetSeconds (), mbs);

  Simulator::Schedule (Seconds (time), &NodeStatistics::CheckStatistics, this, time);
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
  double atp = totalEnergy / stepsTime;
  totalEnergy = 0;
  totalTime = 0;
  m_output_power.Add (pos.x, atp);
  m_output.Add (pos.x, mbs);
  pos.x += stepsSize;
  SetPosition (node, pos);
  NS_LOG_INFO ("At time " << Simulator::Now ().GetSeconds () << " sec; setting new position to " << pos);
  Simulator::Schedule (Seconds (stepsTime), &NodeStatistics::AdvancePosition, this, node, stepsSize, stepsTime);
}

Gnuplot2dDataset
NodeStatistics::GetDatafile ()
{
  return m_output;
}

Gnuplot2dDataset
NodeStatistics::GetPowerDatafile ()
{
  return m_output_power;
}

void PowerCallback (std::string path, uint8_t power, Mac48Address dest)
{
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " " << dest << " Power " << (int)power);
}

void BluesPowerCallback (std::string path, std::string type, uint8_t power, Mac48Address dest)
{
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " station: " << dest << ", frame sent with " << type << " power: " << (int)power);
}

void RateCallback (std::string path, uint32_t rate, Mac48Address dest)
{
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " " << dest << " Rate " <<  rate);
}

void BluesRateCallback (std::string path, std::string type, uint32_t rate, Mac48Address dest)
{
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " station: " << dest << ", frame sent with " << type << " rate: " <<  rate);
}

void
PhyRxOkCallback (std::string path, Ptr<const Packet> packet, double snr, WifiMode mode, enum WifiPreamble preamble)
{
  WifiMacHeader head;
  packet->PeekHeader (head);
  Mac48Address dest = head.GetAddr1 ();

  if (head.GetType() == WIFI_MAC_DATA)
    {
      NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " " << dest << " Mode " << mode << " SNR " << 10.0*std::log10(snr) );
    }
}

int main (int argc, char *argv[])
{
  double maxPower = 22;
  double minPower = 1;
  uint32_t powerLevels = 22;
  double edThreshold = -96.0;

  uint32_t rtsThreshold = 2346;
  std::string manager = "ns3::ConstantRateWifiManager";
  std::string mode = "OfdmRate48Mbps";
  bool fixedRate = false;
  std::string outputFileName = "validation";
  int ap1_x = 0;
  int ap1_y = 0;
  int sta1_x = 10;
  int sta1_y = 0;

  bool printMBTable = false;

  uint32_t simuTime = 100;

  CommandLine cmd;
  cmd.AddValue ("manager", "PRC Manager", manager);
  cmd.AddValue ("mode", "Data mode", mode);
  cmd.AddValue ("fixedRate", "Use a fixed rate with Minstrel Blues", fixedRate);
  cmd.AddValue ("edThreshold", "The energy detection threshold of the PHY layer.", edThreshold);
  cmd.AddValue ("rtsThreshold", "RTS threshold", rtsThreshold);
  cmd.AddValue ("outputFileName", "Output filename", outputFileName);
  cmd.AddValue ("maxPower", "Maximum available transmission level (dbm).", maxPower);
  cmd.AddValue ("minPower", "Minimum available transmission level (dbm).", minPower);
  cmd.AddValue ("powerLevels", "Number of transmission power levels available between "
                "TxPowerStart and TxPowerEnd included.", powerLevels);
  cmd.AddValue ("AP1_x", "Position of AP1 in x coordinate", ap1_x);
  cmd.AddValue ("AP1_y", "Position of AP1 in y coordinate", ap1_y);
  cmd.AddValue ("STA1_x", "Position of STA1 in x coordinate", sta1_x);
  cmd.AddValue ("STA1_y", "Position of STA1 in y coordinate", sta1_y);
  cmd.AddValue ("printMBTable", "Print the statistics table of Minstrel-Blues", printMBTable);
  cmd.AddValue ("simuTime", "Simulation time (seconds).", simuTime);
  cmd.Parse (argc, argv);

  // Define the APs
  NodeContainer wifiApNodes;
  wifiApNodes.Create (1);

  //Define the STAs
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (1);

  WifiHelper wifi = WifiHelper::Default ();
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
  wifi.SetPowerLimitation(WIFI_PHY_POWER_LIMITATION_WISTRON_DCMA_82_80211a);
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();

  wifiPhy.SetChannel (wifiChannel.Create ());

  NetDeviceContainer wifiApDevices;
  NetDeviceContainer wifiStaDevices;
  NetDeviceContainer wifiDevices;

  //Configure the AP and STA node
  if (manager.compare ("ns3::ConstantRateWifiManager") == 0)
    {
      wifi.SetRemoteStationManager (manager, "DataMode", StringValue (mode), "DefaultTxPowerLevel", UintegerValue (powerLevels-1), "RtsCtsThreshold", UintegerValue (rtsThreshold));
    }
  else if (manager.compare ("ns3::MinstrelBluesWifiManager") == 0)
    {
      wifi.SetRemoteStationManager (manager, "FixedRate", BooleanValue (fixedRate), "DataMode", StringValue (mode), "PrintTable", BooleanValue (printMBTable), "DefaultTxPowerLevel", UintegerValue (powerLevels-1), "RtsCtsThreshold", UintegerValue (rtsThreshold));
    }
  else
    {
      wifi.SetRemoteStationManager (manager, "DefaultTxPowerLevel", UintegerValue (powerLevels-1), "RtsCtsThreshold", UintegerValue (rtsThreshold));
    }
  wifiPhy.Set ("TxPowerStart", DoubleValue (minPower));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (maxPower));
  wifiPhy.Set ("TxPowerLevels", UintegerValue (powerLevels));

  //Configure the AP node
  Ssid ssid = Ssid ("AP");
  wifiMac.SetType ("ns3::ApWifiMac",
                   "Ssid", SsidValue (ssid));
  wifiApDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiApNodes.Get (0)));

  wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (edThreshold));

  //Configure the STA node
  ssid = Ssid ("AP");
  wifiMac.SetType ("ns3::StaWifiMac",
                   "Ssid", SsidValue (ssid),
                   "ActiveProbing", BooleanValue (false));
  wifiStaDevices.Add (wifi.Install (wifiPhy, wifiMac, wifiStaNodes.Get (0)));



  wifiDevices.Add (wifiStaDevices);
  wifiDevices.Add (wifiApDevices);

  // Configure the mobility.
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  //Initial position of AP and STA
  positionAlloc->Add (Vector (ap1_x, ap1_y, 0.0));
  NS_LOG_INFO ("Setting initial AP position to " << Vector (ap1_x, ap1_y, 0.0));
  positionAlloc->Add (Vector (sta1_x, sta1_y, 0.0));
  NS_LOG_INFO ("Setting initial STA position to " << Vector (sta1_x, sta1_y, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNodes.Get (0));
  mobility.Install (wifiStaNodes.Get (0));
 
  //Statistics counter
  NodeStatistics statistics = NodeStatistics (wifiApDevices, wifiStaDevices);

  //Configure the IP stack
  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = address.Assign (wifiDevices);
  Ipv4Address sinkAddress = i.GetAddress (0);
  uint16_t port = 9;

  //Configure the CBR generator
  PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
  ApplicationContainer apps_sink = sink.Install (wifiStaNodes.Get (0));

  OnOffHelper onoff ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
  onoff.SetConstantRate (DataRate ("54Mb/s"), packetSize);
  onoff.SetAttribute ("StartTime", TimeValue (Seconds (0.5)));
  onoff.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  ApplicationContainer apps_source = onoff.Install (wifiApNodes.Get (0));

  apps_sink.Start (Seconds (0.5));
  apps_sink.Stop (Seconds (simuTime));

  //------------------------------------------------------------
  //-- Setup stats and data collection
  //--------------------------------------------

  //Register packet receptions to calculate throughput
  Config::Connect ("/NodeList/1/ApplicationList/*/$ns3::PacketSink/Rx",
                   MakeCallback (&NodeStatistics::RxCallback, &statistics));

  //Register power and rate changes to calculate the Average Transmit Power
  if (manager.compare ("ns3::MinstrelBluesWifiManager") == 0)
    {
      Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
                         MakeCallback (&NodeStatistics::BluesPowerCallback, &statistics));
      Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChange",
                         MakeCallback (&NodeStatistics::BluesRateCallback, &statistics));
    }
  else
    {
      Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
		       MakeCallback (&NodeStatistics::PowerCallback, &statistics));
      Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChange",
		       MakeCallback (&NodeStatistics::RateCallback, &statistics));
    }

  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
                   MakeCallback (&NodeStatistics::PhyCallback, &statistics));

  statistics.CheckStatistics (1);

  //Callbacks to print every change of power and rate
  if (manager.compare ("ns3::MinstrelBluesWifiManager") == 0)
      {
        Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
                           MakeCallback (BluesPowerCallback));
        Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChange",
                         MakeCallback (BluesRateCallback));
      }
  else
    {
      Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
		       MakeCallback (PowerCallback));
      Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChange",
		       MakeCallback (RateCallback));
    }

  //Callback to print info of packets received
  Config::Connect ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Phy/State/RxOk",
                   MakeCallback (PhyRxOkCallback));

  //Start with fixed max power and at 120 sec, enable blues
  Ptr<NetDevice> device = wifiApDevices.Get (0);
  Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice> (device);
  Ptr<WifiPhy> phy = wifiDevice->GetPhy ();
  Ptr<YansWifiPhy> yansPhy = DynamicCast<YansWifiPhy> (phy);
  Simulator::Schedule (Seconds (120), &YansWifiPhy::SetTxPowerStart, yansPhy, 1);
  Simulator::Schedule (Seconds (120), &YansWifiPhy::SetTxPowerEnd, yansPhy, 22);
  Ptr<WifiRemoteStationManager> apManager = wifiDevice->GetRemoteStationManager();
  Simulator::Schedule (Seconds (120), &WifiRemoteStationManager::SetupPhy, apManager, yansPhy);

  //At 240s increase sensitivity
  Ptr<NetDevice> staDevice = wifiStaDevices.Get (0);
  Ptr<WifiNetDevice> wifiStaDevice = DynamicCast<WifiNetDevice> (staDevice);
  Ptr<WifiPhy> staPhy = wifiStaDevice->GetPhy ();
  Ptr<YansWifiPhy> yansStaPhy = DynamicCast<YansWifiPhy> (staPhy);
  Simulator::Schedule (Seconds (240), &YansWifiPhy::SetEdThreshold, yansStaPhy, -96.0);

  //At 360s decrease sensitivity
  Simulator::Schedule (Seconds (360), &YansWifiPhy::SetEdThreshold, yansStaPhy, -65.0);

  Simulator::Stop (Seconds (simuTime));
  Simulator::Run ();

  std::ofstream outfile (("throughput-" + outputFileName + ".plt").c_str ());
  Gnuplot gnuplot = Gnuplot (("throughput-" + outputFileName + ".eps").c_str (), "Throughput");
  gnuplot.SetTerminal ("post eps color enhanced");
  gnuplot.SetLegend ("Time (seconds)", "Throughput (Mb/s)");
  gnuplot.SetTitle ("Throughput (AP to STA) vs time");
  gnuplot.AddDataset (statistics.GetDatafile ());
  gnuplot.GenerateOutput (outfile);

  if (manager.compare ("ns3::ParfWifiManager") == 0 ||
      manager.compare ("ns3::AparfWifiManager") == 0)
    {
      std::ofstream outfile2 (("power-" + outputFileName + ".plt").c_str ());
      gnuplot = Gnuplot (("power-" + outputFileName + ".eps").c_str (), "Average Transmit Power");
      gnuplot.SetTerminal ("post eps color enhanced");
      gnuplot.SetLegend ("Time (seconds)", "Power (mW)");
      gnuplot.SetTitle ("Average transmit power (AP to STA) vs time");
      gnuplot.AddDataset (statistics.GetPowerDatafile ());
      gnuplot.GenerateOutput (outfile2);
    }

  Simulator::Destroy ();

  return 0;
}
