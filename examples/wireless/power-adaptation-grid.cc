/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
* This simulation consist of a square scenario of scenarioSize meters of side.
* APs are placed on a grid topology inside the scenario with a distance between APs of gridDist.
* So scenarioSize and gridDist determine the number of APs (density) in the simulation.
* For each AP there is a corresponding STA acting as a client.
* Each STA is deployed randomly inside a circle of center the AP and radius gridDist/2.
* The APs generate UDP traffic with a CBR of 54 Mbps to the STAs.
* The APs use any power and rate control mechanism and the STAs use only rate control.
*
* The objective is to test power and rate control in the links with interference from the other links.
*
*   STA0
*      \  <-gridDist->
*	AP0----------AP1
*	 |            | \
*	 |   STA3     | STA1
*	 |  /         |  STA4
*	 | /          | /
*	AP3----------AP4
*
* The output consists of:
* - If enabled, the changes of power and rate to standard output.
* - If enabled, the average throughput, delay, jitter and tx opportunity for the total simulation time.
*
* Example usage:
* ./waf --run "power-adaptation-grid --manager=ns3::AparfWifiManager"
*
* Another example (changing APs position):
* ./waf --run "power-adaptation-grid --manager=ns3::AparfWifiManager --scenarioSize=500 --gridDist=100"
*
* * Another example (controlling CBR rate, PathLoss exponent, APs positions, simulation time and random variable generator):
* ./waf --run "power-adaptation-grid --manager=ns3::AparfWifiManager --longOutput=false --gridDist=50 --scenarioSize=200
* --simuTime=100 --rate=20Mbps --exponent=4 --RngRun=2 >> output_aparf.log 2>&1
*
* To enable the log of rate and power changes:
* export NS_LOG=PowerAdaptationGrid=level_info
*/

#include <sstream>
#include <fstream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/csma-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"

#include "ns3/stats-module.h"

#include "ns3/flow-monitor-module.h"

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("PowerAdaptationGrid");

// packet size generated at the AP
static const uint32_t packetSize = 1420;

class NodeStatistics
{
public:
  NodeStatistics ();
  NodeStatistics (NetDeviceContainer aps, NetDeviceContainer stas);

  void Setup (NetDeviceContainer aps, NetDeviceContainer stas);

  void CheckStatistics (double time);

  void PhyCallback (std::string path, Ptr<const Packet> packet);
  void RxCallback (std::string path, Ptr<const Packet> packet, const Address &from);
  void PowerCallback (std::string path, uint8_t power, Mac48Address dest);
  void RateCallback (std::string path, uint32_t rate, Mac48Address dest);
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

  std::map<Mac48Address, uint32_t> actualPower;
  std::map<Mac48Address, WifiMode> actualMode;
  uint32_t m_bytesTotal;
  double totalEnergy;
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
  TxTime timeTable;
  Gnuplot2dDataset m_output;
  Gnuplot2dDataset m_output_power;
  Gnuplot2dDataset m_output_idle;
  Gnuplot2dDataset m_output_busy;
  Gnuplot2dDataset m_output_rx;
  Gnuplot2dDataset m_output_tx;
};

NodeStatistics::NodeStatistics ()
{
  totalEnergy = 0;
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

NodeStatistics::NodeStatistics (NetDeviceContainer aps, NetDeviceContainer stas)
{
  Ptr<NetDevice> device = aps.Get (0);
  Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice> (device);
  Ptr<WifiPhy> phy = wifiDevice->GetPhy ();
  myPhy = phy;
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
NodeStatistics::Setup (NetDeviceContainer aps, NetDeviceContainer stas)
{
  Ptr<NetDevice> device = aps.Get (0);
  Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice> (device);
  Ptr<WifiPhy> phy = wifiDevice->GetPhy ();
  myPhy = phy;
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
      totalEnergy += pow(10, actualPower[dest] / 10) * GetCalcTxTime (actualMode[dest]).GetSeconds ();
      totalTime += GetCalcTxTime (actualMode[dest]).GetSeconds ();
    }
}

void
NodeStatistics::PowerCallback (std::string path, uint8_t power, Mac48Address dest)
{
  double   txPowerBaseDbm = myPhy->GetTxPowerStart ();
  double   txPowerEndDbm = myPhy->GetTxPowerEnd ();
  uint32_t nTxPower = myPhy->GetNTxPower ();
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
NodeStatistics::RateCallback (std::string path, uint32_t rate, Mac48Address dest)
{
  actualMode[dest] = myPhy->GetMode (rate);
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
  double atp = totalEnergy / time;
  totalEnergy = 0;
  totalTime = 0;
  m_output_power.Add ((Simulator::Now ()).GetSeconds (), atp);
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
NodeStatistics::GetPowerDatafile ()
{
  return m_output_power;
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

double
NodeStatistics::GetBusyTime ()
{
  return totalBusyTime + totalRxTime;
}

void PowerCallback (std::string path, uint8_t power, Mac48Address dest)
{
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " " << dest << " Power " <<  (int)power);
}

void BluesPowerCallback (std::string path, uint8_t power, Mac48Address dest, std::string type)
{
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " station: " << dest << ", frame sent with " << type << " power: " << (int)power);
}

void RateCallback (std::string path, uint32_t rate, Mac48Address dest)
{
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " " << dest << " Rate " <<  rate);
}

void BluesRateCallback (std::string path, uint32_t rate, Mac48Address dest, std::string type)
{
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " station: " << dest << ", frame sent with " << type << " rate: " <<  rate);
}

int main (int argc, char *argv[])
{
  double maxPower = 22;
  double minPower = 1;
  uint32_t powerLevels = 22;
  double edThreshold = -96.0;

  uint32_t rtsThreshold=2346;
  std::string manager="ns3::ParfWifiManager";
  std::string outputFileName = "parf";
  std::string rate="54Mbps";
  double exponent = 3.0;
  int simuTime = 100;
  int gridDist = 100;
  int scenarioSize = 500;
  bool longOutput = true;
  uint32_t nActiveStas = 0;
  std::string protocol = "UDP";
  uint32_t maxBytes = 100000000;

  CommandLine cmd;
  cmd.AddValue ("manager", "PRC Manager", manager);
  cmd.AddValue ("rate", "Data Rate", rate);
  cmd.AddValue ("exponent", "Loss Model Exponent", exponent);
  cmd.AddValue ("edThreshold", "The energy detection threshold of the PHY layer.", edThreshold);
  cmd.AddValue ("rtsThreshold", "RTS threshold", rtsThreshold);
  cmd.AddValue ("outputFileName", "Output filename", outputFileName);
  cmd.AddValue ("simuTime", "Total simulation time", simuTime);
  cmd.AddValue ("scenarioSize", "Size of the side of the square scenario", scenarioSize);
  cmd.AddValue ("gridDist", "Distance between APs on the grid", gridDist);
  cmd.AddValue ("activeStas", "Number of active stations", nActiveStas);
  cmd.AddValue ("longOutput", "Print verbose statistics of the simulation", longOutput);
  cmd.AddValue ("maxPower", "Maximum available transmission level (dbm).", maxPower);
  cmd.AddValue ("minPower", "Minimum available transmission level (dbm).", minPower);
  cmd.AddValue ("powerLevels", "Number of transmission power levels available between "
          "TxPowerStart and TxPowerEnd included.", powerLevels);
  cmd.AddValue ("protocol", "Use UDP or TCP to generate traffic.", protocol);
  cmd.AddValue ("maxBytes", "When using TCP, how many bytes to send. Use 0 for no limit.", maxBytes);
  cmd.Parse (argc, argv);

  uint32_t gridWidth = scenarioSize/gridDist + 1;
  uint32_t numAps = gridWidth*gridWidth;
  uint32_t numStas = numAps;
  if (nActiveStas == 0 || nActiveStas > numStas)
    nActiveStas = numStas;

  // Define the APs
  NodeContainer wifiApNodes;
  wifiApNodes.Create (numAps);

  //Define the STAs
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create(numStas);

  WifiHelper wifi = WifiHelper::Default ();
  wifi.SetStandard(WIFI_PHY_STANDARD_80211a);
  wifi.SetPowerLimitation(WIFI_PHY_POWER_LIMITATION_WISTRON_DCMA_82_80211a);
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel;// = YansWifiChannelHelper::Default ();
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::ThreeLogDistancePropagationLossModel");
  //wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");

  wifiPhy.SetChannel (wifiChannel.Create ());

  NetDeviceContainer wifiApDevices;
  NetDeviceContainer wifiStaDevices;
  NetDeviceContainer wifiDevices;

  //Configure the STA nodes
  wifi.SetRemoteStationManager ("ns3::MinstrelWifiManager", "RtsCtsThreshold", UintegerValue(rtsThreshold));
  wifiPhy.Set("TxPowerStart", DoubleValue(maxPower));
  wifiPhy.Set("TxPowerEnd", DoubleValue(maxPower));

  for (uint32_t i = 0; i < numStas; i++)
    {
      NetDeviceContainer staDev;

      // setup the AP.
      char numstr[21]; // enough to hold all numbers up to 64-bits
      sprintf(numstr, "%d", i);
      std::string name = "AP";
      std::string result = name + numstr;
      Ssid ssid = Ssid (result);
      wifiMac.SetType ("ns3::StaWifiMac",
                       "Ssid", SsidValue (ssid),
                       "ActiveProbing", BooleanValue (false));
      staDev = (wifi.Install (wifiPhy, wifiMac, wifiStaNodes.Get(i)));
      wifiStaDevices.Add(staDev);
    }

  //Configure the AP nodes
  wifi.SetRemoteStationManager (manager, "DefaultTxPowerLevel", UintegerValue(maxPower), "RtsCtsThreshold", UintegerValue(rtsThreshold));
  wifiPhy.Set("TxPowerStart", DoubleValue(minPower));
  wifiPhy.Set("TxPowerEnd", DoubleValue(maxPower));
  wifiPhy.Set("TxPowerLevels", UintegerValue(powerLevels));
  wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-80));
  //wifiPhy.Set ("CcaMode1Threshold", DoubleValue (-70.0));

  for (uint32_t i = 0; i < numAps; i++)
    {
      NetDeviceContainer apDev;

      // setup the AP.
      char numstr[21]; // enough to hold all numbers up to 64-bits
      sprintf(numstr, "%d", i);
      std::string name = "AP";
      std::string result = name + numstr;
      Ssid ssid = Ssid (result);
      wifiMac.SetType ("ns3::ApWifiMac",
                       "Ssid", SsidValue (ssid),
                       "BeaconInterval", TimeValue(MicroSeconds(103424+i)));
      apDev = (wifi.Install (wifiPhy, wifiMac, wifiApNodes.Get(i)));
      wifiApDevices.Add(apDev);
    }

  wifiDevices.Add(wifiStaDevices);
  wifiDevices.Add(wifiApDevices);

  // Configure the mobility.
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
      "MinX", DoubleValue (0.0),
      "MinY", DoubleValue (0.0),
      "DeltaX", DoubleValue (gridDist),
      "DeltaY", DoubleValue (gridDist),
      "GridWidth", UintegerValue (gridWidth),
      "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNodes);

  /*
  Ptr<UniformRandomVariable> rho = CreateObject<UniformRandomVariable> ();
  rho->SetAttribute ("Min", DoubleValue (0.0));
  rho->SetAttribute ("Max", DoubleValue (gridDist/4));

  for (uint32_t i=0;i<numStas;i++)
   {
     uint32_t x = (i%gridWidth)*gridDist;
     uint32_t y = (i/gridWidth)*gridDist;
     mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
	"X", DoubleValue(x),
	"Y", DoubleValue(y),
	 "Rho", PointerValue(rho));
     mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
     mobility.Install(wifiStaNodes.Get(i));
   }
   */

  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (-5, 12, 0.0));
  positionAlloc->Add (Vector (gridDist+15, 15, 0.0));
  positionAlloc->Add (Vector (12, gridDist+12, 0.0));
  positionAlloc->Add (Vector (gridDist-12, gridDist-12, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiStaNodes);

  //Configure the IP stack
  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer iface = address.Assign (wifiDevices);

  //Configure the CBR generator
  ApplicationContainer apps_sink;
  ApplicationContainer apps_source;
  uint16_t port = 9;

  if (protocol.compare ("UDP"))
    {
      for (uint32_t i=0;i<nActiveStas;i++)
        {
          Ipv4Address sinkAddress = iface.GetAddress(i);
          PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
          apps_sink.Add(sink.Install (wifiStaNodes.Get (i)));

          OnOffHelper onoff ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
          onoff.SetConstantRate (DataRate (rate), 1420);
          onoff.SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
          onoff.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
          onoff.Install (wifiApNodes.Get (i));
        }
    }
  else
    {
      for (uint32_t i=0;i<nActiveStas;i++)
        {
          Ipv4Address sinkAddress = iface.GetAddress(i);
          PacketSinkHelper sink ("ns3::TcpSocketFactory", InetSocketAddress (sinkAddress, port));
          apps_sink = sink.Install (wifiStaNodes.Get (i));

          BulkSendHelper source ("ns3::TcpSocketFactory", InetSocketAddress (sinkAddress, port));
          source.SetAttribute("MaxBytes", UintegerValue(maxBytes));
          apps_source = source.Install (wifiApNodes.Get (i));
        }
    }

  apps_sink.Start (Seconds (0.5));
  apps_sink.Stop (Seconds (simuTime));

  //------------------------------------------------------------
  //-- Setup stats and data collection
  //--------------------------------------------

  //Statics counters
  NodeStatistics* statisticsCounter = new NodeStatistics[numAps+numStas];

  for (uint32_t i=0;i<numAps;i++)
    {
      statisticsCounter[i].Setup (wifiApDevices, wifiStaDevices);
      statisticsCounter[i+numAps].Setup (wifiApDevices, wifiStaDevices);

      //Register packet receptions to calculate throughput
      std::stringstream stas;
      stas << i+numAps;
      Config::Connect ("/NodeList/"+stas.str()+"/ApplicationList/*/$ns3::PacketSink/Rx",
                       MakeCallback (&NodeStatistics::RxCallback, &statisticsCounter[i]));

      //Register power and rate changes to calculate the Average Transmit Power
      std::stringstream aps;
      aps << i;

      Config::Connect ("/NodeList/"+aps.str()+"/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
                       MakeCallback (&NodeStatistics::PowerCallback, &statisticsCounter[i]));
      Config::Connect ("/NodeList/"+aps.str()+"/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChange",
                       MakeCallback (&NodeStatistics::RateCallback, &statisticsCounter[i]));

      Config::Connect ("/NodeList/"+aps.str()+"/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
                       MakeCallback (&NodeStatistics::PhyCallback, &statisticsCounter[i]));

      //Register States of APs
      Config::Connect ("/NodeList/"+aps.str()+"/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/State/State",
                       MakeCallback (&NodeStatistics::StateCallback, &statisticsCounter[i]));

      //Register States of STAs
      Config::Connect ("/NodeList/"+stas.str()+"/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/State/State",
                       MakeCallback (&NodeStatistics::StateCallback, &statisticsCounter[i+numAps]));

      statisticsCounter[i].CheckStatistics(1);
      statisticsCounter[i+numAps].CheckStatistics(1);

      //Callbacks to print every change of power and rate
      if (manager.compare ("ns3::MinstrelBluesWifiManager") == 0)
        {
          Config::Connect ("/NodeList/"+aps.str()+"/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChangeWithInfo",
                             MakeCallback (BluesPowerCallback));
          Config::Connect ("/NodeList/"+aps.str()+"/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChangeWithInfo",
                           MakeCallback (BluesRateCallback));
        }
      else
        {
          Config::Connect ("/NodeList/"+aps.str()+"/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
                            MakeCallback (PowerCallback));
          Config::Connect ("/NodeList/"+aps.str()+"/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChange",
                            MakeCallback (RateCallback));
        }
    }

  // Calculate Throughput using Flowmonitor
  //

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  Simulator::Stop (Seconds (simuTime));
  Simulator::Run ();

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

  if (longOutput)
    {
      double totalTh = 0;
      for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
	{
	  Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
	  double th = i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds())/1024/1024;
	  totalTh = totalTh + th;
	  NS_LOG_INFO("Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n");
	  NS_LOG_INFO("  Tx Bytes:   " << i->second.txBytes << "\n");
	  NS_LOG_INFO("  Rx Bytes:   " << i->second.rxBytes << "\n");
	  NS_LOG_INFO("  Throughput: " << th  << " Mbps\n");
	  NS_LOG_INFO("  Mean delay:   " << i->second.delaySum.GetSeconds() / i->second.rxPackets << "\n");
	  NS_LOG_INFO("  Mean jitter:   " << i->second.jitterSum.GetSeconds() / (i->second.rxPackets - 1) << "\n");
	  NS_LOG_INFO("  Tx Opp: " << 1 - (statisticsCounter[i->first-1].GetBusyTime() / simuTime));
	}
      NS_LOG_INFO("  Total Throughput: " << totalTh  << " Mbps\n");
    }
  else
    {
      //Print a row with all the statistics.
      double totalTh = 0;
      for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
	{
	  double th = i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds())/1024/1024;
	  totalTh += th;
	  NS_LOG_UNCOND(manager << " " << RngSeedManager::GetRun () << " " << i->first
	                << " " << th
	                << " " << i->second.delaySum.GetSeconds() / i->second.rxPackets
	                << " " << i->second.jitterSum.GetSeconds() / (i->second.rxPackets - 1)
	                << " " << 1 - (statisticsCounter[i->first-1].GetBusyTime() / simuTime));
	}
      NS_LOG_UNCOND("  Total Throughput: " << totalTh  << " Mbps\n");
    }

  for (uint32_t i=0;i<numAps;i++)
    {
      std::stringstream ap;
      ap << i;
      std::ofstream outfileTh (("throughput-" + outputFileName + "-AP" + ap.str() + ".plt").c_str ());
      Gnuplot gnuplot = Gnuplot (("throughput-" + outputFileName + "-AP" + ap.str() + ".eps").c_str (), "Throughput");
      gnuplot.SetTerminal ("post eps color enhanced");
      gnuplot.SetLegend ("Time (seconds)", "Throughput (Mb/s)");
      gnuplot.SetTitle (("Throughput (AP" + ap.str() + " to STA) vs time").c_str ());
      gnuplot.AddDataset (statisticsCounter[i].GetDatafile ());
      gnuplot.GenerateOutput (outfileTh);

      /*******/

      std::ofstream outfileTx (("tx-" + outputFileName + "-AP" + ap.str() + ".plt").c_str ());
      gnuplot = Gnuplot (("tx-" + outputFileName + "-AP" + ap.str() + ".eps").c_str (), "Time in TX State");
      gnuplot.SetTerminal ("post eps color enhanced");
      gnuplot.SetLegend ("Time (seconds)", "Percent");
      gnuplot.SetTitle (("Percentage time AP" + ap.str() + " in TX state vs time").c_str ());
      gnuplot.AddDataset (statisticsCounter[i].GetTxDatafile ());
      gnuplot.GenerateOutput (outfileTx);

      std::ofstream outfileRx (("rx-" + outputFileName + "-AP" + ap.str() + ".plt").c_str ());
      gnuplot = Gnuplot (("rx-" + outputFileName + "-AP" + ap.str() + ".eps").c_str (), "Time in RX State");
      gnuplot.SetTerminal ("post eps color enhanced");
      gnuplot.SetLegend ("Time (seconds)", "Percent");
      gnuplot.SetTitle (("Percentage time AP" + ap.str() + " in RX state vs time").c_str ());
      gnuplot.AddDataset (statisticsCounter[i].GetRxDatafile ());
      gnuplot.GenerateOutput (outfileRx);

      std::ofstream outfileBusy (("busy-" + outputFileName + "-AP" + ap.str() + ".plt").c_str ());
      gnuplot = Gnuplot (("busy-" + outputFileName + "-AP" + ap.str() + ".eps").c_str (), "Time in Busy State");
      gnuplot.SetTerminal ("post eps color enhanced");
      gnuplot.SetLegend ("Time (seconds)", "Percent");
      gnuplot.SetTitle (("Percentage time AP" + ap.str() + " in Busy state vs time").c_str ());
      gnuplot.AddDataset (statisticsCounter[i].GetBusyDatafile ());
      gnuplot.GenerateOutput (outfileBusy);

      std::ofstream outfileIdle (("idle-" + outputFileName + "-AP" + ap.str() + ".plt").c_str ());
      gnuplot = Gnuplot (("idle-" + outputFileName + "-AP" + ap.str() + ".eps").c_str (), "Time in Idle State");
      gnuplot.SetTerminal ("post eps color enhanced");
      gnuplot.SetLegend ("Time (seconds)", "Percent");
      gnuplot.SetTitle (("Percentage time AP" + ap.str() + " in Idle state vs time").c_str ());
      gnuplot.AddDataset (statisticsCounter[i].GetIdleDatafile ());
      gnuplot.GenerateOutput (outfileIdle);

      /*******/

      uint32_t j = i+numAps;
      std::ofstream outfileTxSta (("tx-" + outputFileName + "-STA" + ap.str() + ".plt").c_str ());
      gnuplot = Gnuplot (("tx-" + outputFileName + "-STA" + ap.str() + ".eps").c_str (), "Time in TX State");
      gnuplot.SetTerminal ("post eps color enhanced");
      gnuplot.SetLegend ("Time (seconds)", "Percent");
      gnuplot.SetTitle (("Percentage time STA" + ap.str() + " in TX state vs time").c_str ());
      gnuplot.AddDataset (statisticsCounter[j].GetTxDatafile ());
      gnuplot.GenerateOutput (outfileTxSta);

      std::ofstream outfileRxSta (("rx-" + outputFileName + "-STA" + ap.str() + ".plt").c_str ());
      gnuplot = Gnuplot (("rx-" + outputFileName + "-STA" + ap.str() + ".eps").c_str (), "Time in RX State");
      gnuplot.SetTerminal ("post eps color enhanced");
      gnuplot.SetLegend ("Time (seconds)", "Percent");
      gnuplot.SetTitle (("Percentage time STA" + ap.str() + " in RX state vs time").c_str ());
      gnuplot.AddDataset (statisticsCounter[j].GetRxDatafile ());
      gnuplot.GenerateOutput (outfileRxSta);

      std::ofstream outfileBusySta (("busy-" + outputFileName + "-STA" + ap.str() + ".plt").c_str ());
      gnuplot = Gnuplot (("busy-" + outputFileName + "-STA" + ap.str() + ".eps").c_str (), "Time in Busy State");
      gnuplot.SetTerminal ("post eps color enhanced");
      gnuplot.SetLegend ("Time (seconds)", "Percent");
      gnuplot.SetTitle (("Percentage time STA" + ap.str() + " in Busy state vs time").c_str ());
      gnuplot.AddDataset (statisticsCounter[j].GetBusyDatafile ());
      gnuplot.GenerateOutput (outfileBusySta);

      std::ofstream outfileIdleSta (("idle-" + outputFileName + "-STA" + ap.str() + ".plt").c_str ());
      gnuplot = Gnuplot (("idle-" + outputFileName + "-STA" + ap.str() + ".eps").c_str (), "Time in Idle State");
      gnuplot.SetTerminal ("post eps color enhanced");
      gnuplot.SetLegend ("Time (seconds)", "Percent");
      gnuplot.SetTitle (("Percentage time STA" + ap.str() + " in Idle state vs time").c_str ());
      gnuplot.AddDataset (statisticsCounter[j].GetIdleDatafile ());
      gnuplot.GenerateOutput (outfileIdleSta);
    }

  delete [] statisticsCounter;

  Simulator::Destroy ();

  return 0;
}
