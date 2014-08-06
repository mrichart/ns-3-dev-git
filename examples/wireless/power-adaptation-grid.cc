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
* Another example (changing STAs position):
* ./waf --run "power-adaptation-grid --manager=ns3::AparfWifiManager --scenarioSize=500 --gridDist=100"
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

class APStatics
{
public:
  APStatics();
  APStatics(NetDeviceContainer aps, NetDeviceContainer stas);
  void SetDevices(NetDeviceContainer aps, NetDeviceContainer stas);

  void CheckStatics (double time);

  void PhyCallback (std::string path, Ptr<const Packet> packet);
  void RxCallback (std::string path, Ptr<const Packet> packet, const Address &from);
  void PowerCallback (std::string path, uint8_t power, Mac48Address dest);
  void RateCallback (std::string path, uint32_t rate, Mac48Address dest);
  void StateCallback (std::string path, Time init, Time duration, enum WifiPhy::State state);

  Gnuplot2dDataset GetDatafile();
  Gnuplot2dDataset GetPowerDatafile();
  Gnuplot2dDataset GetIdleDatafile();
  Gnuplot2dDataset GetBusyDatafile();
  Gnuplot2dDataset GetTxDatafile();
  Gnuplot2dDataset GetRxDatafile();

  double GetBusyTime();

private:
  typedef std::vector<std::pair<Time,WifiMode> > TxTime;
  void SetupPhy (Ptr<WifiPhy> phy);
  Time GetCalcTxTime (WifiMode mode);

  std::map<Mac48Address, uint32_t> actualPower;
  std::map<Mac48Address, WifiMode> actualMode;
  uint32_t m_bytesTotal;
  double totalPower;
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

APStatics::APStatics()
{
  totalPower = 0;
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
  m_output.SetTitle("Throughput Mbits/s");
  m_output_idle.SetTitle("Idle Time");
  m_output_busy.SetTitle("Busy Time");
  m_output_rx.SetTitle("RX Time");
  m_output_tx.SetTitle("TX Time");
}

APStatics::APStatics(NetDeviceContainer aps, NetDeviceContainer stas)
{
  Ptr<NetDevice> device = aps.Get(0);
  Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice> (device);
  Ptr<WifiPhy> phy = wifiDevice->GetPhy();
  myPhy = phy;
  SetupPhy(phy);
  for (uint32_t j=0; j<stas.GetN(); j++)
    {
      Ptr<NetDevice> staDevice = stas.Get(j);
      Ptr<WifiNetDevice> wifiStaDevice = DynamicCast<WifiNetDevice> (staDevice);
      Mac48Address addr = wifiStaDevice->GetMac()->GetAddress();
      actualPower[addr] = 17;
      actualMode[addr] = phy->GetMode (0);
    }
  actualMode[Mac48Address("ff:ff:ff:ff:ff:ff")] = phy->GetMode (0);
  totalPower = 0;
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
  m_output.SetTitle("Throughput Mbits/s");
  m_output_idle.SetTitle("Idle Time");
  m_output_busy.SetTitle("Busy Time");
  m_output_rx.SetTitle("RX Time");
  m_output_tx.SetTitle("TX Time");
}

void
APStatics::SetDevices(NetDeviceContainer aps, NetDeviceContainer stas)
{
  Ptr<NetDevice> device = aps.Get(0);
  Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice> (device);
  Ptr<WifiPhy> phy = wifiDevice->GetPhy();
  myPhy = phy;
  SetupPhy(phy);
  for (uint32_t j=0; j<stas.GetN(); j++)
    {
      Ptr<NetDevice> staDevice = stas.Get(j);
      Ptr<WifiNetDevice> wifiStaDevice = DynamicCast<WifiNetDevice> (staDevice);
      Mac48Address addr = wifiStaDevice->GetMac()->GetAddress();
      actualPower[addr] = 17;
      actualMode[addr] = phy->GetMode (0);
    }
  actualMode[Mac48Address("ff:ff:ff:ff:ff:ff")] = phy->GetMode (0);
}

void
APStatics::SetupPhy (Ptr<WifiPhy> phy)
{
  uint32_t nModes = phy->GetNModes ();
  for (uint32_t i = 0; i < nModes; i++)
    {
      WifiMode mode = phy->GetMode (i);
      WifiTxVector txVector;
      txVector.SetMode(mode);
      timeTable.push_back (std::make_pair (phy->CalculateTxDuration (1420, txVector, WIFI_PREAMBLE_LONG), mode));
    }
}

Time
APStatics::GetCalcTxTime (WifiMode mode)
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
APStatics::PhyCallback (std::string path, Ptr<const Packet> packet)
{
  WifiMacHeader head;
  packet->PeekHeader(head);
  Mac48Address dest = head.GetAddr1();

  //NS_LOG_UNCOND ((Simulator::Now ()).GetSeconds () << " " << node  << " " << dest << " " << actualMode[node][dest].GetDataRate()/1000000 << " " << (int)actualPower[node][dest]);

  totalPower += actualPower[dest] * GetCalcTxTime(actualMode[dest]).GetSeconds();
  totalTime += GetCalcTxTime(actualMode[dest]).GetSeconds();

}

void
APStatics::PowerCallback (std::string path, uint8_t power, Mac48Address dest)
{
  double   txPowerBaseDbm = myPhy->GetTxPowerStart();
  double   txPowerEndDbm = myPhy->GetTxPowerEnd();
  uint32_t nTxPower = myPhy->GetNTxPower();
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
APStatics::RateCallback (std::string path, uint32_t rate, Mac48Address dest)
{
  actualMode[dest] = myPhy->GetMode(rate);
}

void
APStatics::StateCallback (std::string path, Time init, Time duration, enum WifiPhy::State state)
{
  if (state == WifiPhy::CCA_BUSY)
    {
      busyTime += duration.GetSeconds();
      totalBusyTime += duration.GetSeconds();
    }
  else if (state == WifiPhy::IDLE)
    {
      idleTime += duration.GetSeconds();
      totalIdleTime += duration.GetSeconds();
    }
  else if (state == WifiPhy::TX)
    {
      txTime += duration.GetSeconds();
      totalTxTime += duration.GetSeconds();
    }
  else if (state == WifiPhy::RX)
    {
      rxTime += duration.GetSeconds();
      totalRxTime += duration.GetSeconds();
    }
}

void
APStatics::RxCallback (std::string path, Ptr<const Packet> packet, const Address &from)
{
  m_bytesTotal += packet->GetSize();
}

void
APStatics::CheckStatics(double time)
{
  double mbs = ((m_bytesTotal * 8.0) / (1000000*time));
  m_bytesTotal = 0;
  double atm = pow (10, ((totalPower / time) / 10));
  totalPower = 0;
  totalTime = 0;
  m_output_power.Add ((Simulator::Now ()).GetSeconds (), atm);
  m_output.Add ((Simulator::Now ()).GetSeconds (), mbs);

  m_output_idle.Add ((Simulator::Now ()).GetSeconds (), idleTime);
  m_output_busy.Add ((Simulator::Now ()).GetSeconds (), busyTime);
  m_output_tx.Add ((Simulator::Now ()).GetSeconds (), txTime);
  m_output_rx.Add ((Simulator::Now ()).GetSeconds (), rxTime);
  busyTime = 0;
  idleTime = 0;
  txTime = 0;
  rxTime = 0;

  Simulator::Schedule (Seconds (time), &APStatics::CheckStatics, this, time);
}

Gnuplot2dDataset
APStatics::GetDatafile()
{ return m_output; }

Gnuplot2dDataset
APStatics::GetPowerDatafile()
{ return m_output_power; }

Gnuplot2dDataset
APStatics::GetIdleDatafile()
{ return m_output_idle; }

Gnuplot2dDataset
APStatics::GetBusyDatafile()
{ return m_output_busy; }

Gnuplot2dDataset
APStatics::GetRxDatafile()
{ return m_output_rx; }

Gnuplot2dDataset
APStatics::GetTxDatafile()
{ return m_output_tx; }

double
APStatics::GetBusyTime()
{
  return totalBusyTime + totalRxTime;
}

void PowerCallback (std::string path, uint8_t power, Mac48Address dest) {
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " " << dest << " Power " <<  (int)power);
  // end PowerCallback
}

void RateCallback (std::string path, uint32_t rate, Mac48Address dest) {
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " " << dest << " Rate " <<  rate);
  // end PowerCallback
}

int main (int argc, char *argv[])
{
  double maxPower = 17;
  double minPower = 0;
  uint32_t powerLevels = 18;

  uint32_t rtsThreshold=2346;
  std::string manager="ns3::ParfWifiManager";
  int ap1_x = 0;
  int ap1_y = 0;
  int sta1_x = 10;
  int sta1_y = 0;
  int ap2_x = 200;
  int ap2_y = 0;
  int sta2_x = 180;
  int sta2_y = 0;
  int simuTime = 100;
  int gridDist = 100;
  int scenarioSize = 500;
  bool longOutput = true;

  CommandLine cmd;
  cmd.AddValue ("manager", "PRC Manager", manager);
  cmd.AddValue ("rtsThreshold", "RTS threshold", rtsThreshold);
  cmd.AddValue ("simuTime", "Total simulation time", simuTime);
  cmd.AddValue ("scenarioSize", "Size of the side of the square scenario", scenarioSize);
  cmd.AddValue ("gridDist", "Distance between APs on the grid", gridDist);
  cmd.AddValue ("longOutput", "Print verbose statistics of the simulation", longOutput);
  cmd.AddValue ("maxPower", "Maximum available transmission level (dbm).", maxPower);
  cmd.AddValue ("minPower", "Minimum available transmission level (dbm).", minPower);
  cmd.AddValue ("powerLevels", "Number of transmission power levels available between "
          "TxPowerStart and TxPowerEnd included.", powerLevels);
  cmd.AddValue ("AP1_x", "Position of AP1 in x coordinate", ap1_x);
  cmd.AddValue ("AP1_y", "Position of AP1 in y coordinate", ap1_y);
  cmd.AddValue ("STA1_x", "Position of STA1 in x coordinate", sta1_x);
  cmd.AddValue ("STA1_y", "Position of STA1 in y coordinate", sta1_y);
  cmd.AddValue ("AP2_x", "Position of AP2 in x coordinate", ap2_x);
  cmd.AddValue ("AP2_y", "Position of AP2 in y coordinate", ap2_y);
  cmd.AddValue ("STA2_x", "Position of STA2 in x coordinate", sta2_x);
  cmd.AddValue ("STA2_y", "Position of STA2 in y coordinate", sta2_y);
  cmd.Parse (argc, argv);

  uint32_t gridWidth = scenarioSize/gridDist + 1;
  uint32_t numAps = gridWidth*gridWidth;
  uint32_t numStas = numAps;

  // Define the APs
  NodeContainer wifiApNodes;
  wifiApNodes.Create (numAps);

  //Define the STAs
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create(numStas);

  WifiHelper wifi = WifiHelper::Default ();
  wifi.SetStandard(WIFI_PHY_STANDARD_80211a);
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
//  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
//  wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel", "Exponent", DoubleValue(5));

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


  for (uint32_t i=0;i<numStas;i++)
    {
      uint32_t x = (i%gridWidth)*gridDist;
      uint32_t y = (i/gridWidth)*gridDist;
      mobility.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
      	  "X", DoubleValue(x),
      	  "Y", DoubleValue(y),
          "Rho", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=50.0]"));
      mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
      mobility.Install(wifiStaNodes.Get(i));
    }

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

  for (uint32_t i=0;i<numStas;i++)
    {
      Ipv4Address sinkAddress = iface.GetAddress(i);
      PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
      apps_sink.Add(sink.Install (wifiStaNodes.Get (i)));

      OnOffHelper onoff ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
      onoff.SetConstantRate (DataRate ("54Mb/s"), 1420);
      onoff.SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
      onoff.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
      onoff.Install (wifiApNodes.Get (i));
    }

  apps_sink.Start (Seconds (0.5));
  apps_sink.Stop (Seconds (simuTime));

  //------------------------------------------------------------
  //-- Setup stats and data collection
  //--------------------------------------------

  //Statics counters
  APStatics staticsCounter[numStas];

  for (uint32_t i=0;i<numStas;i++)
    {
      staticsCounter[i].SetDevices(wifiApDevices, wifiStaDevices);
      //Register packet receptions to calculate throughput
      std::stringstream stas;
      stas << i+numStas;
      Config::Connect ("/NodeList/"+stas.str()+"/ApplicationList/*/$ns3::PacketSink/Rx",
                         MakeCallback (&APStatics::RxCallback, &staticsCounter[i]));
      //Register power and rate changes to calculate the Average Transmit Power
      std::stringstream aps;
      aps << i;
      Config::Connect ("/NodeList/"+aps.str()+"/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
                       MakeCallback (&APStatics::PowerCallback, &staticsCounter[i]));
      Config::Connect ("/NodeList/"+aps.str()+"/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
                       MakeCallback (&APStatics::PhyCallback, &staticsCounter[i]));
      //Register States
      Config::Connect ("/NodeList/"+aps.str()+"/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/State/State",
                        MakeCallback (&APStatics::StateCallback, &staticsCounter[i]));
      staticsCounter[i].CheckStatics(1);
    }

  //Callbacks to print every change of power and rate
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
                    MakeCallback (PowerCallback));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChange",
                    MakeCallback (RateCallback));

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
	  NS_LOG_INFO("  Tx Opp: " << 1 - (staticsCounter[i->first-1].GetBusyTime() / simuTime));
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
	  totalTh = totalTh + th;
	  NS_LOG_UNCOND(manager << " " << RngSeedManager::GetRun () << " " << i->first
	                << " " << th
	                << " " << i->second.delaySum.GetSeconds() / i->second.rxPackets
	                << " " << i->second.jitterSum.GetSeconds() / (i->second.rxPackets - 1)
	                << " " << 1 - (staticsCounter[i->first-1].GetBusyTime() / simuTime));
	}
    }

  Simulator::Destroy ();

  return 0;
}
