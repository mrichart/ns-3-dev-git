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
* This simulation consist of 4 nodes, two APs and two STAs.
* The APs generates UDP traffic with a CBR of 54 Mbps to the STAs.
* The APa use any power and rate control mechanism and the STAs use only rate control.
* The STAs can be configured to be at any distance from the APs.
*
* The objective is to test power and rate control in the links with interference from the other link.
*
* The output consists of:
* - A plot of average throughput vs. time.
* - A plot of average transmit power vs. time.
* - Plots for the percentage of time the APs are in each MAC state (IDLE, TX, RX, BUSY)
* - If enabled, the changes of power and rate to standard output.
* - If enabled, the average throughput, delay, jitter and tx opportunity for the total simulation time.
*
* Example usage:
* ./waf --run "power-adaptation-interf --manager=ns3::AparfWifiManager --outputFileName=aparf"
*
* Another example (changing STAs position):
* ./waf --run "power-adaptation-interf --manager=ns3::AparfWifiManager --outputFileName=aparf --STA1_x=5 --STA2_x=205"
*
* To enable the log of rate and power changes:
* export NS_LOG=PowerAdaptationInterf=level_info
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

NS_LOG_COMPONENT_DEFINE ("PowerAdaptationInterf");

class APStatics
{
public:
  APStatics(NetDeviceContainer aps, NetDeviceContainer stas);

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
APStatics::SetupPhy (Ptr<WifiPhy> phy)
{
  uint32_t nModes = phy->GetNModes ();
  for (uint32_t i = 0; i < nModes; i++)
    {
      WifiMode mode = phy->GetMode (i);
      WifiTxVector txVector;
      txVector.SetMode(mode);
      timeTable.push_back (std::make_pair (phy->CalculateTxDuration (1420, txVector, WIFI_PREAMBLE_LONG, phy->GetFrequency()), mode));
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
  //LogComponentEnable("ConstantRateWifiManager", LOG_LEVEL_FUNCTION);

  double maxPower = 17;
  double minPower = 0;
  uint32_t powerLevels = 18;

  uint32_t rtsThreshold=2346;
  std::string manager="ns3::ParfWifiManager";
  std::string outputFileName="parf";
  int ap1_x = 0;
  int ap1_y = 0;
  int sta1_x = 10;
  int sta1_y = 0;
  int ap2_x = 200;
  int ap2_y = 0;
  int sta2_x = 180;
  int sta2_y = 0;
  int simuTime = 100;

  CommandLine cmd;
  cmd.AddValue ("manager", "PRC Manager", manager);
  cmd.AddValue ("rtsThreshold", "RTS threshold", rtsThreshold);
  cmd.AddValue ("outputFileName", "Output filename", outputFileName);
  cmd.AddValue ("simuTime", "Total simulation time", simuTime);
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

  // Define the APs
  NodeContainer wifiApNodes;
  wifiApNodes.Create (2);

  //Define the STAs
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create(2);

  WifiHelper wifi = WifiHelper::Default ();
  wifi.SetStandard(WIFI_PHY_STANDARD_80211a);
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();

  wifiPhy.SetChannel (wifiChannel.Create ());

  NetDeviceContainer wifiApDevices;
  NetDeviceContainer wifiStaDevices;
  NetDeviceContainer wifiDevices;

  //Configure the STA nodes
  wifi.SetRemoteStationManager ("ns3::AarfWifiManager", "RtsCtsThreshold", UintegerValue(rtsThreshold));
  //wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",StringValue ("ErpOfdmRate6Mbps"),"ControlMode",StringValue ("ErpOfdmRate6Mbps"));
  wifiPhy.Set("TxPowerStart", DoubleValue(maxPower));
  wifiPhy.Set("TxPowerEnd", DoubleValue(maxPower));

  Ssid ssid = Ssid ("AP0");
  wifiMac.SetType ("ns3::StaWifiMac",
		  	  	  "Ssid", SsidValue (ssid),
		  	  	  "ActiveProbing", BooleanValue (false),
		  	  	   "MaxMissedBeacons", UintegerValue(1000));
  wifiStaDevices.Add(wifi.Install (wifiPhy, wifiMac, wifiStaNodes.Get(0)));

  ssid = Ssid ("AP1");
  wifiMac.SetType ("ns3::StaWifiMac",
        	   "Ssid", SsidValue (ssid),
        	   "ActiveProbing", BooleanValue (false));
  wifiStaDevices.Add(wifi.Install (wifiPhy, wifiMac, wifiStaNodes.Get(1)));

  //Configure the AP nodes
  wifi.SetRemoteStationManager (manager, "DefaultTxPowerLevel", UintegerValue(maxPower), "RtsCtsThreshold", UintegerValue(rtsThreshold));
  wifiPhy.Set("TxPowerStart", DoubleValue(minPower));
  wifiPhy.Set("TxPowerEnd", DoubleValue(maxPower));
  wifiPhy.Set("TxPowerLevels", UintegerValue(powerLevels));

  ssid = Ssid ("AP0");
  wifiMac.SetType ("ns3::ApWifiMac",
		  	  	  "Ssid", SsidValue (ssid));
  wifiApDevices.Add(wifi.Install (wifiPhy, wifiMac, wifiApNodes.Get(0)));

  ssid = Ssid ("AP1");
  wifiMac.SetType ("ns3::ApWifiMac",
                   "Ssid", SsidValue (ssid),
                   "BeaconInterval", TimeValue(MicroSeconds(103424))); //for avoiding collisions);
  wifiApDevices.Add(wifi.Install (wifiPhy, wifiMac, wifiApNodes.Get(1)));

  wifiDevices.Add(wifiStaDevices);
  wifiDevices.Add(wifiApDevices);

  // Configure the mobility.
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (ap1_x, ap1_y, 0.0));
  positionAlloc->Add (Vector (sta1_x, sta1_y, 0.0));
  positionAlloc->Add (Vector (ap2_x, ap2_y, 0.0));
  positionAlloc->Add (Vector (sta2_x, sta2_y, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNodes.Get(0));
  mobility.Install (wifiStaNodes.Get(0));
  mobility.Install (wifiApNodes.Get(1));
  mobility.Install (wifiStaNodes.Get(1));


  //Configure the IP stack
  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = address.Assign (wifiDevices);
  Ipv4Address sinkAddress = i.GetAddress(0);
  Ipv4Address sinkAddress1 = i.GetAddress(1);
  uint16_t port = 9;

  //Configure the CBR generator
  PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
  ApplicationContainer apps_sink = sink.Install (wifiStaNodes.Get (0));

  OnOffHelper onoff ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
  onoff.SetConstantRate (DataRate ("54Mb/s"), 1420);
  onoff.SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  onoff.SetAttribute ("StopTime", TimeValue (Seconds (100.0)));
  ApplicationContainer apps_source = onoff.Install (wifiApNodes.Get (0));

  PacketSinkHelper sink1 ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress1, port));
  apps_sink.Add(sink1.Install (wifiStaNodes.Get (1)));

  OnOffHelper onoff1 ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress1, port));
  onoff1.SetConstantRate (DataRate ("54Mb/s"), 1420);
  onoff1.SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  onoff1.SetAttribute ("StopTime", TimeValue (Seconds (100.0)));
  apps_source.Add(onoff1.Install (wifiApNodes.Get (1)));

  apps_sink.Start (Seconds (0.5));
  apps_sink.Stop (Seconds (simuTime));

  //------------------------------------------------------------
  //-- Setup stats and data collection
  //--------------------------------------------

  //Statics counters
  APStatics staticsCounterAp0 = APStatics(wifiApDevices, wifiStaDevices);
  APStatics staticsCounterAp1 = APStatics(wifiApDevices, wifiStaDevices);

  //Register packet receptions to calculate throughput
  Config::Connect ("/NodeList/2/ApplicationList/*/$ns3::PacketSink/Rx",
                   MakeCallback (&APStatics::RxCallback, &staticsCounterAp0));
  Config::Connect ("/NodeList/3/ApplicationList/*/$ns3::PacketSink/Rx",
                   MakeCallback (&APStatics::RxCallback, &staticsCounterAp1));

  //Register power and rate changes to calculate the Average Transmit Power
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
                   MakeCallback (&APStatics::PowerCallback, &staticsCounterAp0));
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChange",
                   MakeCallback (&APStatics::RateCallback, &staticsCounterAp0));
  Config::Connect ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
                   MakeCallback (&APStatics::PowerCallback, &staticsCounterAp1));
  Config::Connect ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChange",
                   MakeCallback (&APStatics::RateCallback, &staticsCounterAp1));

  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
                   MakeCallback (&APStatics::PhyCallback, &staticsCounterAp0));
  Config::Connect ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
                   MakeCallback (&APStatics::PhyCallback, &staticsCounterAp1));

  //Register States
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/State/State",
                    MakeCallback (&APStatics::StateCallback, &staticsCounterAp0));
  Config::Connect ("/NodeList/1/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::YansWifiPhy/State/State",
                    MakeCallback (&APStatics::StateCallback, &staticsCounterAp1));

  staticsCounterAp0.CheckStatics(1);
  staticsCounterAp1.CheckStatics(1);

  //Callbacks to print every change of power and rate
  Config::Connect ("/NodeList/[0-1]/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
                    MakeCallback (PowerCallback));
  Config::Connect ("/NodeList/[0-1]/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChange",
                    MakeCallback (RateCallback));


  // Calculate Throughput using Flowmonitor
  //

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  Simulator::Stop (Seconds (simuTime));
  Simulator::Run ();

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      if ((t.sourceAddress=="10.1.1.3" && t.destinationAddress == "10.1.1.1"))
        {
          NS_LOG_INFO("Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n");
          NS_LOG_INFO("  Tx Bytes:   " << i->second.txBytes << "\n");
          NS_LOG_INFO("  Rx Bytes:   " << i->second.rxBytes << "\n");
          NS_LOG_UNCOND("  Throughput: " << i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds())/1024/1024  << " Mbps\n");
          NS_LOG_INFO("  Mean delay:   " << i->second.delaySum.GetSeconds() / i->second.rxPackets << "\n");
          NS_LOG_INFO("  Mean jitter:   " << i->second.jitterSum.GetSeconds() / (i->second.rxPackets - 1) << "\n");
          NS_LOG_INFO("  Tx Opp: " << 1 - (staticsCounterAp0.GetBusyTime() / simuTime));
        }
      if ((t.sourceAddress=="10.1.1.4" && t.destinationAddress == "10.1.1.2"))
        {
          NS_LOG_INFO("Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n");
          NS_LOG_INFO("  Tx Bytes:   " << i->second.txBytes << "\n");
          NS_LOG_INFO("  Rx Bytes:   " << i->second.rxBytes << "\n");
          NS_LOG_UNCOND("  Throughput: " << i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds())/1024/1024  << " Mbps\n");
          NS_LOG_INFO("  Mean delay:   " << i->second.delaySum.GetSeconds() / i->second.rxPackets << "\n");
          NS_LOG_INFO("  Mean jitter:   " << i->second.jitterSum.GetSeconds() / (i->second.rxPackets - 1) << "\n");
          NS_LOG_INFO("  Tx Opp: " << 1 - (staticsCounterAp1.GetBusyTime() / simuTime));
        }
  }

  //Plots for AP0
  std::ofstream outfileTh0 (("throughput-" + outputFileName + "-0.plt").c_str ());
  Gnuplot gnuplot = Gnuplot(("throughput-" + outputFileName + "-0.eps").c_str (), "Throughput");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (staticsCounterAp0.GetDatafile());
  gnuplot.GenerateOutput (outfileTh0);

  std::ofstream outfilePower0 (("power-" + outputFileName + "-0.plt").c_str ());
  gnuplot = Gnuplot(("power-" + outputFileName + "-0.eps").c_str (), "Average Transmit Power");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (staticsCounterAp0.GetPowerDatafile());
  gnuplot.GenerateOutput (outfilePower0);

  std::ofstream outfileTx0 (("tx-" + outputFileName + "-0.plt").c_str ());
  gnuplot = Gnuplot(("tx-" + outputFileName + "-0.eps").c_str (), "Time in TX State");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (staticsCounterAp0.GetTxDatafile());
  gnuplot.GenerateOutput (outfileTx0);

  std::ofstream outfileRx0 (("rx-" + outputFileName + "-0.plt").c_str ());
  gnuplot = Gnuplot(("rx-" + outputFileName + "-0.eps").c_str (), "Time in RX State");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (staticsCounterAp0.GetRxDatafile());
  gnuplot.GenerateOutput (outfileRx0);

  std::ofstream outfileBusy0 (("busy-" + outputFileName + "-0.plt").c_str ());
  gnuplot = Gnuplot(("busy-" + outputFileName + "-0.eps").c_str (), "Time in Busy State");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (staticsCounterAp0.GetBusyDatafile());
  gnuplot.GenerateOutput (outfileBusy0);

  std::ofstream outfileIdle0 (("idle-" + outputFileName + "-0.plt").c_str ());
  gnuplot = Gnuplot(("idle-" + outputFileName + "-0.eps").c_str (), "Time in Idle State");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (staticsCounterAp0.GetIdleDatafile());
  gnuplot.GenerateOutput (outfileIdle0);

  //Plots for AP1
  std::ofstream outfileTh1 (("throughput-" + outputFileName + "-1.plt").c_str ());
  gnuplot = Gnuplot(("throughput-" + outputFileName + "-1.eps").c_str (), "Throughput");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (staticsCounterAp1.GetDatafile());
  gnuplot.GenerateOutput (outfileTh1);

  std::ofstream outfilePower1 (("power-" + outputFileName + "-1.plt").c_str ());
  gnuplot = Gnuplot(("power-" + outputFileName + "-1.eps").c_str (), "Average Transmit Power");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (staticsCounterAp1.GetPowerDatafile());
  gnuplot.GenerateOutput (outfilePower1);

  std::ofstream outfileTx1 (("tx-" + outputFileName + "-1.plt").c_str ());
  gnuplot = Gnuplot(("tx-" + outputFileName + "-1.eps").c_str (), "Time in TX State");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (staticsCounterAp1.GetTxDatafile());
  gnuplot.GenerateOutput (outfileTx1);

  std::ofstream outfileRx1 (("rx-" + outputFileName + "-1.plt").c_str ());
  gnuplot = Gnuplot(("rx-" + outputFileName + "-1.eps").c_str (), "Time in RX State");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (staticsCounterAp1.GetRxDatafile());
  gnuplot.GenerateOutput (outfileRx1);

  std::ofstream outfileBusy1 (("busy-" + outputFileName + "-1.plt").c_str ());
  gnuplot = Gnuplot(("busy-" + outputFileName + "-1.eps").c_str (), "Time in Busy State");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (staticsCounterAp1.GetBusyDatafile());
  gnuplot.GenerateOutput (outfileBusy1);

  std::ofstream outfileIdle1 (("idle-" + outputFileName + "-1.plt").c_str ());
  gnuplot = Gnuplot(("idle-" + outputFileName + "-1.eps").c_str (), "Time in Idle State");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (staticsCounterAp1.GetIdleDatafile());
  gnuplot.GenerateOutput (outfileIdle1);

  Simulator::Destroy ();

  return 0;
}
