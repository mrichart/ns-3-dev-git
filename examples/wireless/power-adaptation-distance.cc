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
* This simulation consist of 2 nodes, one AP and one STA.
* The AP generates UDP traffic with a CBR of 54 Mbps to the STA.
* The AP can use any power and rate control mechanism and the STA use only rate control.
* The STA can be configured to move away from (or towards to) the AP.
*
* The output consists of:
* - A plot of average throughput vs. distance.
* - A plot of average transmit power vs. distance.
* - If enabled, the changes of power and rate to standard output.
*
* Example usage:
* ./waf --run "power-adaptation-distance --manager=ns3::AparfWifiManager --outputFileName=aparf"
*
* Another example (moving towards the AP):
* ./waf --run "power-adaptation-distance --manager=ns3::AparfWifiManager --outputFileName=aparf --stepsSize=-1 --STA1_x=200"
*
* To enable the log of rate and power changes:
* export NS_LOG=PowerAdaptationDistance=level_info
*/

#include <sstream>
#include <fstream>
#include <math.h>

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

NS_LOG_COMPONENT_DEFINE ("PowerAdaptationDistance");

class APStatics
{
public:
  APStatics(NetDeviceContainer aps, NetDeviceContainer stas);

  void CheckStatics (double time);

  void PhyCallback (std::string path, Ptr<const Packet> packet);
  void RxCallback (std::string path, Ptr<const Packet> packet, const Address &from);
  void PowerCallback (std::string path, std::string type, uint8_t power, Mac48Address dest);
  void RateCallback (std::string path, uint32_t rate, Mac48Address dest);
  void SetPosition (Ptr<Node> node, Vector position);
  void AdvancePosition (Ptr<Node> node, int stepsSize, int stepsTime);
  Vector GetPosition (Ptr<Node> node);

  Gnuplot2dDataset GetDatafile();
  Gnuplot2dDataset GetPowerDatafile();

private:
  typedef std::vector<std::pair<Time,WifiMode> > TxTime;
  void SetupPhy (Ptr<WifiPhy> phy);
  Time GetCalcTxTime (WifiMode mode);

  std::map<Mac48Address, uint32_t> actualPower;
  std::map<Mac48Address, WifiMode> actualMode;
  uint32_t m_bytesTotal;
  double totalPower;
  double totalTime;
  Ptr<WifiPhy> myPhy;
  TxTime timeTable;
  Gnuplot2dDataset m_output;
  Gnuplot2dDataset m_output_power;
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
  m_bytesTotal = 0;
  m_output.SetTitle("Throughput Mbits/s");
  m_output_power.SetTitle("Average Transmit Power");
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
APStatics::PowerCallback (std::string path, std::string type, uint8_t power, Mac48Address dest)
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
APStatics::RxCallback (std::string path, Ptr<const Packet> packet, const Address &from)
{
  m_bytesTotal += packet->GetSize();
}

void
APStatics::CheckStatics(double time)
{

}

void
APStatics::SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

Vector
APStatics::GetPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

void
APStatics::AdvancePosition (Ptr<Node> node, int stepsSize, int stepsTime)
{
  Vector pos = GetPosition (node);
  double mbs = ((m_bytesTotal * 8.0) / (1000000*stepsTime));
  m_bytesTotal = 0;
  double atm = pow (10, ((totalPower / stepsTime) / 10));
  totalPower = 0;
  totalTime = 0;
  m_output_power.Add (pos.x, atm);
  //Simulator::Schedule (Seconds (time), &APStatics::CheckStatics, this, time);
  m_output.Add (pos.x, mbs);
  pos.x += stepsSize;
  SetPosition (node, pos);
  //std::cout << "x="<<pos.x << std::endl;
  Simulator::Schedule (Seconds (stepsTime), &APStatics::AdvancePosition, this, node, stepsSize, stepsTime);
}

Gnuplot2dDataset
APStatics::GetDatafile()
{ return m_output; }

Gnuplot2dDataset
APStatics::GetPowerDatafile()
{ return m_output_power; }

void PowerCallback (std::string path, std::string type, uint8_t power, Mac48Address dest) {
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " " << dest << " Power " << type << " " <<  (int)power);
}

void RateCallback (std::string path, uint32_t rate, Mac48Address dest) {
  NS_LOG_INFO ((Simulator::Now ()).GetSeconds () << " " << dest << " Rate " <<  rate);
}

int main (int argc, char *argv[])
{
  double maxPower = 17;
  double minPower = 0;
  uint32_t powerLevels = 18;

  uint32_t rtsThreshold=2346;
  std::string manager="ns3::ParfWifiManager";
  std::string outputFileName="parf";
  int ap1_x = 0;
  int ap1_y = 0;
  int sta1_x = 5;
  int sta1_y = 0;
  int steps = 200;
  int stepsSize = 1;
  int stepsTime = 1;

  CommandLine cmd;
  cmd.AddValue ("manager", "PRC Manager", manager);
  cmd.AddValue ("rtsThreshold", "RTS threshold", rtsThreshold);
  cmd.AddValue ("outputFileName", "Output filename", outputFileName);
  cmd.AddValue ("steps", "How many different distances to try", steps);
  cmd.AddValue ("stepsTime", "Time on each step", stepsTime);
  cmd.AddValue ("stepsSize", "Distance between steps", stepsSize);
  cmd.AddValue ("maxPower", "Maximum available transmission level (dbm).", maxPower);
  cmd.AddValue ("minPower", "Minimum available transmission level (dbm).", minPower);
  cmd.AddValue ("powerLevels", "Number of transmission power levels available between "
          "TxPowerStart and TxPowerEnd included.", powerLevels);
  cmd.AddValue ("AP1_x", "Position of AP1 in x coordinate", ap1_x);
  cmd.AddValue ("AP1_y", "Position of AP1 in y coordinate", ap1_y);
  cmd.AddValue ("STA1_x", "Position of STA1 in x coordinate", sta1_x);
  cmd.AddValue ("STA1_y", "Position of STA1 in y coordinate", sta1_y);
  cmd.Parse (argc, argv);

  int simuTime = steps*stepsTime;

  // Define the APs
  NodeContainer wifiApNodes;
  wifiApNodes.Create (1);

  //Define the STAs
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create(1);

  WifiHelper wifi = WifiHelper::Default ();
  wifi.SetStandard(WIFI_PHY_STANDARD_80211a);
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();

  wifiPhy.SetChannel (wifiChannel.Create ());

  NetDeviceContainer wifiApDevices;
  NetDeviceContainer wifiStaDevices;
  NetDeviceContainer wifiDevices;

  //Configure the STA node
  wifi.SetRemoteStationManager ("ns3::MinstrelWifiManager", "RtsCtsThreshold", UintegerValue(rtsThreshold));
  wifiPhy.Set("TxPowerStart", DoubleValue(maxPower));
  wifiPhy.Set("TxPowerEnd", DoubleValue(maxPower));

  Ssid ssid = Ssid ("AP");
  wifiMac.SetType ("ns3::StaWifiMac",
                  "Ssid", SsidValue (ssid),
                  "ActiveProbing", BooleanValue (false));
  wifiStaDevices.Add(wifi.Install (wifiPhy, wifiMac, wifiStaNodes.Get(0)));

  //Configure the AP node
  wifi.SetRemoteStationManager (manager, "DefaultTxPowerLevel", UintegerValue(maxPower), "RtsCtsThreshold", UintegerValue(rtsThreshold));
  wifiPhy.Set("TxPowerStart", DoubleValue(minPower));
  wifiPhy.Set("TxPowerEnd", DoubleValue(maxPower));
  wifiPhy.Set("TxPowerLevels", UintegerValue(powerLevels));

  ssid = Ssid ("AP");
  wifiMac.SetType ("ns3::ApWifiMac",
                  "Ssid", SsidValue (ssid));
  wifiApDevices.Add(wifi.Install (wifiPhy, wifiMac, wifiApNodes.Get(0)));

  wifiDevices.Add(wifiStaDevices);
  wifiDevices.Add(wifiApDevices);

  // Configure the mobility.
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  //Initial position of AP and STA
  positionAlloc->Add (Vector (ap1_x, ap1_y, 0.0));
  positionAlloc->Add (Vector (sta1_x, sta1_y, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNodes.Get(0));
  mobility.Install (wifiStaNodes.Get(0));

  //Statics counter
  APStatics atpCounter = APStatics(wifiApDevices, wifiStaDevices);

  //Move the STA by stepsSize meters every stepsTime seconds
  Simulator::Schedule (Seconds (0.5+stepsTime), &APStatics::AdvancePosition, &atpCounter, wifiStaNodes.Get (0), stepsSize, stepsTime);

  //Configure the IP stack
  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = address.Assign (wifiDevices);
  Ipv4Address sinkAddress = i.GetAddress(0);
  uint16_t port = 9;

  //Configure the CBR generator
  PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
  ApplicationContainer apps_sink = sink.Install (wifiStaNodes.Get (0));

  OnOffHelper onoff ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
  onoff.SetConstantRate (DataRate ("10Mb/s"), 1420);
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
		  	  	  	MakeCallback (&APStatics::RxCallback, &atpCounter));

  //Register power and rate changes to calculate the Average Transmit Power
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
                    MakeCallback (&APStatics::PowerCallback, &atpCounter));
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChange",
                    MakeCallback (&APStatics::RateCallback, &atpCounter));

  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
                    MakeCallback (&APStatics::PhyCallback, &atpCounter));

  //Callbacks to print every change of power and rate
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/PowerChange",
                    MakeCallback (PowerCallback));
  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$" + manager + "/RateChange",
                    MakeCallback (RateCallback));

  Simulator::Stop (Seconds (simuTime));
  Simulator::Run ();

  std::ofstream outfile (("throughput-" + outputFileName + ".plt").c_str ());
  Gnuplot gnuplot = Gnuplot(("throughput-" + outputFileName + ".eps").c_str (), "Throughput");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (atpCounter.GetDatafile());
  gnuplot.GenerateOutput (outfile);

  std::ofstream outfile2 (("power-" + outputFileName + ".plt").c_str ());
  gnuplot = Gnuplot(("power-" + outputFileName + ".eps").c_str (), "Average Transmit Power");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (atpCounter.GetPowerDatafile());
  gnuplot.GenerateOutput (outfile2);

  Simulator::Destroy ();

  return 0;
}
