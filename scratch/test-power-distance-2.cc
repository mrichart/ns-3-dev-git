/*
* This simulation tests a power control mechanism with 2 links.
* Each link consists of a client connected to an AP which transmits data with a CBR of 54Mb/s
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

NS_LOG_COMPONENT_DEFINE ("PowerExperimentDistance");

static void
SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

static Vector
GetPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

static void
AdvancePosition (Ptr<Node> node, int stepsSize, int stepsTime)
{
  Vector pos = GetPosition (node);
  pos.x += stepsSize;
  SetPosition (node, pos);
  //std::cout << "x="<<pos.x << std::endl;
  Simulator::Schedule (Seconds (stepsTime), &AdvancePosition, node, stepsSize, stepsTime);
}

void
PowerCallback (std::string path, uint8_t power)
{
  NS_LOG_UNCOND ((Simulator::Now ()).GetSeconds () << " " << (int)power);
}

void
RateCallback (std::string path, uint32_t rate)
{
  NS_LOG_UNCOND ((Simulator::Now ()).GetSeconds () << " " << rate);
}

void
TxVectorCallback (std::string path, WifiTxVector vector)
{
  NS_LOG_UNCOND (path << " " << (Simulator::Now ()).GetSeconds () << " " <<  vector.GetMode().GetDataRate()/1000000  << " " << (int)vector.GetTxPowerLevel());
}

class ThroughputCounter
{
public:
  ThroughputCounter();
  ThroughputCounter(Ptr<WifiPhy> phy);

  typedef std::vector<std::pair<Time,WifiMode> > TxTime;

  void SetupPhy (Ptr<WifiPhy> phy);
  Time GetCalcTxTime (WifiMode mode);

  void RxCallback (std::string path, Ptr<const Packet> packet, const Address &from);
  void PhyCallback (std::string path, Ptr<const Packet> packet);
  void PowerCallback (std::string path, uint8_t power);
  void RateCallback (std::string path, uint32_t rate);
  void CheckThroughput (int time);
  void TxVectorCallback (std::string path, WifiTxVector vector);
  void StateCallback (std::string path, Time init, Time duration, enum WifiPhy::State state);
  Gnuplot2dDataset GetDatafile();
  Gnuplot2dDataset GetPowerDatafile();
  Gnuplot2dDataset GetIdleDatafile();
  Gnuplot2dDataset GetBusyDatafile();
  Gnuplot2dDataset GetTxDatafile();
  Gnuplot2dDataset GetRxDatafile();
  double GetBusyTime();

  void CheckTime();

  uint32_t bytesTotal;
  uint32_t actualPower;
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
  double minPower;
  double maxPower;
  Ptr<WifiPhy> myPhy;
  uint32_t powerLevels;
  WifiMode actualMode;
  TxTime timeTable;
  Gnuplot2dDataset m_output;
  Gnuplot2dDataset m_output_power;
  Gnuplot2dDataset m_output_idle;
  Gnuplot2dDataset m_output_busy;
  Gnuplot2dDataset m_output_rx;
  Gnuplot2dDataset m_output_tx;
};

ThroughputCounter::ThroughputCounter(Ptr<WifiPhy> phy) : m_output ("Throughput Mbit/s")
{
  bytesTotal = 0;
  actualPower = 17;
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
  myPhy = phy;
  m_output.SetStyle (Gnuplot2dDataset::LINES);
  m_output_power.SetStyle (Gnuplot2dDataset::LINES);
  m_output_idle.SetStyle (Gnuplot2dDataset::LINES);
  m_output_busy.SetStyle (Gnuplot2dDataset::LINES);
  m_output_rx.SetStyle (Gnuplot2dDataset::LINES);
  m_output_tx.SetStyle (Gnuplot2dDataset::LINES);
  SetupPhy(phy);
}

ThroughputCounter::ThroughputCounter() : m_output ("Throughput Mbit/s")
{
  bytesTotal = 0;
  actualPower = 17;
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
  m_output.SetStyle (Gnuplot2dDataset::LINES);
  m_output_power.SetStyle (Gnuplot2dDataset::LINES);
  m_output_idle.SetStyle (Gnuplot2dDataset::LINES);
  m_output_busy.SetStyle (Gnuplot2dDataset::LINES);
  m_output_rx.SetStyle (Gnuplot2dDataset::LINES);
  m_output_tx.SetStyle (Gnuplot2dDataset::LINES);
}

void
ThroughputCounter::SetupPhy (Ptr<WifiPhy> phy)
{
  uint32_t nModes = phy->GetNModes ();
  for (uint32_t i = 0; i < nModes; i++)
    {
      WifiMode mode = phy->GetMode (i);
      WifiTxVector txVector;
      txVector.SetMode(mode);
      timeTable.push_back (std::make_pair (phy->CalculateTxDuration (1420, txVector, WIFI_PREAMBLE_LONG), mode));
    }
  actualMode = phy->GetMode (3);
}

Time
ThroughputCounter::GetCalcTxTime (WifiMode mode)
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
ThroughputCounter::RxCallback (std::string path, Ptr<const Packet> packet, const Address &from)
{
  bytesTotal += packet->GetSize();
}


void
ThroughputCounter::PhyCallback (std::string path, Ptr<const Packet> packet)
{
  totalPower += actualPower * GetCalcTxTime(actualMode).GetSeconds();
  totalTime += GetCalcTxTime(actualMode).GetSeconds();
  //NS_LOG_UNCOND ((Simulator::Now ()).GetSeconds () << " " << actualMode.GetDataRate()/1000000 << " " << (int)actualPower);
}

void
ThroughputCounter::PowerCallback (std::string path, uint8_t power)
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
  actualPower = dbm;
}

void
ThroughputCounter::RateCallback (std::string path, uint32_t rate)
{
  actualMode = myPhy->GetMode(rate);
}

void
ThroughputCounter::TxVectorCallback (std::string path, WifiTxVector vector)
{
  double   txPowerBaseDbm = myPhy->GetTxPowerStart();
  double   txPowerEndDbm = myPhy->GetTxPowerEnd();
  uint32_t nTxPower = myPhy->GetNTxPower();
  uint8_t power = vector.GetTxPowerLevel();
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
  actualPower = dbm;
  actualMode = vector.GetMode();
}

void
ThroughputCounter::StateCallback (std::string path, Time init, Time duration, enum WifiPhy::State state) {
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
ThroughputCounter::CheckThroughput(int time)
{
  double mbs = ((bytesTotal * 8.0) / (time*1000000));
  double atm = pow (10, ((totalPower / time) / 10));
  totalPower = 0;
  totalTime = 0;
  bytesTotal = 0;
  m_output.Add ((Simulator::Now ()).GetSeconds () - 20, mbs);
  m_output_power.Add ((Simulator::Now ()).GetSeconds () - 20, atm);
  Simulator::Schedule (Seconds (time), &ThroughputCounter::CheckThroughput, this, time);
}

void
ThroughputCounter::CheckTime()
{
  m_output_idle.Add ((Simulator::Now ()).GetSeconds (), idleTime);
  m_output_busy.Add ((Simulator::Now ()).GetSeconds (), busyTime);
  m_output_tx.Add ((Simulator::Now ()).GetSeconds (), txTime);
  m_output_rx.Add ((Simulator::Now ()).GetSeconds (), rxTime);
  busyTime = 0;
  idleTime = 0;
  txTime = 0;
  rxTime = 0;
  Simulator::Schedule (Seconds (0.1), &ThroughputCounter::CheckTime, this);
}

Gnuplot2dDataset
ThroughputCounter::GetDatafile()
{ return m_output; }

Gnuplot2dDataset
ThroughputCounter::GetPowerDatafile()
{ return m_output_power; }

Gnuplot2dDataset
ThroughputCounter::GetIdleDatafile()
{ return m_output_idle; }

Gnuplot2dDataset
ThroughputCounter::GetBusyDatafile()
{ return m_output_busy; }

Gnuplot2dDataset
ThroughputCounter::GetRxDatafile()
{ return m_output_rx; }

Gnuplot2dDataset
ThroughputCounter::GetTxDatafile()
{ return m_output_tx; }

double
ThroughputCounter::GetBusyTime()
{ return totalBusyTime+totalRxTime; }


int main (int argc, char *argv[])
{
  //LogComponentEnable("DcaTxop", LOG_LEVEL_DEBUG);
  //LogComponentEnable("MacLow", LOG_LEVEL_DEBUG);

  double maxPower = 17;
  double minPower = 0;
  uint32_t powerLevels = 18;

  int ap1_x = 0;
  int ap1_y = 0;
  int sta1_x = 0;
  int sta1_y = 0;
  int manager = 0;
  int steps = 20;
  int stepsSize = 10;
  int stepsTime = 10;

  CommandLine cmd;
  cmd.AddValue ("manager", "PRC Manager: 0-PARF, 1-APARF, 2-Minstrel-Piano, 3-Rule-Based 4-Aarf(fixed max power)", manager);
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

  int simuTime = steps*stepsTime + 2*stepsTime;

  // Define the APs
  NodeContainer wifiApNodes;
  wifiApNodes.Create (1);

  //Define the STAs
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create(1);

  WifiHelper wifi = WifiHelper::Default ();

  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();

  //Create a channel helper in a default working state. By default, we create a channel model with a propagation delay equal to a constant, the speed of light,
  // and a propagation loss based on a log distance model with a reference loss of 46.6777 dB at reference distance of 1m
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());

  wifi.SetStandard(WIFI_PHY_STANDARD_80211g);

  NetDeviceContainer wifiApDevices;
  NetDeviceContainer wifiStaDevices;
  NetDeviceContainer wifiDevices;

  //wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",StringValue ("ErpOfdmRate12Mbps"));
  wifi.SetRemoteStationManager ("ns3::AarfWifiManager");
  wifiPhy.Set("TxPowerStart", DoubleValue(maxPower));
  wifiPhy.Set("TxPowerEnd", DoubleValue(maxPower));

  Ssid ssid = Ssid ("AP0");
  wifiMac.SetType ("ns3::StaWifiMac",
                  "Ssid", SsidValue (ssid),
                  "ActiveProbing", BooleanValue (false));
  wifiStaDevices.Add(wifi.Install (wifiPhy, wifiMac, wifiStaNodes.Get(0)));


  if (manager == 0)
    {
      wifi.SetRemoteStationManager ("ns3::ParfWifiManager", "DefaultTxPowerLevel", UintegerValue(maxPower));
    }
  else if (manager == 1)
    {
      wifi.SetRemoteStationManager ("ns3::AparfWifiManager", "DefaultTxPowerLevel", UintegerValue(maxPower));
    }
  else if (manager == 2)
    {
      wifi.SetRemoteStationManager ("ns3::MinstrelPianoWifiManager", "DefaultTxPowerLevel", UintegerValue(maxPower));
    }
  else if (manager == 3)
    {
	  wifi.SetRemoteStationManager ("ns3::AarfWifiManager");
	  minPower = 17;
	  powerLevels = 1;
    }
  else if (manager == 4)
    {
	  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",StringValue ("ErpOfdmRate6Mbps"));
	  minPower = 17;
	  powerLevels = 1;
    }
  else
    {

    }

  wifiPhy.Set("TxPowerStart", DoubleValue(minPower));
  wifiPhy.Set("TxPowerEnd", DoubleValue(maxPower));
  wifiPhy.Set("TxPowerLevels", UintegerValue(powerLevels));

  ssid = Ssid ("AP0");
  wifiMac.SetType ("ns3::ApWifiMac",
                  "Ssid", SsidValue (ssid));
  wifiApDevices.Add(wifi.Install (wifiPhy, wifiMac, wifiApNodes.Get(0)));

  wifiDevices.Add(wifiStaDevices);
  wifiDevices.Add(wifiApDevices);

  // mobility.
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (ap1_x, ap1_y, 0.0));
  positionAlloc->Add (Vector (sta1_x, sta1_y, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNodes.Get(0));
  mobility.Install (wifiStaNodes.Get(0));

  Simulator::Schedule (Seconds (stepsTime*2), &AdvancePosition, wifiStaNodes.Get (0), stepsSize, stepsTime);

  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);

  Ipv4AddressHelper address;

  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = address.Assign (wifiDevices);

  Ipv4Address sinkAddress = i.GetAddress(0);
  uint16_t port = 9;

  PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
  ApplicationContainer apps_sink = sink.Install (wifiStaNodes.Get (0));

  OnOffHelper onoff ("ns3::UdpSocketFactory", InetSocketAddress (sinkAddress, port));
  onoff.SetConstantRate (DataRate ("54Mb/s"), 1420);
  onoff.SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  onoff.SetAttribute ("StopTime", TimeValue (Seconds (simuTime)));
  ApplicationContainer apps_source = onoff.Install (wifiApNodes.Get (0));

  apps_sink.Start (Seconds (0.0));
  apps_sink.Stop (Seconds (simuTime));

    //------------------------------------------------------------
    //-- Setup stats and data collection
    //--------------------------------------------

  ThroughputCounter* throughputCounter = new ThroughputCounter();

  Config::Connect ("/NodeList/1/ApplicationList/*/$ns3::PacketSink/Rx",
    				MakeCallback (&ThroughputCounter::RxCallback, throughputCounter));

  throughputCounter->CheckThroughput(stepsTime);

  Ptr<NetDevice> device = wifiApDevices.Get(0);
  Ptr<WifiNetDevice> wifiDevice = DynamicCast<WifiNetDevice> (device);
  Ptr<WifiPhy> phy = wifiDevice->GetPhy();

  ThroughputCounter* atmCounter = new ThroughputCounter(phy);

  if (manager == 0)
    {
      Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$ns3::ParfWifiManager/PowerChange",
                                    MakeCallback (&ThroughputCounter::PowerCallback, atmCounter));
      Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$ns3::ParfWifiManager/RateChange",
                                    MakeCallback (&ThroughputCounter::RateCallback, atmCounter));
    }
  else if (manager == 1)
    {
      Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$ns3::AparfWifiManager/PowerChange",
                                    MakeCallback (&ThroughputCounter::PowerCallback, atmCounter));
      Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$ns3::AparfWifiManager/RateChange",
                                    MakeCallback (&ThroughputCounter::RateCallback, atmCounter));
    }
  else if (manager == 2)
    {
      Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$ns3::MinstrelPianoWifiManager/PowerChange",
                                    MakeCallback (&ThroughputCounter::PowerCallback, atmCounter));
      Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$ns3::MinstrelPianoWifiManager/RateChange",
                                    MakeCallback (&ThroughputCounter::RateCallback, atmCounter));
    }
  else if (manager == 3)
    {
      Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$ns3::AarfWifiManager/RateChange",
                                    MakeCallback (&ThroughputCounter::RateCallback, atmCounter));
    }
  else
    {
      Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$ns3::ConstantRateWifiManager/RateChange",
                                    MakeCallback (&ThroughputCounter::RateCallback, atmCounter));
    }

  Config::Connect ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Phy/PhyTxBegin",
                    MakeCallback (&ThroughputCounter::PhyCallback, atmCounter));

  atmCounter->CheckThroughput(stepsTime);

  Simulator::Stop (Seconds (simuTime));
  Simulator::Run ();

  stringstream name;
  name << "throughput-" << manager << "-" << RngSeedManager::GetRun () << "-" << sta1_x << ".txt";
  std::ofstream outfile (name.str().c_str());
  Gnuplot gnuplot = Gnuplot("th-link1.eps", "Throughput");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (throughputCounter->GetDatafile());
  gnuplot.GenerateOutput (outfile);

  stringstream name2;
  name2 << "power-" << manager << "-" << RngSeedManager::GetRun () << "-" << sta1_x <<  ".txt";
  std::ofstream outfilePower (name2.str().c_str());
  gnuplot = Gnuplot("pow-link1.eps", "Average Transmit Power");
  gnuplot.SetTerminal("post eps color enhanced");
  gnuplot.AddDataset (atmCounter->GetPowerDatafile());
  gnuplot.GenerateOutput (outfilePower);

  Simulator::Destroy ();

  return 0;
}
