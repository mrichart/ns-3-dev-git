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
 * Author: Duy Nguyen <duy@soe.ucsc.edu>
 */

/**
 * Scenarios: 100 nodes, multiple simultaneous flows, multi-hop ad hoc, routing,
 * and mobility
 *
 * QUICK INSTRUCTIONS:
 *
 * To optimize build: 
 * ./waf -d optimized configure
 * ./waf
 *
 * To compile:
 * ./waf --run multirate
 *
 * To compile with command line(useful for varying parameters):
 * ./waf --run "multirate --totalTime=0.3s --rateManager=ns3::MinstrelWifiManager"
 *
 * To turn on NS_LOG:
 * export NS_LOG=multirate=level_all
 * (can only view log if built with ./waf -d debug configure)
 *
 * To debug:
 * ./waf --shell
 * gdb ./build/debug/examples/wireless/multirate
 *
 * To view pcap files:
 * tcpdump -nn -tt -r filename.pcap
 *
 * To monitor the files:
 * tail -f filename.pcap
 *
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/stats-module.h"

#include <iostream>
#include <fstream>
#include <string>

NS_LOG_COMPONENT_DEFINE ("multirate");

using namespace ns3;

class Experiment
{
public:

  Experiment ();
  Experiment (std::string name);
  Gnuplot2dDataset Run (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy,
                        const NqosWifiMacHelper &wifiMac, const YansWifiChannelHelper &wifiChannel, const MobilityHelper &mobility);

  bool CommandSetup (int argc, char **argv);
  bool IsRouting () { return (enableRouting == 1) ? 1:0; }
  bool IsMobility () { return (enableMobility == 1) ? 1:0; }

  uint32_t GetScenario () {return scenario; }
  uint32_t GetStandard() { return ieeeStandard; }
  uint32_t GetPacketSize () { return packetSize; }
  uint32_t GetGridSize () { return gridSize; } 
  uint32_t GetTransmitPower () { return transmitPower; }

  double GetNodeDistance () { return nodeDistance; } 
  uint32_t GetTotalTime() { return totalTime; } 

  std::string GetRtsThreshold () { return rtsThreshold; }
  std::string GetOutputFileName () { return outputFileName; }
  std::string GetRateManager () { return rateManager; }
  std::string GetDataMode () { return dataMode; }
  std::string GetPropLoss() { return propLoss; }
  std::string GetPropDelay() { return propDelay; }
  
private:

  Vector GetPosition (Ptr<Node> node);
  Ptr<Socket> SetupPacketReceive (Ptr<Node> node);
  NodeContainer GenerateNeighbors(NodeContainer c, uint32_t senderId);
  NodeContainer GenerateNeighbors2(NodeContainer c, uint32_t senderId);

  void ApplicationSetup (Ptr<Node> client, Ptr<Node> server, double start, double stop);
  void AssignNeighbors (NodeContainer c);
  void SelectSrcDest (NodeContainer c);
  void ReceivePacket (Ptr<Socket> socket);
  void CheckThroughput ();
  void SendMultiDestinations (Ptr<Node> sender, NodeContainer c);
   void SetPosition (Ptr<Node> node, Vector position);

  Gnuplot2dDataset m_output;

  double totalTime; 
  double expMean;

  uint32_t bytesTotal;
  uint32_t packetSize;
  uint32_t gridSize; 
  uint32_t transmitPower;

  double nodeDistance;
  uint32_t port;
  uint32_t scenario;
  uint32_t ieeeStandard;

  bool enablePcap;
  bool enableTracing;
  bool enableFlowMon;
  bool enableRouting;
  bool enableMobility;

  NodeContainer containerA, containerB, containerC, containerD; 
  std::string rtsThreshold, rateManager, dataMode, outputFileName, propLoss, propDelay;
};

Experiment::Experiment ()
{}

Experiment::Experiment (std::string name) : 
  m_output (name),
  totalTime (30), 
  expMean (3), //flows being exponentially distributed
  bytesTotal(0),
  packetSize (256), //try 256 512 1024 2048 bytes
  gridSize (10), //10x10 grid  for a total of 100 nodes
  transmitPower (19), //10x10 grid  for a total of 100 nodes
  nodeDistance (60),
  port (5000),
  scenario (2), 
  ieeeStandard(0), // 0 = 11a; 1 = 11b; 4 = holland  
  enablePcap (false), 
  enableTracing (false),
  enableFlowMon (false),
  enableRouting (false),
  enableMobility (false),
  rtsThreshold ("2200"), //0 for enabling rts/cts
  rateManager ("ns3::ConstantRateWifiManager"),
  dataMode ("OfdmRate18Mbps"),
  outputFileName ("constant"),
  propLoss("ns3::LogDistancePropagationLossModel"),
  //propLoss("ns3::RandomPropagationLossModel"),
  propDelay("ns3::ConstantSpeedPropagationDelayModel")
{
  m_output.SetStyle (Gnuplot2dDataset::LINES);
}

void
Experiment::SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}


Ptr<Socket>
Experiment::SetupPacketReceive (Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), port);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&Experiment::ReceivePacket, this));

  return sink;
}

void
Experiment::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while (packet = socket->Recv ())
  {
    bytesTotal += packet->GetSize();
  }
}

void
Experiment::CheckThroughput()
{
  double mbs = ((bytesTotal * 8.0) /1000000);
  bytesTotal = 0;
  m_output.Add ((Simulator::Now ()).GetSeconds (), mbs);

  //check throughput every 1/10 of a second 
  Simulator::Schedule (Seconds (1), &Experiment::CheckThroughput, this);
}

Vector
Experiment::GetPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

/**
 *
 * Take the grid map, divide it into 4 quadrants
 * Assign all nodes from each quadrant to a specific container 
 * 
 */
void
Experiment::AssignNeighbors (NodeContainer c)
{
  uint32_t totalNodes = c.GetN ();
  for (uint32_t i=0; i< totalNodes; i++)
    {
      if ( (i % gridSize) <= (gridSize/2 - 1))
        {
          //lower left quadrant
	  if ( i < totalNodes/2 )
	    {
	      containerA.Add(c.Get(i));
            }
      
          //upper left quadrant
          if ( i >= (uint32_t)(4*totalNodes)/10 )
	    {
	      containerC.Add(c.Get(i));  
            }
	}
      if ( (i % gridSize) >= (gridSize/2 - 1))
        {
          //lower right quadrant
	  if ( i < totalNodes/2 )
	    {
	      containerB.Add(c.Get(i));  
            }

          //upper right quadrant
          if ( i >= (uint32_t)(4*totalNodes)/10  )
	    {
	      containerD.Add(c.Get(i));  
            }
	}
    }
}

/**
 * Generate 1-hop
 *
 */
NodeContainer
Experiment::GenerateNeighbors2 (NodeContainer c, uint32_t sid)
{
  NodeContainer nc;

  nc.Add(c.Get(sid));
  nc.Add(c.Get(sid + 1));
  nc.Add(c.Get(sid- 1));
  nc.Add(c.Get(sid + 10));
  nc.Add(c.Get(sid + 10 + 1));
  nc.Add(c.Get(sid + 10 - 1));
  nc.Add(c.Get(sid - 10));
  nc.Add(c.Get(sid - 10 + 1));
  nc.Add(c.Get(sid - 10 - 1));


  return nc;
}


/**
 * Generate 1-hop and 2-hop neighbors of a node in grid topology
 *
 */
NodeContainer
Experiment::GenerateNeighbors (NodeContainer c, uint32_t senderId)
{
  NodeContainer nc;
  uint32_t limit = senderId + 2;  
  for (uint32_t i= senderId - 2; i <= limit; i++)
    {
      //must ensure the boundaries for other topologies
      nc.Add(c.Get(i));
      nc.Add(c.Get(i + 10));
      nc.Add(c.Get(i + 20));
      nc.Add(c.Get(i - 10));
      nc.Add(c.Get(i - 20));
    }
  return nc;
}

/**
 * Sources and destinations are randomly selected such that a node 
 * may be the source for multiple destinations and a node maybe a destination 
 * for multiple sources. 
 */
void
Experiment::SelectSrcDest (NodeContainer c)
{
  uint32_t totalNodes = c.GetN();
  Ptr<UniformRandomVariable> uvSrc = CreateObject<UniformRandomVariable> ();
  uvSrc->SetAttribute ("Min", DoubleValue (0.0));
  uvSrc->SetAttribute ("Max", DoubleValue (totalNodes/2 -1));
  Ptr<UniformRandomVariable> uvDest = CreateObject<UniformRandomVariable> ();
  uvDest->SetAttribute ("Min", DoubleValue (totalNodes/2));
  uvDest->SetAttribute ("Max", DoubleValue (totalNodes));

  for (uint32_t i=0; i < totalNodes/3; i++)
    {
      ApplicationSetup (c.Get(uvSrc->GetInteger()), c.Get(uvDest->GetInteger()) ,  0, totalTime);
    }
}

/**
 *
 * A sender node will  set up a flow to each of the its neighbors
 * in its quadrant randomly.  All the flows are exponentially distributed
 *
 */
void
Experiment::SendMultiDestinations(Ptr<Node> sender, NodeContainer c)
{

  // UniformVariable params: (Xrange, Yrange)

  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
  uv->SetAttribute ("Min", DoubleValue (0.0));
  uv->SetAttribute ("Max", DoubleValue (c.GetN ()));

  // ExponentialVariable params: (mean, upperbound)
  Ptr<ExponentialRandomVariable> ev = CreateObject<ExponentialRandomVariable> ();
  ev->SetAttribute ("Mean", DoubleValue (expMean));
  ev->SetAttribute ("Bound", DoubleValue (totalTime));

  double start=0.0, stop=totalTime;
  uint32_t destIndex; 

  for (uint32_t i=0; i < c.GetN (); i++)
    {
      stop = start + ev->GetValue();
      NS_LOG_DEBUG("Start=" << start << " Stop=" << stop);

      do {
          destIndex = (uint32_t) uv->GetValue();
      } while ( (c.Get(destIndex))->GetId () == sender->GetId ());
      
      ApplicationSetup (sender, c.Get(destIndex) ,  start, stop);

      start = stop;

      if(start > totalTime) 
        {
          break;
        }
    }
}

void
Experiment::ApplicationSetup (Ptr<Node> client, Ptr<Node> server, double start, double stop)
{

  Vector serverPos = GetPosition (server);
  Vector clientPos = GetPosition (client);


  Ptr<Ipv4> ipv4Server = server->GetObject<Ipv4>();
  Ptr<Ipv4> ipv4Client = client->GetObject<Ipv4>();

  Ipv4InterfaceAddress iaddrServer = ipv4Server->GetAddress(1,0);
  Ipv4InterfaceAddress iaddrClient = ipv4Client->GetAddress(1,0);

  Ipv4Address ipv4AddrServer = iaddrServer.GetLocal ();
  Ipv4Address ipv4AddrClient = iaddrClient.GetLocal ();


  //NS_LOG_DEBUG("Set up Client Device " <<  (client->GetDevice(0))->GetAddress () 
  std::cout << "Set up Client Device " <<  (client->GetDevice(0))->GetAddress () 
            << " with ip " << ipv4AddrClient 
            << " position (" << clientPos.x << "," << clientPos.y << "," << clientPos.z << ")"
            << std::endl; 

  //NS_LOG_DEBUG("Set up Server Device " <<  (server->GetDevice(0))->GetAddress () 
  std::cout << "Set up Server Device " <<  (server->GetDevice(0))->GetAddress () 
            << " with ip " << ipv4AddrServer 
            << " position (" << serverPos.x << "," << serverPos.y << "," << serverPos.z << ")" <<std::endl;

   
  // Equipping the source  node with OnOff Application used for sending 
  OnOffHelper onoff ("ns3::UdpSocketFactory", Address(InetSocketAddress(Ipv4Address("10.0.0.1"), port)));
  onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
  onoff.SetAttribute ("DataRate", DataRateValue (DataRate (60000000)));
  onoff.SetAttribute ("PacketSize", UintegerValue (packetSize));
  onoff.SetAttribute ("Remote", AddressValue(InetSocketAddress (ipv4AddrServer, port)));

  ApplicationContainer apps = onoff.Install (client);
  apps.Start (Seconds (start));
  apps.Stop (Seconds (stop));

  Ptr<Socket> sink = SetupPacketReceive (server);

}

Gnuplot2dDataset
Experiment::Run (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy,
                 const NqosWifiMacHelper &wifiMac, const YansWifiChannelHelper &wifiChannel, const MobilityHelper &mobility)
{


  uint32_t nodeSize = gridSize*gridSize;

  if( nodeSize == 1)  nodeSize = 2;

  NodeContainer c;
  c.Create (nodeSize);

  NodeContainer d;
  d.Create (16);

  OlsrHelper olsr;
  Ipv4StaticRoutingHelper staticRouting;

  Ipv4ListRoutingHelper list;
  
  if (enableRouting)
    {
      list.Add (staticRouting, 0);
      list.Add (olsr, 10);
    }

  InternetStackHelper internet;

  if (enableRouting)
    {
      internet.SetRoutingHelper(list);
    }
  internet.Install (c);
  internet.Install (d);

  YansWifiPhyHelper phy = wifiPhy;
  phy.SetChannel (wifiChannel.Create ());

  NqosWifiMacHelper mac = wifiMac;
  NetDeviceContainer devices = wifi.Install (phy, mac, c);

  NetDeviceContainer devices2 = wifi.Install (phy, mac, d);






  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.255.255.0");

  Ipv4InterfaceContainer ipInterfaces;
  ipInterfaces = address.Assign(devices);

  ipInterfaces = address.Assign(devices2);
  
  
  MobilityHelper mobil= mobility;

  if (enableMobility == false) {

  mobil.SetPositionAllocator ("ns3::GridPositionAllocator",
                                "MinX", DoubleValue (0.0),
                                "MinY", DoubleValue (0.0),
                                "DeltaX", DoubleValue (nodeDistance),
                                "DeltaY", DoubleValue (nodeDistance),
                                "GridWidth", UintegerValue (gridSize),
                                "LayoutType", StringValue ("RowFirst"));

  mobil.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  
  }
  else {
     //random placement

     mobil.SetPositionAllocator ("ns3::RandomDiscPositionAllocator",
                                  "X", StringValue ("100.0"),
                                  "Y", StringValue ("100.0"),
                                  "Rho", StringValue ("Uniform:0:1000"));

  }



  MobilityHelper mobil2 = mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();

  //grid distance 90
  positionAlloc->Add (Vector (20.0, 0.0, 0.0));
  positionAlloc->Add (Vector (290.0, 0.0, 0.0));
  positionAlloc->Add (Vector (560.0, 0.0, 0.0));
  positionAlloc->Add (Vector (830.0, 0.0, 0.0));

  positionAlloc->Add (Vector (20.0, 270.0, 0.0));
  positionAlloc->Add (Vector (290.0, 270.0, 0.0));
  positionAlloc->Add (Vector (560.0, 270.0, 0.0));
  positionAlloc->Add (Vector (830.0, 270.0, 0.0));

  positionAlloc->Add (Vector (20.0, 540.0, 0.0));
  positionAlloc->Add (Vector (290.0, 540.0, 0.0));
  positionAlloc->Add (Vector (560.0, 540.0, 0.0));
  positionAlloc->Add (Vector (830.0, 540.0, 0.0));

  positionAlloc->Add (Vector (20.0, 810.0, 0.0));
  positionAlloc->Add (Vector (290.0, 810.0, 0.0));
  positionAlloc->Add (Vector (560.0, 810.0, 0.0));
  positionAlloc->Add (Vector (830.0, 810.0, 0.0));



  mobil2.SetPositionAllocator (positionAlloc);
  mobil2.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobil2.Install (d);



  if (enableMobility && enableRouting)
    {
      //Rectangle (xMin, xMax, yMin, yMax)
      mobil.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
                              "Bounds", RectangleValue (Rectangle (0, 1000, 0, 1000)),
                              "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=10]"),
                              "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.2]"));
    }
  mobil.Install (c);


//    NS_LOG_INFO ("Enabling global routing on all nodes");
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  if ( scenario == 1 )
    {
      SelectSrcDest(c);
    }
  else if ( scenario == 2)
    {
         
      //All flows begin at the same time
      for (uint32_t i = 0; i < nodeSize - 1; i = i+2)
        {
          ApplicationSetup (c.Get (i), c.Get (i+1),  0, totalTime);
        }
    }
  else if ( scenario == 3)
    {
      AssignNeighbors(c);



// n10  n11  n12  n13  n14 n15 n16 n17 n18 n19
// n40  n41  n42  n43  n44 n45 n46 n47 n48 n49
// n30  n31  n32  n33  n44 n35 n36 n37 n38 n39
// n20  n21  n22  n23  n24 n25 n26 n27 n28 n29
// n10  n11  n12  n13  n14 n15 n16 n17 n18 n19
// n0   n1   n2   n3   n4  n5  n6  n7  n8  n9



      //Note: these senders are hand-picked in order to ensure good coverage
      //for 10x10 grid, basically one sender for each quadrant
      //you might have to change these values for other grids 
      NS_LOG_DEBUG(">>>>>>>>>region A<<<<<<<<<");
      SendMultiDestinations(c.Get(22), containerA);

      NS_LOG_DEBUG(">>>>>>>>>region B<<<<<<<<<");
      SendMultiDestinations(c.Get(26), containerB);

      NS_LOG_DEBUG(">>>>>>>>>region C<<<<<<<<<");
      SendMultiDestinations(c.Get(72), containerC);

      NS_LOG_DEBUG(">>>>>>>>>region D<<<<<<<<<");
      SendMultiDestinations(c.Get(76), containerD);
    }
  else if ( scenario == 4)
    {


// n90  n91  n92  n93  n94 n95 n96 n97 n98 n99
// n80  n81  n82  n83  n84 n85 n86 n87 n88 n89
// n70  n71  n72  n73  n74 n75 n76 n77 n78 n79
// n60  n61  n62  n63  n64 n65 n66 n67 n68 n69
// n50  n51  n52  n53  n54 n55 n56 n57 n58 n59
// n40  n41  n42  n43  n44 n45 n46 n47 n48 n49
// n30  n31  n32  n33  n44 n35 n36 n37 n38 n39
// n20  n21  n22  n23  n24 n25 n26 n27 n28 n29
// n10  n11  n12  n13  n14 n15 n16 n17 n18 n19
// n0   n1   n2   n3   n4  n5  n6  n7  n8  n9

      //GenerateNeighbors(NodeContainer, uint32_t sender)
      //Note: these senders are hand-picked in order to ensure good coverage
      //you might have to change these values for other grids 
      NodeContainer c11, c13, c15, c17;
      NodeContainer c31, c33, c35, c37;
      NodeContainer c51, c53, c55, c57;
      NodeContainer c71, c73, c75, c77;

      c11 = GenerateNeighbors2(c, 11);
      c13 = GenerateNeighbors2(c, 13);;
      c15 = GenerateNeighbors2(c, 15);;
      c17 = GenerateNeighbors2(c, 17);;
      c31 = GenerateNeighbors2(c, 31);
      c33 = GenerateNeighbors2(c, 33);;
      c35 = GenerateNeighbors2(c, 35);;
      c37 = GenerateNeighbors2(c, 37);;
      c51 = GenerateNeighbors2(c, 51);
      c53 = GenerateNeighbors2(c, 53);;
      c55 = GenerateNeighbors2(c, 55);;
      c57 = GenerateNeighbors2(c, 57);;
      c71 = GenerateNeighbors2(c, 71);
      c73 = GenerateNeighbors2(c, 73);;
      c75 = GenerateNeighbors2(c, 75);;
      c77 = GenerateNeighbors2(c, 77);;

      SendMultiDestinations(c.Get(11), c11);
      SendMultiDestinations(c.Get(13), c13);
      SendMultiDestinations(c.Get(15), c15);
      SendMultiDestinations(c.Get(17), c17);

      SendMultiDestinations(c.Get(31), c31);
      SendMultiDestinations(c.Get(33), c33);
      SendMultiDestinations(c.Get(35), c35);
      SendMultiDestinations(c.Get(37), c37);

      SendMultiDestinations(c.Get(51), c51);
      SendMultiDestinations(c.Get(53), c53);
      SendMultiDestinations(c.Get(55), c55);
      SendMultiDestinations(c.Get(57), c57);

      SendMultiDestinations(c.Get(71), c71);
      SendMultiDestinations(c.Get(73), c73);
      SendMultiDestinations(c.Get(75), c75);
      SendMultiDestinations(c.Get(77), c77);

    }
  else if ( scenario == 5)
    {

	//need gridsize of 10x10 and distance 100


 	ApplicationSetup (c.Get (0), d.Get (0),  0, totalTime);
	ApplicationSetup (c.Get (3), d.Get (1),  0, totalTime);
 	ApplicationSetup (c.Get (6), d.Get (2),  0, totalTime);
 	ApplicationSetup (c.Get (9), d.Get (3),  0, totalTime);


 	ApplicationSetup (c.Get (30), d.Get (4),  0, totalTime);
 	ApplicationSetup (c.Get (33), d.Get (5),  0, totalTime);
	ApplicationSetup (c.Get (36), d.Get (6),  0, totalTime);
	ApplicationSetup (c.Get (39), d.Get (7),  0, totalTime);


 	ApplicationSetup (c.Get (60), d.Get (8),  0, totalTime);
 	ApplicationSetup (c.Get (63), d.Get (9),  0, totalTime);
 	ApplicationSetup (c.Get (66), d.Get (10),  0, totalTime);
 	ApplicationSetup (c.Get (69), d.Get (11),  0, totalTime);


 	ApplicationSetup (c.Get (90), d.Get (12),  0, totalTime);
 	ApplicationSetup (c.Get (93), d.Get (13),  0, totalTime);
 	ApplicationSetup (c.Get (96), d.Get (14),  0, totalTime);
 	ApplicationSetup (c.Get (99), d.Get (15),  0, totalTime);




    }
    else if ( scenario == 6) {
// n90  n91  n92  n93  n94 n95 n96 n97 n98 n99
// n80  n81  n82  n83  n84 n85 n86 n87 n88 n89
// n70  n71  n72  n73  n74 n75 n76 n77 n78 n79
// n60  n61  n62  n63  n64 n65 n66 n67 n68 n69
// n50  n51  n52  n53  n54 n55 n56 n57 n58 n59
// n40  n41  n42  n43  n44 n45 n46 n47 n48 n49
// n30  n31  n32  n33  n44 n35 n36 n37 n38 n39
// n20  n21  n22  n23  n24 n25 n26 n27 n28 n29
// n10  n11  n12  n13  n14 n15 n16 n17 n18 n19
// n0   n1   n2   n3   n4  n5  n6  n7  n8  n9
 

	//GenerateNeighbors(NodeContainer, uint32_t sender)
      //Note: these senders are hand-picked in order to ensure good coverage
      //you might have to change these values for other grids 
      NodeContainer c1, c2, c3, c4, c5, c6, c7, c8, c9;

      c1 = GenerateNeighbors(c, 22);
      c2 = GenerateNeighbors(c, 24);;
      c3 = GenerateNeighbors(c, 26);;
      c4 = GenerateNeighbors(c, 42);;
      c5 = GenerateNeighbors(c, 44);;
      c6 = GenerateNeighbors(c, 46);;
      c7 = GenerateNeighbors(c, 62);;
      c8 = GenerateNeighbors(c, 64);;
      c9 = GenerateNeighbors(c, 66);;

      SendMultiDestinations(c.Get(22), c1);
      SendMultiDestinations(c.Get(24), c2);
      SendMultiDestinations(c.Get(26), c3);
      SendMultiDestinations(c.Get(42), c4);
      SendMultiDestinations(c.Get(44), c5);
      SendMultiDestinations(c.Get(46), c6);
      SendMultiDestinations(c.Get(62), c7);
      SendMultiDestinations(c.Get(64), c8);
      SendMultiDestinations(c.Get(66), c9);

   }
  

  CheckThroughput ();

  if (enablePcap)
    {
      phy.EnablePcapAll(GetOutputFileName());
    }

  if (enableTracing)
    {
      AsciiTraceHelper ascii;
      phy.EnableAsciiAll (ascii.CreateFileStream (GetOutputFileName() + ".tr"));
    }

  Ptr<FlowMonitor> flowmon;

  if (enableFlowMon)
    {
      FlowMonitorHelper flowmonHelper;
      flowmon = flowmonHelper.InstallAll ();
    }

  Simulator::Stop (Seconds (totalTime));
  Simulator::Run ();

  if (enableFlowMon)
    {
      flowmon->SerializeToXmlFile ((GetOutputFileName() + ".flomon"), false, false);
    }

  Simulator::Destroy ();
  
  return m_output;
}

bool
Experiment::CommandSetup (int argc, char **argv)
{
  // for commandline input
  CommandLine cmd;

  cmd.AddValue ("packetSize", "packet size", packetSize);
  cmd.AddValue ("gridSize", "grid Size", gridSize);
  cmd.AddValue ("transmitPower", "transmitPower", transmitPower);
  cmd.AddValue ("nodeDistance", "node distance", nodeDistance);
  cmd.AddValue ("totalTime", "simulation time", totalTime);
  cmd.AddValue ("rtsThreshold", "rts threshold", rtsThreshold);
  cmd.AddValue ("rateManager", "type of rate", rateManager);
  cmd.AddValue ("dataMode", "type of rate", dataMode);
  cmd.AddValue ("ieeeStandard", "type of standards", ieeeStandard);
  cmd.AddValue ("propLoss", "propagation loss", propLoss);
  cmd.AddValue ("propDelay", "propagation delay", propDelay);
  cmd.AddValue ("outputFileName", "output filename", outputFileName);
  cmd.AddValue ("enableRouting", "enable Routing", enableRouting);
  cmd.AddValue ("enableMobility", "enable Mobility", enableMobility);
  cmd.AddValue ("scenario", "scenario ", scenario);

  cmd.Parse (argc, argv);
  return true;
}

int main (int argc, char *argv[])
{

  Experiment experiment;
  experiment = Experiment ("multirate");

  //for commandline input
  experiment.CommandSetup(argc, argv);

  // set value to 0 for enabling fragmentation
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue (experiment.GetRtsThreshold()));

  std::ofstream outfile ((experiment.GetOutputFileName()+ ".plt").c_str());

  MobilityHelper mobility;
  Gnuplot gnuplot;
  Gnuplot2dDataset dataset;

  WifiHelper wifi = WifiHelper ();
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();

  //wifiPhy.Set ("TxPowerStart", DoubleValue (experiment.GetTransmitPower()) );
  //wifiPhy.Set ("TxPowerEnd", DoubleValue (experiment.GetTransmitPower()) );


  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay (experiment.GetPropDelay());
  wifiChannel.AddPropagationLoss (experiment.GetPropLoss());


  Ssid ssid = Ssid ("Testbed");
  
  wifiMac.SetType ("ns3::AdhocWifiMac", "Ssid", SsidValue(ssid));
  if (experiment.GetStandard () == 0)
    {
      wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
    }
  else if (experiment.GetStandard () == 1)
    {
      wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
    }
  else if (experiment.GetStandard () == 4)
    {
      wifi.SetStandard (WIFI_PHY_STANDARD_holland);
    }

  if (experiment.GetRateManager().compare("ns3::ConstantRateWifiManager") == 0) 
    {
      wifi.SetRemoteStationManager (experiment.GetRateManager(), "DataMode", StringValue(experiment.GetDataMode()));
    }
  else 
    {
      wifi.SetRemoteStationManager (experiment.GetRateManager());
    }

/*
  NS_LOG_INFO ("Scenario: " << experiment.GetScenario ());
  NS_LOG_INFO ("Rts Threshold: " << experiment.GetRtsThreshold());
  NS_LOG_INFO ("Name:  " << experiment.GetOutputFileName());
  NS_LOG_INFO ("Rate:  " << experiment.GetRateManager());
  NS_LOG_INFO ("Routing: " << experiment.IsRouting());
  NS_LOG_INFO ("Mobility: " << experiment.IsMobility());
*/
  std::cout <<  "Rate:  " << experiment.GetRateManager() << std::endl;;
  std::cout <<  "Standard:  " << experiment.GetStandard () << std::endl;;
  std::cout <<  "Packet Size:  " << experiment.GetPacketSize() << std::endl;;
  std::cout <<  "Grid Size:  " << experiment.GetGridSize() << std::endl;;
  std::cout <<  "Node distance:  " << experiment.GetNodeDistance() << std::endl;;
  std::cout <<  "Loss Prop:  " << experiment.GetPropLoss () << std::endl;;
  std::cout <<  "Delay Prop:  " << experiment.GetPropDelay () << std::endl;;
  std::cout << "Scenario: " << experiment.GetScenario () << std::endl;
  std::cout << "Total time: " << experiment.GetTotalTime () << std::endl;

//  std::cout << "Rts Threshold: " << experiment.GetRtsThreshold() << std::endl;
  std::cout << "Name:  " << experiment.GetOutputFileName() << std::endl;
  std::cout << "Routing: " << experiment.IsRouting() << std::endl;
  std::cout << "Mobility: " << experiment.IsMobility() << std::endl;

  dataset = experiment.Run (wifi, wifiPhy, wifiMac, wifiChannel, mobility);

  gnuplot.AddDataset (dataset);
  gnuplot.GenerateOutput (outfile);

  return 0;
}
