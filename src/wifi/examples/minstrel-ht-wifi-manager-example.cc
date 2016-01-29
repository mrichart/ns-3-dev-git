/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 University of Washington
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
 * Author: Tom Henderson <tomhend@u.washington.edu>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"
#include "ns3/stats-module.h"
#include "ns3/mobility-module.h"
#include "ns3/propagation-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("Iwf");

// 20 MHz, 290K
const double NOISE_DBM = -101.0;

double g_intervalBytes = 0;
uint64_t g_intervalRate = 0;

void
PacketRx (Ptr<const Packet> pkt, const Address &addr)
{
  NS_LOG_DEBUG ("Received size " << pkt->GetSize ());
  g_intervalBytes += pkt->GetSize ();
}

void
RateChange (uint64_t newVal, Mac48Address dest)
{
  NS_LOG_DEBUG ("Change to " << newVal);
  g_intervalRate = newVal;
}

struct Step
{
  double stepSize;
  double stepTime;
};

struct StandardInfo
{
  StandardInfo (std::string name, enum WifiPhyStandard standard, double snrLow, double snrHigh)
   : m_name (name),
     m_standard (standard),
     m_snrLow (snrLow),
     m_snrHigh (snrHigh)
    {}
  std::string m_name;
  enum WifiPhyStandard m_standard;
  double m_snrLow;
  double m_snrHigh;
};

void
ChangeSignalAndReportRate (Ptr<FixedRssLossModel> rssModel, struct Step step, double rss, Gnuplot2dDataset& rateDataset, Gnuplot2dDataset& actualDataset)
{
  NS_LOG_FUNCTION (rssModel << step.stepSize << step.stepTime << rss);
  double snr = rss - NOISE_DBM; //dB
  rateDataset.Add (snr, g_intervalRate / 1000000.0);
  // Calculate received rate since last interval
  double currentRate = ((g_intervalBytes * 8)/step.stepTime) / 1e6; // Mb/s
  actualDataset.Add (snr, currentRate);
  rssModel->SetRss (rss - step.stepSize);
  NS_LOG_INFO ("At time " << Simulator::Now ().As (Time::S) << "; observed rate " << currentRate << "; setting new power to " << rss - step.stepSize);
  g_intervalBytes = 0;
  Simulator::Schedule (Seconds (step.stepTime), &ChangeSignalAndReportRate, rssModel, step, (rss - step.stepSize), rateDataset, actualDataset);
}
int main (int argc, char *argv[])
{
  std::ofstream outfile ("ideal-wifi-manager.plt");
  std::vector <StandardInfo> standards;
  int steps;
  uint32_t rtsThreshold = 2346;
  double stepSize = 1; // dBm
  double stepTime = 1; // seconds
  int broadcast = 0;
  int ap1_x = 0;
  int ap1_y = 0;
  int sta1_x = 5;
  int sta1_y = 0;

  CommandLine cmd;
  cmd.AddValue ("rtsThreshold", "RTS threshold", rtsThreshold);
  cmd.AddValue ("stepSize", "Power between steps (dBm)", stepSize);
  cmd.AddValue ("stepTime", "Time on each step (seconds)", stepTime);
  cmd.AddValue ("broadcast", "Send broadcast instead of unicast", broadcast);
  cmd.Parse (argc, argv);
  Gnuplot gnuplot = Gnuplot ("minstrel-ht-wifi-manager.eps");
  // 802.11a has interesting values between SNR 3 dB and SNR 27 dB
  //standards.push_back (StandardInfo ("802.11a", WIFI_PHY_STANDARD_80211a, 3, 27));
  //standards.push_back (StandardInfo ("802.11b", WIFI_PHY_STANDARD_80211b, -5, 11));
  //standards.push_back (StandardInfo ("802.11g", WIFI_PHY_STANDARD_80211g, -5, 27));
  standards.push_back (StandardInfo ("802.11n-5GHz", WIFI_PHY_STANDARD_80211n_5GHZ, 5, 27));
#ifdef NOTYET
  standards.push_back (StandardInfo ("802.11n-2.4GHz", WIFI_PHY_STANDARD_80211n_2_4GHZ, 5, 27));
  standards.push_back (WIFI_PHY_STANDARD_80211_10MHZ);
  standards.push_back (WIFI_PHY_STANDARD_80211_5MHZ);
  standards.push_back (WIFI_PHY_STANDARD_80211_holland);
  standards.push_back (WIFI_PHY_STANDARD_80211_ac);
#endif
  for (std::vector<StandardInfo>::size_type i = 0; i != standards.size(); i++)
    {
      std::cout << "Testing " << standards[i].m_name << "..." << std::endl;
      NS_ABORT_MSG_IF (standards[i].m_snrLow >= standards[i].m_snrHigh, "SNR values in wrong order");
      steps = std::abs ((int) (standards[i].m_snrHigh - standards[i].m_snrLow )/stepTime) + 1;
      Ptr<Node> clientNode = CreateObject<Node> ();
      Ptr<Node> serverNode = CreateObject<Node> ();
      WifiHelper wifi = WifiHelper::Default ();
      wifi.SetStandard (standards[i].m_standard);
      YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
      wifiPhy.Set ("RxGain", DoubleValue (0.0));
      wifiPhy.Set ("RxNoiseFigure", DoubleValue (0.0));
      wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-110.0));
      wifiPhy.Set ("CcaMode1Threshold", DoubleValue (-110.0));
      Ptr<YansWifiChannel> wifiChannel = CreateObject<YansWifiChannel> ();
      Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
      wifiChannel->SetPropagationDelayModel (delayModel);
      Ptr<FixedRssLossModel> rssLossModel = CreateObject<FixedRssLossModel> ();
      wifiChannel->SetPropagationLossModel (rssLossModel);
      wifiPhy.SetChannel (wifiChannel);
      wifi.SetRemoteStationManager ("ns3::MinstrelHtWifiManager", "RtsCtsThreshold", UintegerValue (rtsThreshold));
      // Use Adhoc so we don't get into association issues
      NetDeviceContainer serverDevice;
      NetDeviceContainer clientDevice;
      if (standards[i].m_name == "802.11a" || standards[i].m_name == "802.11b" || standards[i].m_name == "802.11g")
        {
          NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
          wifiMac.SetType ("ns3::AdhocWifiMac");
          serverDevice = wifi.Install (wifiPhy, wifiMac, serverNode);
          clientDevice = wifi.Install (wifiPhy, wifiMac, clientNode);
        }
      else if (standards[i].m_name == "802.11n-5GHz" || standards[i].m_name == "802.11n-2.4GHz")
        {
          HtWifiMacHelper wifiMac = HtWifiMacHelper::Default ();
          wifiMac.SetType ("ns3::AdhocWifiMac");
          /*wifiMac.SetBlockAckThresholdForAc (AC_BE, 2);
          wifiMac.SetMpduAggregatorForAc (AC_BE, "ns3::MpduStandardAggregator",
                                          "MaxAmpduSize", UintegerValue (65535));*/
          serverDevice = wifi.Install (wifiPhy, wifiMac, serverNode);
          clientDevice = wifi.Install (wifiPhy, wifiMac, clientNode);
        }
      NS_ABORT_MSG_IF (serverDevice.GetN () == 0, "unknown standard");
      Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/$ns3::MinstrelHtWifiManager/RateChange", MakeCallback(&RateChange));
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
      mobility.Install (clientNode);
      mobility.Install (serverNode);
      Gnuplot2dDataset rateDataset (standards[i].m_name + std::string ("-rate selected"));
      Gnuplot2dDataset actualDataset (standards[i].m_name + std::string ("-observed"));
      struct Step step;
      step.stepSize = stepSize;
      step.stepTime = stepTime;

      //  rss = snr + NOISE
      double rssCurrent = (standards[i].m_snrHigh + NOISE_DBM);
      rssLossModel->SetRss (rssCurrent);
      NS_LOG_INFO ("Setting initial Rss to " << rssCurrent);
      //Move the STA by stepsSize meters every stepTime seconds
      Simulator::Schedule (Seconds (0.5 + stepTime), &ChangeSignalAndReportRate, rssLossModel, step, rssCurrent, rateDataset, actualDataset);
      PacketSocketHelper packetSocketHelper;
      packetSocketHelper.Install (serverNode);
      packetSocketHelper.Install (clientNode);

      PacketSocketAddress socketAddr;
      socketAddr.SetSingleDevice (serverDevice.Get (0)->GetIfIndex ());
      if (broadcast)
        {
          socketAddr.SetPhysicalAddress (serverDevice.Get (0)->GetBroadcast ());
        }
      else
        {
          socketAddr.SetPhysicalAddress (serverDevice.Get (0)->GetAddress ());
        }
      // Arbitrary protocol type.
      // Note: PacketSocket doesn't have any L4 multiplexing or demultiplexing
      //       The only mux/demux is based on the protocol field
      socketAddr.SetProtocol (1);
      Ptr<PacketSocketClient> client = CreateObject<PacketSocketClient> ();
      client->SetRemote (socketAddr);
      client->SetStartTime (Seconds (0.5));
      client->SetAttribute ("MaxPackets", UintegerValue (0));
      client->SetAttribute ("PacketSize", UintegerValue (1024));
      client->SetAttribute ("Interval", TimeValue (MicroSeconds (20)));
      clientNode->AddApplication (client);
      Ptr<PacketSocketServer> server = CreateObject<PacketSocketServer> ();
      server->SetLocal (socketAddr);
      server->TraceConnectWithoutContext ("Rx", MakeCallback (&PacketRx));
      serverNode->AddApplication (server);
      Simulator::Stop (Seconds ((steps + 1) * stepTime));
      Simulator::Run ();
      Simulator::Destroy ();
      gnuplot.AddDataset (rateDataset);
      gnuplot.AddDataset (actualDataset);
    }
  gnuplot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  gnuplot.SetLegend ("SNR (dB)", "Rate (Mb/s)");
  gnuplot.SetExtra  ("set xrange [0:50]");
  gnuplot.SetExtra  ("set yrange [0:72]");
  gnuplot.GenerateOutput (outfile);
  outfile.close ();
  return 0;
}
