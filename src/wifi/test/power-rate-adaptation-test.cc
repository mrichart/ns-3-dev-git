/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA
 *               2010      NICTA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 *         Quincy Tse <quincy.tse@nicta.com.au> (Case for Bug 991)
 */

#include "ns3/wifi-net-device.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/adhoc-wifi-mac.h"
#include "ns3/yans-wifi-phy.h"
#include "ns3/parf-wifi-manager.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/error-rate-model.h"
#include "ns3/yans-error-rate-model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/node.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/object-factory.h"
#include "ns3/dca-txop.h"
#include "ns3/mac-rx-middle.h"
#include "ns3/pointer.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/edca-txop-n.h"
#include "ns3/config.h"
#include "ns3/boolean.h"

using namespace ns3;

class PowerRateAdaptationTest : public TestCase
{
public:
  PowerRateAdaptationTest ();

  virtual void DoRun (void);
private:
  void TestParf ();
  Ptr<Node> ConfigureNode ();
};

PowerRateAdaptationTest::PowerRateAdaptationTest ()
  : TestCase ("PowerRateAdaptation")
{
}

Ptr<Node>
PowerRateAdaptationTest::ConfigureNode ()
{
  /*
   * Create channel model. Is is necessary to configure correctly the phy layer.
   */
  Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel> ();

  Ptr<Node> node = CreateObject<Node> ();
  Ptr<WifiNetDevice> dev = CreateObject<WifiNetDevice> ();

  /*
   * Create mac layer. We use Adhoc because association is not needed to get supported rates.
   */
  Ptr<AdhocWifiMac> mac = CreateObject<AdhocWifiMac> ();
  mac->ConfigureStandard (WIFI_PHY_STANDARD_80211a);

  /*
   * Create mobility model. Is needed by the phy layer for transmission.
   */

  Ptr<ConstantPositionMobilityModel> mobility = CreateObject<ConstantPositionMobilityModel> ();

  /*
   * Create and configure phy layer.
   */
  Ptr<YansWifiPhy> phy = CreateObject<YansWifiPhy> ();
  phy->SetChannel (channel);
  phy->SetDevice (dev);
  phy->SetMobility (mobility);
  phy->ConfigureStandard (WIFI_PHY_STANDARD_80211a);

  /*
   * Configure power control parameters.
   */
  phy->SetNTxPower(18);
  phy->SetTxPowerStart(0);
  phy->SetTxPowerEnd(17);

  /*
   * Create manager. Configure thresholds for rate and power control.
   */
  Ptr<WifiRemoteStationManager> manager = CreateObject<ParfWifiManager> ();
  manager->SetAttribute("AttemptThreshold",UintegerValue (15));
  manager->SetAttribute("SuccessThreshold",UintegerValue(10));

  /*
   * Configure node. Add mac and phy layer and the manager.
   */
  mac->SetAddress (Mac48Address::Allocate ());
  dev->SetMac (mac);
  dev->SetPhy (phy);
  dev->SetRemoteStationManager (manager);
  node->AddDevice (dev);

  return node;
}

void
PowerRateAdaptationTest::TestParf ()
{
  Ptr<Node> node = ConfigureNode();
  Ptr<WifiNetDevice> dev = DynamicCast<WifiNetDevice> (node->GetDevice(0));
  Ptr<WifiRemoteStationManager> manager = dev->GetRemoteStationManager();

  /*
   * Create a dummy packet to simulate transmission.
   */
  Mac48Address remoteAddress = Mac48Address::Allocate ();
  WifiMacHeader packetHeader;
  packetHeader.SetTypeData ();
  packetHeader.SetQosTid (0);
  Ptr<Packet> packet = Create<Packet> (10);
  WifiMode ackMode;

  /*
   * To initialize the manager we need to generate a transmission.
   */
  Ptr<Packet> p = Create<Packet> ();
  dev->Send (p, remoteAddress, 1);

  //-----------------------------------------------------------------------------------------------------

  /*
   * Parf initiates with maximal rate and power.
   */
  WifiTxVector txVector = manager->GetDataTxVector(remoteAddress,&packetHeader,packet,packet->GetSize());
  WifiMode mode = txVector.GetMode();
  int power = (int) txVector.GetTxPowerLevel();

  NS_TEST_ASSERT_MSG_EQ (mode.GetDataRate(), 54000000, "Initial data rate wrong");
  NS_TEST_ASSERT_MSG_EQ (power, 17, "Initial power level wrong");

  //-----------------------------------------------------------------------------------------------------

  /*
   * After 10 consecutive successful transmissions parf increase rate or decrease power.
   * As we are at maximal rate, the power should be decreased. recoveryPower=true.
   */
  for(int i = 0; i<10; i++)
    {
      manager->ReportDataOk(remoteAddress, &packetHeader, 0, ackMode, 0);
    }

  txVector = manager->GetDataTxVector(remoteAddress,&packetHeader,packet,packet->GetSize());
  mode = txVector.GetMode();
  power = (int) txVector.GetTxPowerLevel();

  NS_TEST_ASSERT_MSG_EQ (mode.GetDataRate(), 54000000, "Incorrect vale of data rate");
  NS_TEST_ASSERT_MSG_EQ (power, 16, "Incorrect value of power level");

  //-----------------------------------------------------------------------------------------------------

  /*
   * As we are using recovery power, one failure make power increase.
   *
   */
  manager->ReportDataFailed(remoteAddress,&packetHeader);

  txVector = manager->GetDataTxVector(remoteAddress,&packetHeader,packet,packet->GetSize());
  mode = txVector.GetMode();
  power = (int) txVector.GetTxPowerLevel();

  NS_TEST_ASSERT_MSG_EQ (mode.GetDataRate(), 54000000, "Incorrect vale of data rate");
  NS_TEST_ASSERT_MSG_EQ (power, 17, "Incorrect value of power level");

  //-----------------------------------------------------------------------------------------------------

  /*
   * After 15 transmissions attempts parf increase rate or decrease power.
   * As we are at maximal rate, the power should be decreased. recoveryPower=true.
   */
  for(int i = 0; i<7; i++)
    {
      manager->ReportDataOk(remoteAddress, &packetHeader, 0, ackMode, 0);
      manager->ReportDataFailed(remoteAddress,&packetHeader);
    }
  manager->ReportDataOk(remoteAddress, &packetHeader, 0, ackMode, 0);

  txVector = manager->GetDataTxVector(remoteAddress,&packetHeader,packet,packet->GetSize());
  mode = txVector.GetMode();
  power = (int) txVector.GetTxPowerLevel();

  NS_TEST_ASSERT_MSG_EQ (mode.GetDataRate(), 54000000, "Incorrect vale of data rate");
  NS_TEST_ASSERT_MSG_EQ (power, 16, "Incorrect value of power level");

  //-----------------------------------------------------------------------------------------------------

  /*
   * As we are using recovery power, one failure make power increase. recoveryPower=false.
   */

  manager->ReportDataFailed(remoteAddress,&packetHeader);

  txVector = manager->GetDataTxVector(remoteAddress,&packetHeader,packet,packet->GetSize());
  mode = txVector.GetMode();
  power = (int) txVector.GetTxPowerLevel();

  NS_TEST_ASSERT_MSG_EQ (mode.GetDataRate(), 54000000, "Incorrect vale of data rate");
  NS_TEST_ASSERT_MSG_EQ (power, 17, "Incorrect value of power level");

  //-----------------------------------------------------------------------------------------------------

  /*
   * After two consecutive fails the rate is decreased or the power increased.
   * As we are at maximal power, the rate should be decreased.
   */
  manager->ReportDataFailed(remoteAddress,&packetHeader);
  manager->ReportDataFailed(remoteAddress,&packetHeader);

  txVector = manager->GetDataTxVector(remoteAddress,&packetHeader,packet,packet->GetSize());
  mode = txVector.GetMode();
  power = (int) txVector.GetTxPowerLevel();

  NS_TEST_ASSERT_MSG_EQ (mode.GetDataRate(), 48000000, "Incorrect vale of data rate");
  NS_TEST_ASSERT_MSG_EQ (power, 17, "Incorrect value of power level");


  Simulator::Stop (Seconds (10.0));

  Simulator::Run ();
  Simulator::Destroy ();

}

void
PowerRateAdaptationTest::DoRun (void)
{
  TestParf ();
}

//-----------------------------------------------------------------------------
class PowerRateAdaptationTestSuite : public TestSuite
{
public:
  PowerRateAdaptationTestSuite ();
};

PowerRateAdaptationTestSuite::PowerRateAdaptationTestSuite ()
  : TestSuite ("power-rate-adaptation-wifi", UNIT)
{
  AddTestCase (new PowerRateAdaptationTest, TestCase::QUICK);
}

static PowerRateAdaptationTestSuite g_powerRateAdaptationTestSuite;
