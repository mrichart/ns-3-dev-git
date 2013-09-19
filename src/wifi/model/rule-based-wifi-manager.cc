/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 Duy Nguyen
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
 * Author: Duy Nguyen <duy@soe.ucsc.edu>
 *
 * Some Comments:
 *
 * 1) Segment Size is declared for completeness but not used  because it has
 *    to do more with the requirement of the specific hardware.
 *
 * 2) By default, Minstrel applies the multi-rate retry(the core of Minstrel
 *    algorithm). Otherwise, please use ConstantRateWifiManager instead.
 *
 * http://linuxwireless.org/en/developers/Documentation/mac80211/RateControl/minstrel
 */

#include "rule-based-wifi-manager.h"
#include "wifi-phy.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/wifi-mac.h"
#include "ns3/assert.h"
#include <vector>

#define Min(a,b) ((a < b) ? a : b)
#define Max(a,b) ((a > b) ? a : b)

NS_LOG_COMPONENT_DEFINE ("RuleBasedWifiManager");


namespace ns3 {

struct RuleBasedWifiRemoteStation : public WifiRemoteStation
{
  uint32_t m_success;
  uint32_t m_attempt;

  uint32_t m_rate;
  uint8_t m_power;
};

NS_OBJECT_ENSURE_REGISTERED (RuleBasedWifiManager);

TypeId
RuleBasedWifiManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RuleBasedWifiManager")
    .SetParent<WifiRemoteStationManager> ()
    .AddConstructor<RuleBasedWifiManager> ()
	.AddTraceSource ("DoGetDataTxVector",
       "A TxVector is ask for a new data frame",
            MakeTraceSourceAccessor (&RuleBasedWifiManager::m_getDataTxVector))
  ;
  return tid;
}

RuleBasedWifiManager::RuleBasedWifiManager ()
{
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();

  m_nsupported = 0;
}

RuleBasedWifiManager::~RuleBasedWifiManager ()
{
}

void
RuleBasedWifiManager::SetupPhy (Ptr<WifiPhy> phy)
{
  m_nPower = phy->GetNTxPower();
  uint32_t nModes = phy->GetNModes ();
  for (uint32_t i = 0; i < nModes; i++)
    {
      WifiMode mode = phy->GetMode (i);
      WifiTxVector txVector;
      txVector.SetMode(mode);
      AddCalcTxTime (mode, phy->CalculateTxDuration (m_pktLen, txVector, WIFI_PREAMBLE_LONG));
    }
  WifiRemoteStationManager::SetupPhy (phy);
}

int64_t
RuleBasedWifiManager::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  return 1;
}

Time
RuleBasedWifiManager::GetCalcTxTime (WifiMode mode) const
{

  for (TxTime::const_iterator i = m_calcTxTime.begin (); i != m_calcTxTime.end (); i++)
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
RuleBasedWifiManager::AddCalcTxTime (WifiMode mode, Time t)
{
  m_calcTxTime.push_back (std::make_pair (t, mode));
}

WifiRemoteStation *
RuleBasedWifiManager::DoCreateStation (void) const
{
  RuleBasedWifiRemoteStation *station = new RuleBasedWifiRemoteStation ();

  station->m_success = 0;
  station->m_attempt = 0;
  station->m_rate = 0;
  station->m_power = m_nPower-1;

  return station;
}

void
RuleBasedWifiManager::DoReportRxOk (WifiRemoteStation *st,
                                   double rxSnr, WifiMode txMode)
{
  NS_LOG_DEBUG ("DoReportRxOk m_rate=" << ((RuleBasedWifiRemoteStation *)st)->m_rate);
}

void
RuleBasedWifiManager::DoReportRtsFailed (WifiRemoteStation *st)
{
  RuleBasedWifiRemoteStation *station = (RuleBasedWifiRemoteStation *)st;

  NS_LOG_DEBUG ("DoReportRtsFailed m_rate=" << station->m_rate);

}

void
RuleBasedWifiManager::DoReportRtsOk (WifiRemoteStation *st, double ctsSnr, WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_DEBUG ("self=" << st << " rts ok");
}

void
RuleBasedWifiManager::DoReportFinalRtsFailed (WifiRemoteStation *st)
{

}

void
RuleBasedWifiManager::DoReportDataFailed (WifiRemoteStation *st)
{
  RuleBasedWifiRemoteStation *station = (RuleBasedWifiRemoteStation *)st;

  station->m_attempt++;

  NS_LOG_DEBUG ("DoReportDataFailed " << station << "\t rate " << station->m_rate << "\tattempts \t" << station->m_attempt);
}

void
RuleBasedWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                     double ackSnr, WifiMode ackMode, double dataSnr)
{
  RuleBasedWifiRemoteStation *station = (RuleBasedWifiRemoteStation *) st;

  station->m_success++;
  station->m_attempt++;
}

void
RuleBasedWifiManager::DoReportFinalDataFailed (WifiRemoteStation *st)
{
  RuleBasedWifiRemoteStation *station = (RuleBasedWifiRemoteStation *) st;

  NS_LOG_DEBUG ("DoReportFinalDataFailed m_rate=" << station->m_rate);
}


WifiTxVector
RuleBasedWifiManager::DoGetDataTxVector (WifiRemoteStation *st,
                                    uint32_t size)
{
  RuleBasedWifiRemoteStation *station = (RuleBasedWifiRemoteStation *) st;

  WifiTxVector vector =  WifiTxVector (GetSupported (station, station->m_rate), station->m_power, GetLongRetryCount (station), GetShortGuardInterval (station), Min (GetNoOfRx (station),GetNoOfTransmitters()), GetNoOfTx (station), GetStbc (station));
  m_getDataTxVector(vector);
  return vector;
}

WifiTxVector
RuleBasedWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  RuleBasedWifiRemoteStation *station = (RuleBasedWifiRemoteStation *) st;
  NS_LOG_DEBUG ("DoGetRtsMode m_rate=" << station->m_rate);

  return WifiTxVector (GetSupported (station, 0), GetDefaultTxPowerLevel (), GetShortRetryCount (station), GetShortGuardInterval (station), Min (GetNoOfRx (station),GetNoOfTransmitters()), GetNoOfTx (station), GetStbc (station));
}

bool
RuleBasedWifiManager::IsLowLatency (void) const
{
  return true;
}

void
RuleBasedWifiManager::DecreaseRate (Mac48Address address, uint32_t levels)
{
  NS_LOG_DEBUG ("Decrease rate");

  uint8_t tid = 0;
  RuleBasedWifiRemoteStation *station = (RuleBasedWifiRemoteStation *) GetStation (address, tid);
  if (station->m_rate > levels)
    {
	  station->m_rate -= levels;
    }
  else
    {
	  station->m_rate = 0;
    }
}

void
RuleBasedWifiManager::IncreaseRate (Mac48Address address, uint32_t levels)
{
  NS_LOG_DEBUG ("Increase rate");

  uint8_t tid = 0;
  RuleBasedWifiRemoteStation *station = (RuleBasedWifiRemoteStation *) GetStation (address, tid);
  if (station->m_rate < (station->m_state->m_operationalRateSet.size () - levels))
    {
	  station->m_rate += levels;
    }
  else
    {
	  station->m_rate = station->m_state->m_operationalRateSet.size () - 1;
    }
}

void
RuleBasedWifiManager::DecreasePower (Mac48Address address, uint32_t levels)
{
  NS_LOG_DEBUG ("Decrease power");

  uint8_t tid = 0;
  RuleBasedWifiRemoteStation *station = (RuleBasedWifiRemoteStation *) GetStation (address, tid);
  if (station->m_power > levels)
    {
	  station->m_power -= levels;
    }
  else
    {
	  station->m_power = 0;
    }
}

void
RuleBasedWifiManager::IncreasePower (Mac48Address address, uint32_t levels)
{
  NS_LOG_DEBUG ("Decrease rate");

  uint8_t tid = 0;
  RuleBasedWifiRemoteStation *station = (RuleBasedWifiRemoteStation *) GetStation (address, tid);
  if (station->m_power < m_nPower - levels)
    {
	  station->m_power += levels;
    }
  else
    {
	  station->m_power = m_nPower - 1;
    }
}

double
RuleBasedWifiManager::GetStats (Mac48Address address)
{
  NS_LOG_DEBUG ("Updating stats=" << this);

  uint8_t tid = 0;
  RuleBasedWifiRemoteStation *station = (RuleBasedWifiRemoteStation *) GetStation (address, tid);

  double prob = 0;

  NS_LOG_DEBUG ("GET STATS=" << station->m_success << " " << station->m_attempt);

  if (station->m_attempt)
  {
	  prob = ((double)station->m_success) / ((double)station->m_attempt);
  }
  station->m_success = 0;
  station->m_attempt = 0;

  NS_LOG_DEBUG ("prob=" << prob);
  return prob;
}

uint32_t
RuleBasedWifiManager::GetCurrentRate (Mac48Address address)
{
  NS_LOG_DEBUG ("Updating stats=" << this);

  uint8_t tid = 0;
  RuleBasedWifiRemoteStation *station = (RuleBasedWifiRemoteStation *) GetStation (address, tid);

  return station->m_rate;
}

uint8_t
RuleBasedWifiManager::GetCurrentPower (Mac48Address address)
{
  NS_LOG_DEBUG ("Updating stats=" << this);

  uint8_t tid = 0;
  RuleBasedWifiRemoteStation *station = (RuleBasedWifiRemoteStation *) GetStation (address, tid);

  return station->m_power;
}

} // namespace ns3





