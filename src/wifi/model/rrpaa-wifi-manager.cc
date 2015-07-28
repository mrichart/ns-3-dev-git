/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015 Universidad de la República - Uruguay
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
 * Author: Matías Richart <mrichart@fing.edu.uy>
 */

#include "rrpaa-wifi-manager.h"
#include "yans-wifi-phy.h"
#include "wifi-phy.h"
#include "wifi-mac.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/simulator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include <cmath>
#define Min(a,b) ((a < b) ? a : b)
NS_LOG_COMPONENT_DEFINE ("RrpaaWifiManager");

namespace ns3 {

/**
 * Hold per-remote-station state for RRPAA Wifi manager.
 *
 * This struct extends from WifiRemoteStation struct to hold additional
 * information required by the APARF Wifi manager
 */
struct RrpaaWifiRemoteStation : public WifiRemoteStation
{
  uint32_t m_counter;            //!< Counter for transmission attempts.
  uint32_t m_nFailed;            //!< Number of failed transmission attempts.
  uint32_t m_rtsWnd;             //!< Window size for the ARts mechanism.
  uint32_t m_rtsCounter;         //!< Counter for rts transmission attempts.
  Time m_lastReset;              //!< Time of the last reset.
  bool m_rtsOn;                  //!< Check if ARts mechanism is on.
  bool m_lastFrameFail;          //!< Flag if the last frame sent has failed.
  bool m_initialized;            //!< For initializing variables.

  uint32_t m_nRate;              //!< Number of supported rates.

  uint32_t m_rate;               //!< Current rate.
  uint8_t m_power;               //!< Current power.

  RrpaaThresholds m_thresholds;  //!< Rrpaa thresholds for this station.

  double** m_pdTable;            //!< Probability table for power and rate changes.
};

NS_OBJECT_ENSURE_REGISTERED (RrpaaWifiManager);

TypeId
RrpaaWifiManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RrpaaWifiManager")
    .SetParent<WifiRemoteStationManager> ()
    .SetGroupName ("Wifi")
    .AddConstructor<RrpaaWifiManager> ()
    .AddAttribute ("Basic",
                   "If true the RRAA-BASIC algorithm will be used, otherwise the RRAA will be used.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RrpaaWifiManager::m_basic),
                   MakeBooleanChecker ())
    .AddAttribute ("Timeout",
                   "Timeout for the RRAA-BASIC loss estimation block (s).",
                   TimeValue (Seconds (0.5)),
                   MakeTimeAccessor (&RrpaaWifiManager::m_timeout),
                   MakeTimeChecker ())
    .AddAttribute ("FrameLength",
                   "The data frame length used for calculating mode TxTime.",
                   UintegerValue (1420),
                   MakeUintegerAccessor (&RrpaaWifiManager::m_frameLength),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("AckFrameLength",
                   "The ACK frame length used for calculating mode TxTime.",
                   DoubleValue (14),
                   MakeDoubleAccessor (&RrpaaWifiManager::m_ackLength),
                   MakeDoubleChecker <uint32_t> ())
    .AddAttribute ("Alpha",
                   "Constant for calculating the MTL threshold.",
                   DoubleValue (1.25),
                   MakeDoubleAccessor (&RrpaaWifiManager::m_alpha),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Beta",
                   "Constant for calculating the ORI threshold.",
                   DoubleValue (2),
                   MakeDoubleAccessor (&RrpaaWifiManager::m_beta),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Tau",
                   "Constant for calculating the EWND size.",
                   DoubleValue (0.015),
                   MakeDoubleAccessor (&RrpaaWifiManager::m_tau),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Gamma",
                   "Constant for Probabilistic Decision Table decrements.",
                   DoubleValue (2),
                   MakeDoubleAccessor (&RrpaaWifiManager::m_gamma),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Delta",
                   "Constant for Probabilistic Decision Table increments.",
                   DoubleValue (1.0905),
                   MakeDoubleAccessor (&RrpaaWifiManager::m_delta),
                   MakeDoubleChecker<double> ())
    .AddTraceSource ("RateChange",
                     "The transmission rate has change.",
                     MakeTraceSourceAccessor (&RrpaaWifiManager::m_rateChange),
                     "ns3::RrpaaWifiManager::RateChangeTracedCallback")
    .AddTraceSource ("PowerChange",
                     "The transmission power has change.",
                     MakeTraceSourceAccessor (&RrpaaWifiManager::m_powerChange),
                     "ns3::RrpaaWifiManager::PowerChangeTracedCallback")
  ;
  return tid;
}


RrpaaWifiManager::RrpaaWifiManager ()
{
  NS_LOG_FUNCTION (this);
}

RrpaaWifiManager::~RrpaaWifiManager ()
{
  NS_LOG_FUNCTION (this);
}

void
RrpaaWifiManager::SetupPhy (Ptr<WifiPhy> phy)
{
  NS_LOG_FUNCTION (this);
  m_minPower = phy->GetTxPowerStart ();
  m_maxPower = phy->GetTxPowerEnd ();
  m_nPower = m_maxPower - m_minPower + 1;
  uint32_t nModes = phy->GetNModes ();
  for (uint32_t i = 0; i < nModes; i++)
    {
      WifiMode mode = phy->GetMode (i);
      WifiTxVector txVector;
      txVector.SetMode (mode);
      /* Calculate the TX Time of the data and the corresponding ACK*/
      AddCalcTxTime (mode, phy->CalculateTxDuration (m_frameLength, txVector, WIFI_PREAMBLE_LONG, phy->GetFrequency (), 0, 0) +
                     phy->CalculateTxDuration (m_ackLength, txVector, WIFI_PREAMBLE_LONG, phy->GetFrequency (), 0, 0));
    }
  WifiRemoteStationManager::SetupPhy (phy);
}

void
RrpaaWifiManager::SetupMac (Ptr<WifiMac> mac)
{
  NS_LOG_FUNCTION (this);
  m_sifs = mac->GetSifs ();
  m_difs = m_sifs + 2 * mac->GetSlot ();
  WifiRemoteStationManager::SetupMac (mac);
}

Time
RrpaaWifiManager::GetCalcTxTime (WifiMode mode) const
{
  NS_LOG_FUNCTION (this << mode);
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
RrpaaWifiManager::AddCalcTxTime (WifiMode mode, Time t)
{
  NS_LOG_FUNCTION (this << mode << t);
  m_calcTxTime.push_back (std::make_pair (t, mode));
}

Thresholds
RrpaaWifiManager::GetThresholds (RrpaaWifiRemoteStation *station, WifiMode mode) const
{
  NS_LOG_FUNCTION (this << station << mode);
  for (RrpaaThresholds::const_iterator i = station->m_thresholds.begin (); i != station->m_thresholds.end (); i++)
    {
      if (mode == i->second)
        {
          return i->first;
        }
    }
  NS_ASSERT (false);
}

WifiRemoteStation *
RrpaaWifiManager::DoCreateStation (void) const
{
  NS_LOG_FUNCTION (this);
  RrpaaWifiRemoteStation *station = new RrpaaWifiRemoteStation ();
  station->m_rtsWnd = 0;
  station->m_rtsCounter = 0;
  station->m_rtsOn = false;
  station->m_lastFrameFail = false;
  station->m_initialized = false;
  return station;
}

void
RrpaaWifiManager::CheckInit (RrpaaWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  if (!station->m_initialized)
    {
      //Note: we appear to be doing late initialization of the table
      //to make sure that the set of supported rates has been initialized
      //before we perform our own initialization.
      station->m_nRate = GetNSupported (station);
      //Initialize at minimal rate and maximal power.
      station->m_rate = 0;
      station->m_power = m_maxPower;
      m_rateChange (station->m_rate, station->m_state->m_address);
      m_powerChange (station->m_power, station->m_state->m_address);

      station->m_pdTable = new double*[station->m_nRate];
      for (uint32_t i = 0; i < station->m_nRate; i++)
        {
          station->m_pdTable[i] = new double[m_nPower];
          for (uint8_t j = 0; j < m_nPower; j++)
            {
              station->m_pdTable[i][j] = 1;
            }
        }

      station->m_initialized = true;

      station->m_thresholds = RrpaaThresholds (station->m_nRate);
      InitThresholds (station);
      ResetCountersBasic (station);
    }
}

void
RrpaaWifiManager::InitThresholds (RrpaaWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  NS_LOG_DEBUG ("InitThresholds = " << station);

  double nextCritical = 0;
  double nextMtl = 0;
  double mtl = 0;
  double ori = 0;
  for (uint32_t i = 0; i < station->m_nRate; i++)
    {
      WifiMode mode = GetSupported (station, i);
      Time totalTxTime = GetCalcTxTime (mode) + m_sifs + m_difs;
      if (i == station->m_nRate - 1)
        {
          ori = 0;
        }
      else
        {
          WifiMode nextMode = GetSupported (station, i + 1);
          Time nextTotalTxTime = GetCalcTxTime (nextMode) + m_sifs + m_difs;
          nextCritical = 1 - (nextTotalTxTime.GetSeconds () / totalTxTime.GetSeconds ());
          nextMtl = m_alpha * nextCritical;
          ori = nextMtl / m_beta;
        }
      if (i == 0)
        {
          mtl = nextMtl;
        }
      Thresholds th;
      th.ewnd = ceil (m_tau / totalTxTime.GetSeconds ());
      th.ori = ori;
      th.mtl = mtl;
      station->m_thresholds.push_back (std::make_pair (th, mode));
      mtl = nextMtl;
      NS_LOG_DEBUG (mode << " " << th.ewnd << " " << th.mtl << " " << th.ori);
    }
}

void
RrpaaWifiManager::ResetCountersBasic (RrpaaWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  station->m_nFailed = 0;
  station->m_counter = GetThresholds (station, station->m_rate).ewnd;
  station->m_lastReset = Simulator::Now ();
}

void
RrpaaWifiManager::DoReportRtsFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
}

void
RrpaaWifiManager::DoReportDataFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  CheckInit (station);
  station->m_lastFrameFail = true;
  CheckTimeout (station);
  station->m_counter--;
  station->m_nFailed++;
  RunBasicAlgorithm (station);
}

void
RrpaaWifiManager::DoReportRxOk (WifiRemoteStation *st,
                                double rxSnr, WifiMode txMode)
{
  NS_LOG_FUNCTION (this << st << rxSnr << txMode);
}

void
RrpaaWifiManager::DoReportRtsOk (WifiRemoteStation *st,
                                 double ctsSnr, WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_FUNCTION (this << st << ctsSnr << ctsMode << rtsSnr);
  NS_LOG_DEBUG ("self=" << st << " rts ok");
}

void
RrpaaWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                  double ackSnr, WifiMode ackMode, double dataSnr)
{
  NS_LOG_FUNCTION (this << st << ackSnr << ackMode << dataSnr);
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  CheckInit (station);
  station->m_lastFrameFail = false;
  CheckTimeout (station);
  station->m_counter--;
  RunBasicAlgorithm (station);
}
void
RrpaaWifiManager::DoReportFinalRtsFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
}
void
RrpaaWifiManager::DoReportFinalDataFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
}

WifiTxVector
RrpaaWifiManager::DoGetDataTxVector (WifiRemoteStation *st,
                                     uint32_t size)
{
  NS_LOG_FUNCTION (this << st << size);
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  CheckInit (station);
  return WifiTxVector (GetSupported (station, station->m_rate), station->m_power, GetLongRetryCount (station), GetShortGuardInterval (station), Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas ()), GetNumberOfTransmitAntennas (station), GetStbc (station));
}
WifiTxVector
RrpaaWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  CheckInit (station);
  return WifiTxVector (GetSupported (st, 0), GetDefaultTxPowerLevel (), GetLongRetryCount (station), GetShortGuardInterval (station), Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas ()), GetNumberOfTransmitAntennas (station), GetStbc (station));
}

bool
RrpaaWifiManager::DoNeedRts (WifiRemoteStation *st,
                             Ptr<const Packet> packet, bool normally)
{
  NS_LOG_FUNCTION (this << st << packet << normally);
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  CheckInit (station);
  if (m_basic)
    {
      return normally;
    }
  ARts (station);
  return station->m_rtsOn;
}

void
RrpaaWifiManager::CheckTimeout (RrpaaWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  Time d = Simulator::Now () - station->m_lastReset;
  if (station->m_counter == 0 || d > m_timeout)
    {
      ResetCountersBasic (station);
    }
}

void
RrpaaWifiManager::RunBasicAlgorithm (RrpaaWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  RngSeedManager::SetSeed (static_cast<uint32_t> (time (0)));
  Thresholds thresholds = GetThresholds (station, station->m_rate);
  double bploss = (double) station->m_nFailed / (double) thresholds.ewnd;
  double wploss = (double) (station->m_counter + station->m_nFailed) / (double) thresholds.ewnd;
  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
  if (bploss >= thresholds.mtl)
    {
      if (station->m_power < m_maxPower)
        {
          station->m_pdTable[station->m_rate][station->m_power] /= m_gamma;
          station->m_power++;
          m_powerChange (station->m_power, station->m_state->m_address);
          ResetCountersBasic (station);
        }
      else if (station->m_rate != 0)
        {
          station->m_pdTable[station->m_rate][station->m_power] /= m_gamma;
          station->m_rate--;
          m_rateChange (station->m_rate, station->m_state->m_address);
          ResetCountersBasic (station);
        }
    }
  else if (wploss <= thresholds.ori)
    {
      if (station->m_rate < station->m_nRate - 1)
        {
          for (uint32_t i = 0; i <= station->m_rate; i++)
            {
              station->m_pdTable[i][station->m_power] *= m_delta;
              if (station->m_pdTable[i][station->m_power] > 1)
                {
                  station->m_pdTable[i][station->m_power] = 1;
                }
            }
          double rand = uv->GetValue (0,1);
          if (rand < station->m_pdTable[station->m_rate + 1][station->m_power])
            {
              station->m_rate++;
              m_rateChange (station->m_rate, station->m_state->m_address);
            }
        }
      else if (station->m_power > m_minPower)
        {
          for (uint32_t i = m_maxPower; i > station->m_power; i--)
            {
              station->m_pdTable[station->m_rate][i] *= m_delta;
              if (station->m_pdTable[station->m_rate][i] > 1)
                {
                  station->m_pdTable[station->m_rate][i] = 1;
                }
            }
          double rand = uv->GetValue (0,1);
          if (rand < station->m_pdTable[station->m_rate][station->m_power - 1])
            {
              station->m_power--;
              m_powerChange (station->m_power, station->m_state->m_address);
            }
        }
      ResetCountersBasic (station);
    }
  else if (bploss > thresholds.ori && wploss < thresholds.mtl)
    {
      if (station->m_power > m_minPower)
        {
          for (uint32_t i = m_maxPower; i >= station->m_power; i--)
            {
              station->m_pdTable[station->m_rate][i] *= m_delta;
              if (station->m_pdTable[station->m_rate][i] > 1)
                {
                  station->m_pdTable[station->m_rate][i] = 1;
                }
            }
          double rand = uv->GetValue (0,1);
          if (rand < station->m_pdTable[station->m_rate][station->m_power - 1])
            {
              station->m_power--;
              m_powerChange (station->m_power, station->m_state->m_address);
            }
          ResetCountersBasic (station);
        }
    }
  if (station->m_counter == 0)
    {
      ResetCountersBasic (station);
    }
}

void
RrpaaWifiManager::ARts (RrpaaWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  if (!station->m_rtsOn
      && station->m_lastFrameFail)
    {
      station->m_rtsWnd += 2;
      station->m_rtsCounter = station->m_rtsWnd;
    }
  else if ((station->m_rtsOn && station->m_lastFrameFail)
           || (!station->m_rtsOn && !station->m_lastFrameFail))
    {
      station->m_rtsWnd = station->m_rtsWnd / 2;
      station->m_rtsCounter = station->m_rtsWnd;
    }
  if (station->m_rtsCounter > 0)
    {
      station->m_rtsOn = true;
      station->m_rtsCounter--;
    }
  else
    {
      station->m_rtsOn = false;
    }
}

Thresholds
RrpaaWifiManager::GetThresholds (RrpaaWifiRemoteStation *station,
                                 uint32_t rate) const
{
  NS_LOG_FUNCTION (this << station << rate);
  WifiMode mode = GetSupported (station, rate);
  return GetThresholds (station, mode);
}

bool
RrpaaWifiManager::IsLowLatency (void) const
{
  NS_LOG_FUNCTION (this);
  return true;
}

} // namespace ns3
