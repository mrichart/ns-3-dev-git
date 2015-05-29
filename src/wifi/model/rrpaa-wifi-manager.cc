/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 Universidad de la República - Uruguay
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
 * Author: Matias Richart <mrichart@fing.edu.uy>
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

struct RrpaaWifiRemoteStation : public WifiRemoteStation
{
  uint32_t m_counter;
  uint32_t m_failed;
  uint32_t m_rtsWnd;
  uint32_t m_rtsCounter;
  Time m_lastReset;
  bool m_rtsOn;
  bool m_lastFrameFail;
  bool m_initialized;

  uint32_t m_nRate;

  uint32_t m_rate;
  uint8_t m_power;

  RrpaaThresholds m_thresholds;

  double** m_priTable;
};

NS_OBJECT_ENSURE_REGISTERED (RrpaaWifiManager);

TypeId
RrpaaWifiManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::RrpaaWifiManager")
    .SetParent<WifiRemoteStationManager> ()
    .AddConstructor<RrpaaWifiManager> ()
    .AddAttribute ("Basic",
                   "If true the RRAA-BASIC algorithm will be used, otherwise the RRAA wil be used",
                   BooleanValue (true),
                   MakeBooleanAccessor (&RrpaaWifiManager::m_basic),
                   MakeBooleanChecker ())
    .AddAttribute ("Timeout",
                   "Timeout for the RRAA BASIC loss estimaton block (s)",
                   TimeValue (Seconds (0.05)),
                   MakeTimeAccessor (&RrpaaWifiManager::m_timeout),
                   MakeTimeChecker ())
    .AddAttribute ("FrameLength",
		  "The frame length used for calculating mode TxTime",
		  UintegerValue (1420),
		  MakeUintegerAccessor (&RrpaaWifiManager::m_frameLength),
		  MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("AckFrameLength",
		  "The ACK frame length used for calculating mode TxTime",
		  DoubleValue (14),
		  MakeDoubleAccessor (&RrpaaWifiManager::m_ackLength),
		  MakeDoubleChecker <uint32_t> ())
    .AddAttribute ("Alpha",
                   "ewnd parameter for 54 Mbs data mode",
                   DoubleValue (1.25),
                   MakeDoubleAccessor (&RrpaaWifiManager::m_alpha),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Beta",
                   "ewnd parameter for 48 Mbs data mode",
                   DoubleValue (2),
		    MakeDoubleAccessor (&RrpaaWifiManager::m_beta),
		    MakeDoubleChecker<double> ())
    .AddAttribute ("Gamma",
                   "ewnd parameter for 36 Mbs data mode",
                   DoubleValue (0.015),
                   MakeDoubleAccessor (&RrpaaWifiManager::m_gamma),
                   MakeDoubleChecker<double> ())
    .AddTraceSource("RateChange",
                   "The transmission rate has change",
                   MakeTraceSourceAccessor(&RrpaaWifiManager::m_rateChange),
                   "ns3::RrpaaWifiManager::RateChangeTracedCallback")
    .AddTraceSource ("PowerChange",
                  "The transmission power has change",
                   MakeTraceSourceAccessor (&RrpaaWifiManager::m_powerChange),
                   "ns3::RrpaaWifiManager::PowerChangeTracedCallback")
  ;
  return tid;
}


RrpaaWifiManager::RrpaaWifiManager ()
{
}
RrpaaWifiManager::~RrpaaWifiManager ()
{
}

void
RrpaaWifiManager::SetupPhy (Ptr<WifiPhy> phy)
{
  NS_LOG_FUNCTION (this);
  m_minPower = phy->GetTxPowerStart();
  m_maxPower = phy->GetTxPowerEnd();
  m_nPower = m_maxPower - m_minPower + 1;
  uint32_t nModes = phy->GetNModes ();
  for (uint32_t i = 0; i < nModes; i++)
    {
      WifiMode mode = phy->GetMode (i);
      WifiTxVector txVector;
      txVector.SetMode(mode);
      /* Calculate the TX Time of the data and the corresponding ACK*/
      AddCalcTxTime (mode, phy->CalculateTxDuration (m_frameLength, txVector, WIFI_PREAMBLE_LONG, phy->GetFrequency(), 0, 0) +
                     phy->CalculateTxDuration (m_ackLength, txVector, WIFI_PREAMBLE_LONG, phy->GetFrequency(), 0, 0));
    }
  WifiRemoteStationManager::SetupPhy (phy);
}

void
RrpaaWifiManager::SetupMac (Ptr<WifiMac> mac)
{
  m_sifs = mac->GetSifs();
  m_difs = m_sifs + 2*mac->GetSlot();
  WifiRemoteStationManager::SetupMac (mac);
}

Time
RrpaaWifiManager::GetCalcTxTime (WifiMode mode) const
{
  NS_LOG_FUNCTION (this);
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
  NS_LOG_FUNCTION (this);
  m_calcTxTime.push_back (std::make_pair (t, mode));
}

Thresholds
RrpaaWifiManager::GetThresholds (RrpaaWifiRemoteStation *station, WifiMode mode) const
{
  NS_LOG_FUNCTION (this);
  for (RrpaaThresholds::const_iterator i = station->m_thresholds.begin (); i != station->m_thresholds.end (); i++)
    {
      if (mode == i->second)
        {
          return i->first;
        }
    }
  NS_ASSERT (false);
}

void
RrpaaWifiManager::AddThresholds (RrpaaWifiRemoteStation *station, WifiMode mode, Thresholds th)
{
  NS_LOG_FUNCTION (this);
  station->m_thresholds.push_back (std::make_pair (th, mode));
}

WifiRemoteStation *
RrpaaWifiManager::DoCreateStation (void) const
{
  RrpaaWifiRemoteStation *station = new RrpaaWifiRemoteStation ();
  station->m_initialized = false;
  station->m_rtsWnd = 0;
  station->m_rtsCounter = 0;
  station->m_rtsOn = false;
  station->m_lastFrameFail = false;
  return station;
}

void
RrpaaWifiManager::CheckInit (RrpaaWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  if (!station->m_initialized)
    {
      // Note: we appear to be doing late initialization of the table
      // to make sure that the set of supported rates has been initialized
      // before we perform our own initialization.
      station->m_nRate = GetNSupported (station);
      station->m_rate = GetMaxRate (station);
      m_rateChange(station->m_rate, station->m_state->m_address);
      station->m_power = m_maxPower;
      m_powerChange(station->m_power, station->m_state->m_address);

      station->m_priTable = new double*[station->m_nRate];
      for (uint32_t i = 0; i < station->m_nRate; i++)
	{
	  station->m_priTable[i] = new double[m_nPower];
	  for (uint8_t j = 0; j < m_nPower; j++)
	    station->m_priTable[i][j] = 1;
	}

      station->m_initialized = true;

      station->m_thresholds = RrpaaThresholds (station->m_nRate);
      InitThresholds (station);
      ResetCountersBasic(station);
    }
}

void
RrpaaWifiManager::InitThresholds (RrpaaWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("InitThresholds = " << station);

  double nextCritical = 0;
  double nextMtl = 0;
  double mtl = 0;
  double ori = 0;
  for (uint32_t i = 0; i < station->m_nRate; i++)
    {
      WifiMode mode = GetSupported(station, i);
      Time totalTxTime = GetCalcTxTime(mode) + m_sifs + m_difs;
      if (i == 0)
	{
	  mtl = 1;
	}
      if (i == station->m_nRate -1)
	{
	  ori = 0;
	}
      else
	{
	  WifiMode nextMode = GetSupported(station, i+1);
          Time nextTotalTxTime = GetCalcTxTime(nextMode) + m_sifs + m_difs;
	  nextCritical = 1 - (nextTotalTxTime.GetSeconds() / totalTxTime.GetSeconds());
	  nextMtl = m_alpha*nextCritical;
	  ori = nextMtl / m_beta;;
	}
      Thresholds th;
      th.ewnd = ceil (m_gamma / totalTxTime.GetSeconds());
      th.ori = ori;
      th.mtl = mtl;
      station->m_thresholds.push_back(std::make_pair (th, mode));
      mtl = nextMtl;
      NS_LOG_DEBUG (mode << " " << th.ewnd << " " << th.mtl << " " << th.ori);
    }
}
void
RrpaaWifiManager::ResetCountersBasic (RrpaaWifiRemoteStation *station)
{
  station->m_failed = 0;
  station->m_counter = GetThresholds (station, station->m_rate).ewnd;
  station->m_lastReset = Simulator::Now ();
}

uint32_t
RrpaaWifiManager::GetMaxRate (RrpaaWifiRemoteStation *station)
{
  return GetNSupported (station) - 1;
}

uint32_t
RrpaaWifiManager::GetMinRate (RrpaaWifiRemoteStation *station)
{
  return 0;
}

void
RrpaaWifiManager::DoReportRtsFailed (WifiRemoteStation *st)
{
}

void
RrpaaWifiManager::DoReportDataFailed (WifiRemoteStation *st)
{
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  CheckInit(station);
  station->m_lastFrameFail = true;
  //CheckTimeout (station);
  station->m_counter--;
  station->m_failed++;
  RunBasicAlgorithm (station);
}

void
RrpaaWifiManager::DoReportRxOk (WifiRemoteStation *st,
                               double rxSnr, WifiMode txMode)
{
}

void
RrpaaWifiManager::DoReportRtsOk (WifiRemoteStation *st,
                                double ctsSnr, WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_DEBUG ("self=" << st << " rts ok");
}

void
RrpaaWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                 double ackSnr, WifiMode ackMode, double dataSnr)
{
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  CheckInit(station);
  station->m_lastFrameFail = false;
  CheckTimeout (station);
  station->m_counter--;
  RunBasicAlgorithm (station);
}
void
RrpaaWifiManager::DoReportFinalRtsFailed (WifiRemoteStation *st)
{
}
void
RrpaaWifiManager::DoReportFinalDataFailed (WifiRemoteStation *st)
{
}

WifiTxVector
RrpaaWifiManager::DoGetDataTxVector (WifiRemoteStation *st,
                                uint32_t size)
{
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  CheckInit(station);
  return WifiTxVector (GetSupported (station, station->m_rate),
                       station->m_power,
                       GetLongRetryCount (station),
                       GetShortGuardInterval (station),
                       Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()),
                       GetNumberOfTransmitAntennas (station),
                       GetStbc (station));
}
WifiTxVector
RrpaaWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  CheckInit(station);
  return WifiTxVector (GetSupported (st, 0),
                       GetDefaultTxPowerLevel (),
                       GetLongRetryCount (station),
                       GetShortGuardInterval (station),
                       Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()),
                       GetNumberOfTransmitAntennas (station),
                       GetStbc (station));
}

bool
RrpaaWifiManager::DoNeedRts (WifiRemoteStation *st,
                            Ptr<const Packet> packet, bool normally)
{
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  CheckInit(station);
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
  Time d = Simulator::Now () - station->m_lastReset;
  if (station->m_counter == 0 || d > m_timeout)
    {
      ResetCountersBasic (station);
    }
}

void
RrpaaWifiManager::RunBasicAlgorithm (RrpaaWifiRemoteStation *station)
{
  RngSeedManager::SetSeed (static_cast<uint32_t> (time (0)));
  Thresholds thresholds = GetThresholds (station, station->m_rate);
  double bploss = (double) station->m_failed / (double) thresholds.ewnd;
  double wploss = (double) (station->m_counter + station->m_failed) / (double) thresholds.ewnd;
  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
  if (bploss > thresholds.mtl && station->m_power < m_maxPower)
    {
      station->m_priTable[station->m_rate][station->m_power] /= 2;
      station->m_power++;
      m_powerChange(station->m_power, station->m_state->m_address);
      ResetCountersBasic (station);
    }
  else if (station->m_rate != 0 && bploss > thresholds.mtl && station->m_power == m_maxPower)
    {
      station->m_priTable[station->m_rate][station->m_power] /= 2;
      station->m_rate--;
      m_rateChange(station->m_rate, station->m_state->m_address);
      ResetCountersBasic (station);
    }
  if (wploss < thresholds.ori)
    {
      for (uint32_t i = 0; i <= station->m_rate; i++)
        {
          station->m_priTable[i][station->m_power] *= 1.0905;
    	  if (station->m_priTable[i][station->m_power] > 1)
    	    {
    	      station->m_priTable[i][station->m_power] = 1;
    	    }
        }
      double rand = uv->GetValue(0,1);
      if ((station->m_rate < station->m_nRate-1) && (rand < station->m_priTable[station->m_rate+1][station->m_power]) && station->m_power == m_maxPower)
        {
          station->m_rate++;
          m_rateChange(station->m_rate, station->m_state->m_address);
        }
      else
        {
    	  for (uint32_t i = m_maxPower; i > station->m_power; i--)
    	    {
              station->m_priTable[station->m_rate][i] *= 1.0905;
	      if (station->m_priTable[station->m_rate][i] > 1)
		{
		  station->m_priTable[station->m_rate][i] = 1;
		}
            }
    	  double rand = uv->GetValue(0,1);
    	  if (rand < station->m_priTable[station->m_rate][station->m_power-1])
    	    {
    	      station->m_power--;
    	      m_powerChange(station->m_power, station->m_state->m_address);
    	    }
        }
      ResetCountersBasic (station);
    }
  else if (bploss >= thresholds.ori && wploss < thresholds.mtl && station->m_power > 0)
    {
      for (uint32_t i = m_maxPower; i >= station->m_power; i--)
        {
          station->m_priTable[station->m_rate][i] *= 1.0905;
          if (station->m_priTable[station->m_rate][i] > 1)
            {
              station->m_priTable[station->m_rate][i] = 1;
            }
        }
      double rand = uv->GetValue(0,1);
      if (rand < station->m_priTable[station->m_rate][station->m_power-1])
	{
	  station->m_power--;
	  m_powerChange(station->m_power, station->m_state->m_address);
	}
      ResetCountersBasic (station);
    }
  if (station->m_counter == 0)
    {
      ResetCountersBasic (station);
    }
}

void
RrpaaWifiManager::ARts (RrpaaWifiRemoteStation *station)
{
  if (!station->m_rtsOn
      && station->m_lastFrameFail)
    {
      station->m_rtsWnd+=2;
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
  WifiMode mode = GetSupported (station, rate);
  return GetThresholds (station, mode);
}

bool
RrpaaWifiManager::IsLowLatency (void) const
{
  return true;
}

} // namespace ns3
