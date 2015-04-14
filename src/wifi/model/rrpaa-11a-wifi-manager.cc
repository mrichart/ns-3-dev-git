/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2004,2005,2006 INRIA
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
 * Author: Federico Maguolo <maguolof@dei.unipd.it>
 * Author: Matias Richart <mrichart@fing.edu.uy>
 */

#include "rrpaa-11a-wifi-manager.h"
#include "yans-wifi-phy.h"
#include "wifi-phy.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/simulator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"

#define Min(a,b) ((a < b) ? a : b)

NS_LOG_COMPONENT_DEFINE ("Rrpaa11aWifiManager");

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

  uint32_t m_rate;
  uint8_t m_power;

  uint32_t m_nRate;

  double** m_priTable;
};

NS_OBJECT_ENSURE_REGISTERED (Rrpaa11aWifiManager);

TypeId
Rrpaa11aWifiManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::Rrpaa11aWifiManager")
    .SetParent<WifiRemoteStationManager> ()
    .AddConstructor<Rrpaa11aWifiManager> ()
    .AddAttribute ("Basic",
                   "If true the RRAA-BASIC algorithm will be used, otherwise the RRAA wil be used",
                   BooleanValue (true),
                   MakeBooleanAccessor (&Rrpaa11aWifiManager::m_basic),
                   MakeBooleanChecker ())
    .AddAttribute ("Timeout",
                   "Timeout for the RRAA BASIC loss estimaton block (s)",
                   TimeValue (Seconds (0.05)),
                   MakeTimeAccessor (&Rrpaa11aWifiManager::m_timeout),
                   MakeTimeChecker ())
    .AddAttribute ("ewndFor54mbps",
                   "ewnd parameter for 54 Mbs data mode",
                   UintegerValue (40),
                   MakeUintegerAccessor (&Rrpaa11aWifiManager::m_ewndfor54),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("ewndFor48mbps",
                   "ewnd parameter for 48 Mbs data mode",
                   UintegerValue (40),
                   MakeUintegerAccessor (&Rrpaa11aWifiManager::m_ewndfor48),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("ewndFor36mbps",
                   "ewnd parameter for 36 Mbs data mode",
                   UintegerValue (40),
                   MakeUintegerAccessor (&Rrpaa11aWifiManager::m_ewndfor36),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("ewndFor24mbps",
                   "ewnd parameter for 24 Mbs data mode",
                   UintegerValue (40),
                   MakeUintegerAccessor (&Rrpaa11aWifiManager::m_ewndfor24),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("ewndFor18mbps",
                   "ewnd parameter for 18 Mbs data mode",
                   UintegerValue (20),
                   MakeUintegerAccessor (&Rrpaa11aWifiManager::m_ewndfor18),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("ewndFor12mbps",
                   "ewnd parameter for 12 Mbs data mode",
                   UintegerValue (20),
                   MakeUintegerAccessor (&Rrpaa11aWifiManager::m_ewndfor12),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("ewndFor9mbps",
                   "ewnd parameter for 9 Mbs data mode",
                   UintegerValue (10),
                   MakeUintegerAccessor (&Rrpaa11aWifiManager::m_ewndfor9),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("ewndFor6mbps",
                   "ewnd parameter for 6 Mbs data mode",
                   UintegerValue (6),
                   MakeUintegerAccessor (&Rrpaa11aWifiManager::m_ewndfor6),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("poriFor48mbps",
                   "Pori parameter for 48 Mbs data mode",
                   DoubleValue (0.047),
                   MakeDoubleAccessor (&Rrpaa11aWifiManager::m_porifor48),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("poriFor36mbps",
                   "Pori parameter for 36 Mbs data mode",
                   DoubleValue (0.125),
                   MakeDoubleAccessor (&Rrpaa11aWifiManager::m_porifor36),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("poriFor24mbps",
                   "Pori parameter for 24 Mbs data mode",
                   DoubleValue (0.1643),
                   MakeDoubleAccessor (&Rrpaa11aWifiManager::m_porifor24),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("poriFor18mbps",
                   "Pori parameter for 18 Mbs data mode",
                   DoubleValue (0.1301),
                   MakeDoubleAccessor (&Rrpaa11aWifiManager::m_porifor18),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("poriFor12mbps",
                   "Pori parameter for 12 Mbs data mode",
                   DoubleValue (0.1884),
                   MakeDoubleAccessor (&Rrpaa11aWifiManager::m_porifor12),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("poriFor9mbps",
                   "Pori parameter for 9 Mbs data mode",
                   DoubleValue (0.1434),
                   MakeDoubleAccessor (&Rrpaa11aWifiManager::m_porifor9),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("poriFor6mbps",
                   "Pori parameter for 6 Mbs data mode",
                   DoubleValue (0.1974),
                   MakeDoubleAccessor (&Rrpaa11aWifiManager::m_porifor6),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("pmtlFor54mbps",
                   "Pmtl parameter for 54 Mbs data mode",
                   DoubleValue (0.0951),
                   MakeDoubleAccessor (&Rrpaa11aWifiManager::m_pmtlfor54),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("pmtlFor48mbps",
                   "Pmtl parameter for 48 Mbs data mode",
                   DoubleValue (0.25),
                   MakeDoubleAccessor (&Rrpaa11aWifiManager::m_pmtlfor48),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("pmtlFor36mbps",
                   "Pmtl parameter for 36 Mbs data mode",
                   DoubleValue (0.3285),
                   MakeDoubleAccessor (&Rrpaa11aWifiManager::m_pmtlfor36),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("pmtlFor24mbps",
                   "Pmtl parameter for 24 Mbs data mode",
                   DoubleValue (0.2602),
                   MakeDoubleAccessor (&Rrpaa11aWifiManager::m_pmtlfor24),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("pmtlFor18mbps",
                   "Pmtl parameter for 18 Mbs data mode",
                   DoubleValue (0.3768),
                   MakeDoubleAccessor (&Rrpaa11aWifiManager::m_pmtlfor18),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("pmtlFor12mbps",
                   "Pmtl parameter for 12 Mbs data mode",
                   DoubleValue (0.2869),
                   MakeDoubleAccessor (&Rrpaa11aWifiManager::m_pmtlfor12),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("pmtlFor9mbps",
                   "Pmtl parameter for 9 Mbs data mode",
                   DoubleValue (0.3949),
                   MakeDoubleAccessor (&Rrpaa11aWifiManager::m_pmtlfor9),
                   MakeDoubleChecker<double> ())
   .AddTraceSource("RateChange",
                   "The transmission rate has change",
                   MakeTraceSourceAccessor(&Rrpaa11aWifiManager::m_rateChange),
                   "ns3::Rrpaa11aWifiManager::RateChangeTracedCallback")
   .AddTraceSource ("PowerChange",
                  "The transmission power has change",
                   MakeTraceSourceAccessor (&Rrpaa11aWifiManager::m_powerChange),
                   "ns3::Rrpaa11aWifiManager::PowerChangeTracedCallback")
  ;
  return tid;
}


Rrpaa11aWifiManager::Rrpaa11aWifiManager ()
{
}
Rrpaa11aWifiManager::~Rrpaa11aWifiManager ()
{
}

void
Rrpaa11aWifiManager::SetupPhy (Ptr<WifiPhy> phy)
{
  m_nPower = phy->GetNTxPower();
  WifiRemoteStationManager::SetupPhy (phy);
}

WifiRemoteStation *
Rrpaa11aWifiManager::DoCreateStation (void) const
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
Rrpaa11aWifiManager::ResetCountersBasic (RrpaaWifiRemoteStation *station)
{
  if (!station->m_initialized)
    {
      station->m_rate = GetMaxRate (station);
      m_rateChange(station->m_rate, station->m_state->m_address);
      station->m_power = m_nPower-1;
      m_powerChange(station->m_power, station->m_state->m_address);

      station->m_nRate = GetNSupported (station);
      station->m_priTable = new double*[station->m_nRate];
      for (uint32_t i = 0; i < station->m_nRate; i++)
        {
          station->m_priTable[i] = new double[m_nPower];
          for (uint8_t j = 0; j < m_nPower; j++)
            station->m_priTable[i][j] = 1;
        }

      station->m_initialized = true;
    }
  station->m_failed = 0;
  station->m_counter = GetThresholds (station, station->m_rate).ewnd;
  station->m_lastReset = Simulator::Now ();
}

uint32_t
Rrpaa11aWifiManager::GetMaxRate (RrpaaWifiRemoteStation *station)
{
  return GetNSupported (station) - 1;
}

uint32_t
Rrpaa11aWifiManager::GetMinRate (RrpaaWifiRemoteStation *station)
{
  return 0;
}

void
Rrpaa11aWifiManager::DoReportRtsFailed (WifiRemoteStation *st)
{
}

void
Rrpaa11aWifiManager::DoReportDataFailed (WifiRemoteStation *st)
{
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  station->m_lastFrameFail = true;
  //CheckTimeout (station);
  station->m_counter--;
  station->m_failed++;
  RunBasicAlgorithm (station);
}

void
Rrpaa11aWifiManager::DoReportRxOk (WifiRemoteStation *st,
                               double rxSnr, WifiMode txMode)
{
}

void
Rrpaa11aWifiManager::DoReportRtsOk (WifiRemoteStation *st,
                                double ctsSnr, WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_DEBUG ("self=" << st << " rts ok");
}

void
Rrpaa11aWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                 double ackSnr, WifiMode ackMode, double dataSnr)
{
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  station->m_lastFrameFail = false;
  CheckTimeout (station);
  station->m_counter--;
  RunBasicAlgorithm (station);
}
void
Rrpaa11aWifiManager::DoReportFinalRtsFailed (WifiRemoteStation *st)
{
}
void
Rrpaa11aWifiManager::DoReportFinalDataFailed (WifiRemoteStation *st)
{
}

WifiTxVector
Rrpaa11aWifiManager::DoGetDataTxVector (WifiRemoteStation *st,
                                uint32_t size)
{
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  if (!station->m_initialized)
    {
      ResetCountersBasic (station);
    }
  return WifiTxVector (GetSupported (station, station->m_rate),
                       station->m_power,
                       GetLongRetryCount (station),
                       GetShortGuardInterval (station),
                       Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()),
                       GetNumberOfTransmitAntennas (station),
                       GetStbc (station));
}
WifiTxVector
Rrpaa11aWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  return WifiTxVector (GetSupported (st, 0),
                       GetDefaultTxPowerLevel (),
                       GetLongRetryCount (station),
                       GetShortGuardInterval (station),
                       Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()),
                       GetNumberOfTransmitAntennas (station),
                       GetStbc (station));
}

bool
Rrpaa11aWifiManager::DoNeedRts (WifiRemoteStation *st,
                            Ptr<const Packet> packet, bool normally)
{
  RrpaaWifiRemoteStation *station = (RrpaaWifiRemoteStation *) st;
  if (m_basic)
    {
      return normally;
    }
  ARts (station);
  return station->m_rtsOn;
}

void
Rrpaa11aWifiManager::CheckTimeout (RrpaaWifiRemoteStation *station)
{
  Time d = Simulator::Now () - station->m_lastReset;
  if (station->m_counter == 0 || d > m_timeout)
    {
      ResetCountersBasic (station);
    }
}

void
Rrpaa11aWifiManager::RunBasicAlgorithm (RrpaaWifiRemoteStation *station)
{
  RngSeedManager::SetSeed (static_cast<uint32_t> (time (0)));
  ThresholdsItem thresholds = GetThresholds (station, station->m_rate);
  double bploss = (double) station->m_failed / (double) thresholds.ewnd;
  double wploss = (double) (station->m_counter + station->m_failed) / (double) thresholds.ewnd;
  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
  if (bploss > thresholds.pmtl && station->m_power < m_nPower-1)
    {
      station->m_priTable[station->m_rate][station->m_power] /= 2;
      station->m_power++;
      m_powerChange(station->m_power, station->m_state->m_address);
      ResetCountersBasic (station);
    }
  else if (station->m_rate != 0 && bploss > thresholds.pmtl && station->m_power == m_nPower-1)
    {
      station->m_priTable[station->m_rate][station->m_power] /= 2;
      station->m_rate--;
      m_rateChange(station->m_rate, station->m_state->m_address);
      ResetCountersBasic (station);
    }
  if (wploss < thresholds.pori)
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
      if ((station->m_rate < station->m_nRate-1) && (rand < station->m_priTable[station->m_rate+1][station->m_power]) && station->m_power == m_nPower-1)
        {
          station->m_rate++;
          m_rateChange(station->m_rate, station->m_state->m_address);
        }
      else
        {
    	  for (uint32_t i = m_nPower-1; i > station->m_power; i--)
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
  else if (bploss >= thresholds.pori && wploss < thresholds.pmtl && station->m_power > 0)
    {
      for (uint32_t i = m_nPower-1; i >= station->m_power; i--)
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
Rrpaa11aWifiManager::ARts (RrpaaWifiRemoteStation *station)
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

struct Rrpaa11aWifiManager::ThresholdsItem
Rrpaa11aWifiManager::GetThresholds (RrpaaWifiRemoteStation *station,
                                uint32_t rate) const
{
  WifiMode mode = GetSupported (station, rate);
  return GetThresholds (mode);
}

struct Rrpaa11aWifiManager::ThresholdsItem
Rrpaa11aWifiManager::GetThresholds (WifiMode mode) const
{
  switch (mode.GetDataRate () / 1000000)
    {
    case 54:
      {
        ThresholdsItem mode54 = {
          54000000,
          0.0,
          m_pmtlfor54,
          m_ewndfor54
        };
        return mode54;
      } break;
    case 48:
      {
        ThresholdsItem mode48 = {
          48000000,
          m_porifor48,
          m_pmtlfor48,
          m_ewndfor48
        };
        return mode48;
      } break;
    case 36:
      {
        ThresholdsItem mode36 = {
          36000000,
          m_porifor36,
          m_pmtlfor36,
          m_ewndfor36
        };
        return mode36;
      } break;
    case 24:
      {
        ThresholdsItem mode24 = {
          24000000,
          m_porifor24,
          m_pmtlfor24,
          m_ewndfor24
        };
        return mode24;
      } break;
    case 18:
      {
        ThresholdsItem mode18 = {
          18000000,
          m_porifor18,
          m_pmtlfor18,
          m_ewndfor18
        };
        return mode18;
      } break;
    case 12:
      {
        ThresholdsItem mode12 = {
          12000000,
          m_porifor12,
          m_pmtlfor12,
          m_ewndfor12
        };
        return mode12;
      } break;
    case 9:
      {
        ThresholdsItem mode9 =  {
          9000000,
          m_porifor9,
          m_pmtlfor9,
          m_ewndfor9
        };
        return mode9;
      } break;
    case 6:
      {
        ThresholdsItem mode6 =  {
          6000000,
          m_porifor6,
          0.8,
          m_ewndfor6
        };
        return mode6;
      } break;
    }
  NS_ASSERT_MSG (false, "Thresholds for an unknown mode are asked (" << mode << ")");
  return ThresholdsItem ();
}

bool
Rrpaa11aWifiManager::IsLowLatency (void) const
{
  return true;
}

} // namespace ns3
