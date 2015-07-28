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

#include "prcs-wifi-manager.h"
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
NS_LOG_COMPONENT_DEFINE ("PrcsWifiManager");

namespace ns3 {

/**
 * Hold per-remote-station state for PRCS Wifi manager.
 *
 * This struct extends from WifiRemoteStation struct to hold additional
 * information required by the APARF Wifi manager
 */
struct PrcsWifiRemoteStation : public WifiRemoteStation
{
  uint32_t m_counter;           //!< Counter for transmission attempts.
  uint32_t m_nFailed;           //!< Number of failed transmission attempts.
  uint32_t m_rtsWnd;            //!< Window size for the ARts mechanism.
  uint32_t m_rtsCounter;        //!< Counter for rts transmission attempts.
  Time m_lastReset;             //!< Time of the last reset.
  bool m_rtsOn;                 //!< Check if ARts mechanism is on.
  bool m_lastFrameFail;         //!< Flag if the last frame sent has failed.
  bool m_initialized;           //!< For initializing variables.

  uint32_t m_nRate;             //!< Number of supported rates.

  uint32_t m_rate;              //!< Current rate.
  uint8_t m_power;              //!< Current power.
  double m_cst;                 //!< Current cs threshold.

  RrpaaThresholds m_thresholds; //!< Rrpaa thresholds for this station.

  double** m_pdTable;           //!< Probability table for power and rate changes.

  double m_interTxTime;         //!< Time between the transmission of the actual frame and the previous frame.
  double m_countBusy;           //!< Counter for BUSY periods.
};

/***************************************************************
 *         Listener for PHY events. Forwards to PrcsWifiManager
 ***************************************************************/

class PhyPrcsListener : public ns3::WifiPhyListener
{
public:
  PhyPrcsListener (ns3::PrcsWifiManager *prcs)
    : m_prcs (prcs)
  {
  }
  virtual ~PhyPrcsListener ()
  {
  }
  virtual void NotifyRxStart (Time duration)
  {
  }
  virtual void NotifyRxEndOk (void)
  {
  }
  virtual void NotifyRxEndError (void)
  {
  }
  virtual void NotifyTxStart (Time duration, double txPowerDbm)
  {
  }
  virtual void NotifyMaybeCcaBusyStart (Time duration)
  {
    m_prcs->NotifyMaybeCcaBusyStartNow (duration);
  }
  virtual void NotifySwitchingStart (Time duration)
  {
  }
  virtual void NotifySleep (void)
  {
  }
  virtual void NotifyWakeup (void)
  {
  }


private:
  ns3::PrcsWifiManager *m_prcs;
};

NS_OBJECT_ENSURE_REGISTERED (PrcsWifiManager);

TypeId
PrcsWifiManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::PrcsWifiManager")
    .SetParent<WifiRemoteStationManager> ()
    .SetGroupName ("Wifi")
    .AddConstructor<PrcsWifiManager> ()
    .AddAttribute ("Basic",
                   "If true the RRAA-BASIC algorithm will be used, otherwise the RRAA will be used.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&PrcsWifiManager::m_basic),
                   MakeBooleanChecker ())
    .AddAttribute ("Timeout",
                   "Timeout for the RRAA-BASIC loss estimation block (s).",
                   TimeValue (Seconds (0.5)),
                   MakeTimeAccessor (&PrcsWifiManager::m_timeout),
                   MakeTimeChecker ())
    .AddAttribute ("FrameLength",
                   "The data frame length used for calculating mode TxTime.",
                   UintegerValue (1420),
                   MakeUintegerAccessor (&PrcsWifiManager::m_frameLength),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("AckFrameLength",
                   "The ACK frame length used for calculating mode TxTime.",
                   DoubleValue (14),
                   MakeDoubleAccessor (&PrcsWifiManager::m_ackLength),
                   MakeDoubleChecker <uint32_t> ())
    .AddAttribute ("Alpha",
                   "Constant for calculating the MTL threshold.",
                   DoubleValue (1.25),
                   MakeDoubleAccessor (&PrcsWifiManager::m_alpha),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Beta",
                   "Constant for calculating the ORI threshold.",
                   DoubleValue (2),
                   MakeDoubleAccessor (&PrcsWifiManager::m_beta),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Tau",
                   "Constant for calculating the EWND size.",
                   DoubleValue (0.015),
                   MakeDoubleAccessor (&PrcsWifiManager::m_tau),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Gamma",
                   "Constant for Probabilistic Decision Table decrements.",
                   DoubleValue (2),
                   MakeDoubleAccessor (&PrcsWifiManager::m_gamma),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("Delta",
                   "Constant for Probabilistic Decision Table increments.",
                   DoubleValue (1.0905),
                   MakeDoubleAccessor (&PrcsWifiManager::m_delta),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("MinCst",
		   "The minimum allowable carrier-sense threshold.",
		   DoubleValue (-99.0),
		   MakeDoubleAccessor (&PrcsWifiManager::m_minCst),
		   MakeDoubleChecker<double> ())
    .AddTraceSource ("RateChange",
		    "The transmission rate has change.",
		    MakeTraceSourceAccessor (&PrcsWifiManager::m_rateChange),
		    "ns3::RrpaaWifiManager::RateChangeTracedCallback")
    .AddTraceSource ("PowerChange",
		    "The transmission power has change.",
		    MakeTraceSourceAccessor (&PrcsWifiManager::m_powerChange),
		    "ns3::RrpaaWifiManager::PowerChangeTracedCallback")
    .AddTraceSource ("CstChange",
                    "The carrier sense threshold has change",
                    MakeTraceSourceAccessor (&PrcsWifiManager::m_cstChange),
		    "ns3::RrpaaWifiManager::CSThresholdChangeTracedCallback")
  ;
  return tid;
}

PrcsWifiManager::PrcsWifiManager ()
  : m_countBusy(0),
    m_prevTime(0)
{
  NS_LOG_FUNCTION (this);
}

PrcsWifiManager::~PrcsWifiManager ()
{
  NS_LOG_FUNCTION (this);
}

void
PrcsWifiManager::SetupPhyPrcsListener (Ptr<WifiPhy> phy)
{
  NS_LOG_FUNCTION (this);
  m_phyPrcsListener = new PhyPrcsListener (this);
  phy->RegisterListener (m_phyPrcsListener);
}

void
PrcsWifiManager::SetupPhy (Ptr<WifiPhy> phy)
{
  NS_LOG_FUNCTION (this);
  m_phy = phy;
  m_nPower = phy->GetNTxPower();
  m_minPower = phy->GetTxPowerStart();
  m_maxPower = phy->GetTxPowerEnd();
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
  SetupPhyPrcsListener(phy);
}

void
PrcsWifiManager::SetupMac (Ptr<WifiMac> mac)
{
  NS_LOG_FUNCTION (this);
  m_sifs = mac->GetSifs();
  m_difs = m_sifs + 2*mac->GetSlot();
  WifiRemoteStationManager::SetupMac (mac);
}

Time
PrcsWifiManager::GetCalcTxTime (WifiMode mode) const
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
PrcsWifiManager::AddCalcTxTime (WifiMode mode, Time t)
{
  NS_LOG_FUNCTION (this);
  m_calcTxTime.push_back (std::make_pair (t, mode));
}

Thresholds
PrcsWifiManager::GetThresholds (PrcsWifiRemoteStation *station, WifiMode mode) const
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

WifiRemoteStation *
PrcsWifiManager::DoCreateStation (void) const
{
  NS_LOG_FUNCTION (this);
  PrcsWifiRemoteStation *station = new PrcsWifiRemoteStation ();
  station->m_rtsWnd = 0;
  station->m_rtsCounter = 0;
  station->m_interTxTime = 0;
  station->m_countBusy = 0;
  station->m_rtsOn = false;
  station->m_lastFrameFail = false;
  station->m_initialized = false;
  return station;
}

void
PrcsWifiManager::CheckInit (PrcsWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  if (!station->m_initialized)
    {
      //Note: we appear to be doing late initialization of the table
      //to make sure that the set of supported rates has been initialized
      //before we perform our own initialization.
      station->m_nRate = GetNSupported (station);
      //Initialize at minimal rate, maximal power and maximal cst.
      station->m_rate = 0;
      station->m_power = m_maxPower;
      station->m_cst = -99.0;
      m_rateChange (station->m_rate, station->m_state->m_address);
      m_powerChange (station->m_power, station->m_state->m_address);
      m_cstChange(station->m_cst, station->m_state->m_address);

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
PrcsWifiManager::InitThresholds (PrcsWifiRemoteStation *station)
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
PrcsWifiManager::ResetCountersBasic (PrcsWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  station->m_nFailed = 0;
  station->m_counter = GetThresholds (station, station->m_rate).ewnd;
  station->m_lastReset = Simulator::Now ();
}

void
PrcsWifiManager::DoReportRtsFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
}

void
PrcsWifiManager::DoReportDataFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  PrcsWifiRemoteStation *station = (PrcsWifiRemoteStation *) st;
  CheckInit (station);
  station->m_lastFrameFail = true;
  CheckTimeout (station);
  station->m_counter--;
  station->m_nFailed++;
  RunBasicAlgorithm (station);
}

void
PrcsWifiManager::DoReportRxOk (WifiRemoteStation *st,
                               double rxSnr, WifiMode txMode)
{
  NS_LOG_FUNCTION (this << st << rxSnr << txMode);
}

void
PrcsWifiManager::DoReportRtsOk (WifiRemoteStation *st,
                                double ctsSnr, WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_FUNCTION (this << st << ctsSnr << ctsMode << rtsSnr);
  NS_LOG_DEBUG ("self=" << st << " rts ok");
}

void
PrcsWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                 double ackSnr, WifiMode ackMode, double dataSnr)
{
  NS_LOG_FUNCTION (this << st << ackSnr << ackMode << dataSnr);
  PrcsWifiRemoteStation *station = (PrcsWifiRemoteStation *) st;
  CheckInit (station);
  station->m_lastFrameFail = false;
  CheckTimeout (station);
  station->m_counter--;
  RunBasicAlgorithm (station);
}

void
PrcsWifiManager::DoReportFinalRtsFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
}

void
PrcsWifiManager::DoReportFinalDataFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
}

WifiTxVector
PrcsWifiManager::DoGetDataTxVector (WifiRemoteStation *st,
                                uint32_t size)
{
  NS_LOG_FUNCTION (this << st << size);
  PrcsWifiRemoteStation *station = (PrcsWifiRemoteStation *) st;
  CheckInit (station);

  Ptr<YansWifiPhy> yansPhy = dynamic_cast<YansWifiPhy *> (PeekPointer (m_phy));
  yansPhy->SetCcaMode1Threshold(station->m_cst);

  return WifiTxVector (GetSupported (station, station->m_rate), station->m_power, GetLongRetryCount (station), GetShortGuardInterval (station), Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas ()), GetNumberOfTransmitAntennas (station), GetStbc (station));
}

WifiTxVector
PrcsWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  PrcsWifiRemoteStation *station = (PrcsWifiRemoteStation *) st;
  CheckInit (station);
  return WifiTxVector (GetSupported (st, 0), GetDefaultTxPowerLevel (), GetLongRetryCount (station), GetShortGuardInterval (station), Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas ()), GetNumberOfTransmitAntennas (station), GetStbc (station));
}

bool
PrcsWifiManager::DoNeedRts (WifiRemoteStation *st,
                            Ptr<const Packet> packet, bool normally)
{
  NS_LOG_FUNCTION (this << st << packet << normally);
  PrcsWifiRemoteStation *station = (PrcsWifiRemoteStation *) st;
  CheckInit (station);
  if (m_basic)
    {
      return normally;
    }
  ARts (station);
  return station->m_rtsOn;
}

void
PrcsWifiManager::CheckTimeout (PrcsWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  Time d = Simulator::Now () - station->m_lastReset;
  if (station->m_counter == 0 || d > m_timeout)
    {
      ResetCountersBasic (station);
    }
}

void
PrcsWifiManager::RunBasicAlgorithm (PrcsWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  RngSeedManager::SetSeed (static_cast<uint32_t> (time (0)));
  Thresholds thresholds = GetThresholds (station, station->m_rate);
  double bploss = (double) station->m_nFailed / (double) thresholds.ewnd;
  double wploss = (double) (station->m_counter + station->m_nFailed) / (double) thresholds.ewnd;
  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
/*
  double ploss = (double) station->m_nFailed / ((double) thresholds.ewnd - (double) station->m_counter);

  double tau = 2*(1-2*ploss) / ((1-2*ploss)*(16+1) + ploss*16*(1-pow(2*ploss,6)));

  double collisionProb = 1-pow((1-tau),1);




  if (collisionProb > (1.5*ploss))
      {
        station->m_cst +=2;
        m_cstChange(station->m_cst, station->m_state->m_address);
        NS_LOG_UNCOND (station->m_state->m_address << " CST " << station->m_cst);
      }
  else if (collisionProb < (ploss/1.5) && station->m_cst > m_minCst)
    	{
    	  station->m_cst-=2;
    	  m_cstChange(station->m_cst, station->m_state->m_address);
    	}
  NS_LOG_UNCOND(station->m_state->m_address << " loss " << ploss);
  NS_LOG_UNCOND(station->m_state->m_address << " tau " << tau);
  NS_LOG_UNCOND(station->m_state->m_address << " collision " << collisionProb);
*/
  WifiMode mode = GetSupported(station, station->m_rate);
  Time totalTxTime = GetCalcTxTime(mode) + m_difs + m_sifs;
  WifiTxVector txVector;
  txVector.SetMode(mode);
  Time dataTxTime = m_phy->CalculateTxDuration(m_frameLength, txVector, WIFI_PREAMBLE_LONG, m_phy->GetFrequency(), 0, 0);

  //double th = (1-ploss)*dataTxTime;

  //double cantNodes =

  double busyProb = station->m_countBusy/station->m_interTxTime;
  double weight = totalTxTime.GetSeconds() / dataTxTime.GetSeconds();
  if (busyProb > (weight*wploss))
    {
      station->m_cst +=2;
      m_cstChange(station->m_cst, station->m_state->m_address);
    }
  station->m_interTxTime = 0;
  station->m_countBusy = 0;

  if (bploss >= thresholds.mtl)
    {
      if (station->m_power < m_maxPower)
        {
          station->m_pdTable[station->m_rate][station->m_power] /= m_gamma;
          station->m_power++;
          m_powerChange (station->m_power, station->m_state->m_address);
          ResetCountersBasic (station);
        }
      else if (station->m_cst > m_minCst)
	{
	  station->m_cst-=2;
	  m_cstChange(station->m_cst, station->m_state->m_address);
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
  /*Thresholds thresholds = GetThresholds (station, station->m_rate);
    double bploss = (double) station->m_nFailed / (double) thresholds.ewnd;
    double wploss = (double) (station->m_counter + station->m_nFailed) / (double) thresholds.ewnd;
    Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
    WifiMode mode = GetSupported(station, station->m_rate);
    Time totalTxTime = GetCalcTxTime(mode) + m_difs + m_sifs;
    WifiTxVector txVector;
    txVector.SetMode(mode);
    Time dataTxTime = m_phy->CalculateTxDuration(m_frameLength, txVector, WIFI_PREAMBLE_LONG, m_phy->GetFrequency(), 0, 0);

    double busyProb = station->m_countBusy/station->m_interTxTime;
    double weight = totalTxTime.GetSeconds() / dataTxTime.GetSeconds();
    if (busyProb > (weight*wploss))
      {
        //station->m_cst +=2;
        //m_cstChange(station->m_cst, station->m_state->m_address);
      }
    station->m_interTxTime = 0;
    station->m_countBusy = 0;

    if (bploss > thresholds.mtl && station->m_power < m_nPower-1)
      {
        station->m_pdTable[station->m_rate][station->m_power] /= 2;
        station->m_power++;
        m_powerChange(station->m_power, station->m_state->m_address);
        //NS_LOG_UNCOND(station->m_state->m_address << " " << station->m_pdTable[station->m_rate][station->m_power]);
        ResetCountersBasic (station);
      }
    else if (bploss > thresholds.mtl && station->m_power == m_nPower-1)
      {
        //if (station->m_cst > m_minCst)
          //{
            //station->m_cst-=2;
            //m_cstChange(station->m_cst, station->m_state->m_address);
          //}
        //else
	if (station->m_rate != 0)
          {
            station->m_pdTable[station->m_rate][station->m_power] /= 2;
            station->m_rate--;
            m_rateChange(station->m_rate, station->m_state->m_address);
            ResetCountersBasic (station);
          }
      }
    if (wploss < thresholds.ori)
      {
        for (uint32_t i = 0; i <= station->m_rate; i++)
          {
            station->m_pdTable[i][station->m_power] *= 1.0905;
            if (station->m_pdTable[i][station->m_power] > 1)
              station->m_pdTable[i][station->m_power] = 1;
          }
        double rand = uv->GetValue(0,1);
        if ((station->m_rate < station->m_nRate-1) && (rand < station->m_pdTable[station->m_rate+1][station->m_power]) && station->m_power == m_nPower-1)
          {
            station->m_rate++;
            m_rateChange(station->m_rate, station->m_state->m_address);
          }
        else
          {
            for (uint32_t i = m_nPower-1; i > station->m_power; i--)
              {
                station->m_pdTable[station->m_rate][i] *= 1.0905;
                if (station->m_pdTable[station->m_rate][i] > 1)
                  station->m_pdTable[station->m_rate][i] = 1;
              }
            double rand = uv->GetValue(0,1);
            if (rand < station->m_pdTable[station->m_rate][station->m_power-1])
              {
                //NS_LOG_UNCOND(rand << " " << station->m_pdTable[station->m_rate][station->m_power-1]);
                station->m_power--;
                m_powerChange(station->m_power, station->m_state->m_address);
              }
          }
        ResetCountersBasic (station);
      }
    else if (bploss >= thresholds.ori && wploss < thresholds.mtl && station->m_power > 0)
      {
        for (uint32_t i = m_nPower-1; i >= station->m_power; i--)
          {
            station->m_pdTable[station->m_rate][i] *= 1.0905;
            if (station->m_pdTable[station->m_rate][i] > 1)
              station->m_pdTable[station->m_rate][i] = 1;
          }
       double rand = uv->GetValue(0,1);
       if (rand < station->m_pdTable[station->m_rate][station->m_power-1])
         {
           station->m_power--;
           m_powerChange(station->m_power, station->m_state->m_address);
           //NS_LOG_UNCOND(station->m_state->m_address << " " << station->m_pdTable[station->m_rate][station->m_power] << " " << rand);
         }
        ResetCountersBasic (station);
      }

    if (station->m_counter == 0)
      ResetCountersBasic (station);*/
}

void
PrcsWifiManager::ARts (PrcsWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  if (!station->m_rtsOn
      && station->m_lastFrameFail)
    {
      station->m_rtsWnd++;
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
PrcsWifiManager::GetThresholds (PrcsWifiRemoteStation *station,
                                uint32_t rate) const
{
  NS_LOG_FUNCTION (this << station << rate);
  WifiMode mode = GetSupported (station, rate);
  return GetThresholds (station, mode);
}

bool
PrcsWifiManager::IsLowLatency (void) const
{
  NS_LOG_FUNCTION (this);
  return true;
}

void
PrcsWifiManager::DoReportTxInit (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  Time currtime = Simulator::Now();
  PrcsWifiRemoteStation *station = (PrcsWifiRemoteStation *) st;
  if (m_prevTime > 0)
    {
      station->m_interTxTime += (currtime.GetSeconds() - m_prevTime.GetSeconds());
      station->m_countBusy += m_countBusy;
      m_countBusy = 0;
    }
  m_prevTime = currtime;
}

void
PrcsWifiManager::NotifyMaybeCcaBusyStartNow (Time duration)
{
  NS_LOG_FUNCTION (this << duration);
  m_countBusy += duration.ToDouble(Time::S);
}

} // namespace ns3
