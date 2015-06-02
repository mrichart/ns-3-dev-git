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

#include "prcs-wifi-manager.h"
#include "yans-wifi-phy.h"
#include "wifi-phy.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/uinteger.h"
#include "ns3/simulator.h"

#define Min(a,b) ((a < b) ? a : b)

NS_LOG_COMPONENT_DEFINE ("PrcsWifiManager");

namespace ns3 {

struct PrcsWifiRemoteStation : public WifiRemoteStation
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
  double m_cst;

  uint32_t m_nRate;

  double** m_priTable;

  PrcsThresholds m_thresholds;

  double m_ett;
  double m_countBusy;
  uint32_t m_countTx;
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
  virtual void NotifyTxStart (Time duration)
  {
  }
  virtual void NotifyMaybeCcaBusyStart (Time duration)
  {
    m_prcs->NotifyMaybeCcaBusyStartNow (duration);
  }
  virtual void NotifySwitchingStart (Time duration)
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
    .AddConstructor<PrcsWifiManager> ()
    .AddAttribute ("Basic",
                   "If true the RRAA-BASIC algorithm will be used, otherwise the RRAA wil be used",
                   BooleanValue (true),
                   MakeBooleanAccessor (&PrcsWifiManager::m_basic),
                   MakeBooleanChecker ())
    .AddAttribute ("Timeout",
                   "Timeout for the RRAA BASIC loss estimaton block (s)",
                   TimeValue (Seconds (0.05)),
                   MakeTimeAccessor (&PrcsWifiManager::m_timeout),
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
                   MakeTraceSourceAccessor(&PrcsWifiManager::m_rateChange))
   .AddTraceSource ("PowerChange",
                  "The transmission power has change",
                   MakeTraceSourceAccessor (&PrcsWifiManager::m_powerChange))
   .AddTraceSource ("CstChange",
                  "The carrier sense threshold has change",
                   MakeTraceSourceAccessor (&PrcsWifiManager::m_cstChange))
  ;
  return tid;
}


PrcsWifiManager::PrcsWifiManager ()
  : m_countBusy(0),
    m_prevTime(0),
    m_maxCst(-99.0)
{
}
PrcsWifiManager::~PrcsWifiManager ()
{
}

void
PrcsWifiManager::SetupPhyPrcsListener (Ptr<WifiPhy> phy)
{
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

void
PrcsWifiManager::AddThresholds (PrcsWifiRemoteStation *station, WifiMode mode, Thresholds th)
{
  NS_LOG_FUNCTION (this);
  station->m_thresholds.push_back (std::make_pair (th, mode));
}

WifiRemoteStation *
PrcsWifiManager::DoCreateStation (void) const
{
  PrcsWifiRemoteStation *station = new PrcsWifiRemoteStation ();
  station->m_initialized = false;
  station->m_rtsWnd = 0;
  station->m_rtsCounter = 0;
  station->m_rtsOn = false;
  station->m_lastFrameFail = false;
  return station;
}

void
PrcsWifiManager::CheckInit (PrcsWifiRemoteStation *station)
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
      station->m_cst = -99.0;
      //m_cstChange(station->m_cst, station->m_state->m_address);

      station->m_priTable = new double*[station->m_nRate];
      for (uint32_t i = 0; i < station->m_nRate; i++)
	{
	  station->m_priTable[i] = new double[m_nPower];
	  for (uint8_t j = 0; j < m_nPower; j++)
	    station->m_priTable[i][j] = 1;
	}

      station->m_initialized = true;

      station->m_thresholds = PrcsThresholds (station->m_nRate);
      InitThresholds (station);
      ResetCountersBasic(station);
    }
}

void
PrcsWifiManager::ResetCountersBasic (PrcsWifiRemoteStation *station)
{
  station->m_failed = 0;
  station->m_counter = GetThresholds (station, station->m_rate).ewnd;
  station->m_lastReset = Simulator::Now ();
}

uint32_t
PrcsWifiManager::GetMaxRate (PrcsWifiRemoteStation *station)
{
  return GetNSupported (station) - 1;
}
uint32_t
PrcsWifiManager::GetMinRate (PrcsWifiRemoteStation *station)
{
  return 0;
}


void
PrcsWifiManager::DoReportRtsFailed (WifiRemoteStation *st)
{
}

void
PrcsWifiManager::DoReportDataFailed (WifiRemoteStation *st)
{
  PrcsWifiRemoteStation *station = (PrcsWifiRemoteStation *) st;
  station->m_lastFrameFail = true;
  //CheckTimeout (station);
  station->m_counter--;
  station->m_failed++;
  RunBasicAlgorithm (station);
}
void
PrcsWifiManager::DoReportRxOk (WifiRemoteStation *st,
                               double rxSnr, WifiMode txMode)
{
}
void
PrcsWifiManager::DoReportRtsOk (WifiRemoteStation *st,
                                double ctsSnr, WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_DEBUG ("self=" << st << " rts ok");
}
void
PrcsWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                 double ackSnr, WifiMode ackMode, double dataSnr)
{
  PrcsWifiRemoteStation *station = (PrcsWifiRemoteStation *) st;
  station->m_lastFrameFail = false;
  //CheckTimeout (station);
  station->m_counter--;
  RunBasicAlgorithm (station);
}
void
PrcsWifiManager::DoReportFinalRtsFailed (WifiRemoteStation *st)
{
}
void
PrcsWifiManager::DoReportFinalDataFailed (WifiRemoteStation *st)
{
}

WifiTxVector
PrcsWifiManager::DoGetDataTxVector (WifiRemoteStation *st,
                                uint32_t size)
{
  PrcsWifiRemoteStation *station = (PrcsWifiRemoteStation *) st;
  if (!station->m_initialized)
    {
      ResetCountersBasic (station);
    }
  Ptr<YansWifiPhy> yansPhy = dynamic_cast<YansWifiPhy *> (PeekPointer (m_phy));
  yansPhy->SetCcaMode1Threshold(station->m_cst);
  return WifiTxVector (GetSupported (station, station->m_rate),
                       station->m_power,
                       GetLongRetryCount (station),
                       GetShortGuardInterval (station),
                       Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()),
                       GetNumberOfTransmitAntennas (station),
                       GetStbc (station));
}
WifiTxVector
PrcsWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  PrcsWifiRemoteStation *station = (PrcsWifiRemoteStation *) st;
  return WifiTxVector (GetSupported (st, 0),
                       GetDefaultTxPowerLevel (),
                       GetLongRetryCount (station),
                       GetShortGuardInterval (station),
                       Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()),
                       GetNumberOfTransmitAntennas (station),
                       GetStbc (station));
}

bool
PrcsWifiManager::DoNeedRts (WifiRemoteStation *st,
                            Ptr<const Packet> packet, bool normally)
{
  PrcsWifiRemoteStation *station = (PrcsWifiRemoteStation *) st;
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
  Time d = Simulator::Now () - station->m_lastReset;
  if (station->m_counter == 0 || d > m_timeout)
    {
      ResetCountersBasic (station);
    }
}

void
PrcsWifiManager::RunBasicAlgorithm (PrcsWifiRemoteStation *station)
{
  Thresholds thresholds = GetThresholds (station, station->m_rate);
  double bploss = (double) station->m_failed / (double) thresholds.ewnd;
  double wploss = (double) (station->m_counter + station->m_failed) / (double) thresholds.ewnd;
  Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
  double busyProb = station->m_countBusy/station->m_ett;
  if (busyProb > (1.52*wploss))
    {
      station->m_cst +=2;
      m_cstChange(station->m_cst, station->m_state->m_address);
    }
  station->m_ett = 0;
  station->m_countBusy = 0;
  station->m_countTx = 0;

  if (bploss > thresholds.mtl && station->m_power < m_nPower-1)
    {
      station->m_priTable[station->m_rate][station->m_power] /= 2;
      station->m_power++;
      m_powerChange(station->m_power, station->m_state->m_address);
      //NS_LOG_UNCOND(station->m_state->m_address << " " << station->m_priTable[station->m_rate][station->m_power]);
      ResetCountersBasic (station);
    }
  else if (bploss > thresholds.mtl && station->m_power == m_nPower-1)
    {
      if (station->m_cst > m_maxCst)
        {
          station->m_cst-=2;
          m_cstChange(station->m_cst, station->m_state->m_address);
        }
      else if (station->m_rate != 0)
        {
          station->m_priTable[station->m_rate][station->m_power] /= 2;
          station->m_rate--;
          m_rateChange(station->m_rate, station->m_state->m_address);
          ResetCountersBasic (station);
        }
    }
  if (wploss < thresholds.ori)
    {
      for (uint32_t i = 0; i <= station->m_rate; i++)
        {
          station->m_priTable[i][station->m_power] *= 1.0905;
          if (station->m_priTable[i][station->m_power] > 1)
            station->m_priTable[i][station->m_power] = 1;
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
                station->m_priTable[station->m_rate][i] = 1;
            }
          double rand = uv->GetValue(0,1);
          if (rand < station->m_priTable[station->m_rate][station->m_power-1])
            {
              //NS_LOG_UNCOND(rand << " " << station->m_priTable[station->m_rate][station->m_power-1]);
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
          station->m_priTable[station->m_rate][i] *= 1.0905;
          if (station->m_priTable[station->m_rate][i] > 1)
            station->m_priTable[station->m_rate][i] = 1;
        }
     double rand = uv->GetValue(0,1);
     if (rand < station->m_priTable[station->m_rate][station->m_power-1])
       {
         station->m_power--;
         m_powerChange(station->m_power, station->m_state->m_address);
         //NS_LOG_UNCOND(station->m_state->m_address << " " << station->m_priTable[station->m_rate][station->m_power] << " " << rand);
       }
      ResetCountersBasic (station);
    }

  if (station->m_counter == 0)
    ResetCountersBasic (station);
}

void
PrcsWifiManager::ARts (PrcsWifiRemoteStation *station)
{
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
  WifiMode mode = GetSupported (station, rate);
  return GetThresholds (station, mode);
}

bool
PrcsWifiManager::IsLowLatency (void) const
{
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
      station->m_ett += (currtime.GetSeconds() - m_prevTime.GetSeconds());
      station->m_countBusy += m_countBusy;
      m_countBusy = 0;
      station->m_countTx++;
    }
  m_prevTime = currtime;
//  if (station->m_countTx >=40)
//    {
//      double busyProb = station->m_countBusy/station->m_ett;
//      //NS_LOG_UNCOND(Simulator::Now().GetSeconds() << " " << station->m_state->m_address << " " << busyProb);
//      if (busyProb > (0.05))
//        {
//          station->m_cst +=2;
//          m_cstChange(station->m_cst, station->m_state->m_address);
//          Ptr<YansWifiPhy> yansPhy = dynamic_cast<YansWifiPhy *> (PeekPointer (m_phy));
//          yansPhy->SetCcaMode1Threshold(station->m_cst);
//          //yansPhy->SetEdThreshold(-70);
//        }
//      station->m_ett = 0;
//      station->m_countBusy = 0;
//      station->m_countTx = 0;
//    }
}

void
PrcsWifiManager::NotifyMaybeCcaBusyStartNow (Time duration)
{
  m_countBusy += duration.ToDouble(Time::S);
}

} // namespace ns3
