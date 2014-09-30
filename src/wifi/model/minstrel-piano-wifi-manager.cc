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

#include "minstrel-piano-wifi-manager.h"
#include "yans-wifi-phy.h"
#include "wifi-phy.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/wifi-mac.h"
#include "ns3/assert.h"
#include <vector>
#include <stdio.h>
#include <fstream>
#include <iomanip>

#define Min(a,b) ((a < b) ? a : b)
#define Max(a,b) ((a > b) ? a : b)

NS_LOG_COMPONENT_DEFINE ("MinstrelPianoWifiManager");


namespace ns3 {


struct MinstrelPianoWifiRemoteStation : public WifiRemoteStation
{
  Time m_nextStatsUpdate;  ///< 10 times every second

  /**
   * To keep track of the current position in the our random sample table
   * going row by row from 1st column until the 10th column(Minstrel defines 10)
   * then we wrap back to the row 1 col 1.
   * note: there are many other ways to do this.
   */
  uint32_t m_col, m_index;
  uint32_t m_maxTpRate;  ///< the current throughput rate
  uint32_t m_maxTpRate2;  ///< second highest throughput rate
  uint32_t m_maxProbRate;  ///< rate with highest prob of success

  int m_packetCount;  ///< total number of packets as of now
  int m_sampleCount;  ///< how many packets we have sample so far

  bool m_isSampling;  ///< a flag to indicate we are currently sampling
  uint32_t m_sampleRate;  ///< current sample rate
  bool  m_sampleRateSlower;  ///< a flag to indicate sample rate is slower
  uint32_t m_currentRate;  ///< current rate we are using

  uint32_t m_shortRetry;  ///< short retries such as control packts
  uint32_t m_longRetry;  ///< long retries such as data packets
  uint32_t m_retry;  ///< total retries short + long
  uint32_t m_err;  ///< retry errors
  uint32_t m_txrate;  ///< current transmit rate

  bool m_initialized;  ///< for initializing tables

  bool m_sampleBest;  ///< a flag to indicate with witch rate we are sampling

  MinstrelPianoRate m_minstrelTable;  ///< minstrel table
  SampleRate m_sampleTable;  ///< sample table

  int m_pianoSampleCount;  ///< how many sample packets we have send so far
  int m_pianoRefCount;  ///< how many reference packets we have send so far

  uint8_t m_txpower;  ///< current transmit power level
  uint8_t m_txRefPower;  ///< current transmit power level for reference packets
  uint8_t m_txSamplePower;  ///< current transmit power level for sample packets
  uint8_t m_txDataPower;  ///< current transmit power level for data packets

  uint32_t m_nsupported;

  bool m_samplingPiano;
  Time m_prevTime;
  double m_ett;
  Time m_avgEtt;
  uint32_t m_countTx;
};

NS_OBJECT_ENSURE_REGISTERED (MinstrelPianoWifiManager);

TypeId
MinstrelPianoWifiManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MinstrelPianoWifiManager")
    .SetParent<WifiRemoteStationManager> ()
    .AddConstructor<MinstrelPianoWifiManager> ()
    .AddAttribute ("UpdateStatistics",
                   "The interval between updating statistics table ",
                   TimeValue (Seconds (0.1)),
                   MakeTimeAccessor (&MinstrelPianoWifiManager::m_updateStats),
                   MakeTimeChecker ())
    .AddAttribute ("LookAroundRate",
                   "the percentage to try other rates",
                   DoubleValue (10),
                   MakeDoubleAccessor (&MinstrelPianoWifiManager::m_lookAroundRate),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("EWMA",
                   "EWMA level",
                   DoubleValue (75),
                   MakeDoubleAccessor (&MinstrelPianoWifiManager::m_ewmaLevel),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("SegmentSize",
                   "The largest allowable segment size packet",
                   DoubleValue (6000),
                   MakeDoubleAccessor (&MinstrelPianoWifiManager::m_segmentSize),
                   MakeDoubleChecker <double> ())
    .AddAttribute ("SampleColumn",
                   "The number of columns used for sampling",
                   DoubleValue (10),
                   MakeDoubleAccessor (&MinstrelPianoWifiManager::m_sampleCol),
                   MakeDoubleChecker <double> ())
    .AddAttribute ("PacketLength",
                   "The packet length used for calculating mode TxTime",
                   DoubleValue (1420),
                   MakeDoubleAccessor (&MinstrelPianoWifiManager::m_pktLen),
                   MakeDoubleChecker <double> ())
    .AddAttribute ("LookAroundSamplePower",
                   "The percentage to try sample power",
                   DoubleValue (10),
                   MakeDoubleAccessor (&MinstrelPianoWifiManager::m_lookAroundSamplePower),
                   MakeDoubleChecker <double> ())
    .AddAttribute ("LookAroundRefPower",
                   "The percentage to try reference power",
                   DoubleValue (10),
                   MakeDoubleAccessor (&MinstrelPianoWifiManager::m_lookAroundRefPower),
                   MakeDoubleChecker <double> ())
    .AddAttribute ("PianoUpdateStats",
                   "How many reference and sample packets are needed for updating stats",
                   UintegerValue (10),
                   MakeUintegerAccessor (&MinstrelPianoWifiManager::m_pianoUpdateStats),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("IncrementLevel",
                   "How much levels to increase power each time",
                   UintegerValue (1),
                   MakeUintegerAccessor (&MinstrelPianoWifiManager::m_deltaInc),
                   MakeUintegerChecker <uint8_t> ())
    .AddAttribute ("DecrementLevel",
                   "How much levels to decrease power each time",
                   UintegerValue (1),
                   MakeUintegerAccessor (&MinstrelPianoWifiManager::m_deltaDec),
                   MakeUintegerChecker <uint8_t> ())
    .AddAttribute ("Delta",
                   "Power levels of separation between data and sample power",
                   UintegerValue (2),
                   MakeUintegerAccessor (&MinstrelPianoWifiManager::m_delta),
                   MakeUintegerChecker <uint8_t> ())
    .AddAttribute ("IncrementThreshold",
                   "The threshold between probabilities needed to increase power",
                   DoubleValue (0.05),
                   MakeDoubleAccessor (&MinstrelPianoWifiManager::m_thInc),
                   MakeDoubleChecker <double> ())
    .AddAttribute ("DecrementThreshold",
                   "The threshold between probabilities needed to decrease power",
                    DoubleValue (0.1),
                    MakeDoubleAccessor (&MinstrelPianoWifiManager::m_thDec),
                    MakeDoubleChecker <double> ())
    .AddTraceSource ("DoGetDataTxVector",
                    "A TxVector is ask for a new data frame",
                    MakeTraceSourceAccessor (&MinstrelPianoWifiManager::m_getDataTxVector))
    .AddTraceSource("PowerChange",
                    "The transmission power has change",
                    MakeTraceSourceAccessor(&MinstrelPianoWifiManager::m_powerChange))
    .AddTraceSource("RateChange",
                    "The transmission rate has change",
                    MakeTraceSourceAccessor(&MinstrelPianoWifiManager::m_rateChange))
  ;
  return tid;
}

MinstrelPianoWifiManager::MinstrelPianoWifiManager ()
  : m_countBusy (0),
    m_countRX (0),
    m_busyRatio (0),
    m_prevTime (Seconds(0))
{
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();

  //m_nsupported = 0;
}

MinstrelPianoWifiManager::~MinstrelPianoWifiManager ()
{
}

void
MinstrelPianoWifiManager::SetupPhy (Ptr<WifiPhy> phy)
{
  m_phy = phy;
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
MinstrelPianoWifiManager::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  return 1;
}

Time
MinstrelPianoWifiManager::GetCalcTxTime (WifiMode mode) const
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
MinstrelPianoWifiManager::AddCalcTxTime (WifiMode mode, Time t)
{
  m_calcTxTime.push_back (std::make_pair (t, mode));
}

WifiRemoteStation *
MinstrelPianoWifiManager::DoCreateStation (void) const
{
  MinstrelPianoWifiRemoteStation *station = new MinstrelPianoWifiRemoteStation ();

  station->m_nextStatsUpdate = Simulator::Now () + m_updateStats;
  station->m_col = 0;
  station->m_index = 0;
  station->m_maxTpRate = 0;
  station->m_maxTpRate2 = 0;
  station->m_maxProbRate = 0;
  station->m_packetCount = 0;
  station->m_sampleCount = 0;
  station->m_isSampling = false;
  station->m_sampleRate = 0;
  station->m_sampleRateSlower = false;
  station->m_currentRate = 0;
  station->m_shortRetry = 0;
  station->m_longRetry = 0;
  station->m_retry = 0;
  station->m_err = 0;
  station->m_txrate = 0;
  station->m_initialized = false;
  station->m_pianoRefCount = 0;
  station->m_pianoSampleCount = 100 / m_lookAroundSamplePower;
  station->m_samplingPiano = false;

  return station;
}

void
MinstrelPianoWifiManager::CheckInit (MinstrelPianoWifiRemoteStation *station)
{
  if (!station->m_initialized && GetNSupported (station) > 1)
    {
      // Note: we appear to be doing late initialization of the table
      // to make sure that the set of supported rates has been initialized
      // before we perform our own initialization.
      station->m_nsupported = GetNSupported (station);
      m_powerChange(station->m_txpower, station->m_state->m_address);
      station->m_txpower = m_nPower-1;
      m_rateChange(station->m_txrate, station->m_state->m_address);
      station->m_minstrelTable = MinstrelPianoRate (station->m_nsupported);
      station->m_sampleTable = SampleRate (station->m_nsupported, std::vector<uint32_t> (m_sampleCol));
      InitSampleTable (station);
      RateInit (station);
      station->m_initialized = true;

      PrintTable (station);
      PrintSampleTable (station);
    }
}

void
MinstrelPianoWifiManager::DoReportRxOk (WifiRemoteStation *st,
                                   double rxSnr, WifiMode txMode)
{
  NS_LOG_FUNCTION (this);
}

void
MinstrelPianoWifiManager::DoReportRtsFailed (WifiRemoteStation *st)
{
  MinstrelPianoWifiRemoteStation *station = (MinstrelPianoWifiRemoteStation *)st;
  NS_LOG_DEBUG ("DoReportRtsFailed m_txrate=" << station->m_txrate);

  station->m_shortRetry++;
}

void
MinstrelPianoWifiManager::DoReportRtsOk (WifiRemoteStation *st, double ctsSnr, WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_DEBUG ("self=" << st << " rts ok");
}

void
MinstrelPianoWifiManager::DoReportFinalRtsFailed (WifiRemoteStation *st)
{
  MinstrelPianoWifiRemoteStation *station = (MinstrelPianoWifiRemoteStation *)st;
  UpdateRetry (station);
  station->m_err++;
}

void
MinstrelPianoWifiManager::DoReportDataFailed (WifiRemoteStation *st)
{
  MinstrelPianoWifiRemoteStation *station = (MinstrelPianoWifiRemoteStation *)st;
  /**
   *
   * Retry Chain table is implemented here
   *
   * Try |         LOOKAROUND RATE              | NORMAL RATE
   *     | random < best    | random > best     |
   * --------------------------------------------------------------
   *  1  | Best throughput  | Random rate       | Best throughput
   *  2  | Random rate      | Best throughput   | Next best throughput
   *  3  | Best probability | Best probability  | Best probability
   *  4  | Lowest Baserate  | Lowest baserate   | Lowest baserate
   *
   * Note: For clarity, multiple blocks of if's and else's are used
   * After a failing 7 times, DoReportFinalDataFailed will be called
   */

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  station->m_longRetry++;
  station->m_minstrelTable[station->m_txrate].numRateAttempt++;

  //In the article the attempts are increased only by one and not by the number of the attempted retries.
  if (station->m_txpower == station->m_minstrelTable[station->m_txrate].refPower)
    {
      station->m_minstrelTable[station->m_txrate].numRefAttempt++;
    }
  if (station->m_txpower == station->m_minstrelTable[station->m_txrate].dataPower)
    {
      station->m_minstrelTable[station->m_txrate].numDataAttempt++;
    }
  if (station->m_txpower == station->m_minstrelTable[station->m_txrate].samplePower)
    {
      station->m_minstrelTable[station->m_txrate].numSampleAttempt++;
    }

  //PrintTable (station);

  NS_LOG_DEBUG ("DoReportDataFailed " << station << " rate " << station->m_txrate << " power " << (int)station->m_txpower << " longRetry " << station->m_longRetry);

  /// for normal rate, we're not currently sampling random rates
  if (!station->m_isSampling)
    {
      NS_LOG_DEBUG ("Failed with normal rate: current=" << station->m_txrate <<
                    ", sample=" << station->m_sampleRate <<
                    ", maxTp=" << station->m_maxTpRate <<
                    ", maxTp2=" << station->m_maxTpRate2 <<
                    ", maxProb=" << station->m_maxProbRate);
      /// use best throughput rate
      if (station->m_longRetry < station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount)
        {
          NS_LOG_DEBUG (" More retries left for the maximum throughput rate.");
          station->m_txrate = station->m_maxTpRate;
        }

      /// use second best throughput rate
      else if (station->m_longRetry <= (station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount +
                                         station->m_minstrelTable[station->m_maxTpRate2].adjustedRetryCount))
        {
          NS_LOG_DEBUG (" More retries left for the second maximum throughput rate.");
          station->m_txrate = station->m_maxTpRate2;
        }

      /// use best probability rate
      else if (station->m_longRetry <= (station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount +
                                         station->m_minstrelTable[station->m_maxTpRate2].adjustedRetryCount +
                                         station->m_minstrelTable[station->m_maxProbRate].adjustedRetryCount))
        {
          NS_LOG_DEBUG (" More retries left for the maximum probability rate.");
          station->m_txrate = station->m_maxProbRate;
        }

      /// use lowest base rate
      else if (station->m_longRetry > (station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount +
                                        station->m_minstrelTable[station->m_maxTpRate2].adjustedRetryCount +
                                        station->m_minstrelTable[station->m_maxProbRate].adjustedRetryCount))
        {
          NS_LOG_DEBUG (" More retries left for the base rate.");
          station->m_txrate = 0;
        }
      if (station->m_samplingPiano)
        station->m_txpower = station->m_minstrelTable[station->m_txrate].samplePower;
      else
        station->m_txpower = station->m_minstrelTable[station->m_txrate].dataPower;
    }

  /// for look-around rate, we're currently sampling random rates
  else
    {
      NS_LOG_DEBUG ("Failed with look around rate: current=" << station->m_txrate <<
                          ", sample=" << station->m_sampleRate <<
                          ", maxTp=" << station->m_maxTpRate <<
                          ", maxTp2=" << station->m_maxTpRate2 <<
                          ", maxProb=" << station->m_maxProbRate);
      /// current sampling rate is slower than the current best rate
      if (station->m_sampleRateSlower)
        {
          NS_LOG_DEBUG ("Look around rate is slower than the maximum throughput rate.");
          /// use best throughput rate
          if (station->m_longRetry < station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount)
            {
              NS_LOG_DEBUG (" More retries left for the maximum throughput rate.");
              station->m_txrate = station->m_maxTpRate;
            }

          ///   use random rate
          else if (station->m_longRetry <= (station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount +
                                             station->m_minstrelTable[station->m_sampleRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG (" More retries left for the sampling rate.");
              station->m_txrate = station->m_sampleRate;
            }

          /// use max probability rate
          else if (station->m_longRetry <= (station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount +
                                             station->m_minstrelTable[station->m_sampleRate].adjustedRetryCount +
                                             station->m_minstrelTable[station->m_maxProbRate].adjustedRetryCount ))
            {
              NS_LOG_DEBUG (" More retries left for the maximum probability rate.");
              station->m_txrate = station->m_maxProbRate;
            }

          /// use lowest base rate
          else if (station->m_longRetry > (station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount +
                                            station->m_minstrelTable[station->m_sampleRate].adjustedRetryCount +
                                            station->m_minstrelTable[station->m_maxProbRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG (" More retries left for the base rate.");
              station->m_txrate = 0;
            }
        }

      /// current sampling rate is better than current best rate
      else
        {
          NS_LOG_DEBUG ("Look around rate is faster than the maximum throughput rate.");
          /// use random rate
          if (station->m_longRetry < station->m_minstrelTable[station->m_sampleRate].adjustedRetryCount)
            {
              NS_LOG_DEBUG (" More retries left for the sampling rate.");
              station->m_txrate = station->m_sampleRate;
            }

          /// use the best throughput rate
          else if (station->m_longRetry <= (station->m_minstrelTable[station->m_sampleRate].adjustedRetryCount +
                                            station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG (" More retries left for the maximum throughput rate.");
              station->m_txrate = station->m_maxTpRate;
            }

          /// use the best probability rate
          else if (station->m_longRetry <= (station->m_minstrelTable[station->m_sampleRate].adjustedRetryCount +
                                            station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount +
                                            station->m_minstrelTable[station->m_maxProbRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG (" More retries left for the maximum probability rate.");
              station->m_txrate = station->m_maxProbRate;
            }

          /// use the lowest base rate
          else if (station->m_longRetry > (station->m_minstrelTable[station->m_sampleRate].adjustedRetryCount +
                                           station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount +
                                           station->m_minstrelTable[station->m_maxProbRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG (" More retries left for the base rate.");
              station->m_txrate = 0;
            }
        }
      station->m_txpower = station->m_minstrelTable[station->m_txrate].refPower;
    }

  m_rateChange(station->m_txrate, station->m_state->m_address);
  m_powerChange(station->m_txpower, station->m_state->m_address);
}

void
MinstrelPianoWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                     double ackSnr, WifiMode ackMode, double dataSnr)
{
  NS_LOG_FUNCTION (st << ackSnr << ackMode << dataSnr);
  MinstrelPianoWifiRemoteStation *station = (MinstrelPianoWifiRemoteStation *) st;

  station->m_isSampling = false;
  station->m_sampleRateSlower = false;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  NS_LOG_DEBUG ("DoReportDataOk m_txrate = " << station->m_txrate << ", power = " << (int) station->m_txpower <<
                ", attempt = " << station->m_minstrelTable[station->m_txrate].numRateAttempt <<
                ", success = " << station->m_minstrelTable[station->m_txrate].numRateSuccess <<
                ", ref attempt = " << station->m_minstrelTable[station->m_txrate].numRefAttempt <<
                ", ref success = " << station->m_minstrelTable[station->m_txrate].numRefSuccess <<
                ", data attempt = " << station->m_minstrelTable[station->m_txrate].numDataAttempt <<
                ", data success = " << station->m_minstrelTable[station->m_txrate].numDataSuccess <<
                ", sample attempt = " << station->m_minstrelTable[station->m_txrate].numSampleAttempt <<
                ", sample success = " << station->m_minstrelTable[station->m_txrate].numSampleSuccess << " (before update).");

  station->m_minstrelTable[station->m_txrate].numRateSuccess++;
  station->m_minstrelTable[station->m_txrate].numRateAttempt++;

  if (station->m_txpower == station->m_minstrelTable[station->m_txrate].refPower)
    {
      station->m_minstrelTable[station->m_txrate].numRefSuccess++;
      station->m_minstrelTable[station->m_txrate].numRefAttempt++;
    }
  if (station->m_txpower == station->m_minstrelTable[station->m_txrate].dataPower)
    {
      station->m_minstrelTable[station->m_txrate].numDataSuccess++;
      station->m_minstrelTable[station->m_txrate].numDataAttempt++;
    }
  if (station->m_txpower == station->m_minstrelTable[station->m_txrate].samplePower)
    {
      station->m_minstrelTable[station->m_txrate].numSampleSuccess++;
      station->m_minstrelTable[station->m_txrate].numSampleAttempt++;
    }

  NS_LOG_DEBUG ("DoReportDataOk m_txrate = " << station->m_txrate << ", power = " << (int) station->m_txpower <<
                  ", attempt = " << station->m_minstrelTable[station->m_txrate].numRateAttempt <<
                  ", success = " << station->m_minstrelTable[station->m_txrate].numRateSuccess <<
                  ", ref attempt = " << station->m_minstrelTable[station->m_txrate].numRefAttempt <<
                  ", ref success = " << station->m_minstrelTable[station->m_txrate].numRefSuccess <<
                  ", data attempt = " << station->m_minstrelTable[station->m_txrate].numDataAttempt <<
                  ", data success = " << station->m_minstrelTable[station->m_txrate].numDataSuccess <<
                  ", sample attempt = " << station->m_minstrelTable[station->m_txrate].numSampleAttempt <<
                  ", sample success = " << station->m_minstrelTable[station->m_txrate].numSampleSuccess << " (after update).");

  UpdateRetry (station); //updates m_retry and set long and short retry to 0

  station->m_packetCount++;

  UpdateStats (station);
  UpdatePowerStats(station);

  if (station->m_nsupported >= 1)
    {
      SetRatePower(station);
      m_rateChange(station->m_txrate, station->m_state->m_address);
      m_powerChange(station->m_txpower, station->m_state->m_address);
    }

  PrintTable (station);
}

//this function is used to say that all retry attempts fail. Before calling this function, DoReportDataFailed was called.

void
MinstrelPianoWifiManager::DoReportFinalDataFailed (WifiRemoteStation *st)
{
  MinstrelPianoWifiRemoteStation *station = (MinstrelPianoWifiRemoteStation *) st;

  station->m_isSampling = false;
  station->m_sampleRateSlower = false;

  UpdateRetry (station);

  station->m_err++;

  UpdateStats (station);
  UpdatePowerStats(station);

  if (station->m_nsupported >= 1)
    {
      SetRatePower(station);
      m_rateChange(station->m_txrate, station->m_state->m_address);
      m_powerChange(station->m_txpower, station->m_state->m_address);
    }

  PrintTable (station);
}

void
MinstrelPianoWifiManager::UpdateRetry (MinstrelPianoWifiRemoteStation *station)
{
  for (uint32_t i = 0; i < station->m_nsupported; i++)
    {
      station->m_retry = station->m_shortRetry + station->m_longRetry;
      station->m_shortRetry = 0;
      station->m_longRetry = 0;
    }
}

WifiTxVector
MinstrelPianoWifiManager::DoGetDataTxVector (WifiRemoteStation *st,
                                    uint32_t size)
{
  MinstrelPianoWifiRemoteStation *station = (MinstrelPianoWifiRemoteStation *) st;
  if (!station->m_initialized)
    {
      CheckInit (station);

      /// start the rate at half way
      station->m_txrate = station->m_nsupported / 2;
      m_rateChange(station->m_txrate, station->m_state->m_address);
    }

  WifiTxVector vector =  WifiTxVector (GetSupported (station, station->m_txrate),
                                       station->m_txpower,
                                       GetLongRetryCount (station),
                                       GetShortGuardInterval (station),
                                       Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()),
                                       GetNumberOfTransmitAntennas (station),
                                       GetStbc (station));
  m_getDataTxVector(vector);
  return vector;
}

WifiTxVector
MinstrelPianoWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  MinstrelPianoWifiRemoteStation *station = (MinstrelPianoWifiRemoteStation *) st;
  NS_LOG_DEBUG ("DoGetRtsMode m_txrate=" << station->m_txrate);

  return WifiTxVector (GetSupported (station, 0),
                        GetDefaultTxPowerLevel (),
                        GetShortRetryCount (station),
                        GetShortGuardInterval (station),
                        Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()),
                        GetNumberOfTransmitAntennas (station),
                        GetStbc (station));
}

bool
MinstrelPianoWifiManager::DoNeedDataRetransmission (WifiRemoteStation *st, Ptr<const Packet> packet, bool normally)
{
  MinstrelPianoWifiRemoteStation *station = (MinstrelPianoWifiRemoteStation *)st;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return normally;
    }

  if (!station->m_isSampling)
    {
      if (station->m_longRetry > (station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount +
                                  station->m_minstrelTable[station->m_maxTpRate2].adjustedRetryCount +
                                  station->m_minstrelTable[station->m_maxProbRate].adjustedRetryCount +
                                  station->m_minstrelTable[0].adjustedRetryCount))
        {
          return false;
        }
      else
        {
          return true;
        }
    }
  else
    {
      if (station->m_longRetry > (station->m_minstrelTable[station->m_sampleRate].adjustedRetryCount +
                                  station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount +
                                  station->m_minstrelTable[station->m_maxProbRate].adjustedRetryCount +
                                  station->m_minstrelTable[0].adjustedRetryCount))
        {
          return false;
        }
      else
        {
          return true;
        }
    }
}

bool
MinstrelPianoWifiManager::IsLowLatency (void) const
{
  return true;
}

uint32_t
MinstrelPianoWifiManager::GetNextSample (MinstrelPianoWifiRemoteStation *station)
{
  uint32_t bitrate;
  bitrate = station->m_sampleTable[station->m_index][station->m_col];
  station->m_index++;

  /// bookeeping for m_index and m_col variables
  if (station->m_index > (station->m_nsupported - 2))
    {
      station->m_index = 0;
      station->m_col++;
      if (station->m_col >= m_sampleCol)
        {
          station->m_col = 0;
        }
    }
  return bitrate;
}

void
MinstrelPianoWifiManager::SetRatePower (MinstrelPianoWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);

  uint32_t rate;
  uint8_t power;

  station->m_samplingPiano = false;

  if ((station->m_sampleCount + station->m_packetCount) == 0)
    {
      rate = 0;
      power = m_nPower;
    }
  else
    {
      /// for determining when to try a sample rate
      int coinFlip = m_uniformRandomVariable->GetInteger (0, 100) % 2;

      /**
       * if we are below the target of look around rate percentage, look around
       * note: do it randomly by flipping a coin instead sampling
       * all at once until it reaches the look around rate
       */
      if ( (((100 * station->m_sampleCount) / (station->m_sampleCount + station->m_packetCount )) < m_lookAroundRate)
           && (coinFlip == 1) )
        {
          NS_LOG_DEBUG ("Using look around rate");
          /// now go through the table and find an index rate
          rate = GetNextSample (station);


          /**
           * This if condition is used to make sure that we don't need to use
           * the sample rate it is the same as our current rate
           */
          if (rate != station->m_maxTpRate && rate != station->m_txrate)
            {

              /// start sample count
              station->m_sampleCount++;

              /// set flag that we are currently sampling
              station->m_isSampling = true;

              /// bookeeping for resetting stuff
              if (station->m_packetCount >= 10000)
                {
                  station->m_sampleCount = 0;
                  station->m_packetCount = 0;
                }

              /// error check
              if (rate >= station->m_nsupported)
                {
                  NS_LOG_DEBUG ("ALERT!!! ERROR");
                }

              /// set the rate that we're currently sampling
              station->m_sampleRate = rate;

              // we make the random not to be the best throughput
              if (station->m_sampleRate == station->m_maxTpRate)
                {
                  station->m_sampleRate = station->m_maxTpRate2;
                }

              /// is this rate slower than the current best rate
              station->m_sampleRateSlower =
                (station->m_minstrelTable[rate].perfectTxTime > station->m_minstrelTable[station->m_maxTpRate].perfectTxTime);

              /// using the best rate instead
              if (station->m_sampleRateSlower)
                {
                  NS_LOG_DEBUG ("The next look around rate is slower than the maximum throughput rate, continue with the maximum throughput rate: " <<
                                station->m_maxTpRate << "(" << GetSupported (station, station->m_maxTpRate) << ")");
                  rate =  station->m_maxTpRate;
                }
            }

          /*
           * When Minstrel sampling use reference power.
           * Use the same power for all rates in the retry chain.
           */
          power = station->m_minstrelTable[rate].refPower;
          NS_LOG_DEBUG ("With sample rate use reference power: " << (int) power);
        }

      ///   continue using the best rate
      else
        {
          NS_LOG_DEBUG ("Continue using the maximum throughput rate: " << station->m_maxTpRate << "(" << GetSupported (station, station->m_maxTpRate) << ")");

          if (station->m_pianoSampleCount > 0)
            {
              NS_LOG_DEBUG ("Setup data packet");
              station->m_pianoSampleCount--;
              rate = station->m_maxTpRate;
              power = station->m_minstrelTable[rate].dataPower;
              NS_LOG_DEBUG("Data packet: rate= " << rate << "(" << GetSupported (station, rate) << ") power= " << (int)power);
            }
          /**
           * if minstrel is not sampling and
           * if we are below the target of look around power percentage, look around
           */
          else
            {
              NS_LOG_DEBUG ("Setup power sampling packet");
              station->m_pianoSampleCount = 100 / m_lookAroundSamplePower;

              rate = (station->m_sampleBest? station->m_maxTpRate : station->m_maxTpRate2);
              station->m_samplingPiano = true;
              station->m_sampleBest = !station->m_sampleBest;

              /// for determining when to try a sample power
              int coinFlip = m_uniformRandomVariable->GetInteger (0, 100) % 2;

              if (coinFlip == 1)
                {
                  power = station->m_minstrelTable[rate].samplePower;
                }
              else
                {
                  power = station->m_minstrelTable[rate].refPower;
                }
              NS_LOG_DEBUG("Sample power packet: rate= " << rate << "(" << GetSupported (station, rate) << ") power= " << (int)power);
            }
        }
    }
  station->m_txrate = rate;
  station->m_txpower = power;
}

void
MinstrelPianoWifiManager::UpdateStats (MinstrelPianoWifiRemoteStation *station)
{
  if (Simulator::Now () <  station->m_nextStatsUpdate)
    {
      return;
    }

  if (!station->m_initialized)
    {
      return;
    }
  NS_LOG_FUNCTION (this);

  station->m_nextStatsUpdate = Simulator::Now () + m_updateStats;
  NS_LOG_DEBUG ("Next update at " << station->m_nextStatsUpdate);
  NS_LOG_DEBUG ("Currently using rate: " << station->m_txrate << " (" << GetSupported (station, station->m_txrate) << ")");

  Time txTime;
  uint32_t tempProb;

  NS_LOG_DEBUG ("Index-Rate\t\tAttempt\tSuccess");
  for (uint32_t i = 0; i < station->m_nsupported; i++)
    {

      /// calculate the perfect tx time for this rate
      txTime = station->m_minstrelTable[i].perfectTxTime;

      /// just for initialization
      if (txTime.GetMicroSeconds () == 0)
        {
          txTime = Seconds (1);
        }

      NS_LOG_DEBUG (i << " " << GetSupported (station, i) <<
                          "\t" << station->m_minstrelTable[i].numRateAttempt <<
                          "\t" << station->m_minstrelTable[i].numRateSuccess);

      /// if we've attempted something
      if (station->m_minstrelTable[i].numRateAttempt)
        {
          /**
           * calculate the probability of success
           * assume probability scales from 0 to 18000
           */
          tempProb = (station->m_minstrelTable[i].numRateSuccess * 18000) / station->m_minstrelTable[i].numRateAttempt;

          /// bookeeping

          station->m_minstrelTable[i].prob = tempProb;

          /// ewma probability (cast for gcc 3.4 compatibility)
          tempProb = static_cast<uint32_t> (((tempProb * (100 - m_ewmaLevel)) + (station->m_minstrelTable[i].ewmaProb * m_ewmaLevel) ) / 100);

          station->m_minstrelTable[i].ewmaProb = tempProb;

          /// calculating throughput
          station->m_minstrelTable[i].throughput = tempProb * (1000000 / txTime.GetMicroSeconds ());

        }

      /// bookeeping

      station->m_minstrelTable[i].numRateSuccess = 0;
      station->m_minstrelTable[i].numRateAttempt = 0;

      /// Sample less often below 10% and  above 95% of success
      if ((station->m_minstrelTable[i].ewmaProb > 17100) || (station->m_minstrelTable[i].ewmaProb < 1800))
        {
          /**
           * See: http://wireless.kernel.org/en/developers/Documentation/mac80211/RateControl/minstrel/
           *
           * Analysis of information showed that the system was sampling too hard at some rates.
           * For those rates that never work (54mb, 500m range) there is no point in sending 10 sample packets (< 6 ms time).
           * Consequently, for the very very low probability rates, we sample at most twice.
           */
          if (station->m_minstrelTable[i].retryCount > 2)
            {
              station->m_minstrelTable[i].adjustedRetryCount = 2;
            }
          else
            {
              station->m_minstrelTable[i].adjustedRetryCount = station->m_minstrelTable[i].retryCount;
            }
        }
      else
        {
          station->m_minstrelTable[i].adjustedRetryCount = station->m_minstrelTable[i].retryCount;
        }

      /// if it's 0 allow one retry limit
      if (station->m_minstrelTable[i].adjustedRetryCount == 0)
        {
          station->m_minstrelTable[i].adjustedRetryCount = 1;
        }
    }
  NS_LOG_DEBUG ("Attempt/success resetted to 0");

  uint32_t max_prob = 0, index_max_prob = 0, max_tp = 0, index_max_tp = 0, index_max_tp2 = 0;

  /// go find max throughput, second maximum throughput, high probability succ
  NS_LOG_DEBUG ("Finding the maximum throughput, second maximum throughput, and highest probability");
  NS_LOG_DEBUG ("Index-Rate\t\tT-put\tEWMA");
  for (uint32_t i = 0; i < station->m_nsupported; i++)
    {
      NS_LOG_DEBUG (i << " " << GetSupported (station, i) <<
                    "\t" << station->m_minstrelTable[i].throughput <<
                    "\t" << station->m_minstrelTable[i].ewmaProb);

      if (max_tp < station->m_minstrelTable[i].throughput)
        {
          index_max_tp = i;
          max_tp = station->m_minstrelTable[i].throughput;
        }

      if (max_prob < station->m_minstrelTable[i].ewmaProb)
        {
          index_max_prob = i;
          max_prob = station->m_minstrelTable[i].ewmaProb;
        }
    }


  max_tp = 0;
  /// find the second highest max
  for (uint32_t i = 0; i < station->m_nsupported; i++)
    {
      if ((i != index_max_tp) && (max_tp < station->m_minstrelTable[i].throughput))
        {
          index_max_tp2 = i;
          max_tp = station->m_minstrelTable[i].throughput;
        }
    }

  station->m_maxTpRate = index_max_tp;
  station->m_maxTpRate2 = index_max_tp2;
  station->m_maxProbRate = index_max_prob;
  station->m_currentRate = index_max_tp;

  if (index_max_tp > station->m_txrate)
    {
      station->m_txrate = index_max_tp;
      m_rateChange(station->m_txrate, station->m_state->m_address);
    }

  NS_LOG_DEBUG ("max throughput=" << index_max_tp << "(" << GetSupported (station, index_max_tp) <<
                ")\tsecond max throughput=" << index_max_tp2 << "(" << GetSupported (station, index_max_tp2) <<
                ")\tmax prob=" << index_max_prob << "(" << GetSupported (station, index_max_prob) << ")");

}

void
MinstrelPianoWifiManager::UpdatePowerStats (MinstrelPianoWifiRemoteStation *station)
{
  if (!station->m_initialized)
    {
      return;
    }
  NS_LOG_FUNCTION (this);

  Time txTime;
  uint32_t refTempProb;
  uint32_t dataTempProb;
  uint32_t sampleTempProb;

  for (uint32_t i = 0; i < station->m_nsupported; i++)
    {

      if (station->m_minstrelTable[i].numSampleAttempt > m_pianoUpdateStats && station->m_minstrelTable[i].numRefAttempt > m_pianoUpdateStats)
        {
          /**
           * calculate the probability of success
           * assume probability scales from 0 to 18000
           */
          refTempProb = (station->m_minstrelTable[i].numRefSuccess * 18000) / station->m_minstrelTable[i].numRefAttempt;
          sampleTempProb = (station->m_minstrelTable[i].numSampleSuccess * 18000) / station->m_minstrelTable[i].numSampleAttempt;

          if (station->m_minstrelTable[i].numDataAttempt)
            {
              dataTempProb = (station->m_minstrelTable[i].numDataSuccess * 18000) / station->m_minstrelTable[i].numDataAttempt;
            }
          else
            {
              dataTempProb = 18000;
            }

          /// bookeeping
          station->m_minstrelTable[i].refProb = refTempProb;
          station->m_minstrelTable[i].dataProb = dataTempProb;
          station->m_minstrelTable[i].sampleProb = sampleTempProb;

          /// ewma probability (cast for gcc 3.4 compatibility)
          refTempProb = static_cast<uint32_t> (((refTempProb * (100 - m_ewmaLevel)) + (station->m_minstrelTable[i].ewmaRefProb * m_ewmaLevel) ) / 100);
          dataTempProb = static_cast<uint32_t> (((dataTempProb * (100 - m_ewmaLevel)) + (station->m_minstrelTable[i].ewmaDataProb * m_ewmaLevel) ) / 100);
          sampleTempProb = static_cast<uint32_t> (((sampleTempProb * (100 - m_ewmaLevel)) + (station->m_minstrelTable[i].ewmaSampleProb * m_ewmaLevel) ) / 100);

          station->m_minstrelTable[i].ewmaRefProb = refTempProb;
          station->m_minstrelTable[i].ewmaDataProb = dataTempProb;
          station->m_minstrelTable[i].ewmaSampleProb = sampleTempProb;

          station->m_minstrelTable[i].numRefSuccess = 0;
          station->m_minstrelTable[i].numRefAttempt = 0;
          station->m_minstrelTable[i].numDataSuccess = 0;
          station->m_minstrelTable[i].numDataAttempt = 0;
          station->m_minstrelTable[i].numSampleSuccess = 0;
          station->m_minstrelTable[i].numSampleAttempt = 0;

          //If throughput collapse, reset to initial settings
          if ((station->m_minstrelTable[i].ewmaDataProb < 0.1*18000) || (station->m_minstrelTable[station->m_txrate].throughput == 0))
          {
        	  //RateInit(station);
        	  NS_LOG_UNCOND("throughput collapse");
          }

          if (station->m_minstrelTable[i].ewmaSampleProb < (station->m_minstrelTable[i].ewmaRefProb - m_thInc*18000))
            {
              station->m_minstrelTable[i].samplePower = Min((m_nPower-1),(station->m_minstrelTable[i].samplePower + m_deltaInc));
            }

          if (station->m_minstrelTable[i].ewmaDataProb > (station->m_minstrelTable[i].ewmaRefProb - m_thDec*18000))
            {
              station->m_minstrelTable[i].samplePower = Max(0,(station->m_minstrelTable[i].samplePower - m_deltaDec));
            }

          if (station->m_minstrelTable[i].ewmaRefProb < (18000 - m_thInc*18000))
            {
              station->m_minstrelTable[i].refPower = Min((m_nPower-1),(station->m_minstrelTable[i].refPower + m_deltaInc));
            }

          if (station->m_minstrelTable[i].ewmaRefProb > (18000 - m_thDec*18000))
            {
              station->m_minstrelTable[i].refPower = Max(0,(station->m_minstrelTable[i].refPower - m_deltaDec));
            }

          station->m_minstrelTable[i].dataPower = station->m_minstrelTable[i].samplePower + m_delta;

          station->m_minstrelTable[i].validityTimer = Simulator::Now();
        }

      if (station->m_minstrelTable[i].numDataAttempt > m_pianoUpdateStats)
        {
          dataTempProb = (station->m_minstrelTable[i].numDataSuccess * 18000) / station->m_minstrelTable[i].numDataAttempt;
          station->m_minstrelTable[i].dataProb = dataTempProb;

          dataTempProb = static_cast<uint32_t> (((dataTempProb * (100 - m_ewmaLevel)) + (station->m_minstrelTable[i].ewmaDataProb * m_ewmaLevel) ) / 100);
          station->m_minstrelTable[i].ewmaDataProb = dataTempProb;
          station->m_minstrelTable[i].numDataSuccess = 0;
          station->m_minstrelTable[i].numDataAttempt = 0;
        }

      if (station->m_minstrelTable[i].validityTimer > Simulator::Now() + Seconds(1))
        {
          if (i > station->m_maxTpRate)
            {
              station->m_minstrelTable[i].dataPower = m_nPower - 1;
              station->m_minstrelTable[i].refPower = m_nPower - 1;
              station->m_minstrelTable[i].samplePower = station->m_minstrelTable[i].refPower - m_delta;
            }
          else
            {
              station->m_minstrelTable[i].dataPower =  station->m_minstrelTable[station->m_maxTpRate2].dataPower;
              station->m_minstrelTable[i].refPower =  station->m_minstrelTable[station->m_maxTpRate2].refPower;
              station->m_minstrelTable[i].samplePower =  station->m_minstrelTable[station->m_maxTpRate2].samplePower;
            }

        }
    }
}

void
MinstrelPianoWifiManager::RateInit (MinstrelPianoWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("RateInit=" << station);

  for (uint32_t i = 0; i < station->m_nsupported; i++)
    {
      NS_LOG_DEBUG ("Initializing rate index " << i << " " << GetSupported (station, i));
      station->m_minstrelTable[i].numRateAttempt = 0;
      station->m_minstrelTable[i].numRateSuccess = 0;
      station->m_minstrelTable[i].prob = 0;
      station->m_minstrelTable[i].ewmaProb = 0;
      station->m_minstrelTable[i].throughput = 0;
      station->m_minstrelTable[i].perfectTxTime = GetCalcTxTime (GetSupported (station, i));
      NS_LOG_DEBUG (" perfectTxTime = " << station->m_minstrelTable[i].perfectTxTime);
      station->m_minstrelTable[i].retryCount = 1;
      station->m_minstrelTable[i].adjustedRetryCount = 1;
      // Emulating minstrel.c::ath_rate_ctl_reset
      // We only check from 2 to 10 retries. This guarantee that
      // at least one retry is permitted.
      Time totalTxTimeWithGivenRetries = Seconds (0.0); // tx_time in minstrel.c
      NS_LOG_DEBUG (" Calculating the number of retries");
      for (uint32_t retries = 2; retries < 11; retries++)
        {
          NS_LOG_DEBUG ("  Checking " << retries << " retries");
          totalTxTimeWithGivenRetries = CalculateTimeUnicastPacket (station->m_minstrelTable[i].perfectTxTime, 0, retries);
          NS_LOG_DEBUG ("   totalTxTimeWithGivenRetries = " << totalTxTimeWithGivenRetries);
          if (totalTxTimeWithGivenRetries > MilliSeconds (6))
            {
              break;
            }
          station->m_minstrelTable[i].retryCount = retries;
          station->m_minstrelTable[i].adjustedRetryCount = retries;
        }
      /*piano init*/
      station->m_minstrelTable[i].numRefAttempt = 0;
      station->m_minstrelTable[i].numRefSuccess = 0;
      station->m_minstrelTable[i].refProb = 0;
      station->m_minstrelTable[i].ewmaRefProb = 0;
      station->m_minstrelTable[i].numDataAttempt = 0;
      station->m_minstrelTable[i].numDataSuccess = 0;
      station->m_minstrelTable[i].dataProb = 0;
      station->m_minstrelTable[i].ewmaDataProb = 0;
      station->m_minstrelTable[i].numSampleAttempt = 0;
      station->m_minstrelTable[i].numSampleSuccess = 0;
      station->m_minstrelTable[i].sampleProb = 0;
      station->m_minstrelTable[i].ewmaSampleProb = 0;
      station->m_minstrelTable[i].dataPower = m_nPower - 1;
      station->m_minstrelTable[i].refPower = m_nPower - 1;
      station->m_minstrelTable[i].samplePower = station->m_minstrelTable[i].refPower - m_delta;
      station->m_minstrelTable[i].validityTimer = Simulator::Now() + Seconds(1);
    }
}


Time
MinstrelPianoWifiManager::CalculateTimeUnicastPacket (Time dataTransmissionTime, uint32_t shortRetries, uint32_t longRetries)
{
  NS_LOG_FUNCTION (this << dataTransmissionTime << shortRetries << longRetries);
  // See rc80211_minstrel.c

  //First transmission (DATA + ACK timeout)
  Time tt = dataTransmissionTime + GetMac ()->GetAckTimeout ();

  uint32_t cwMax = 1023;
  uint32_t cw = 31;
  for (uint32_t retry = 0; retry < longRetries; retry++)
    {
      // Add one re-transmission (DATA + ACK timeout)
      tt += dataTransmissionTime + GetMac ()->GetAckTimeout ();

      // Add average back off (half the current contention window)
      tt += NanoSeconds ((cw / 2) * GetMac ()->GetSlot ());

      // Update contention window
      cw = std::min (cwMax, (cw + 1) * 2);
    }


  // First, we have to sense idle channel for DIFS (SIFS + 2*SLOT)
//  Time tt = GetMac ()->GetSifs () + GetMac ()->GetSlot () + GetMac ()->GetSlot ();
//  NS_LOG_DEBUG ("tt (DIFS) = " << tt);
//
//  // Next, we add the ACK timeout duration. Since we are given longRetries, the number of ACK timeout
//  // is longRetries + 1 (first transmission and longRetries times).
//  tt += NanoSeconds ((longRetries + 1) * GetMac ()->GetAckTimeout ());
//  NS_LOG_DEBUG ("tt (DIFS + ACKs) = " << tt << " (ACK TO) = " << GetMac ()->GetAckTimeout ());
//  // Next, we add the time to send (longRetries + 1) DATA. Same logic as ACK timeout.
//  // They can be combined, but separated for clarity.
//  tt += NanoSeconds ((longRetries + 1) * dataTransmissionTime);
//  NS_LOG_DEBUG ("tt (DIFS + ACKs + DATAs) = " << tt);
//
//  // Finally, we account for the backoff time between retransmissions.
//  // The original minstrel code seems to estimate the time as half the current contention window.
//  // The calculation of the original minsrel code is a little bit off (not exactly half) so we do the same.
//  // In addition, WIFI_CW_MIN is set to 31 in the original code.
//  uint32_t cwMax = 1023;
//  uint32_t cw = 31;
//  for (uint32_t i = 0; i <= (shortRetries + longRetries); i++)
//    {
//      cw = std::min (cwMax, (cw + 1) * 2); // estimate the current contention window size (I think it's a bit off)
//      NS_LOG_DEBUG (" cw = " << cw);
//      tt += NanoSeconds ((cw / 2) * GetMac ()->GetSlot ()); // average is about half
//      NS_LOG_DEBUG (" tt (DIFS + ACKs + DATAs + " << cw << " cw) = " << tt);
//    }

  return tt;
}

void
MinstrelPianoWifiManager::InitSampleTable (MinstrelPianoWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("InitSampleTable=" << this);

  station->m_col = station->m_index = 0;

  /// for off-seting to make rates fall between 0 and numrates
  uint32_t numSampleRates = station->m_nsupported;

  uint32_t newIndex;
  for (uint32_t col = 0; col < m_sampleCol; col++)
    {
      for (uint32_t i = 0; i < numSampleRates; i++ )
        {

          /**
           * The next two lines basically tries to generate a random number
           * between 0 and the number of available rates
           */
          int uv = m_uniformRandomVariable->GetInteger (0, numSampleRates);
          newIndex = (i + uv) % numSampleRates;

          /// this loop is used for filling in other uninitilized places
          while (station->m_sampleTable[newIndex][col] != 0)
            {
              newIndex = (newIndex + 1) % station->m_nsupported;
            }
          station->m_sampleTable[newIndex][col] = i;

        }
    }
}

void
MinstrelPianoWifiManager::PrintSampleTable (MinstrelPianoWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("PrintSampleTable=" << station);

  uint32_t numSampleRates = station->m_nsupported;
  for (uint32_t i = 0; i < numSampleRates; i++)
    {
      for (uint32_t j = 0; j < m_sampleCol; j++)
        {
          NS_LOG_DEBUG(station->m_sampleTable[i][j] << "\t");
        }
      NS_LOG_DEBUG("\n");
    }
}

void
MinstrelPianoWifiManager::PrintTable (MinstrelPianoWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("PrintTable=" << station);

  NS_LOG_DEBUG ("index throughput ewmaProb prob success(attempt) refPower samplePower dataPower ewmaRefProb ewmaSampleProb ewmaDataProb refSuccess(refAttempt) sampleSucces(sampleAttempt) dataSuccess(dataAttempt)\n");
  for (uint32_t i = 0; i < station->m_nsupported; i++)
    {
      NS_LOG_DEBUG( std::setw(2) << i << std::setw(10) << station->m_minstrelTable[i].throughput << std::setw(10) << station->m_minstrelTable[i].ewmaProb << std::setw(10) << station->m_minstrelTable[i].prob
                  << std::setw(10) << station->m_minstrelTable[i].numRateSuccess << "(" << station->m_minstrelTable[i].numRateAttempt << ")"
                  << std::setw(10) << (int) station->m_minstrelTable[i].refPower << std::setw(10) << (int) station->m_minstrelTable[i].samplePower << std::setw(10) << (int) station->m_minstrelTable[i].dataPower
                  << std::setw(10) << station->m_minstrelTable[i].ewmaRefProb << std::setw(10) << station->m_minstrelTable[i].ewmaSampleProb << std::setw(10) << station->m_minstrelTable[i].ewmaDataProb
                  << std::setw(15) << station->m_minstrelTable[i].numRefSuccess << "(" << station->m_minstrelTable[i].numRefAttempt << ")"
                  << std::setw(15) << station->m_minstrelTable[i].numSampleSuccess << "(" << station->m_minstrelTable[i].numSampleAttempt << ")"
                  << std::setw(15) << station->m_minstrelTable[i].numDataSuccess << "(" << station->m_minstrelTable[i].numDataAttempt << ")"
                  <<"\n");
    }
}

} // namespace ns3






