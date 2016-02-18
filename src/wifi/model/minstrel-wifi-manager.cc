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

#include "minstrel-wifi-manager.h"
#include "wifi-phy.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/wifi-mac.h"
#include "ns3/assert.h"
#include <vector>

#define Min(a,b) ((a < b) ? a : b)

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MinstrelWifiManager");

NS_OBJECT_ENSURE_REGISTERED (MinstrelWifiManager);

TypeId
MinstrelWifiManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MinstrelWifiManager")
    .SetParent<WifiRemoteStationManager> ()
    .SetGroupName ("Wifi")
    .AddConstructor<MinstrelWifiManager> ()
    .AddAttribute ("UpdateStatistics",
                   "The interval between updating statistics table ",
                   TimeValue (Seconds (0.1)),
                   MakeTimeAccessor (&MinstrelWifiManager::m_updateStatsInterval),
                   MakeTimeChecker ())
    .AddAttribute ("LookAroundRate",
                   "the percentage to try other rates",
                   DoubleValue (10),
                   MakeDoubleAccessor (&MinstrelWifiManager::m_lookAroundRate),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("EWMA",
                   "EWMA level",
                   DoubleValue (75),
                   MakeDoubleAccessor (&MinstrelWifiManager::m_ewmaLevel),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("SampleColumn",
                   "The number of columns used for sampling",
                   DoubleValue (10),
                   MakeDoubleAccessor (&MinstrelWifiManager::m_sampleCol),
                   MakeDoubleChecker <double> ())
    .AddAttribute ("PacketLength",
                   "The packet length used for calculating mode TxTime",
                   DoubleValue (1200),
                   MakeDoubleAccessor (&MinstrelWifiManager::m_segmentSize),
                   MakeDoubleChecker <double> ())
  ;
  return tid;
}

MinstrelWifiManager::MinstrelWifiManager ()
{
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();

  m_nsupported = 0;
}

MinstrelWifiManager::~MinstrelWifiManager ()
{
}

void
MinstrelWifiManager::SetupPhy (Ptr<WifiPhy> phy)
{
  NS_LOG_FUNCTION (this << phy);
  uint32_t nModes = phy->GetNModes ();
  for (uint32_t i = 0; i < nModes; i++)
    {
      WifiMode mode = phy->GetMode (i);
      WifiTxVector txVector;
      txVector.SetMode (mode);
      AddCalcTxTime (mode, phy->CalculateTxDuration (m_segmentSize, txVector, WIFI_PREAMBLE_LONG, phy->GetFrequency ()));
    }
  m_phy = phy;
  WifiRemoteStationManager::SetupPhy (phy);
}

int64_t
MinstrelWifiManager::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  return 1;
}

Time
MinstrelWifiManager::GetCalcTxTime (WifiMode mode) const
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
MinstrelWifiManager::AddCalcTxTime (WifiMode mode, Time t)
{
  m_calcTxTime.push_back (std::make_pair (t, mode));
}

WifiRemoteStation *
MinstrelWifiManager::DoCreateStation (void) const
{
  MinstrelWifiRemoteStation *station = new MinstrelWifiRemoteStation ();

  station->m_nextStatsUpdate = Simulator::Now () + m_updateStatsInterval;
  station->m_spAckDur = 0;
  station->m_rateAvg = 0;
  station->m_lowestRix = 0;
  for (uint32_t i = 0; i < N_MAX_TH_RATES; i++)
      station->m_maxTpRate[i]=0;
  station->m_maxProbRate = 0;
  station->m_packetCount = 0;
  station->m_sampleCount = 0;
  station->m_sampleDeferred = false;
  station->m_sampleColumn = 0;
  station->m_sampleRow = 0;
  station->m_numRates = 0;
  station->m_secondStageSampling = false;
  station->m_currentRate = 0;
  station->m_chainIndex = 0;
  station->m_initialized = false;

  return station;
}

void
MinstrelWifiManager::CheckInit (MinstrelWifiRemoteStation *station)
{
  if (!station->m_initialized && GetNSupported (station) > 1)
    {
      //Note: we appear to be doing late initialization of the table
      //to make sure that the set of supported rates has been initialized
      //before we perform our own initialization.

      station->m_sampleTable = SampleRateVector (station->m_numRates, std::vector<uint32_t> (station->m_sampleColumn));
      InitSampleTable (station);
      RateInit (station);
      station->m_initialized = true;

      PrintTable (station);
      PrintSampleTable (station);
    }
}

void
MinstrelWifiManager::DoReportRxOk (WifiRemoteStation *st,
                                   double rxSnr, WifiMode txMode)
{
  NS_LOG_FUNCTION (this);
}

void
MinstrelWifiManager::DoReportRtsFailed (WifiRemoteStation *st)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *)st;
  NS_LOG_DEBUG ("DoReportRtsFailed m_currentRate=" << station->m_currentRate);

  station->m_shortRetry++;
}

void
MinstrelWifiManager::DoReportRtsOk (WifiRemoteStation *st, double ctsSnr, WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_DEBUG ("self=" << st << " rts ok");
}

void
MinstrelWifiManager::DoReportFinalRtsFailed (WifiRemoteStation *st)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *)st;
  UpdateRetry (station);
}

void
MinstrelWifiManager::DoReportDataFailed (WifiRemoteStation *st)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *)st;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  station->m_longRetry++;


  MinstrelRate rate = station->m_minstrelTable[station->m_currentRate];
  MinstrelRateStats rateStats = rate.stats;

  /**
   * Increment the attempts counter for the rate and power used.
   * In the article the attempts are increased only by one and not by the number of the attempted retries.
   * However the correct behavior is to count every attempt.
   */
  rateStats.numAttempts++;

  //PrintTable (station);

  NS_LOG_DEBUG ("DoReportDataFailed " << station << " rate " << station->m_currentRate << " longRetry " << station->m_longRetry);

  uint32_t counter = 0;
  for (uint32_t i=0; i<=station->m_chainIndex; i++)
    {
      counter += station->m_chain[i].count;
    }

  if (station->m_longRetry >= counter)
    {
      if (station->m_chainIndex < IEEE80211_TX_RATE_TABLE_SIZE - 1)
        {
          NS_LOG_DEBUG ("Move to the next rate.");
          station->m_chainIndex++;
          station->m_currentRate = station->m_chain[station->m_chainIndex].rate;
        }
      else
          NS_LOG_DEBUG ("All retry attempts consumed");
    }
}

/**
 * This function is used to say that all retry attempts fail.
 * Before calling this function, DoReportDataFailed was called.
 */
void
MinstrelWifiManager::DoReportFinalDataFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (st);
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  NS_LOG_DEBUG ("DoReportFinalDataFailed m_currentRate = " << station->m_currentRate);

  UpdateRetry (station);

  /**
   * If enough samples, stats will be updated.
   */
  if (Simulator::Now () >=  station->m_nextStatsUpdate)
    {
      UpdateStats (station);
      station->m_nextStatsUpdate = Simulator::Now () + m_updateStatsInterval;
      NS_LOG_DEBUG ("Next update at " << station->m_nextStatsUpdate);
    }

  UpdateRates(station);

  PrintTable (station);
}

void
MinstrelWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                     double ackSnr, WifiMode ackMode, double dataSnr)
{
  NS_LOG_FUNCTION (st << ackSnr << ackMode << dataSnr);
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  MinstrelRateStats rateStats = station->m_minstrelTable[station->m_currentRate].stats;

  NS_LOG_DEBUG ("DoReportDataOk m_currentRate = " << station->m_currentRate << ", attempt = " << rateStats.numAttempts << ", success = " << rateStats.numSuccess << " (before update).");

  rateStats.numSuccess++;
  rateStats.numAttempts++;

  NS_LOG_DEBUG ("DoReportDataOk m_currentRate = " << station->m_currentRate << ", attempt = " << rateStats.numAttempts << ", success = " << rateStats.numSuccess << " (after update).");

  UpdateRetry (station);

  station->m_packetCount++;

  /**
   * If enough samples, stats will be updated.
   */
  if (Simulator::Now () >=  station->m_nextStatsUpdate)
    {
      UpdateStats (station);
      station->m_nextStatsUpdate = Simulator::Now () + m_updateStatsInterval;
      NS_LOG_DEBUG ("Next update at " << station->m_nextStatsUpdate);
    }

  UpdateRates(station);

  PrintTable (station);
}

void
MinstrelWifiManager::UpdateRetry (MinstrelWifiRemoteStation *station)
{
  station->m_shortRetry = 0;
  station->m_longRetry = 0;
}

WifiTxVector
MinstrelWifiManager::DoGetDataTxVector (WifiRemoteStation *st,
                                        uint32_t size)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;
  uint32_t channelWidth = GetChannelWidth (station);
  if (channelWidth > 20 && channelWidth != 22)
    {
      //avoid to use legacy rate adaptation algorithms for IEEE 802.11n/ac
      channelWidth = 20;
    }
  if (!station->m_initialized)
    {
      CheckInit (station);

      //start the rate at half way
      station->m_currentRate = m_nsupported / 2;
    }
  UpdateStats (station);
  return WifiTxVector (GetSupported (station, station->m_currentRate), GetDefaultTxPowerLevel (), GetLongRetryCount (station), false, 1, 0, channelWidth, GetAggregation (station), false);
}

WifiTxVector
MinstrelWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *) st;
  NS_LOG_DEBUG ("DoGetRtsMode m_currentRate=" << station->m_currentRate);
  uint32_t channelWidth = GetChannelWidth (station);
  if (channelWidth > 20 && channelWidth != 22)
    {
      //avoid to use legacy rate adaptation algorithms for IEEE 802.11n/ac
      channelWidth = 20;
    }
  return WifiTxVector (GetSupported (station, 0), GetDefaultTxPowerLevel (), GetShortRetryCount (station), false, 1, 0, channelWidth, GetAggregation (station), false);
}

bool
MinstrelWifiManager::DoNeedDataRetransmission (WifiRemoteStation *st, Ptr<const Packet> packet, bool normally)
{
  MinstrelWifiRemoteStation *station = (MinstrelWifiRemoteStation *)st;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return normally;
    }

  uint32_t counter = 0;
  for (uint32_t i=0; i<IEEE80211_TX_RATE_TABLE_SIZE; i++)
    {
      counter += station->m_chain[i].count;
    }

  if (station->m_longRetry >= counter)
    {
      return false;
    }
  else
    {
      return true;
    }
}

bool
MinstrelWifiManager::IsLowLatency (void) const
{
  NS_LOG_FUNCTION (this);
  return true;
}

uint32_t
MinstrelWifiManager::GetNextSample (MinstrelWifiRemoteStation *station)
{
  uint32_t bitrate;
  bitrate = station->m_sampleTable[station->m_sampleRow][station->m_sampleColumn];
  station->m_sampleRow++;

  if (station->m_sampleRow > (m_nsupported - 2))
    {
      station->m_sampleRow = 0;
      station->m_sampleColumn++;
      if (station->m_sampleColumn >= m_sampleCol)
        {
          station->m_sampleColumn = 0;
        }
    }
  return bitrate;
}

void
MinstrelWifiManager::UpdateRates (MinstrelWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);

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

  /**
   * If we have not sent any frames yet, then use low rate and high power.
   */
  if ((station->m_sampleCount + station->m_packetCount) == 0)
    {
      for (uint32_t i=0; i<IEEE80211_TX_RATE_TABLE_SIZE; i++)
        {
          station->m_chain[i].rate = station->m_lowestRix;
        }
    }
  else
    {
      SetRetryChainElement(station, 0, station->m_maxTpRate[0]);
      SetRetryChainElement(station, 1, station->m_maxTpRate[1]);
      SetRetryChainElement(station, 2, station->m_maxProbRate);
      station->m_chain[3].rate = station->m_lowestRix;
      station->m_chain[3].count = m_maxRetry;

      /* Now will check if the retry chain has to be changed for sampling*/
      int delta = (station->m_packetCount * m_lookAroundRate / 100) - (station->m_sampleCount + station->m_sampleDeferred / 2);

      /// delta < 0: no sampling required
      if (delta >= 0)
        {
          if (station->m_packetCount >= 10000)
            {
              station->m_sampleDeferred;
              station->m_sampleCount = 0;
              station->m_packetCount = 0;
            }
          else if (delta > station->m_numRates * 2)
            {
              /* With multi-rate retry, not every planned sample
               * attempt actually gets used, due to the way the retry
               * chain is set up - [max_tp,sample,prob,lowest] for
               * sample_rate < max_tp.
               *
               * If there's too much sampling backlog and the link
               * starts getting worse, minstrel would start bursting
               * out lots of sampling frames, which would result
               * in a large throughput loss. */
              station->m_sampleCount += (delta - station->m_numRates * 2);
            }

          /// Now go through the table and find an index rate.
          uint32_t sampleIndex = GetNextSample (station);
          MinstrelRate sampleRate = station->m_minstrelTable[sampleIndex];
          MinstrelRate bestRate = station->m_minstrelTable[station->m_maxTpRate[0]];

          /* Decide if direct ( 1st mrr stage) or indirect (2nd mrr stage)
           * rate sampling method should be used.
           * Respect such rates that are not sampled for 20 interations.
           */
          if (sampleRate.perfectTxTime > bestRate.perfectTxTime  &&
              sampleRate.stats.sampleSkipped < 20)
            {
              /* Only use m_secondStageSampling to mark
               * packets that have the sampling rate deferred to the
               * second MRR stage. Increase the sample counter only
               * if the deferred sample rate was actually used.
               * Use the sample_deferred counter to make sure that
               * the sampling is not done in large bursts */
              station->m_secondStageSampling = true;
              SetRetryChainElement(station, 1, sampleIndex);
              station->m_sampleDeferred++;
            }
          else if (sampleRate.sampleLimit)
            {
              station->m_sampleCount++;
              if (sampleRate.sampleLimit > 0)
                sampleRate.sampleLimit--;
              SetRetryChainElement(station, 0, sampleIndex);
            }
        }
    }

  station->m_chainIndex = 0;
  station->m_currentRate = station->m_chain[0].rate;
  NS_LOG_DEBUG ("Rate = " << station->m_currentRate << "(" << GetSupported (station, station->m_currentRate) << ")");
}

void
MinstrelWifiManager::SetRetryChainElement(MinstrelWifiRemoteStation *station, int chainOffset, int rateIndex)
{
  MinstrelRate rate = station->m_minstrelTable[rateIndex];
  station->m_chain[chainOffset].rate = rate.rix;
  station->m_chain[chainOffset].count = rate.adjustedRetryCount;
}

void
MinstrelWifiManager::UpdateStats (MinstrelWifiRemoteStation *station)
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
  station->m_nextStatsUpdate = Simulator::Now () + m_updateStatsInterval;
  NS_LOG_DEBUG ("Next update at " << station->m_nextStatsUpdate);
  NS_LOG_DEBUG ("Currently using rate: " << station->m_currentRate << " (" << GetSupported (station, station->m_currentRate) << ")");

  Time txTime;
  uint32_t tempProb;

  NS_LOG_DEBUG ("Index-Rate\t\tAttempt\tSuccess");
  for (uint32_t i = 0; i < m_nsupported; i++)
    {

      MinstrelRate rate = station->m_minstrelTable[i];
      MinstrelRateStats rateStats = rate.stats;

      //calculate the perfect tx time for this rate
      txTime = rate.perfectTxTime;

      //just for initialization
      if (txTime.GetMicroSeconds () == 0)
        {
          txTime = Seconds (1);
        }

      NS_LOG_DEBUG (i << " " << GetSupported (station, i) <<
                    "\t" << rateStats.numAttempts <<
                    "\t" << rateStats.numSuccess);

      //if we've attempted something
      if (rateStats.numAttempts)
        {
          /**
           * calculate the probability of success
           * assume probability scales from 0 to 18000
           */
          tempProb = (rateStats.numSuccess * 18000) / rateStats.numAttempts;

          //bookeeping
          rateStats.currentProb = tempProb;

          //ewma probability (cast for gcc 3.4 compatibility)
          tempProb = static_cast<uint32_t> (((tempProb * (100 - m_ewmaLevel)) + (rateStats.ewmaProb * m_ewmaLevel) ) / 100);

          rateStats.ewmaProb = tempProb;

          //calculating throughput
          station->m_minstrelTable[i].throughput = tempProb * (1000000 / txTime.GetMicroSeconds ());

        }

      //bookeeping
      rateStats.numSuccess = 0;
      rateStats.numAttempts = 0;

      //Sample less often below 10% and  above 95% of success
      if ((rateStats.ewmaProb > 17100) || (rateStats.ewmaProb < 1800))
        {
          /**
           * See: http://wireless.kernel.org/en/developers/Documentation/mac80211/RateControl/minstrel/
           *
           * Analysis of information showed that the system was sampling too hard at some rates.
           * For those rates that never work (54mb, 500m range) there is no point in sending 10 sample packets (< 6 ms time).
           * Consequently, for the very very low probability rates, we sample at most twice.
           */
          if (rateStats.retryCount > 2)
            {
              rate.adjustedRetryCount = 2;
            }
          else
            {
              rate.adjustedRetryCount = rateStats.retryCount;
            }
        }
      else
        {
          rate.adjustedRetryCount = rateStats.retryCount;
        }

      //if it's 0 allow one retry limit
      if (station->m_minstrelTable[i].adjustedRetryCount == 0)
        {
          station->m_minstrelTable[i].adjustedRetryCount = 1;
        }
    }

  NS_LOG_DEBUG ("Attempt/success resetted to 0");

  uint32_t max_prob = 0, index_max_prob = 0, max_tp = 0, index_max_tp = 0, index_max_tp2 = 0;

  //go find max throughput, second maximum throughput, high probability succ
  NS_LOG_DEBUG ("Finding the maximum throughput, second maximum throughput, and highest probability");
  NS_LOG_DEBUG ("Index-Rate\t\tT-put\tEWMA");
  for (uint32_t i = 0; i < m_nsupported; i++)
    {
      MinstrelRate rate = station->m_minstrelTable[i];
      MinstrelRateStats rateStats = rate.stats;
      NS_LOG_DEBUG (i << " " << GetSupported (station, i) <<
                    "\t" << station->m_minstrelTable[i].throughput <<
                    "\t" << rateStats.ewmaProb);

      if (max_tp < station->m_minstrelTable[i].throughput)
        {
          index_max_tp = i;
          max_tp = station->m_minstrelTable[i].throughput;
        }

      if (max_prob < rateStats.ewmaProb)
        {
          index_max_prob = i;
          max_prob = rateStats.ewmaProb;
        }
    }


  max_tp = 0;
  //find the second highest max
  for (uint32_t i = 0; i < m_nsupported; i++)
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

  if (index_max_tp > station->m_currentRate)
    {
      station->m_currentRate = index_max_tp;
    }

  NS_LOG_DEBUG ("max throughput=" << index_max_tp << "(" << GetSupported (station, index_max_tp) <<
                ")\tsecond max throughput=" << index_max_tp2 << "(" << GetSupported (station, index_max_tp2) <<
                ")\tmax prob=" << index_max_prob << "(" << GetSupported (station, index_max_prob) << ")");
}

void
MinstrelWifiManager::RateInit (MinstrelWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (station);

  station->m_lowestRix = 0; // Because of how the supported rates are saved, 0 is always the index of the lowest rate.

  WifiMode mode = m_phy->GetMode (station->m_lowestRix);
  WifiTxVector txVector;
  txVector.SetMode (mode);
  station->m_spAckDur = m_phy->CalculateTxDuration (10, txVector, WIFI_PREAMBLE_SHORT, m_phy->GetFrequency ());

  station->m_numRates = GetNSupported (station);
  station->m_minstrelTable = MinstrelRateVector (station->m_numRates);

  for (uint32_t i = 0; i < station->m_numRates; i++)
    {
      NS_LOG_DEBUG ("Initializing rate index " << i << " " << GetSupported (station, i));
      MinstrelRate rate = station->m_minstrelTable[i];

      MinstrelRateStats rateStats = rate.stats;
      rateStats.currentProb = 0;
      rateStats.ewmaProb = 0;
      rateStats.numAttempts = 0;
      rateStats.numSuccess = 0;
      rateStats.retryUpdated = false;
      rateStats.sampleSkipped = 0;

      rate.rix = i;
      rate.bitrate = mode;
      rate.perfectTxTime = GetCalcTxTime (rate.bitrate);
      rate.ackTime = GetCalcTxTime (rate.bitrate);
      NS_LOG_DEBUG (" perfectTxTime = " << rate.perfectTxTime);

      /* calculate maximum number of retransmissions before
       * fallback (based on maximum segment size) */
      rate.sampleLimit = 1;
      rateStats.retryCount = 1;
      rate.retryCountCts = 1;
      rateStats.retryCountRtsCts = 1;
      station->m_minstrelTable[i].adjustedRetryCount = 1;
      Time txTime = rate.perfectTxTime + station->m_spAckDur;
      //Emulating minstrel.c::ath_rate_ctl_reset
      //We only check from 2 to 10 retries. This guarantee that
      //at least one retry is permitter.
      Time totalTxTimeWithGivenRetries = Seconds (0.0); //tx_time in minstrel.c
      NS_LOG_DEBUG (" Calculating the number of retries");
      for (uint32_t retries = 2; retries < 11; retries++)
        {
          NS_LOG_DEBUG ("  Checking " << retries << " retries");
          totalTxTimeWithGivenRetries = CalculateTimeUnicastPacket (rate.perfectTxTime, station->m_spAckDur, 0, retries);
          NS_LOG_DEBUG ("   totalTxTimeWithGivenRetries = " << totalTxTimeWithGivenRetries);
          if (totalTxTimeWithGivenRetries > MilliSeconds (6))
            {
              break;
            }
          rateStats.retryCount = retries;
          rate.adjustedRetryCount = retries;
        }
    }
}

Time
MinstrelWifiManager::CalculateTimeUnicastPacket (Time dataTransmissionTime, Time ackTransmissionTime, uint32_t shortRetries, uint32_t longRetries)
{
  NS_LOG_FUNCTION (this << dataTransmissionTime << shortRetries << longRetries);
  //See rc80211_minstrel.c

  //Correct transmission time (DATA + ACK)
  Time tt = dataTransmissionTime + ackTransmissionTime;

  uint32_t cwMax = 1023;
  uint32_t cw = 31;
  for (uint32_t retry = 0; retry < longRetries; retry++)
    {
      //Add one re-transmission (DATA + ACK timeout)
      tt += dataTransmissionTime + GetMac ()->GetAckTimeout ();

      //Add average back off (half the current contention window)
      tt += NanoSeconds ((cw / 2) * GetMac ()->GetSlot ());

      //Update contention window
      cw = std::min (cwMax, (cw + 1) * 2);
    }

  return tt;
}

void
MinstrelWifiManager::InitSampleTable (MinstrelWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("InitSampleTable=" << this);

  station->m_sampleColumn = station->m_sampleRow = 0;

  //for off-seting to make rates fall between 0 and numrates
  uint32_t numSampleRates = m_nsupported;

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

          //this loop is used for filling in other uninitilized places
          while (station->m_sampleTable[newIndex][col] != 0)
            {
              newIndex = (newIndex + 1) % m_nsupported;
            }
          station->m_sampleTable[newIndex][col] = i;
        }
    }
}

void
MinstrelWifiManager::PrintSampleTable (MinstrelWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("PrintSampleTable=" << station);

  uint32_t numSampleRates = m_nsupported;
  std::stringstream table;
  for (uint32_t i = 0; i < numSampleRates; i++)
    {
      for (uint32_t j = 0; j < m_sampleCol; j++)
        {
          table << station->m_sampleTable[i][j] << "\t";
        }
      table << std::endl;
    }
  NS_LOG_DEBUG (table.str ());
}

void
MinstrelWifiManager::PrintTable (MinstrelWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("PrintTable=" << station);

  for (uint32_t i = 0; i < m_nsupported; i++)
    {
      NS_LOG_DEBUG (i << " (" << GetSupported (station, i) << "): "  << station->m_minstrelTable[i].perfectTxTime << ", retryCount = " << station->m_minstrelTable[i].stats.retryCount << ", adjustedRetryCount = " << station->m_minstrelTable[i].adjustedRetryCount);
    }
}

} //namespace ns3
