/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 Duy Nguyen
 * Copyright (c) 2015 Ghada Badawy
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
 *         Ghada Badawy <gbadawy@gmail.com>
 *
 * Some Comments:
 *
 * 1) Segment Size is declared for completeness but not used  because it has
 *    to do more with the requirement of the specific hardware.
 *
 * 2) By default, Minstrel applies the multi-rate retry(the core of Minstrel
 *    algorithm). Otherwise, please use ConstantRateWifiManager instead.
 *
 * 3) 40Mhz can't fall back to 20MHz
 *
 * reference: http://lwn.net/Articles/376765/
 */

#include "minstrel-ht-wifi-manager.h"
#include "wifi-phy.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/wifi-mac.h"
#include "ns3/assert.h"
#include <vector>

#define Min(a,b) ((a < b) ? a : b)

NS_LOG_COMPONENT_DEFINE ("MinstrelHtWifiManager");


namespace ns3 {


struct MinstrelHtWifiRemoteStation : public WifiRemoteStation
{
  void DisposeStation ();
  Time m_nextStatsUpdate;   //!< Time when stats will be updated.

  /**
   * To keep track of the current position in the our random sample table
   * going row by row from 1st column until the 10th column(Minstrel defines 10)
   * then we wrap back to the row 1 col 1.
   * note: there are many other ways to do this.
   */
  uint32_t m_col, m_index;

  uint32_t m_maxTpRate;     //!< The highest throughput rate.
  uint32_t m_maxTpRate2;    //!< The second highest throughput rate.
  uint32_t m_maxProbRate;   //!< The rate with highest probability of success.

  uint32_t m_frameCount;    //!< Total number of frames transmitted as of now.
  uint32_t m_sampleCount;   //!< How many packets we have sample so far.

  bool m_isSampling;        //!< A flag to indicate we are currently sampling.
  uint32_t m_sampleRate;    //!< The current sample rate.
  bool  m_sampleRateSlower; //!< A flag to indicate sample rate is slower.
  uint32_t m_sampleGroup;   //!< The group that the sample rate belongs to.

  uint32_t m_shortRetry;    //!< Number of short retries (such as control frames).
  uint32_t m_longRetry;     //!< Number of long retries (such as data packets).
  uint32_t m_retry;         //!< Number of retries (short + long).
  uint32_t m_err;           //!< Number of retry errors (all retransmission attempts failed).

  uint32_t m_txRate;        //!< Current transmission rate.

  bool m_initialized;       //!< For initializing variables.

  HtSampleRate m_sampleTable;   //!< Sample rates table.
  GroupInfo m_mcsTable[];       //!< Minstrel HT table.
};

void
MinstrelHtWifiRemoteStation::DisposeStation ()
{/*
  std::vector<std::vector<uint32_t> >().swap(m_sampleTable);
  for (uint8_t j=0; j< m_mcsTable.size();j++)
    std::vector<struct HtRateInfo>().swap(m_mcsTable[j].m_minstrelTable);
  std::vector<struct GroupInfo> ().swap(m_mcsTable);
  */
}

NS_OBJECT_ENSURE_REGISTERED (MinstrelHtWifiManager);

TypeId
MinstrelHtWifiManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MinstrelHtWifiManager")
    .SetParent<WifiRemoteStationManager> ()
    .AddConstructor<MinstrelHtWifiManager> ()
    .SetGroupName ("Wifi")
    .AddAttribute ("UpdateStatistics",
                   "The interval between updating statistics table ",
                   TimeValue (Seconds (0.1)),
                   MakeTimeAccessor (&MinstrelHtWifiManager::m_updateStats),
                   MakeTimeChecker ())
    .AddAttribute ("LookAroundRate",
                   "the percentage to try other rates",
                   DoubleValue (10),
                   MakeDoubleAccessor (&MinstrelHtWifiManager::m_lookAroundRate),
                   MakeDoubleChecker<double> (0, 100))
    .AddAttribute ("EWMA",
                   "EWMA level",
                   DoubleValue (75),
                   MakeDoubleAccessor (&MinstrelHtWifiManager::m_ewmaLevel),
                   MakeDoubleChecker<double> (0, 100))
    .AddAttribute ("SampleColumn",
                   "The number of columns used for sampling",
                   UintegerValue (10),
                   MakeUintegerAccessor (&MinstrelHtWifiManager::m_sampleCol),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("PacketLength",
                   "The packet length used for calculating mode TxTime",
                   UintegerValue (1200),
                   MakeUintegerAccessor (&MinstrelHtWifiManager::m_frameLength),
                   MakeUintegerChecker <uint32_t> ())
   .AddTraceSource ("RateChange",
                    "The transmission rate has change",
                    MakeTraceSourceAccessor (&MinstrelHtWifiManager::m_rateChange),
                    "ns3::MinstrelHtWifiManager::RateChangeTracedCallback")
  ;
  return tid;
}

MinstrelHtWifiManager::MinstrelHtWifiManager ()
  : m_nSupportedMcs (0),
    m_nSupportedGroups (0)
{
  NS_LOG_FUNCTION (this);
}

MinstrelHtWifiManager::~MinstrelHtWifiManager ()
{
  NS_LOG_FUNCTION (this);
  for (uint32_t i = 0; i < sizeof(m_groups); i++)
    {
      m_groups[i].calcTxTime.clear();
    }
}

void
MinstrelHtWifiManager::SetupPhy (Ptr<WifiPhy> phy)
{
  NS_LOG_FUNCTION (this << phy);

  m_nSupportedStreams = phy->GetNumberOfTransmitAntennas();
  if (phy->GetGuardInterval() && phy->GetChannelWidth() == 40)
    m_nStreamGroups = 4;
  else if (phy->GetGuardInterval() || phy->GetChannelWidth() == 40)
    m_nStreamGroups = 2;
  else
    m_nStreamGroups = 1;

    m_nSupportedGroupRates = phy->GetNMcs();
    m_nHtGroups = m_nSupportedGroupRates * m_nStreamGroups;
    m_nMinstrelGroups = m_nHtGroups;

  /**
   * Initialize the groups array.
   */
  m_groups = McsGroup (m_nMinstrelGroups);
  for (uint8_t streams = 1; streams <= m_nSupportedStreams; streams++)
    {
      for (uint8_t sgi = 0; sgi <= 1; sgi++)
        {
          uint32_t chWidth = 20;

          McsGroup group20;
          m_groups[GetGroupId(streams, sgi, 0)] = group20;
          group20.streams = streams;
          group20.sgi = sgi;
          group20.chWidth = chWidth;
          for (uint8_t i = 0; i < N_MCS_GROUP_RATES; i++)
            {
              WifiMode mode = phy->GetMcs(i);
              AddCalcTxTime (GetGroupId(streams, sgi, 0), mode, CalculateTxDuration(phy, streams, sgi, chWidth, mode));
            }

          chWidth = 40;

          McsGroup group40;
          m_groups[GetGroupId(streams, sgi, 1)] = group40;
          group40.streams = streams;
          group40.sgi = sgi;
          group40.chWidth = 40;
          for (uint8_t i = 0; i < N_MCS_GROUP_RATES; i++)
            {
              WifiMode mode = phy->GetMcs(i);
              AddCalcTxTime (GetGroupId(streams, sgi, 1), mode, CalculateTxDuration(phy, streams, sgi, chWidth, mode));
            }
        }
    }
  WifiRemoteStationManager::SetupPhy (phy);
}

Time
MinstrelHtWifiManager::CalculateTxDuration(Ptr<WifiPhy> phy, uint8_t streams, uint8_t sgi, uint32_t chWidth, WifiMode mode)
{
  NS_LOG_FUNCTION (this << phy << streams << sgi << chWidth << mode);

  WifiTxVector txvector;
  txvector.SetNss(streams);
  txvector.SetShortGuardInterval(sgi);
  txvector.SetChannelWidth(chWidth);
  txvector.SetTxPowerLevel(0);
  txvector.SetNess(0);
  txvector.SetStbc(phy->GetStbc());
  txvector.SetMode(mode);
  return phy->CalculateTxDuration (m_frameLength, txvector, WIFI_PREAMBLE_HT_MF, phy->GetFrequency(), NORMAL_MPDU,0);
}

Time
MinstrelHtWifiManager::GetCalcTxTime (uint32_t groupId, WifiMode mode) const
{
  NS_LOG_FUNCTION (this << groupId << mode);

  for (TxTime::const_iterator i = m_groups[groupId].calcTxTime.begin (); i != m_groups[groupId].calcTxTime.end (); i++)
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
MinstrelHtWifiManager::AddCalcTxTime (uint32_t groupId, WifiMode mode, Time t)
{
  NS_LOG_FUNCTION (this << groupId << mode << t);

  m_groups[groupId].calcTxTime.push_back (std::make_pair (t, mode));
}

WifiRemoteStation *
MinstrelHtWifiManager::DoCreateStation (void) const
{
  NS_LOG_FUNCTION (this);

  MinstrelHtWifiRemoteStation *station = new MinstrelHtWifiRemoteStation ();

  station->m_nextStatsUpdate = Simulator::Now () + m_updateStats;
  station->m_col = 0;
  station->m_index = 0;
  station->m_maxTpRate = 0;
  station->m_maxTpRate2 = 0;
  station->m_maxProbRate = 0;
  station->m_frameCount = 0;
  station->m_sampleCount = 0;
  station->m_isSampling = false;
  station->m_sampleRate = 0;
  station->m_sampleRateSlower = false;
  station->m_shortRetry = 0;
  station->m_longRetry = 0;
  station->m_retry = 0;
  station->m_err = 0;
  station->m_txRate = 0;
  station->m_initialized = false;
  station->m_sampleGroup = 0;
  return station;
}

void
MinstrelHtWifiManager::CheckInit (MinstrelHtWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);

  if (!station->m_initialized && GetNMcsSupported (station) > 1)
    {
      // Note: we appear to be doing late initialization of the table
      // to make sure that the set of supported rates has been initialized
      // before we perform our own initialization.

      NS_LOG_DEBUG ("Number of supported groups: " << (int) m_nSupportedGroups);

      station->m_sampleTable = HtSampleRate (N_MCS_GROUP_RATES, std::vector<uint32_t> (m_sampleCol));
      InitSampleTable (station);
      RateInit (station);
      station->m_initialized = true;
    }
}

void
MinstrelHtWifiManager::DoReportRxOk (WifiRemoteStation *st,
                                   double rxSnr, WifiMode txMode)
{
  NS_LOG_FUNCTION (this << st);

  NS_LOG_DEBUG ("DoReportRxOk m_txRate=" << ((MinstrelHtWifiRemoteStation *)st)->m_txRate);
}

void
MinstrelHtWifiManager::DoReportRtsFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);

  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *)st;
  NS_LOG_DEBUG ("DoReportRtsFailed m_txRate=" << station->m_txRate);

  station->m_shortRetry++;
}

void
MinstrelHtWifiManager::DoReportRtsOk (WifiRemoteStation *st, double ctsSnr, WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_FUNCTION (this << st);

  NS_LOG_DEBUG ("self=" << st << " rts ok");
}

void
MinstrelHtWifiManager::DoReportFinalRtsFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);

  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *)st;
  NS_LOG_DEBUG ("Final RTS failed");
  UpdateRetry (station);
  station->m_err++;
}

void
MinstrelHtWifiManager::DoReportDataFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);

  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *)st;
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

  /**
   * Get the ids for all rates.
   */
  uint32_t currentRateId = GetRateId (station->m_txRate);
  uint32_t currentGroupId = GetGroupId (station->m_txRate);
  uint32_t maxTpRateId = GetRateId (station->m_maxTpRate);
  uint32_t maxTpGroupId = GetGroupId (station->m_maxTpRate);
  uint32_t maxTp2RateId = GetRateId (station->m_maxTpRate2);
  uint32_t maxTp2GroupId = GetGroupId (station->m_maxTpRate2);
  uint32_t sampleTateId = GetRateId (station->m_sampleRate);
  uint32_t sampleGroupId = GetGroupId (station->m_sampleRate);

  station->m_mcsTable[currentGroupId].m_minstrelTable[currentRateId].numRateAttempt++; // Increment the attempts counter for the rate used.

  NS_LOG_DEBUG ("DoReportDataFailed " << station << "\t rate " << station->m_txRate << "\tlongRetry \t" << station->m_longRetry);

  /// For normal rate, we're not currently sampling random rates.
  if (!station->m_isSampling)
    {
      /// Use best throughput rate.
      if (station->m_longRetry <  station->m_mcsTable[currentGroupId].m_minstrelTable[currentRateId].adjustedRetryCount)
        {
          NS_LOG_DEBUG ("Not Sampling use the same rate again");
          station->m_txRate = station->m_maxTpRate;  //!<  There are still a few retries.
        }

      /// Use second best throughput rate.
      else if (station->m_longRetry <= ( station->m_mcsTable[currentGroupId].m_minstrelTable[currentRateId].adjustedRetryCount +
                                         station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxTpRateId].adjustedRetryCount))
        {
          NS_LOG_DEBUG ("Not Sampling use the Max TP2");
          station->m_txRate = station->m_maxTpRate2;
        }

      /// Use best probability rate.
      else if (station->m_longRetry <= ( station->m_mcsTable[currentGroupId].m_minstrelTable[currentRateId].adjustedRetryCount +
                                         station->m_mcsTable[maxTp2GroupId].m_minstrelTable[maxTp2RateId].adjustedRetryCount +
                                         station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxTpRateId].adjustedRetryCount))
        {
          NS_LOG_DEBUG ("Not Sampling use Max Prob");
          station->m_txRate = station->m_maxProbRate;
        }

      /// Use lowest base rate.
      /* Acording to linux code, it seems that the lowest base rate is not used in MinstrelHT.
      else if (station->m_longRetry > ( station->m_mcsTable[currentGroupId].m_minstrelTable[currentRateId].adjustedRetryCount +
                                        station->m_mcsTable[maxTp2GroupId].m_minstrelTable[maxTp2RateId].adjustedRetryCount +
                                        station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxTpRateId].adjustedRetryCount))
        {
          NS_LOG_DEBUG ("Not Sampling use MCS0");
          station->m_txRate = 0;
        }
        */
    }

  /// for look-around rate, we're currently sampling random rates
  else
    {
      /// current sampling rate is slower than the current best rate
      if (station->m_sampleRateSlower)
        {
          /// use best throughput rate
          if (station->m_longRetry <  station->m_mcsTable[currentGroupId].m_minstrelTable[currentRateId].adjustedRetryCount)
            {
                 NS_LOG_DEBUG ("Sampling use the same rate again");
               station->m_txRate = station->m_maxTpRate;///<  there are a few retries left
            }

          ///	use random rate
          else if (station->m_longRetry <= ( station->m_mcsTable[currentGroupId].m_minstrelTable[currentRateId].adjustedRetryCount +
                                             station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxtprateid].adjustedRetryCount))
            {
                NS_LOG_DEBUG ("Sampling use the sample rate");
              station->m_txRate = station->m_sampleRate;
            }

          /// use max probability rate
          else if (station->m_longRetry <= ( station->m_mcsTable[currentGroupId].m_minstrelTable[currentRateId].adjustedRetryCount +
                                             station->m_mcsTable[samplegroupid].m_minstrelTable[samplerateid].adjustedRetryCount +
                                             station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxtprateid].adjustedRetryCount ))
            {
                NS_LOG_DEBUG ("Sampling use Max prob");
              station->m_txRate =  station->m_maxProbRate;
            }

          /// use lowest base rate
          else if (station->m_longRetry > ( station->m_mcsTable[currentGroupId].m_minstrelTable[currentRateId].adjustedRetryCount +
                                            station->m_mcsTable[samplegroupid].m_minstrelTable[samplerateid].adjustedRetryCount +
                                            station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxtprateid].adjustedRetryCount))
            {
                NS_LOG_DEBUG ("Sampling use MCS0");
              station->m_txRate = 0;
            }
        }

      /// current sampling rate is better than current best rate
      else
        {
          /// use random rate
          if (station->m_longRetry <  station->m_mcsTable[groupid].m_minstrelTable[currentRateId].adjustedRetryCount)
            {
                 NS_LOG_DEBUG ("Sampling use the same sample rate");
               station->m_txRate = station->m_sampleRate;    ///< keep using it
            }

          /// use the best rate
          else if (station->m_longRetry <= ( station->m_mcsTable[currentGroupId].m_minstrelTable[currentRateId].adjustedRetryCount +
                                             station->m_mcsTable[samplegroupid].m_minstrelTable[samplerateid].adjustedRetryCount))
            {
                NS_LOG_DEBUG ("Sampling use the MaxTP rate");
              station->m_txRate = station->m_maxTpRate;
            }

          /// use the best probability rate
          else if (station->m_longRetry <= ( station->m_mcsTable[currentGroupId].m_minstrelTable[currentRateId].adjustedRetryCount +
                                             station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxtprateid].adjustedRetryCount +
                                             station->m_mcsTable[samplegroupid].m_minstrelTable[samplerateid].adjustedRetryCount))
            {
                NS_LOG_DEBUG ("Sampling use the MaxProb rate");
              station->m_txRate =station->m_maxProbRate;
            }

          /// use the lowest base rate
          else if (station->m_longRetry > ( station->m_mcsTable[currentGroupId].m_minstrelTable[currentRateId].adjustedRetryCount +
                                            station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxtprateid].adjustedRetryCount +
                                            station->m_mcsTable[samplegroupid].m_minstrelTable[samplerateid].adjustedRetryCount))
            {
              NS_LOG_DEBUG ("Sampling use the MCS0");
              station->m_txRate = 0;
            }
        }
    }
  NS_LOG_DEBUG ("Txrate = " << station->m_txRate  );
}

void
MinstrelHtWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                     double ackSnr, WifiMode ackMode, double dataSnr)
{
  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *) st;
  station->m_isSampling = false;
  station->m_sampleRateSlower = false;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }
 
  uint32_t rateid = GetRateId(station->m_txRate);
  uint32_t groupid = GetGroupId(station->m_txRate);
  station->m_mcsTable[groupid].m_minstrelTable[rateid].numRateSuccess++;
  station->m_mcsTable[groupid].m_minstrelTable[rateid].numRateAttempt++;

  UpdateRetry (station);

  //station->m_minstrelTable[station->m_txrate].numRateAttempt += station->m_retry;
  station->m_frameCount++;

  if (m_nSupportedMcs >= 1)
    {
      station->m_txRate = FindRate (station);
    }
  NS_LOG_DEBUG ("Data OK - Txrate = " << station->m_txRate  );
}

void
MinstrelHtWifiManager::DoReportFinalDataFailed (WifiRemoteStation *st)
{
  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *) st;
  NS_LOG_DEBUG ("DoReportFinalDataFailed m_txRate=" << station->m_txRate);

  station->m_isSampling = false;
  station->m_sampleRateSlower = false;

  UpdateRetry (station);

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  uint32_t rateid = GetRateId(station->m_txRate);
  uint32_t groupid = GetGroupId(station->m_txRate);
  station->m_mcsTable[groupid].m_minstrelTable[rateid].numRateAttempt++;
  station->m_err++;

  if (m_nSupportedMcs >= 1)
    {
      station->m_txRate = FindRate (station);
    }
   NS_LOG_DEBUG ("Txrate = " << station->m_txRate  );
}

void
MinstrelHtWifiManager::UpdateRetry (MinstrelHtWifiRemoteStation *station)
{
  station->m_retry = station->m_shortRetry + station->m_longRetry;
  station->m_shortRetry = 0;
  station->m_longRetry = 0;
}
void 
MinstrelHtWifiManager::DoDisposeStation (WifiRemoteStation *st)
{
   MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *) st;
   station->DisposeStation();
}

WifiTxVector
MinstrelHtWifiManager::DoGetDataTxVector (WifiRemoteStation *st,
                                    uint32_t size)
{
  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *) st;
  if (!station->m_initialized)
    {
     
      CheckInit (station);
      if (!station->m_initialized)
        {
             station->m_txRate = 0;
         }
      else
         {
           /// start the rate at half way
           station->m_txRate = m_nSupportedMcs / 2;
         }
    }
  NS_LOG_DEBUG ("DoGetDataMode m_txRate=" << station->m_txRate << " m_nsupported " << m_nSupportedMcs);
  UpdateStats (station);

  uint32_t rateId = GetRateId(station->m_txRate);
  uint32_t groupId = GetGroupId(station->m_txRate);
  McsGroup group = m_groups[groupId];

  return WifiTxVector (GetMcsSupported (station, rateId), GetDefaultTxPowerLevel (), GetLongRetryCount (station), group.sgi, group.streams, GetNess(station), group.chWidth, GetAggregation (station), GetStbc(station));
}

WifiTxVector
MinstrelHtWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *) st;
  NS_LOG_DEBUG ("DoGetRtsMode m_txRate=" << station->m_txRate);

  //Using the lowest rate of the group selected by minstrel.
  //FIXME Check which is the right thing to do here. View Section 9.7.6 of 802.11-2012 standard.
  uint32_t groupId = GetGroupId(station->m_txRate);
  McsGroup group = m_groups[groupId];

  return WifiTxVector (GetMcsSupported (station, 0), GetDefaultTxPowerLevel (), GetShortRetryCount (station), group.sgi, group.streams, GetNess(station), group.chWidth, GetAggregation (station), GetStbc(station));
}

bool
MinstrelHtWifiManager::DoNeedDataRetransmission (WifiRemoteStation *st, Ptr<const Packet> packet, bool normally)
{
  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *)st;
  uint32_t maxprobrateid = GetRateId (station->m_maxProbRate);
  uint32_t maxprobgroupid = GetGroupId (station->m_maxProbRate);
  uint32_t maxtprateid = GetRateId (station->m_maxTpRate);
  uint32_t maxtpgroupid = GetGroupId (station->m_maxTpRate);
  uint32_t maxtp2rateid = GetRateId (station->m_maxTpRate2);
  uint32_t maxtp2groupid = GetGroupId (station->m_maxTpRate2);
  uint32_t samplerateid = GetRateId (station->m_sampleRate);
  uint32_t samplegroupid = GetGroupId (station->m_sampleRate);

  CheckInit (station);
  if (!station->m_initialized)
    {
      return normally;
    }

  if (!station->m_isSampling)
    {
      if (station->m_longRetry > (station->m_mcsTable[maxtpgroupid].m_minstrelTable[maxtprateid].adjustedRetryCount +
                                  station->m_mcsTable[maxtp2groupid].m_minstrelTable[maxtp2rateid].adjustedRetryCount +
                                  station->m_mcsTable[maxprobgroupid].m_minstrelTable[maxprobrateid].adjustedRetryCount +
                                  station->m_mcsTable[0].m_minstrelTable[0].adjustedRetryCount))
        {
         NS_LOG_DEBUG ("No re-transmission allowed" );
          return false;
        }
      else
        {
          NS_LOG_DEBUG ("Re-tranmsit" );
          return true;
        }
    }
  else
    {
      if (station->m_longRetry > (station->m_mcsTable[samplegroupid].m_minstrelTable[samplerateid].adjustedRetryCount +
                                  station->m_mcsTable[maxtpgroupid].m_minstrelTable[maxtprateid].adjustedRetryCount +
                                  station->m_mcsTable[maxprobgroupid].m_minstrelTable[maxprobrateid].adjustedRetryCount +
                                  station->m_mcsTable[0].m_minstrelTable[0].adjustedRetryCount))
        {
          NS_LOG_DEBUG ("No re-transmission allowed" );
          return false;
        }
      else
        {
          NS_LOG_DEBUG ("Re-tranmsit" );
          return true;
        }
    }
}

bool
MinstrelHtWifiManager::IsLowLatency (void) const
{
  return true;
}
uint8_t
MinstrelHtWifiManager::GetStreams (uint32_t groupId, MinstrelHtWifiRemoteStation *station)
{
  uint8_t nstreams = 1;
  if (GetShortGuardInterval(station) && m_nSupportedGroups > 2)
    {
      //SGI is supported and we have more than one stream
      if (groupId > 2) //group 0 and 1 are SGI and LGI 1 stream
        nstreams = 2;
    }
  else if (!GetShortGuardInterval(station) && m_nSupportedGroups > 1)
   {
     //SGI is not supported and we have more than one stream
     if (groupId == 1)
       nstreams = 2;
   }
  return nstreams;
}
uint32_t
MinstrelHtWifiManager::GetNextSample (MinstrelHtWifiRemoteStation *station)
{
  uint32_t sampleIndex;
  uint32_t bitrate;
  sampleIndex = station->m_sampleTable[station->m_mcsTable[station->m_sampleGroup].m_index][station->m_mcsTable[station->m_sampleGroup].m_col];
  bitrate = GetIndex(station->m_sampleGroup, sampleIndex);
  station->m_mcsTable[station->m_sampleGroup].m_index++;
  station->m_sampleGroup++;
  /// bookeeping for m_index and m_col variables
  station->m_sampleGroup %= m_nSupportedGroups;
  if (station->m_mcsTable[station->m_sampleGroup].m_index > 6)
    {
      station->m_mcsTable[station->m_sampleGroup].m_index = 0;
      station->m_mcsTable[station->m_sampleGroup].m_col++;
      if (station->m_mcsTable[station->m_sampleGroup].m_col >= m_sampleCol)
        {
          station->m_mcsTable[station->m_sampleGroup].m_col = 0;
        }
    }
  NS_LOG_DEBUG ("Next Sample is" << bitrate );
  return bitrate;
}

uint32_t
MinstrelHtWifiManager::FindRate (MinstrelHtWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("FindRate " << "packet=" << station->m_frameCount );

  if ((station->m_sampleCount + station->m_frameCount) == 0)
    {
      return 0;
    }
  
  uint32_t idx;

  /// for determining when to try a sample rate
  Ptr<UniformRandomVariable> coinFlip = CreateObject<UniformRandomVariable> ();
  coinFlip->SetAttribute ("Min", DoubleValue (0));
  coinFlip->SetAttribute ("Max", DoubleValue (100));

  /**
   * if we are below the target of look around rate percentage, look around
   * note: do it randomly by flipping a coin instead sampling
   * all at once until it reaches the look around rate
   */
  if ( (((100 * station->m_sampleCount) / (station->m_sampleCount + station->m_frameCount )) < m_lookAroundRate)
       && ((int)coinFlip->GetValue ()) % 2 == 1 )
    {
       NS_LOG_DEBUG ("Sampling");
      /// now go through the table and find an index rate
      idx = GetNextSample (station);
      NS_LOG_DEBUG ("Sampling rate = " << idx);

      /**
       * This if condition is used to make sure that we don't need to use
       * the sample rate it is the same as our current rate
       */
      if (idx != station->m_maxTpRate && idx != station->m_txRate)
        {

          /// start sample count
          station->m_sampleCount++;

          /// set flag that we are currently sampling
          station->m_isSampling = true;

          /// bookeeping for resetting stuff
          if (station->m_frameCount >= 10000)
            {
              station->m_sampleCount = 0;
              station->m_frameCount = 0;
            }

          /// error check
         // if (idx >= m_nSupportedMcs)//FIXME
           // {
             // NS_LOG_DEBUG ("ALERT!!! ERROR");
           // }

          /// set the rate that we're currently sampling
          station->m_sampleRate = idx;

          if (station->m_sampleRate == station->m_maxTpRate)
            {
              station->m_sampleRate = station->m_maxTpRate2;
            }

          /// is this rate slower than the current best rate
          uint32_t idxgroupid = GetGroupId(idx);
          uint32_t idxrateid = GetRateId(idx);
          uint32_t maxTpgroupid = GetGroupId(station->m_maxTpRate);
          uint32_t maxTprateid = GetRateId(station->m_maxTpRate);
          station->m_sampleRateSlower =
            (station->m_mcsTable[idxgroupid].m_minstrelTable[idxrateid].perfectTxTime >
            station->m_mcsTable[maxTpgroupid].m_minstrelTable[maxTprateid].perfectTxTime);

          /// using the best rate instead
          if (station->m_sampleRateSlower)
            {
              idx =  station->m_maxTpRate;
            }
        }

    }

  ///	continue using the best rate
  else
    {
      idx = station->m_maxTpRate;
    }


  NS_LOG_DEBUG ("FindRate " << "sample rate=" << idx);

  return idx;
}
uint32_t
MinstrelHtWifiManager::GetIndex(uint32_t groupid, uint32_t mcsIndex)
{
  uint32_t index;
  index = groupid * MCS_GROUP_RATES + mcsIndex;
  return index;
}
void
MinstrelHtWifiManager::UpdateStats (MinstrelHtWifiRemoteStation *station)
{
  if (Simulator::Now () <  station->m_nextStatsUpdate)
    {
      return;
    }

  if (!station->m_initialized)
    {
      return;
    }

  NS_LOG_DEBUG ("Updating stats=" << this);

  station->m_nextStatsUpdate = Simulator::Now () + m_updateStats;
 
  Time txTime;
  uint32_t tempProb;

 
  /// Update throughput and EWMA for each rate inside each group.
  for (uint32_t j=0 ; j < m_nSupportedGroups ; j++)
    {
      for (uint32_t i = 0; i < m_nSupportedMcs; i++)
        {

          /// Calculate the perfect tx time for this rate.
          txTime =  station->m_mcsTable[j].m_minstrelTable[i].perfectTxTime;

          /// Just for initialization.
          if (txTime.GetMicroSeconds () == 0)
            {
              txTime = Seconds (1);
            }

          NS_LOG_DEBUG (i << " " << GetMcsSupported (station, i) <<
                        "\t attempt=" << station->m_mcsTable[j].m_minstrelTable[i].numRateAttempt <<
                        "\t success=" << station->m_mcsTable[j].m_minstrelTable[i].numRateSuccess);

          /// If we've attempted something.
          if (station->m_mcsTable[j].m_minstrelTable[i].numRateAttempt)
            {
              /**
               * Calculate the probability of success.
               * Assume probability scales from 0 to 18000.
               */
              tempProb = (station->m_mcsTable[j].m_minstrelTable[i].numRateSuccess * 18000) / station->m_mcsTable[j].m_minstrelTable[i].numRateAttempt;

              /// Bookeeping.
              station->m_mcsTable[j].m_minstrelTable[i].prob = tempProb;

              /// EWMA probability (cast for gcc 3.4 compatibility).
              tempProb = static_cast<uint32_t> (((tempProb * (100 - m_ewmaLevel)) + (station->m_mcsTable[j].m_minstrelTable[i].ewmaProb * m_ewmaLevel) ) / 100);

              station->m_mcsTable[j].m_minstrelTable[i].ewmaProb = tempProb;

              /// Calculating throughput.
              station->m_mcsTable[j].m_minstrelTable[i].throughput = tempProb * (1000000 / txTime.GetMicroSeconds ());

            }

          /// Bookeeping.
          station->m_mcsTable[j].m_minstrelTable[i].numRateSuccess = 0;
          station->m_mcsTable[j].m_minstrelTable[i].numRateAttempt = 0;

          /// Sample less often below 10% and  above 95% of success.
          if ((station->m_mcsTable[j].m_minstrelTable[i].ewmaProb > 17100) || (station->m_mcsTable[j].m_minstrelTable[i].ewmaProb < 1800))
            {
              /**
               * retry count denotes the number of retries permitted for each rate
               * # retry_count/2
               */

              if (station->m_mcsTable[j].m_minstrelTable[i].adjustedRetryCount > 2)
                {
                  station->m_mcsTable[j].m_minstrelTable[i].adjustedRetryCount = 2;
                }
              else
                {
                  station->m_mcsTable[j].m_minstrelTable[i].adjustedRetryCount = station->m_mcsTable[j].m_minstrelTable[i].retryCount;
                }
            }
          else
            {
              station->m_mcsTable[j].m_minstrelTable[i].adjustedRetryCount = station->m_mcsTable[j].m_minstrelTable[i].retryCount;
            }

          /// If it's 0 allow one retry limit.
          if (station->m_mcsTable[j].m_minstrelTable[i].adjustedRetryCount == 0)
            {
              station->m_mcsTable[j].m_minstrelTable[i].adjustedRetryCount = 1;
            }
        }
    }

 uint32_t max_prob = 0, index_max_prob = 0, max_tp = 0, index_max_tp = 0, index_max_tp2 = 0;
 uint32_t index = 0;

 /// For each group get the max_tp and max_tp2.
 for (uint32_t j = 0; j < m_nSupportedGroups; j++)
   {
     max_prob = 0;
     index_max_prob = 0;
     max_tp = 0;
     index_max_tp = 0;
     index_max_tp2 = 0;

    /// Go find maximum throughput, second maximum throughput and high probability of success rates.
    for (uint32_t i = 0; i < m_nSupportedMcs; i++)
      {
        index = GetIndex (j,i);

        NS_LOG_DEBUG ("throughput" << station->m_mcsTable[j].m_minstrelTable[i].throughput <<
                      "\n ewma" << station->m_mcsTable[j].m_minstrelTable[i].ewmaProb);

        if (max_tp < station->m_mcsTable[j].m_minstrelTable[i].throughput)
          {
            index_max_tp = index;
            max_tp = station->m_mcsTable[j].m_minstrelTable[i].throughput;
          }

        if (max_prob < station->m_mcsTable[j].m_minstrelTable[i].ewmaProb)
          {
            index_max_prob = index;
            max_prob = station->m_mcsTable[j].m_minstrelTable[i].ewmaProb;
          }
      }
    max_tp = 0;
    /// Find the second maximum throughput rate.
    for (uint32_t i = 0; i < m_nSupportedMcs; i++)
      {
        index = GetIndex (j,i);

        if ((i != index_max_tp) && (max_tp < station->m_mcsTable[j].m_minstrelTable[i].throughput))
          {
            index_max_tp2 = index;
            max_tp = station->m_mcsTable[j].m_minstrelTable[i].throughput;
          }
      }

    station->m_mcsTable[j].m_maxTpRate = index_max_tp;
    station->m_mcsTable[j].m_maxTpRate2 = index_max_tp2;
    station->m_mcsTable[j].m_maxProbRate = index_max_prob;
  }

  /// Get the max_tp and max_tp2 from all groups.
  index = 0;
  max_prob = 0;
  index_max_prob = 0;
  max_tp = 0;
  index_max_tp = 0;
  index_max_tp2 = 0;
  
  for (uint32_t j = 0; j < m_nSupportedGroups; j++)
    {
      /// Go find maximum throughput, second maximum throughput and high probability of success rates.
      if (max_tp < station->m_mcsTable[j].m_minstrelTable[station->m_mcsTable[j].m_maxTpRate].throughput)
        {
          index_max_tp = station->m_mcsTable[j].m_maxTpRate;
          max_tp = station->m_mcsTable[j].m_minstrelTable[GetIndex(j, station->m_mcsTable[j].m_maxTpRate)].throughput;
        }

      if (max_prob < station->m_mcsTable[j].m_minstrelTable[station->m_mcsTable[j].m_maxProbRate].ewmaProb)
        {
          index_max_prob = station->m_mcsTable[j].m_maxProbRate;
          max_prob = station->m_mcsTable[j].m_minstrelTable[GetIndex(j, station->m_mcsTable[j].m_maxProbRate)].ewmaProb;
        }
    }
  max_tp = 0;
  /// Find the second highest maximum throughput rate.
  for (uint32_t i = 0; i <m_nSupportedGroups; i++)
    {
      if ((GetIndex(i,station->m_mcsTable[i].m_maxTpRate) != index_max_tp) && (max_tp < station->m_mcsTable[i].m_minstrelTable[station->m_mcsTable[i].m_maxTpRate].throughput))
        {
          /// Find if another group's max_tp is better than the max_tp2.
          index_max_tp2 = station->m_mcsTable[i].m_maxTpRate;
          max_tp = station->m_mcsTable[i].m_minstrelTable[GetIndex (i, station->m_mcsTable[i].m_maxTpRate)].throughput;
        }
      if (max_tp < station->m_mcsTable[i].m_minstrelTable[station->m_mcsTable[i].m_maxTpRate2].throughput)
        {
          /// Find if another group's max_tp2 is better than max_tp2.
          index_max_tp2 = station->m_mcsTable[i].m_maxTpRate2;
          max_tp = station->m_mcsTable[i].m_minstrelTable[GetIndex (i, station->m_mcsTable[i].m_maxTpRate2)].throughput;
        }
    }

  station->m_maxTpRate = index_max_tp;
  station->m_maxTpRate2 = index_max_tp2;
  station->m_maxProbRate = index_max_prob;

  /// If the max_tp rate is bigger than the current rate and uses the same number of streams.
  if ((index_max_tp > station->m_txRate) && (m_groups[GetGroupId(index_max_tp)].streams >= m_groups[GetGroupId(station->m_txRate)].streams) )
    {
      station->m_txRate = index_max_tp;
    }

  NS_LOG_DEBUG ("max tp=" << index_max_tp << "\nmax tp2=" << index_max_tp2 << "\nmax prob=" << index_max_prob);

  /// reset it
  //RateInit (station);
}

void
MinstrelHtWifiManager::RateInit (MinstrelHtWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("RateInit=" << station);
 
  for (uint32_t j = 0; j < N_MINSTREL_GROUPS; j++)
  {
    /// Check if the group is supported
    if (!(m_groups[j].sgi && GetShortGuardInterval(station))) ///Is SGI supported?
      {
        station->m_mcsTable[j].m_supported = false;
        break;
      }
    if (m_groups[j].chWidth > GetChannelWidth(station))  //Is channel width supported?
      {
        station->m_mcsTable[j].m_supported = false;
        break;
      }
    if (m_groups[j].streams != GetNumberOfReceiveAntennas(station)) //FIXME Is this the correct way to check the number of streams?
    {
      station->m_mcsTable[j].m_supported = false;
      break;
    }

    station->m_mcsTable[j].m_supported = true;
    station->m_mcsTable[j].m_minstrelTable = HtMinstrelRate (N_MCS_GROUP_RATES);
    station->m_mcsTable[j].m_col = 0;
    station->m_mcsTable[j].m_index = 0;
    for (uint32_t i = 0; i < m_nSupportedMcs; i++)
     {
       station->m_mcsTable[j].m_minstrelTable[i].numRateAttempt = 0;
       station->m_mcsTable[j].m_minstrelTable[i].numRateSuccess = 0;
       station->m_mcsTable[j].m_minstrelTable[i].prob = 0;
       station->m_mcsTable[j].m_minstrelTable[i].ewmaProb = 0;
       station->m_mcsTable[j].m_minstrelTable[i].prevNumRateAttempt = 0;
       station->m_mcsTable[j].m_minstrelTable[i].prevNumRateSuccess = 0;
       station->m_mcsTable[j].m_minstrelTable[i].successHist = 0;
       station->m_mcsTable[j].m_minstrelTable[i].attemptHist = 0;
       station->m_mcsTable[j].m_minstrelTable[i].throughput = 0;
       station->m_mcsTable[j].m_minstrelTable[i].perfectTxTime = GetCalcTxTime (j, GetPhy()->GetMcs(i));
       station->m_mcsTable[j].m_minstrelTable[i].retryCount = 1;
       station->m_mcsTable[j].m_minstrelTable[i].adjustedRetryCount = 1;
       //Emulating minstrel.c::ath_rate_ctl_reset
       //We only check from 2 to 10 retries. This guarantee that
       //at least one retry is permitted.
       Time totalTxTimeWithGivenRetries = Seconds (0.0); //tx_time in minstrel.c
       NS_LOG_DEBUG (" Calculating the number of retries");
       for (uint32_t retries = 2; retries < 11; retries++)
         {
           NS_LOG_DEBUG ("  Checking " << retries << " retries");
           totalTxTimeWithGivenRetries = CalculateTimeUnicastPacket (station->m_mcsTable[j].m_minstrelTable[i].perfectTxTime, 0, retries);
           NS_LOG_DEBUG ("   totalTxTimeWithGivenRetries = " << totalTxTimeWithGivenRetries);
           if (totalTxTimeWithGivenRetries > MilliSeconds (6))
             {
               break;
             }
           station->m_mcsTable[j].m_minstrelTable[i].retryCount = retries;
           station->m_mcsTable[j].m_minstrelTable[i].adjustedRetryCount = retries;
         }
    }
  }
}

Time
MinstrelHtWifiManager::CalculateTimeUnicastPacket (Time dataTransmissionTime, uint32_t shortRetries, uint32_t longRetries)
{
  NS_LOG_FUNCTION (this << dataTransmissionTime << shortRetries << longRetries);
  //See rc80211_minstrel.c

  //First transmission (DATA + ACK timeout)
  Time tt = dataTransmissionTime + GetMac ()->GetAckTimeout ();

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
MinstrelHtWifiManager::InitSampleTable (MinstrelHtWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("InitSampleTable=" << this);

  station->m_col = station->m_index = 0;

  /// for off-seting to make rates fall between 0 and numrates
  uint32_t numSampleRates = 8;

  uint32_t newIndex;
  for (uint32_t col = 0; col < m_sampleCol; col++)
    {
      for (uint32_t i = 0; i < numSampleRates; i++ )
        {

          /**
           * The next two lines basically tries to generate a random number
           * between 0 and the number of available rates
           */
          Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable> ();
          uv->SetAttribute ("Min", DoubleValue (0));
          uv->SetAttribute ("Max", DoubleValue (numSampleRates));
        
          newIndex = (i + (uint32_t)uv->GetValue ()) % numSampleRates;

          /// this loop is used for filling in other uninitilized places
          while (station->m_sampleTable[newIndex][col] != 0)
            {
              newIndex = (newIndex + 1) % 8;
            }
          station->m_sampleTable[newIndex][col] = i;

        }
    }
}

void
MinstrelHtWifiManager::PrintSampleTable (MinstrelHtWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("PrintSampleTable=" << station);

  uint32_t numSampleRates = 8;
  for (uint32_t i = 0; i < numSampleRates; i++)
    {
      for (uint32_t j = 0; j < m_sampleCol; j++)
        {
          std::cout << station->m_sampleTable[i][j] << "\t";
        }
      std::cout << std::endl;
    }
}

void
MinstrelHtWifiManager::PrintTable (MinstrelHtWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("PrintTable=" << station);
  for (uint32_t j = 0; j < m_nSupportedGroups; j++)
   {
     for (uint32_t i = 0; i < 8; i++)
      {
        std::cout << "index(" << i << ") = " << station->m_mcsTable[j].m_minstrelTable[i].perfectTxTime << "\n";
      }
   }
}

uint32_t
MinstrelHtWifiManager::GetRateId(uint32_t index)
{
  NS_LOG_FUNCTION (this << index);

  uint32_t id;
  id = index % N_MCS_GROUP_RATES;
  return id;
}

uint32_t
MinstrelHtWifiManager::GetGroupId(uint32_t index)
{
  NS_LOG_FUNCTION (this << index);

  return index / N_MCS_GROUP_RATES;
}

uint32_t
MinstrelHtWifiManager::GetGroupId(uint8_t txstreams, uint8_t sgi, uint8_t ht40)
{
  NS_LOG_FUNCTION (this << txstreams << sgi << ht40);

  return N_MAX_STREAMS * 2 * ht40 + N_MAX_STREAMS * sgi + txstreams - 1;
}

} // namespace ns3





