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
 *         Matias Richart <mrichart@fing.edu.uy>
 *
 * Some Comments:
 *
 * 1) By default, Minstrel applies the multi-rate retry(the core of Minstrel
 *    algorithm). Otherwise, please use ConstantRateWifiManager instead.
 *
 * 2) Currently it doesn't support aggregation. It is not considered in tx time calculations
 *    and in retries.
 *
 * 3) Sampling is done different as it was in legacy Minstrel. Minstrel-HT tries to sample
 * all rates in all groups at least once and to avoid many consecutive samplings.
 *
 * 4) Sample rate is tried only once, at first place of the MRR chain.
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
#include "ns3/boolean.h"
#include <vector>

#define Min(a,b) ((a < b) ? a : b)

NS_LOG_COMPONENT_DEFINE ("MinstrelHtWifiManager");


namespace ns3 {


struct MinstrelHtWifiRemoteStation : MinstrelWifiRemoteStation
{
  void DisposeStation ();

  uint32_t m_sampleGroup;     //!< The group that the sample rate belongs to.

  uint32_t m_sampleWait;    //!< How many transmission attempts to wait until a new sample.
  uint32_t m_sampleTries;   //!< Number of sample tries after waiting sampleWait.
  uint32_t m_sampleCount;   //!< Max number of samples per update interval.
  uint32_t m_numSamplesSlow;  //!< Number of times a slow rate was sampled.

  double m_avgAmpduLen;    //!< Average number of MPDUs in an A-MPDU.
  double m_ampduLen;    //!< Number of MPDUs in an A-MPDU.
  uint32_t m_ampduPacketCount;  //!< Number of A-MPDUs transmitted.

  McsGroupData m_mcsTable;      //!< Table of groups with stats.
  bool m_isHt;                  //!< If the station is HT capable.
};

void
MinstrelHtWifiRemoteStation::DisposeStation ()
{
  if (m_isHt)
    {
      std::vector<std::vector<uint32_t> > ().swap (m_sampleTable);
      for (uint8_t j = 0; j < m_mcsTable.size (); j++)
        {
          std::vector<struct HtRateInfo> ().swap (m_mcsTable[j].m_minstrelTable);
        }
      std::vector<struct GroupInfo> ().swap (m_mcsTable);
    }
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
                   "The percentage to try other rates (for legacy Minstrel)",
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
                   MakeUintegerAccessor (&MinstrelHtWifiManager::m_nSampleCol),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("PacketLength",
                   "The packet length used for calculating mode TxTime",
                   UintegerValue (1200),
                   MakeUintegerAccessor (&MinstrelHtWifiManager::m_frameLength),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("UseVhtOnly",
                   "The packet length used for calculating mode TxTime",
                   BooleanValue (false),
                   MakeBooleanAccessor (&MinstrelHtWifiManager::m_useVhtOnly),
                   MakeBooleanChecker())
    .AddTraceSource ("RateChange",
                     "The transmission rate has change",
                     MakeTraceSourceAccessor (&MinstrelHtWifiManager::m_rateChange),
                     "ns3::MinstrelHtWifiManager::RateChangeTracedCallback")
  ;
  return tid;
}

MinstrelHtWifiManager::MinstrelHtWifiManager ()
  : m_numGroups (0),
    m_numRates (0)
{
  NS_LOG_FUNCTION (this);
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
  /**
   *  Create the legacy Minstrel manager in case HT is not supported by the device
   *  or non-HT stations want to associate.
   */
  m_legacyManager = CreateObject<MinstrelWifiManager>();
}

MinstrelHtWifiManager::~MinstrelHtWifiManager ()
{
  NS_LOG_FUNCTION (this);
  if (HasHtSupported())
    {
      for (uint32_t i = 0; i < m_numGroups; i++)
        {
          m_minstrelGroups[i].firstMpduTxTime.clear ();
          m_minstrelGroups[i].mpduTxTime.clear ();
        }
    }
}

int64_t
MinstrelHtWifiManager::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  m_legacyManager->AssignStreams(stream);
  return 1;
}

void
MinstrelHtWifiManager::SetupPhy (Ptr<WifiPhy> phy)
{
  NS_LOG_FUNCTION (this << phy);

  // Check if the device supports HT
  if (HasHtSupported() || HasVhtSupported())
    {
      if (HasVhtSupported())
        {
          m_numGroups = MAX_SUPPORTED_STREAMS * MAX_VHT_STREAM_GROUPS;
          m_numRates = MAX_HT_GROUP_RATES + MAX_VHT_GROUP_RATES;
          m_maxChWidth = MAX_VHT_WIDTH;
        }
      else
        {
          m_numGroups = MAX_SUPPORTED_STREAMS * MAX_HT_STREAM_GROUPS;
          m_numRates = MAX_HT_GROUP_RATES;
          m_maxChWidth = MAX_HT_WIDTH;
        }
      /**
       * Initialize the groups array.
       */
      NS_LOG_DEBUG ("Initialize MCS Groups:");
      m_minstrelGroups = MinstrelMcsGroups (m_numGroups);
      for (uint8_t streams = 1; streams <= MAX_SUPPORTED_STREAMS; streams++)
        {
          for (uint8_t sgi = 0; sgi <= 1; sgi++)
            {
              for (uint32_t chWidth = 20; chWidth <= m_maxChWidth; chWidth*=2)
                {
                  m_minstrelGroups[GetGroupId (streams, sgi, chWidth)].streams = streams;
                  m_minstrelGroups[GetGroupId (streams, sgi, chWidth)].sgi = sgi;
                  m_minstrelGroups[GetGroupId (streams, sgi, chWidth)].chWidth = chWidth;
                  for (uint8_t i = 0; i < m_numRates; i++)
                    {
                      ///Check for invalid VHT MCS
                      ///TODO Other combinations are also invalid
                      if (!(chWidth == 20) || (chWidth == 20 && phy->GetMcs(i).GetMcsValue() != 9))
                        {
                          if (streams > 4 || chWidth > 40 || i > 7)
                            {
                              m_minstrelGroups[GetGroupId (streams, sgi, chWidth)].isVht = true;
                            }
                          else
                            {
                              m_minstrelGroups[GetGroupId (streams, sgi, chWidth)].isVht = false;
                            }
                          WifiMode mode = phy->GetMcs (i);
                          AddFirstMpduTxTime (GetGroupId (streams, sgi, chWidth), mode, CalculateFirstMpduTxDuration (phy, streams, sgi, chWidth, mode));
                          AddMpduTxTime (GetGroupId (streams, sgi, chWidth), mode, CalculateMpduTxDuration (phy, streams, sgi, chWidth, mode));
                        }
                    }
                  NS_LOG_DEBUG ("Initialized group " << GetGroupId (streams, sgi, chWidth) << ": (" << (uint32_t)streams << "," << (uint32_t)sgi << "," << chWidth << ")");
                }
            }
        }
    }

  // Setup phy for legacy manager.
  m_legacyManager->SetupPhy(phy);
  WifiRemoteStationManager::SetupPhy (phy);
}

void
MinstrelHtWifiManager::SetupMac (Ptr<WifiMac> mac)
{
  m_legacyManager->SetupMac(mac);
  WifiRemoteStationManager::SetupMac (mac);
}

Time
MinstrelHtWifiManager::CalculateFirstMpduTxDuration (Ptr<WifiPhy> phy, uint8_t streams, uint8_t sgi, uint32_t chWidth, WifiMode mode)
{
  NS_LOG_FUNCTION (this << phy << (int)streams << (int)sgi << chWidth << mode);

  WifiTxVector txvector;
  txvector.SetNss (streams);
  txvector.SetShortGuardInterval (sgi);
  txvector.SetChannelWidth (chWidth);
  txvector.SetNess (0);
  txvector.SetStbc (phy->GetStbc ());
  txvector.SetMode (mode);
  return phy->CalculateTxDuration (m_frameLength, txvector, WIFI_PREAMBLE_HT_MF, phy->GetFrequency (), MPDU_IN_AGGREGATE, 0);
}

Time
MinstrelHtWifiManager::CalculateMpduTxDuration (Ptr<WifiPhy> phy, uint8_t streams, uint8_t sgi, uint32_t chWidth, WifiMode mode)
{
  NS_LOG_FUNCTION (this << phy << (int)streams << (int)sgi << chWidth << mode);

  WifiTxVector txvector;
  txvector.SetNss (streams);
  txvector.SetShortGuardInterval (sgi);
  txvector.SetChannelWidth (chWidth);
  txvector.SetNess (0);
  txvector.SetStbc (phy->GetStbc ());
  txvector.SetMode (mode);
  return phy->CalculateTxDuration (m_frameLength, txvector, WIFI_PREAMBLE_NONE, phy->GetFrequency (), MPDU_IN_AGGREGATE, 0);
}

Time
MinstrelHtWifiManager::GetFirstMpduTxTime (uint32_t groupId, WifiMode mode) const
{
  NS_LOG_FUNCTION (this << groupId << mode);

  for (TxTime::const_iterator i = m_minstrelGroups[groupId].firstMpduTxTime.begin (); i != m_minstrelGroups[groupId].firstMpduTxTime.end (); i++)
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
MinstrelHtWifiManager::AddFirstMpduTxTime (uint32_t groupId, WifiMode mode, Time t)
{
  NS_LOG_FUNCTION (this << groupId << mode << t);

  m_minstrelGroups[groupId].firstMpduTxTime.push_back (std::make_pair (t, mode));
}

Time
MinstrelHtWifiManager::GetMpduTxTime (uint32_t groupId, WifiMode mode) const
{
  NS_LOG_FUNCTION (this << groupId << mode);

  for (TxTime::const_iterator i = m_minstrelGroups[groupId].mpduTxTime.begin (); i != m_minstrelGroups[groupId].mpduTxTime.end (); i++)
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
MinstrelHtWifiManager::AddMpduTxTime (uint32_t groupId, WifiMode mode, Time t)
{
  NS_LOG_FUNCTION (this << groupId << mode << t);

  m_minstrelGroups[groupId].mpduTxTime.push_back (std::make_pair (t, mode));
}

WifiRemoteStation *
MinstrelHtWifiManager::DoCreateStation (void) const
{
  NS_LOG_FUNCTION (this);

  MinstrelHtWifiRemoteStation *station = new MinstrelHtWifiRemoteStation ();

  // Initialize variables common to both stations.
  station->m_nextStatsUpdate = Simulator::Now ();
  station->m_col = 0;
  station->m_index = 0;
  station->m_maxTpRate = 0;
  station->m_maxTpRate2 = 0;
  station->m_maxProbRate = 0;
  station->m_nModes = 0;
  station->m_totalPacketsCount = 0;
  station->m_samplePacketsCount = 0;
  station->m_isSampling = false;
  station->m_sampleRate = 0;
  station->m_sampleRateSlower = false;
  station->m_shortRetry = 0;
  station->m_longRetry = 0;
  station->m_txrate = 0;
  station->m_initialized = false;

  // Variables specific to HT station
  station->m_sampleGroup = 0;
  station->m_numSamplesSlow = 0;
  station->m_sampleCount = 16;
  station->m_sampleWait = 0;
  station->m_sampleTries = 4;

  station->m_avgAmpduLen = 1;
  station->m_ampduLen = 0;
  station->m_ampduPacketCount = 0;

  // If the device supports HT
  if (HasHtSupported() || HasVhtSupported())
    {
      /**
       * Assume the station is HT.
       * When correct information available it will be checked.
       */
      station->m_isHt = true;
    }
  // Use the variable in the station to indicate that the device do not support HT
  else
    {
      station->m_isHt = false;
    }
  return station;
}

void
MinstrelHtWifiManager::CheckInit (MinstrelHtWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);

  // Note: we appear to be doing late initialization of the table
  // to make sure that the set of supported rates has been initialized
  // before we perform our own initialization.
  if (!station->m_initialized)
    {
      /**
       *  Check if the station supports HT.
       *  Assume that if the device do not supports HT then
       *  the station will not support HT either.
       *  We save from using another check and variable.
       */
      if (!GetHtSupported(station) && !GetVhtSupported(station))
        {
          NS_LOG_DEBUG ("Non-HT station " << station);
          station->m_isHt = false;
          m_legacyManager->SetAttribute("UpdateStatistics", TimeValue (m_updateStats));
          m_legacyManager->SetAttribute("LookAroundRate", DoubleValue (m_lookAroundRate));
          m_legacyManager->SetAttribute("EWMA", DoubleValue (m_ewmaLevel));
          m_legacyManager->SetAttribute("SampleColumn", UintegerValue (m_nSampleCol));
          m_legacyManager->SetAttribute("PacketLength", UintegerValue (m_frameLength));
          m_legacyManager->CheckInit(station);
        }
      else
        {
          NS_LOG_DEBUG ("HT station " << station);
          station->m_isHt = true;
          station->m_nModes = GetNMcsSupported (station);
          station->m_sampleTable = SampleRate (station->m_nModes, std::vector<uint32_t> (m_nSampleCol));
          m_legacyManager->InitSampleTable (station);
          RateInit (station);
          station->m_initialized = true;
        }
    }
}

void
MinstrelHtWifiManager::DoReportRxOk (WifiRemoteStation *st,
                                     double rxSnr, WifiMode txMode)
{
  NS_LOG_FUNCTION (this << st);

  NS_LOG_DEBUG ("DoReportRxOk m_txrate=" << ((MinstrelHtWifiRemoteStation *)st)->m_txrate);
}

void
MinstrelHtWifiManager::DoReportRtsFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);

  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *)st;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  NS_LOG_DEBUG ("DoReportRtsFailed m_txrate=" << station->m_txrate);
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

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  UpdateRetry (station);
}

void
MinstrelHtWifiManager::DoReportDataFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);

  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *)st;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  if (!station->m_isHt)
    {
      m_legacyManager->UpdateRate(station);
    }
//  else
//    {
//      NS_ASSERT_MSG(false,"DoReportDataFailed should not be called in HT or VHT modes.");
//    }
}

void
MinstrelHtWifiManager::UpdateRate(MinstrelHtWifiRemoteStation *station)
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
   *
   * Note: For clarity, multiple blocks of if's and else's are used
   * Following implementation in linux, in MinstrelHT Lowest baserate is not used.
   * Explanation can be found here: http://marc.info/?l=linux-wireless&m=144602778611966&w=2
   */

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }
  station->m_longRetry += 60;

  /**
   * Get the ids for all rates.
   */
  uint32_t currentRateId = GetRateId (station->m_txrate);
  uint32_t currentGroupId = GetGroupId (station->m_txrate);
  uint32_t maxTpRateId = GetRateId (station->m_maxTpRate);
  uint32_t maxTpGroupId = GetGroupId (station->m_maxTpRate);
  uint32_t maxTp2RateId = GetRateId (station->m_maxTpRate2);
  uint32_t maxTp2GroupId = GetGroupId (station->m_maxTpRate2);
  uint32_t maxProbRateId = GetRateId (station->m_maxProbRate);
  uint32_t maxProbGroupId = GetGroupId (station->m_maxProbRate);

  station->m_mcsTable[currentGroupId].m_minstrelTable[currentRateId].numRateAttempt++; // Increment the attempts counter for the rate used.

  NS_LOG_DEBUG ("DoReportDataFailed " << station << "\t rate " << station->m_txrate << "\tlongRetry \t" << station->m_longRetry);

  /// For normal rate, we're not currently sampling random rates.
  if (!station->m_isSampling)
    {
      /// Use best throughput rate.
      if (station->m_longRetry <  station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxTpRateId].adjustedRetryCount)
        {
          NS_LOG_DEBUG ("Not Sampling use the same rate again");
          station->m_txrate = station->m_maxTpRate;  //!<  There are still a few retries.
        }

      /// Use second best throughput rate.
      else if (station->m_longRetry < ( station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxTpRateId].adjustedRetryCount +
                                         station->m_mcsTable[maxTp2GroupId].m_minstrelTable[maxTp2RateId].adjustedRetryCount))
        {
          NS_LOG_DEBUG ("Not Sampling use the Max TP2");
          station->m_txrate = station->m_maxTpRate2;
        }

      /// Use best probability rate.
      else if (station->m_longRetry <= ( station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxTpRateId].adjustedRetryCount +
                                         station->m_mcsTable[maxTp2GroupId].m_minstrelTable[maxTp2RateId].adjustedRetryCount +
                                         station->m_mcsTable[maxProbGroupId].m_minstrelTable[maxProbRateId].adjustedRetryCount))
        {
          NS_LOG_DEBUG ("Not Sampling use Max Prob");
          station->m_txrate = station->m_maxProbRate;
        }
      else
        {
          NS_ASSERT_MSG(false,"Max retries reached and m_longRetry not cleared properly.");
        }
    }

  /// We're currently sampling random rates.
  else
    {
      /// Sample rate is used only once
      /// Use the best rate.
      if (station->m_longRetry < 1 + station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxTp2RateId].adjustedRetryCount)
        {
          NS_LOG_DEBUG ("Sampling use the MaxTP rate");
          station->m_txrate = station->m_maxTpRate2;
        }

      /// Use the best probability rate.
      else if (station->m_longRetry <= 1 + station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxTp2RateId].adjustedRetryCount +
                                         station->m_mcsTable[maxProbGroupId].m_minstrelTable[maxProbRateId].adjustedRetryCount)
        {
          NS_LOG_DEBUG ("Sampling use the MaxProb rate");
          station->m_txrate = station->m_maxProbRate;
        }
      else
        {
          NS_ASSERT_MSG(false,"Max retries reached and m_longRetry not cleared properly.");
        }
    }
  NS_LOG_DEBUG ("Next rate to use TxRate = " << station->m_txrate);
}

void
MinstrelHtWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                       double ackSnr, WifiMode ackMode, double dataSnr)
{
  NS_LOG_FUNCTION (this << st << ackSnr << ackMode << dataSnr);
  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *) st;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  NS_LOG_DEBUG ("Data OK - Txrate = " << station->m_txrate  );

  station->m_isSampling = false;
  station->m_sampleRateSlower = false;

  station->m_totalPacketsCount++;

  if (!station->m_isHt)
    {
      station->m_minstrelTable[station->m_txrate].numRateSuccess++;
      station->m_minstrelTable[station->m_txrate].numRateAttempt++;

      UpdateRetry (station);
      m_legacyManager->UpdateStats (station);

      if (station->m_nModes >= 1)
        {
          station->m_txrate = m_legacyManager->FindRate(station);
        }
    }
//  ack of management frames (like assoc request/response) generates a call
//  else
//    {
//      NS_ASSERT_MSG(false,"DoReportDataOk should not be called in HT or VHT modes.");
//    }

  NS_LOG_DEBUG ("Next rate to use TxRate = " << station->m_txrate  );
}

void
MinstrelHtWifiManager::DoReportFinalDataFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *) st;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  NS_LOG_DEBUG ("DoReportFinalDataFailed - TxRate=" << station->m_txrate);

  station->m_isSampling = false;
  station->m_sampleRateSlower = false;
  UpdateRetry (station);

  if (!station->m_isHt)
    {
      m_legacyManager->UpdateStats (station);
      if (station->m_nModes >= 1)
        {
          station->m_txrate =m_legacyManager-> FindRate (station);
        }
    }
  else
    {
      NS_ASSERT_MSG(false,"DoReportFinalDataFailed should not be called in HT or VHT modes.");
    }
}

void
MinstrelHtWifiManager::DoReportAmpduTxStatus (WifiRemoteStation *st, uint32_t nSuccessfulMpdus, uint32_t nFailedMpdus)
{
  NS_LOG_FUNCTION (this << st << nSuccessfulMpdus << nFailedMpdus);
  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *) st;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  if (!station->m_isHt)
    {
      NS_ASSERT_MSG(false,"A-MPDU Tx Status called but no HT or VHT supported.");
    }

  NS_LOG_DEBUG ("DoReportAmpduTxStatus. TxRate=" << station->m_txrate << " SuccMpdus= " <<
                nSuccessfulMpdus << " FailedMpdus= " << nFailedMpdus);

  station->m_ampduPacketCount++;
  station->m_ampduLen += nSuccessfulMpdus + nFailedMpdus;// TODO numMpdus;

  if (nSuccessfulMpdus == 0 && nFailedMpdus == 0)
    {
      // We not receive a BlockAck. The entire AMPDU fail.
      UpdateRate(station); //TODO we should update attempts with the number of MPDUs
    }
  else
    {
      uint32_t rateId = GetRateId (station->m_txrate);
      uint32_t groupId = GetGroupId (station->m_txrate);
      station->m_mcsTable[groupId].m_minstrelTable[rateId].numRateSuccess += nSuccessfulMpdus;
      station->m_mcsTable[groupId].m_minstrelTable[rateId].numRateAttempt += nSuccessfulMpdus + nFailedMpdus;

      station->m_isSampling = false;
      station->m_sampleRateSlower = false;

      UpdateRetry (station);
      UpdateSampleCounts (station);
      UpdateStats (station);

      if (station->m_nModes >= 1)
        {
          station->m_txrate = FindRate (station);
        }
      NS_LOG_DEBUG ("Next rate to use TxRate = " << station->m_txrate  );
    }
}

void
MinstrelHtWifiManager::UpdateRetry (MinstrelHtWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  station->m_shortRetry = 0;
  station->m_longRetry = 0;

}

void
MinstrelHtWifiManager::UpdateSampleCounts (MinstrelHtWifiRemoteStation *station)
{
  if (!station->m_sampleWait && !station->m_sampleTries && station->m_sampleCount > 0)
    {
      station->m_sampleWait = 16 + 2 * station->m_avgAmpduLen;
      station->m_sampleTries = 1;
      station->m_sampleCount--;
    }
}
void
MinstrelHtWifiManager::DoDisposeStation (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *) st;
  station->DisposeStation ();
}

WifiTxVector
MinstrelHtWifiManager::DoGetDataTxVector (WifiRemoteStation *st,
                                          uint32_t size)
{
  NS_LOG_FUNCTION (this << st << size);
  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *) st;

  if (!station->m_initialized)
    {
      CheckInit (station);
    }

  if (!station->m_isHt)
    {
      WifiTxVector vector = m_legacyManager->GetDataTxVector(station);

      uint64_t dataRate = vector.GetMode().GetDataRate(vector.GetChannelWidth(), vector.IsShortGuardInterval(), vector.GetNss());
      m_rateChange (dataRate, station->m_state->m_address);

      return vector;
    }
  else
    {
      NS_LOG_DEBUG ("DoGetDataMode m_txrate= " << station->m_txrate);

      uint32_t rateId = GetRateId (station->m_txrate);
      uint32_t groupId = GetGroupId (station->m_txrate);

      NS_LOG_DEBUG ("DoGetDataMode rateId= " << rateId << " groupId= " << groupId << " mode= " << GetMcsSupported (station, rateId));

      McsGroup group = m_minstrelGroups[groupId];

      // Check consistency of rate selected.
      if ((group.sgi && !GetShortGuardInterval (station)) || group.chWidth > GetChannelWidth (station)  ||  (uint32_t) group.streams > GetNumberOfReceiveAntennas (station))
        {
          NS_ASSERT_MSG (false,"Inconsistent group selected. Group: (" << (uint32_t)group.streams << "," << (uint32_t)group.sgi << "," << group.chWidth << ")" <<
                         " Station capabilities: (" << GetNumberOfReceiveAntennas (station) << "," << GetShortGuardInterval (station) << "," << GetChannelWidth (station) << ")");
        }


      uint64_t dataRate = GetMcsSupported (station, rateId).GetDataRate(group.chWidth, group.sgi, group.streams);
      m_rateChange (dataRate, station->m_state->m_address);

      return WifiTxVector (GetMcsSupported (station, rateId), GetDefaultTxPowerLevel (), GetLongRetryCount (station), group.sgi, group.streams, GetNess (station), group.chWidth, GetAggregation (station), GetStbc (station));
    }
}

WifiTxVector
MinstrelHtWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *) st;

  if (!station->m_initialized)
    {
      CheckInit (station);
    }

  if (!station->m_isHt)
    {
      return m_legacyManager->GetRtsTxVector(station);
    }
  else
    {
      NS_LOG_DEBUG ("DoGetRtsMode m_txrate=" << station->m_txrate);

      /* RTS is sent in a non-HT frame. RTS with HT is not supported yet in NS3.
       * When supported, decision of using HT has to follow rules in Section 9.7.6 from 802.11-2012.
       * From Sec. 9.7.6.5: "A frame other than a BlockAckReq or BlockAck that is carried in a
       * non-HT PPDU shall be transmitted by the STA using a rate no higher than the highest
       * rate in  the BSSBasicRateSet parameter that is less than or equal to the rate or
       * non-HT reference rate (see 9.7.9) of the previously transmitted frame that was
       * directed to the same receiving STA. If no rate in the BSSBasicRateSet parameter meets
       * these conditions, the control frame shall be transmitted at a rate no higher than the
       * highest mandatory rate of the attached PHY that is less than or equal to the rate
       * or non-HT reference rate (see 9.7.9) of the previously transmitted frame that was
       * directed to the same receiving STA."
       */

      // As we are in Minstrel HT, assume the last rate was an HT rate.
      // FIXME: I think this should be done in parent class??
      WifiMode lastRate = GetMcsSupported(station, GetRateId(station->m_txrate));
      uint8_t streams = m_minstrelGroups[GetGroupId(station->m_txrate)].streams;
      WifiMode referenceRate = lastRate.GetNonHtReferenceRate(streams);
      uint64_t lastDataRate = referenceRate.GetDataRate(20,false,1);
      uint32_t nBasicRates = GetNBasicModes();

      WifiMode rtsRate;
      bool rateFound = false;

      for (uint32_t i = 0; i < nBasicRates; i++)
        {
          uint64_t rate = GetBasicMode(i).GetDataRate(20,false,1);
          if (rate <= lastDataRate)
            {
              rtsRate = GetBasicMode(i);
              rateFound = true;
            }
        }

      if (!rateFound)
        {
          Ptr<WifiPhy> phy = GetPhy();
          uint32_t nSupportRates = phy->GetNModes();
          for (uint32_t i = 0; i < nSupportRates; i++)
            {
              uint64_t rate = phy->GetMode(i).GetDataRate(20,false,1);
              if (rate <= lastDataRate)
                {
                  rtsRate = phy->GetMode(i);
                  rateFound = true;
                }
            }
        }

      NS_ASSERT(rateFound);

      uint32_t channelWidth = GetChannelWidth (station);
        if (channelWidth > 20 && channelWidth != 22)
          {
            //avoid to use legacy rate adaptation algorithms for IEEE 802.11n/ac
            channelWidth = 20;
          }
        return WifiTxVector (GetSupported (station, 0), GetDefaultTxPowerLevel (), GetShortRetryCount (station), false, 1, 0, channelWidth, GetAggregation (station), false);
    }
}

bool
MinstrelHtWifiManager::DoNeedDataRetransmission (WifiRemoteStation *st, Ptr<const Packet> packet, bool normally)
{
  NS_LOG_FUNCTION (this << st << packet << normally);

  MinstrelHtWifiRemoteStation *station = (MinstrelHtWifiRemoteStation *)st;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return normally;
    }

  uint32_t maxRetries;

  if (!station->m_isHt)
    {
      maxRetries = m_legacyManager->CountRetries(station);
    }
  else
    {
      maxRetries = CountRetries(station);
    }

  if (station->m_longRetry >= maxRetries)
    {
      NS_LOG_DEBUG ("No re-transmission allowed. Retries: " <<  station->m_longRetry << " Max retries: " << maxRetries);
      return false;
    }
  else
    {
      NS_LOG_DEBUG ("Re-transmit. Retries: " <<  station->m_longRetry << " Max retries: " << maxRetries);
      return true;
    }
}

uint32_t
MinstrelHtWifiManager::CountRetries (MinstrelHtWifiRemoteStation *station)
{
  uint32_t maxProbRateId = GetRateId (station->m_maxProbRate);
  uint32_t maxProbGroupId = GetGroupId (station->m_maxProbRate);
  uint32_t maxTpRateId = GetRateId (station->m_maxTpRate);
  uint32_t maxTpGroupId = GetGroupId (station->m_maxTpRate);
  uint32_t maxTp2RateId = GetRateId (station->m_maxTpRate2);
  uint32_t maxTp2GroupId = GetGroupId (station->m_maxTpRate2);

  if (!station->m_isSampling)
    {
      return station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxTpRateId].adjustedRetryCount +
             station->m_mcsTable[maxTp2GroupId].m_minstrelTable[maxTp2RateId].adjustedRetryCount +
             station->m_mcsTable[maxProbGroupId].m_minstrelTable[maxProbRateId].adjustedRetryCount;
    }
  else
    {
      return 1 + station->m_mcsTable[maxTpGroupId].m_minstrelTable[maxTp2RateId].adjustedRetryCount +
             station->m_mcsTable[maxProbGroupId].m_minstrelTable[maxProbRateId].adjustedRetryCount;
    }
}

bool
MinstrelHtWifiManager::IsLowLatency (void) const
{
  NS_LOG_FUNCTION (this);
  return true;
}

uint32_t
MinstrelHtWifiManager::GetNextSample (MinstrelHtWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);

  uint32_t sampleGroup = station->m_sampleGroup;

  uint32_t index = station->m_mcsTable[sampleGroup].m_index;
  uint32_t col = station->m_mcsTable[sampleGroup].m_col;

  uint32_t sampleIndex = station->m_sampleTable[index][col];

  uint32_t rateIndex = GetIndex (sampleGroup, sampleIndex);
  NS_LOG_DEBUG ("Next Sample is " << rateIndex );

  SetNextSample (station); //Calculate the next sample rate.

  return rateIndex;
}

void
MinstrelHtWifiManager::SetNextSample (MinstrelHtWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  do
    {
      station->m_sampleGroup++;
      station->m_sampleGroup %= m_numGroups;
    }
  while (!station->m_mcsTable[station->m_sampleGroup].m_supported);

  station->m_mcsTable[station->m_sampleGroup].m_index++;

  uint32_t sampleGroup = station->m_sampleGroup;
  uint8_t index = station->m_mcsTable[station->m_sampleGroup].m_index;
  uint8_t col = station->m_mcsTable[sampleGroup].m_col;

  if (index >= station->m_nModes ||
      !station->m_mcsTable[sampleGroup].m_minstrelTable[station->m_sampleTable[index][col]].supported)
    {
      station->m_mcsTable[station->m_sampleGroup].m_index = 0;
      station->m_mcsTable[station->m_sampleGroup].m_col++;
      if (station->m_mcsTable[station->m_sampleGroup].m_col >= m_nSampleCol)
        {
          station->m_mcsTable[station->m_sampleGroup].m_col = 0;
        }
      index = station->m_mcsTable[station->m_sampleGroup].m_index;
      col = station->m_mcsTable[sampleGroup].m_col;
    }
  NS_LOG_DEBUG("New sample set: group= " << sampleGroup << " index= " << station->m_sampleTable[index][col]);
}

uint32_t
MinstrelHtWifiManager::FindRate (MinstrelHtWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  NS_LOG_DEBUG ("FindRate " << "packet=" << station->m_totalPacketsCount );

  if ((station->m_samplePacketsCount + station->m_totalPacketsCount) == 0)
    {
      return station->m_maxTpRate;
    }

  if (station->m_sampleWait == 0 && station->m_sampleTries != 0)
    {
      //SAMPLING
      NS_LOG_DEBUG ("Obtaining a sampling rate");
      /// Now go through the table and find an index rate.
      uint32_t sampleIdx = GetNextSample (station);
      NS_LOG_DEBUG ("Sampling rate = " << sampleIdx);


      //Evaluate if the sampling rate selected should be used.
      uint32_t sampleGroupId = GetGroupId (sampleIdx);
      uint32_t sampleRateId = GetRateId (sampleIdx);
      McsGroup sampleGroup = m_minstrelGroups[sampleGroupId];
      HtRateInfo sampleRateInfo = station->m_mcsTable[sampleGroupId].m_minstrelTable[sampleRateId];

      /**
       * Sampling might add some overhead (RTS, no aggregation)
       * to the frame. Hence, don't use sampling for the currently
       * used rates.
       *
       * Also do not sample if the probability is already higher than 95%
       * to avoid wasting airtime.
       */
      NS_LOG_DEBUG ("Use sample rate? MaxTpRate= " << station->m_maxTpRate << " CurrentRate= " << station->m_txrate <<
                    " SampleRate= " << sampleIdx << " SampleProb= " << sampleRateInfo.ewmaProb);
      if (sampleIdx != station->m_maxTpRate && sampleIdx != station->m_maxTpRate2 &&
          sampleIdx != station->m_maxProbRate && sampleRateInfo.ewmaProb <= 95*180)
        {

          /*
           * Make sure that lower rates get sampled only occasionally,
           * if the link is working perfectly.
           */

          uint32_t maxTpGroupId = GetGroupId (station->m_maxTpRate);
          uint32_t maxTp2GroupId = GetGroupId (station->m_maxTpRate2);
          uint32_t maxTp2RateId = GetRateId (station->m_maxTpRate2);
          uint32_t maxProbGroupId = GetGroupId (station->m_maxProbRate);
          uint32_t maxProbRateId = GetRateId (station->m_maxProbRate);

          uint8_t maxTpStreams = m_minstrelGroups[maxTpGroupId].streams;
          uint8_t sampleStreams = m_minstrelGroups[sampleGroupId].streams;

          Time sampleDuration = sampleRateInfo.perfectTxTime;
          Time maxTp2Duration = station->m_mcsTable[maxTp2GroupId].m_minstrelTable[maxTp2RateId].perfectTxTime;
          Time maxProbDuration = station->m_mcsTable[maxProbGroupId].m_minstrelTable[maxProbRateId].perfectTxTime;

          NS_LOG_DEBUG ("Use sample rate? SampleDuration= " << sampleDuration << " maxTp2Duration= " << maxTp2Duration <<
                        " maxProbDuration= " << maxProbDuration << " sampleStreams= " << (uint32_t)sampleStreams <<
                        " maxTpStreams= " << (uint32_t)maxTpStreams);
          if (sampleDuration < maxTp2Duration || (sampleStreams <= maxTpStreams - 1 && sampleDuration < maxProbDuration))
            {
              /// Start sample count.
              station->m_samplePacketsCount++;

              /// Set flag that we are currently sampling.
              station->m_isSampling = true;

              /// Bookkeeping for resetting stuff.
              if (station->m_totalPacketsCount >= 10000)
                {
                  station->m_samplePacketsCount = 0;
                  station->m_totalPacketsCount = 0;
                }

              /// set the rate that we're currently sampling
              station->m_sampleRate = sampleIdx;

              NS_LOG_DEBUG ("FindRate " << "sampleRate=" << sampleIdx);
              station->m_sampleTries--;
              return sampleIdx;
            }
          else
            {
              station->m_numSamplesSlow++;
              if (sampleRateInfo.numSamplesSkipped >= 20 && station->m_numSamplesSlow <= 2)
                {
                  /// Start sample count.
                  station->m_samplePacketsCount++;

                  /// Set flag that we are currently sampling.
                  station->m_isSampling = true;

                  /// Bookkeeping for resetting stuff.
                  if (station->m_totalPacketsCount >= 10000)
                    {
                      station->m_samplePacketsCount = 0;
                      station->m_totalPacketsCount = 0;
                    }

                  /// set the rate that we're currently sampling
                  station->m_sampleRate = sampleIdx;

                  NS_LOG_DEBUG ("FindRate " << "sampleRate=" << sampleIdx);
                  station->m_sampleTries--;
                  return sampleIdx;
                }
            }
        }
    }
  if (station->m_sampleWait > 0)
    {
      station->m_sampleWait--;
    }

  ///	Continue using the best rate.

  NS_LOG_DEBUG ("FindRate " << "maxTpRrate=" << station->m_maxTpRate);
  return station->m_maxTpRate;
}
uint32_t
MinstrelHtWifiManager::GetIndex (uint32_t groupid, uint32_t mcsIndex)
{
  NS_LOG_FUNCTION (this << groupid << mcsIndex);
  uint32_t index;
  index = groupid * m_numRates + mcsIndex;
  return index;
}
void
MinstrelHtWifiManager::UpdateStats (MinstrelHtWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  if (Simulator::Now () <  station->m_nextStatsUpdate)
    {
      return;
    }

  NS_LOG_DEBUG ("Updating stats=" << this);

  station->m_nextStatsUpdate = Simulator::Now () + m_updateStats;

  station->m_numSamplesSlow = 0;
  station->m_sampleCount = 0;

  Time txTime;
  uint32_t tempProb;

  if (station->m_ampduPacketCount > 0)
    {
      double newLen = station->m_ampduLen / station->m_ampduPacketCount;
      station->m_avgAmpduLen= ( newLen * (100 - m_ewmaLevel) + (station->m_avgAmpduLen * m_ewmaLevel) ) / 100;
      station->m_ampduLen = 0;
      station->m_ampduPacketCount = 0;
    }


  /// Update throughput and EWMA for each rate inside each group.
  for (uint32_t j = 0; j < m_numGroups; j++)
    {
      if (station->m_mcsTable[j].m_supported)
        {
          station->m_sampleCount++;
          for (uint32_t i = 0; i < station->m_nModes; i++)
            {
              if (station->m_mcsTable[j].m_minstrelTable[i].supported)
                {
                  station->m_mcsTable[j].m_minstrelTable[i].retryUpdated = false;

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
                      station->m_mcsTable[j].m_minstrelTable[i].numSamplesSkipped = 0;
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

                      /**
                       * Calculating throughput.
                       * Do not account throughput if sucess prob is below 10% (as done in minstrel_hc linux implementation).
                       */
                      if (tempProb < 10*180)
                        station->m_mcsTable[j].m_minstrelTable[i].throughput = 0;
                      else
                        {
                          /**
                           * For the throughput calculation, limit the probability value to 90% to
                           * account for collision related packet error rate fluctuation.
                           */
                          if (tempProb > 90*180)
                            station->m_mcsTable[j].m_minstrelTable[i].throughput = 90*180 * (1000000 / txTime.GetMicroSeconds ());
                          else
                            station->m_mcsTable[j].m_minstrelTable[i].throughput = tempProb * (1000000 / txTime.GetMicroSeconds ());
                        }
                    }
                  else
                    {
                      station->m_mcsTable[j].m_minstrelTable[i].numSamplesSkipped++;
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
        }
    }

  uint32_t max_prob = 0, index_max_prob = 0, max_tp = 0, index_max_tp = 0, index_max_tp2 = 0;
  uint32_t index = 0;

  /// For each group get the max_tp and max_tp2.
  for (uint32_t j = 0; j < m_numGroups; j++)
    {
      max_prob = 0;
      index_max_prob = GetIndex (j,0);
      max_tp = 0;
      index_max_tp = GetIndex (j,0);
      index_max_tp2 = GetIndex (j,0);

      if (station->m_mcsTable[j].m_supported)
        {
          /// Go find maximum throughput, second maximum throughput and high probability of success rates.
          for (uint32_t i = 0; i < station->m_nModes; i++)
            {
              if (station->m_mcsTable[j].m_minstrelTable[i].supported)
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
            }
          max_tp = 0;
          /// Find the second maximum throughput rate.
          for (uint32_t i = 0; i < station->m_nModes; i++)
            {
              if (station->m_mcsTable[j].m_minstrelTable[i].supported)
                {
                  index = GetIndex (j,i);

                  if ((i != index_max_tp) && (max_tp < station->m_mcsTable[j].m_minstrelTable[i].throughput))
                    {
                      index_max_tp2 = index;
                      max_tp = station->m_mcsTable[j].m_minstrelTable[i].throughput;
                    }
                }
            }

          station->m_mcsTable[j].m_maxTpRate = index_max_tp;
          station->m_mcsTable[j].m_maxTpRate2 = index_max_tp2;
          station->m_mcsTable[j].m_maxProbRate = index_max_prob;
          NS_LOG_DEBUG ("Group: " << j << " max tp=" << index_max_tp << "\nmax tp2=" << index_max_tp2 << "\nmax prob=" << index_max_prob);
        }
    }

  /// Get the max_tp and max_tp2 from all groups.
  max_prob = 0;
  max_tp = 0;
  //Find the lowest supported group.
  uint32_t k = 0;
  while (!station->m_mcsTable[k].m_supported)
    {
      k++;
    }
  index_max_prob = GetIndex (k,0);
  index_max_tp = GetIndex (k,0);
  index_max_tp2 = GetIndex (k,0);


  for (uint32_t j = 0; j < m_numGroups; j++)
    {
      if (station->m_mcsTable[j].m_supported)
        {
          /// Go find maximum throughput, second maximum throughput and high probability of success rates.
          if (max_tp < station->m_mcsTable[j].m_minstrelTable[GetRateId (station->m_mcsTable[j].m_maxTpRate)].throughput)
            {
              index_max_tp = station->m_mcsTable[j].m_maxTpRate;
              max_tp = station->m_mcsTable[j].m_minstrelTable[GetRateId (station->m_mcsTable[j].m_maxTpRate)].throughput;
            }

          if (max_prob < station->m_mcsTable[j].m_minstrelTable[GetRateId (station->m_mcsTable[j].m_maxProbRate)].ewmaProb)
            {
              index_max_prob = station->m_mcsTable[j].m_maxProbRate;
              max_prob = station->m_mcsTable[j].m_minstrelTable[GetRateId (station->m_mcsTable[j].m_maxProbRate)].ewmaProb;
            }
        }
    }
  max_tp = 0;
  /// Find the second highest maximum throughput rate.
  for (uint32_t i = 0; i < m_numGroups; i++)
    {
      if (station->m_mcsTable[i].m_supported)
        {
          if ((station->m_mcsTable[i].m_maxTpRate != index_max_tp) && (max_tp < station->m_mcsTable[i].m_minstrelTable[GetRateId (station->m_mcsTable[i].m_maxTpRate)].throughput))
            {
              /// Find if another group's max_tp is better than the max_tp2.
              index_max_tp2 = station->m_mcsTable[i].m_maxTpRate;
              max_tp = station->m_mcsTable[i].m_minstrelTable[GetRateId (station->m_mcsTable[i].m_maxTpRate)].throughput;
            }
          if (max_tp < station->m_mcsTable[i].m_minstrelTable[GetRateId (station->m_mcsTable[i].m_maxTpRate2)].throughput)
            {
              /// Find if another group's max_tp2 is better than max_tp2.
              index_max_tp2 = station->m_mcsTable[i].m_maxTpRate2;
              max_tp = station->m_mcsTable[i].m_minstrelTable[GetRateId (station->m_mcsTable[i].m_maxTpRate2)].throughput;
            }
        }
    }

  /* try to sample all available rates during each interval */
  station->m_sampleCount *= 8;

  station->m_maxTpRate = index_max_tp;
  station->m_maxTpRate2 = index_max_tp2;
  station->m_maxProbRate = index_max_prob;

  //Recalculate retries for the rates selected.
  CalculateRetransmits(station, station->m_maxTpRate);
  CalculateRetransmits(station, station->m_maxTpRate2);
  CalculateRetransmits(station, station->m_maxProbRate);

  /// If the max_tp rate is bigger than the current rate and uses the same number of streams.
  if ((index_max_tp > station->m_txrate) && (m_minstrelGroups[GetGroupId (index_max_tp)].streams >= m_minstrelGroups[GetGroupId (station->m_txrate)].streams) )
    {
      station->m_txrate = index_max_tp;
    }

  NS_LOG_DEBUG ("max tp=" << index_max_tp << "\nmax tp2=" << index_max_tp2 << "\nmax prob=" << index_max_prob);
}

void
MinstrelHtWifiManager::RateInit (MinstrelHtWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
  NS_LOG_DEBUG ("RateInit=" << station);

  station->m_mcsTable = McsGroupData (m_numGroups);

  NS_LOG_DEBUG ("Supported groups by station:");
  for (uint32_t j = 0; j < m_numGroups; j++)
    {
      /// Check if the group is supported
      station->m_mcsTable[j].m_supported = false;
      if (m_minstrelGroups[j].isVht || (!m_minstrelGroups[j].isVht && !m_useVhtOnly))                             ///If it is an HT MCS, check if VHT only is disabled
        {
          if (!(!GetPhy()->GetGuardInterval() && m_minstrelGroups[j].sgi)          ///Is SGI supported by the transmitter?
              && (GetPhy()->GetChannelWidth() >= m_minstrelGroups[j].chWidth)         ///Is channel width supported by the transmitter?
              && (GetPhy()->GetNumberOfTransmitAntennas () >= m_minstrelGroups[j].streams)) ///Are streams supported by the transmitter? FIXME Is this the correct way to check the number of streams?
            {
              if (!(!GetVhtSupported(station) && m_minstrelGroups[j].isVht) &&              ///Is VHT supported by the receiver?
                  !(!GetShortGuardInterval (station) && m_minstrelGroups[j].sgi)          ///Is SGI supported by the receiver?
                  && (GetChannelWidth (station) >= m_minstrelGroups[j].chWidth)         ///Is channel width supported by the receiver?
                  && (GetNumberOfReceiveAntennas (station) >= m_minstrelGroups[j].streams)) ///Are streams supported by the receiver? FIXME Is this the correct way to check the number of streams?
                {
                  NS_LOG_DEBUG ("Group " << j << ": (" << (uint32_t)m_minstrelGroups[j].streams << "," << (uint32_t)m_minstrelGroups[j].sgi << "," << m_minstrelGroups[j].chWidth << ")");
                  station->m_mcsTable[j].m_supported = true;

                  station->m_mcsTable[j].m_minstrelTable = HtMinstrelRate (station->m_nModes);
                  station->m_mcsTable[j].m_col = 0;
                  station->m_mcsTable[j].m_index = 0;
                  for (uint32_t i = 0; i < station->m_nModes; i++)
                    {
                      NS_LOG_DEBUG ("Mode " << i << ": " << GetMcsSupported(station,i));
                      ///Check for invalid VHT MCS
                      ///TODO Other combinations are also invalid
                      if  (!(m_minstrelGroups[j].chWidth == 20) || (m_minstrelGroups[j].chWidth == 20 && (GetMcsSupported(station,i).GetMcsValue() != 9)))
                        {
                          station->m_mcsTable[j].m_minstrelTable[i].supported = true;
                          station->m_mcsTable[j].m_minstrelTable[i].numRateAttempt = 0;
                          station->m_mcsTable[j].m_minstrelTable[i].numRateSuccess = 0;
                          station->m_mcsTable[j].m_minstrelTable[i].prob = 0;
                          station->m_mcsTable[j].m_minstrelTable[i].ewmaProb = 0;
                          station->m_mcsTable[j].m_minstrelTable[i].prevNumRateAttempt = 0;
                          station->m_mcsTable[j].m_minstrelTable[i].prevNumRateSuccess = 0;
                          station->m_mcsTable[j].m_minstrelTable[i].numSamplesSkipped = 0;
                          station->m_mcsTable[j].m_minstrelTable[i].successHist = 0;
                          station->m_mcsTable[j].m_minstrelTable[i].attemptHist = 0;
                          station->m_mcsTable[j].m_minstrelTable[i].throughput = 0;
                          station->m_mcsTable[j].m_minstrelTable[i].perfectTxTime = GetFirstMpduTxTime (j, GetMcsSupported(station,i));
                          station->m_mcsTable[j].m_minstrelTable[i].retryCount = 1;
                          station->m_mcsTable[j].m_minstrelTable[i].adjustedRetryCount = 1;
                          CalculateRetransmits(station, j, i);
                        }
                      else
                        {
                          station->m_mcsTable[j].m_minstrelTable[i].supported = false;
                        }
                    }
                }
            }
        }
    }
  SetNextSample (station);                 /// Select the initial sample index.
  UpdateStats (station);                   /// Calculate the initial high throughput rates.
  station->m_txrate = FindRate (station); /// Select the rate to use.
}

void
MinstrelHtWifiManager::CalculateRetransmits (MinstrelHtWifiRemoteStation *station, uint32_t index)
{
  NS_LOG_FUNCTION (this << station << index);
  uint32_t groupId = GetGroupId(index);
  uint32_t rateId = GetRateId(index);
  if (!station->m_mcsTable[groupId].m_minstrelTable[rateId].retryUpdated)
    {
      CalculateRetransmits (station, groupId, rateId);
    }
}

void
MinstrelHtWifiManager::CalculateRetransmits (MinstrelHtWifiRemoteStation *station, uint32_t groupId, uint32_t rateId)
{
  NS_LOG_FUNCTION (this << station << groupId << groupId);

  Time totalTxTimeWithGivenRetries = Seconds (0.0); //tx_time in minstrel.c
  NS_LOG_DEBUG (" Calculating the number of retries");


  Time dataTransmissionTime = GetFirstMpduTxTime(groupId, GetMcsSupported(station,rateId)) +
      GetMpduTxTime(groupId, GetMcsSupported(station, rateId)) * station->m_avgAmpduLen;
  for (uint32_t retries = 2; retries < 11; retries++)
    {
      NS_LOG_DEBUG ("  Checking " << retries << " retries");

      totalTxTimeWithGivenRetries = CalculateTimeUnicastPacket (dataTransmissionTime, 0, retries);
      NS_LOG_DEBUG ("   totalTxTimeWithGivenRetries = " << totalTxTimeWithGivenRetries);
      if (totalTxTimeWithGivenRetries > MilliSeconds (6))
        {
          break;
        }
      station->m_mcsTable[groupId].m_minstrelTable[rateId].retryCount = retries;
      station->m_mcsTable[groupId].m_minstrelTable[rateId].adjustedRetryCount = retries;
      station->m_mcsTable[groupId].m_minstrelTable[rateId].retryUpdated = true;
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
MinstrelHtWifiManager::PrintSampleTable (MinstrelHtWifiRemoteStation *station, std::ostream &os)
{
  NS_LOG_FUNCTION (this << station);
  NS_LOG_DEBUG ("PrintSampleTable=" << station);

  uint32_t numSampleRates = station->m_nModes;
  for (uint32_t i = 0; i < numSampleRates; i++)
    {
      for (uint32_t j = 0; j < m_nSampleCol; j++)
        {
          os << station->m_sampleTable[i][j] << "\t";
        }
      os << std::endl;
    }
}

void
MinstrelHtWifiManager::PrintTable (MinstrelHtWifiRemoteStation *station, std::ostream &os)
{
  NS_LOG_FUNCTION (this << station);
  NS_LOG_DEBUG ("PrintTable=" << station);
  uint32_t numRates = station->m_nModes;
  for (uint32_t j = 0; j < m_numGroups; j++)
    {
      for (uint32_t i = 0; i < numRates; i++)
        {
          if (station->m_mcsTable[j].m_supported && station->m_mcsTable[j].m_minstrelTable[i].supported)
            os << "index(" << i << ") = " << station->m_mcsTable[j].m_minstrelTable[i].perfectTxTime << "\n";
        }
    }
}

uint32_t
MinstrelHtWifiManager::GetRateId (uint32_t index)
{
  NS_LOG_FUNCTION (this << index);

  uint32_t id;
  id = index % m_numRates;
  return id;
}

uint32_t
MinstrelHtWifiManager::GetGroupId (uint32_t index)
{
  NS_LOG_FUNCTION (this << index);

  return index / m_numRates;
}

uint32_t
MinstrelHtWifiManager::GetGroupId (uint8_t txstreams, uint8_t sgi, uint32_t chWidth)
{
  NS_LOG_FUNCTION (this << (int)txstreams << (int)sgi << chWidth);

  return MAX_SUPPORTED_STREAMS * 2 * (chWidth==160 ? 3 : chWidth==80 ? 2 : chWidth==40 ? 1 : 0) + MAX_SUPPORTED_STREAMS * sgi + txstreams - 1;
}

uint32_t
MinstrelHtWifiManager::GetVhtGroupId (uint8_t txstreams, uint8_t sgi, uint32_t chWidth)
{
  NS_LOG_FUNCTION (this << (int)txstreams << (int)sgi << chWidth);

  return MAX_SUPPORTED_STREAMS * 2 * (chWidth==160 ? 3 : chWidth==80 ? 2 : chWidth==40 ? 1 : 0) + MAX_SUPPORTED_STREAMS * sgi + txstreams - 1;
}

} // namespace ns3




