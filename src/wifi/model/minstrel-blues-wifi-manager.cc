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
 * Author: Matias Richart <mrichart@fing.edu.uy>
 *
 */

#include "minstrel-blues-wifi-manager.h"
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

NS_LOG_COMPONENT_DEFINE ("MinstrelBluesWifiManager");


namespace ns3 {


struct MinstrelBluesWifiRemoteStation : public WifiRemoteStation
{
  Time m_nextStatsUpdate;  //!< Time when stats will be updated.

  /**
   * To keep track of the current position in our random sample table
   * going row by row from 1st column until the 10th column(Minstrel defines 10)
   * then we wrap back to the row 1 col 1.
   * note: there are many other ways to do this.
   */
  uint32_t m_col, m_index;

  uint32_t* m_sortedThRates; //!< List of rates sorted by throughput.
  uint32_t m_maxThRate;      //!< The rate with highest throughput.
  uint32_t m_maxThRate2;      //!< The rate with second highest throughput.
  uint32_t m_maxProbRate;  //!< The rate with highest probability of success.

  uint32_t m_currentRate;  //!< Current transmission rate.

  uint32_t m_frameCount;  //!< Number of frames transmitted.
  uint32_t m_minstrelSampleCount;  //!< Number of sample frames transmitted.

  bool m_isSampling;  //!< A flag to indicate we are currently sampling.
  uint32_t m_sampleRate;  //!< Current sample rate.
  bool  m_sampleRateSlower;  //!< A flag to indicate sample rate is slower than highest throughput rate.

  uint32_t m_shortRetryCount;  //!< Number of short retries (such as control frames).
  uint32_t m_longRetryCount;  //!< Number of long retries (such as data packets).
  uint32_t m_retryCount;  //!< Number of retries (short + long).
  uint32_t m_retryErrorCount;  //!< Number of retry errors (all retransmission attempts failed).

  bool m_initialized;  //!< For initializing variables.

  MinstrelBluesRate m_minstrelBluesTable;  //!< Minstrel Blues table.
  SampleRate m_sampleTable;  //!< Sample rates table.

  uint32_t m_bluesSampleCount;  //!< Number of sample frames transmitted.
  uint32_t m_bluesRefCount;  //!< Number of reference frames transmitted.

  uint8_t m_currentPower;  //!< Current transmission power level.
  uint8_t m_refPower;  //!< Current transmission power level for reference frames.
  uint8_t m_samplePower;  //!< Current transmission power level for sample frames.
  uint8_t m_dataPower;  //!< Current transmission power level for data frames.

  uint32_t m_nSupported; //!< Number of supported rates by the remote station.

  bool m_samplingBlues; //!< If Blues is sampling.
  bool m_bluesSampleRateSlower;  //!< A flag to indicate blues sample rate is slower than highest throughput rate.
  uint32_t m_bluesSampleRate; //!< Blues sample rate.
  uint32_t m_bluesRoundRobinIndex; //!< Index on the sorted list of the last sample rate elected.

  /**
   * Best four rates from blues utility function.
   */
  double m_maxURate;
  double m_maxURate2;
};

NS_OBJECT_ENSURE_REGISTERED (MinstrelBluesWifiManager);

TypeId
MinstrelBluesWifiManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MinstrelBluesWifiManager")
    .SetParent<WifiRemoteStationManager> ()
    .AddConstructor<MinstrelBluesWifiManager> ()
    .AddAttribute ("UpdateStatistics",
                   "The interval between updating statistics table.",
                   TimeValue (Seconds (0.1)),
                   MakeTimeAccessor (&MinstrelBluesWifiManager::m_updateStats),
                   MakeTimeChecker ())
    .AddAttribute ("LookAroundRate",
                   "The percentage to try other rates than our current rate.",
                   DoubleValue (10),
                   MakeDoubleAccessor (&MinstrelBluesWifiManager::m_minstrelSamplingRatio),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("EWMA",
                   "The EWMA coefficient",
                   DoubleValue (75),
                   MakeDoubleAccessor (&MinstrelBluesWifiManager::m_ewmaCoefficient),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("SampleColumn",
                   "The number of columns used for sampling",
                   DoubleValue (10),
                   MakeDoubleAccessor (&MinstrelBluesWifiManager::m_nSampleColumns),
                   MakeDoubleChecker <uint32_t> ())
    .AddAttribute ("FrameLength",
                   "The frame length used for calculating mode TxTime",
                   DoubleValue (1420),
                   MakeDoubleAccessor (&MinstrelBluesWifiManager::m_frameLength),
                   MakeDoubleChecker <uint32_t> ())
    .AddAttribute ("LookAroundSamplePower",
                   "The percentage to try other sample powers than our current sample power.",
                   DoubleValue (10),
                   MakeDoubleAccessor (&MinstrelBluesWifiManager::m_bluesSamplingRatio),
                   MakeDoubleChecker <double> ())
    .AddAttribute ("BluesUpdateStats",
                   "How many reference and sample frames are needed for updating statistics.",
                   UintegerValue (10),
                   MakeUintegerAccessor (&MinstrelBluesWifiManager::m_bluesUpdateStatsThreshold),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("IncrementLevel",
                   "How many levels to increase power each time.",
                   UintegerValue (2),
                   MakeUintegerAccessor (&MinstrelBluesWifiManager::m_deltaIncPower),
                   MakeUintegerChecker <uint8_t> ())
    .AddAttribute ("DecrementLevel",
                   "How many levels to decrease power each time.",
                   UintegerValue (1),
                   MakeUintegerAccessor (&MinstrelBluesWifiManager::m_deltaDecPower),
                   MakeUintegerChecker <uint8_t> ())
    .AddAttribute ("Delta",
                   "Power levels of separation between data and sample power.",
                   UintegerValue (2),
                   MakeUintegerAccessor (&MinstrelBluesWifiManager::m_deltaDataSamplePower),
                   MakeUintegerChecker <uint8_t> ())
   .AddAttribute ("BluesPowerStep",
                  "Minimum separation between sample and reference power.",
                  UintegerValue (2),
                  MakeUintegerAccessor (&MinstrelBluesWifiManager::m_bluesPowerStep),
                  MakeUintegerChecker <uint8_t> ())
    .AddAttribute ("IncrementThreshold",
                   "The difference between probabilities needed to increase power.",
                   DoubleValue (0.1),
                   MakeDoubleAccessor (&MinstrelBluesWifiManager::m_thIncPower),
                   MakeDoubleChecker <double> ())
    .AddAttribute ("DecrementThreshold",
                   "The difference between probabilities needed to decrease power.",
                    DoubleValue (0.2),
                    MakeDoubleAccessor (&MinstrelBluesWifiManager::m_thDecPower),
                    MakeDoubleChecker <double> ())
    .AddAttribute ("EmergencyThreshold",
		   "The probability of data power needed for determining a throughput collapse.",
		    DoubleValue (0.1),
		    MakeDoubleAccessor (&MinstrelBluesWifiManager::m_thEmergency),
		    MakeDoubleChecker <double> ())
    .AddAttribute ("BluesUtilityWeight",
		   "The weight to use in the utility function.",
		    DoubleValue (10),
		    MakeDoubleAccessor (&MinstrelBluesWifiManager::m_bluesUilityWeight),
		    MakeDoubleChecker <double> ())
    .AddTraceSource ("PowerChange",
                    "The transmission power has change",
                    MakeTraceSourceAccessor (&MinstrelBluesWifiManager::m_powerChange),
                    "ns3::MinstrelBluesWifiManager::PowerChangeTracedCallback")
    .AddTraceSource ("RateChange",
		     "The transmission rate has change",
		     MakeTraceSourceAccessor (&MinstrelBluesWifiManager::m_rateChange),
		     "ns3::MinstrelBluesWifiManager::RateChangeTracedCallback")
  ;
  return tid;
}

MinstrelBluesWifiManager::MinstrelBluesWifiManager ()
{
  NS_LOG_FUNCTION (this);
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();
}

MinstrelBluesWifiManager::~MinstrelBluesWifiManager ()
{
  NS_LOG_FUNCTION (this);
}

void
MinstrelBluesWifiManager::SetupPhy (Ptr<WifiPhy> phy)
{
  NS_LOG_FUNCTION (this);
  m_minPower = phy->GetTxPowerStart();
  m_maxPower = phy->GetTxPowerEnd();
  uint32_t nModes = phy->GetNModes ();
  for (uint32_t i = 0; i < nModes; i++)
    {
      WifiMode mode = phy->GetMode (i);
      WifiTxVector txVector;
      txVector.SetMode(mode);
      AddCalcTxTime (mode, phy->CalculateTxDuration (m_frameLength, txVector, WIFI_PREAMBLE_LONG, phy->GetFrequency(), 0, 0));
    }
  WifiRemoteStationManager::SetupPhy (phy);
}

int64_t
MinstrelBluesWifiManager::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_uniformRandomVariable->SetStream (stream);
  return 1;
}

Time
MinstrelBluesWifiManager::GetCalcTxTime (WifiMode mode) const
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
MinstrelBluesWifiManager::AddCalcTxTime (WifiMode mode, Time t)
{
  NS_LOG_FUNCTION (this);
  m_calcTxTime.push_back (std::make_pair (t, mode));
}

WifiRemoteStation *
MinstrelBluesWifiManager::DoCreateStation (void) const
{
  NS_LOG_FUNCTION (this);
  MinstrelBluesWifiRemoteStation *station = new MinstrelBluesWifiRemoteStation ();

  station->m_nextStatsUpdate = Simulator::Now () + m_updateStats;
  station->m_col = 0;
  station->m_index = 0;
  station->m_sortedThRates = new uint32_t[N_MAX_TH_RATES];
  station->m_maxThRate = 0;
  station->m_maxThRate2 = 0;
  station->m_maxProbRate = 0;
  station->m_currentRate = 0;
  station->m_frameCount = 0;
  station->m_minstrelSampleCount = 0;
  station->m_isSampling = false;
  station->m_sampleRate = 0;
  station->m_sampleRateSlower = false;
  station->m_shortRetryCount = 0;
  station->m_longRetryCount = 0;
  station->m_retryCount = 0;
  station->m_retryErrorCount = 0;
  station->m_initialized = false;
  station->m_bluesSampleRateSlower = false;
  station->m_bluesRefCount = 0;
  station->m_bluesSampleCount = 100 / m_bluesSamplingRatio;
  station->m_samplingBlues = false;
  station->m_bluesSampleRate = 0;
  station->m_bluesRoundRobinIndex = 0;
  station->m_currentPower = m_maxPower;
  station->m_dataPower = m_maxPower;
  station->m_refPower = m_maxPower;
  station->m_samplePower = m_maxPower-m_bluesPowerStep;//TODO
  station->m_maxURate = 0;
  station->m_maxURate2 = 0;

  return station;
}

void
MinstrelBluesWifiManager::CheckInit (MinstrelBluesWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  if (!station->m_initialized)
    {
      // Note: we appear to be doing late initialization of the table
      // to make sure that the set of supported rates has been initialized
      // before we perform our own initialization.
      station->m_nSupported = GetNSupported (station);
      station->m_minstrelBluesTable = MinstrelBluesRate (station->m_nSupported);
      station->m_sampleTable = SampleRate (station->m_nSupported, std::vector<uint32_t> (m_nSampleColumns));
      RateInit (station);
      InitSampleTable (station);
      m_powerChange("init", station->m_currentPower, station->m_state->m_address);
      m_rateChange("init", station->m_currentRate, station->m_state->m_address);
      PrintTable (station);
      PrintSampleTable (station);
      station->m_initialized = true;
    }
}

void
MinstrelBluesWifiManager::DoReportRxOk (WifiRemoteStation *st, double rxSnr, WifiMode txMode)
{
  NS_LOG_FUNCTION (this);
}

void
MinstrelBluesWifiManager::DoReportRtsFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this);
  MinstrelBluesWifiRemoteStation *station = (MinstrelBluesWifiRemoteStation *)st;
  NS_LOG_DEBUG ("DoReportRtsFailed m_currentRate=" << station->m_currentRate);

  station->m_shortRetryCount++;
}

void
MinstrelBluesWifiManager::DoReportRtsOk (WifiRemoteStation *st, double ctsSnr,
                                         WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("station=" << st << " rts ok");
}

void
MinstrelBluesWifiManager::DoReportFinalRtsFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this);
  MinstrelBluesWifiRemoteStation *station = (MinstrelBluesWifiRemoteStation *)st;
  UpdateRetry (station);
  station->m_retryErrorCount++;
}

void
MinstrelBluesWifiManager::DoReportDataFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this);
  MinstrelBluesWifiRemoteStation *station = (MinstrelBluesWifiRemoteStation *)st;
  /**
   *
   * Retry Chain table is implemented here
   * In Minstrel Blues we have three cases:
   * 1- Minstrel sampling, Retry Chain is the same as Minstrel, with ref_power.
   * 2- Blues sampling, round robin rate for the first stage, with sample_power or ref_power. The rest of the chain as Minstrel, with data_power.
   * 3- No sampling, Retry Chain assembled with Blues Utility function and data_power.
   *
   *	          MINSTREL SAMPLING		|      		    |                     |
   * Try |         LOOKAROUND RATE              |  BLUES SAMPLING   |    NORMAL RATE      |
   *     | random < best    | random > best     |                   |          	          |
   * ---------------------------------------------------------------------------------------------------------
   *  1  | Best throughput  | Random rate       | Round Robin rate  | Best utility	  |
   *  2  | Random rate      | Best throughput   | Best throughput   | Second best utility |
   *  3  | Best probability | Best probability  | Best probability  | Best probability    |
   *  4  | Lowest Baserate  | Lowest baserate   | Lowest baserate   | Lowest baserate     |
   *
   * Note: For clarity, multiple blocks of if's and else's are used
   * After a failing 7 times, DoReportFinalDataFailed will be called
   */

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  station->m_longRetryCount++;

  /**
   * Increment the attempts counter for the rate and power used.
   * In the article the attempts are increased only by one and not by the number of the attempted retries.
   * However the correct behavior is to count every attempt.
   */
  station->m_minstrelBluesTable[station->m_currentRate].numRateAttempt++;
  if (station->m_currentPower == station->m_minstrelBluesTable[station->m_currentRate].refPower)
    {
      station->m_minstrelBluesTable[station->m_currentRate].numRefAttempt++;
    }
  else if (station->m_currentPower == station->m_minstrelBluesTable[station->m_currentRate].dataPower)
    {
      station->m_minstrelBluesTable[station->m_currentRate].numDataAttempt++;
    }
  else if (station->m_currentPower == station->m_minstrelBluesTable[station->m_currentRate].samplePower)
    {
      station->m_minstrelBluesTable[station->m_currentRate].numSampleAttempt++;
    }

  //PrintTable (station);

  NS_LOG_DEBUG ("DoReportDataFailed " << station << " rate " << station->m_currentRate << " power " << (int)station->m_currentPower << " longRetry " << station->m_longRetryCount);

  /// NORMAL RATE
  if (!station->m_isSampling && !station->m_samplingBlues)
    {
      NS_LOG_DEBUG ("Failed with normal rate: current=" << station->m_currentRate <<
                    ", maxUtil=" << station->m_maxURate <<
                    ", maxUtil2=" << station->m_maxURate2 <<
                    ", maxProb=" << station->m_maxProbRate);
      /// use best utility rate
      if (station->m_longRetryCount < station->m_minstrelBluesTable[station->m_maxURate].adjustedRetryCount)
        {
          NS_LOG_DEBUG ("More retries left for the maximum utility rate.");
          station->m_currentRate = station->m_maxURate;
        }

      /// use second best utility rate
      else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_maxURate].adjustedRetryCount +
                                         station->m_minstrelBluesTable[station->m_maxURate2].adjustedRetryCount))
        {
          NS_LOG_DEBUG ("More retries left for the second best utility rate.");
          station->m_currentRate = station->m_maxURate2;
        }

      /// use best probability rate
      else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_maxURate].adjustedRetryCount +
                                         station->m_minstrelBluesTable[station->m_maxURate2].adjustedRetryCount +
                                         station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount))
        {
          NS_LOG_DEBUG ("More retries left for the maximum probability rate.");
          station->m_currentRate = station->m_maxProbRate;
        }

      /// use lowest rate
      else if (station->m_longRetryCount > (station->m_minstrelBluesTable[station->m_maxURate].adjustedRetryCount +
                                        station->m_minstrelBluesTable[station->m_maxURate2].adjustedRetryCount +
                                        station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount))
        {
          NS_LOG_DEBUG ("More retries left for the base rate.");
          station->m_currentRate = 0;
        }

      station->m_currentPower = station->m_minstrelBluesTable[station->m_currentRate].dataPower;
      m_rateChange("normal", station->m_currentRate, station->m_state->m_address);
      m_powerChange("data", station->m_currentPower, station->m_state->m_address);
    }

  /// MINSTREL SAMPLING
  /// for look-around rate, we're currently sampling random rates
  else if (station->m_isSampling)
    {
      NS_LOG_DEBUG ("Failed with look around rate: current=" << station->m_currentRate <<
                          ", sample=" << station->m_sampleRate <<
                          ", maxTp=" << station->m_maxThRate <<
                          ", maxTp2=" << station->m_maxThRate2 <<
                          ", maxProb=" << station->m_maxProbRate);
      /// current sampling rate is slower than the current best rate
      if (station->m_sampleRateSlower)
        {
          NS_LOG_DEBUG ("Look around rate is slower than the maximum throughput rate.");
          /// use best throughput rate
          if (station->m_longRetryCount < station->m_minstrelBluesTable[station->m_maxThRate].adjustedRetryCount)
            {
              NS_LOG_DEBUG ("More retries left for the maximum throughput rate.");
              station->m_currentRate = station->m_maxThRate;
            }

          ///   use random rate
          else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_maxThRate].adjustedRetryCount +
                                             station->m_minstrelBluesTable[station->m_sampleRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG ("More retries left for the sampling rate.");
              station->m_currentRate = station->m_sampleRate;
            }

          /// use max probability rate
          else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_maxThRate].adjustedRetryCount +
                                             station->m_minstrelBluesTable[station->m_sampleRate].adjustedRetryCount +
                                             station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount ))
            {
              NS_LOG_DEBUG ("More retries left for the maximum probability rate.");
              station->m_currentRate = station->m_maxProbRate;
            }

          /// use lowest base rate
          else if (station->m_longRetryCount > (station->m_minstrelBluesTable[station->m_maxThRate].adjustedRetryCount +
                                            station->m_minstrelBluesTable[station->m_sampleRate].adjustedRetryCount +
                                            station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG ("More retries left for the base rate.");
              station->m_currentRate = 0;
            }
        }

      /// current sampling rate is better than current best rate
      else
        {
          NS_LOG_DEBUG ("Look around rate is faster than the maximum throughput rate.");
          /// use random rate
          if (station->m_longRetryCount < station->m_minstrelBluesTable[station->m_sampleRate].adjustedRetryCount)
            {
              NS_LOG_DEBUG ("More retries left for the sampling rate.");
              station->m_currentRate = station->m_sampleRate;
            }

          /// use the best throughput rate
          else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_sampleRate].adjustedRetryCount +
                                            station->m_minstrelBluesTable[station->m_maxThRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG ("More retries left for the maximum throughput rate.");
              station->m_currentRate = station->m_maxThRate;
            }

          /// use the best probability rate
          else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_sampleRate].adjustedRetryCount +
                                            station->m_minstrelBluesTable[station->m_maxThRate].adjustedRetryCount +
                                            station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG ("More retries left for the maximum probability rate.");
              station->m_currentRate = station->m_maxProbRate;
            }

          /// use the lowest base rate
          else if (station->m_longRetryCount > (station->m_minstrelBluesTable[station->m_sampleRate].adjustedRetryCount +
                                           station->m_minstrelBluesTable[station->m_maxThRate].adjustedRetryCount +
                                           station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG ("More retries left for the base rate.");
              station->m_currentRate = 0;
            }
        }
      station->m_currentPower = station->m_minstrelBluesTable[station->m_currentRate].refPower;
      m_rateChange("minstrel", station->m_currentRate, station->m_state->m_address);
      m_powerChange("ref", station->m_currentPower, station->m_state->m_address);
    }
  //BLUES SAMPLING
  else
      {
	NS_LOG_DEBUG ("Failed with blues sample rate: current=" << station->m_currentRate <<
                                ", sample=" << station->m_bluesSampleRate <<
                                ", maxTp=" << station->m_maxThRate <<
                                ", maxProb=" << station->m_maxProbRate);
	NS_LOG_DEBUG ("Look around rate is faster than the maximum throughput rate.");
	/// use random rate
	if (station->m_longRetryCount < station->m_minstrelBluesTable[station->m_bluesSampleRate].adjustedRetryCount)
	  {
	    NS_LOG_DEBUG ("More retries left for the sampling rate.");
	    station->m_currentRate = station->m_bluesSampleRate;
	    station->m_currentPower = station->m_minstrelBluesTable[station->m_bluesSampleRate].samplePower;
	    m_powerChange("sample", station->m_currentPower, station->m_state->m_address);
	  }

	/// use the best throughput rate
	else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_bluesSampleRate].adjustedRetryCount +
					  station->m_minstrelBluesTable[station->m_maxThRate].adjustedRetryCount))
	  {
	    NS_LOG_DEBUG (" More retries left for the maximum throughput rate.");
	    station->m_currentRate = station->m_maxThRate;
	    station->m_currentPower = station->m_minstrelBluesTable[station->m_currentRate].dataPower;
	    m_powerChange("data", station->m_currentPower, station->m_state->m_address);
	  }

	/// use the best probability rate
	else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_bluesSampleRate].adjustedRetryCount +
					  station->m_minstrelBluesTable[station->m_maxThRate].adjustedRetryCount +
					  station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount))
	  {
	    NS_LOG_DEBUG (" More retries left for the maximum probability rate.");
	    station->m_currentRate = station->m_maxProbRate;
	    station->m_currentPower = station->m_minstrelBluesTable[station->m_currentRate].dataPower;
	    m_powerChange("data", station->m_currentPower, station->m_state->m_address);
	  }

	/// use the lowest base rate
	else if (station->m_longRetryCount > (station->m_minstrelBluesTable[station->m_bluesSampleRate].adjustedRetryCount +
					 station->m_minstrelBluesTable[station->m_maxThRate].adjustedRetryCount +
					 station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount))
	  {
	    NS_LOG_DEBUG (" More retries left for the base rate.");
	    station->m_currentRate = 0;
	    station->m_currentPower = station->m_minstrelBluesTable[station->m_currentRate].dataPower;
	    m_powerChange("data", station->m_currentPower, station->m_state->m_address);
	  }
        m_rateChange("blues", station->m_currentRate, station->m_state->m_address);
      }
}

/**
 * This function is used to say that all retry attempts fail.
 * Before calling this function, DoReportDataFailed was called.
 */
void
MinstrelBluesWifiManager::DoReportFinalDataFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this);
  MinstrelBluesWifiRemoteStation *station = (MinstrelBluesWifiRemoteStation *) st;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  station->m_isSampling = false;
  station->m_sampleRateSlower = false;
  station->m_samplingBlues = false;
  station->m_bluesSampleRateSlower = false;

  UpdateRetry (station);

  station->m_retryErrorCount++;

  /**
   * If enough samples, stats will be updated.
   */
  MinstrelUpdateStats (station);
  BluesUpdateStats(station);

  SetRatePower(station);

  //PrintTable (station);
}

void
MinstrelBluesWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                     double ackSnr, WifiMode ackMode, double dataSnr)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_FUNCTION (st << ackSnr << ackMode << dataSnr);
  MinstrelBluesWifiRemoteStation *station = (MinstrelBluesWifiRemoteStation *) st;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  station->m_isSampling = false;
  station->m_sampleRateSlower = false;
  station->m_samplingBlues = false;
  station->m_bluesSampleRateSlower = false;

  /*NS_LOG_DEBUG ("DoReportDataOk m_txrate = " << station->m_currentRate << ", power = " << (int) station->m_currentPower <<
                ", attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numRateAttempt <<
                ", success = " << station->m_minstrelBluesTable[station->m_currentRate].numRateSuccess <<
                ", ref attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numRefAttempt <<
                ", ref success = " << station->m_minstrelBluesTable[station->m_currentRate].numRefSuccess <<
                ", data attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numDataAttempt <<
                ", data success = " << station->m_minstrelBluesTable[station->m_currentRate].numDataSuccess <<
                ", sample attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numSampleAttempt <<
                ", sample success = " << station->m_minstrelBluesTable[station->m_currentRate].numSampleSuccess << " (before update).");*/

  station->m_minstrelBluesTable[station->m_currentRate].numRateSuccess++;
  station->m_minstrelBluesTable[station->m_currentRate].numRateAttempt++;

  //TODO esto es lo que no funciona bien, si no estanbien separados los powers.
  if (station->m_currentPower == station->m_minstrelBluesTable[station->m_currentRate].refPower)
    {
      station->m_minstrelBluesTable[station->m_currentRate].numRefSuccess++;
      station->m_minstrelBluesTable[station->m_currentRate].numRefAttempt++;
    }
  else if (station->m_currentPower == station->m_minstrelBluesTable[station->m_currentRate].dataPower)
    {
      station->m_minstrelBluesTable[station->m_currentRate].numDataSuccess++;
      station->m_minstrelBluesTable[station->m_currentRate].numDataAttempt++;
    }
  else if (station->m_currentPower == station->m_minstrelBluesTable[station->m_currentRate].samplePower)
    {
      station->m_minstrelBluesTable[station->m_currentRate].numSampleSuccess++;
      station->m_minstrelBluesTable[station->m_currentRate].numSampleAttempt++;
    }

 /*NS_LOG_DEBUG ("DoReportDataOk m_txrate = " << station->m_currentRate << ", power = " << (int) station->m_currentPower <<
                  ", attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numRateAttempt <<
                  ", success = " << station->m_minstrelBluesTable[station->m_currentRate].numRateSuccess <<
                  ", ref attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numRefAttempt <<
                  ", ref success = " << station->m_minstrelBluesTable[station->m_currentRate].numRefSuccess <<
                  ", data attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numDataAttempt <<
                  ", data success = " << station->m_minstrelBluesTable[station->m_currentRate].numDataSuccess <<
                  ", sample attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numSampleAttempt <<
                  ", sample success = " << station->m_minstrelBluesTable[station->m_currentRate].numSampleSuccess << " (after update).");*/

  UpdateRetry (station); //updates m_retryCount and set long and short retry to 0

  station->m_frameCount++;

  /**
   * If enough samples, stats will be updated.
   */
  MinstrelUpdateStats (station);
  BluesUpdateStats(station);

  SetRatePower(station);

  //PrintTable (station);
}

void
MinstrelBluesWifiManager::UpdateRetry (MinstrelBluesWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  for (uint32_t i = 0; i < station->m_nSupported; i++)
    {
      station->m_retryCount = station->m_shortRetryCount + station->m_longRetryCount; //FIXME retryCount is never used!
      station->m_shortRetryCount = 0;
      station->m_longRetryCount = 0;
    }
}

WifiTxVector
MinstrelBluesWifiManager::DoGetDataTxVector (WifiRemoteStation *st,
                                    uint32_t size)
{
  NS_LOG_FUNCTION (this);
  MinstrelBluesWifiRemoteStation *station = (MinstrelBluesWifiRemoteStation *) st;
  if (!station->m_initialized)
    {
      CheckInit (station);

      /// start the rate at half way
      station->m_currentRate = station->m_nSupported / 2;
      m_rateChange("init", station->m_currentRate, station->m_state->m_address);
    }

  WifiTxVector vector =  WifiTxVector (GetSupported (station, station->m_currentRate),
                                       station->m_currentPower,
                                       GetLongRetryCount (station),
                                       GetShortGuardInterval (station),
                                       Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()),
                                       GetNumberOfTransmitAntennas (station),
                                       GetStbc (station));
  return vector;
}

WifiTxVector
MinstrelBluesWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this);
  MinstrelBluesWifiRemoteStation *station = (MinstrelBluesWifiRemoteStation *) st;
  NS_LOG_DEBUG ("DoGetRtsMode m_txrate=" << station->m_currentRate);

  return WifiTxVector (GetSupported (station, 0),
                        GetDefaultTxPowerLevel (),
                        GetShortRetryCount (station),
                        GetShortGuardInterval (station),
                        Min (GetNumberOfReceiveAntennas (station),GetNumberOfTransmitAntennas()),
                        GetNumberOfTransmitAntennas (station),
                        GetStbc (station));
}

bool
MinstrelBluesWifiManager::DoNeedDataRetransmission (WifiRemoteStation *st, Ptr<const Packet> packet, bool normally)
{
  NS_LOG_FUNCTION (this);
  MinstrelBluesWifiRemoteStation *station = (MinstrelBluesWifiRemoteStation *)st;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return normally;
    }

  if (!station->m_isSampling)
    {
      if (station->m_longRetryCount > (station->m_minstrelBluesTable[station->m_maxThRate].adjustedRetryCount +
                                  station->m_minstrelBluesTable[station->m_maxThRate2].adjustedRetryCount +
                                  station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount +
                                  station->m_minstrelBluesTable[0].adjustedRetryCount))
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
      if (station->m_longRetryCount > (station->m_minstrelBluesTable[station->m_sampleRate].adjustedRetryCount +
                                  station->m_minstrelBluesTable[station->m_maxThRate].adjustedRetryCount +
                                  station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount +
                                  station->m_minstrelBluesTable[0].adjustedRetryCount))
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
MinstrelBluesWifiManager::IsLowLatency (void) const
{
  NS_LOG_FUNCTION (this);
  return true;
}

uint32_t
MinstrelBluesWifiManager::GetNextSample (MinstrelBluesWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  uint32_t bitrate;
  bitrate = station->m_sampleTable[station->m_index][station->m_col];
  station->m_index++;

  /// bookeeping for m_index and m_col variables
  if (station->m_index > (station->m_nSupported - 2))
    {
      station->m_index = 0;
      station->m_col++;
      if (station->m_col >= m_nSampleColumns)
        {
          station->m_col = 0;
        }
    }
  return bitrate;
}

void
MinstrelBluesWifiManager::SetRatePower (MinstrelBluesWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);

  uint32_t rate;
  uint8_t power;

  station->m_samplingBlues = false;

  /**
   * If we have not sent any frames yet, then use low rate and high power.
   * FIXME is this ok?
   */
  if ((station->m_minstrelSampleCount + station->m_frameCount) == 0)
    {
      rate = 0;
      power = m_maxPower;
      m_rateChange("init", rate, station->m_state->m_address);
      m_powerChange("init", power, station->m_state->m_address);
    }
  else
    {
      /// For determining when to try a sample rate.
      int coinFlip = m_uniformRandomVariable->GetInteger (0, 100) % 2;

      /**
       * If we are below the target of look around rate percentage, look around.
       * Note: do it randomly by flipping a coin instead of sampling
       * all at once until it reaches the look around rate.
       */
      if ( (((100 * station->m_minstrelSampleCount) / (station->m_minstrelSampleCount + station->m_frameCount )) < m_minstrelSamplingRatio)
           && (coinFlip == 1) )
        {
	  /// MINSTREL SAMPLING

	  NS_LOG_DEBUG ("Using look around rate");


	  /// set flag that we are currently sampling
	  station->m_isSampling = true;

          /// Now go through the table and find an index rate.
          rate = GetNextSample (station);

	  /// start sample count
	  station->m_minstrelSampleCount++;

	  /// bookeeping for resetting stuff
	  if (station->m_frameCount >= 10000)
	    {
	      station->m_minstrelSampleCount = 0;
	      station->m_frameCount = 0;
	    }

	  /// Set the rate that we're currently sampling.
	  station->m_sampleRate = rate;

	  /*
	   This was done in the ns3 minstrel implementation but I think it
	   is not part of the specification. FIXME
	  /// We make the random not to be the best throughput.
	  if (station->m_sampleRate == station->m_maxTpRate)
	    {
	      station->m_sampleRate = station->m_maxTpRate2;
	    }
	  */

	  /// Is this rate slower than the current best rate.
	  station->m_sampleRateSlower =
	    (station->m_minstrelBluesTable[rate].perfectTxTime > station->m_minstrelBluesTable[station->m_maxThRate].perfectTxTime);

	  /// using the best rate instead
	  if (station->m_sampleRateSlower)
	    {
	      NS_LOG_DEBUG ("The next look around rate is slower than the maximum throughput rate, continue with the maximum throughput rate: " <<
			    station->m_maxThRate << "(" << GetSupported (station, station->m_maxThRate) << ")");
	      rate =  station->m_maxThRate;
	    }
          m_rateChange("minstrel", rate, station->m_state->m_address);

          /*
           * When Minstrel sampling use reference power.
           * Use the same power for all rates in the retry chain.
           */
          power = station->m_minstrelBluesTable[rate].refPower;
          m_powerChange("ref", power, station->m_state->m_address);
          NS_LOG_DEBUG ("With sample rate use reference power: " << (int) power);
        }
      else if (station->m_bluesSampleCount > 1)  //suggested by thomas
	{
	  /// NO SAMPLING, NORMAL RATE
	  NS_LOG_DEBUG ("Using normal rate");
	  station->m_bluesSampleCount--;
	  rate = station->m_maxURate;
	  power = station->m_minstrelBluesTable[rate].dataPower;
	  m_rateChange("normal", rate, station->m_state->m_address);
	  m_powerChange("data", power, station->m_state->m_address);
	  NS_LOG_DEBUG("Data packet: rate= " << rate << "(" << GetSupported (station, rate) << ") power= " << (int)power);
	}
      else
	{
	  // BLUES SAMPLING
	  /**
	   * The set of rates that Blues considers for its own sampling consists of the four highest
	   * throughput rates determined by Minstrels statistics. With this approach, we reduce the rates that Blues
	   * uses for sampling to the data-rates that support potentially high throughput.
	   * If any data-rate below the maximum throughput rate is selected for power sampling,
	   * the rate order in the multi-rate-retry (mrr) chain is also changed accordingly, for consistency.
	   * Consulted Thomas about this, the change consists on selecting the third highest throughput rate
	   * only if it is faster than the first. Else, the max prob rate is selected.
	   */
	  NS_LOG_DEBUG ("Using blues sampling rate");
	  station->m_samplingBlues = true;
	  station->m_bluesSampleCount = 100 / m_bluesSamplingRatio;

	  // choose round robin between the 4 best rate
	  if (station->m_bluesRoundRobinIndex < 2)
	    {
              station->m_bluesSampleRate = station->m_sortedThRates[station->m_bluesRoundRobinIndex];
	    }
	  else
	    {
	      NS_ASSERT(station->m_bluesRoundRobinIndex == 2);
	      if (station->m_minstrelBluesTable[station->m_sortedThRates[station->m_bluesRoundRobinIndex]].perfectTxTime >
	          station->m_minstrelBluesTable[station->m_sortedThRates[0]].perfectTxTime)
                {
                  station->m_bluesSampleRate = station->m_sortedThRates[station->m_bluesRoundRobinIndex];
                }
              else
                {
                  station->m_bluesSampleRate = station->m_maxProbRate;
                }
	    }
          station->m_bluesRoundRobinIndex = (station->m_bluesRoundRobinIndex + 1) % (N_MAX_TH_RATES - 1);
	  rate = station->m_bluesSampleRate;

	  m_rateChange("blues", rate, station->m_state->m_address);

	  // Keep a balanced sampling between tpc_ref and tpc_sample.
	  if (station->m_minstrelBluesTable[rate].numRefAttempt > station->m_minstrelBluesTable[rate].numSampleAttempt)
	    {
	      power = station->m_minstrelBluesTable[rate].samplePower;
	      m_powerChange("sample", power, station->m_state->m_address);
	    }
	  else
	    {
	      power = station->m_minstrelBluesTable[rate].refPower;
	      m_powerChange("ref", power, station->m_state->m_address);
	    }
	  NS_LOG_DEBUG("Sample power packet: rate= " << rate << "(" << GetSupported (station, rate) << ") power= " << (int)power);
	}
    }
  station->m_currentRate = rate;
  station->m_currentPower = power;
}

double
MinstrelBluesWifiManager::BluesUtility (MinstrelBluesWifiRemoteStation *station, uint32_t rate)
{
  NS_LOG_FUNCTION (this);
  uint32_t th = station->m_minstrelBluesTable[rate].throughput;
  uint32_t bestTh = station->m_minstrelBluesTable[station->m_maxThRate].throughput;
  if (th != 0 && bestTh != 0)
    {
      double benefit = (th/(double)bestTh)*100;
      uint8_t power = station->m_minstrelBluesTable[rate].dataPower;
      uint8_t bestPower = station->m_minstrelBluesTable[station->m_maxThRate].dataPower;
      double cost = (power/(double)th)*(bestTh/(double)bestPower)*100;
      return m_bluesUilityWeight*benefit-cost;
    }
  else
    {
      return 0;
    }
}

void
MinstrelBluesWifiManager::MinstrelUpdateStats (MinstrelBluesWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  if (Simulator::Now () <  station->m_nextStatsUpdate)
    {
      return;
    }

  if (!station->m_initialized)
    {
      return;
    }

  //Reset sorted list.
  for (uint32_t i = 0; i < N_MAX_TH_RATES; i++)
      station->m_sortedThRates[i]=0;

  station->m_nextStatsUpdate = Simulator::Now () + m_updateStats;
  NS_LOG_DEBUG ("Next update at " << station->m_nextStatsUpdate);
  NS_LOG_DEBUG ("Currently using rate: " << station->m_currentRate << " (" << GetSupported (station, station->m_currentRate) << ")");

  Time txTime;
  uint32_t tempProb;

  uint32_t index_max_prob = 0;

  NS_LOG_DEBUG ("Index-Rate\t\tAttempt\tSuccess");
  for (uint32_t i = 0; i < station->m_nSupported; i++)
    {

      /// obtain the perfect tx time for this rate
      txTime = station->m_minstrelBluesTable[i].perfectTxTime;

      /// just for initialization
      if (txTime.GetMicroSeconds () == 0)
        {
          txTime = Seconds (1);
        }

      NS_LOG_DEBUG (i << " " << GetSupported (station, i) <<
                          "\t" << station->m_minstrelBluesTable[i].numRateAttempt <<
                          "\t" << station->m_minstrelBluesTable[i].numRateSuccess);

      /// if we've attempted something
      if (station->m_minstrelBluesTable[i].numRateAttempt)
        {
          /**
           * Calculate the probability of success.
           * Assume probability scales from 0 to 18000
           */
          tempProb = (station->m_minstrelBluesTable[i].numRateSuccess * 18000) / station->m_minstrelBluesTable[i].numRateAttempt;

          /// bookeeping
          station->m_minstrelBluesTable[i].prob = tempProb;

          /// ewma probability (cast for gcc 3.4 compatibility)
          tempProb = static_cast<uint32_t> (((tempProb * (100 - m_ewmaCoefficient)) + (station->m_minstrelBluesTable[i].ewmaProb * m_ewmaCoefficient) ) / 100);

          station->m_minstrelBluesTable[i].ewmaProb = tempProb;

          /// calculating throughput
          station->m_minstrelBluesTable[i].throughput = tempProb * (1000000 / txTime.GetMicroSeconds ());

        }

      /// Reset counters.
      station->m_minstrelBluesTable[i].numRateSuccess = 0;
      station->m_minstrelBluesTable[i].numRateAttempt = 0;
      NS_LOG_DEBUG ("Attempt/success resetted to 0");

      /// Sample less often below 10% and above 95% of success
      if ((station->m_minstrelBluesTable[i].ewmaProb > 17100) || (station->m_minstrelBluesTable[i].ewmaProb < 1800))
        {
          /**
           * See: http://wireless.kernel.org/en/developers/Documentation/mac80211/RateControl/minstrel/
           *
           * Analysis of information showed that the system was sampling too hard at some rates.
           * For those rates that never work (54mb, 500m range) there is no point in sending 10 sample packets (< 6 ms time).
           * Consequently, for the very very low probability rates, we sample at most twice.
           */
          if (station->m_minstrelBluesTable[i].retryCount > 2)
            {
              station->m_minstrelBluesTable[i].adjustedRetryCount = 2;
            }
          else
            {
              station->m_minstrelBluesTable[i].adjustedRetryCount = station->m_minstrelBluesTable[i].retryCount;
            }
        }
      else
        {
          station->m_minstrelBluesTable[i].adjustedRetryCount = station->m_minstrelBluesTable[i].retryCount;
        }

      /// If it's 0 allow one retry limit.
      if (station->m_minstrelBluesTable[i].adjustedRetryCount == 0)
        {
          station->m_minstrelBluesTable[i].adjustedRetryCount = 1;
        }

      /**
       * After updating stats, found the highest throughput rates and the high probability rate.
       */

      MinstrelSortBestThRates(station, i);

      /**
       * To determine the most robust rate (max_prob_rate) used at 3rd mmr stage we distinct between two cases:
       * (1) if any success probabilitiy >= 95%, out of those rates choose the maximum throughput rate as max_prob_rate
       * (2) if all success probabilities < 95%, the rate with highest success probability is chosen as max_prob_rate
       */
      if (station->m_minstrelBluesTable[i].ewmaProb >= 17100)
	{
	  if (station->m_minstrelBluesTable[i].throughput >= station->m_minstrelBluesTable[index_max_prob].throughput)
	    index_max_prob = i;
	}
      else
	{
	  if (station->m_minstrelBluesTable[i].ewmaProb >= station->m_minstrelBluesTable[index_max_prob].ewmaProb)
	    index_max_prob = i;
	}
    }

  /// Set the rates.
  station->m_maxThRate = station->m_sortedThRates[0];
  station->m_maxThRate2 = station->m_sortedThRates[1];
  station->m_maxProbRate = index_max_prob;

  /// Obtain the 2 best rates for blues utility function.
  double max_util = 0, max_util2 = 0;
  uint32_t index_max_util = 0, index_max_util2 = 0;
  for (uint32_t i = 0; i < N_MAX_TH_RATES; i++)
    {
      double max_temp = BluesUtility(station, station->m_sortedThRates[i]);
      NS_LOG_DEBUG (i << " " << station->m_sortedThRates[i] << " " << max_temp << " " <<
                    station->m_minstrelBluesTable[station->m_sortedThRates[i]].throughput << " " <<
                    (int)station->m_minstrelBluesTable[station->m_sortedThRates[i]].dataPower);

      if (max_util < max_temp)
        {
	  index_max_util2 = index_max_util;
          index_max_util = station->m_sortedThRates[i];
          max_util2 = max_util;
          max_util = max_temp;
        }
      else if (max_util2 < max_temp)
	{
	  index_max_util2 = station->m_sortedThRates[i];
	  max_util2 = max_temp;
	}
    }
  station->m_maxURate = index_max_util;
  station->m_maxURate2 = index_max_util2;

  NS_LOG_DEBUG ("maxThRate=" << station->m_maxThRate << "(" << GetSupported (station, station->m_maxThRate) <<
                ") th=" << station->m_minstrelBluesTable[station->m_maxThRate].throughput <<
                "\nmaxThRate2=" << station->m_maxThRate2  << "(" << GetSupported (station, station->m_maxThRate2 ) <<
                ") th=" << station->m_minstrelBluesTable[station->m_maxThRate2].throughput <<
                "\nmaxThRate3=" << station->m_sortedThRates[2] << "(" << GetSupported (station, station->m_sortedThRates[2]) <<
                ") th=" << station->m_minstrelBluesTable[station->m_sortedThRates[2]].throughput <<
                "\nmaxThRate4=" << station->m_sortedThRates[2]  << "(" << GetSupported (station, station->m_sortedThRates[2] ) <<
                ") th=" << station->m_minstrelBluesTable[station->m_sortedThRates[3]].throughput <<
                ")\nmaxProbRate=" << station->m_maxProbRate << "(" << GetSupported (station, station->m_maxProbRate) <<
                ") prob=" << station->m_minstrelBluesTable[station->m_maxProbRate].ewmaProb <<
                "\nmaxURate=" << station->m_maxURate << "(" << GetSupported (station, station->m_maxURate) <<
                ") utility=" << max_util <<
                "\nmaxURate2=" << station->m_maxURate2 << "(" << GetSupported (station, station->m_maxURate2) <<
                ") utility=" << max_util2);
}

void
MinstrelBluesWifiManager::MinstrelSortBestThRates (MinstrelBluesWifiRemoteStation *station, uint32_t i)
{
  NS_LOG_FUNCTION (this);

  //FIXME I "copy" what is done in linux wireless minstrel algorithm but it appears to be a bug, when
  // rate 0 is the higher (or one of the highest) th rates.
  uint32_t j = N_MAX_TH_RATES - 1;
  while (j > 0 && station->m_minstrelBluesTable[i].throughput > station->m_minstrelBluesTable[station->m_sortedThRates[j]].throughput)
    {
      station->m_sortedThRates[j] = station->m_sortedThRates[j-1];
      j--;
    }
  if (j < N_MAX_TH_RATES - 1)
    station->m_sortedThRates[j] = i;
}

void
MinstrelBluesWifiManager::BluesUpdateStats (MinstrelBluesWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  if (!station->m_initialized)
    {
      return;
    }

  Time txTime;
  uint32_t refTempProb;
  uint32_t dataTempProb;
  uint32_t sampleTempProb;

  for (uint32_t i = 0; i < station->m_nSupported; i++)
    {
      /**
       * If we have enough samples, update sample and reference statistics.
       * Then update all power levels.
       */
      if (station->m_minstrelBluesTable[i].numSampleAttempt > m_bluesUpdateStatsThreshold && station->m_minstrelBluesTable[i].numRefAttempt > m_bluesUpdateStatsThreshold)
        {
          /**
           * Calculate the probability of success.
           * Assume probability scales from 0 to 18000.
           */
          refTempProb = (station->m_minstrelBluesTable[i].numRefSuccess * 18000) / station->m_minstrelBluesTable[i].numRefAttempt;
          sampleTempProb = (station->m_minstrelBluesTable[i].numSampleSuccess * 18000) / station->m_minstrelBluesTable[i].numSampleAttempt;

          if (station->m_minstrelBluesTable[i].numDataAttempt)
            {
              dataTempProb = (station->m_minstrelBluesTable[i].numDataSuccess * 18000) / station->m_minstrelBluesTable[i].numDataAttempt;
            }
          else
            {
              dataTempProb = 18000;
            }

          /// bookeeping
          //TODO is necesary?
//          station->m_minstrelBluesTable[i].refProb = refTempProb;
//          station->m_minstrelBluesTable[i].dataProb = dataTempProb;
//          station->m_minstrelBluesTable[i].sampleProb = sampleTempProb;

          /// ewma probability (cast for gcc 3.4 compatibility)
          refTempProb = static_cast<uint32_t> (((refTempProb * (100 - m_ewmaCoefficient)) + (station->m_minstrelBluesTable[i].ewmaRefProb * m_ewmaCoefficient)) / 100);
          dataTempProb = static_cast<uint32_t> (((dataTempProb * (100 - m_ewmaCoefficient)) + (station->m_minstrelBluesTable[i].ewmaDataProb * m_ewmaCoefficient)) / 100);
          sampleTempProb = static_cast<uint32_t> (((sampleTempProb * (100 - m_ewmaCoefficient)) + (station->m_minstrelBluesTable[i].ewmaSampleProb * m_ewmaCoefficient)) / 100);

          // Update probabilities.
          station->m_minstrelBluesTable[i].ewmaRefProb = refTempProb;
          station->m_minstrelBluesTable[i].ewmaDataProb = dataTempProb;
          station->m_minstrelBluesTable[i].ewmaSampleProb = sampleTempProb;

          // Reset counters.
          station->m_minstrelBluesTable[i].numRefSuccess = 0;
          station->m_minstrelBluesTable[i].numRefAttempt = 0;
          station->m_minstrelBluesTable[i].numDataSuccess = 0;
          station->m_minstrelBluesTable[i].numDataAttempt = 0;
          station->m_minstrelBluesTable[i].numSampleSuccess = 0;
          station->m_minstrelBluesTable[i].numSampleAttempt = 0;

          //If throughput collapse, reset to initial settings
          if ((station->m_minstrelBluesTable[i].ewmaDataProb < m_thEmergency*18000) || (station->m_minstrelBluesTable[i].throughput == 0))
          {
        	  RateInit(station);
        	  NS_LOG_DEBUG("Throughput collapse at rate: " << i);
          }

          // Update powers.
          if (station->m_minstrelBluesTable[i].ewmaSampleProb < (station->m_minstrelBluesTable[i].ewmaRefProb - m_thIncPower*18000))
            {
              station->m_minstrelBluesTable[i].samplePower = Min((m_maxPower-m_bluesPowerStep), (station->m_minstrelBluesTable[i].samplePower + m_deltaIncPower)); //TODO According to Thomas sample power is always ref_power-tpc_power_step
            }

          if (station->m_minstrelBluesTable[i].ewmaDataProb > (station->m_minstrelBluesTable[i].ewmaRefProb - m_thDecPower*18000))
            {
              station->m_minstrelBluesTable[i].samplePower = Max(0,(station->m_minstrelBluesTable[i].samplePower - m_deltaDecPower));
            }

          if (station->m_minstrelBluesTable[i].ewmaRefProb < (18000 - m_thIncPower*18000))
            {
              station->m_minstrelBluesTable[i].refPower = Min((m_maxPower),(station->m_minstrelBluesTable[i].refPower + m_deltaIncPower));
            }

          if (station->m_minstrelBluesTable[i].ewmaRefProb > (18000 - m_thDecPower*18000))
            {
              station->m_minstrelBluesTable[i].refPower = Max(m_bluesPowerStep,(station->m_minstrelBluesTable[i].refPower - m_deltaDecPower)); //TODO minimum cannot be 0 but it should be tpc_power_step
            }

          station->m_minstrelBluesTable[i].dataPower = station->m_minstrelBluesTable[i].samplePower + m_deltaDataSamplePower;

          station->m_minstrelBluesTable[i].validityTimer = Simulator::Now();
        }
      /**
      * If we have enough data power samples, update data power statistics.
      * TODO en la thesis se ajustan las estadisticas de los sample power.
      */
      if (station->m_minstrelBluesTable[i].numDataAttempt > m_bluesUpdateStatsThreshold)
        {
          dataTempProb = (station->m_minstrelBluesTable[i].numDataSuccess * 18000) / station->m_minstrelBluesTable[i].numDataAttempt;
          //station->m_minstrelBluesTable[i].dataProb = dataTempProb;

          dataTempProb = static_cast<uint32_t> (((dataTempProb * (100 - m_ewmaCoefficient)) + (station->m_minstrelBluesTable[i].ewmaDataProb * m_ewmaCoefficient) ) / 100);
          station->m_minstrelBluesTable[i].ewmaDataProb = dataTempProb;
          station->m_minstrelBluesTable[i].numDataSuccess = 0;
          station->m_minstrelBluesTable[i].numDataAttempt = 0;
        }

      /**
       * Blues maintains a validity timer for the power statistics at each sampled rate
       * (set to 1 second by default). Power levels of such rates that Blues does not
       * actively sample, or rates where the statistic validity timer expired are set as follows:
       * (1) all higher data-rates above the highest throughput rate are assigned the maximum
       * power level and (2) the power level of the fourth highest throughput rate is used to
       * set the power level of the lower data-rates.
       * Thomas tell me that the validity timer is 10 seconds.
       */
// TODO revisar si esto es lo que se pretende
      if (station->m_minstrelBluesTable[i].validityTimer > Simulator::Now() + Seconds(10))
        {
          if (i > station->m_maxThRate)
            {
              station->m_minstrelBluesTable[i].dataPower = m_maxPower;
              station->m_minstrelBluesTable[i].refPower = m_maxPower;
              station->m_minstrelBluesTable[i].samplePower = station->m_minstrelBluesTable[i].refPower - 3;
            }
          else
            {
              station->m_minstrelBluesTable[i].dataPower =  station->m_minstrelBluesTable[station->m_sortedThRates[N_MAX_TH_RATES-1]].dataPower;
              station->m_minstrelBluesTable[i].refPower =  station->m_minstrelBluesTable[station->m_sortedThRates[N_MAX_TH_RATES-1]].refPower;
              station->m_minstrelBluesTable[i].samplePower =  station->m_minstrelBluesTable[station->m_sortedThRates[N_MAX_TH_RATES-1]].samplePower;
            }

        }
    }
}

void
MinstrelBluesWifiManager::RateInit (MinstrelBluesWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("RateInit=" << station);

  for (uint32_t i = 0; i < N_MAX_TH_RATES; i++)
    station->m_sortedThRates[i]=0;

  for (uint32_t i = 0; i < station->m_nSupported; i++)
    {
      NS_LOG_DEBUG ("Initializing rate index " << i << " " << GetSupported (station, i));
      station->m_minstrelBluesTable[i].numRateAttempt = 0;
      station->m_minstrelBluesTable[i].numRateSuccess = 0;
      station->m_minstrelBluesTable[i].prob = 0;
      station->m_minstrelBluesTable[i].ewmaProb = 0;
      station->m_minstrelBluesTable[i].throughput = 0;
      station->m_minstrelBluesTable[i].perfectTxTime = GetCalcTxTime (GetSupported (station, i));
      NS_LOG_DEBUG (" perfectTxTime = " << station->m_minstrelBluesTable[i].perfectTxTime);
      station->m_minstrelBluesTable[i].retryCount = 1;
      station->m_minstrelBluesTable[i].adjustedRetryCount = 1;
      // Emulating minstrel.c::ath_rate_ctl_reset
      // We only check from 2 to 10 retries. This guarantee that
      // at least one retry is permitted.
      Time totalTxTimeWithGivenRetries = Seconds (0.0); // tx_time in minstrel.c
      NS_LOG_DEBUG (" Calculating the number of retries");
      for (uint32_t retries = 2; retries < 11; retries++)
        {
          NS_LOG_DEBUG ("  Checking " << retries << " retries");
          totalTxTimeWithGivenRetries = CalculateTimeUnicastPacket (station->m_minstrelBluesTable[i].perfectTxTime, 0, retries);
          NS_LOG_DEBUG ("   totalTxTimeWithGivenRetries = " << totalTxTimeWithGivenRetries);
          if (totalTxTimeWithGivenRetries > MilliSeconds (6))
            {
              break;
            }
          station->m_minstrelBluesTable[i].retryCount = retries;
          station->m_minstrelBluesTable[i].adjustedRetryCount = retries;
        }
      /*blues init*/
      station->m_minstrelBluesTable[i].numRefAttempt = 0;
      station->m_minstrelBluesTable[i].numRefSuccess = 0;
      //station->m_minstrelBluesTable[i].refProb = 0;
      station->m_minstrelBluesTable[i].ewmaRefProb = 0;
      station->m_minstrelBluesTable[i].numDataAttempt = 0;
      station->m_minstrelBluesTable[i].numDataSuccess = 0;
      //station->m_minstrelBluesTable[i].dataProb = 0;
      station->m_minstrelBluesTable[i].ewmaDataProb = 0;
      station->m_minstrelBluesTable[i].numSampleAttempt = 0;
      station->m_minstrelBluesTable[i].numSampleSuccess = 0;
      //station->m_minstrelBluesTable[i].sampleProb = 0;
      station->m_minstrelBluesTable[i].ewmaSampleProb = 0;
      station->m_minstrelBluesTable[i].dataPower = m_maxPower;
      station->m_minstrelBluesTable[i].refPower = m_maxPower;
      station->m_minstrelBluesTable[i].samplePower = station->m_minstrelBluesTable[i].refPower - 3;
      station->m_minstrelBluesTable[i].validityTimer = Simulator::Now();
    }
}

Time
MinstrelBluesWifiManager::CalculateTimeUnicastPacket (Time dataTransmissionTime, uint32_t shortRetries, uint32_t longRetries)
{
  NS_LOG_FUNCTION (this);
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

  return tt;
}

void
MinstrelBluesWifiManager::InitSampleTable (MinstrelBluesWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("InitSampleTable=" << this);

  station->m_col = station->m_index = 0;

  /// for off-seting to make rates fall between 0 and numrates
  uint32_t numSampleRates = station->m_nSupported;

  uint32_t newIndex;
  for (uint32_t col = 0; col < m_nSampleColumns; col++)
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
              newIndex = (newIndex + 1) % station->m_nSupported;
            }
          station->m_sampleTable[newIndex][col] = i;

        }
    }
}

void
MinstrelBluesWifiManager::PrintSampleTable (MinstrelBluesWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("PrintSampleTable=" << station);

  uint32_t numSampleRates = station->m_nSupported;
  for (uint32_t i = 0; i < numSampleRates; i++)
    {
      for (uint32_t j = 0; j < m_nSampleColumns; j++)
        {
          NS_LOG_DEBUG(station->m_sampleTable[i][j] << "\t");
        }
      NS_LOG_DEBUG("\n");
    }
}

void
MinstrelBluesWifiManager::PrintTable (MinstrelBluesWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("PrintTable=" << station);

  NS_LOG_DEBUG ("index throughput ewmaProb prob success(attempt) refPower samplePower dataPower ewmaRefProb ewmaSampleProb ewmaDataProb refSuccess(refAttempt) sampleSucces(sampleAttempt) dataSuccess(dataAttempt)\n");
  for (uint32_t i = 0; i < station->m_nSupported; i++)
    {
      NS_LOG_DEBUG( std::setw(2) << i << std::setw(10) << station->m_minstrelBluesTable[i].throughput << std::setw(10) << station->m_minstrelBluesTable[i].ewmaProb << std::setw(10) << station->m_minstrelBluesTable[i].prob
                  << std::setw(10) << station->m_minstrelBluesTable[i].numRateSuccess << "(" << station->m_minstrelBluesTable[i].numRateAttempt << ")"
                  << std::setw(10) << (int) station->m_minstrelBluesTable[i].refPower << std::setw(10) << (int) station->m_minstrelBluesTable[i].samplePower << std::setw(10) << (int) station->m_minstrelBluesTable[i].dataPower
                  << std::setw(10) << station->m_minstrelBluesTable[i].ewmaRefProb << std::setw(10) << station->m_minstrelBluesTable[i].ewmaSampleProb << std::setw(10) << station->m_minstrelBluesTable[i].ewmaDataProb
                  << std::setw(15) << station->m_minstrelBluesTable[i].numRefSuccess << "(" << station->m_minstrelBluesTable[i].numRefAttempt << ")"
                  << std::setw(15) << station->m_minstrelBluesTable[i].numSampleSuccess << "(" << station->m_minstrelBluesTable[i].numSampleAttempt << ")"
                  << std::setw(15) << station->m_minstrelBluesTable[i].numDataSuccess << "(" << station->m_minstrelBluesTable[i].numDataAttempt << ")"
                  <<"\n");
    }
}

} // namespace ns3






