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

  uint32_t m_maxTpRate;  //!< The current highest throughput rate.
  uint32_t m_maxTpRate2;  //!< The second highest throughput rate.
  uint32_t m_maxProbRate;  //!< The rate with highest probability of success.

  uint32_t m_currentRate;  //!< Current transmission rate.

  uint32_t m_frameCount;  //!< Number of frames transmitted.
  uint32_t m_sampleCount;  //!< Number of sample frames transmitted.

  bool m_isSampling;  //!< A flag to indicate we are currently sampling.
  uint32_t m_sampleRate;  //!< Current sample rate.
  bool  m_sampleRateSlower;  //!< A flag to indicate sample rate is slower.

  uint32_t m_shortRetryCount;  //!< Number of short retries (such as control frames).
  uint32_t m_longRetryCount;  //!< Number of long retries (such as data packets).
  uint32_t m_retryCount;  //!< Number of retries (short + long).
  uint32_t m_retryErrorCount;  //!< Number of retry errors (all retransmission attempts failed).

  bool m_initialized;  //!< For initializing variables.

  bool m_sampleBest;  //!< A flag to indicate if we are sampling with the best rate.

  MinstrelBluesRate m_minstrelBluesTable;  //!< Minstrel Blues table.
  SampleRate m_sampleTable;  //!< Sample rates table.

  uint32_t m_bluesSampleCount;  //!< Number of sample frames transmitted.
  uint32_t m_bluesRefCount;  //!< Number of reference frames transmitted.

  uint8_t m_currentPower;  //!< Current transmission power level.
  uint8_t m_refPower;  //!< Current transmission power level for reference frames.
  uint8_t m_samplePower;  //!< Current transmission power level for sample frames.
  uint8_t m_dataPower;  //!< Current transmission power level for data frames.

  uint32_t m_nSupported; //!< Number of supported rates by the remote station.

  bool m_samplingBlues; //! If Blues is sampling.
  uint32_t m_bluesSampleRate; //! Blues sample rate.

  /**
   * Best four rates from blues utility function.
   */
  uint32_t m_maxURate;
  uint32_t m_maxURate2;
  uint32_t m_maxURate3;
  uint32_t m_maxURate4;
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
                   MakeDoubleAccessor (&MinstrelBluesWifiManager::m_lookAroundRatePercentage),
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
                   MakeDoubleAccessor (&MinstrelBluesWifiManager::m_lookAroundSamplePowerPercentage),
                   MakeDoubleChecker <double> ())
   .AddAttribute ("LookAroundReferencePower",
		  "The percentage to try other reference powers than our current reference power.",
		  DoubleValue (10),
		  MakeDoubleAccessor (&MinstrelBluesWifiManager::m_lookAroundReferencePowerPercentage),
		  MakeDoubleChecker <double> ())
    .AddAttribute ("BluesUpdateStats",
                   "How many reference and sample frames are needed for updating statistics.",
                   UintegerValue (10),
                   MakeUintegerAccessor (&MinstrelBluesWifiManager::m_bluesUpdateStatsThreshold),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("IncrementLevel",
                   "How many levels to increase power each time.",
                   UintegerValue (1),
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
    .AddAttribute ("BluesUtilityWeight",
		   "The weight to use in the utility function.",
		    DoubleValue (10),
		    MakeDoubleAccessor (&MinstrelBluesWifiManager::m_bluesUilityWeight),
		    MakeDoubleChecker <double> ())
    .AddTraceSource("PowerChange",
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
  station->m_maxTpRate = 0;
  station->m_maxTpRate2 = 0;
  station->m_maxProbRate = 0;
  station->m_currentRate = 0;
  station->m_frameCount = 0;
  station->m_sampleCount = 0;
  station->m_isSampling = false;
  station->m_sampleRate = 0;
  station->m_sampleRateSlower = false;
  station->m_shortRetryCount = 0;
  station->m_longRetryCount = 0;
  station->m_retryCount = 0;
  station->m_retryErrorCount = 0;
  station->m_initialized = false;
  station->m_sampleBest = false;
  station->m_bluesRefCount = 0;
  station->m_bluesSampleCount = 100 / m_lookAroundSamplePowerPercentage;
  station->m_samplingBlues = false;
  station->m_bluesSampleRate = 0;
  station->m_currentPower = m_maxPower;
  station->m_dataPower = m_maxPower;
  station->m_refPower = m_maxPower;
  station->m_samplePower = m_maxPower-3;
  station->m_maxURate = 0;
  station->m_maxURate2 = 0;
  station->m_maxURate3 = 0;
  station->m_maxURate4 = 0;
  station->m_bluesSampleRate = 0;

  m_powerChange("init", station->m_currentPower, station->m_state->m_address);
  m_rateChange(station->m_currentRate, station->m_state->m_address);

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
   *	          MINSTREL SAMPLING		|      		     |
   * Try |         LOOKAROUND RATE              |   BLUES SAMPLING   | NORMAL RATE
   *     | random < best    | random > best     |		     |
   * -------------------------------------------------------------------------------------------
   *  1  | Best throughput  | Random rate       | Round Robin rate   | Best utility
   *  2  | Random rate      | Best throughput   | Best throughput    | Second best utility
   *  3  | Best probability | Best probability  | Best probability   | Best probability
   *  4  | Lowest Baserate  | Lowest baserate   | Lowest baserate    | Lowest baserate
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
  station->m_minstrelBluesTable[station->m_currentRate].numRateAttempt++;

  //In the article the attempts are increased only by one and not by the number of the attempted retries.
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

  /// for normal rate, we're not currently sampling random rates
  if (!station->m_isSampling && station->m_samplingBlues)
    {
      NS_LOG_DEBUG ("Failed with normal rate: current=" << station->m_currentRate <<
                    ", maxUtil=" << station->m_maxURate <<
                    ", maxUtil2=" << station->m_maxURate2 <<
                    ", maxProb=" << station->m_maxProbRate);
      /// use best utility rate
      if (station->m_longRetryCount < station->m_minstrelBluesTable[station->m_maxURate].adjustedRetryCount)
        {
          NS_LOG_DEBUG (" More retries left for the maximum utility rate.");
          //station->m_txrate = station->m_maxURate;
        }

      /// use second best utility rate
      else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_maxURate].adjustedRetryCount +
                                         station->m_minstrelBluesTable[station->m_maxURate2].adjustedRetryCount))
        {
          NS_LOG_DEBUG (" More retries left for the second maximum throughput rate.");
          station->m_currentRate = station->m_maxURate2;
        }

      /// use best probability rate
      else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_maxURate].adjustedRetryCount +
                                         station->m_minstrelBluesTable[station->m_maxURate2].adjustedRetryCount +
                                         station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount))
        {
          NS_LOG_DEBUG (" More retries left for the maximum probability rate.");
          station->m_currentRate = station->m_maxProbRate;
        }

      /// use fourth best utility rate
      else if (station->m_longRetryCount > (station->m_minstrelBluesTable[station->m_maxURate].adjustedRetryCount +
                                        station->m_minstrelBluesTable[station->m_maxURate2].adjustedRetryCount +
                                        station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount))
        {
          NS_LOG_DEBUG (" More retries left for the base rate.");
          station->m_currentRate = 0;
        }

      station->m_currentPower = station->m_minstrelBluesTable[station->m_currentRate].dataPower;
      m_rateChange(station->m_currentRate, station->m_state->m_address);
      m_powerChange("data", station->m_currentPower, station->m_state->m_address);
    }

  /// MINSTREL SAMPLING
  /// for look-around rate, we're currently sampling random rates
  else if (station->m_isSampling)
    {
      NS_LOG_DEBUG ("Failed with look around rate: current=" << station->m_currentRate <<
                          ", sample=" << station->m_sampleRate <<
                          ", maxTp=" << station->m_maxTpRate <<
                          ", maxTp2=" << station->m_maxTpRate2 <<
                          ", maxProb=" << station->m_maxProbRate);
      /// current sampling rate is slower than the current best rate
      if (station->m_sampleRateSlower)
        {
          NS_LOG_DEBUG ("Look around rate is slower than the maximum throughput rate.");
          /// use best throughput rate
          if (station->m_longRetryCount < station->m_minstrelBluesTable[station->m_maxTpRate].adjustedRetryCount)
            {
              NS_LOG_DEBUG (" More retries left for the maximum throughput rate.");
              //station->m_txrate = station->m_maxTpRate;
            }

          ///   use random rate
          else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_maxTpRate].adjustedRetryCount +
                                             station->m_minstrelBluesTable[station->m_sampleRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG (" More retries left for the sampling rate.");
              station->m_currentRate = station->m_sampleRate;
            }

          /// use max probability rate
          else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_maxTpRate].adjustedRetryCount +
                                             station->m_minstrelBluesTable[station->m_sampleRate].adjustedRetryCount +
                                             station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount ))
            {
              NS_LOG_DEBUG (" More retries left for the maximum probability rate.");
              station->m_currentRate = station->m_maxProbRate;
            }

          /// use lowest base rate
          else if (station->m_longRetryCount > (station->m_minstrelBluesTable[station->m_maxTpRate].adjustedRetryCount +
                                            station->m_minstrelBluesTable[station->m_sampleRate].adjustedRetryCount +
                                            station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG (" More retries left for the base rate.");
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
              NS_LOG_DEBUG (" More retries left for the sampling rate.");
              //station->m_txrate = station->m_sampleRate;
            }

          /// use the best throughput rate
          else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_sampleRate].adjustedRetryCount +
                                            station->m_minstrelBluesTable[station->m_maxTpRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG (" More retries left for the maximum throughput rate.");
              station->m_currentRate = station->m_maxTpRate;
            }

          /// use the best probability rate
          else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_sampleRate].adjustedRetryCount +
                                            station->m_minstrelBluesTable[station->m_maxTpRate].adjustedRetryCount +
                                            station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG (" More retries left for the maximum probability rate.");
              station->m_currentRate = station->m_maxProbRate;
            }

          /// use the lowest base rate
          else if (station->m_longRetryCount > (station->m_minstrelBluesTable[station->m_sampleRate].adjustedRetryCount +
                                           station->m_minstrelBluesTable[station->m_maxTpRate].adjustedRetryCount +
                                           station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount))
            {
              NS_LOG_DEBUG (" More retries left for the base rate.");
              station->m_currentRate = 0;
            }
        }
      station->m_currentPower = station->m_minstrelBluesTable[station->m_currentRate].refPower;
      m_rateChange(station->m_currentRate, station->m_state->m_address);
      m_powerChange("ref", station->m_currentPower, station->m_state->m_address);
    }
  //BLUES SAMPLING
  else
      {
	NS_LOG_DEBUG ("Look around rate is faster than the maximum throughput rate.");
	/// use random rate
	if (station->m_longRetryCount < station->m_minstrelBluesTable[station->m_bluesSampleRate].adjustedRetryCount)
	  {
	    NS_LOG_DEBUG (" More retries left for the sampling rate.");
	    //station->m_txrate = station->m_bluesSampleRate;
	    //station->m_txpower = station->m_minstrelTable[station->m_bluesSampleRate].samplePower;
	    m_rateChange(station->m_currentRate, station->m_state->m_address);
	    m_powerChange("sample", station->m_currentPower, station->m_state->m_address);
	  }

	/// use the best throughput rate
	else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_bluesSampleRate].adjustedRetryCount +
					  station->m_minstrelBluesTable[station->m_maxTpRate].adjustedRetryCount))
	  {
	    NS_LOG_DEBUG (" More retries left for the maximum throughput rate.");
	    station->m_currentRate = station->m_maxTpRate;
	    station->m_currentPower = station->m_minstrelBluesTable[station->m_currentRate].refPower;
	    m_rateChange(station->m_currentRate, station->m_state->m_address);
	    m_powerChange("ref", station->m_currentPower, station->m_state->m_address);
	  }

	/// use the best probability rate
	else if (station->m_longRetryCount <= (station->m_minstrelBluesTable[station->m_bluesSampleRate].adjustedRetryCount +
					  station->m_minstrelBluesTable[station->m_maxTpRate].adjustedRetryCount +
					  station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount))
	  {
	    NS_LOG_DEBUG (" More retries left for the maximum probability rate.");
	    station->m_currentRate = station->m_maxProbRate;
	    station->m_currentPower = station->m_minstrelBluesTable[station->m_currentRate].refPower;
	    m_rateChange(station->m_currentRate, station->m_state->m_address);
	    m_powerChange("ref", station->m_currentPower, station->m_state->m_address);
	  }

	/// use the lowest base rate
	else if (station->m_longRetryCount > (station->m_minstrelBluesTable[station->m_bluesSampleRate].adjustedRetryCount +
					 station->m_minstrelBluesTable[station->m_maxTpRate].adjustedRetryCount +
					 station->m_minstrelBluesTable[station->m_maxProbRate].adjustedRetryCount))
	  {
	    NS_LOG_DEBUG (" More retries left for the base rate.");
	    station->m_currentRate = 0;
	    station->m_currentPower = station->m_minstrelBluesTable[station->m_currentRate].refPower;
	    m_rateChange(station->m_currentRate, station->m_state->m_address);
	    m_powerChange("ref", station->m_currentPower, station->m_state->m_address);
	  }
      }
}

void
MinstrelBluesWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                     double ackSnr, WifiMode ackMode, double dataSnr)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_FUNCTION (st << ackSnr << ackMode << dataSnr);
  MinstrelBluesWifiRemoteStation *station = (MinstrelBluesWifiRemoteStation *) st;

  station->m_isSampling = false;
  station->m_sampleRateSlower = false;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  NS_LOG_DEBUG ("DoReportDataOk m_txrate = " << station->m_currentRate << ", power = " << (int) station->m_currentPower <<
                ", attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numRateAttempt <<
                ", success = " << station->m_minstrelBluesTable[station->m_currentRate].numRateSuccess <<
                ", ref attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numRefAttempt <<
                ", ref success = " << station->m_minstrelBluesTable[station->m_currentRate].numRefSuccess <<
                ", data attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numDataAttempt <<
                ", data success = " << station->m_minstrelBluesTable[station->m_currentRate].numDataSuccess <<
                ", sample attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numSampleAttempt <<
                ", sample success = " << station->m_minstrelBluesTable[station->m_currentRate].numSampleSuccess << " (before update).");

  station->m_minstrelBluesTable[station->m_currentRate].numRateSuccess++;
  station->m_minstrelBluesTable[station->m_currentRate].numRateAttempt++;

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

  NS_LOG_DEBUG ("DoReportDataOk m_txrate = " << station->m_currentRate << ", power = " << (int) station->m_currentPower <<
                  ", attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numRateAttempt <<
                  ", success = " << station->m_minstrelBluesTable[station->m_currentRate].numRateSuccess <<
                  ", ref attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numRefAttempt <<
                  ", ref success = " << station->m_minstrelBluesTable[station->m_currentRate].numRefSuccess <<
                  ", data attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numDataAttempt <<
                  ", data success = " << station->m_minstrelBluesTable[station->m_currentRate].numDataSuccess <<
                  ", sample attempt = " << station->m_minstrelBluesTable[station->m_currentRate].numSampleAttempt <<
                  ", sample success = " << station->m_minstrelBluesTable[station->m_currentRate].numSampleSuccess << " (after update).");

  UpdateRetry (station); //updates m_retry and set long and short retry to 0

  station->m_frameCount++;

  UpdateStats (station);
  UpdatePowerStats(station);

  if (station->m_nSupported >= 1)
    {
      SetRatePower(station);
    }

  PrintTable (station);
}

//this function is used to say that all retry attempts fail. Before calling this function, DoReportDataFailed was called.

void
MinstrelBluesWifiManager::DoReportFinalDataFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this);
  MinstrelBluesWifiRemoteStation *station = (MinstrelBluesWifiRemoteStation *) st;

  station->m_isSampling = false;
  station->m_sampleRateSlower = false;

  UpdateRetry (station);

  station->m_retryErrorCount++;

  UpdateStats (station);
  UpdatePowerStats(station);

  if (station->m_nSupported >= 1)
    {
      SetRatePower(station);
    }

  PrintTable (station);
}

void
MinstrelBluesWifiManager::UpdateRetry (MinstrelBluesWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  for (uint32_t i = 0; i < station->m_nSupported; i++)
    {
      station->m_retryCount = station->m_shortRetryCount + station->m_longRetryCount;
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
      m_rateChange(station->m_currentRate, station->m_state->m_address);
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
      if (station->m_longRetryCount > (station->m_minstrelBluesTable[station->m_maxTpRate].adjustedRetryCount +
                                  station->m_minstrelBluesTable[station->m_maxTpRate2].adjustedRetryCount +
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
                                  station->m_minstrelBluesTable[station->m_maxTpRate].adjustedRetryCount +
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

  if ((station->m_sampleCount + station->m_frameCount) == 0)
    {
      rate = 0;
      power = m_maxPower;
      m_rateChange(rate, station->m_state->m_address);
      m_powerChange("init", power, station->m_state->m_address);
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
      if ( (((100 * station->m_sampleCount) / (station->m_sampleCount + station->m_frameCount )) < m_lookAroundRatePercentage)
           && (coinFlip == 1) )
        {
          NS_LOG_DEBUG ("Using look around rate");
          /// now go through the table and find an index rate
          rate = GetNextSample (station);


          /**
           * This if condition is used to make sure that we don't need to use
           * the sample rate it is the same as our current rate
           */
          if (rate != station->m_maxTpRate && rate != station->m_currentRate)
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
              if (rate >= station->m_nSupported)
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
                (station->m_minstrelBluesTable[rate].perfectTxTime > station->m_minstrelBluesTable[station->m_maxTpRate].perfectTxTime);

              /// using the best rate instead
              if (station->m_sampleRateSlower)
                {
                  NS_LOG_DEBUG ("The next look around rate is slower than the maximum throughput rate, continue with the maximum throughput rate: " <<
                                station->m_maxTpRate << "(" << GetSupported (station, station->m_maxTpRate) << ")");
                  rate =  station->m_maxTpRate;
                }
            }
          m_rateChange(rate, station->m_state->m_address);

          /*
           * When Minstrel sampling use reference power.
           * Use the same power for all rates in the retry chain.
           */
          power = station->m_minstrelBluesTable[rate].refPower;
          m_powerChange("ref", power, station->m_state->m_address);
          NS_LOG_DEBUG ("With sample rate use reference power: " << (int) power);
        }

      ///   NO SAMPLING
      /// use best utility rate
      else
        {
          NS_LOG_DEBUG ("Continue using the maximum utility rate: " << station->m_maxURate << "(" << GetSupported (station, station->m_maxURate) << ")");

          if (station->m_bluesSampleCount > 0)
            {
              NS_LOG_DEBUG ("Setup data packet");
              station->m_bluesSampleCount--;
              rate = station->m_maxURate;
              power = station->m_minstrelBluesTable[rate].dataPower;
              m_rateChange(rate, station->m_state->m_address);
              m_powerChange("data", power, station->m_state->m_address);
              NS_LOG_DEBUG("Data packet: rate= " << rate << "(" << GetSupported (station, rate) << ") power= " << (int)power);
            }
          /**
           * if minstrel is not sampling and
           * if we are below the target of look around power percentage, look around
           */
          else
            {
              NS_LOG_DEBUG ("Setup power sampling packet");
              station->m_bluesSampleCount = 100 / m_lookAroundSamplePowerPercentage;

              // choose round robin between the 2 bests
              /// TODO choose between the 4 best rates
              station->m_bluesSampleRate = (station->m_sampleBest? station->m_maxTpRate : station->m_maxTpRate2);
              rate = station->m_bluesSampleRate;
              m_rateChange(rate, station->m_state->m_address);
              station->m_samplingBlues = true;
              station->m_sampleBest = !station->m_sampleBest;

              /// for determining when to try a sample power
              int coinFlip = m_uniformRandomVariable->GetInteger (0, 100) % 2;

              if (coinFlip == 1)
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
    }
  station->m_currentRate = rate;
  station->m_currentPower = power;
}

double
MinstrelBluesWifiManager::BluesUtility (MinstrelBluesWifiRemoteStation *station, uint32_t rate)
{
  NS_LOG_FUNCTION (this);
  uint32_t th = station->m_minstrelBluesTable[rate].throughput;
  uint32_t bestTh = station->m_minstrelBluesTable[station->m_maxTpRate].throughput;
  if (th != 0 && bestTh != 0)
    {
      double benefit = (th / bestTh)*100;
      uint8_t power = station->m_minstrelBluesTable[rate].dataPower;
      uint8_t bestPower = station->m_minstrelBluesTable[station->m_maxTpRate].dataPower;
      double cost = (power/th)*(bestTh/bestPower)*100;
      return m_bluesUilityWeight*benefit-cost;
    }
  else
    {
      return 0;
    }
}

void
MinstrelBluesWifiManager::UpdateStats (MinstrelBluesWifiRemoteStation *station)
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
  NS_LOG_FUNCTION (this);

  station->m_nextStatsUpdate = Simulator::Now () + m_updateStats;
  NS_LOG_DEBUG ("Next update at " << station->m_nextStatsUpdate);
  NS_LOG_DEBUG ("Currently using rate: " << station->m_currentRate << " (" << GetSupported (station, station->m_currentRate) << ")");

  Time txTime;
  uint32_t tempProb;

  NS_LOG_DEBUG ("Index-Rate\t\tAttempt\tSuccess");
  for (uint32_t i = 0; i < station->m_nSupported; i++)
    {

      /// calculate the perfect tx time for this rate
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
           * calculate the probability of success
           * assume probability scales from 0 to 18000
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

      /// bookeeping

      station->m_minstrelBluesTable[i].numRateSuccess = 0;
      station->m_minstrelBluesTable[i].numRateAttempt = 0;

      /// Sample less often below 10% and  above 95% of success
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

      /// if it's 0 allow one retry limit
      if (station->m_minstrelBluesTable[i].adjustedRetryCount == 0)
        {
          station->m_minstrelBluesTable[i].adjustedRetryCount = 1;
        }
    }
  NS_LOG_DEBUG ("Attempt/success resetted to 0");

  uint32_t max_prob = 0, index_max_prob = 0, max_tp = 0, index_max_tp = 0, index_max_tp2 = 0;

  /// go find max throughput, second maximum throughput, high probability succ
  NS_LOG_DEBUG ("Finding the maximum throughput, second maximum throughput, and highest probability");
  NS_LOG_DEBUG ("Index-Rate\t\tT-put\tEWMA");
  for (uint32_t i = 0; i < station->m_nSupported; i++)
    {
      NS_LOG_DEBUG (i << " " << GetSupported (station, i) <<
                    "\t" << station->m_minstrelBluesTable[i].throughput <<
                    "\t" << station->m_minstrelBluesTable[i].ewmaProb);

      if (max_tp < station->m_minstrelBluesTable[i].throughput)
        {
          index_max_tp = i;
          max_tp = station->m_minstrelBluesTable[i].throughput;
        }

      if (max_prob < station->m_minstrelBluesTable[i].ewmaProb)
        {
          index_max_prob = i;
          max_prob = station->m_minstrelBluesTable[i].ewmaProb;
        }
    }


  max_tp = 0;
  /// find the second highest max
  for (uint32_t i = 0; i < station->m_nSupported; i++)
    {
      if ((i != index_max_tp) && (max_tp < station->m_minstrelBluesTable[i].throughput))
        {
          index_max_tp2 = i;
          max_tp = station->m_minstrelBluesTable[i].throughput;
        }
    }

  station->m_maxTpRate = index_max_tp;
  station->m_maxTpRate2 = index_max_tp2;
  station->m_maxProbRate = index_max_prob;
  station->m_currentRate = index_max_tp;

  //obtain the 4 best rates for blues utility function
  uint32_t max_util = 0, index_max_util = 0, index_max_util2 = 0;
  for (uint32_t i = 0; i < station->m_nSupported; i++)
    {
      NS_LOG_DEBUG (i << " " << GetSupported (station, i) <<
                    "\t" << BluesUtility(station, i));

      if (max_util < BluesUtility(station, i))
        {
          index_max_util2 = index_max_util;
          index_max_util = i;
          max_util = BluesUtility(station, i);
        }
    }

  station->m_maxURate = index_max_util;
  station->m_maxURate2 = index_max_util2;

//  if (index_max_tp > station->m_txrate)
//    {
//      station->m_txrate = index_max_tp;
//      m_rateChange(station->m_txrate, station->m_state->m_address);
//    }

  NS_LOG_DEBUG ("max throughput=" << index_max_tp << "(" << GetSupported (station, index_max_tp) <<
                ")\tsecond max throughput=" << index_max_tp2 << "(" << GetSupported (station, index_max_tp2) <<
                ")\tmax prob=" << index_max_prob << "(" << GetSupported (station, index_max_prob) << ")");

}

void
MinstrelBluesWifiManager::UpdatePowerStats (MinstrelBluesWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  if (!station->m_initialized)
    {
      return;
    }
  NS_LOG_FUNCTION (this);

  Time txTime;
  uint32_t refTempProb;
  uint32_t dataTempProb;
  uint32_t sampleTempProb;

  for (uint32_t i = 0; i < station->m_nSupported; i++)
    {

      if (station->m_minstrelBluesTable[i].numSampleAttempt > m_bluesUpdateStatsThreshold && station->m_minstrelBluesTable[i].numRefAttempt > m_bluesUpdateStatsThreshold)
        {
          /**
           * calculate the probability of success
           * assume probability scales from 0 to 18000
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
          station->m_minstrelBluesTable[i].refProb = refTempProb;
          station->m_minstrelBluesTable[i].dataProb = dataTempProb;
          station->m_minstrelBluesTable[i].sampleProb = sampleTempProb;

          /// ewma probability (cast for gcc 3.4 compatibility)
          refTempProb = static_cast<uint32_t> (((refTempProb * (100 - m_ewmaCoefficient)) + (station->m_minstrelBluesTable[i].ewmaRefProb * m_ewmaCoefficient) ) / 100);
          dataTempProb = static_cast<uint32_t> (((dataTempProb * (100 - m_ewmaCoefficient)) + (station->m_minstrelBluesTable[i].ewmaDataProb * m_ewmaCoefficient) ) / 100);
          sampleTempProb = static_cast<uint32_t> (((sampleTempProb * (100 - m_ewmaCoefficient)) + (station->m_minstrelBluesTable[i].ewmaSampleProb * m_ewmaCoefficient) ) / 100);

          station->m_minstrelBluesTable[i].ewmaRefProb = refTempProb;
          station->m_minstrelBluesTable[i].ewmaDataProb = dataTempProb;
          station->m_minstrelBluesTable[i].ewmaSampleProb = sampleTempProb;

          station->m_minstrelBluesTable[i].numRefSuccess = 0;
          station->m_minstrelBluesTable[i].numRefAttempt = 0;
          station->m_minstrelBluesTable[i].numDataSuccess = 0;
          station->m_minstrelBluesTable[i].numDataAttempt = 0;
          station->m_minstrelBluesTable[i].numSampleSuccess = 0;
          station->m_minstrelBluesTable[i].numSampleAttempt = 0;

          //If throughput collapse, reset to initial settings
          if ((station->m_minstrelBluesTable[i].ewmaDataProb < 0.1*18000) || (station->m_minstrelBluesTable[station->m_currentRate].throughput == 0))
          {
        	  //RateInit(station);
        	  NS_LOG_UNCOND("throughput collapse");
          }

          if (station->m_minstrelBluesTable[i].ewmaSampleProb < (station->m_minstrelBluesTable[i].ewmaRefProb - m_thIncPower*18000))
            {
              station->m_minstrelBluesTable[i].samplePower = Min((m_maxPower-3),(station->m_minstrelBluesTable[i].samplePower + m_deltaIncPower));
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
              station->m_minstrelBluesTable[i].refPower = Max(3,(station->m_minstrelBluesTable[i].refPower - m_deltaDecPower));
            }

          station->m_minstrelBluesTable[i].dataPower = station->m_minstrelBluesTable[i].samplePower + m_deltaDataSamplePower;

          station->m_minstrelBluesTable[i].validityTimer = Simulator::Now();
        }

      if (station->m_minstrelBluesTable[i].numDataAttempt > m_bluesUpdateStatsThreshold)
        {
          dataTempProb = (station->m_minstrelBluesTable[i].numDataSuccess * 18000) / station->m_minstrelBluesTable[i].numDataAttempt;
          station->m_minstrelBluesTable[i].dataProb = dataTempProb;

          dataTempProb = static_cast<uint32_t> (((dataTempProb * (100 - m_ewmaCoefficient)) + (station->m_minstrelBluesTable[i].ewmaDataProb * m_ewmaCoefficient) ) / 100);
          station->m_minstrelBluesTable[i].ewmaDataProb = dataTempProb;
          station->m_minstrelBluesTable[i].numDataSuccess = 0;
          station->m_minstrelBluesTable[i].numDataAttempt = 0;
        }

//      if (station->m_minstrelTable[i].validityTimer > Simulator::Now() + Seconds(5))
//        {
//          if (i > station->m_maxTpRate)
//            {
//              station->m_minstrelTable[i].dataPower = m_nPower - 1;
//              station->m_minstrelTable[i].refPower = m_nPower - 1;
//              station->m_minstrelTable[i].samplePower = station->m_minstrelTable[i].refPower - m_delta;
//            }
//          else
//            {
//              station->m_minstrelTable[i].dataPower =  station->m_minstrelTable[station->m_maxTpRate2].dataPower;
//              station->m_minstrelTable[i].refPower =  station->m_minstrelTable[station->m_maxTpRate2].refPower;
//              station->m_minstrelTable[i].samplePower =  station->m_minstrelTable[station->m_maxTpRate2].samplePower;
//            }
//
//        }
    }
}

void
MinstrelBluesWifiManager::RateInit (MinstrelBluesWifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("RateInit=" << station);

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
      station->m_minstrelBluesTable[i].refProb = 0;
      station->m_minstrelBluesTable[i].ewmaRefProb = 0;
      station->m_minstrelBluesTable[i].numDataAttempt = 0;
      station->m_minstrelBluesTable[i].numDataSuccess = 0;
      station->m_minstrelBluesTable[i].dataProb = 0;
      station->m_minstrelBluesTable[i].ewmaDataProb = 0;
      station->m_minstrelBluesTable[i].numSampleAttempt = 0;
      station->m_minstrelBluesTable[i].numSampleSuccess = 0;
      station->m_minstrelBluesTable[i].sampleProb = 0;
      station->m_minstrelBluesTable[i].ewmaSampleProb = 0;
      station->m_minstrelBluesTable[i].dataPower = m_maxPower;
      station->m_minstrelBluesTable[i].refPower = m_maxPower;
      station->m_minstrelBluesTable[i].samplePower = station->m_minstrelBluesTable[i].refPower - 3;
      station->m_minstrelBluesTable[i].validityTimer = Simulator::Now() + Seconds(1);
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

  NS_LOG_UNCOND ("index throughput ewmaProb prob success(attempt) refPower samplePower dataPower ewmaRefProb ewmaSampleProb ewmaDataProb refSuccess(refAttempt) sampleSucces(sampleAttempt) dataSuccess(dataAttempt)\n");
  for (uint32_t i = 0; i < station->m_nSupported; i++)
    {
      NS_LOG_UNCOND( std::setw(2) << i << std::setw(10) << station->m_minstrelBluesTable[i].throughput << std::setw(10) << station->m_minstrelBluesTable[i].ewmaProb << std::setw(10) << station->m_minstrelBluesTable[i].prob
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






