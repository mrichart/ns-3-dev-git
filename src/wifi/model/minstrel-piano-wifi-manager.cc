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

//  uint32_t m_shortRetry;  ///< short retries such as control packts
//  uint32_t m_longRetry;  ///< long retries such as data packets
//  uint32_t m_retry;  ///< total retries short + long
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
    .AddAttribute ("LookAroundrefPower",
                   "The percentage to try reference power",
                   DoubleValue (10),
                   MakeDoubleAccessor (&MinstrelPianoWifiManager::m_lookAroundRefPower),
                   MakeDoubleChecker <double> ())
    .AddAttribute ("PianoUpdateStats",
                   "How many reference or sample packets are needed for updating stats",
                   UintegerValue (100),
                   MakeUintegerAccessor (&MinstrelPianoWifiManager::m_pianoUpdateStats),
                   MakeUintegerChecker <uint32_t> ())
    .AddAttribute ("IncrementLevel",
                   "How much levels to increase power each time",
                   UintegerValue (1),
                   MakeUintegerAccessor (&MinstrelPianoWifiManager::m_deltaInc),
                   MakeUintegerChecker <uint8_t> ())
    .AddAttribute ("DecrementLevel",
                   "How much levels to decrease power each time",
                   UintegerValue (2),
                   MakeUintegerAccessor (&MinstrelPianoWifiManager::m_deltaDec),
                   MakeUintegerChecker <uint8_t> ())
    .AddAttribute ("Delta",
                   "Power levels of separation between data and sample power",
                   UintegerValue (1),
                   MakeUintegerAccessor (&MinstrelPianoWifiManager::m_delta),
                   MakeUintegerChecker <uint8_t> ())
    .AddAttribute ("IncrementThreshold",
                   "The threshold between probabilities needed to increase power",
                   DoubleValue (0.1),
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
{
  m_uniformRandomVariable = CreateObject<UniformRandomVariable> ();

  m_nsupported = 0;
}

MinstrelPianoWifiManager::~MinstrelPianoWifiManager ()
{
}

void
MinstrelPianoWifiManager::SetupPhy (Ptr<WifiPhy> phy)
{
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
//  station->m_shortRetry = 0;
//  station->m_longRetry = 0;
//  station->m_retry = 0;
  station->m_err = 0;
  station->m_txrate = 0;
  station->m_initialized = false;
  station->m_pianoRefCount = 0;
  station->m_pianoSampleCount = 100 / m_lookAroundSamplePower;
  station->m_txpower = m_nPower-1;
  m_powerChange(station->m_txpower);
  m_rateChange(station->m_txrate);

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
      m_nsupported = GetNSupported (station);
      station->m_minstrelTable = MinstrelPianoRate (m_nsupported);
      station->m_sampleTable = SampleRate (m_nsupported, std::vector<uint32_t> (m_sampleCol));
      InitSampleTable (station);
      RateInit (station);
      station->m_initialized = true;
    }
}

void
MinstrelPianoWifiManager::DoReportRxOk (WifiRemoteStation *st,
                                   double rxSnr, WifiMode txMode)
{
  NS_LOG_DEBUG ("DoReportRxOk m_txrate=" << ((MinstrelPianoWifiRemoteStation *)st)->m_txrate);
}

void
MinstrelPianoWifiManager::DoReportRtsFailed (WifiRemoteStation *st)
{
  MinstrelPianoWifiRemoteStation *station = (MinstrelPianoWifiRemoteStation *)st;
  NS_LOG_DEBUG ("DoReportRtsFailed m_txrate=" << station->m_txrate);

  station->m_minstrelTable[station->m_txrate].shortRetry++;
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

  station->m_minstrelTable[station->m_txrate].longRetry++;

  //NS_LOG_DEBUG ("DoReportDataFailed " << station << "\t rate " << station->m_txrate << "\tlongRetry \t" << station->m_longRetry);

  /// for normal rate, we're not currently sampling random rates
  if (!station->m_isSampling)
    {
      /// use best throughput rate
      if (station->m_minstrelTable[station->m_txrate].longRetry < station->m_minstrelTable[station->m_txrate].adjustedRetryCount)
        {
          ;  ///<  there's still a few retries left
        }

      /// use second best throughput rate
      else if (station->m_minstrelTable[station->m_txrate].longRetry <= (station->m_minstrelTable[station->m_txrate].adjustedRetryCount +
                                        station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount))
        {
          station->m_txrate = station->m_maxTpRate2;
          m_rateChange(station->m_txrate);
        }

      /// use best probability rate
      else if (station->m_minstrelTable[station->m_txrate].longRetry <= (station->m_minstrelTable[station->m_txrate].adjustedRetryCount +
                                        station->m_minstrelTable[station->m_maxTpRate2].adjustedRetryCount +
                                        station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount))
        {
          station->m_txrate = station->m_maxProbRate;
          m_rateChange(station->m_txrate);
        }

      /// use lowest base rate
      else if (station->m_minstrelTable[station->m_txrate].longRetry > (station->m_minstrelTable[station->m_txrate].adjustedRetryCount +
                                       station->m_minstrelTable[station->m_maxTpRate2].adjustedRetryCount +
                                       station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount))
        {
          station->m_txrate = 0;
          m_rateChange(station->m_txrate);
        }
    }

  /// for look-around rate, we're currently sampling random rates
  else
    {
      /// current sampling rate is slower than the current best rate
      if (station->m_sampleRateSlower)
        {
          /// use best throughput rate
          if (station->m_minstrelTable[station->m_txrate].longRetry < station->m_minstrelTable[station->m_txrate].adjustedRetryCount)
            {
              ; ///<  there are a few retries left
            }

          ///	use random rate
          else if (station->m_minstrelTable[station->m_txrate].longRetry <= (station->m_minstrelTable[station->m_txrate].adjustedRetryCount +
                                            station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount))
            {
              station->m_txrate = station->m_sampleRate;
              m_rateChange(station->m_txrate);
            }

          /// use max probability rate
          else if (station->m_minstrelTable[station->m_txrate].longRetry <= (station->m_minstrelTable[station->m_txrate].adjustedRetryCount +
                                            station->m_minstrelTable[station->m_sampleRate].adjustedRetryCount +
                                            station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount ))
            {
              station->m_txrate = station->m_maxProbRate;
              m_rateChange(station->m_txrate);
            }

          /// use lowest base rate
          else if (station->m_minstrelTable[station->m_txrate].longRetry > (station->m_minstrelTable[station->m_txrate].adjustedRetryCount +
                                           station->m_minstrelTable[station->m_sampleRate].adjustedRetryCount +
                                           station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount))
            {
              station->m_txrate = 0;
              m_rateChange(station->m_txrate);
            }
        }

      /// current sampling rate is better than current best rate
      else
        {
          /// use random rate
          if (station->m_minstrelTable[station->m_txrate].longRetry < station->m_minstrelTable[station->m_txrate].adjustedRetryCount)
            {
              ;    ///< keep using it
            }

          /// use the best rate
          else if (station->m_minstrelTable[station->m_txrate].longRetry <= (station->m_minstrelTable[station->m_txrate].adjustedRetryCount +
                                            station->m_minstrelTable[station->m_sampleRate].adjustedRetryCount))
            {
              station->m_txrate = station->m_maxTpRate;
              m_rateChange(station->m_txrate);
            }

          /// use the best probability rate
          else if (station->m_minstrelTable[station->m_txrate].longRetry <= (station->m_minstrelTable[station->m_txrate].adjustedRetryCount +
                                            station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount +
                                            station->m_minstrelTable[station->m_sampleRate].adjustedRetryCount))
            {
              station->m_txrate = station->m_maxProbRate;
              m_rateChange(station->m_txrate);
            }

          /// use the lowest base rate
          else if (station->m_minstrelTable[station->m_txrate].longRetry > (station->m_minstrelTable[station->m_txrate].adjustedRetryCount +
                                           station->m_minstrelTable[station->m_maxTpRate].adjustedRetryCount +
                                           station->m_minstrelTable[station->m_sampleRate].adjustedRetryCount))
            {
              station->m_txrate = 0;
              m_rateChange(station->m_txrate);
            }
        }
    }
}

void
MinstrelPianoWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                     double ackSnr, WifiMode ackMode, double dataSnr)
{
  MinstrelPianoWifiRemoteStation *station = (MinstrelPianoWifiRemoteStation *) st;

  station->m_isSampling = false;
  station->m_sampleRateSlower = false;

  CheckInit (station);
  if (!station->m_initialized)
    {
      return;
    }

  UpdateRetry (station); //updates m_retry and set long and short retry to 0
  UpdateAttempts (station); //use m_retry to update attemps for each rate

  station->m_minstrelTable[station->m_txrate].numRateSuccess++;
  station->m_minstrelTable[station->m_txrate].numRateAttempt++;

  if (station->m_txpower == station->m_minstrelTable[station->m_txrate].refPower)
    {
  	  station->m_minstrelTable[station->m_txrate].numRefSuccess++;
  	  station->m_minstrelTable[station->m_txrate].numRefAttempt++;
    }
  else if (station->m_txpower == station->m_minstrelTable[station->m_txrate].dataPower)
    {
  	  station->m_minstrelTable[station->m_txrate].numDataSuccess++;
  	  station->m_minstrelTable[station->m_txrate].numDataAttempt++;
    }
  else if (station->m_txpower == station->m_minstrelTable[station->m_txrate].samplePower)
    {
  	  station->m_minstrelTable[station->m_txrate].numSampleSuccess++;
  	  station->m_minstrelTable[station->m_txrate].numSampleAttempt++;
    }

  station->m_packetCount++;

  if (m_nsupported >= 1)
    {
      /*station->m_txrate = FindRate (station);//returns a sample rate or the best rate
      station->m_txpower = FindPower(station);// retruns the power for the rate selected*/
	  SetRatePower(station);
      m_rateChange(station->m_txrate);
      m_powerChange(station->m_txpower);
    }
}

//this function is used to say that all retry attempts fail

void
MinstrelPianoWifiManager::DoReportFinalDataFailed (WifiRemoteStation *st)
{
  MinstrelPianoWifiRemoteStation *station = (MinstrelPianoWifiRemoteStation *) st;
  NS_LOG_DEBUG ("DoReportFinalDataFailed m_txrate=" << station->m_txrate);

  station->m_isSampling = false;
  station->m_sampleRateSlower = false;

  UpdateRetry (station);
  UpdateAttempts (station);
  station->m_err++;

  if (m_nsupported >= 1)
    {
/*      station->m_txrate = FindRate (station);
      station->m_txpower = FindPower(station);*/
	  SetRatePower(station);
      m_rateChange(station->m_txrate);
      m_powerChange(station->m_txpower);
    }
}

void
MinstrelPianoWifiManager::UpdateRetry (MinstrelPianoWifiRemoteStation *station)
{
  for (uint32_t i = 0; i < m_nsupported; i++)
    {
      station->m_minstrelTable[i].retry = station->m_minstrelTable[i].shortRetry + station->m_minstrelTable[i].longRetry;
      station->m_minstrelTable[i].shortRetry = 0;
      station->m_minstrelTable[i].longRetry = 0;
    }
}

void
MinstrelPianoWifiManager::UpdateAttempts (MinstrelPianoWifiRemoteStation *station)
{
  for (uint32_t i = 0; i < m_nsupported; i++)
    {
      station->m_minstrelTable[i].numRateAttempt += station->m_minstrelTable[i].retry;
      if (station->m_txpower == station->m_minstrelTable[i].refPower)
          {
                station->m_minstrelTable[i].numRefAttempt += station->m_minstrelTable[i].retry; //this is not in the paper but i think its neeeded
          }
        else if (station->m_txpower == station->m_minstrelTable[i].dataPower)
          {
                station->m_minstrelTable[i].numDataAttempt += station->m_minstrelTable[i].retry;
          }
        else if (station->m_txpower == station->m_minstrelTable[i].samplePower)
          {
                station->m_minstrelTable[i].numSampleAttempt += station->m_minstrelTable[i].retry;
          }
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
      station->m_txrate = m_nsupported / 2;
      m_rateChange(station->m_txrate);
    }
  UpdateStats (station);
  UpdatePowerStats (station);
  WifiTxVector vector =  WifiTxVector (GetSupported (station, station->m_txrate), station->m_txpower, GetLongRetryCount (station), GetShortGuardInterval (station), Min (GetNoOfRx (station),GetNoOfTransmitters()), GetNoOfTx (station), GetStbc (station));
  m_getDataTxVector(vector);
  return vector;
}

WifiTxVector
MinstrelPianoWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  MinstrelPianoWifiRemoteStation *station = (MinstrelPianoWifiRemoteStation *) st;
  NS_LOG_DEBUG ("DoGetRtsMode m_txrate=" << station->m_txrate);

  return WifiTxVector (GetSupported (station, 0), GetDefaultTxPowerLevel (), GetShortRetryCount (station), GetShortGuardInterval (station), Min (GetNoOfRx (station),GetNoOfTransmitters()), GetNoOfTx (station), GetStbc (station));
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
  if (station->m_index > (m_nsupported - 2))
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

uint32_t
MinstrelPianoWifiManager::FindRate (MinstrelPianoWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("FindRate " << "packet=" << station->m_packetCount );

  if ((station->m_sampleCount + station->m_packetCount) == 0)
    {
      return 0;
    }


  uint32_t idx;

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

      /// now go through the table and find an index rate
      idx = GetNextSample (station);


      /**
       * This if condition is used to make sure that we don't need to use
       * the sample rate it is the same as our current rate
       */
      if (idx != station->m_maxTpRate && idx != station->m_txrate)
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
          if (idx >= m_nsupported)
            {
              NS_LOG_DEBUG ("ALERT!!! ERROR");
            }

          /// set the rate that we're currently sampling
          station->m_sampleRate = idx;

          // we make the random not to be the best throughput
          if (station->m_sampleRate == station->m_maxTpRate)
            {
              station->m_sampleRate = station->m_maxTpRate2;
            }

          /// is this rate slower than the current best rate
          station->m_sampleRateSlower =
            (station->m_minstrelTable[idx].perfectTxTime > station->m_minstrelTable[station->m_maxTpRate].perfectTxTime);

          /// using the best rate instead
          if (station->m_sampleRateSlower)
            {
              idx =  station->m_maxTpRate;
            }
        }

    }

  /*
   * Piano does not interfere with minstrel sampling.
   * Since the power level used for sampling depends on the rate, in Piano,
   * we alternate between the best and 2nd best throughput rates.
   */
  else
    {
      idx = (station->m_sampleBest? station->m_maxTpRate : station->m_maxTpRate2);
      station->m_sampleBest = !station->m_sampleBest;
    }


  NS_LOG_DEBUG ("FindRate " << "sample rate=" << idx);

  return idx;
}

uint8_t
MinstrelPianoWifiManager::FindPower (MinstrelPianoWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("FindPower " << "packet=" << station->m_packetCount );

  if ((station->m_pianoSampleCount + station->m_pianoRefCount + station->m_packetCount) == 0)
    {
	  return m_nPower/2;
    }


  /*
   * Piano does not interfere with minstrel sampling.
   */
  if (station->m_isSampling)
    {
	  return station->m_minstrelTable[station->m_txrate].refPower;
    }

  /// for determining when to try a sample power
  int coinFlip = m_uniformRandomVariable->GetInteger (0, 100) % 2;

  /**
   * if minstrel is not sampling and
   * if we are below the target of look around power percentage, look around
   * note: do it randomly by flipping a coin instead sampling
   * all at once until it reaches the look around power
   */
  if (coinFlip == 1)
    {
	  coinFlip = m_uniformRandomVariable->GetInteger (0, 100) % 2;
	  if ( (((100 * station->m_pianoSampleCount) / (station->m_pianoSampleCount + station->m_packetCount )) < m_lookAroundSamplePower) && (coinFlip == 1) )
	    {
		  /// start sample count
		  station->m_pianoSampleCount++;

		  /// bookeeping for resetting stuff
		  if (station->m_packetCount >= 10000)
			{
			  station->m_sampleCount = 0;
			  station->m_packetCount = 0;
			  station->m_pianoSampleCount = 0;
			  station->m_pianoRefCount = 0;
			}

		  return station->m_minstrelTable[station->m_txrate].samplePower;
	    }
	  else if (((100 * station->m_pianoRefCount) / (station->m_pianoRefCount + station->m_packetCount )) < m_lookAroundRefPower)
	    {
		  /// start sample count
		  station->m_pianoRefCount++;

		  /// bookeeping for resetting stuff
		  if (station->m_packetCount >= 10000)
			{
			  station->m_sampleCount = 0;
			  station->m_packetCount = 0;
			  station->m_pianoSampleCount = 0;
			  station->m_pianoRefCount = 0;
			}

		  return station->m_minstrelTable[station->m_txrate].refPower;
	    }
    }

    return station->m_minstrelTable[station->m_txrate].dataPower;
}

void
MinstrelPianoWifiManager::SetRatePower (MinstrelPianoWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("SetRatePower " << "packet=" << station->m_packetCount );

  uint32_t rate;
  uint8_t power;

  if ((station->m_sampleCount + station->m_packetCount) == 0)
    {
      rate = 0;
    }

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
          if (rate >= m_nsupported)
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
              rate =  station->m_maxTpRate;
            }
        }

      /* When Minstrel sampling use reference power*/
      power = station->m_minstrelTable[rate].refPower;

    }
  else
    {
	  if (station->m_pianoSampleCount > 0)
	    {
		  station->m_pianoSampleCount--;
	      rate = station->m_maxTpRate;
	      power = station->m_minstrelTable[rate].dataPower;
	    }
	  /**
	   * if minstrel is not sampling and
	   * if we are below the target of look around power percentage, look around
	   */
	  else
	    {
		  station->m_pianoSampleCount = 100 / m_lookAroundSamplePower;

		  rate = (station->m_sampleBest? station->m_maxTpRate : station->m_maxTpRate2);
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
  NS_LOG_DEBUG ("Updating stats=" << this);

  station->m_nextStatsUpdate = Simulator::Now () + m_updateStats;

  Time txTime;
  uint32_t tempProb;

  for (uint32_t i = 0; i < m_nsupported; i++)
    {

      /// calculate the perfect tx time for this rate
      txTime = station->m_minstrelTable[i].perfectTxTime;

      /// just for initialization
      if (txTime.GetMicroSeconds () == 0)
        {
          txTime = Seconds (1);
        }

      NS_LOG_DEBUG ("m_txrate=" << station->m_txrate <<
                    "\t attempt=" << station->m_minstrelTable[i].numRateAttempt <<
                    "\t success=" << station->m_minstrelTable[i].numRateSuccess);

      /// if we've attempted something
      if (station->m_minstrelTable[i].numRateAttempt)
        {
          /**
           * calculate the probability of success
           * assume probability scales from 0 to 18000
           */
          tempProb = (station->m_minstrelTable[i].numRateSuccess * 18000) / station->m_minstrelTable[i].numRateAttempt;

          /// bookeeping
          station->m_minstrelTable[i].successHist += station->m_minstrelTable[i].numRateSuccess;
          station->m_minstrelTable[i].attemptHist += station->m_minstrelTable[i].numRateAttempt;
          station->m_minstrelTable[i].prob = tempProb;

          /// ewma probability (cast for gcc 3.4 compatibility)
          tempProb = static_cast<uint32_t> (((tempProb * (100 - m_ewmaLevel)) + (station->m_minstrelTable[i].ewmaProb * m_ewmaLevel) ) / 100);

          station->m_minstrelTable[i].ewmaProb = tempProb;

          /// calculating throughput
          station->m_minstrelTable[i].throughput = tempProb * (1000000 / txTime.GetMicroSeconds ());

        }

      /// bookeeping
      station->m_minstrelTable[i].prevNumRateAttempt = station->m_minstrelTable[i].numRateAttempt;
      station->m_minstrelTable[i].prevNumRateSuccess = station->m_minstrelTable[i].numRateSuccess;
      station->m_minstrelTable[i].numRateSuccess = 0;
      station->m_minstrelTable[i].numRateAttempt = 0;

      /// Sample less often below 10% and  above 95% of success
      if ((station->m_minstrelTable[i].ewmaProb > 17100) || (station->m_minstrelTable[i].ewmaProb < 1800))
        {
          /**
           * retry count denotes the number of retries permitted for each rate
           * # retry_count/2
           */
          station->m_minstrelTable[i].adjustedRetryCount = station->m_minstrelTable[i].retryCount >> 1;
          if (station->m_minstrelTable[i].adjustedRetryCount > 2)
            {
              station->m_minstrelTable[i].adjustedRetryCount = 2;
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


  uint32_t max_prob = 0, index_max_prob = 0, max_tp = 0, index_max_tp = 0, index_max_tp2 = 0;

  /// go find max throughput, second maximum throughput, high probability succ
  for (uint32_t i = 0; i < m_nsupported; i++)
    {
      NS_LOG_DEBUG ("throughput" << station->m_minstrelTable[i].throughput <<
                    "\n ewma" << station->m_minstrelTable[i].ewmaProb);

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
  station->m_currentRate = index_max_tp;

  if (index_max_tp > station->m_txrate)
    {
      station->m_txrate = index_max_tp;
      m_rateChange(station->m_txrate);
    }

  NS_LOG_DEBUG ("max tp=" << index_max_tp << "\nmax tp2=" << index_max_tp2 << "\nmax prob=" << index_max_prob);
}

void
MinstrelPianoWifiManager::UpdatePowerStats (MinstrelPianoWifiRemoteStation *station)
{
//  if ( (station->m_minstrelTable[station->m_txrate].numSampleAttempt <= m_min_update) && (station->m_minstrelTable[station->m_txrate].numRefAttempt <= m_min_update))
//    {
//      return;
//    }

  if (!station->m_initialized)
    {
      return;
    }
  NS_LOG_DEBUG ("Updating power stats=" << this);

  Time txTime;
  uint32_t refTempProb;
  uint32_t dataTempProb;
  uint32_t sampleTempProb;

  for (uint32_t i = 0; i < m_nsupported; i++)
    {

      NS_LOG_DEBUG ("m_txrate=" << station->m_txrate <<
                    "\t attempt=" << station->m_minstrelTable[i].numRateAttempt <<
                    "\t success=" << station->m_minstrelTable[i].numRateSuccess);

      /// if we've attempted something
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
          station->m_minstrelTable[i].successRefHist += station->m_minstrelTable[i].numRefSuccess;
          station->m_minstrelTable[i].attemptRefHist += station->m_minstrelTable[i].numRefAttempt;
          station->m_minstrelTable[i].refProb = refTempProb;
          station->m_minstrelTable[i].successDataHist += station->m_minstrelTable[i].numDataSuccess;
          station->m_minstrelTable[i].attemptDataHist += station->m_minstrelTable[i].numDataAttempt;
          station->m_minstrelTable[i].dataProb = dataTempProb;
          station->m_minstrelTable[i].successSampleHist += station->m_minstrelTable[i].numSampleSuccess;
          station->m_minstrelTable[i].attemptSampleHist += station->m_minstrelTable[i].numSampleAttempt;
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
        }

      if (station->m_minstrelTable[i].numDataAttempt > m_pianoUpdateStats)
        {
          station->m_minstrelTable[i].successSampleHist += station->m_minstrelTable[i].numSampleSuccess;
          station->m_minstrelTable[i].attemptSampleHist += station->m_minstrelTable[i].numSampleAttempt;
          station->m_minstrelTable[i].sampleProb = sampleTempProb;

          sampleTempProb = static_cast<uint32_t> (((sampleTempProb * (100 - m_ewmaLevel)) + (station->m_minstrelTable[i].ewmaSampleProb * m_ewmaLevel) ) / 100);
          station->m_minstrelTable[i].ewmaSampleProb = sampleTempProb;
		  station->m_minstrelTable[i].numDataSuccess = 0;
		  station->m_minstrelTable[i].numDataAttempt = 0;
        }
    }
}

void
MinstrelPianoWifiManager::RateInit (MinstrelPianoWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("RateInit=" << station);

  for (uint32_t i = 0; i < m_nsupported; i++)
    {
      station->m_minstrelTable[i].numRateAttempt = 0;
      station->m_minstrelTable[i].numRateSuccess = 0;
      station->m_minstrelTable[i].longRetry = 0;
      station->m_minstrelTable[i].shortRetry = 0;
      station->m_minstrelTable[i].retry = 0;
      station->m_minstrelTable[i].prob = 0;
      station->m_minstrelTable[i].ewmaProb = 0;
      station->m_minstrelTable[i].prevNumRateAttempt = 0;
      station->m_minstrelTable[i].prevNumRateSuccess = 0;
      station->m_minstrelTable[i].successHist = 0;
      station->m_minstrelTable[i].attemptHist = 0;
      station->m_minstrelTable[i].throughput = 0;
      station->m_minstrelTable[i].perfectTxTime = GetCalcTxTime (GetSupported (station, i));
      station->m_minstrelTable[i].retryCount = 1;
      station->m_minstrelTable[i].adjustedRetryCount = 1;
      /*piano init*/
      station->m_minstrelTable[i].numRefAttempt = 0;
      station->m_minstrelTable[i].numRefSuccess = 0;
      station->m_minstrelTable[i].refProb = 0;
      station->m_minstrelTable[i].ewmaRefProb = 0;
      station->m_minstrelTable[i].prevNumRefAttempt = 0;
      station->m_minstrelTable[i].prevNumRefSuccess = 0;
      station->m_minstrelTable[i].successRefHist = 0;
      station->m_minstrelTable[i].attemptRefHist = 0;
      station->m_minstrelTable[i].numDataAttempt = 0;
      station->m_minstrelTable[i].numDataSuccess = 0;
      station->m_minstrelTable[i].dataProb = 0;
      station->m_minstrelTable[i].ewmaDataProb = 0;
      station->m_minstrelTable[i].prevNumDataAttempt = 0;
      station->m_minstrelTable[i].prevNumDataSuccess = 0;
      station->m_minstrelTable[i].successDataHist = 0;
      station->m_minstrelTable[i].attemptDataHist = 0;
      station->m_minstrelTable[i].numSampleAttempt = 0;
      station->m_minstrelTable[i].numSampleSuccess = 0;
      station->m_minstrelTable[i].sampleProb = 0;
      station->m_minstrelTable[i].ewmaSampleProb = 0;
      station->m_minstrelTable[i].prevNumSampleAttempt = 0;
      station->m_minstrelTable[i].prevNumSampleSuccess = 0;
      station->m_minstrelTable[i].successSampleHist = 0;
      station->m_minstrelTable[i].attemptSampleHist = 0;
      station->m_minstrelTable[i].dataPower = m_nPower - 1;
      station->m_minstrelTable[i].refPower = m_nPower - 1;
      station->m_minstrelTable[i].samplePower = station->m_minstrelTable[i].refPower - m_delta;
    }
}

void
MinstrelPianoWifiManager::InitSampleTable (MinstrelPianoWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("InitSampleTable=" << this);

  station->m_col = station->m_index = 0;

  /// for off-seting to make rates fall between 0 and numrates
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

          /// this loop is used for filling in other uninitilized places
          while (station->m_sampleTable[newIndex][col] != 0)
            {
              newIndex = (newIndex + 1) % m_nsupported;
            }
          station->m_sampleTable[newIndex][col] = i;

        }
    }
}

void
MinstrelPianoWifiManager::PrintSampleTable (MinstrelPianoWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("PrintSampleTable=" << station);

  uint32_t numSampleRates = m_nsupported;
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
MinstrelPianoWifiManager::PrintTable (MinstrelPianoWifiRemoteStation *station)
{
  NS_LOG_DEBUG ("PrintTable=" << station);

  for (uint32_t i = 0; i < m_nsupported; i++)
    {
      std::cout << "index(" << i << ") = " << station->m_minstrelTable[i].perfectTxTime << "\n";
    }
}

} // namespace ns3





