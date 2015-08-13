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
 * Author: Matias Richart <mrichart@fing.edu.uy>
 */
#ifndef MINSTREL_BLUES_WIFI_MANAGER_H
#define MINSTREL_BLUES_WIFI_MANAGER_H

#include "wifi-remote-station-manager.h"
#include "wifi-mode.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"

namespace ns3 {

struct MinstrelBluesWifiRemoteStation;

/**
 * A struct to contain all information related to a data rate.
 * For each rate statistics information is saved and three power options.
 */
struct BluesRatePowerInfo
{
  /**
   * Perfect transmission time calculation, or frame calculation
   * given a bit rate and a packet length in bytes.
   */
  Time perfectTxTime;

  uint32_t retryCount;  //!< Retry limit.
  uint32_t adjustedRetryCount;  //!< Adjust the retry limit for this rate.
  uint32_t numRateAttempt;  //!< Number of transmission attempts so far.
  uint32_t numRateSuccess;    //!< Number of successful frames transmitted so far.
  uint32_t prob;  //!< (# frames success )/(# total frames)
  //uint32_t longRetry; //! Number of retransmissions to try.
  //uint32_t shortRetry; //! Number of retransmissions to try.
  //uint32_t retry;

  /**
   * EWMA calculation
   * ewma_prob =[prob *(100 - ewma_level) + (ewma_prob_old * ewma_level)]/100
   */
  uint32_t ewmaProb; //! Averaged probability to try other rates.

  uint32_t prevNumRateAttempt;  ///< from last rate
  uint32_t prevNumRateSuccess;  ///< from last rate
  uint64_t successHist;  ///< aggregate of all successes
  uint64_t attemptHist;  ///< aggregate of all attempts
  uint32_t throughput;  ///< throughput of a rate

  /*
  * Information for reference packets
  */
  uint32_t numRefAttempt;  ///< how many number of attempts so far
  uint32_t numRefSuccess;    ///< number of successful pkts
  //uint32_t refProb;  ///< (# pkts success )/(# total pkts)

  uint32_t ewmaRefProb;

  uint32_t prevNumRefAttempt;  ///< from last rate
  uint32_t prevNumRefSuccess;  ///< from last rate

  /*
  * Information for data packets
  */
  uint32_t numDataAttempt;  ///< how many number of attempts so far
  uint32_t numDataSuccess;    ///< number of successful pkts
  //uint32_t dataProb;  ///< (# pkts success )/(# total pkts)

  uint32_t ewmaDataProb;

  uint32_t prevNumDataAttempt;  ///< from last rate
  uint32_t prevNumDataSuccess;  ///< from last rate

  /*
  * Information for sample packets
  */
  uint32_t numSampleAttempt;  ///< how many number of attempts so far
  uint32_t numSampleSuccess;    ///< number of successful pkts
  //uint32_t sampleProb;  ///< (# pkts success )/(# total pkts)

  uint32_t ewmaSampleProb;

  uint32_t prevNumSampleAttempt;  ///< from last rate
  uint32_t prevNumSampleSuccess;  ///< from last rate

  /*
  * Powers for this rate
  */

  uint8_t refPower;
  uint8_t dataPower;
  uint8_t samplePower;

  Time validityTimer;
};

const uint32_t N_MAX_TH_RATES = 4; //!< Number of highest throughput rates to consider.

const uint32_t IEEE80211_TX_RATE_TABLE_SIZE = 4; //!< Maximum number of rate table entries.

/**
 * A struct to contain all the rate and associated retry count.
 */
struct MRRChainElement {
        uint32_t rate;
        uint8_t power;
        uint32_t count;
};

/**
 * Data structure for a Minstrel Rate table
 * A vector of a struct RateInfo
 */
typedef std::vector<struct BluesRatePowerInfo> MinstrelBluesRate;

/**
 * Data structure for a Sample Rate table
 * A vector of a vector uint32_t
 */
typedef std::vector<std::vector<uint32_t> > SampleRate;

/**
 * \ingroup wifi
 * Minstrel-Blues Power and rate control algorithm
 *
 * Implementation of Minstrel Blues Rate and Power Control Algorithm
 * based on implementation of Duy Nguyen of Minstrel Rate Control Algorithm.
 * Minstrel-Blues is described in the Phd. Thesis <i>A Measurement-Based Joint
 * Power and Rate Controller for IEEE 802.11 Networks</i> by Thomas Huehn, 2013.
 */
class MinstrelBluesWifiManager : public WifiRemoteStationManager
{

public:
  static TypeId GetTypeId (void);
  MinstrelBluesWifiManager ();
  virtual ~MinstrelBluesWifiManager ();

  virtual void SetupPhy (Ptr<WifiPhy> phy);

 /**
  * Assign a fixed random variable stream number to the random variables
  * used by this model.  Return the number of streams (possibly zero) that
  * have been assigned.
  *
  * \param stream first stream index to use
  * \return the number of stream indices assigned by this model
  */
  int64_t AssignStreams (int64_t stream);

  /**
   * TracedCallback signature for power change events.
   *
   * \param [in] power The new power.
   * \param [in] address The remote station MAC address.
   */
  typedef void (*PowerChangeTracedCallback)(const std::string, const uint8_t power, const Mac48Address remoteAddress);

  /**
   * TracedCallback signature for rate change events.
   *
   * \param [in] rate The new rate.
   * \param [in] address The remote station MAC address.
   */
  typedef void (*RateChangeTracedCallback)(const std::string, const uint32_t rate, const Mac48Address remoteAddress);

private:
  // overriden from base class
  virtual WifiRemoteStation * DoCreateStation (void) const;
  virtual void DoReportRxOk (WifiRemoteStation *station,
                             double rxSnr, WifiMode txMode);
  virtual void DoReportRtsFailed (WifiRemoteStation *station);
  virtual void DoReportDataFailed (WifiRemoteStation *station);
  virtual void DoReportRtsOk (WifiRemoteStation *station,
                              double ctsSnr, WifiMode ctsMode, double rtsSnr);
  virtual void DoReportDataOk (WifiRemoteStation *station,
                               double ackSnr, WifiMode ackMode, double dataSnr);
  virtual void DoReportFinalRtsFailed (WifiRemoteStation *station);
  virtual void DoReportFinalDataFailed (WifiRemoteStation *station);
 virtual WifiTxVector DoGetDataTxVector (WifiRemoteStation *station, uint32_t size);
  virtual WifiTxVector DoGetRtsTxVector (WifiRemoteStation *station);
  virtual bool DoNeedDataRetransmission (WifiRemoteStation *st, Ptr<const Packet> packet, bool normally);
  virtual bool IsLowLatency (void) const;

  void SetupPhyMPListener (Ptr<WifiPhy> phy);

  /// for estimating the TxTime of a packet with a given mode
  Time GetCalcTxTime (WifiMode mode) const;
  void AddCalcTxTime (WifiMode mode, Time t);

  /// update the number of retries and reset accordingly
  void UpdateRetry (MinstrelBluesWifiRemoteStation *station);

  void UpdateAttempts (MinstrelBluesWifiRemoteStation *station);

  /// getting the next sample from Sample Table
  uint32_t GetNextSample (MinstrelBluesWifiRemoteStation *station);

  /// find a rate to use from Minstrel Piano Table
  uint32_t FindRate (MinstrelBluesWifiRemoteStation *station);

  /// find a power to use from Minstrel Piano Table
  uint8_t FindPower (MinstrelBluesWifiRemoteStation *station);

  void SetRatePower (MinstrelBluesWifiRemoteStation *station);

  double BluesUtility (MinstrelBluesWifiRemoteStation *station, uint32_t rate);

  /// Updates the Minstrel Table every 1/10 seconds and sort the rates.
  void MinstrelUpdateStats (MinstrelBluesWifiRemoteStation *station);

  /// Sort the list of highest throughput rates.
  void MinstrelSortBestThRates (MinstrelBluesWifiRemoteStation *station, uint32_t i);

  /// updating the Minstrel Piano Table every 10 packets
  void BluesUpdateStats (MinstrelBluesWifiRemoteStation *station);

  /// initialize Minstrel Table
  void RateInit (MinstrelBluesWifiRemoteStation *station);
  void Reset (MinstrelBluesWifiRemoteStation *station, uint32_t rate);

  /**
   * Estimate the time to transmit the given packet with the given number of retries.
   * This function is "roughly" the function "calc_usecs_unicast_packet" in minstrel.c
   * in the madwifi implementation.
   *
   * The basic idea is that, we try to estimate the "average" time used to transmit the
   * packet for the given number of retries while also accounting for the 802.11 congestion
   * window change. The original code in the madwifi seems to estimate the number of backoff
   * slots as the half of the current CW size.
   *
   * There are four main parts:
   *  - wait for DIFS (sense idle channel)
   *  - ACK timeouts
   *  - DATA transmission
   *  - backoffs according to CW
   */
  Time CalculateTimeUnicastPacket (Time dataTransmissionTime, uint32_t shortRetries, uint32_t longRetries);

  /// initialize Sample Table
  void InitSampleTable (MinstrelBluesWifiRemoteStation *station);

  /// printing Sample Table
  void PrintSampleTable (MinstrelBluesWifiRemoteStation *station);

  /// printing Minstrel Table
  void PrintTable (MinstrelBluesWifiRemoteStation *station);

  void CheckInit (MinstrelBluesWifiRemoteStation *station);  ///< check for initializations


  typedef std::vector<std::pair<Time,WifiMode> > TxTime;

  TxTime m_calcTxTime;  ///< to hold all the calculated TxTime for all modes

  Time m_updateStats;  //!< How frequent do we calculate the statistics.
  double m_minstrelSamplingRatio;  //!< The ratio to try other rates than our current rate.
  double m_ewmaCoefficient;  //!< Exponential weighted moving average coefficient.

  uint32_t m_nSampleColumns;  //!< Number of sample columns.
  uint32_t m_frameLength;  //!< Frame length used  for calculate mode TxTime.

  double m_bluesSamplingRatio; //!< The ratio to sample power.

  uint32_t m_bluesUpdateStatsThreshold; //!< Minimal number of packets needed for update blues statistics.
  uint8_t m_deltaIncPower; //!< How many levels to increase power.
  uint8_t m_deltaDecPower; //!< How many levels to decrease power.
  uint8_t m_deltaDataSamplePower; //!< How many levels of separation between data and sample powers.
  uint8_t m_bluesPowerStep; //!< Minimum separation between sample and reference power.

  /**
   * Differently form rate, power levels do not depend on the remote station.
   * The levels depend only on the physical layer of the device.
   */
  double m_minPower; //!< Minimal power in dBm.
  double m_maxPower; //! Maximal power in dBm.
  uint32_t m_nPower; //! Number of power levels.
  uint8_t m_maxPowerLevel; //! Maximal power level.


  double m_thIncPower; //!< Threshold for increasing power.
  double m_thDecPower; //!< Threshold for decreasing power.
  double m_thEmergency; //!< Threshold for determining a throughput collapse.

  double m_bluesUilityWeight;

  bool m_perStagePower; //!< If true, use different power levels for each rate-retry chain stage.

  bool m_fixedRate;     //!< If true, use different the same rate for all data frames (the algorithm will only do power control).
  WifiMode m_dataMode;  //!< The rate to use when fixed rate is enabled.

  Ptr<UniformRandomVariable> m_uniformRandomVariable; //!< Provides uniform random variables.

  /**
   * The trace source fired when the transmission power change.
   */
  TracedCallback<std::string, uint8_t, Mac48Address> m_powerChange;
  /**
   * The trace source fired when the transmission rate change.
   */
  TracedCallback<std::string, uint32_t, Mac48Address> m_rateChange;
};

} // namespace ns3

#endif /* MINSTREL_BLUES_WIFI_MANAGER_H */
