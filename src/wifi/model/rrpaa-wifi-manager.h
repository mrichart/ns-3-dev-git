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
#ifndef RRPAA_WIFI_MANAGER_H
#define RRPAA_WIFI_MANAGER_H

#include "ns3/nstime.h"
#include "wifi-remote-station-manager.h"

namespace ns3 {

struct RrpaaWifiRemoteStation;

/**
 * \ingroup wifi
 * Robust Rate and Power Adaptation Algorithm
 *
 * This class implements the RRPAA algorithm as described in <i>Self Management of Power,
 * Rate and Carrier Sense Threshold for Interference Mitigation in IEEE 802.11 Netwroks</i>
 * by Matías Richart; Jorge Visca and Javier Baliosian in Network and Service Management (CNSM),
 * 2014 10th International Conference on (pp. 264-267). IEEE.
 * http://www.cnsm-conf.org/2014/proceedings/pdf/36.pdf
 *
 * RRPAA adds power control to the RRAA mechanism. RRAA is described in
 * <i>Robust rate adaptation for 802.11 wireless networks</i> by Starsky H. Y. Wong;
 * Hao Yang; Songwu Lu and Vaduvur Bharghavan in Proceedings of the 12th annual
 * international conference on Mobile computing and networking (pp. 146-157). ACM.
 * http://ocw.cs.pub.ro/courses/_media/isrm/articole/rrate_adapt_mobicom06.pdf
 *
 */

/**
 * For each rate there is a Opportunistic Rate Increase threshold,
 * a Maximum Tolerable Loss threshold and an Evaluation Window.
 */
struct Thresholds
{
  double ori;
  double mtl;
  uint32_t ewnd;
};

/**
 * List of thresholds for each mode.
 */
typedef std::vector<std::pair<Thresholds,WifiMode> > RrpaaThresholds;

class RrpaaWifiManager : public WifiRemoteStationManager
{
public:
  /**
   * Register this type.
   * \return The object TypeId.
   */
  static TypeId GetTypeId (void);
  RrpaaWifiManager ();
  virtual ~RrpaaWifiManager ();

  virtual void SetupPhy (Ptr<WifiPhy> phy);
  virtual void SetupMac (Ptr<WifiMac> mac);

  /**
   * TracedCallback signature for power change events.
   *
   * \param [in] power The new power.
   * \param [in] address The remote station MAC address.
   */
  typedef void (* PowerChangeTracedCallback)(const uint8_t power, const Mac48Address remoteAddress);

  /**
   * TracedCallback signature for rate change events.
   *
   * \param [in] rate The new rate.
   * \param [in] address The remote station MAC address.
   */
  typedef void (* RateChangeTracedCallback)(const uint32_t rate, const Mac48Address remoteAddress);

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
  virtual bool DoNeedRts (WifiRemoteStation *st,
                          Ptr<const Packet> packet, bool normally);
  virtual bool IsLowLatency (void) const;

  /**
   * Check for initializations.
   * \param station The remote station.
   */
  void CheckInit (RrpaaWifiRemoteStation *station);

  /**
   * Check if the counter should be resetted.
   *
   * \param station
   */
  void CheckTimeout (RrpaaWifiRemoteStation *station);
  /**
   * Find an appropriate rate and power for the given station, using
   * a basic algorithm.
   *
   * \param station
   */
  void RunBasicAlgorithm (RrpaaWifiRemoteStation *station);
  /**
   * Activate the use of RTS for the given station if the conditions are met.
   *
   * \param station
   */
  void ARts (RrpaaWifiRemoteStation *station);
  /**
   * Reset the counters of the given station.
   *
   * \param station
   */
  void ResetCountersBasic (RrpaaWifiRemoteStation *station);

  /**
   * Initialize the thresholds internal list for the given station.
   *
   * \param station
   */
  void InitThresholds (RrpaaWifiRemoteStation *station);

  /**
   * Get the thresholds for the given station and mode.
   *
   * \param station
   * \param mode
   * \return threshold
   */
  Thresholds GetThresholds(RrpaaWifiRemoteStation *station, WifiMode mode) const;

  /**
   * Get the thresholds for the given station and mode index.
   *
   * \param station
   * \param rate
   * \return threshold
   */
  Thresholds GetThresholds (RrpaaWifiRemoteStation *station, uint32_t rate) const;

  /**
   * Get the estimated TxTime of a packet with a given mode.
   *
   * \param mode
   * \return time
   */
  Time GetCalcTxTime (WifiMode mode) const;
  /**
   * Add transmission time for the given mode to an internal list.
   *
   * \param mode Wi-Fi mode
   * \param t transmission time
   */
  void AddCalcTxTime (WifiMode mode, Time t);

  /**
   * typedef for a vector of a pair of Time, WifiMode.
   * Essentially a list for WifiMode and its corresponding transmission time
   * to transmit a reference packet.
   */
  typedef std::vector<std::pair<Time,WifiMode> > TxTime;

  TxTime m_calcTxTime;  //!< To hold all the calculated TxTime for all modes.
  Time m_sifs; //!< Value of SIFS configured in the device.
  Time m_difs; //!< Value of DIFS configured in the device.

  uint32_t m_frameLength;  //!< Data frame length used for calculate mode TxTime.
  uint32_t m_ackLength;  //!< Ack frame length used for calculate mode TxTime.

  bool m_basic; //!< If using the basic algorithm (without RTS/CTS).
  Time m_timeout; //!< Timeout for the RRAA BASIC loss estimaton block.
  double m_alpha; //!< Alpha value for RRPAA (value for calculating MTL threshold)
  double m_beta; //!< Beta value for RRPAA (value for calculating ORI threshold).
  double m_tau; //!< Tau value for RRPAA (value for calculating EWND size).
  double m_gamma; //!< Gamma value for RRPAA (value for pdTable decrements).
  double m_delta; //!< Delta value for RRPAA (value for pdTable increments).

  /**
   * Differently form rate, power levels do not depend on the remote station.
   * The levels depend only on the physical layer of the device.
   */
  uint32_t m_minPower; //!< Minimal power level.
  uint32_t m_maxPower; //!< Maximal power level.
  uint32_t m_nPower; //!< Number of power levels.

  /**
   * The trace source fired when the transmission power change
   */
  TracedCallback<uint8_t, Mac48Address> m_powerChange;
  /**
   * The trace source fired when the transmission rate change
   */
  TracedCallback<uint32_t, Mac48Address> m_rateChange;
};

} // namespace ns3

#endif /* RRPAA__WIFI_MANAGER_H */
