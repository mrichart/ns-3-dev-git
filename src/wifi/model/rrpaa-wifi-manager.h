/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2014 Universidad de la República - Uruguay
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
 * \brief Robust Rate and Power Adaptation Algorithm
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
 */

struct Thresholds
{
  double ori;
  double mtl;
  uint32_t ewnd;
};

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

  uint32_t GetMaxRate (RrpaaWifiRemoteStation *station);
  uint32_t GetMinRate (RrpaaWifiRemoteStation *station);
  void CheckTimeout (RrpaaWifiRemoteStation *station);
  void RunBasicAlgorithm (RrpaaWifiRemoteStation *station);
  void ARts (RrpaaWifiRemoteStation *station);
  void ResetCountersBasic (RrpaaWifiRemoteStation *station);
  Thresholds GetThresholds (RrpaaWifiRemoteStation *station, uint32_t rate) const;

  /// for estimating the TxTime of a packet with a given mode
  Time GetCalcTxTime (WifiMode mode) const;
  void AddCalcTxTime (WifiMode mode, Time t);  /// for estimating the TxTime of a packet with a given mode

  typedef std::vector<std::pair<Time,WifiMode> > TxTime;

  Thresholds GetThresholds(RrpaaWifiRemoteStation *station, WifiMode mode) const;
  void AddThresholds (RrpaaWifiRemoteStation *station, WifiMode mode, Thresholds th);

  void InitThresholds (RrpaaWifiRemoteStation *station);

  void CheckInit (RrpaaWifiRemoteStation *station);  ///< check for initializations

  TxTime m_calcTxTime;  ///< to hold all the calculated TxTime for all modes
  Time m_sifs;
  Time m_difs;

  uint32_t m_frameLength;  //!< Frame length used  for calculate mode TxTime.
  uint32_t m_ackLength;  //!< Frame length used  for calculate mode TxTime.

  bool m_basic;
  Time m_timeout;
  double m_alpha;
  double m_beta;
  double m_gamma;

  /**
   * Differently form rate, power levels do not depend on the remote station.
   * The levels depend only on the physical layer of the device.
   */
  uint32_t m_minPower; //!< Minimal power level.
  uint32_t m_maxPower; //! Maximal power level.
  uint32_t m_nPower; //! Number of power levels.

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
