/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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

#ifndef PRCS_WIFI_MANAGER_H
#define PRCS_WIFI_MANAGER_H

#include "ns3/nstime.h"
#include "wifi-remote-station-manager.h"

namespace ns3 {

struct PrcsWifiRemoteStation;

/**
 * \brief Power Rate and Carrier Sense control
 * \ingroup wifi
 *
 * This is an implementation of PRCS as described in
 * "Self Management of Power, Rate and Carrier Sense Threshold
 * for Interference Mitigation in IEEE 802.11 Netwroks"
 * by "Matías Richart", "Jorge Visca", and,
 * "Javier Baliosian" presented at CNSM 2014.
 */

struct Thresholds
{
  double ori;
  double mtl;
  uint32_t ewnd;
};

typedef std::vector<std::pair<Thresholds,WifiMode> > PrcsThresholds;

class PrcsWifiManager : public WifiRemoteStationManager
{
public:
  static TypeId GetTypeId (void);

  PrcsWifiManager ();
  virtual ~PrcsWifiManager ();

  virtual void SetupPhy (Ptr<WifiPhy> phy);

  void NotifyMaybeCcaBusyStartNow (Time duration);

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

  virtual void DoReportTxInit (WifiRemoteStation *st);
  void SetupPhyPrcsListener (Ptr<WifiPhy> phy);

  uint32_t GetMaxRate (PrcsWifiRemoteStation *station);
  uint32_t GetMinRate (PrcsWifiRemoteStation *station);
  void CheckTimeout (PrcsWifiRemoteStation *station);
  void RunBasicAlgorithm (PrcsWifiRemoteStation *station);
  void ARts (PrcsWifiRemoteStation *station);
  void ResetCountersBasic (PrcsWifiRemoteStation *station);
  Thresholds GetThresholds (PrcsWifiRemoteStation *station, uint32_t rate) const;

  /// for estimating the TxTime of a packet with a given mode
  Time GetCalcTxTime (WifiMode mode) const;
  void AddCalcTxTime (WifiMode mode, Time t);  /// for estimating the TxTime of a packet with a given mode

  typedef std::vector<std::pair<Time,WifiMode> > TxTime;

  Thresholds GetThresholds(PrcsWifiRemoteStation *station, WifiMode mode) const;
  void AddThresholds (PrcsWifiRemoteStation *station, WifiMode mode, Thresholds th);

  void InitThresholds (PrcsWifiRemoteStation *station);

  void CheckInit (PrcsWifiRemoteStation *station);  ///< check for initializations

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

  double m_countBusy;
  Time m_prevTime;
  double m_maxCst;

  Ptr<WifiPhy> m_phy;

  // Listerner needed to monitor when a channel switching occurs.
  class PhyPrcsListener * m_phyPrcsListener;
  /**
   * The trace source fired when the transmission rate change
   */
  TracedCallback<uint32_t, Mac48Address> m_rateChange;


  TracedCallback<uint8_t, Mac48Address> m_powerChange;

  TracedCallback<double, Mac48Address> m_cstChange;
};

} // namespace ns3

#endif /* PRCS_WIFI_MANAGER_H */
