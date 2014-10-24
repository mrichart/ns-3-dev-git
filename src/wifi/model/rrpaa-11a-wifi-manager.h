/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2005,2006 INRIA
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
 * Author: Federico Maguolo <maguolof@dei.unipd.it>
 * Author: Matías Richart <mrichart@fing.edu.uy>
 */
#ifndef RRPAA_WIFI_MANAGER_H
#define RRPAA_WIFI_MANAGER_H

#include "ns3/nstime.h"
#include "wifi-remote-station-manager.h"

namespace ns3 {

struct RrpaaWifiRemoteStation;

/**
 * \brief Robust Rate and Power Adaptation Algorithm
 * \ingroup wifi
 *
 * This is an implementation of RRPAA which adds power
 * control to the RRAA mechanism. RRAA is described in
 * "Robust rate adaptation for 802.11 wireless networks"
 * by "Starsky H. Y. Wong", "Hao Yang", "Songwu Lu", and,
 * "Vaduvur Bharghavan" published in Mobicom 06.
 *
 * RRPAAIS  described in
 * "Self Management of Power, Rate and Carrier Sense Threshold
 * for Interference Mitigation in IEEE 802.11 Netwroks"
 * by "Matías Richart", "Jorge Visca", and,
 * "Javier Baliosian" presented at CNSM 2014.
 */
class Rrpaa11aWifiManager : public WifiRemoteStationManager
{
public:
  static TypeId GetTypeId (void);

  Rrpaa11aWifiManager ();
  virtual ~Rrpaa11aWifiManager ();

  virtual void SetupPhy (Ptr<WifiPhy> phy);

private:
  struct ThresholdsItem
  {
    uint32_t datarate;
    double pori;
    double pmtl;
    uint32_t ewnd;
  };

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
  struct ThresholdsItem GetThresholds (WifiMode mode) const;
  struct ThresholdsItem GetThresholds (RrpaaWifiRemoteStation *station, uint32_t rate) const;

  bool m_basic;
  Time m_timeout;
  uint32_t m_ewndfor54;
  uint32_t m_ewndfor48;
  uint32_t m_ewndfor36;
  uint32_t m_ewndfor24;
  uint32_t m_ewndfor18;
  uint32_t m_ewndfor12;
  uint32_t m_ewndfor9;
  uint32_t m_ewndfor6;
  uint32_t m_ewndfor11;
  uint32_t m_ewndfor5;
  uint32_t m_ewndfor2;
  uint32_t m_ewndfor1;
  double m_porifor48;
  double m_porifor36;
  double m_porifor24;
  double m_porifor18;
  double m_porifor12;
  double m_porifor9;
  double m_porifor6;
  double m_porifor11;
  double m_porifor5;
  double m_porifor2;
  double m_porifor1;
  double m_pmtlfor54;
  double m_pmtlfor48;
  double m_pmtlfor36;
  double m_pmtlfor24;
  double m_pmtlfor18;
  double m_pmtlfor12;
  double m_pmtlfor9;
  double m_pmtlfor6;
  double m_pmtlfor11;
  double m_pmtlfor5;
  double m_pmtlfor2;

  uint8_t m_nPower;



  /**
   * The trace source fired when the transmission rate change
   */
  TracedCallback<uint32_t, Mac48Address> m_rateChange;


  TracedCallback<uint8_t, Mac48Address> m_powerChange;
};

} // namespace ns3

#endif /* RRAA_WIFI_MANAGER_H */
