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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 * Author: Matías Richart <mrichart@fing.edu.uy>
 */

#ifndef PARF_WIFI_MANAGER_H
#define PARF_WIFI_MANAGER_H

#include "wifi-remote-station-manager.h"

namespace ns3 {

struct ParfWifiRemoteStation;
/**
 * \ingroup wifi
 * \brief PARF Rate control algorithm
 *
 * This class implements the PARF algorithm as described in
 * <i>Self-management in chaotic wireless deployments</i>, by
 * Akella, A.; Judd, G.; Seshan, S. & Steenkiste, P. in
 * Wireless Networks, Kluwer Academic Publishers, 2007, 13, 737-755
 *
 */
class ParfWifiManager : public WifiRemoteStationManager
{
public:
  static TypeId GetTypeId (void);
  ParfWifiManager ();
  virtual ~ParfWifiManager ();

  virtual void SetupPhy (Ptr<WifiPhy> phy);

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
  virtual bool IsLowLatency (void) const;

  void CheckInit (ParfWifiRemoteStation *station);


  uint32_t m_timerThreshold;
  uint32_t m_successThreshold;
  uint32_t m_nPower;

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

#endif /* PARF_WIFI_MANAGER_H */
