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
 */



#ifndef RULE_BASED_WIFI_MANAGER_H
#define RULE_BASED_WIFI_MANAGER_H

#include "wifi-remote-station-manager.h"
#include "wifi-mode.h"
#include "ns3/nstime.h"
#include "ns3/random-variable-stream.h"

namespace ns3 {

/**
 * \author Matias Richart
 * \ingroup wifi
 *
 */
class RuleBasedWifiManager : public WifiRemoteStationManager
{

public:
  static TypeId GetTypeId (void);
  RuleBasedWifiManager ();
  virtual ~RuleBasedWifiManager ();

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

  void DecreaseRate (Mac48Address address, uint32_t levels);
  void IncreaseRate (Mac48Address address, uint32_t levels);
  void DecreasePower (Mac48Address address, uint32_t levels);
  void IncreasePower (Mac48Address address, uint32_t levels);
  uint32_t GetStats (Mac48Address address);
  uint32_t GetCurrentRate (Mac48Address address);
  uint8_t GetCurrentPower (Mac48Address address);

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

  /// for estimating the TxTime of a packet with a given mode
  Time GetCalcTxTime (WifiMode mode) const;
  void AddCalcTxTime (WifiMode mode, Time t);


  typedef std::vector<std::pair<Time,WifiMode> > TxTime;

  TxTime m_calcTxTime;  ///< to hold all the calculated TxTime for all modes
  uint32_t m_pktLen;  ///< packet length used  for calculate mode TxTime
  uint32_t m_nsupported;  ///< modes supported
  uint8_t m_nPower;

  /// provides uniform random variables.
  Ptr<UniformRandomVariable> m_uniformRandomVariable;

  /**
   * The trace source fired when a txVector is asked
   */
  TracedCallback<WifiTxVector> m_getDataTxVector;
};

} // namespace ns3

#endif /* RULE_BASED_WIFI_MANAGER_H */
