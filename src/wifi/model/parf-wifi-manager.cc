/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2004,2005,2006 INRIA
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
 * Author: Matias Richart <mrichart@fing.edu.uy>
 */

#include "parf-wifi-manager.h"
#include "wifi-phy.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/trace-source-accessor.h"

#define Min(a,b) ((a < b) ? a : b)

NS_LOG_COMPONENT_DEFINE ("ns3::ParfWifiManager");


namespace ns3 {

struct ParfWifiRemoteStation : public WifiRemoteStation
{
  uint32_t m_timer;
  uint32_t m_success;
  uint32_t m_failed;
  bool m_recoveryRate;
  bool m_recoveryPower;
  uint32_t m_retry;

  uint32_t m_timerTimeout;
  uint32_t m_successThreshold;

  uint32_t m_rate;

  uint8_t m_power;
};

NS_OBJECT_ENSURE_REGISTERED (ParfWifiManager);

TypeId
ParfWifiManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::ParfWifiManager")
    .SetParent<WifiRemoteStationManager> ()
    .AddConstructor<ParfWifiManager> ()
    .AddAttribute ("TimerThreshold", "The 'timer' threshold in the ARF algorithm.",
                   UintegerValue (15),
                   MakeUintegerAccessor (&ParfWifiManager::m_timerThreshold),
                   MakeUintegerChecker<uint32_t> ())
    .AddAttribute ("SuccessThreshold",
                   "The minimum number of sucessfull transmissions to try a new rate.",
                   UintegerValue (10),
                   MakeUintegerAccessor (&ParfWifiManager::m_successThreshold),
                   MakeUintegerChecker<uint32_t> ())
    .AddTraceSource ("PowerChange",
                   "The transmission power has change",
                    MakeTraceSourceAccessor (&ParfWifiManager::m_powerChange))
    .AddTraceSource ("RateChange",
                   "The transmission rate has change",
                    MakeTraceSourceAccessor (&ParfWifiManager::m_rateChange))
  ;
  return tid;
}

ParfWifiManager::ParfWifiManager ()
{
  NS_LOG_FUNCTION (this);
}
ParfWifiManager::~ParfWifiManager ()
{
  NS_LOG_FUNCTION (this);
}

void
ParfWifiManager::SetupPhy (Ptr<WifiPhy> phy)
{
  m_nPower = phy->GetNTxPower();
  WifiRemoteStationManager::SetupPhy (phy);
}

WifiRemoteStation *
ParfWifiManager::DoCreateStation (void) const
{
  NS_LOG_FUNCTION (this);
  ParfWifiRemoteStation *station = new ParfWifiRemoteStation ();

  station->m_successThreshold = m_successThreshold;
  station->m_timerTimeout = m_timerThreshold;
  station->m_rate = 0;
  station->m_success = 0;
  station->m_failed = 0;
  station->m_recoveryRate = false;
  station->m_recoveryPower = false;
  station->m_retry = 0;
  station->m_timer = 0;
  station->m_power = m_nPower-1;
  m_powerChange(station->m_power);
  m_rateChange(station->m_rate);

  NS_LOG_DEBUG ("create station=" << station << ", timer=" << station->m_timer << ", rate=" << station->m_rate << ", power=" << (int)station->m_power);

  return station;
}

void
ParfWifiManager::DoReportRtsFailed (WifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
}
/**
 * It is important to realize that "recovery" mode starts after failure of
 * the first transmission after a rate increase and ends at the first successful
 * transmission. Specifically, recovery mode transcends retransmissions boundaries.
 * Fundamentally, ARF handles each data transmission independently, whether it
 * is the initial transmission of a packet or the retransmission of a packet.
 * The fundamental reason for this is that there is a backoff between each data
 * transmission, be it an initial transmission or a retransmission.
 */
void
ParfWifiManager::DoReportDataFailed (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  ParfWifiRemoteStation *station = (ParfWifiRemoteStation *)st;
  station->m_timer++;
  station->m_failed++;
  station->m_retry++;
  station->m_success = 0;

  NS_LOG_DEBUG ("station=" << station << " data fail retry=" << station->m_retry << ", timer=" << station->m_timer
		        << ", rate=" << station->m_rate << ", power=" << (int)station->m_power);
  if (station->m_recoveryRate)
    {
      NS_ASSERT (station->m_retry >= 1);
      if (station->m_retry == 1)
        {
          // need recovery fallback
          if (station->m_rate != 0)
            {
              NS_LOG_DEBUG ("station=" << station << " dec rate");
              station->m_rate--;
              m_rateChange(station->m_rate);
              station->m_recoveryRate = false;
            }
        }
      station->m_timer = 0;
    }
  else if (station->m_recoveryPower)
    {
	  NS_ASSERT (station->m_retry >= 1);
	    if (station->m_retry == 1)
	      {
	        // need recovery fallback
	        if (station->m_power < m_nPower-1)
	          {
	            NS_LOG_DEBUG ("station=" << station << " inc power");
	            station->m_power++;
	            m_powerChange(station->m_power);
	            station->m_recoveryPower = false;
	          }
	      }
	    station->m_timer = 0;
	}
  else
    {
      NS_ASSERT (station->m_retry >= 1);
      if (((station->m_retry - 1) % 2) == 1)
        {
          // need normal fallback
    	  if (station->m_power == m_nPower-1)
    	    {
              if (station->m_rate != 0)
                {
                  NS_LOG_DEBUG ("station=" << station << " dec rate");
                  station->m_rate--;
                  m_rateChange(station->m_rate);
                }
    	    }
    	  else
    	    {
    	      NS_LOG_DEBUG ("station=" << station << " inc power");
    		  station->m_power++;
	          m_powerChange(station->m_power);
    	    }
        }
      if (station->m_retry >= 2)
        {
          station->m_timer = 0;
        }
    }
}
void
ParfWifiManager::DoReportRxOk (WifiRemoteStation *station,
                              double rxSnr, WifiMode txMode)
{
  NS_LOG_FUNCTION (this << station << rxSnr << txMode);
}
void ParfWifiManager::DoReportRtsOk (WifiRemoteStation *station,
                                    double ctsSnr, WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_FUNCTION (this << station << ctsSnr << ctsMode << rtsSnr);
  NS_LOG_DEBUG ("station=" << station << " rts ok");
}
void ParfWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                     double ackSnr, WifiMode ackMode, double dataSnr)
{
  NS_LOG_FUNCTION (this << st << ackSnr << ackMode << dataSnr);
  ParfWifiRemoteStation *station = (ParfWifiRemoteStation *) st;
  station->m_timer++;
  station->m_success++;
  station->m_failed = 0;
  station->m_recoveryRate = false;
  station->m_recoveryPower = false;
  station->m_retry = 0;
  NS_LOG_DEBUG ("station=" << station << " data ok success=" << station->m_success << ", timer=" << station->m_timer << ", rate=" << station->m_rate << ", power=" << (int)station->m_power);
  if ((station->m_success == m_successThreshold
       || station->m_timer == m_timerThreshold)
      && (station->m_rate < (station->m_state->m_operationalRateSet.size () - 1)))
    {
      NS_LOG_DEBUG ("station=" << station << " inc rate");
      station->m_rate++;
      m_rateChange(station->m_rate);
      station->m_timer = 0;
      station->m_success = 0;
      station->m_recoveryRate = true;
    }
  else if (station->m_success == m_successThreshold || station->m_timer == m_timerThreshold)
  {
	  //we are at the maximum rate, we decrease power
	  if (station->m_power != 0)
	  {
	    NS_LOG_DEBUG ("station=" << station << " dec power");
            station->m_power--;
            m_powerChange(station->m_power);
	  }
	  station->m_timer = 0;
      station->m_success = 0;
      station->m_recoveryPower = true;
  }
}
void
ParfWifiManager::DoReportFinalRtsFailed (WifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
}
void
ParfWifiManager::DoReportFinalDataFailed (WifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
}

WifiTxVector
ParfWifiManager::DoGetDataTxVector (WifiRemoteStation *st, uint32_t size)
{
  NS_LOG_FUNCTION (this << st << size);
  ParfWifiRemoteStation *station = (ParfWifiRemoteStation *) st;
  return WifiTxVector (GetSupported (station, station->m_rate), station->m_power, GetLongRetryCount (station), GetShortGuardInterval (station), Min (GetNoOfRx (station),GetNoOfTransmitters()), GetNoOfTx (station), GetStbc (station));
}
WifiTxVector
ParfWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  /// \todo we could/should implement the Arf algorithm for
  /// RTS only by picking a single rate within the BasicRateSet.
  ParfWifiRemoteStation *station = (ParfWifiRemoteStation *) st;
  return WifiTxVector (GetSupported (station, 0), GetDefaultTxPowerLevel (), GetShortRetryCount (station), GetShortGuardInterval (station), Min (GetNoOfRx (station),GetNoOfTransmitters()), GetNoOfTx (station), GetStbc (station));
}

bool
ParfWifiManager::IsLowLatency (void) const
{
  NS_LOG_FUNCTION (this);
  return true;
}

} // namespace ns3
