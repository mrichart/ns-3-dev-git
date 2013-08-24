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
#include "aparf-wifi-manager.h"
#include "wifi-phy.h"
#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/uinteger.h"
#include "ns3/trace-source-accessor.h"
#define Min(a,b) ((a < b) ? a : b)
NS_LOG_COMPONENT_DEFINE ("ns3::AparfWifiManager");

namespace ns3 {

struct
AparfWifiRemoteStation: public WifiRemoteStation
{
  uint32_t m_success;
  uint32_t m_failed;
  uint32_t m_pCount;

  uint32_t m_successThreshold;
  uint32_t m_failThreshold;

  uint32_t m_rate;
  uint32_t m_rateCrit;
  uint8_t m_power;

  AparfWifiManager::State m_aparfState;
};

NS_OBJECT_ENSURE_REGISTERED (AparfWifiManager);

TypeId
AparfWifiManager::GetTypeId(void)
{
  static TypeId tid = TypeId("ns3::AparfWifiManager") .SetParent<WifiRemoteStationManager> ()
    .AddConstructor<AparfWifiManager> ()
    .AddAttribute("SuccessThreshold 1",
                  "The minimum number of sucessfull transmissions in \"High\" state.",
                  UintegerValue(3),
                  MakeUintegerAccessor(&AparfWifiManager::m_succesMax1),
                  MakeUintegerChecker<uint32_t> ())
    .AddAttribute("SuccessThreshold 2",
                  "The minimum number of sucessfull transmissions in \"Low\" state.",
                  UintegerValue(10),
                  MakeUintegerAccessor(&AparfWifiManager::m_succesMax2),
                  MakeUintegerChecker<uint32_t> ())
    .AddAttribute("FailThreshold",
                  "The minimum number of failed transmissions.",
                  UintegerValue(1),
                  MakeUintegerAccessor(&AparfWifiManager::m_failMax),
                  MakeUintegerChecker<uint32_t> ())
    .AddAttribute("PowerThreshold",
                  "The maximum number of power changes.",
                  UintegerValue(10),
                  MakeUintegerAccessor(&AparfWifiManager::m_powerMax),
                  MakeUintegerChecker<uint32_t> ())
    .AddAttribute("Power decrement step",
                  "Step size for decrement the power.",
                  UintegerValue(1),
                  MakeUintegerAccessor(&AparfWifiManager::m_powerDec),
                  MakeUintegerChecker<uint32_t> ())
    .AddAttribute("Power increment step",
                  "Step size for increment the power.",
                  UintegerValue(1),
                  MakeUintegerAccessor(&AparfWifiManager::m_powerInc),
                  MakeUintegerChecker<uint32_t> ())
    .AddAttribute("Rate decrement step",
                  "Step size for decrement the rate.",
                  UintegerValue(1),
                  MakeUintegerAccessor(&AparfWifiManager::m_rateDec),
                  MakeUintegerChecker<uint32_t> ())
    .AddAttribute("Rate increment step",
                  "Step size for increment the rate.",
                  UintegerValue(1),
                  MakeUintegerAccessor(&AparfWifiManager::m_rateInc),
                  MakeUintegerChecker<uint32_t> ())
    .AddTraceSource("PowerChange",
                    "The transmission power has change",
                    MakeTraceSourceAccessor(&AparfWifiManager::m_powerChange))
    .AddTraceSource("RateChange",
                    "The transmission rate has change",
                    MakeTraceSourceAccessor(&AparfWifiManager::m_rateChange))
    ;
  return tid;
}

AparfWifiManager::AparfWifiManager()
{
  NS_LOG_FUNCTION (this);
}
AparfWifiManager::~AparfWifiManager()
{
  NS_LOG_FUNCTION (this);
}

void AparfWifiManager::SetupPhy(Ptr<WifiPhy> phy)
{
  m_nPower = phy->GetNTxPower();
  m_nRate = phy->GetNModes();
  WifiRemoteStationManager::SetupPhy(phy);
}

WifiRemoteStation *
AparfWifiManager::DoCreateStation(void) const
{
  NS_LOG_FUNCTION (this);
  AparfWifiRemoteStation *station = new AparfWifiRemoteStation();

  station->m_successThreshold = m_succesMax1;
  station->m_failThreshold = m_failMax;
  station->m_rate = m_nRate - 1;
  station->m_power = m_nPower - 1;
  station->m_success = 0;
  station->m_failed = 0;
  station->m_pCount = 0;
  station->m_aparfState = AparfWifiManager::High;
  m_powerChange(station->m_power);
  m_rateChange(station->m_rate);

  NS_LOG_DEBUG ("create station=" << station << ", rate=" << station->m_rate << ", power=" << (int)station->m_power);

  return station;
}

void AparfWifiManager::DoReportRtsFailed(WifiRemoteStation *station)
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
void AparfWifiManager::DoReportDataFailed(WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  AparfWifiRemoteStation *station = (AparfWifiRemoteStation *) st;
  station->m_failed++;
  station->m_success = 0;
  NS_LOG_DEBUG ("station=" << station << ", rate=" << station->m_rate << ", power=" << (int)station->m_power);

  if (station->m_aparfState == AparfWifiManager::Low)
    {
      station->m_aparfState = AparfWifiManager::High;
      station->m_successThreshold = m_succesMax1;
    }
  else if (station->m_aparfState == AparfWifiManager::Spread)
    {
      station->m_aparfState = AparfWifiManager::Low;
      station->m_successThreshold = m_succesMax2;
    }

  if (station->m_failed == station->m_failThreshold)
    {
      station->m_failed = 0;
      station->m_success = 0;
      station->m_pCount--;
      if (station->m_power == (m_nPower - 1))
        {
          station->m_rateCrit = station->m_rate;
	  if (station->m_rate != 0)
	    {
	      NS_LOG_DEBUG ("station=" << station << " dec rate");
	      station->m_rate -= m_rateDec;
	      m_rateChange(station->m_rate);
	    }
        }
      else
        {
          NS_LOG_DEBUG ("station=" << station << " inc power");
          station->m_power += m_powerInc;
          m_powerChange(station->m_power);
         }
      }
}
void
AparfWifiManager::DoReportRxOk(WifiRemoteStation *station, double rxSnr, WifiMode txMode)
{
  NS_LOG_FUNCTION (this << station << rxSnr << txMode);
}
void
AparfWifiManager::DoReportRtsOk(WifiRemoteStation *station, double ctsSnr,
                                    WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_FUNCTION (this << station << ctsSnr << ctsMode << rtsSnr); NS_LOG_DEBUG ("station=" << station << " rts ok");
}
void
AparfWifiManager::DoReportDataOk(WifiRemoteStation *st, double ackSnr,
		                     WifiMode ackMode, double dataSnr)
{
  NS_LOG_FUNCTION (this << st << ackSnr << ackMode << dataSnr);
  AparfWifiRemoteStation *station = (AparfWifiRemoteStation *) st;
  station->m_success++;
  station->m_failed = 0;
  NS_LOG_DEBUG ("station=" << station << " data ok success=" << station->m_success << ", rate=" << station->m_rate << ", power=" << (int)station->m_power);

  if ((station->m_aparfState == AparfWifiManager::High) && (station->m_success >= station->m_successThreshold))
    {
      station->m_aparfState = AparfWifiManager::Spread;
    }
  else if ((station->m_aparfState == AparfWifiManager::Low) && (station->m_success >= station->m_successThreshold))
    {
      station->m_aparfState = AparfWifiManager::Spread;
    }
  else if (station->m_aparfState == AparfWifiManager::Spread)
    {
      station->m_aparfState = AparfWifiManager::High;
      station->m_successThreshold = m_succesMax1;
    }

  if (station->m_success == station->m_successThreshold)
    {
      station->m_success = 0;
      station->m_failed = 0;
      if (station->m_rate == (station->m_state->m_operationalRateSet.size() - 1))
        {
          if (station->m_power != 0)
            {
              NS_LOG_DEBUG ("station=" << station << " dec power");
              station->m_power -= m_powerDec;
              m_powerChange(station->m_power);
            }
        }
      else
        {
          if (station->m_rateCrit == 0)
            {
              if (station->m_rate != (station->m_state->m_operationalRateSet.size() - 1))
                {
                  NS_LOG_DEBUG ("station=" << station << " inc rate");
                  station->m_rate += m_rateInc;
                  m_rateChange(station->m_rate);
                }
            }
          else
            {
              if (station->m_pCount == m_powerMax)
                {
                  station->m_power = (m_nPower - 1);
                  m_powerChange(station->m_power);
                  station->m_rate = station->m_rateCrit;
                  m_rateChange(station->m_rate);
                  station->m_pCount = 0;
                  station->m_rateCrit = 0;
                }
              else
                {
                  station->m_power -= m_powerDec;
                  m_powerChange(station->m_power);
                  station->m_pCount++;
                }
            }
        }
    }
}
void
AparfWifiManager::DoReportFinalRtsFailed(WifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
}
void
AparfWifiManager::DoReportFinalDataFailed(WifiRemoteStation *station)
{
  NS_LOG_FUNCTION (this << station);
}

WifiTxVector
AparfWifiManager::DoGetDataTxVector(WifiRemoteStation *st, uint32_t size)
{
  NS_LOG_FUNCTION (this << st << size);
  AparfWifiRemoteStation *station = (AparfWifiRemoteStation *) st;
  return WifiTxVector(GetSupported(station, station->m_rate), station->m_power, GetLongRetryCount(station),
                  GetShortGuardInterval(station), Min (GetNoOfRx (station),GetNoOfTransmitters()),
                  GetNoOfTx(station), GetStbc(station));
}
WifiTxVector
AparfWifiManager::DoGetRtsTxVector(WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  /// \todo we could/should implement the Arf algorithm for
  /// RTS only by picking a single rate within the BasicRateSet.
  AparfWifiRemoteStation *station = (AparfWifiRemoteStation *) st;
  return WifiTxVector(GetSupported(station, 0), GetDefaultTxPowerLevel(), GetShortRetryCount(station), GetShortGuardInterval(station),
                  Min (GetNoOfRx (station),GetNoOfTransmitters()), GetNoOfTx(station), GetStbc(station));
}

bool
AparfWifiManager::IsLowLatency(void) const
{
  NS_LOG_FUNCTION (this);
  return true;
}

} // namespace ns3
