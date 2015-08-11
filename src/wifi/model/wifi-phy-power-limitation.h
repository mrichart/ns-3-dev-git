/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015 Universidad de la República
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
#ifndef WIFI_PHY_POWER_LIMITATION_H
#define WIFI_PHY_POWER_LIMITATION_H

#include "wifi-mode.h"

namespace ns3 {


/**
 * \ingroup wifi
 * Identifies the PHY power limitations model.
 */
enum WifiPhyPowerLimitation
{
  /** Wistron DCMA-82 (at 5.825 GHz, 802.11a channel 165) */
  WIFI_PHY_POWER_LIMITATION_WISTRON_DCMA_82_80211a,
  /** Wistron CM9 (at 5.825 GHz, 802.11a channel 165) */
  WIFI_PHY_POWER_LIMITATION_WISTRON_CM9_80211a
};

struct WifiPowerLimitationElement
{
  WifiMode mode;
  uint8_t mcsIndex;
  uint8_t powerLimit;
};

typedef std::vector<WifiPowerLimitationElement> WifiPowerLimitationList;

} //namespace ns3

#endif /* WIFI_PHY_POWER_LIMITATION_H */
