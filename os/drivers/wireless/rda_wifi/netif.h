/*****************************************************************************
 *
 * Copyright 2017 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/

#include "dev.h"

#ifndef __RDA_NETIF_H__
#define __RDA_NETIF_H__

/* netif queues
 * ---------------------------------------------
 *	1 Queue for Security frames (EAPOL, WAPI etc)
 *	1 Queue for Broadcast/Multicast when in AP mode.
 *	4 Queues per Peer
 *
 *	STA/ADHOC
 *	Queues
 *	-------------------------------------------------------
 *	|    0    |    1    |  2 - 5  |  6  |  7  |  8  |  9  |
 *	-------------------------------------------------------
 *	| Eapol   | Discard |   Not   | AC  | AC  | AC  | AC  |
 *	| Frames  | Queue   |   Used  |  0  |  1  |  2  |  3  |
 *	-------------------------------------------------------
 *
 *	AP
 *	Queues
 *	                                                            --------------------------------------------------------
 *	                                                            |   Peer 1 ACs (0 - 4)  |   Peer 2 ACs (0 - 4)  | ......
 *	--------------------------------------------------------------------------------------------------------------------
 *	|    0    |    1    |    2    |    3    |    4    |    5    |  6  |  7  |  8  |  9  |  10 |  11 |  12 |  13 | ......
 *	--------------------------------------------------------------------------------------------------------------------
 *	| Eapol   | Discard |B/M Cast |B/M Cast |B/M Cast |B/M Cast | AC  | AC  | AC  | AC  | AC  | AC  | AC  | AC  | ......
 *	| Frames  |  Queue  |  AC 0   |  AC 1   |  AC 2   |  AC 3   |  0  |  1  |  2  |  3  |  0  |  1  |  2  |  3  | ......
 *	--------------------------------------------------------------------------------------------------------------------
 */


static inline void *netdev_priv(const struct netif *dev)
{
	if (dev) {
		return (FAR struct netdev_vif *)dev->d_private;
	} else {
		return NULL;
	}
}



#endif /*__rda_NETIF_H__*/
