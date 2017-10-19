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
#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <errno.h>
#include <tinyara/irq.h>
#include <tinyara/kmalloc.h>
#include <tinyara/clock.h>
#include <tinyara/net/net.h>
#include <arpa/inet.h>
#include <net/lwip/netif/etharp.h>
#include <net/lwip/ipv4/igmp.h>

#include "netif.h"


/* Net Device callback operations */
static int rda_net_open(struct netif *dev)
{
	return 0;
}

/* This is called after the WE handlers */
static int rda_net_ioctl(struct netif *dev, struct ifreq *rq, int cmd)
{
	if (cmd == 0x89f0 + 1) {
		return 0;
	}

	return -EOPNOTSUPP;
}


static err_t rda_linkoutput(struct netif *dev, struct pbuf *buf)
{
	
}

static err_t slsi_set_multicast_list(struct netif *dev, ip_addr_t *group, u8_t action)
{

}

static struct netif *rda_alloc_netdev(int sizeof_priv)
{
	struct netif *dev;
	void *priv;

	rda_INFO_NODEV("rda_alloc_netdev\n");

	dev = kmm_zalloc(sizeof(struct netif));
	if (dev == NULL) {
		return NULL;
	}

	priv = kmm_zalloc(sizeof_priv);
	if (priv == NULL) {
		kmm_free(dev);
		return NULL;
	}

	/* Initialize the driver structure */
	dev->d_private = priv;
	dev->d_ifup = rda_net_open;
	dev->d_ifdown = rda_net_stop;
	dev->linkoutput = rda_linkoutput;
	dev->output = etharp_output;
	dev->igmp_mac_filter = rda_set_multicast_list;
#ifdef CONFIG_NETDEV_PHY_IOCTL
	dev->d_ioctl = rda_net_ioctl;
#endif
	dev->mtu = CONFIG_NET_ETH_MTU;
	dev->hwaddr_len = IFHWADDRLEN;

	return dev;
}

static void rda_free_netdev(struct netif *dev)
{
	if (dev->d_private) {
		kmm_free(dev->d_private);
	}

	kmm_free(dev);
}




