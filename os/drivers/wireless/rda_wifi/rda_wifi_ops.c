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

#include "rda_wifi_ops.h"
#include "netif.h"
#include "wlan_80211_utils.h"



#ifdef CONFIG_RDA_ADV_FEATURE
struct wireless_dev *rda_add_virtual_intf(struct wiphy *wiphy, const char *name, enum rda_80211_iftype type, u32 *flags, struct vif_params *params)
{
	
}

int rda_del_virtual_intf(struct wiphy *wiphy, struct wireless_dev *wdev)
{
	return 0;
}

int rda_change_virtual_intf(struct wiphy *wiphy, struct net_device *dev, enum rda_80211_iftype type, u32 *flags, struct vif_params *params)
{
	return 0;
}

#endif	/*CONFIG_RDA_ADV_FEATURE*/	

int rda_add_key(const char *ifname, void *priv, enum wpa_alg alg, const u8 *mac_addr, int key_index, int set_tx, const u8 *seq, size_t seq_len, const u8 *key, size_t key_len)
{
	return 0;
}

int rda_get_ssid(void *priv, u8 *ssid)
{
	return 0;
}

int rda_get_key(const char *ifname, void *priv, const u8 *addr, int idx, u8 *seq)
{
	return 0;
}

const u8 *rda_get_mac_addr(void *priv)
{
	return 0;
}

int rda_hw_scan(void *priv, struct wpa_driver_scan_params *request)
{
	return 0;
}


/**
 * rda_get_scan_results
 *
 * Description:
 *	Process the cached scan results of driver and send it to supplicant
 *	in the appropriate (struct wpa_scan_results) format.
 *
 * Parameters:
 *	priv - Pointer to the private driver data
 *
 * Returns: Scan results on Success, NULL on failure
 */
struct wpa_scan_results *rda_get_scan_results(void *priv)
{

}

int rda_disconnect(void *priv, const u8 *addr, int reason_code)
{
	return 0;
}

int rda_set_rts(void *priv, int rts)
{
	return 0;
}

int rda_get_signal_poll(void *priv, struct wpa_signal_info *si)
{
	return 0;
}

int rda_set_frag_threshold(void *priv, int frag_threshold)
{
	return 0;
}

int rda_set_tx_power(void *priv, int dbm)
{
	return 0;
}

int rda_get_tx_power(void *priv)
{
	return 0;
}

int rda_start_ap(void *priv, struct wpa_driver_ap_params *settings)
{
	return 0;
}

int rda_stop_ap(void *priv)
{
	return 0;
}

int rda_set_panic(void *priv)
{
	return 0;
}



void *rda_wifi_init(void *ctx, const char *ifname, void *global_priv)
{
	return drv;
}

int rda_wifi_deinit(void *priv)
{
	return 0;
}

int rda_wifi_set_param(void *priv, const char *param)
{
	return 0;
}

struct hostapd_hw_modes *rda_get_hw_feature_data(void *priv, u16 *num_modes, u16 *flags)
{
	
}

int rda_wifi_get_capa(void *priv, struct wpa_driver_capa *capa)
{
	return 0;
}



ssize_t rda_set_country(void *priv, const char *country_code)
{

}

ssize_t rda_get_country(void *priv, char *country_code)
{
	
}
