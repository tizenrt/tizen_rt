/*
 * Interaction with the rda5981 wifi driver
 * Copyright (c) 2016, Samsung Electronics Co., Ltd.
 */

#include "includes.h"
#include "driver.h"

#ifdef CONFIG_RDA5981_WLAN
extern void *rda_wifi_init(void *ctx, const char *ifname, void *global_priv);
extern void rda_wifi_deinit(void *priv);
extern const u8 *rda_get_mac_addr(void *priv);
extern int rda_wifi_get_capa(void *priv, struct wpa_driver_capa *capa);
extern int rda_hw_scan(void *priv, struct wpa_driver_scan_params *params);
extern struct wpa_scan_results *rda_get_scan_results(void *priv);
extern int rda_connect(void *priv, struct wpa_driver_associate_params *params);
extern int rda_disconnect(void *priv, const u8 *addr, int reason_code);
extern int rda_start_ap(void *priv, struct wpa_driver_ap_params *settings);
extern int rda_stop_ap(void *priv);
extern int rda_del_station(void *priv, const u8 *addr, int reason);
extern ssize_t rda_set_country(void *priv, const char *country_code);
extern ssize_t rda_get_country(void *priv, char *country_code);
extern int rda_set_rts(void *priv, int rts);
extern int rda_get_signal_poll(void *priv, struct wpa_signal_info *si);
extern int rda_set_frag_threshold(void *priv, int frag_threshold);
extern int rda_get_ap_bssid(void *priv, u8 *bssid);
extern struct hostapd_hw_modes *rda_get_hw_feature_data(void *priv, u16 *num_modes, u16 *flags);

extern int rda_mlme_send_ap_eapol(void *priv, const u8 *addr, const u8 *data, size_t data_len, int encrypt, const u8 *own_addr, u32 flags);
extern int rda_get_key(const char *ifname, void *priv, const u8 *addr, int idx, u8 *seq);
extern int rda_set_tx_power(void *priv, int dbm);
extern int rda_get_tx_power(void *priv);
extern int rda_set_panic(void *priv);

//extern struct wireless_dev *rda_add_virtual_intf(struct wiphy *wiphy, const char *name, enum nl80211_iftype type, u32 *flags, struct vif_params *params);
extern int rda_add_key(const char *ifname, void *priv, enum wpa_alg alg, const u8 *mac_addr, int key_idx, int set_tx, const u8 *seq, size_t seq_len, const u8 *key, size_t key_len);
extern int rda_get_ssid(void *priv, u8 *ssid);

int rda_sta_remove(void *priv, const u8 *addr)
{
	return rda_del_station(priv, addr, 0);
}

int rda_sta_deauth(void *priv, const u8 *own_addr, const u8 *addr, int reason)
{
	return rda_del_station(priv, addr, reason);
}

const struct wpa_driver_ops wpa_driver_rda5981_ops = {
	.name = "rda_wifi",
	.desc = "RDA WIFI Driver",
	.init2 = rda_wifi_init,
	.deinit = rda_wifi_deinit,
	.get_mac_addr = rda_get_mac_addr,
	.get_capa = rda_wifi_get_capa,
	.scan2 = rda_hw_scan,
	.get_scan_results2 = rda_get_scan_results,
	.associate = rda_connect,
	.deauthenticate = rda_disconnect,
	.set_ap = rda_start_ap,
	.stop_ap = rda_stop_ap,
	.sta_remove = rda_sta_remove,
	.sta_deauth = rda_sta_deauth,
	.set_frag = rda_set_frag_threshold,
	.set_rts = rda_set_rts,
	.signal_poll = rda_get_signal_poll,
	.get_country = rda_get_country,
	.set_country = rda_set_country,
	.get_bssid = rda_get_ap_bssid,
	.get_hw_feature_data = rda_get_hw_feature_data,
	.set_key = rda_add_key,
	.get_ssid = rda_get_ssid,
	.hapd_send_eapol = rda_mlme_send_ap_eapol,
	.get_seqnum = rda_get_key,
	.set_tx_power = rda_set_tx_power,
	.get_tx_power = rda_get_tx_power,
	.set_panic = rda_set_panic,
//  .if_add = rda_add_virtual_intf
};
#else
int rda_sta_remove(void *priv, const u8 *addr)
{
	return 0;
}

int rda_sta_deauth(void *priv, const u8 *own_addr, const u8 *addr, int reason)
{
	return 0;
}

const struct wpa_driver_ops wpa_driver_rda5981_ops = {
	.name = "rda_wifi",
	.desc = "RDA WIFI Driver",
};
#endif
