/*
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2010-2011  ST-Ericsson SA
 *
 *  Author: Hemant Gupta <hemant.gupta@stericsson.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdlib.h>
#include <errno.h>

#include <dbus/dbus.h>
#include <glib.h>

#include "log.h"
#include "adapter.h"
#include "../src/device.h"
#include "device.h"
#include "ipc.h"
#include "manager.h"
#include "avdtp.h"
#include "sink.h"
#include "source.h"
#include "unix.h"
#include "media.h"
#include "a2dp.h"
#include "ste-qos.h"

/* QoS specific constants */
#define L2CAP_HEADER_LEN	17
#define EDR_L2CAP_BUNCHING	4
#define BR_L2CAP_BUNCHING	3
#define EDR_VS_QOS_MARGIN	30	/* In percent */
#define BR_VS_QOS_MARGIN	5	/* In percent */
#define EDR_STD_QOS_MARGIN	40	/* In percent */
#define BR_STD_QOS_MARGIN	15	/* In percent */
#define FREQ_KHZ_TO_HZ_FACTOR	1000
#define SLOT_LEN		625	/* In microsecs */
#define HIGH_QUALITY_BIT_RATE	345	/* In Kbps */
#define MEDIUM_QUALITY_BIT_RATE	237	/* In Kbps */

#define SBC_EDR_MAX_BITPOOL	53
#define SBC_BR_MAX_BITPOOL	33

/*
 * Maximum buffering capacity of remote device.
 * This is generally within 100 msec but for robustness we use 60 msec.
 */
#define MAX_SERVICE_INTERVAL	96	/* In slots */

#define ROUND_UP(x, y) (((x) + ((y) - 1)) / (y))
#define ADDR_OFF 3
#define TOKEN_RATE		100	/* In octects per sec */

/* role_switch_blacklist data */
struct role_switch_blacklist {
	bdaddr_t bd_addr;
	char *name;
};

static GSList *role_switch_blacklist;

static void device_create_role_switch_blacklist(void)
{
	GError *err;
	GKeyFile *config;
	char *bd_addr;
	char *name;
	const char *file = CONFIGDIR "/qos_role_switch_blacklist.conf";
	struct role_switch_blacklist *new_entry;
	char **list;
	char **config_entry;
	int i;

	DBG("parsing qos_role_switch_blacklist.conf");

	config = g_key_file_new();

	g_key_file_set_list_separator(config, ';');

	if (!g_key_file_load_from_file(config, file, 0, &err)) {
		error("Parsing %s failed: %s", file, err->message);
		g_error_free(err);
		goto free_config;
	}

	list = g_key_file_get_string_list(config, "General", "DeviceList",
			NULL, NULL);

	for (i = 0; list && list[i] != NULL; i++) {
		config_entry = g_strsplit(list[i], ",", 2);
		bd_addr = config_entry[0];
		name = config_entry[1];
		if (!bd_addr || !name) {
			g_strfreev(config_entry);
			continue;
		}
		new_entry = g_new(struct role_switch_blacklist, 1);
		new_entry->name = (char *)g_malloc(strlen(name) + 1);

		str2ba(bd_addr, &new_entry->bd_addr);
		strcpy(new_entry->name, name);

		role_switch_blacklist = g_slist_prepend(role_switch_blacklist,
								new_entry);

		DBG("Entry Address = %s Name = %s", bd_addr, new_entry->name);

		g_strfreev(config_entry);
	}

	g_strfreev(list);
free_config:
	g_key_file_free(config);
}

static void device_info_free(struct role_switch_blacklist *dev)
{
	g_free(dev->name);
	g_free(dev);
}

static void device_free_role_switch_blacklist(void)
{
	if (!role_switch_blacklist)
		return;

	g_slist_foreach(role_switch_blacklist, (GFunc) device_info_free, NULL);
	g_slist_free(role_switch_blacklist);
	role_switch_blacklist = NULL;
}

static int device_cmp(gconstpointer data, gconstpointer user_data)
{
	const struct role_switch_blacklist *entry = data;
	struct btd_device *device = (void *)user_data;
	bdaddr_t ba;
	char name[MAX_NAME_LENGTH + 1];

	device_get_address(device, &ba, NULL);
	device_get_name(device, name, sizeof(name));

	/*
	 * Check if the First 3 bytes of BD Address and device name match the
	 * role_switch_blacklist. If yes return TRUE.
	 */
	if (!memcmp(&ba.b[ADDR_OFF], &entry->bd_addr.b[ADDR_OFF], ADDR_OFF) &&
			strstr(name, entry->name))
		return 0;

	return -1;
}

/*
 * Some headsets switch to master role immediately after the streaming
 * is started first time after A2DP connection is made. This behaviour
 * results in audio cuts. To avoid this scenario, it's better to forbid
 * Role Switch with these headsets. We do that only for devices which are
 * known to not misbehave when forbidden role switch.
 */
static void forbid_role_switch(struct btd_device *device)
{
	int err;
	bdaddr_t ba;

	if (!g_slist_find_custom(role_switch_blacklist, device, device_cmp))
		return;

	device_get_address(device, &ba, NULL);

	err = btd_adapter_forbid_role_switch(device_get_adapter(device), &ba);
	if (err < 0)
		error("Forbid role switch failed (%s)", strerror(-err));
}

static uint16_t adjust_out_service_window(uint16_t out_window, uint16_t factor)
{
	return factor * ROUND_UP(out_window, factor);
}

static uint16_t get_freq(uint8_t freq)
{
	switch (freq) {
	case BT_SBC_SAMPLING_FREQ_16000:
		return 16000;
	case BT_SBC_SAMPLING_FREQ_32000:
		return 32000;
	case BT_SBC_SAMPLING_FREQ_44100:
		return 44100;
	case BT_SBC_SAMPLING_FREQ_48000:
		return 48000;
	default:
		error("Bad SBC frequency used (%d)", freq);
		return 0;
	}
}

static uint8_t get_block_len(uint8_t len)
{
	switch (len) {
	case BT_A2DP_BLOCK_LENGTH_4:
		return 4;
	case BT_A2DP_BLOCK_LENGTH_8:
		return 8;
	case BT_A2DP_BLOCK_LENGTH_12:
		return 12;
	case BT_A2DP_BLOCK_LENGTH_16:
		return 16;
	default:
		error("Bad SBC block length used (%d)", len);
		return 0;
	}
}

static uint8_t get_subbands(uint8_t subbands)
{
	switch (subbands) {
	case BT_A2DP_SUBBANDS_4:
		return 4;
	case BT_A2DP_SUBBANDS_8:
		return 8;
	default:
		error("Bad SBC subbands used (%d)", subbands);
		return 0;
	}
}

static uint8_t get_sbc_frame_len(struct sbc_codec_cap *sbc_cap)
{
	uint8_t subbands;
	uint8_t block_len;
	uint8_t max_bitpool;
	uint8_t alloc_method;

	alloc_method = sbc_cap->allocation_method;
	subbands = get_subbands(sbc_cap->subbands);
	block_len = get_block_len(sbc_cap->block_length);
	max_bitpool = sbc_cap->max_bitpool;

	DBG("get_sbc_frame_len: alloc_method = %d, subbands = %d, "
			"block_len = %d, max_bitpool = %d",
			alloc_method, subbands, block_len, max_bitpool);

	/*
	 * Below formulae for calculation of frame length are described in
	 * section 12.9 of A2DP specification 1.2
	 */
	switch (alloc_method) {
	case BT_A2DP_CHANNEL_MODE_MONO:
		return 4 + (subbands / 2) +
			(((block_len * max_bitpool) + 7) / 8);
	case BT_A2DP_CHANNEL_MODE_DUAL_CHANNEL:
		return 4 + subbands + (((block_len * max_bitpool) + 3) / 4);
	case BT_A2DP_CHANNEL_MODE_STEREO:
		return 4 + subbands + (((block_len * max_bitpool)  + 7) / 8);
	case BT_A2DP_CHANNEL_MODE_JOINT_STEREO:
		return 4 + subbands +
			((subbands + (block_len * max_bitpool) + 7) / 8);
	default:
		error("Bad SBC allocation method used (%d)", alloc_method);
		return 0;
	}
}

static uint8_t get_cqae(uint8_t *features)
{
	/*
	 * The below order of returning is chosen based on robustness of
	 * packet type and also the supported packet types by remote device.
	 */
	if ((features[4] & LMP_EDR_3SLOT) && (features[3] & LMP_EDR_ACL_2M))
		return CQAE_2DH3;

	if ((features[5] & LMP_EDR_5SLOT) && (features[3] & LMP_EDR_ACL_2M))
		return CQAE_2DH5;

	if ((features[4] & LMP_EDR_3SLOT) && (features[3] & LMP_EDR_ACL_3M))
		return CQAE_3DH3;

	if ((features[5] & LMP_EDR_5SLOT) && (features[3] & LMP_EDR_ACL_3M))
		return CQAE_3DH5;

	if (features[3] & LMP_EDR_ACL_3M)
		return CQAE_3DH1;

	if (features[3] & LMP_EDR_ACL_2M)
		return CQAE_2DH1;

	if (features[0] & LMP_5SLOT)
		return CQAE_DM5;

	if (features[0] & LMP_3SLOT)
		return CQAE_DM3;

	return CQAE_DM1;
}

static uint16_t get_packet_payload_len(uint8_t cqae)
{
	switch (cqae) {
	case CQAE_DM3:
		return PACKET_PAYLOAD_DM3;
	case CQAE_DM5:
		return PACKET_PAYLOAD_DM5;
	case CQAE_2DH1:
		return PACKET_PAYLOAD_2DH1;
	case CQAE_2DH3:
		return PACKET_PAYLOAD_2DH3;
	case CQAE_2DH5:
		return PACKET_PAYLOAD_2DH5;
	case CQAE_3DH1:
		return PACKET_PAYLOAD_3DH1;
	case CQAE_3DH3:
		return PACKET_PAYLOAD_3DH3;
	case CQAE_3DH5:
		return PACKET_PAYLOAD_3DH5;
	case CQAE_DM1:
	default:
		return PACKET_PAYLOAD_DM1;
	}
}

static uint8_t get_packet_len_in_slots(uint8_t cqae)
{
	switch (cqae) {
	case CQAE_DM3:
	case CQAE_2DH3:
	case CQAE_3DH3:
		return PACKET_LEN_3_SLOT_PKT;
	case CQAE_DM5:
	case CQAE_2DH5:
	case CQAE_3DH5:
		return PACKET_LEN_5_SLOT_PKT;
	case CQAE_DM1:
	case CQAE_2DH1:
	case CQAE_3DH1:
	default:
		return PACKET_LEN_1_SLOT_PKT;
	}
}

static void adjust_service_interval(uint16_t *service_interval,
				int l2cap_interval, uint8_t *l2cap_bunching)
{
	uint8_t l2cap_new_bunching;

	if (!service_interval || !l2cap_bunching)
		return;

	l2cap_new_bunching = *l2cap_bunching;
	while (*service_interval > MAX_SERVICE_INTERVAL) {
		/*
		 * Adjust service interval by reducing l2cap bunching
		 * until it reaches acceptable limit.
		 */
		l2cap_new_bunching--;
		if (l2cap_new_bunching == 0) {
			/* This condition should never be reached. */
			error("l2cap bunching reached 0, last interval = %d",
							*service_interval);
			l2cap_new_bunching++;
			goto done;
		}
		*service_interval = l2cap_interval * l2cap_new_bunching * 2;
	}

done:
	*l2cap_bunching = l2cap_new_bunching;
}

/* Check remote  if remote device is EDR on ACL */
static int is_edr(struct btd_device *device)
{
	uint8_t *features = device_get_features(device);

	return features[3] & LMP_EDR_ACL_2M;
}

static void get_ste_qos_params(struct btd_device *device,
				struct avdtp_media_codec_capability *codec_cap,
				uint16_t omtu, struct ste_qos_params *params)
{
	struct sbc_codec_cap *sbc_cap = (struct sbc_codec_cap *)codec_cap;
	uint16_t frequency;
	uint8_t sub_bands;
	uint8_t block_len;

	int sbc_per_l2cap;
	int l2cap_interval;
	int l2cap_size;
	int sbc_frame_len;
	int nbr_bt_pkts_per_mtu;
	int nbr_bt_pkts_per_l2cap;
	uint8_t l2cap_bunching;
	uint8_t qos_margin;
	uint8_t std_qos_margin;
	uint16_t bit_rate;

	frequency = get_freq(sbc_cap->frequency);
	if (!frequency) {
		error("get_ste_qos_params: Got zero frequency");
		return;
	}

	sub_bands = get_subbands(sbc_cap->subbands);
	block_len = get_block_len(sbc_cap->block_length);

	sbc_frame_len = get_sbc_frame_len(sbc_cap);
	if (!sbc_frame_len) {
		error("get_ste_qos_params: Got zero SBC frame length");
		return;
	}

	sbc_per_l2cap = (omtu - L2CAP_HEADER_LEN) / sbc_frame_len;
	/*
	 * Calculation has been altered to avoid floating point arithmetic.
	 * 1.25 is converted to 5/4
	 */
	l2cap_interval = (sub_bands * block_len * sbc_per_l2cap *
			FREQ_KHZ_TO_HZ_FACTOR * 4) / (5 * frequency);
	l2cap_size = sbc_per_l2cap * sbc_frame_len + L2CAP_HEADER_LEN;

	DBG("get_ste_qos_params: sbc_frame_len = %d, sbc_per_l2cap = %d "
		"l2cap_interval = %d l2cap_size = %d",
		sbc_frame_len, sbc_per_l2cap, l2cap_interval, l2cap_size);

	params->packet_size = omtu;
	params->token_bucket_size = omtu;
	params->flags = 0x00;
	params->direction = FLOW_DIRECTION_OUTGOING;
	params->service_type = SERVICE_TYPE_GUARANTEED;
	params->cqae = get_cqae(device_get_features(device));
	nbr_bt_pkts_per_mtu = ROUND_UP(omtu,
					get_packet_payload_len(params->cqae));
	nbr_bt_pkts_per_l2cap = ROUND_UP(l2cap_size,
					get_packet_payload_len(params->cqae));
	if (is_edr(device)) {
		l2cap_bunching = EDR_L2CAP_BUNCHING;
		qos_margin = EDR_VS_QOS_MARGIN;
		bit_rate = HIGH_QUALITY_BIT_RATE;
		std_qos_margin = EDR_STD_QOS_MARGIN;
	} else {
		l2cap_bunching = BR_L2CAP_BUNCHING;
		qos_margin = BR_VS_QOS_MARGIN;
		bit_rate = MEDIUM_QUALITY_BIT_RATE;
		std_qos_margin = BR_STD_QOS_MARGIN;
	}
	params->service_interval = l2cap_interval * l2cap_bunching * 2;
	adjust_service_interval(&params->service_interval, l2cap_interval,
			&l2cap_bunching);
	params->out_service_window = l2cap_size * l2cap_bunching *
			(100 + qos_margin) / 100;
	/*
	 * Add workaround to limit Token Rate so that the command is not
	 * rejected by BT Firmware. This would be removed when the firmware
	 * issue is resolved.
	 */
	params->token_rate = TOKEN_RATE;
	params->peak_bandwidth = params->token_rate;
	/*
	 * Out Service Window should be rounded up to integer multiple of
	 * (L2capSize / NrBtPktsPerL2cap).
	 */
	params->out_service_window =
		adjust_out_service_window(params->out_service_window,
				l2cap_size / nbr_bt_pkts_per_l2cap);
	params->access_latency = params->service_interval * SLOT_LEN;
	/* Keep in_service_window as zero */
	params->in_service_window = 0;
}

void ste_qos_disable(struct btd_device *device)
{
	struct ste_qos_params params;

	memset(&params, 0, sizeof(params));
	device_set_qos(device, &params);
}

void ste_qos_enable( struct btd_device *device, struct avdtp_stream *stream)
{
	struct avdtp_media_codec_capability *codec_cap = NULL;
	struct sbc_codec_cap *sbc_cap;
	struct ste_qos_params params;
	uint16_t omtu;
	GSList *caps = NULL;

	avdtp_stream_get_transport(stream, NULL, NULL, &omtu, &caps);
	if (!caps) {
		error("ste_qos_enable: No A2DP caps");
		return;
	}

	DBG("omtu %d", omtu);

	/*
	 * MTU is propagated as one of the parameter when setting up
	 * QoS on controller, valid values can't overrun 3-DH5 size.
	 */
	if (omtu > PACKET_PAYLOAD_3DH5)
		omtu = PACKET_PAYLOAD_3DH5;

	for (; caps != NULL; caps = g_slist_next(caps)) {
		struct avdtp_service_capability *cap = caps->data;

		if (cap->category != AVDTP_MEDIA_CODEC)
			continue;

		codec_cap = (struct avdtp_media_codec_capability *) cap->data;
		break;
	}

	if (!codec_cap) {
		error("ste_qos_enable: Cannot find Media capabilities");
		return;
	}

	if (codec_cap->media_type != AVDTP_MEDIA_TYPE_AUDIO ||
			codec_cap->media_codec_type != A2DP_CODEC_SBC) {
		error("ste_qos_enable: Not a SBC stream (%d, %d)",
				codec_cap->media_type,
				codec_cap->media_codec_type);
		return;
	}

	get_ste_qos_params(device, codec_cap, omtu, &params);

	/* No need to forbid role switch when disabling QoS */
	if (params.out_service_window != 0 || params.in_service_window != 0)
		forbid_role_switch(device);

	device_set_qos(device, &params);
}

void ste_qos_clip_sbc_cap(struct btd_device *device, void *cap)
{
	sbc_capabilities_t *sbc = cap;

	/*
	 * Update the maximum bitpool based on remote device supported
	 * data rate. This is to ensure we reserve bandwidth so that
	 * more than one ACL can still be supported for OPP/FTP while
	 * A2DP streaming with QoS is active.
	 */
	if (is_edr(device))
		sbc->max_bitpool = MIN(SBC_EDR_MAX_BITPOOL, sbc->max_bitpool);
	else
		sbc->max_bitpool = MIN(SBC_BR_MAX_BITPOOL, sbc->max_bitpool);

	/* Make sure min bitpool is not larger than max bitpool */
	sbc->min_bitpool = MIN(sbc->min_bitpool, sbc->max_bitpool);
}

void ste_qos_init(void)
{
	device_create_role_switch_blacklist();
}

void ste_qos_exit(void)
{
	device_free_role_switch_blacklist();
}
