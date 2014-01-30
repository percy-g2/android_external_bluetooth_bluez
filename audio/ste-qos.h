/*
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  Copyright (C) 2010-2011  ST-Ericsson-2011 SA
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

struct ste_qos_params {
	/* VS Extended Flow Specification Parameters */
	uint16_t service_interval;
	uint16_t out_service_window;
	uint16_t in_service_window;
	uint8_t cqae;
	uint16_t packet_size;
	/* Flow Specification Parameters */
	uint16_t token_bucket_size;
	uint16_t token_rate;
	uint16_t access_latency;
	uint16_t peak_bandwidth;
	uint8_t direction;
	uint8_t service_type;
	uint8_t flags;
};

struct avdtp;
struct avdtp_stream;

void ste_qos_enable( struct btd_device *device, struct avdtp_stream *stream);
void ste_qos_disable(struct btd_device *device);

void ste_qos_clip_sbc_cap(struct btd_device *device, void *cap);

void ste_qos_init(void);
void ste_qos_exit(void);
