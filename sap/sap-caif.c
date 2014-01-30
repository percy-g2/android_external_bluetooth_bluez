/*
 *  BlueZ - Bluetooth protocol stack for Linux
 *
 *  SAP Driver for ST-Ericsson platform using CAIF AT channel
 *
 *  Copyright (C) 2012 ST-Ericsson SA
 *
 *  Author: Tieto Poland BT TEAM.
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

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <glib.h>
#include <string.h>
#include <linux/caif/caif_socket.h>

#include "log.h"
#include "sap.h"

#ifdef STE_SAP_DEBUG
#define DBG_VERBOSE(fmt, arg...) DBG(fmt, arg)
#else
#define DBG_VERBOSE(fmt...)
#endif

#define sap_error(fmt, arg...) do { \
	error("STE caif SAP: " fmt, ## arg); \
	} while (0)

#define sap_info(fmt, arg...) do { \
	info("STE caif SAP: " fmt, ## arg); \
	} while (0)

/* Card reader status bits as described in GSM 11.14, Section 12.33
 * Bits 0-2 are for card reader identity and always zeros. */
#define ICC_READER_REMOVABLE	(1 << 3)
#define ICC_READER_PRESENT	(1 << 4)
#define ICC_READER_ID1		(1 << 5)
#define ICC_READER_CARD_PRESENT	(1 << 6)
#define ICC_READER_CARD_POWERED	(1 << 7)

/* SAP backend's routine cleanup timeout */
#define CLEANUP_TIMEOUT 15

#define MODEM_READY "\r\n*EMRDY:"
#define AT_OK "\r\nOK"
#define AT_ERROR "\r\nERROR"
#define AT_STATUS_IND "\r\n*ESAPSTATUSU:"

#define MAX_APDU_BYTES 256
#define MAX_APDU_CHARS (MAX_APDU_BYTES * 2)
#define MAX_ATR_BYTES 33
#define MAX_ATR_CHARS (MAX_APDU_CHARS * 2)

enum ste_state {
	STE_DISABLED,		/* Reader not present or removed */
	STE_POWERED_OFF,	/* Card in the reader but powered off */
	STE_NO_CARD,		/* No card in the reader */
	STE_ENABLED,		/* Card in the reader and powered on */
	STE_SIM_BUSY,		/* Modem is busy with ongoing call.*/
	STE_STATE_MAX
};

enum ste_status {
	STE_STATUS_OK		= 0x00000000,
	STE_STATUS_FAILURE	= 0x00000001,
	STE_STATUS_BUSY_CALL	= 0x00000002
};

enum caif_command {
	ESAP_APDU = 0,
	ESAP_ATR = 1,
	ESAP_CTRL_POWEROFF = 2,
	ESAP_CTRL_POWERON = 3,
	ESAP_CTRL_RESET = 4,
	ESAP_CRSTATUS = 5, /* get card reader status */
	ESAP_CONNECT = 6,
	ESAP_DISCONNECT = 7,
	ESAP_SUBSCRIBE = 8, /* subscribe to get card indications */
	ESAP_SUBSCRIBE_CHECK = 9,
	ESAP_UNSUBSCRIBE = 10, /* unsubscribe to get card indications */
	ESAP_ESIMR = 11, /* reset sim when sap is disconnected */
	ESAP_MAX
};

struct ste_caif {
	GIOChannel *io;
	GSList *req_list;
	enum ste_state state;
	enum caif_command req;
	void *sap_data;
	unsigned short bt_linklost;
	unsigned int timer_id;
	int (*event)(void *);
};

struct wait_req {
	enum caif_command req_id;
	char *req_at_buf;
};

typedef void (*rsp_handler)(void *, gsize);

static struct ste_caif caif;

static const uint8_t sim2sap_result[ESAP_MAX][STE_STATE_MAX] = {
	/* SAP results for SEND APDU message */
	{
		SAP_RESULT_ERROR_NOT_ACCESSIBLE,	/* STE_DISABLED */
		SAP_RESULT_ERROR_POWERED_OFF,		/* STE_POWERED_OFF */
		SAP_RESULT_ERROR_CARD_REMOVED,		/* STE_NO_CARD */
		SAP_RESULT_ERROR_NO_REASON		/* STE_ENABLED */
	},

	/* SAP results for GET_ATR message */
	{
		SAP_RESULT_ERROR_NO_REASON,
		SAP_RESULT_ERROR_POWERED_OFF,
		SAP_RESULT_ERROR_CARD_REMOVED,
		SAP_RESULT_ERROR_NO_REASON
	},

	/* SAP results POWER OFF message */
	{
		SAP_RESULT_ERROR_NO_REASON,
		SAP_RESULT_ERROR_POWERED_OFF,
		SAP_RESULT_ERROR_CARD_REMOVED,
		SAP_RESULT_ERROR_NO_REASON
	},

	/* SAP results POWER ON message */
	{
		SAP_RESULT_ERROR_NO_REASON,
		SAP_RESULT_ERROR_NOT_ACCESSIBLE,
		SAP_RESULT_ERROR_CARD_REMOVED,
		SAP_RESULT_ERROR_POWERED_ON
	},

	/* SAP results SIM RESET message */
	{
		SAP_RESULT_ERROR_NO_REASON,
		SAP_RESULT_ERROR_POWERED_OFF,
		SAP_RESULT_ERROR_CARD_REMOVED,
		SAP_RESULT_ERROR_NOT_ACCESSIBLE
	},

	/* SAP results GET STATUS message */
	{
		SAP_RESULT_ERROR_NO_REASON,
		SAP_RESULT_ERROR_NO_REASON,
		SAP_RESULT_ERROR_NO_REASON,
		SAP_RESULT_ERROR_NO_REASON
	}
};


static uint8_t get_sap_result(enum caif_command msg, uint32_t status)
{
	if (!caif.io)
		return SAP_RESULT_ERROR_NO_REASON;

	switch (status) {
	case STE_STATUS_OK:
		return SAP_RESULT_OK;

	case STE_STATUS_FAILURE:
		return sim2sap_result[msg][caif.state];

	default:
		DBG("Can't convert a result (status %u)", status);
		return SAP_RESULT_ERROR_NO_REASON;
	}
}

static uint8_t get_sap_status_change(void *msg)
{
	int result;
	unsigned int status;
	DBG("");

	if (!caif.io)
		return SAP_STATUS_CHANGE_UNKNOWN_ERROR;

	result = sscanf(msg, "%*[^:]:%u", &status);

	/* SAP status indication format mismatch */
	if (1 != result)
		return SAP_STATUS_CHANGE_UNKNOWN_ERROR;

	/* handle valid SAP indication data
	   check error status */
	switch (status) {
	case SAP_STATUS_CHANGE_CARD_RESET:
		caif.state = STE_ENABLED;
		return SAP_STATUS_CHANGE_CARD_RESET;
	case SAP_STATUS_CHANGE_CARD_REMOVED:
		caif.state = STE_NO_CARD;
		return SAP_STATUS_CHANGE_CARD_REMOVED;
	case SAP_STATUS_CHANGE_CARD_INSERTED:
		caif.state = STE_DISABLED;
		return SAP_STATUS_CHANGE_CARD_INSERTED;
	case SAP_STATUS_CHANGE_CARD_NOT_ACCESSIBLE:
		caif.state = STE_DISABLED;
		return SAP_STATUS_CHANGE_CARD_NOT_ACCESSIBLE;
	case SAP_STATUS_CHANGE_CARD_RECOVERED:
		caif.state = STE_DISABLED;
		return SAP_STATUS_CHANGE_CARD_RECOVERED;
	default:
		DBG("Can't convert status change %u", status);
		return SAP_STATUS_CHANGE_UNKNOWN_ERROR;

	}
	return SAP_STATUS_CHANGE_UNKNOWN_ERROR;
}

inline static char *req2str(enum caif_command req)
{
	switch (req) {
	case ESAP_CONNECT:
		return "AT*ESAPCONNECT";
	case ESAP_DISCONNECT:
		return "AT*ESAPDISCONNECT";
	case ESAP_CTRL_POWERON:
	case ESAP_CTRL_POWEROFF:
	case ESAP_CTRL_RESET:
		return "AT*ESAPCTRL";
	case ESAP_CRSTATUS:
		return "AT*ESAPCRSTATUS";
	case ESAP_APDU:
		return "AT*ESAPAPDU";
	case ESAP_ATR:
		return "AT*ESAPATR";
	case ESAP_SUBSCRIBE:
	case ESAP_UNSUBSCRIBE:
	case ESAP_SUBSCRIBE_CHECK:
		return "AT*ESAPSTATUS";
	case ESAP_ESIMR:
		return "AT*ESIMR";
	default:
		return NULL;
	}
	return NULL;
}

static int send_message(GIOChannel *io, void *buf, size_t size)
{
	gsize written;
	DBG("io %p, size %zu", io, size);

	if (g_io_channel_write_chars(io, buf, size, &written, NULL) !=
							G_IO_STATUS_NORMAL)
		return -EIO;

	return written;
}

static int send_request(GIOChannel *io, enum caif_command req,
		struct sap_parameter *param, uint16_t *maxmsgsize,
		struct wait_req *req_waiting)
{
	int ret;
	char *buf = req2str(req);
	size_t len = strlen(buf);
	char *at_req, append[4] = {0};
	struct wait_req *req_buffered;

	if (!caif.io) {
		DBG("no caif channel");
		return -EIO;
	}

	if (req_waiting) {
		at_req = req_waiting->req_at_buf;
		req = req_waiting->req_id;
		DBG_VERBOSE("get queued request (%p), %u, %s",
				req_waiting, req, at_req);
	} else {
		if (!buf) {
			DBG("invalid request");
			return -EIO;
		}

		switch (req) {
			case ESAP_CONNECT:
				sprintf(append, "=%hu", *maxmsgsize);
				break;
			case ESAP_CTRL_POWERON:
				sprintf(append, "=1");
				break;
			case ESAP_CTRL_POWEROFF:
				sprintf(append, "=0");
				break;
			case ESAP_CTRL_RESET:
				sprintf(append, "=2");
				break;
			case ESAP_APDU:
				sprintf(append, "=");
				break;
			case ESAP_SUBSCRIBE:
				sprintf(append, "=1");
				break;
			case ESAP_SUBSCRIBE_CHECK:
				sprintf(append, "?");
				break;
			case ESAP_UNSUBSCRIBE:
				sprintf(append, "=0");
				break;
			case ESAP_ESIMR:
				sprintf(append, "=0");
				break;
			default:
				DBG_VERBOSE("request without args");
				break;
		}

		len += strlen(append);
		if (req == ESAP_APDU && param)
			len += param->len;

		len += 3; /* <AT*cmd>\r\n'\0' */
		at_req = g_malloc0(sizeof(char)*len);

		if (req == ESAP_APDU)
			sprintf(at_req, "%s%s\r\n", buf, (char *)param->val);
		else
			sprintf(at_req, "%s%s\r\n", buf, append);

		DBG_VERBOSE("command: %s", at_req);

		if (ESAP_MAX > caif.req) {
			/* add AT command to queue, another one is executed */
			req_buffered = g_malloc0(sizeof(struct wait_req));
			req_buffered->req_id = req;
			req_buffered->req_at_buf = at_req;
			caif.req_list = g_slist_append(caif.req_list, req_buffered);
			DBG_VERBOSE("request %s (%u) queued (%p)",
					req2str(req), req, caif.req_list->data);
			return 0;
		}
	}

	caif.req = req;
	ret = send_message(io, at_req, strlen(at_req)+1);

	if (ret < 0) {
		sap_error("sending request failed: %s", strerror(-ret));
	} else if (ret != (int) strlen(at_req)+1) {
		sap_error("sending request failed: %d out of %zu bytes sent",
								ret, len);
		ret = -EIO;
	}

	g_free(at_req);
	return ret;
}

static int rsp_status_ind(void *msg)
{
	if (!caif.event)
		return -EIO;

	if (strncasecmp(msg, AT_STATUS_IND, strlen(AT_STATUS_IND)) != 0)
		return -EIO;

	DBG("");
	return sap_status_ind(caif.sap_data, get_sap_status_change(msg));
}

static void rsp_control(void *msg, gsize len)
{
	int result = -1;
	unsigned int status = 0xFFFF;
	DBG("");

	result = sscanf(msg, "%*[^:]:%u", &status);

	/* response to SAP Control request invalid */
	if (1 != result) {
		DBG_VERBOSE("unexpected result (%u)", result);
		return;
	}

	if (SAP_RESULT_NOT_SUPPORTED < status) {
		DBG_VERBOSE("unexpected status (%u)", status);
		return;
	}

	/* response format matched, check status */
	switch (caif.req) {
	case ESAP_CTRL_POWERON:
		switch (status) {
		case SAP_RESULT_OK:
			caif.state = STE_ENABLED;
			break;
		case SAP_RESULT_ERROR_NO_REASON:
		case SAP_RESULT_ERROR_NOT_ACCESSIBLE:
		case SAP_RESULT_ERROR_POWERED_ON:
		case SAP_RESULT_ERROR_CARD_REMOVED:
			break;
		default:
			DBG_VERBOSE("power on: invalid status %u", status);
			status = SAP_RESULT_ERROR_NO_REASON;
			break;
		}
		sap_power_sim_on_rsp(caif.sap_data, status);
		break;
	case ESAP_CTRL_POWEROFF:
		switch (status) {
		case SAP_RESULT_OK:
			caif.state = STE_POWERED_OFF;
			break;
		case SAP_RESULT_ERROR_NO_REASON:
		case SAP_RESULT_ERROR_POWERED_OFF:
		case SAP_RESULT_ERROR_CARD_REMOVED:
			break;
		default:
			DBG_VERBOSE("power off: invalid status %u", status);
			status = SAP_RESULT_ERROR_NO_REASON;
			break;
		}
		sap_power_sim_off_rsp(caif.sap_data, status);
		break;
	case ESAP_ESIMR:
		switch (status) {
		case SAP_RESULT_OK:
			caif.state = STE_ENABLED;
			break;
		case SAP_RESULT_ERROR_NO_REASON:
		case SAP_RESULT_ERROR_NOT_ACCESSIBLE:
		case SAP_RESULT_ERROR_POWERED_OFF:
		case SAP_RESULT_ERROR_CARD_REMOVED:
			break;
		default:
			DBG_VERBOSE("reset: invalid status %u",
							status);
			status = SAP_RESULT_ERROR_NO_REASON;
			break;
		}
		sap_reset_sim_rsp(caif.sap_data, status);
		break;
	case ESAP_CTRL_RESET:
		switch (status) {
		case SAP_RESULT_OK:
			caif.state = STE_ENABLED;
			break;
		case SAP_RESULT_ERROR_NO_REASON:
		case SAP_RESULT_ERROR_NOT_ACCESSIBLE:
		case SAP_RESULT_ERROR_POWERED_OFF:
		case SAP_RESULT_ERROR_CARD_REMOVED:
			break;
		default:
			DBG_VERBOSE("reset: invalid status %u",
							status);
			status = SAP_RESULT_ERROR_NO_REASON;
			break;
		}
		sap_reset_sim_rsp(caif.sap_data, status);
		break;
	default:
		DBG("handler called in no at_ctrl command context!");
		break;
	}
}

static void rsp_card_status(void *msg, gsize len)
{
	uint8_t crs = 0;
	int result = -1;
	unsigned int status = 0xFFFF;
	unsigned int cr_removable, cr_present, cr_id1, card_present, card_powered;

	DBG("");

	result = sscanf(msg, "%*[^:]:%u,%u,%u,%u,%u,%u",
			&status, &cr_removable, &cr_present, &cr_id1,
			&card_present, &card_powered);

	switch (result) {
	/* all tokens matched in getting SAP Card Reader status */
	case 6:
		switch (status) {
		case SAP_RESULT_OK:
			if (cr_removable)
				crs |= ICC_READER_REMOVABLE;
			if (cr_present)
				crs |= ICC_READER_PRESENT;
			if (cr_id1)
				crs |= ICC_READER_ID1;
			if (card_present)
				crs |= ICC_READER_CARD_PRESENT;
			if (card_powered)
				crs |= ICC_READER_CARD_POWERED;
			DBG_VERBOSE("rsp status %u, crs %#hx", status, crs);
			break;
		default:
			if (status != SAP_RESULT_ERROR_NO_REASON &&
					status != SAP_RESULT_ERROR_NO_DATA)
				status = SAP_RESULT_ERROR_NO_REASON;
			DBG_VERBOSE("rsp status %#x", status);
			break;
		}
		break;
	/* response to SAP Card Reader request contains
	   only status, check error status */
	case 1:
		if (status != SAP_RESULT_ERROR_NO_REASON &&
				status != SAP_RESULT_ERROR_NO_DATA)
			status = SAP_RESULT_ERROR_NO_REASON;
		DBG_VERBOSE("rsp status %#x", status);
		break;
	default:
		status = SAP_RESULT_ERROR_NO_REASON;
		DBG_VERBOSE("rsp status %#x", status);
		break;
	}

	sap_transfer_card_reader_status_rsp(caif.sap_data, status, crs);
}

inline static void hexstr2byte(const char *hex, int len, uint8_t *tab)
{
	int i, j = 0;
	for (i = 0; i < len; i++) {
		if (EOF != sscanf(hex+j, "%2hhx", &tab[i]))
			j += 2;
	}
}

static void rsp_apdu(void *msg, gsize len)
{
	int result = -1;
	unsigned int status = 0xFFFF;
	char buf_hexchar_adpu[MAX_APDU_CHARS];
	uint8_t buf_bytes[MAX_APDU_BYTES];
	DBG("");

	result = sscanf(msg, "%*[^:]:%u,\"%[A-Fa-f0-9]\"",
			&status, buf_hexchar_adpu);

	switch (result) {
	/* SAP APDU response format matched */
	case 2:
		if (status == SAP_RESULT_OK)
			hexstr2byte(buf_hexchar_adpu, MAX_APDU_BYTES, buf_bytes);
		break;
	/* SAP APDU response contains only status
	   check error status */
	case 1:
		switch (status) {
		case SAP_RESULT_ERROR_NO_REASON:
		case SAP_RESULT_ERROR_NOT_ACCESSIBLE:
		case SAP_RESULT_ERROR_POWERED_OFF:
		case SAP_RESULT_ERROR_CARD_REMOVED:
			break;
		default:
			DBG("Invalid reader status %u", status);
			status = SAP_RESULT_ERROR_NO_REASON;
			break;
		}
		break;
	default:
		status = get_sap_result(ESAP_APDU, STE_STATUS_FAILURE);
		break;
	}

	sap_transfer_apdu_rsp(caif.sap_data, status, buf_bytes,
			MAX_APDU_BYTES);
}

static void caif_set_events(enum caif_command cmd)
{
	switch (cmd) {
	case ESAP_SUBSCRIBE:
		if (!caif.event &&
			send_request(caif.io, cmd, NULL, NULL, NULL) < 0) {
			sap_error("subscribe to sim events failed");
		} else {
			caif.event = rsp_status_ind;
			DBG_VERBOSE("%s", "events subscription indicator set");
		}
		break;
	case ESAP_SUBSCRIBE_CHECK:
		if (caif.event) {
			if (send_request(caif.io, cmd, NULL, NULL, NULL) < 0) {
				sap_error("checking subscribe to sim events failed");
			} else {
				DBG_VERBOSE("%s",
						"checking subscription status to sim events");
			}
		}
		break;
	default:
		if (caif.event && send_request(caif.io,
				ESAP_UNSUBSCRIBE, NULL, NULL, NULL) < 0) {
			sap_error("unsubscribe to sim events failed");
		} else {
			caif.event = NULL;
			DBG_VERBOSE("%s",
					"events subscription indicator reset");
		}
		break;
	}
}

static void caif_data_reset(void)
{
	GSList *list = NULL;
	DBG("io %p", caif.io);

	if (!caif.io)
		return;

	caif.state = STE_DISABLED;
	caif.sap_data = NULL;
	caif.event = NULL;
	caif.bt_linklost = 0;

	for (list = caif.req_list; list; list = list->next) {
		g_free(((struct wait_req*)(list->data))->req_at_buf);
		g_free(list->data);
	}

	g_slist_free(caif.req_list);
	caif.req_list = NULL;
	g_io_channel_shutdown(caif.io, TRUE, NULL);
	g_io_channel_unref(caif.io);
	caif.io = NULL;
}

static gboolean caif_cleanup_timeout(gpointer data)
{
	DBG("%p", data);
	if (!caif.io)
		return FALSE;

	caif.timer_id = 0;
	caif_data_reset();

	return FALSE;
}

static void caif_cleanup_reset_timer()
{
	DBG("");

	if (caif.timer_id &&
		g_source_remove(caif.timer_id))
			caif.timer_id = 0;
}

static void caif_cleanup_timer(guint interval, void *data)
{
	DBG("%p", data);

	if (!caif.io)
		return;

	if (!caif.timer_id) {
		caif.timer_id =
			g_timeout_add_seconds(interval, caif_cleanup_timeout, data);

		if (caif.state == STE_POWERED_OFF)
			send_request(caif.io, ESAP_CTRL_POWERON, NULL, NULL, NULL);

		caif_set_events(ESAP_UNSUBSCRIBE);

		if (send_request(caif.io,
			ESAP_DISCONNECT, NULL, NULL, NULL) < 0) {
				sap_error("request failed, continue cleaning");
				caif_cleanup_reset_timer();
		}
	} else {
		error("Timer is already active.");
	}
	if(send_request(caif.io,
			ESAP_CTRL_RESET, NULL, NULL, NULL) < 0) {
				sap_error("request failed, continue cleaning");
				caif_cleanup_reset_timer();
	}
	else {
	/* reset sim, to transfer newtwork back to board*/
	send_request(caif.io,
			ESAP_ESIMR, NULL, NULL, NULL);
	}
}

static void caif_cleanup_data_cb(gpointer data)
{
	DBG("");
	g_free(data);
	caif_cleanup_reset_timer();
	caif_data_reset();
}

static void rsp_connect(void *msg, gsize len)
{
	int res = -1;
	unsigned int conn_status;
	unsigned int maxmsg;

	res = sscanf(msg, "%*[^:]:%u,%u", &conn_status, &maxmsg);
	DBG_VERBOSE("result %d, status %u", res, conn_status);

	/* status part only matched in response to SAP Connect request
	   check it, 0 means success */
	if (1 == res && SAP_STATUS_OK == conn_status) {
		sap_connect_rsp(caif.sap_data, SAP_STATUS_OK, 0);
		caif.state = STE_ENABLED;
		sap_status_ind(caif.sap_data, SAP_STATUS_CHANGE_CARD_RESET);
	/* SAP msg size still isn't negotiated in last CONNECT request
	   check status and supported SAP msg size */
	} else if (2 == res &&
			SAP_STATUS_MAX_MSG_SIZE_NOT_SUPPORTED == conn_status) {
		DBG_VERBOSE("max supported msg size = %d", maxmsg);
		sap_connect_rsp(caif.sap_data,
				SAP_STATUS_MAX_MSG_SIZE_NOT_SUPPORTED, maxmsg);
	} else if (2 == res &&
			SAP_STATUS_MAX_MSG_SIZE_TOO_SMALL == conn_status) {
		DBG_VERBOSE("max supported msg size = %d", maxmsg);
		sap_connect_rsp(caif.sap_data,
				SAP_STATUS_MAX_MSG_SIZE_TOO_SMALL, maxmsg);
	} else {
		sap_connect_rsp(caif.sap_data,
				SAP_STATUS_CONNECTION_FAILED, 0);
		caif_data_reset();
	}
	caif.req = ESAP_MAX;
}

static void rsp_disconnect(void *msg, gsize len)
{
	DBG("%p", caif.sap_data);

	if (caif.sap_data)
		sap_disconnect_rsp(caif.sap_data);
}

static void rsp_subscribe(void *msg, gsize len)
{
	DBG("");
}

static void rsp_atr(void *msg, gsize len)
{
	int res = -1;
	unsigned int atr_status;
	char buf_hexchar_atr[MAX_ATR_CHARS];
	uint8_t buf_bytes[MAX_ATR_BYTES];

	DBG("");

	res = sscanf(msg, "%*[^:]:%u,\"%[0-9a-fA-F]\"",
			&atr_status, buf_hexchar_atr);
	/* ATR response format matched
	   validate status and response buffer */
	if (2 == res && STE_STATUS_OK == atr_status) {
		DBG_VERBOSE("atr buf %s", buf_hexchar_atr);
		hexstr2byte(buf_hexchar_atr, MAX_ATR_BYTES, buf_bytes);
		sap_transfer_atr_rsp(caif.sap_data,
				atr_status, buf_bytes, MAX_ATR_BYTES);
		caif.state = STE_ENABLED;
	} else {
		sap_transfer_atr_rsp(caif.sap_data,
				STE_STATUS_FAILURE, NULL, 0);
	}
}

static void caif_cleanup(void *data)
{
	DBG("");
	if (caif.state != STE_DISABLED || caif.bt_linklost )
		caif_cleanup_timer(CLEANUP_TIMEOUT, data);
	else
		caif_data_reset();

}

void sap_disconnect_req(void *sap_device, uint8_t linkloss)
{
	sap_info("disconnect request %s", linkloss ? "by link loss" : "");
	DBG("sap_device %p linkloss %u", sap_device, linkloss);

	if (!linkloss)
		sap_disconnect_rsp(sap_device);
	else
		caif.bt_linklost = 1;

	caif_cleanup(sap_device);
}

void sap_transfer_apdu_req(void *sap_device, struct sap_parameter *param)
{
	uint8_t result;

	DBG_VERBOSE("sap_device %p param %p", sap_device, param);

	if (caif.state != STE_ENABLED) {
		result = get_sap_result(ESAP_APDU, STE_STATUS_FAILURE);
		sap_transfer_apdu_rsp(sap_device, result, NULL, 0);
		return;
	}

	if (send_request(caif.io, ESAP_APDU, param, NULL, NULL) < 0)
		sap_transfer_apdu_rsp(sap_device,
				SAP_RESULT_ERROR_NO_DATA, NULL, 0);
}

void sap_transfer_atr_req(void *sap_device)
{
	uint8_t result;
	DBG_VERBOSE("sap_device %p", sap_device);

	if (caif.state != STE_ENABLED) {
		result = get_sap_result(ESAP_ATR, STE_STATUS_FAILURE);
		sap_transfer_atr_rsp(sap_device, result, NULL, 0);
		return;
	}

	if (send_request(caif.io, ESAP_ATR, NULL, NULL, NULL) < 0)
		sap_transfer_atr_rsp(sap_device,
			SAP_RESULT_ERROR_NO_DATA, NULL, 0);
}

void sap_power_sim_off_req(void *sap_device)
{
	uint8_t result;
	DBG_VERBOSE("sap_device %p", sap_device);

	if (caif.state != STE_ENABLED) {
		result = get_sap_result(ESAP_CTRL_POWEROFF, STE_STATUS_FAILURE);
		sap_power_sim_off_rsp(sap_device, result);
		return;
	}

	if (send_request(caif.io, ESAP_CTRL_POWEROFF, NULL, NULL, NULL) < 0)
		sap_power_sim_off_rsp(sap_device, SAP_RESULT_ERROR_NO_REASON);
}

void sap_power_sim_on_req(void *sap_device)
{
	uint8_t result;
	DBG_VERBOSE("sap_device %p", sap_device);

	if (caif.state != STE_POWERED_OFF) {
		result = get_sap_result(ESAP_CTRL_POWERON, STE_STATUS_FAILURE);
		sap_power_sim_on_rsp(sap_device, result);
		return;
	}

	if (send_request(caif.io, ESAP_CTRL_POWERON, NULL, NULL, NULL) < 0)
		sap_power_sim_on_rsp(sap_device, SAP_RESULT_ERROR_NO_REASON);
}

void sap_reset_sim_req(void *sap_device)
{
	uint8_t result;
	DBG_VERBOSE("sap_device %p", sap_device);

	if (caif.state != STE_ENABLED) {
		result = get_sap_result(ESAP_CTRL_RESET, STE_STATUS_FAILURE);
		sap_reset_sim_rsp(sap_device, result);
		return;
	}

	if (send_request(caif.io, ESAP_CTRL_RESET, NULL, NULL, NULL) < 0)
		sap_reset_sim_rsp(sap_device, SAP_RESULT_ERROR_NO_REASON);
}

void sap_transfer_card_reader_status_req(void *sap_device)
{
	uint8_t result;
	DBG_VERBOSE("sap_device %p", sap_device);

	if (caif.state == STE_DISABLED) {
		result = get_sap_result(ESAP_CRSTATUS, STE_STATUS_FAILURE);
		sap_transfer_card_reader_status_rsp(sap_device, result, 0);
		return;
	}

	if (send_request(caif.io, ESAP_CRSTATUS, NULL, NULL, NULL) < 0)
		sap_transfer_card_reader_status_rsp(sap_device,
						SAP_RESULT_ERROR_NO_DATA, 0);
}

void sap_set_transport_protocol_req(void *sap_device,
						struct sap_parameter *param)
{
	DBG_VERBOSE("sap_device %p", sap_device);
	sap_transport_protocol_rsp(sap_device, SAP_RESULT_NOT_SUPPORTED);
}

static rsp_handler responses[ESAP_MAX] = {
	rsp_apdu,
	rsp_atr,
	rsp_control,
	rsp_control,
	rsp_control,
	rsp_card_status,
	rsp_connect,
	rsp_disconnect,
	rsp_subscribe,
	rsp_subscribe,
	rsp_subscribe
};

static gboolean caif_data_cb(GIOChannel *io, GIOCondition cond, gpointer data)
{
	char buf[SAP_BUF_SIZE] = {0};
	gsize bytes_read;
	GIOStatus gstatus;
	struct wait_req *req;

	if (cond & (G_IO_NVAL | G_IO_HUP | G_IO_ERR)) {
		DBG_VERBOSE("exiting 0x%x",
				cond & (G_IO_NVAL | G_IO_HUP | G_IO_ERR));
		caif_cleanup_data_cb(data);
		return FALSE;
	}

	gstatus = g_io_channel_read_chars(io, buf, sizeof(buf), &bytes_read, NULL);
	if (G_IO_STATUS_NORMAL != gstatus) {
		sap_error("Error (%d) while reading from channel", gstatus);
		return TRUE;
	}

	DBG_VERBOSE("tid %u, bt_linklost %hu, read: %zu buf: %s",
			caif.timer_id, caif.bt_linklost, (size_t)bytes_read, buf);

	if (0 == strncasecmp(buf, AT_ERROR, strlen(AT_ERROR))) {
		DBG_VERBOSE("%s", "ERROR");
		caif_cleanup_data_cb(data);
		return TRUE;
	}

	if (0 == strncasecmp(buf, MODEM_READY, strlen(MODEM_READY))) {
		sap_info("get *EMRDY");
		return TRUE;
	}

	/* handle returned echo responses since echo isn't turn off */
	if (ESAP_MAX > caif.req &&
		strncasecmp(buf, req2str(caif.req),
				strlen(req2str(caif.req))) == 0)
		return TRUE;

	if (0 == strncasecmp(buf, AT_OK, strlen(AT_OK))) {
		caif.req = ESAP_MAX;
		DBG_VERBOSE("%s", "get OK");
		if (NULL != caif.req_list && NULL != (req = caif.req_list->data)) {
			send_request(io, req->req_id, NULL, NULL, req);
			g_free(req);
			caif.req_list = g_slist_remove(caif.req_list, req);
			return TRUE;
		}
	}

	if (caif.bt_linklost) {
		DBG_VERBOSE("%s", "bt_linklost");
		return TRUE;
	}

	if (g_slist_length(caif.req_list)) {
		DBG_VERBOSE("%s", "queue not empty");
		return TRUE;
	}

	if (caif.timer_id) {
		caif_cleanup_data_cb(data);
		return TRUE;
	}

	if (caif.event)
		caif.event(buf);

	if (ESAP_MAX > caif.req)
		responses[caif.req](buf, bytes_read);

	return TRUE;
}

static void caif_watch(int sock, void *sap_data)
{
	GIOChannel *io;
	DBG("sock %d, sap_data %p ", sock, sap_data);
	io = g_io_channel_unix_new(sock);
	g_io_channel_set_encoding(io, NULL, NULL);
	g_io_channel_set_buffered(io, FALSE);

	caif.io = io;
	caif.sap_data = sap_data;
	caif.state = STE_DISABLED;
	caif.event = NULL;
	caif.req = ESAP_MAX;
	caif.bt_linklost = 0;
	caif.timer_id = 0;

	g_io_add_watch_full(io, G_PRIORITY_DEFAULT,
			G_IO_IN | G_IO_ERR | G_IO_HUP | G_IO_NVAL,
			caif_data_cb, NULL, NULL);

	caif_set_events(ESAP_SUBSCRIBE);

	DBG_VERBOSE("io %p, refcount %u", caif.io, io->ref_count);
}

static int caif_connect(void *sap_data)
{
	int cf_prio = CAIF_PRIO_NORMAL;
	int modem_fd;
	int err;

	static struct sockaddr_caif cf_addr = {
		.family = AF_CAIF,
		.u.at.type = CAIF_ATTYPE_PLAIN
	};

	sap_info("connect to caif AT channel");

	if (caif.io)
		return -EALREADY;

	modem_fd = socket(PF_CAIF, SOCK_SEQPACKET, CAIFPROTO_AT);
	if (modem_fd < 0) {
		err = errno;
		sap_error("creating socket failed: %s", strerror(err));
		return -err;
	}
	if (setsockopt(modem_fd, SOL_SOCKET, SO_PRIORITY, &cf_prio,
				sizeof(cf_prio)) < 0)
		sap_error("Not able to set socket priority. Errno:%d, %s",
				errno, strerror(errno));
	if (connect(modem_fd, (struct sockaddr *)&cf_addr,
				sizeof(cf_addr)) < 0) {
		err = errno;
		sap_error("connect to the socket failed: %s", strerror(err));
		goto failed;
	}
	if (fcntl(modem_fd, F_SETFL, O_NONBLOCK) > 0) {
		err = errno;
		sap_error("setting up socket failed: %s", strerror(err));
		goto failed;
	}

	caif_watch(modem_fd, sap_data);
	return 0;

failed:
	close(modem_fd);
	return -err;
}

void sap_connect_req(void *sap_device, uint16_t maxmsgsize)
{
	sap_info("connect request");
	DBG("sap_device %p maxmsgsize %u", sap_device, maxmsgsize);
	caif.bt_linklost = 0;

	if (!caif.io && caif_connect(sap_device) < 0) {
		sap_connect_rsp(sap_device,
				SAP_STATUS_CONNECTION_FAILED, SAP_BUF_SIZE);
		return;
	}

	if (sap_device != caif.sap_data) {
		caif_data_reset();
		sap_connect_rsp(sap_device,
				SAP_STATUS_CONNECTION_FAILED, SAP_BUF_SIZE);
		return;
	}

	if (send_request(caif.io, ESAP_CONNECT, NULL, &maxmsgsize, NULL) < 0) {
		sap_connect_rsp(sap_device,
				SAP_STATUS_CONNECTION_FAILED, SAP_BUF_SIZE);
		caif_data_reset();
	}
}

int sap_init(void)
{
	caif.req = ESAP_MAX;
	caif.state = STE_DISABLED;
	info("STE SAP caif driver initialized");
	return 0;
}

void sap_exit(void)
{
}
