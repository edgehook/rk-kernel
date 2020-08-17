/** @file mlan_diag.c
 *
 *  @brief This file contains the handling of diag loopback code.
 *
 *  Copyright (C) 2008-2018, Marvell International Ltd.
 *
 *  This software file (the "File") is distributed by Marvell International
 *  Ltd. under the terms of the GNU General Public License Version 2, June 1991
 *  (the "License").  You may use, redistribute and/or modify this File in
 *  accordance with the terms and conditions of the License, a copy of which
 *  is available by writing to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
 *  worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
 *
 *  THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE
 *  ARE EXPRESSLY DISCLAIMED.  The License provides additional details about
 *  this warranty disclaimer.
 *
 */

/******************************************************
Change log:
    3/25/2009: initial version
******************************************************/

#include "mlan.h"
#include "mlan_decl.h"
#include "mlan_diag.h"
#include "mlan_fw.h"
#include "mlan_util.h"
#include "mlan_join.h"
#include "mlan_main.h"
#include "mlan_sdio.h"

/********************************************************
			Local Variables
********************************************************/

/********************************************************
			Global Variables
********************************************************/

/********************************************************
			Local Functions
********************************************************/

/********************************************************
			Global Functions
********************************************************/

/**
 *  @brief This function sends diag event
 *
 *  @param priv         A pointer to mlan_private structure
 *  @param event_type   Event type
 *  @param pdata        A pointer to data buffer
 *  @return             MLAN_STATUS_SUCCESS
 */
void
wlan_send_diag_event(mlan_private *priv, t_u16 event_type, t_void *pdata)
{
	t_u8 event_buf[100];
	mlan_event *pevent = (mlan_event *)event_buf;
	evt_diag_loopback *diag_event = MNULL;

	ENTER();

	if (!priv) {
		PRINTM(MERROR,
		       "wlan_send_diag_event priv NULL. event_type=%d\n",
		       event_type);
		LEAVE();
		return;
	}

	pevent->bss_index = priv->bss_index;
	pevent->event_id = MLAN_EVENT_ID_DRV_PASSTHRU;
	pevent->event_len = sizeof(event_header) + sizeof(*diag_event);
	((event_header *)(pevent->event_buf))->event_id = EVENT_DIAG_LOOPBACK;
	diag_event =
		(evt_diag_loopback *) (pevent->event_buf +
				       sizeof(event_header));
	diag_event->event_type = event_type;
	diag_event->test_item = priv->adapter->diag_test.current_item;

	if (event_type == EVENT_TYPE_RESULT) {
		memcpy(priv->adapter, &diag_event->body.result_info,
		       (diag_loopback_result_info *) pdata,
		       sizeof(diag_loopback_result_info));
	} else if (event_type == EVENT_TYPE_WRONG_PKT) {
		memcpy(priv->adapter, &diag_event->body.wrong_pkt_info,
		       (diag_loopback_wrong_pkt_info *) pdata,
		       sizeof(diag_loopback_wrong_pkt_info));
	}
	wlan_recv_event(priv, pevent->event_id, pevent);
	LEAVE();
}

/**
 *  @brief This function sends diag loopback command to firmware.
 *
 *  @param pmpriv     	A pointer to mlan_private structure
 *  @param cmd      	A pointer to HostCmd_DS_COMMAND structure
 *  @param cmd_action 	The action: GET or SET
 *  @param pdata_buf	A pointer to data buffer
 *
 *  @return         	MLAN_STATUS_SUCCESS
 */
mlan_status
wlan_cmd_diag_loopback(IN pmlan_private pmpriv,
		       IN HostCmd_DS_COMMAND *cmd,
		       IN t_u16 cmd_action, IN t_void *pdata_buf)
{
	mlan_ds_misc_diag_loopback *pdata_diag_loopback =
		(mlan_ds_misc_diag_loopback *)pdata_buf;
	HostCmd_DS_DIAG_LOOPBACK_TEST *cmd_diag_loopback =
		(HostCmd_DS_DIAG_LOOPBACK_TEST *)&cmd->params.diag_loopback;

	ENTER();

	/* Copy the Diag command to command buffer */
	cmd->command = wlan_cpu_to_le16(HostCmd_CMD_DIAG_LOOPBACK_TEST);
	cmd->size =
		wlan_cpu_to_le16(sizeof(HostCmd_DS_DIAG_LOOPBACK_TEST) +
				 S_DS_GEN);
	cmd_diag_loopback->action =
		wlan_cpu_to_le16(pdata_diag_loopback->action);
	cmd_diag_loopback->test_item =
		wlan_cpu_to_le16(pdata_diag_loopback->test_item);
	cmd_diag_loopback->loopback_size =
		wlan_cpu_to_le16(pdata_diag_loopback->loopback_size);
	cmd_diag_loopback->extra_config =
		wlan_cpu_to_le16(pdata_diag_loopback->extra_config);
	cmd->result = 0;

	LEAVE();

	return MLAN_STATUS_SUCCESS;
}

/**
 * @brief This function handles the command response of diag loopback
 *
 *  @param pmpriv       A pointer to mlan_private structure
 *  @param resp         A pointer to HostCmd_DS_COMMAND
 *  @param pioctl_buf   A pointer to mlan_ioctl_req structure
 *
 *  @return             MLAN_STATUS_SUCCESS
 */
mlan_status
wlan_ret_diag_loopback(IN pmlan_private pmpriv,
		       IN HostCmd_DS_COMMAND *resp,
		       IN mlan_ioctl_req *pioctl_buf)
{
	t_u16 action;
	mlan_adapter *pmadapter = pmpriv->adapter;
	HostCmd_DS_DIAG_LOOPBACK_TEST *cmd_diag_loopback =
		(HostCmd_DS_DIAG_LOOPBACK_TEST *)&resp->params.diag_loopback;

	ENTER();

	if (pioctl_buf) {
		action = wlan_le16_to_cpu(cmd_diag_loopback->action);
		if (action == HostCmd_ACT_TEST_START)
			pmadapter->diag_test.diag_enabled = MTRUE;
		else if (action == HostCmd_ACT_TEST_STOP)
			pmadapter->diag_test.diag_enabled = MFALSE;
	}

	LEAVE();
	return MLAN_STATUS_SUCCESS;
}

/**
 *  @brief send diag loopback cmd
 *
 *  @param pmadapter	A pointer to mlan_adapter structure
 *  @param pioctl_req	A pointer to ioctl request buffer
 *
 *  @return		MLAN_STATUS_PENDING --success, otherwise fail
 */
mlan_status
wlan_misc_ioctl_diag_loopback(IN pmlan_adapter pmadapter,
			      IN pmlan_ioctl_req pioctl_req)
{
	mlan_status ret = MLAN_STATUS_SUCCESS;
	mlan_private *pmpriv = pmadapter->priv[pioctl_req->bss_index];
	mlan_ds_misc_cfg *misc = MNULL;
	t_u32 TXthruput, RXthruput;
	t_u32 TXNum, RXNum, TestTime;
	t_u32 diag_sec = 0, diag_usec = 0;
	diag_loopback_result_info result_info;

	ENTER();

	misc = (mlan_ds_misc_cfg *)pioctl_req->pbuf;

	pmadapter->diag_test.current_item = misc->param.diag_cmd.test_item;
	pmadapter->diag_test.extra_config = misc->param.diag_cmd.extra_config;

	/* Set test data size, add 4 bytes sequence number */
	misc->param.diag_cmd.loopback_size = (t_u16)DIAG_LOOPBACK_DEFAULT_SIZE +
		DIAG_LOOPBACK_SEQNUM_SIZE;
	pmadapter->diag_test.test_size = misc->param.diag_cmd.loopback_size;

	if (misc->param.diag_cmd.action == HostCmd_ACT_TEST_START) {
		/* Diag enabled flag will be set in command response handler */
		pmadapter->callbacks.moal_get_system_time(pmadapter->
							  pmoal_handle,
							  &diag_sec,
							  &diag_usec);
		pmadapter->diag_test.start_time =
			diag_sec * 1000000 + diag_usec;
		pmadapter->diag_test.num_packet_rx = 0;
		pmadapter->diag_test.num_packet_tx = 0;
		pmadapter->diag_test.num_error = 0;
		pmadapter->diag_test.seq_num = 0;

		if (pmadapter->diag_test.current_item == DIAG_TEST_PINGPONG)
			pmadapter->diag_test.ping_back = MTRUE;
	} else if (misc->param.diag_cmd.action == HostCmd_ACT_TEST_STOP) {
		/* Diag enabled flag will be cleared in command response handler */
		pmadapter->callbacks.moal_get_system_time(pmadapter->
							  pmoal_handle,
							  &diag_sec,
							  &diag_usec);
		pmadapter->diag_test.end_time = diag_sec * 1000000 + diag_usec;
		pmadapter->diag_test.diag_enabled = MFALSE;
		if (pmadapter->diag_test.current_item == DIAG_TEST_PINGPONG)
			pmadapter->diag_test.ping_back = MFALSE;

		result_info.num_packet_tx = pmadapter->diag_test.num_packet_tx;
		result_info.num_packet_rx = pmadapter->diag_test.num_packet_rx;
		result_info.num_error = pmadapter->diag_test.num_error;

		/* Calculate the throughput */
		TestTime =
			pmadapter->diag_test.end_time -
			pmadapter->diag_test.start_time;
		TXNum = (result_info.num_packet_tx *
			 pmadapter->diag_test.test_size);
		RXNum = (result_info.num_packet_rx *
			 pmadapter->diag_test.test_size);
		TXthruput = TXNum / (TestTime / 1000);
		RXthruput = RXNum / (TestTime / 1000);

		switch (pmadapter->diag_test.current_item) {
		case DIAG_TEST_TX:	/* TX Test */
			PRINTM(MMSG,
			       "------------------------------------------------------------\n");
			PRINTM(MMSG,
			       "DIAG: TX %u packets (%u bytes) in time %d.%d.%d\n",
			       result_info.num_packet_tx,
			       result_info.num_packet_tx *
			       pmadapter->diag_test.test_size,
			       TestTime / 1000000, (TestTime % 1000000) / 1000,
			       (TestTime % 1000000) % 1000);
			PRINTM(MMSG, "DIAG: TX throughput = %d.%d Mbps\n",
			       (TXthruput * 8) / 1000, (TXthruput * 8) % 1000);
			PRINTM(MMSG,
			       "------------------------------------------------------------\n");
			break;
		case DIAG_TEST_RX:	/* RX Test */
			PRINTM(MMSG,
			       "------------------------------------------------------------\n");
			PRINTM(MMSG,
			       "DIAG: RX %u packets (%u bytes) in time %d.%d.%d\n",
			       result_info.num_packet_rx,
			       result_info.num_packet_rx *
			       pmadapter->diag_test.test_size,
			       TestTime / 1000000, (TestTime % 1000000) / 1000,
			       (TestTime % 1000000) % 1000);
			PRINTM(MMSG, "DIAG: RX throughput = %d.%d Mbps\n",
			       (RXthruput * 8) / 1000, (RXthruput * 8) % 1000);
			PRINTM(MMSG,
			       "------------------------------------------------------------\n");
			break;
		case DIAG_TEST_TXRX:	/* TX/RX Test */
			PRINTM(MMSG,
			       "------------------------------------------------------------\n");
			PRINTM(MMSG,
			       "DIAG: TX %u packets (%u bytes) in time %d.%d.%d\n",
			       result_info.num_packet_tx,
			       result_info.num_packet_tx *
			       pmadapter->diag_test.test_size,
			       TestTime / 1000000, (TestTime % 1000000) / 1000,
			       (TestTime % 1000000) % 1000);
			PRINTM(MMSG, "DIAG: TX throughput = %d.%d Mbps\n",
			       (TXthruput * 8) / 1000, (TXthruput * 8) % 1000);
			PRINTM(MMSG,
			       "DIAG: RX %u packets (%u bytes) in time %d.%d.%d\n",
			       result_info.num_packet_rx,
			       result_info.num_packet_rx *
			       pmadapter->diag_test.test_size,
			       TestTime / 1000000, (TestTime % 1000000) / 1000,
			       (TestTime % 1000000) % 1000);
			PRINTM(MMSG, "DIAG: RX throughput = %d.%d Mbps\n",
			       (RXthruput * 8) / 1000, (RXthruput * 8) % 1000);
			PRINTM(MMSG,
			       "------------------------------------------------------------\n");
			break;
		case DIAG_TEST_PINGPONG:	/* Ping-pong Test */
			PRINTM(MMSG,
			       "------------------------------------------------------------\n");
			PRINTM(MMSG,
			       "DIAG: Ping-pong Test succeed %u packets in time %d.%d.%d\n",
			       result_info.num_packet_rx -
			       result_info.num_error, TestTime / 1000000,
			       (TestTime % 1000000) / 1000,
			       (TestTime % 1000000) % 1000);
			PRINTM(MMSG, "DIAG: Ping-pong Test Error count = %d\n",
			       result_info.num_error);
			PRINTM(MMSG,
			       "------------------------------------------------------------\n");
			break;
		default:
			break;
		}
		/* delay 100ms */
		wlan_send_diag_event(pmpriv, EVENT_TYPE_RESULT,
				     (t_void *)(&result_info));
		misc->param.diag_cmd.loopback_info.event_type =
			EVENT_TYPE_RESULT;
		misc->param.diag_cmd.loopback_info.test_item =
			pmadapter->diag_test.current_item;
		memcpy(pmadapter,
		       &(misc->param.diag_cmd.loopback_info.body.result_info),
		       &result_info, sizeof(result_info));
	}

	/* Send request to firmware */
	ret = wlan_prepare_cmd(pmpriv,
			       HostCmd_CMD_DIAG_LOOPBACK_TEST,
			       0,
			       0,
			       (t_void *)pioctl_req,
			       (t_void *)&misc->param.diag_cmd);
	if (ret == MLAN_STATUS_SUCCESS)
		ret = MLAN_STATUS_PENDING;

	LEAVE();
	return ret;
}

/**
 * @brief This function handles the diag loopback processing in main thread.
 *
 * @param pmadapter Pointer to the mlan_adapter driver data struct
 *
 * @return          None
 */
INLINE void
wlan_process_diag_loopback(pmlan_adapter pmadapter)
{
	mlan_tx_param tx_param;
	mlan_status ret = MLAN_STATUS_SUCCESS;
	t_u16 size = pmadapter->diag_test.test_size;
	mlan_buffer *pmbuf = MNULL;
	t_u8 *tmpbuf = MNULL;
	t_u32 diag_sec = 0, diag_usec = 0, random_count;
	t_u8 data_pattern = 0x5a;

	ENTER();

	while (!pmadapter->data_sent &&
	       pmadapter->diag_test.diag_enabled &&
	       (pmadapter->diag_test.current_item == DIAG_TEST_TX ||
		pmadapter->diag_test.current_item == DIAG_TEST_TXRX ||
		(pmadapter->diag_test.current_item == DIAG_TEST_PINGPONG &&
		 pmadapter->diag_test.ping_back))) {
		/* The buffer address should be 16-byte aligned */
		pmbuf = wlan_alloc_mlan_buffer(pmadapter, (size + 63) & ~31, 0,
					       MOAL_MALLOC_BUFFER);
		if (pmbuf) {

			tmpbuf = pmbuf->pbuf + pmbuf->data_offset;
			/* Set 4-byte sequence number */
			memcpy(pmadapter, tmpbuf + DIAG_LOOPBACK_HEADER_SIZE,
			       &pmadapter->diag_test.seq_num,
			       DIAG_LOOPBACK_SEQNUM_SIZE);

			if (pmadapter->diag_test.current_item ==
			    DIAG_TEST_PINGPONG) {
				/* Raw data length of Ping-pong Test */
				size = MAX(sizeof(TxPD), sizeof(RxPD));
				pmadapter->callbacks.
					moal_get_system_time(pmadapter->
							     pmoal_handle,
							     &diag_sec,
							     &diag_usec);
				random_count = diag_sec * 1000000 + diag_usec;
				size += (t_u16)(random_count %
						(pmadapter->diag_test.
						 test_size -
						 DIAG_LOOPBACK_SEQNUM_SIZE -
						 size));
				/* Data of Ping-pong Test */
				data_pattern =
					(t_u8)(random_count & 0x000000FF);
				memset(pmadapter,
				       tmpbuf + DIAG_LOOPBACK_HEADER_SIZE +
				       DIAG_LOOPBACK_SEQNUM_SIZE, data_pattern,
				       size);

				size += DIAG_LOOPBACK_SEQNUM_SIZE;
				pmadapter->diag_test.data_len =
					MIN(DIAG_LOOPBACK_DATA_BUF_SIZE, size);
				memcpy(pmadapter, pmadapter->diag_test.data,
				       tmpbuf + DIAG_LOOPBACK_HEADER_SIZE,
				       MIN(DIAG_LOOPBACK_DATA_BUF_SIZE, size));
			} else {
				memset(pmadapter,
				       tmpbuf + DIAG_LOOPBACK_HEADER_SIZE +
				       DIAG_LOOPBACK_SEQNUM_SIZE, data_pattern,
				       size - DIAG_LOOPBACK_SEQNUM_SIZE);
			}

			PRINTM_GET_SYS_TIME(MDATA, &diag_sec, &diag_usec);
			PRINTM(MDATA, "%lu.%06lu : Diag Data => FW\n", diag_sec,
			       diag_usec);
			pmbuf->data_len = size + DIAG_LOOPBACK_HEADER_SIZE;
			PRINTM(MDATA, "Tx Seq Num = %d data_len\n",
			       pmadapter->diag_test.seq_num, pmbuf->data_len);

			DBG_HEXDUMP(MDAT_D, "Diag Tx",
				    pmbuf->pbuf + pmbuf->data_offset, MIN(size,
									  MAX_DATA_DUMP_LEN));

			/* Make pmbuf can be freed in wlan_write_data_complete */
			pmbuf->buf_type = MLAN_BUF_TYPE_DATA;
			if (pmadapter->diag_test.current_item ==
			    DIAG_TEST_PINGPONG)
				tx_param.next_pkt_len = 0;
			else
				tx_param.next_pkt_len = size;

			ret = wlan_sdio_host_to_card(pmadapter, MLAN_TYPE_DIAG,
						     pmbuf, &tx_param);

			switch (ret) {
			case MLAN_STATUS_RESOURCE:
				wlan_free_mlan_buffer(pmadapter, pmbuf);
				PRINTM(MERROR,
				       "STA Tx Error: Failed to send loopback packet!\n");
				pmadapter->dbg.num_tx_host_to_card_failure++;
				goto done;
			case MLAN_STATUS_FAILURE:
				pmadapter->data_sent = MFALSE;
				wlan_free_mlan_buffer(pmadapter, pmbuf);
				PRINTM(MERROR,
				       "STA Tx Error: Failed to send loopback packet!\n");
				pmadapter->dbg.num_tx_host_to_card_failure++;
				pmadapter->diag_test.num_error++;
				goto done;
			case MLAN_STATUS_SUCCESS:
				wlan_free_mlan_buffer(pmadapter, pmbuf);
				PRINTM(MINFO,
				       "STA Tx: Successfully send the loopback packet\n");
				break;
			case MLAN_STATUS_PENDING:
				break;
			default:
				break;
			}
			pmadapter->diag_test.num_packet_tx++;
			pmadapter->diag_test.seq_num++;
			if (pmadapter->diag_test.current_item ==
			    DIAG_TEST_PINGPONG)
				pmadapter->diag_test.ping_back = MFALSE;
		}
	}
done:
	LEAVE();
}

/**
 *  @brief This function processes diag loopback received packet
 *
 *  @param pmadapter A pointer to mlan_adapter
 *  @param pmbuf     A pointer to mlan_buffer which includes the received packet
 *
 *  @return 	     None
 */
INLINE void
wlan_handle_diag_rx_packet(pmlan_adapter pmadapter, pmlan_buffer pmbuf)
{
	t_u32 seq_num;
	/* diag buffer start from sequence number */
	t_u8 *diag_buf = MNULL;
#ifdef DEBUG_LEVEL1
	t_u32 diag_sec = 0, diag_usec = 0;
#endif
	mlan_private *priv = pmadapter->priv[pmbuf->bss_index];
	diag_loopback_wrong_pkt_info wrong_pkt_info;

	ENTER();

	PRINTM_GET_SYS_TIME(MDATA, &diag_sec, &diag_usec);
	PRINTM(MDATA, "%lu.%06lu : Diag Data <= FW\n", diag_sec, diag_usec);
    /** chop off 4 bytes header for SDIO/PCIE/USB */
	pmbuf->data_len = (pmadapter->upld_len - DIAG_LOOPBACK_HEADER_SIZE);
	pmbuf->data_offset += DIAG_LOOPBACK_HEADER_SIZE;
	pmadapter->upld_len -= DIAG_LOOPBACK_HEADER_SIZE;
	diag_buf = pmbuf->pbuf + pmbuf->data_offset;

	seq_num = wlan_le32_to_cpu(*(t_u32 *)diag_buf);
	PRINTM(MDATA, "Rx Seq Num = %d data_len=%d\n", seq_num,
	       pmbuf->data_len);
	DBG_HEXDUMP(MDAT_D, "Diag Rx", pmbuf->pbuf + pmbuf->data_offset,
		    MIN(pmadapter->upld_len, MAX_DATA_DUMP_LEN));

	if (pmadapter->diag_test.diag_enabled == MTRUE) {
		/* RX Test only check packet length */
		if (pmadapter->diag_test.current_item == DIAG_TEST_RX ||
		    pmadapter->diag_test.current_item == DIAG_TEST_TXRX) {
			if (pmadapter->upld_len !=
			    pmadapter->diag_test.test_size) {
				PRINTM(MERROR,
				       "DIAG LOOPBACK ERROR: size %d %d\n",
				       pmadapter->upld_len,
				       pmadapter->diag_test.test_size);
			}
		}

		/* Ping-pong Test, compare received packet with send packet */
		if (pmadapter->diag_test.current_item == DIAG_TEST_PINGPONG) {
			/* Only compare data and skip sequence number */
			if (memcmp
			    (pmadapter,
			     pmadapter->diag_test.data +
			     DIAG_LOOPBACK_SEQNUM_SIZE,
			     diag_buf + DIAG_LOOPBACK_SEQNUM_SIZE,
			     pmadapter->diag_test.data_len -
			     DIAG_LOOPBACK_SEQNUM_SIZE)) {
				pmadapter->diag_test.num_error++;
				PRINTM(MERROR,
				       "DIAG LOOPBACK ERROR: seq #%d\n",
				       seq_num);
				wrong_pkt_info.seq_num = seq_num;
				wrong_pkt_info.pkt_size =
					pmadapter->diag_test.data_len;
				wlan_send_diag_event(priv, EVENT_TYPE_WRONG_PKT,
						     (t_void
						      *)(&wrong_pkt_info));
			}

			pmadapter->diag_test.ping_back = MTRUE;

			/* Send next diag tx in Ping-pong Test */
			wlan_process_diag_loopback(pmadapter);
		}
		pmadapter->diag_test.num_packet_rx += 1;
	}
	wlan_free_mlan_buffer(pmadapter, pmbuf);
	LEAVE();
}
