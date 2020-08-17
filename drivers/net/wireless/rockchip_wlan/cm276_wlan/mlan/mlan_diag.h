/** @file mlan_diag.h
 *
 *  @brief This file contains diag loopback code defines,
 *  structures.
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
 */

/******************************************************
Change log:
    3/25/2009: initial version
******************************************************/

#ifndef _MLAN_DIAG_H_
#define _MLAN_DIAG_H_

/** Host Command ID : Diagnostic loopback test */
#define HostCmd_CMD_DIAG_LOOPBACK_TEST        0x00c8

/** Diagnostic data buffer allocated size */
#define DIAG_LOOPBACK_DATA_BUF_SIZE     1530
/** Diagnostic data default size */
#define DIAG_LOOPBACK_DEFAULT_SIZE      1500
/** Diagnostic data header size */
#define DIAG_LOOPBACK_HEADER_SIZE       4
/** Diagnostic data sequence number size */
#define DIAG_LOOPBACK_SEQNUM_SIZE       4

/** Definition of diagnostic test item, TX=1 */
#define DIAG_TEST_TX            1
/** Definition of diagnostic test item, RX=2 */
#define DIAG_TEST_RX            2
/** Definition of diagnostic test item, TX/RX=3 */
#define DIAG_TEST_TXRX          3
/** Definition of diagnostic test item, Ping-Pong=4 */
#define DIAG_TEST_PINGPONG      4

/** Type diag payload */
#define MLAN_TYPE_DIAG		6

/** Check diag enabled or data sent to SDIO */
#define IS_DIAG_ENABLED(adapter) \
	((adapter)->diag_test.diag_enabled)
#define IS_DIAG_DATA_SENT(adapter) \
	((adapter)->diag_test.diag_enabled && (adapter)->data_sent)
#define IS_DIAG_DATA_WAIT_PONG_OR_RX_ONLY(adapter) \
	((adapter)->diag_test.diag_enabled && !((adapter)->data_sent) && \
	((((adapter)->diag_test.current_item == DIAG_TEST_PINGPONG) && !((adapter)->diag_test.ping_back)) || \
	((adapter)->diag_test.current_item == DIAG_TEST_RX)))

/** Data structure for Diagnostic Loopback Test */
typedef struct {
    /** Diagnostic enable flag */
	t_u8 diag_enabled;
    /** Diagnostic test settings */
	t_u16 current_item;
    /** Diagnostic test extra config */
	t_u16 extra_config;
    /** Size of diagnostic data */
	t_u16 test_size;
    /** Statistic packet counter */
    /** Number of packet transmitted */
	t_u32 num_packet_tx;
    /** Number of packet received */
	t_u32 num_packet_rx;
    /** Number of error */
	t_u32 num_error;
    /** Sequence number of test packet */
	t_u32 seq_num;
    /** Flag for returned packet received in Ping-pong Test */
	t_u8 ping_back;
    /** Time variable */
    /** Start time of testing */
	t_u32 start_time;
    /** End time of testing */
	t_u32 end_time;
    /** Diagnostic data pattern */
	t_u8 data[DIAG_LOOPBACK_DATA_BUF_SIZE];
    /** Length of diagnostic data pattern */
	t_u16 data_len;
} diag_loopback_t;

#define EVENT_DIAG_LOOPBACK         0x00000090
#define EVENT_TYPE_WRONG_PKT  0
#define EVENT_TYPE_RESULT     1
/** Event header */
typedef MLAN_PACK_START struct _event_header {
    /** Event ID */
	t_u32 event_id;
    /** Event data */
	t_u8 event_data[0];
} MLAN_PACK_END event_header;

#endif /* !_MLAN_DIAG_H_ */
