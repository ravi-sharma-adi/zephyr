/*
 * Copyright (c) 2020 Demant
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Common Kconfig settings
 */

#ifndef CONFIG_BT_LL_SW_SPLIT
#define CONFIG_BT_LL_SW_SPLIT y
#endif

#define CONFIG_BT_CONN
#define CONFIG_BT_MAX_CONN 4

/* ensure that proper configuration is set */

#ifndef CONFIG_BT_PERIPHERAL
#define CONFIG_BT_PERIPHERAL y
#endif
#ifndef CONFIG_BT_CENTRAL
#define CONFIG_BT_CENTRAL y
#endif

#ifndef CONFIG_BT_CTLR_PHY
#define CONFIG_BT_CTLR_PHY 1
#endif

#ifndef CONFIG_BT_CTLR_PHY_2M
#define CONFIG_BT_CTLR_PHY_2M y
#endif

#ifndef CONFIG_BT_CTLR_PHY_CODED
#define CONFIG_BT_CTLR_PHY_CODED y
#endif

#ifndef CONFIG_BT_CTLR_LOW_LAT
#define CONFIG_BT_CTLR_LOW_LAT y
#endif

#ifndef CONFIG_BT_CTLR_ULL_HIGH_PRIO
#define CONFIG_BT_CTLR_ULL_HIGH_PRIO 1
#endif

#ifndef CONFIG_BT_CTLR_ULL_LOW_PRIO
#define CONFIG_BT_CTLR_ULL_LOW_PRIO 1
#endif

#ifndef CONFIG_BT_CTLR_DATA_LENGTH
#define CONFIG_BT_CTLR_DATA_LENGTH y
#endif
#ifndef CONFIG_BT_CTLR_DATA_LENGTH_MAX
#define CONFIG_BT_CTLR_DATA_LENGTH_MAX 251
#endif

#ifndef CONFIG_BT_CTLR_LE_ENC
#define CONFIG_BT_CTLR_LE_ENC y
#endif

#ifndef CONFIG_BT_CTLR_LE_PING
#define CONFIG_BT_CTLR_LE_PING y
#endif

#ifndef CONFIG_BT_CTLR_PER_INIT_FEAT_XCHG
#define CONFIG_BT_CTLR_PER_INIT_FEAT_XCHG y
#endif

#ifndef CONFIG_BT_CTLR_CONN_RSSI
#define CONFIG_BT_CTLR_CONN_RSSI y
#endif

#ifndef CONFIG_BT_CTLR_MIN_USED_CHAN
#define CONFIG_BT_CTLR_MIN_USED_CHAN y
#endif

#ifndef CONFIG_BT_CTLR_CONN_PARAM_REQ
#define CONFIG_BT_CTLR_CONN_PARAM_REQ y
#endif

#ifndef CONFIG_BT_CTLR_EXT_REJ_IND
#define CONFIG_BT_CTLR_EXT_REJ_IND y
#endif

#ifndef CONFIG_BT_CTLR_XTAL_ADVANCED
#define CONFIG_BT_CTLR_XTAL_ADVANCED y
#endif

#ifndef CONFIG_BT_CTLR_LLCP_CONN
#define CONFIG_BT_CTLR_LLCP_CONN 4
#endif

#ifndef CONFIG_BT_CTLR_LLCP_TX_PER_CONN_TX_CTRL_BUF_NUM_MAX
#define CONFIG_BT_CTLR_LLCP_TX_PER_CONN_TX_CTRL_BUF_NUM_MAX (4)
#endif

#ifndef CONFIG_BT_CTLR_LLCP_LOCAL_PROC_CTX_BUF_NUM
#define CONFIG_BT_CTLR_LLCP_LOCAL_PROC_CTX_BUF_NUM CONFIG_BT_CTLR_LLCP_CONN
#endif

#ifndef CONFIG_BT_CTLR_LLCP_REMOTE_PROC_CTX_BUF_NUM
#define CONFIG_BT_CTLR_LLCP_REMOTE_PROC_CTX_BUF_NUM CONFIG_BT_CTLR_LLCP_CONN
#endif

#ifndef CONFIG_BT_CTLR_LLCP_COMMON_TX_CTRL_BUF_NUM
#define CONFIG_BT_CTLR_LLCP_COMMON_TX_CTRL_BUF_NUM 2
#endif

#ifndef CONFIG_BT_CTLR_LLCP_PER_CONN_TX_CTRL_BUF_NUM
#define CONFIG_BT_CTLR_LLCP_PER_CONN_TX_CTRL_BUF_NUM 1
#endif

/*
 * Direction finding related Kconfig settings
 */

/* Direction finding non LE Features configs */
#ifndef CONFIG_BT_CTLR_DF
#define CONFIG_BT_CTLR_DF y
#endif

#ifndef CONFIG_BT_CTLR_DF_CTE_TX
#define CONFIG_BT_CTLR_DF_CTE_TX y
#endif

#ifndef CONFIG_BT_CTLR_DF_CTE_RX_SAMPLE_1US
#define CONFIG_BT_CTLR_DF_CTE_RX_SAMPLE_1US y
#endif

#ifndef CONFIG_BT_CTLR_DF_ANT_SWITCH_1US
#define CONFIG_BT_CTLR_DF_ANT_SWITCH_1US y
#endif

/* Direction finding LE Features configs */
#ifndef CONFIG_BT_CTLR_DF_CONN_CTE_RX
#define CONFIG_BT_CTLR_DF_CONN_CTE_RX y
#endif

#ifndef CONFIG_BT_CTLR_DF_CONN_CTE_TX
#define CONFIG_BT_CTLR_DF_CONN_CTE_TX y
#endif

#ifndef CONFIG_BT_CTLR_DF_CONN_CTE_REQ
#define CONFIG_BT_CTLR_DF_CONN_CTE_REQ y
#endif

#ifndef CONFIG_BT_CTLR_DF_CONN_CTE_RSP
#define CONFIG_BT_CTLR_DF_CONN_CTE_RSP y
#endif

#ifndef CONFIG_BT_CTLR_DF_ANT_SWITCH_TX
#define CONFIG_BT_CTLR_DF_ANT_SWITCH_TX y
#endif

#ifndef CONFIG_BT_CTLR_DF_ANT_SWITCH_RX
#define CONFIG_BT_CTLR_DF_ANT_SWITCH_RX y
#endif

#ifndef CONFIG_BT_CTLR_DF_CTE_RX
#define CONFIG_BT_CTLR_DF_CTE_RX y
#endif

#ifndef CONFIG_BT_CTLR_SCA_UPDATE
#define CONFIG_BT_CTLR_SCA_UPDATE y
#endif

#ifndef CONFIG_BT_CTLR_DF_MAX_ANT_SW_PATTERN_LEN
#define CONFIG_BT_CTLR_DF_MAX_ANT_SW_PATTERN_LEN 38
#endif

#ifndef CONFIG_BT_CTLR_PERIPHERAL_ISO
#define CONFIG_BT_CTLR_PERIPHERAL_ISO y
#endif

/* Kconfig Cheats */
#define CONFIG_BT_LOG_LEVEL 1
#define CONFIG_BT_CTLR_COMPANY_ID 0x1234
#define CONFIG_BT_CTLR_SUBVERSION_NUMBER 0x5678
#define CONFIG_BT_CTLR_ASSERT_HANDLER y
#define CONFIG_BT_BUF_ACL_TX_COUNT 7
#define CONFIG_BT_BUF_ACL_TX_SIZE 251
#define CONFIG_BT_CTLR_RX_BUFFERS 7
#define CONFIG_NET_BUF_USER_DATA_SIZE 8
