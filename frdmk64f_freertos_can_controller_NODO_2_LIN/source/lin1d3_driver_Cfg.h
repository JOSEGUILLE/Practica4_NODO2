/*
 * lin1d3_driver_Cfg.h
 *
 *  Created on: Sep 14, 2018
 *      Author: Nico
 */

#ifndef LIN1D3_DRIVER_CFG_H_
#define LIN1D3_DRIVER_CFG_H_

#define lin1d3_max_supported_messages_per_node_cfg_d	(16)

#define lin1d3_msg_num (3U)

#define app_message_id_1_d (0x01<<2|message_size_2_bytes_d)
#define app_message_id_2_d (0x02<<2|message_size_4_bytes_d)
#define app_message_id_3_d (0x03<<2|message_size_8_bytes_d)

#endif /* LIN1D3_DRIVER_CFG_H_ */
