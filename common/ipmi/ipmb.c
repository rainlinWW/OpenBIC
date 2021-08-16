#include <zephyr.h>
#include <kernel.h>
#include <stdio.h>
#include <string.h>
#include "cmsis_os2.h"
#include "hal_i2c.h"
#include "pal.h"
#include "timer.h"
#include "ipmi.h"

static osMutexId_t mutex_id[MAX_IPMB_IDX]; // mutex for sequence link list insert/find
static osMutexId_t mutex_send_req, mutex_send_res;
static const struct device *dev_ipmb[MAX_I2C_BUS_NUM];

const osMutexAttr_t IPMB_SeqQ_Mutex_attr = {
	"IPMBSeqQueueMutex",    // human readable mutex name
	osMutexPrioInherit,     // attr_bits, osMutexRobust: unlock mutex while thread terminated, osMutexPrioInherit: adjuct thread priority to avoid dead lock
	NULL,                   // memory for control block
	0U,                     // size for control block
};

const osMutexAttr_t IPMB_RES_Mutex_attr = {
	"IPMBRESQueueMutex",    // human readable mutex name
	osMutexPrioInherit,     // attr_bits, osMutexRobust: unlock mutex while thread terminated, osMutexPrioInherit: adjuct thread priority to avoid dead lock
	NULL,                   // memory for control block
	0U,                     // size for control block
};

const osMutexAttr_t IPMB_REQ_Mutex_attr = {
	"IPMBREQQueueMutex",    // human readable mutex name
	osMutexPrioInherit,     // attr_bits, osMutexRobust: unlock mutex while thread terminated, osMutexPrioInherit: adjuct thread priority to avoid dead lock
	NULL,                   // memory for control block
	0U,                     // size for control block
};

osMessageQueueId_t ipmb_txqueue[MAX_IPMB_IDX]; // Message queue for receiving IPMB message in Tx thread and write to I2C or other interfaces
extern osMessageQueueId_t IPMI_msg_queue;

IPMB_config *IPMB_config_table;
ipmi_msg_cfg *P_start[MAX_IPMB_IDX], *P_temp; // pointer for sequence queue(link list)
static unsigned int seq_current_count = 0;
static uint32_t tick_fix;
static uint32_t sys_tick_freq;

static uint8_t current_seq[MAX_IPMB_IDX]; // Sequence in BIC for sending sequence to other IPMB devices

ipmb_error ipmb_assert_chksum(uint8_t *buffer, uint8_t buffer_len);
ipmb_error ipmb_encode(uint8_t *buffer, ipmi_msg *msg);
ipmb_error ipmb_decode(ipmi_msg *msg, uint8_t *buffer, uint8_t len);
ipmb_error ipmb_notify_client(ipmi_msg_cfg *msg_cfg);

void Queue_timeout_handler(uint32_t arug0, uint32_t arug1);

uint8_t IPMB_inf_index_map[MAX_IPMB_IDX + 1]; // map IPMB source/target interface to bus

// map IPMB interface to index
void map_inf_index(void)
{
	uint8_t inf, index_num;

	memset(IPMB_inf_index_map, MAX_IPMB_IDX, sizeof(IPMB_inf_index_map));
	for (inf = 0x01; inf != Reserve_IFs; inf++) { // interface 0x0 reserved for BIC itself
		for (index_num = 0; index_num < MAX_IPMB_IDX; index_num++) {
			if (IPMB_config_table[index_num].Inf_source == inf) {
				IPMB_inf_index_map[inf] = index_num;
				break;
			}
		}
	}
}

uint8_t calculate_chksum(uint8_t *buffer, uint8_t range)
{
	uint8_t chksum = 0;
	uint8_t i;

	chksum -= buffer[0] << 1; // Use 7bit address for I2C transection, but 8bit address for checksum calculate

	for (i = 1; i < range; i++) {
		chksum -= buffer[i];
	}

	return chksum;
}

// Record IPMB request for checking response sequence and finding source sequence for bridge command
void insert_node(ipmi_msg_cfg *pnode, ipmi_msg *msg, uint8_t index)
{
	ipmi_msg_cfg *ptr_start = pnode;

	osMutexAcquire(mutex_id[index], osWaitForever);

	if (seq_current_count == MAX_DATA_QUENE) {
		ipmi_msg_cfg *temp;
		temp = pnode->next;
		pnode->next = temp->next;
		k_free(temp);
		seq_current_count--;
	}

	while (pnode->next != ptr_start) {
		pnode = pnode->next;
	}

	/* Allocate memory for the new node and put data in it.*/
	pnode->next = k_malloc(sizeof(ipmi_msg_cfg));
	if (pnode->next == NULL) {
		return;
	}
	pnode = pnode->next;

	msg->timestamp = osKernelGetSysTimerCount();
	memcpy(&pnode->buffer, msg, sizeof(ipmi_msg));

	pnode->next = ptr_start;
	seq_current_count++;
	osMutexRelease(mutex_id[index]);
	return;

}

// find if any IPMB request record match receiving response
bool find_node(ipmi_msg_cfg *pnode, ipmi_msg *msg, int seq_index, uint8_t index)
{
	ipmi_msg_cfg *ptr_start = pnode;

	osMutexAcquire(mutex_id[index], osWaitForever);

	if (seq_index == 0) { // receive response and find sent request
		while ((pnode->next != ptr_start) &&
		       (((pnode->next)->buffer.netfn != (msg->netfn - 1)) || ((pnode->next)->buffer.cmd != msg->cmd) || ((pnode->next)->buffer.seq_target != msg->seq_target))) {
			printf("no req match recv resp\n");
			printf("node netfn: %x,cmd: %x, seq_t: %x\n", (pnode->next)->buffer.netfn, (pnode->next)->buffer.cmd, (pnode->next)->buffer.seq_target);
			printf("msg netfn: %x,cmd: %x, seq_t: %x\n\n", msg->netfn, msg->cmd, msg->seq_target);
			pnode = pnode->next;
		}
	} else {
		while ((pnode->next != ptr_start) &&
		       (((pnode->next)->buffer.netfn != msg->netfn) || ((pnode->next)->buffer.cmd != msg->cmd) || ((pnode->next)->buffer.seq_source != msg->seq_source))) {
			pnode = pnode->next;
		}
	}

	if (pnode->next == ptr_start) {
		osMutexRelease(mutex_id[index]);
		return false;
	}

	/* Now pointer points to a node and the node next to it has to be removed */
	ipmi_msg_cfg *temp;
	temp = pnode->next;
	/*get the target<->Bridge IC IPMB seq number*/
	if (seq_index == 0) {
		// find source sequence for responding
		msg->seq_source = temp->buffer.seq_source;
	} else {
		msg->seq_target = temp->buffer.seq_target;
	}

	msg->InF_source = temp->buffer.InF_source;
	msg->InF_target = temp->buffer.InF_target;
	/*temp points to the node which has to be removed*/
	pnode->next = temp->next;
	/*We removed the node which is next to the pointer (which is also temp) */

	/* Beacuse we deleted the node, we no longer require the memory used for it .
	   free() will deallocate the memory.
	 */
	k_free(temp);
	seq_current_count--;

	osMutexRelease(mutex_id[index]);
	return true;
}

void IPMB_TXTask(void *pvParameters)
{
	ipmi_msg_cfg *current_msg_tx;
	IPMB_config ipmb_cfg;
	I2C_MSG *msg; // lrain - should add back later
	uint8_t ipmb_buffer_tx[IPMI_MSG_MAX_LENGTH + IPMB_RESP_HEADER_LENGTH], status = 0, retry = 5;

	memcpy(&ipmb_cfg, (IPMB_config *)pvParameters, sizeof(IPMB_config));

	while (1) {
		osMessageQueueGet(ipmb_txqueue[ipmb_cfg.index], &current_msg_tx, NULL, osWaitForever); // Wait for OS queue send interrupt

		if (current_msg_tx->buffer.data_len <= 0) {
			continue;
		}

		if (IS_RESPONSE(current_msg_tx->buffer)) {
			/* We're sending a response */

			/**********************************/
			/*       Error checking           */
			/**********************************/

			/* See if we've already tried sending this message 3 times */
			if (current_msg_tx->retries > IPMB_MAX_RETRIES) {
				/* Free the message buffer */
				printf("IPMB IF %x write reach MAX retry\n", current_msg_tx->buffer.InF_source);
				k_free(current_msg_tx);
				current_msg_tx = NULL;
				continue;
			}
			/**********************************/
			/*     Try sending the message    */
			/**********************************/
			/* Fix IPMB target address */
			current_msg_tx->buffer.dest_addr = ipmb_cfg.target_addr;
			/* Encode the message buffer to the IPMB format */
			ipmb_encode(&ipmb_buffer_tx[0], &current_msg_tx->buffer);
			uint8_t resp_tx_size = current_msg_tx->buffer.data_len + IPMB_RESP_HEADER_LENGTH;

			if (ipmb_cfg.Inf == I2C_IF) {
				msg = k_malloc(sizeof(I2C_MSG));
				msg->bus = ipmb_cfg.bus;
				msg->slave_addr = ipmb_cfg.target_addr;
				msg->tx_len = resp_tx_size;
				memcpy(&msg->data[0], &ipmb_buffer_tx[1], resp_tx_size);
				status = i2c_master_write(msg, retry); // lrain - should add back later

				k_free(msg);

			} else { // TODO: else if (ipmb_cfg.Inf == I3C_IF)
				printf("IPMB %d using not support interface: %x\n", ipmb_cfg.index, ipmb_cfg.Inf);
				continue;
			}

			if (!status) {
				/* Message couldn't be transmitted right now, increase retry counter and try again later */
				current_msg_tx->retries++;
				osMessageQueuePut(ipmb_txqueue[ipmb_cfg.index], &current_msg_tx, 0, 0);
				k_msleep(IPMB_RETRY_DELAY_ms);
			} else {
				/* Success case*/
				/* Free the message buffer */
				if (DEBUG_IPMI) {
					printf("ipmb_txqueue[%x] resp netfn: %x, cmd: %x, CC: %x, target_addr: %x\n", ipmb_cfg.index, current_msg_tx->buffer.netfn, current_msg_tx->buffer.cmd, current_msg_tx->buffer.completion_code, ipmb_cfg.target_addr);
					for (int i = 0; i < resp_tx_size + 1; i++) {
						printf(" %x", ipmb_buffer_tx[i]);
					}
					printf("\n");
				}
				k_free(current_msg_tx);
				current_msg_tx = NULL;
			}

		} else {

			/***************************************/
			/* Sending new outgoing request        */
			/***************************************/
			ipmb_encode(&ipmb_buffer_tx[0], &current_msg_tx->buffer);

			uint8_t req_tx_size = current_msg_tx->buffer.data_len + IPMB_REQ_HEADER_LENGTH;
			if (DEBUG_IPMI) {
				uint8_t i;
				printf("Tx send req: ");
				for (i = 0; i < req_tx_size + 1; i++) {
					printf(" %x", ipmb_buffer_tx[i]);
				}
				printf("\n");
			}

			if (ipmb_cfg.Inf == I2C_IF) {
				msg = k_malloc(sizeof(I2C_MSG));
				msg->bus = ipmb_cfg.bus;
				msg->slave_addr = ipmb_cfg.target_addr;
				msg->tx_len = req_tx_size;
				memcpy(&msg->data[0], &ipmb_buffer_tx[1], req_tx_size);

				status = i2c_master_write(msg, retry); // lrain - should add back later
				k_free(msg);
			} else { // TODO: else if (ipmb_cfg.Inf == I3C_IF)
				printf("IPMB %d using not support interface: %x\n", ipmb_cfg.index, ipmb_cfg.Inf);
				continue;
			}

			if (!status) {
				current_msg_tx->retries++;

				if (current_msg_tx->retries > IPMB_MAX_RETRIES) {
					/* Return fail status to request source */
					if (current_msg_tx->buffer.InF_source == Reserve_IFs) {
						printf("IPMB_TXTask: Bridging msg from reserve IFs\n");
					} else if (current_msg_tx->buffer.InF_source == Self_IFs) {
						printf("IPMB_TXTask: BIC sending command fail\n"); // Rain - Should record or notice command fail
					} else {
						ipmb_error status;
						current_msg_tx->buffer.data_len = 0;
						current_msg_tx->buffer.netfn = NETFN_OEM_REQ;
						current_msg_tx->buffer.cmd = CMD_OEM_MSG_OUT;
						current_msg_tx->buffer.completion_code = CC_NODE_BUSY;
						status = ipmb_send_response(&current_msg_tx->buffer, IPMB_inf_index_map[current_msg_tx->buffer.InF_source]);

						if (status != ipmb_error_success) {
							printf("IPMB_TXTask: Send IPMB req fail status: %x", status);
						}
					}

					/* Free the message buffer */
					k_free(current_msg_tx);
					printf("IPMB send fail after retry %d times, source: %x, cmd: 0x%x 0x%x\n", current_msg_tx->retries, current_msg_tx->buffer.InF_source, current_msg_tx->buffer.netfn, current_msg_tx->buffer.cmd);
					current_msg_tx = NULL;
				} else {
					osMessageQueuePut(ipmb_txqueue[ipmb_cfg.index], &current_msg_tx, 0, 0);
					k_msleep(IPMB_RETRY_DELAY_ms);
				}

			} else {
				/* Request was successfully sent, keep a copy here for future comparison and clean the last used buffer */
				// void insert_node(ipmi_msg_cfg *pnode, ipmi_msg *msg)
				current_msg_tx->buffer.seq_target = current_msg_tx->buffer.seq;
				insert_node(P_start[ipmb_cfg.index], &current_msg_tx->buffer, ipmb_cfg.index);
				if (DEBUG_IPMI) {
					printf("Insert node[%x] seq_s: %x, seq_t: %x\n", ipmb_cfg.Inf_source, current_msg_tx->buffer.seq_source, current_msg_tx->buffer.seq_target);
				}
				k_free(current_msg_tx);
			}
		}
		k_msleep(10);

	}
}

void IPMB_RXTask(void *pvParameters)
{
	ipmi_msg_cfg *current_msg_rx;
	IPMB_config ipmb_cfg;
  struct ipmb_msg *msg = NULL;
	uint8_t ipmb_buffer_rx[IPMI_MSG_MAX_LENGTH + IPMB_RESP_HEADER_LENGTH];
	uint8_t *kcs_buff;
	uint16_t rx_len;
	static uint16_t i = 0;
	int ret;

	memcpy(&ipmb_cfg, (IPMB_config *)pvParameters, sizeof(IPMB_config));
	ipmb_buffer_rx[0] = ipmb_cfg.slave_addr;

	if (DEBUG_IPMI) {
		printf("Rx poll bus %d, index %d\n", ipmb_cfg.bus, ipmb_cfg.index);
	}

	while (1) {
		k_msleep(IPMB_MQUEUE_POLL_DELAY_ms);

		rx_len = 0;
		if (ipmb_cfg.Inf == I2C_IF) {
      ret = ipmb_slave_read(dev_ipmb[ipmb_cfg.bus], &msg, &rx_len);
      if (!ret) {
        ipmb_slave_remove(dev_ipmb[ipmb_cfg.bus]);
      } else {
        continue;
      }
      memcpy(ipmb_buffer_rx, (uint8_t *)msg, rx_len);
      ipmb_buffer_rx[0] = ipmb_buffer_rx[0] >> 1;
/*			ipmb_buffer_rx[1] = 0x18;
			ipmb_buffer_rx[2] = 0xa8;
			ipmb_buffer_rx[3] = 0x40;
			ipmb_buffer_rx[4] = 0xd4;
			ipmb_buffer_rx[5] = 0x01;
			ipmb_buffer_rx[6] = 0xeb;
			rx_len = 6;
			i = 1;*/
		} else if (ipmb_cfg.Inf == I3C_IF) {
			;
		} else {
			printf("IPMB_RXTask: Invalid IPMB interface \n");
		}

		if (rx_len > 0) {

			if (DEBUG_IPMI) {
				printf("recv[%d]", rx_len);
				for (i = 0; i < rx_len; i++) {
					printf(" %x", ipmb_buffer_rx[i + 1]);
				}
				printf("\n");
			}

			/* Perform a checksum test on the message, if it doesn't pass, just ignore it.
			 * Following the IPMB specs, we have no way to know if we're the one who should
			 * receive it. In MicroTCA crates with star topology for IPMB, we are assured we
			 * are the recipients, however, malformed messages may be safely ignored as the
			 * MCMC should take care of retrying.
			 */
			if (ipmb_assert_chksum(ipmb_buffer_rx, rx_len) != ipmb_error_success) {
				printf("Recv invalid chksum from index %d\n", ipmb_cfg.index);
				continue;
			}
			current_msg_rx = k_malloc(sizeof(ipmi_msg_cfg));
			if (!current_msg_rx) {
				printf("current_msg_rx buf is NULL\n");
			}

			/* Clear our local buffer before writing new data into it */
			memset(current_msg_rx, 0, sizeof(ipmi_msg_cfg));
			ipmb_error IPMB_ERR_STATUS;
			IPMB_ERR_STATUS = ipmb_decode(&(current_msg_rx->buffer), ipmb_buffer_rx, rx_len);

			if (IPMB_ERR_STATUS != ipmb_error_success) {
				printf("IPMB_ERR_STATUS: %x\n", IPMB_ERR_STATUS);
			}

			if (DEBUG_IPMI) {
				printf("buff[%d]", current_msg_rx->buffer.data_len);
				for (i = 0; i < current_msg_rx->buffer.data_len; i++) {
					printf(" %x", current_msg_rx->buffer.data[i]);
				}
				printf("\n");
			}

			if (IS_RESPONSE(current_msg_rx->buffer)) {
				/* The message is a response, check if it's request from BIC and respond in time */
				current_msg_rx->buffer.seq_target = current_msg_rx->buffer.seq;
				if (find_node(P_start[ipmb_cfg.index], &(current_msg_rx->buffer), 0, ipmb_cfg.index)) {
					if (DEBUG_IPMI) {
						printf("find node IFs:%x, IFt:%x\n", current_msg_rx->buffer.InF_source, current_msg_rx->buffer.InF_target);
						printf("find buff[%d] seq %x:", current_msg_rx->buffer.data_len, current_msg_rx->buffer.seq_target);
						for (i = 0; i < current_msg_rx->buffer.data_len; i++) {
							printf(" %x", current_msg_rx->buffer.data[i]);
						}
						printf("\n");
					}

					if (current_msg_rx->buffer.InF_source == SELF_IPMB_IF) {        // Send from other thread
						;
/*          } else if (current_msg_rx->buffer.InF_source == HOST_KCS_IFs) {
            if ( DEBUG_KCS ) {
                          printf("To KCS: netfn=0x%02x, cmd=0x%02x, cmlp=0x%02x, data[%d]:\n", current_msg_rx->buffer.netfn, current_msg_rx->buffer.cmd, current_msg_rx->buffer.completion_code, current_msg_rx->buffer.data_len);
              for (i = 2; i < current_msg_rx->buffer.data_len; ++i) {
                if (i && (i % 16 == 0))
                  printf("\n");
                printf("%02x ", ipmb_buffer_rx[i]);
              }
              printf("\n");
            }
            kcs_buff = k_malloc(KCS_buff_size * sizeof(uint8_t));
            kcs_buff[0] = current_msg_rx->buffer.netfn;
            kcs_buff[1] = current_msg_rx->buffer.cmd;
            kcs_buff[2] = current_msg_rx->buffer.completion_code;
            if(current_msg_rx->buffer.data_len > 0) {
              memcpy(&kcs_buff[3], &current_msg_rx->buffer.data[0], current_msg_rx->buffer.data_len);
            }

            rc = aspeed_kcs_write(&kcs3, kcs_buff, current_msg_rx->buffer.data_len + 3); // data len + netfn + cmd + cc
            if (rc < 0) {
              printf("failed to write KCS data, rc=%d\n", rc);
            }
            k_free(kcs_buff); */                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       // lrain - should add back later
					} else {                                                        // Bridge response to other fru
						ipmb_error status;
						ipmi_msg find_msg;
						find_msg.data[0] = WW_IANA_ID & 0xFF; // Move target response to bridge response data
						find_msg.data[1] = (WW_IANA_ID >> 8) & 0xFF;
						find_msg.data[2] = (WW_IANA_ID >> 16) & 0xFF;
						find_msg.data[3] = IPMB_config_table[ipmb_cfg.index].Inf_source;        // return response source as request target
						find_msg.data[4] = current_msg_rx->buffer.netfn;                        // Move target response to bridge response data
						find_msg.data[5] = current_msg_rx->buffer.cmd;
						find_msg.data[6] = current_msg_rx->buffer.completion_code;
						find_msg.data_len = current_msg_rx->buffer.data_len + 7;        // add 7 byte len for bridge header
						memcpy(&find_msg.data[7], &current_msg_rx->buffer.data[0], current_msg_rx->buffer.data_len);
						find_msg.netfn = NETFN_OEM_REQ;                                 // Add bridge response header
						find_msg.cmd = CMD_OEM_MSG_OUT;
						find_msg.completion_code = CC_SUCCESS;
						find_msg.seq = current_msg_rx->buffer.seq_source;

						if (DEBUG_IPMI) {
							printf("bridge[%d] seq_s %x, seq_t %x, Inf_source %x, Inf_source_index %x :\n", find_msg.data_len, current_msg_rx->buffer.seq_source, current_msg_rx->buffer.seq_target, current_msg_rx->buffer.InF_source, IPMB_inf_index_map[current_msg_rx->buffer.InF_source]);
							for (i = 0; i < find_msg.data_len; i++) {
								printf(" %x", find_msg.data[i]);
							}
							printf("\n");
						}

						status = ipmb_send_response(&find_msg, IPMB_inf_index_map[current_msg_rx->buffer.InF_source]);
						if (status != ipmb_error_success) {
							printf("IPMB_RXTask: Send IPMB resp fail status: %x", status);
						}
					}
					k_free(current_msg_rx);
				}

			} else {
				/* The received message is a request */
				/* Record sequence number for later response */
				current_msg_rx->buffer.seq_source = current_msg_rx->buffer.seq;
				/* Record source interface for later bridge response */
				current_msg_rx->buffer.InF_source = IPMB_config_table[ipmb_cfg.index].Inf_source;
				/* Notify the client about the new request */
				if (DEBUG_IPMI) {
					printf("recv req: data: %x, InfS: %x, seq_s: %x\n", current_msg_rx->buffer.netfn, current_msg_rx->buffer.InF_source, current_msg_rx->buffer.seq_source);
				}
				ipmb_notify_client(current_msg_rx);
			}
		}
	}
}

ipmb_error ipmb_send_request(ipmi_msg *req, uint8_t index)
{
	osMutexAcquire(mutex_send_req, time_100_ms);
	ipmi_msg_cfg *req_cfg = k_malloc(sizeof(ipmi_msg_cfg));
	/* Builds the message according to the IPMB specification */
	/* Copies data from the msg struct passed by caller */
	memcpy(&req_cfg->buffer.data, &req->data, req->data_len);
	req_cfg->buffer.netfn = req->netfn;
	req_cfg->buffer.cmd = req->cmd;
	req_cfg->buffer.data_len = req->data_len;
	req_cfg->buffer.InF_source = req->InF_source;
	req_cfg->buffer.InF_target = req->InF_target;
	req_cfg->buffer.completion_code = req->completion_code;
	/* Write necessary fields */
	req_cfg->buffer.dest_addr = IPMB_config_table[index].target_addr;
	req_cfg->buffer.dest_LUN = 0;
	req_cfg->buffer.src_addr = IPMB_config_table[index].slave_addr << 1;
	req_cfg->buffer.seq = (current_seq[index]++) & 0x3f;
	req_cfg->buffer.seq_source = req->seq_source;
	req_cfg->buffer.src_LUN = 0;
	req_cfg->retries = 0;
	/* Blocks here until is able put message in tx queue */
	if (osMessageQueuePut(ipmb_txqueue[index], &req_cfg, 0, osWaitForever) != osOK) {
		k_free(req_cfg);
		osMutexRelease(mutex_send_req);
		return ipmb_error_failure;
	}
	osMutexRelease(mutex_send_req);
	return ipmi_error_success;
}

ipmb_error ipmb_send_response(ipmi_msg *resp, uint8_t index)
{
	osMutexAcquire(mutex_send_res, time_100_ms);

	ipmi_msg_cfg *resp_cfg = k_malloc(sizeof(ipmi_msg_cfg));

	/* Builds the message according to the IPMB specification */
	/* Copies data from the response msg struct passed by caller */
	memcpy(&resp_cfg->buffer.data, &resp->data, resp->data_len);
	resp_cfg->buffer.netfn = resp->netfn;
	resp_cfg->buffer.cmd = resp->cmd;
	resp_cfg->buffer.data_len = resp->data_len;
	resp_cfg->buffer.InF_source = resp->InF_source;
	resp_cfg->buffer.InF_target = resp->InF_target;
	resp_cfg->buffer.completion_code = resp->completion_code;
	resp_cfg->buffer.dest_addr = IPMB_config_table[index].target_addr;
	resp_cfg->buffer.netfn = resp->netfn + 1;
	resp_cfg->buffer.dest_LUN = resp->src_LUN;
	resp_cfg->buffer.src_addr = IPMB_config_table[index].slave_addr << 1;
	resp_cfg->buffer.seq = resp->seq;
	resp_cfg->buffer.seq_source = resp_cfg->buffer.seq;
	resp_cfg->buffer.InF_source = resp->InF_source;
	resp_cfg->buffer.src_LUN = resp->dest_LUN;
	resp_cfg->buffer.cmd = resp->cmd;
	resp_cfg->retries = 0;

	if (DEBUG_IPMI) {
		uint8_t i;
		printf("send resp[%d] index %x cc %x :", resp_cfg->buffer.data_len, index, resp_cfg->buffer.completion_code);
		for (i = 0; i < resp_cfg->buffer.data_len; i++) {
			printf(" %x", resp_cfg->buffer.data[i]);
		}
		printf("\n");
	}

	/* Blocks here until is able put message in tx queue */
	if (osMessageQueuePut(ipmb_txqueue[index], &resp_cfg, 0, osWaitForever) != osOK) {
		k_free(resp_cfg);
		osMutexRelease(mutex_send_res);
		return ipmb_error_failure;
	}
	osMutexRelease(mutex_send_res);
	return ipmi_error_success;
}

// run IPMI handler and notify command process status
ipmb_error ipmb_notify_client(ipmi_msg_cfg *msg_cfg)
{
	ipmi_error status;

	/* Sends only the ipmi msg, not the control struct */
	if (!IS_RESPONSE(msg_cfg->buffer)) {
		if ((status = (IPMI_handler(msg_cfg))) != ipmi_error_success) {
			/* This shouldn't happen, but if it does, clear the message buffer, since the IPMB_TX task gives us its ownership */
			printf("IPMI_handler fail with status: %x\n", status);
			return ipmb_error_timeout;
		}
	}
	/* The message has already been copied to the responsible task, free it so we don't run out of resources */
	// k_free( msg_cfg );

	return ipmb_error_success;
}

ipmb_error ipmb_assert_chksum(uint8_t *buffer, uint8_t buffer_len)
{
	uint8_t header_chksum = buffer[2];
	uint8_t msg_chksum = buffer[buffer_len - 1];
	uint8_t calc_header_chksum = calculate_chksum(buffer, IPMI_HEADER_CHECKSUM_POSITION);
	uint8_t calc_msg_chksum = calculate_chksum(buffer, buffer_len - 1);

	if (header_chksum == calc_header_chksum) {
		if (msg_chksum == calc_msg_chksum) {
			return ipmb_error_success;
		}


		return ipmb_error_hdr_chksum;
	}
	return ipmb_error_msg_chksum;
}

ipmb_error ipmb_encode(uint8_t *buffer, ipmi_msg *msg)
{
	uint8_t i = 0;

	/* Use this variable to address the buffer dynamically */

	buffer[i++] = msg->dest_addr;
	buffer[i++] = (((msg->netfn << 2) & IPMB_NETFN_MASK) | (msg->dest_LUN & IPMB_DEST_LUN_MASK));
	buffer[i++] = calculate_chksum(&buffer[0], IPMI_HEADER_CHECKSUM_POSITION);
	buffer[i++] = msg->src_addr;
	buffer[i++] = (((msg->seq << 2) & IPMB_SEQ_MASK) | (msg->src_LUN & IPMB_SRC_LUN_MASK));
	buffer[i++] = msg->cmd;
	if (IS_RESPONSE((*msg))) {
		buffer[i++] = msg->completion_code;
	}
	memcpy(&buffer[i], &msg->data[0], msg->data_len);
	i += msg->data_len;

	buffer[i] = calculate_chksum(&buffer[0], i);
	return ipmb_error_success;
}

ipmb_error ipmb_decode(ipmi_msg *msg, uint8_t *buffer, uint8_t len)
{
	/* Use this variable to address the buffer dynamically */
	uint8_t i = 0;

	msg->dest_addr = buffer[i++];
	msg->netfn = buffer[i] >> 2;
	msg->dest_LUN = (buffer[i++] & IPMB_DEST_LUN_MASK);
	msg->hdr_chksum = buffer[i++];
	msg->src_addr = buffer[i++];
	msg->seq = buffer[i] >> 2;
	msg->src_LUN = (buffer[i++] & IPMB_SRC_LUN_MASK);
	msg->cmd = buffer[i++];
	/* Checks if the message is a response and if so, fills the completion code field */
	if (IS_RESPONSE((*msg))) {
		msg->completion_code = buffer[i++];
	}
	msg->data_len = (len > i + 1) ? len - i - 1 : 0;
	memcpy(&msg->data[0], &buffer[i], msg->data_len);
	msg->msg_chksum = buffer[len - 1];

	return ipmb_error_success;
}

void IPMB_SeqTimeout_handler(void *arug0)
{
	ipmi_msg_cfg *pnode;
	uint8_t index;
	uint32_t current_time;

	while (1) {
		// k_msleep(IPMB_SEQ_TIMEOUT_ms);
		k_msleep(IPMB_SEQ_TIMEOUT_ms);

		for (index = 0; index < MAX_IPMB_IDX; index++) {
			if (!IPMB_config_table[index].EnStatus) {
				continue;
			}

			osMutexAcquire(mutex_id[index], osWaitForever);
			pnode = P_start[index];
			while (pnode->next != P_start[index]) {
				current_time = osKernelGetSysTimerCount();
				// printf("cur: %d, next_node: %d, time: %d, freq: %d, check: %d\n",current_time+tick_fix,(pnode->next)->buffer.timestamp, current_time + tick_fix - ((pnode->next)->buffer.timestamp), sys_tick_freq, IPMB_timeout_S * sys_tick_freq);
				// The queue stay more than 1000 ms need to be killed
				if (current_time + tick_fix - ((pnode->next)->buffer.timestamp) > IPMB_timeout_S * sys_tick_freq || (current_time + tick_fix - ((pnode->next)->buffer.timestamp)) & 0x80000000) {
					ipmi_msg_cfg *temp;
					temp = pnode->next;
					pnode->next = temp->next;
					k_free(temp);
					seq_current_count--;
				}

				pnode = pnode->next;
			}

			osMutexRelease(mutex_id[index]);
		}
	}
}

static void init_ipmb_slave_dev(void) {
  int status;

#ifdef DEV_IPMB_0
  dev_ipmb[0] = device_get_binding("IPMB_0");
  if (status)
    printk("ipmb0 mutex init fail\n");
  if (i2c_slave_driver_register(dev_ipmb[0]))
    printk("IPMB0: Slave Device driver not found.");
printk("ipmb0 init\n");
#endif
#ifdef DEV_IPMB_1
  dev_ipmb[1] = device_get_binding("IPMB_1");
  if (status)
    printk("ipmb1 mutex init fail\n");
  if (i2c_slave_driver_register(dev_ipmb[1]))
    printk("IPMB: Slave Device driver not found.");
printk("ipmb1 init\n");
#endif
#ifdef DEV_IPMB_2
  dev_ipmb[2] = device_get_binding("IPMB_2");
  if (status)
    printk("ipmb2 mutex init fail\n");
  if (i2c_slave_driver_register(dev_ipmb[2]))
    printk("IPMB2: Slave Device driver not found.");
printk("ipmb2 init\n");
#endif
#ifdef DEV_IPMB_3
  dev_ipmb[3] = device_get_binding("IPMB_3");
  if (status)
    printk("ipmb3 mutex init fail\n");
  if (i2c_slave_driver_register(dev_ipmb[3]))
    printk("IPMB3: Slave Device driver not found.");
printk("ipmb3 init\n");
#endif
#ifdef DEV_IPMB_4
  dev_ipmb[4] = device_get_binding("IPMB_4");
  if (status)
    printk("ipmb4 mutex init fail\n");
  if (i2c_slave_driver_register(dev_ipmb[4]))
    printk("IPMB4: Slave Device driver not found.");
printk("ipmb4 init\n");
#endif
#ifdef DEV_IPMB_5
  dev_ipmb[5] = device_get_binding("IPMB_5");
  if (status)
    printk("ipmb5 mutex init fail\n");
  if (i2c_slave_driver_register(dev_ipmb[5]))
    printk("IPMB5: Slave Device driver not found.");
printk("ipmb5 init\n");
#endif
#ifdef DEV_IPMB_6
  dev_ipmb[6] = device_get_binding("IPMB_6");
  if (status)
    printk("ipmb6 mutex init fail\n");
  if (i2c_slave_driver_register(dev_ipmb[6]))
    printk("IPMB6: Slave Device driver not found.");
printk("ipmb6 init\n");
#endif
#ifdef DEV_IPMB_7
  dev_ipmb[7] = device_get_binding("IPMB_7");
  if (status)
    printk("ipmb7 mutex init fail\n");
  if (i2c_slave_driver_register(dev_ipmb[7]))
    printk("IPMB7: Slave Device driver not found.");
printk("ipmb7 init\n");
#endif
#ifdef DEV_IPMB_8
  dev_ipmb[8] = device_get_binding("IPMB_8");
  if (status)
    printk("ipmb8 mutex init fail\n");
  if (i2c_slave_driver_register(dev_ipmb[8]))
    printk("IPMB8: Slave Device driver not found.");
printk("ipmb8 init\n");
#endif
#ifdef DEV_IPMB_9
  dev_ipmb[9] = device_get_binding("IPMB_9");
  if (status)
    printk("ipmb9 mutex init fail\n");
  if (i2c_slave_driver_register(dev_ipmb[9]))
    printk("IPMB9: Slave Device driver not found.");
printk("ipmb9 init\n");
#endif

}

void ipmb_util_init(uint8_t index)
{
	uint8_t I2C_index = 0;

	osThreadId_t IPMB_thread_ids_Tx[MAX_IPMB_IDX];
	osThreadId_t IPMB_thread_ids_Rx[MAX_IPMB_IDX];
	osThreadAttr_t IPMB_TxTask_attr;
	osThreadAttr_t IPMB_RxTask_attr;

	memset(&IPMB_TxTask_attr, 0, sizeof(IPMB_TxTask_attr));
	memset(&IPMB_RxTask_attr, 0, sizeof(IPMB_RxTask_attr));

	P_start[index] = k_malloc(sizeof(ipmi_msg_cfg));
	P_temp = P_start[index];
	P_temp->next = P_temp;

	mutex_id[index] = osMutexNew(&IPMB_SeqQ_Mutex_attr);
	if (mutex_id[index] == NULL) {
		printf("IPMB mutex %d init fail\n", index);
	}

	ipmb_txqueue[index] = osMessageQueueNew(IPMB_TXQUEUE_LEN, sizeof(ipmi_msg_cfg *), NULL);

	IPMB_TxTask_attr.name = IPMB_config_table[index].Tx_attr_name;
	IPMB_TxTask_attr.priority = osPriorityBelowNormal;
	IPMB_TxTask_attr.stack_size = 0x1000;
	IPMB_thread_ids_Tx[index] = osThreadNew(IPMB_TXTask, (void *)&IPMB_config_table[index], &IPMB_TxTask_attr);

	IPMB_RxTask_attr.name = IPMB_config_table[index].Rx_attr_name;
	IPMB_RxTask_attr.priority = osPriorityBelowNormal;
	IPMB_RxTask_attr.stack_size = 0x1000;
	IPMB_thread_ids_Rx[index] = osThreadNew(IPMB_RXTask, (void *)&IPMB_config_table[index], &IPMB_RxTask_attr);

	if (DEBUG_IPMI) {
		printf("Create %s and %s \n", osThreadGetName(IPMB_thread_ids_Tx[index]), osThreadGetName(IPMB_thread_ids_Rx[index]));
		printf("Init I2C bus %d, addr %x\n", IPMB_config_table[index].bus, IPMB_config_table[index].slave_addr);
	}

	return;
}

void ipmb_init(void)
{
	uint8_t index;
	osThreadId_t IPMB_SeqTimeout_task;
	osThreadAttr_t IPMB_SeqTimeout_attr;

  init_ipmb_slave_dev();

	IPMB_config_table = k_malloc(MAX_IPMB_IDX * sizeof(IPMB_config));
	if (IPMB_config_table != NULL) {
		pal_load_IPMB_config();
	} else {
		printf("IPMB_config_table alloc fail\n");
		return false;
	}

	map_inf_index();

	memset(&current_seq, 0, sizeof(uint8_t) * MAX_IPMB_IDX);

	memset(&IPMB_SeqTimeout_attr, 0, sizeof(IPMB_SeqTimeout_attr));


	sys_tick_freq = osKernelGetSysTimerFreq();
	tick_fix = sys_tick_freq / 1000;

	mutex_send_req = osMutexNew(&IPMB_REQ_Mutex_attr);
	mutex_send_res = osMutexNew(&IPMB_RES_Mutex_attr);

	for (index = 0; index < MAX_IPMB_IDX; index++) {
		if (IPMB_config_table[index].EnStatus) {
			ipmb_util_init(index);
		}
	}

	IPMB_SeqTimeout_attr.name = "IPMB_Seq_timeout_handler";
	IPMB_SeqTimeout_attr.priority = osPriorityBelowNormal;
	IPMB_SeqTimeout_task = osThreadNew(IPMB_SeqTimeout_handler, NULL, &IPMB_SeqTimeout_attr);
}
