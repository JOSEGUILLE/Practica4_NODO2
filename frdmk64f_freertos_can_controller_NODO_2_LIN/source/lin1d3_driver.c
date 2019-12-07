/*
 * lin1d3_driver.c
 *
 *  Created on: Sep 14, 2018
 *      Author: Nico
 */
#include "lin1d3_driver.h"
#include <string.h>
#include <fsl_debug_console.h>

#define master_stack_size_d	(256)
#define master_task_priority (configMAX_PRIORITIES - 1)
#define master_queue_size_d	(8)

#define slave_stack_size_d	(256)
#define slave_task_priority (configMAX_PRIORITIES - 1)

#define size_of_uart_buffer	(10)

#define size_of_lin_header_d (2)





/*Static function prototypes */
static void master_task(void *pvParameters);
static void slave_task(void *pvParameters);
static uint8_t parityBitP0(uint8_t header);
static uint8_t parityBitP1(uint8_t header);
static uint8_t checksum(uint8_t * data, uint8_t dataLength);
static lin1d3_master_msg_T masterMsgTypefromID(uint8_t ID);

/* variables */
static lin1d3_master_msg_type_ID_T lin1d3_master_msg_type_ID_map[lin1d3_msg_num] = {{app_message_id_1_d, lin1d3_master_msg},\
													{app_message_id_2_d,lin1d3_slave_msg },{app_message_id_2_d, lin1d3_master_msg}};
static uint8_t ledValue = 1;
static uint8_t ledValueSlave = 0;

/******************************************************************************
 * Public functions
 *
 *****************************************************************************/

/*
 * Init a LIN node
 * */
lin1d3_handle_t* lin1d3_InitNode(lin1d3_nodeConfig_t config)
{
	lin1d3_handle_t* handle = NULL;
	static uint8_t node_idx = 0;
	char master_task_name[] = "linMaster0";
	char slave_task_name[] = "linSlave0";
	/* Check init parameters */
	if(config.type >= lin1d3_max_nodeType) {
		return NULL;
	}

	/* Create the handle structure and */
	handle = (lin1d3_handle_t*)pvPortMalloc(sizeof(lin1d3_handle_t));
	if(handle ==  NULL) {
		/* Failed to allocate memory for the node handle */
		return NULL;
	}
	/* Init the handle structure with 0s */
	memset(handle, 0, sizeof(lin1d3_handle_t));
	/* Copy the config */
	memcpy(&(handle->config), &config, sizeof(lin1d3_nodeConfig_t));

	/* Create the Node Task */
	if(lin1d3_master_nodeType == config.type) {
		/* Create a queue for User message requests */
		handle->node_queue = xQueueCreate( master_queue_size_d, sizeof(uint8_t));
		if(handle->node_queue == NULL){
			vPortFree(handle);
			return NULL;
		}
		/* Create a task for the node */
		master_task_name[strlen(master_task_name)-1] += node_idx++;
		if (xTaskCreate(master_task, master_task_name, master_stack_size_d, handle, master_task_priority, &(handle->task_handle)) != pdPASS) {
			vPortFree(handle);
			return NULL;
		}
	}
	else if(lin1d3_slave_nodeType == config.type) {
		/* Create a task for the node */
		slave_task_name[strlen(slave_task_name)-1] += node_idx++;
		if (xTaskCreate(slave_task, slave_task_name, slave_stack_size_d, handle, slave_task_priority, &(handle->task_handle)) != pdPASS) {
			vPortFree(handle);
			return NULL;
		}
	}

	return handle;
}

/*
 * Send a message frame from a LIN Master node
 * */
uint32_t lin1d3_masterSendMessage(lin1d3_handle_t* handle, uint8_t ID)
{
	if(handle !=  NULL) {
		/* Put the requested ID on the master queue */
		xQueueSend( handle->node_queue, &ID, ( TickType_t ) 0 );
	}
	return 0;
}

/******************************************************************************
 * Static functions
 *
 *****************************************************************************/
static void master_task(void *pvParameters)
{
	lin1d3_handle_t* handle = (lin1d3_handle_t*)pvParameters;
	uint8_t  ID;
	uint8_t  lin1p3_header[2] = {0x55, 0x00};
	uint8_t  lin1p3_master_data[9] = {};
	uint8_t  lin1p3_message[size_of_uart_buffer];
	uint8_t  message_size = 0;
	size_t n;
	uint8_t  msg_idx;
	uint8_t headerAux = 0u;
	uint8_t chckSumAux = 0u;
	lin1d3_master_msg_T masterMsgType_t = lin1d3_master_msg;

	if(handle == NULL) {
		vTaskSuspend(NULL);
	}

	/* Init/Configure the UART */
	handle->uart_config.base = handle->config.uartBase;
	handle->uart_config.srcclk = handle->config.srcclk;
	handle->uart_config.baudrate = handle->config.bitrate;
	handle->uart_config.parity = kUART_ParityDisabled;
	handle->uart_config.stopbits = kUART_OneStopBit;
	handle->uart_config.buffer = pvPortMalloc(size_of_uart_buffer);
	handle->uart_config.buffer_size = size_of_uart_buffer;
	if(handle->uart_config.buffer == NULL){
		vTaskSuspend(NULL);
	}

    if (0 > UART_RTOS_Init(&(handle->uart_rtos_handle), &(handle->uart_handle), &(handle->uart_config)))
    {
        vTaskSuspend(NULL);
    }

    /* Make a pause here so the other task can be ready to wait for messages */
    vTaskDelay(100);

    /* Configure 13 bits break transmission */
    handle->uart_rtos_handle.base->S2 |= (1<<2);

    while(1) {
    	/* Wait for messages on the Queue */
        if(xQueueReceive(handle->node_queue, &ID, portMAX_DELAY)){
        	masterMsgType_t = masterMsgTypefromID(ID);
        	msg_idx = 0;
        	/*Look for the ID in the message table */
        	while(msg_idx < lin1d3_max_supported_messages_per_node_cfg_d) {
        		if(handle->config.messageTable[msg_idx].ID == ID) {
        			break;
        		}
        		msg_idx++;
        	}

        	/* If the message ID was not found then ignore it */
        	if(msg_idx == lin1d3_max_supported_messages_per_node_cfg_d) continue;

        	/* Put the ID into the header */
        	lin1p3_header[1] = ID<<2;
        	/* TODO: put the parity bits */
        	headerAux = parityBitP0(lin1p3_header[1]);
        	lin1p3_header[1] |= headerAux;
        	headerAux = parityBitP1(lin1p3_header[1]);
        	lin1p3_header[1] |= headerAux;

        	/* Init the message recevie buffer */
        	memset(lin1p3_message, 0, size_of_uart_buffer);
        	/* Calc the message size */
        	switch(ID&0x03) {
        		case 0x00: message_size = 2;
        		break;
        		case 0x01: message_size = 2;
        		break;
        		case 0x02: message_size = 4;
        		break;
        		case 0x03: message_size = 8;
        		break;
        	}

        	/* Send a Break It is just sending one byte 0, *** CHANGE THIS WITH A REAL SYNCH BREAK ****/
            /* Send the break signal */
        	handle->uart_rtos_handle.base->C2 |= 0x01;
        	handle->uart_rtos_handle.base->C2 &= 0xFE;
            vTaskDelay(1);// make a small pause to prevent issues due to Rx slow break detection
            if(lin1d3_master_msg == masterMsgType_t)
            {
            	message_size+=1;
            	/* Send the header */
            	UART_RTOS_Send(&(handle->uart_rtos_handle), (uint8_t *)lin1p3_header, size_of_lin_header_d);
            	/* Wait for the response */
            	UART_RTOS_Receive(&(handle->uart_rtos_handle), lin1p3_message, message_size, &n);
            	message_size--;
            	/* TODO: Check the checksum */
            	chckSumAux = checksum((uint8_t *)&lin1p3_message[0], message_size);
            	if(chckSumAux != lin1p3_message[message_size])
            	{
            		continue;
            	}
            	/* Call the message callback */
            	handle->config.messageTable[msg_idx].handler((void*)lin1p3_message);
            }
            else
            {

            	UART_RTOS_Send(&(handle->uart_rtos_handle), (uint8_t *)lin1p3_header, size_of_lin_header_d);
            	vTaskDelay(1);// make a small pause
            	lin1p3_master_data[0] = ledValue;
    			chckSumAux = checksum((uint8_t *)&lin1p3_master_data[0], message_size);
    			lin1p3_master_data[message_size] = chckSumAux;
            	UART_RTOS_Send(&(handle->uart_rtos_handle), (uint8_t *)lin1p3_master_data, (message_size + 1));

            }
        }
    }
}

static void slave_task(void *pvParameters)
{
	lin1d3_handle_t* handle = (lin1d3_handle_t*)pvParameters;
	uint8_t  ID;
	uint8_t  lin1p3_header[size_of_lin_header_d];
	uint8_t  lin1p3_message[size_of_uart_buffer];
	uint8_t  lin1p3_message_master[size_of_uart_buffer];
	uint8_t  message_size = 0;
	size_t n;
	uint8_t  msg_idx;
	uint8_t synch_break_byte = 0;
	uint8_t headerAuxP0 = 0u;
	uint8_t headerAuxP1 = 0u;
	uint8_t chckSumAux = 0u;
	lin1d3_master_msg_T masterMsgType_t = lin1d3_master_msg;


	if(handle == NULL) {
		vTaskSuspend(NULL);
	}

	/* Init/Configure the UART */
	handle->uart_config.base = handle->config.uartBase;
	handle->uart_config.srcclk = handle->config.srcclk;
	handle->uart_config.baudrate = handle->config.bitrate;
	handle->uart_config.parity = kUART_ParityDisabled;
	handle->uart_config.stopbits = kUART_OneStopBit;
	handle->uart_config.buffer = pvPortMalloc(size_of_uart_buffer);
	handle->uart_config.buffer_size = size_of_uart_buffer;
	if(handle->uart_config.buffer == NULL){
		vTaskSuspend(NULL);
	}

    if (0 > UART_RTOS_Init(&(handle->uart_rtos_handle), &(handle->uart_handle), &(handle->uart_config)))
    {
        vTaskSuspend(NULL);
    }

    while(1) {
    	char dummy;
    	/* Init the message header buffer */
    	memset(lin1p3_header, 0, size_of_lin_header_d);
    	/* Wait for break */
    	DisableIRQ(UART4_RX_TX_IRQn); //Disable RX interrupt so the break won't mess with the UART_RTOS driver
    	handle->uart_config.base->S2 |= 0x01<<7; //Clear the LIN Break Detect Interrupt Flag
    	handle->uart_config.base->S2 |= 0x01<<1; //Enable LIN Break Detection
    	while((handle->uart_config.base->S2 &  0x01<<7) == 0x00) vTaskDelay(1); //Wait for the flag to be set
    	handle->uart_config.base->S2 &= ~(0x01<<1); //Disable LIN Break Detection
    	handle->uart_config.base->S2 |= 0x01<<7; //Clear the LIN Break Detect Interrupt Flag
    	EnableIRQ(UART4_RX_TX_IRQn); //Enable RX interrupt so the UART_RTOS driver works again

    	/* Wait for header on the UART */
    	UART_RTOS_Receive(&(handle->uart_rtos_handle), lin1p3_header, size_of_lin_header_d, &n);
    	/* Check header */
    	if (lin1p3_header[0] == 0x55) {
    		/* Check ID parity bits */
    		/* Header is not correct we are ignoring the header */
    		headerAuxP0 = lin1p3_header[1] & 0x02;
    		headerAuxP1 = lin1p3_header[1] & 0x01;
    		if((headerAuxP0 != parityBitP0(lin1p3_header[1])) || (headerAuxP1 != parityBitP1(lin1p3_header[1])))
    		{
    			continue;
    		}
    	}
    	else
    	{
    		continue;
    	}
    	/* Get the message ID */
    	ID = (lin1p3_header[1] & 0xFC)>>2;
    	masterMsgType_t = masterMsgTypefromID(ID);
    	/* If the header is correct, check if the message is in the table */
    	msg_idx = 0;
    	/*Look for the ID in the message table */
    	while(msg_idx < lin1d3_max_supported_messages_per_node_cfg_d) {
    		if(handle->config.messageTable[msg_idx].ID == ID) {
    			break;
    		}
    		msg_idx++;
    	}
    	/* If the message ID was not found then ignore it */
    	if(msg_idx == lin1d3_max_supported_messages_per_node_cfg_d) continue;
    	/* Init the message transmit buffer */
    	memset(lin1p3_message, 0, size_of_uart_buffer);
    	/*If the message is in the table call the message callback */
    	handle->config.messageTable[msg_idx].handler((void*)lin1p3_message);

    	/* Calc the message size */
    	switch(ID&0x03) {
    		case 0x00: message_size = 2;
    		break;
    		case 0x01: message_size = 2;
    		break;
    		case 0x02: message_size = 4;
    		break;
    		case 0x03: message_size = 8;
    		break;
    	}
        if(lin1d3_master_msg == masterMsgType_t)
        {
			/* TODO: Add the checksum to the message */
			chckSumAux = checksum((uint8_t *)&lin1p3_message[0], message_size);
			lin1p3_message[message_size] = chckSumAux;
			message_size+=1;
			/* Send the message data */
			UART_RTOS_Send(&(handle->uart_rtos_handle), (uint8_t *)lin1p3_message, message_size);
        }
        else
        {

        	UART_RTOS_Receive(&(handle->uart_rtos_handle), lin1p3_message_master, (message_size+1), &n);
        	chckSumAux = checksum((uint8_t *)&lin1p3_message_master[0], message_size);
        	if(chckSumAux != lin1p3_message_master[message_size])
        	{
        		continue;
        	}
        	ledValueSlave = lin1p3_message_master[0];
        }
        if ( ledValueSlave == 1)
        {
        	LED_BLUE_ON();/* Here you have to handle the LED */
        }
        else
        {
        	LED_BLUE_OFF();
        }

    }
}

static uint8_t parityBitP0(uint8_t header)
{
	uint8_t returnVal = 0;
	uint8_t bitID0 = header & 0x80;
	uint8_t bitID1 = header & 0x40;
	uint8_t bitID2 = header & 0x20;
	uint8_t bitID4 = header & 0x08;
    bitID0 >>= 7;
    bitID1 >>= 6;
    bitID2 >>= 5;
    bitID4 >>= 3;
    returnVal = bitID0 ^ bitID1;
    returnVal ^= bitID2 ^ bitID4;
    returnVal <<= 0x01;
    return returnVal;
}


static uint8_t parityBitP1(uint8_t header)
{
	uint8_t returnVal = 0;
	uint8_t bitID1 = header & 0x40;
	uint8_t bitID3 = header & 0x10;
	uint8_t bitID4 = header & 0x08;
	uint8_t bitID5 = header & 0x04;
    bitID1 >>= 6;
    bitID3 >>= 4;
    bitID4 >>= 3;
    bitID5 >>= 2;
    returnVal = bitID1 ^ bitID3;
    returnVal ^= bitID4 ^ bitID5;
    returnVal ^= 0x01;
    return returnVal;
}

static uint8_t checksum(uint8_t * data, uint8_t dataLength)
{
	uint16_t auxSum = 0;
	uint8_t returnVal = 0;
	uint8_t index = 0;
    for(index = 0; dataLength > index; index++)
    {
        auxSum += data[index];
    }
    returnVal = auxSum % 255;

    return returnVal;
}

static lin1d3_master_msg_T masterMsgTypefromID(uint8_t ID)
{
	uint8_t index = 0;
	lin1d3_master_msg_T lin1d3_master_msg_t= lin1d3_master_msg;
	uint8_t msgNum = sizeof(lin1d3_master_msg_type_ID_map)/sizeof(lin1d3_master_msg_type_ID_T);
    for(index = 0; msgNum > index; index++)
    {
    	if(ID == lin1d3_master_msg_type_ID_map[index].ID)
    	{
    		lin1d3_master_msg_t = lin1d3_master_msg_type_ID_map[index].lin1d3_master_msg_type;
    		index = msgNum;
    	}
    }
    return lin1d3_master_msg_t;
}
