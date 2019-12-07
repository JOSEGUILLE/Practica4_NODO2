/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_flexcan.h"
#include "board.h"

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"

#include "can_example.h"

#include "pin_mux.h"
#include "clock_config.h"
#include "lin1d3_driver.h"
#include "FreeRTOSConfig.h"
#include "fsl_adc16.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Period */
#define OS_TICK_PERIOD_100MS 100
#define OS_TICK_PERIOD_50MS 50
/* Task priorities. */
#define hello_task_PRIORITY (configMAX_PRIORITIES - 1)
#define test_task_heap_size_d	(192)

/* CAN defines */
#define EXAMPLE_CAN CAN0
#define EXAMPLE_CAN_CLKSRC kCLOCK_BusClk
#define EXAMPLE_CAN_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)

#define RX1_MESSAGE_BUFFER_NUM (10)
#define RX2_MESSAGE_BUFFER_NUM (9)
#define TX100_MESSAGE_BUFFER_NUM (8)
#define TX50_MESSAGE_BUFFER_NUM (7)

#define xJUST_MASTER

/* UART instance and clock */
#define MASTER_UART UART3
#define MASTER_UART_CLKSRC UART3_CLK_SRC
#define MASTER_UART_CLK_FREQ CLOCK_GetFreq(UART3_CLK_SRC)
#define MASTER_UART_RX_TX_IRQn UART3_RX_TX_IRQn

/* UART instance and clock */
#define SLAVE_UART UART4
#define SLAVE_UART_CLKSRC UART4_CLK_SRC
#define SLAVE_UART_CLK_FREQ CLOCK_GetFreq(UART4_CLK_SRC)
#define SLAVE_UART_RX_TX_IRQn UART4_RX_TX_IRQn

/* Task priorities. */
#define init_task_PRIORITY (configMAX_PRIORITIES - 2)
#define test_task_heap_size_d	(192)

/*ADC configuration */
#define DEMO_ADC16_BASE (ADC0)
#define DEMO_ADC16_CHANNEL_GROUP (0U)
#define DEMO_ADC16_USER_CHANNEL (12U)

/* LED AZUL*/
#define BOARD_LED_B_GPIO BOARD_LED_BLUE_GPIO
#define BOARD_LED_B_GPIO_PIN BOARD_LED_BLUE_GPIO_PIN

/* LED ROJO*/
#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void ADC0_IRQHandler(void);
void ADC_init(void);


static void task_100ms(void *pvParameters);
static void task_50ms(void *pvParameters);
static void task_rx(void *pvParameters);

static void test_task(void *pvParameters);

static void	message_1_callback_master(void* message);
static void	message_2_callback_master(void* message);
static void	message_3_callback_master(void* message);
static void	message_1_callback_slave(void* message);
static void	message_2_callback_slave(void* message);
static void	message_3_callback_slave(void* message);

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile bool txComplete = false;
volatile bool receiving = false;
volatile bool message_received = false;
volatile uint32_t received_mb_idx;
flexcan_handle_t flexcanHandle;
flexcan_mb_transfer_t tx100Xfer, tx50Xfer, rx1Xfer, rx2Xfer,rx3Xfer, rx4Xfer;
flexcan_frame_t tx100Frame, tx50Frame, rx1Frame, rx2Frame, rx3Frame, rx4Frame;
uint32_t tx50Identifier = 0x030;   // NODO2
uint32_t tx100Identifier = 0x031;   // NODO3
uint32_t rx1Identifier = 0x010;  // nodo 2,3
uint32_t rx2Identifier = 0x011;  // nodo 2,3

uint16_t adc1 = 0;               // nodo3
uint8_t period1 = 10;             // nodo2,3

// ADC
volatile bool g_Adc16ConversionDoneFlag = false;
volatile uint32_t g_Adc16ConversionValue;
volatile uint32_t g_Adc16ConversionReturnValue;
static adc16_channel_config_t adc16ChannelConfigStruct;



/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
/*!
 * @brief FlexCAN Call Back function
 */

void ADC0_IRQHandler(void)
{
    g_Adc16ConversionDoneFlag = true;
    g_Adc16ConversionValue = ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void ADC_init(void)
{
    adc16_config_t adc16ConfigStruct;


    NVIC_EnableIRQ(ADC0_IRQn);
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
    (void)ADC16_DoAutoCalibration(DEMO_ADC16_BASE);
    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true;
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
    ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);

}





static void flexcan_callback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
    switch (status)
    {
        /* Process FlexCAN Rx event. */
        case kStatus_FLEXCAN_RxIdle:
        	//receiving = false;
        	message_received = true;
        	received_mb_idx = result;

            break;

        /* Process FlexCAN Tx event. */
        case kStatus_FLEXCAN_TxIdle:
        	txComplete = true;
            break;

        default:
            break;
    }
}

void CAN_Init(void)
{
	flexcan_config_t flexcanConfig;
	flexcan_rx_mb_config_t mbConfig;

	PRINTF("\r\n==FlexCAN example -- Start.==\r\n\r\n");


	    /* Init FlexCAN module. */
	    /*
	     * flexcanConfig.clkSrc = kFLEXCAN_ClkSrcOsc;
	     * flexcanConfig.baudRate = 125000U;
	     * flexcanConfig.maxMbNum = 16;
	     * flexcanConfig.enableLoopBack = false;
	     * flexcanConfig.enableSelfWakeup = false;
	     * flexcanConfig.enableIndividMask = false;
	     * flexcanConfig.enableDoze = false;
	     */
	    FLEXCAN_GetDefaultConfig(&flexcanConfig);
	#if (!defined(FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE)) || !FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE
	    flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
	#endif /* FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE */
	    flexcanConfig.enableLoopBack = false;
	    FLEXCAN_Init(EXAMPLE_CAN, &flexcanConfig, EXAMPLE_CAN_CLK_FREQ);

	    /* Create FlexCAN handle structure and set call back function. */
	    FLEXCAN_TransferCreateHandle(EXAMPLE_CAN, &flexcanHandle, flexcan_callback, NULL);

	    /* Set Rx Masking mechanism. */
	    //FLEXCAN_SetRxMbGlobalMask(EXAMPLE_CAN, FLEXCAN_RX_MB_STD_MASK(rx1Identifier, 0, 0));

	    /* Setup Rx Message Buffers. */
	    mbConfig.format = kFLEXCAN_FrameFormatStandard;
	    mbConfig.type = kFLEXCAN_FrameTypeData;
	    mbConfig.id = FLEXCAN_ID_STD(rx1Identifier);
	    FLEXCAN_SetRxMbConfig(EXAMPLE_CAN, RX1_MESSAGE_BUFFER_NUM, &mbConfig, true);

	    mbConfig.id = FLEXCAN_ID_STD(rx2Identifier);
	    FLEXCAN_SetRxMbConfig(EXAMPLE_CAN, RX2_MESSAGE_BUFFER_NUM, &mbConfig, true);




	    /* Setup Tx Message Buffers. */
	    FLEXCAN_SetTxMbConfig(EXAMPLE_CAN, TX100_MESSAGE_BUFFER_NUM, true);
	    FLEXCAN_SetTxMbConfig(EXAMPLE_CAN, TX50_MESSAGE_BUFFER_NUM, true);
}

int main(void)
{

	/* Initialize board hardware. */
	BOARD_InitPins();
	BOARD_BootClockRUN();
	BOARD_InitDebugConsole();
    NVIC_SetPriority(MASTER_UART_RX_TX_IRQn, 5);
    NVIC_SetPriority(SLAVE_UART_RX_TX_IRQn, 5);
	CAN_Init();
	ADC_init();
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };

	GPIO_PinInit(BOARD_LED_B_GPIO, BOARD_LED_B_GPIO_PIN, &led_config);
	LED_BLUE_OFF();


    PRINTF(" *** LIN driver demo ***\r\n");

    xTaskCreate(task_100ms, "100ms Task", configMINIMAL_STACK_SIZE + 10, NULL, hello_task_PRIORITY, NULL);
    xTaskCreate(task_50ms, "50ms Task", configMINIMAL_STACK_SIZE + 10, NULL, hello_task_PRIORITY, NULL);
    xTaskCreate(task_rx, "rx Task", (configMINIMAL_STACK_SIZE + 10)*2, NULL, hello_task_PRIORITY, NULL);
    if (xTaskCreate(test_task, "test_task", test_task_heap_size_d, NULL, init_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Init Task creation failed!.\r\n");
        while (1)
            ;
    }
    vTaskStartScheduler();

    for (;;)
    	;
}

/*!
 * @brief Task responsible for sending the 100ms message.
 */
static void task_100ms(void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = OS_TICK_PERIOD_100MS;
	volatile uint32_t can_flags = 0;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
    	/* Get the flags from the can Driver */
    	can_flags = FLEXCAN_GetStatusFlags(EXAMPLE_CAN);

    	/* Send a Can message */
    	tx100Frame.id = FLEXCAN_ID_STD(tx100Identifier);
    	tx100Frame.format = kFLEXCAN_FrameFormatStandard;
    	tx100Frame.type = kFLEXCAN_FrameTypeData;
    	tx100Frame.length = 8;
    	tx100Xfer.frame = &tx100Frame;
    	tx100Xfer.mbIdx = TX100_MESSAGE_BUFFER_NUM;
    	FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &tx100Xfer);

     	tx100Frame.dataByte0 = (g_Adc16ConversionReturnValue & 0xFF);
    	tx100Frame.dataByte1 = (g_Adc16ConversionReturnValue >> 8);


        // Wait for the next cycle.
        vTaskDelayUntil(  &xLastWakeTime, (xFrequency*period1)/portTICK_PERIOD_MS);  //nodo1
        //adc1 += 100;   //nodo3
    }
}

/*!
 * @brief Task responsible for sending the 50ms message.
 */
static void task_50ms(void *pvParameters)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = OS_TICK_PERIOD_50MS;
	volatile uint32_t can_flags = 0;

	// Initialize the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
    	/* Get the flags from the can Driver */
    	can_flags = FLEXCAN_GetStatusFlags(EXAMPLE_CAN);

    	/* Send a Can message */
    	tx50Frame.id = FLEXCAN_ID_STD(tx50Identifier);
    	tx50Frame.format = kFLEXCAN_FrameFormatStandard;
    	tx50Frame.type = kFLEXCAN_FrameTypeData;
    	tx50Frame.length = 8;
    	tx50Xfer.frame = &tx50Frame;
    	tx50Xfer.mbIdx = TX50_MESSAGE_BUFFER_NUM;
    	FLEXCAN_TransferSendNonBlocking(EXAMPLE_CAN, &flexcanHandle, &tx50Xfer);

    	tx50Frame.dataByte0 = (g_Adc16ConversionValue)&0xFF; //nodo2
    	tx50Frame.dataByte1 = (g_Adc16ConversionValue&0xFF00)>>8;;  // nodo2

    	//tx50Frame.dataByte0 = 50;
    	//tx50Frame.dataByte1++;


    	// Send to update ADC
    	if(true == g_Adc16ConversionDoneFlag)
    	{
    		ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
    		g_Adc16ConversionDoneFlag = false;
    	}

        // Wait for the next cycle.
        vTaskDelayUntil( &xLastWakeTime, (xFrequency*period1)/portTICK_PERIOD_MS);
    }
}

/*!
 * @brief Task responsible for checking rx messages.
 */
static void task_rx(void *pvParameters)
{
	volatile uint32_t can_flags = 0;
	flexcan_frame_t* rxFrame;

    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };

	GPIO_PinInit(BOARD_LED_B_GPIO, BOARD_LED_B_GPIO_PIN, &led_config);

	LED_BLUE_OFF();
	// Initialize the xLastWakeTime variable with the current time.
    for (;;)
    {
    	/* Get the flags from the can Driver */
    	can_flags = FLEXCAN_GetStatusFlags(EXAMPLE_CAN);

    	if(!receiving) {
        	/* Start receive data through Rx Message Buffer. */
        	rx1Xfer.frame = &rx1Frame;
        	rx1Xfer.mbIdx = RX1_MESSAGE_BUFFER_NUM;
    		FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rx1Xfer);

        	/* Start receive data through Rx Message Buffer. */
        	rx2Xfer.frame = &rx2Frame;
        	rx2Xfer.mbIdx = RX2_MESSAGE_BUFFER_NUM;
    		FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rx2Xfer);



    		receiving = true;
    	}

    	if(message_received){
    		switch(received_mb_idx){
    		case RX1_MESSAGE_BUFFER_NUM:
    			/* Start the reception over */
            	rx1Xfer.frame = &rx1Frame;
            	rx1Xfer.mbIdx = RX1_MESSAGE_BUFFER_NUM;
        		FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rx1Xfer);
    			rxFrame = &rx1Frame;
    			if((rx1Frame.dataByte0 & 0x02) == 2) {PRINTF("LED ON\r\n"); LED_BLUE_ON();}  //nodo2
    			else {PRINTF("LED OFF\r\n"); LED_BLUE_OFF();}  //nodO2
    			if((rx1Frame.dataByte0 & 0x04) == 4)
    			{
    				lin1d3_ledValueSet(1);
    			}
    			else
    			{
    				lin1d3_ledValueSet(0);
    			}
//    			if((rx1Frame.dataByte0 & 0x04) == 4) {PRINTF("LED ON\r\n"); LED_BLUE_ON();}  //nodo2
//    			else {PRINTF("LED OFF\r\n"); LED_BLUE_OFF();}  //nodO2
    			break;
    		case RX2_MESSAGE_BUFFER_NUM:
    			/* Start the reception over */
            	rx2Xfer.frame = &rx2Frame;
            	rx2Xfer.mbIdx = RX2_MESSAGE_BUFFER_NUM;
        		FLEXCAN_TransferReceiveNonBlocking(EXAMPLE_CAN, &flexcanHandle, &rx2Xfer);
    			rxFrame = &rx2Frame;
    			period1 = rx2Frame.dataByte0;
    			break;
    		default:
    			break;
    		}
    		PRINTF("Received message: MB: %d, ID: 0x%x, data: %d,%d,%d,%d,%d,%d,%d,%d\r\n", received_mb_idx,
    				rxFrame->id>>18,
    				rxFrame->dataByte0, rxFrame->dataByte1, rxFrame->dataByte2, rxFrame->dataByte3,
					rxFrame->dataByte4, rxFrame->dataByte5, rxFrame->dataByte6, rxFrame->dataByte7
					);
    		message_received = false;
    	}
    	vTaskDelay(10);
    }
}

static void test_task(void *pvParameters)
{
	int error;
	lin1d3_nodeConfig_t node_config;
	lin1d3_handle_t* master_handle;
	lin1d3_handle_t* slave_handle;
	/* Set Master Config */
	node_config.type = lin1d3_master_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = MASTER_UART;
	node_config.srcclk = MASTER_UART_CLK_FREQ;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].handler = message_1_callback_master;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].handler = message_2_callback_master;
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].handler = message_3_callback_master;
	/* Init Master node */
	master_handle = lin1d3_InitNode(node_config);
#if !defined(JUST_MASTER)
	/* Set Slave Config */
	node_config.type = lin1d3_slave_nodeType;
	node_config.bitrate = 9600;
	node_config.uartBase = SLAVE_UART;
	node_config.srcclk = SLAVE_UART_CLK_FREQ;
	memset(node_config.messageTable,0, (sizeof(node_config.messageTable[0])*lin1d3_max_supported_messages_per_node_cfg_d));
	node_config.messageTable[0].ID = app_message_id_1_d;
	node_config.messageTable[0].handler = message_1_callback_slave;
	node_config.messageTable[1].ID = app_message_id_2_d;
	node_config.messageTable[1].handler = message_2_callback_slave;
	node_config.messageTable[2].ID = app_message_id_3_d;
	node_config.messageTable[2].handler = message_3_callback_slave;
	/* Init Slave Node*/
	slave_handle = lin1d3_InitNode(node_config);
#endif

	if((NULL == master_handle)
#if !defined(JUST_MASTER)
		|| (NULL == slave_handle)
#endif
	   ){
		error = kStatus_Fail;
	}
	else {
		error = kStatus_Success;
	}

	while (kStatus_Success == error)
    {
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_1_d);
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_2_d);
    	vTaskDelay(200);
    	lin1d3_masterSendMessage(master_handle, app_message_id_3_d);
    }

    vTaskSuspend(NULL);
}


static void	message_1_callback_master(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Master got response to message 1 %d,%d\r\n", message_data[0], message_data[1]);
	g_Adc16ConversionReturnValue = 0u;
	g_Adc16ConversionReturnValue = message_data[0];
	g_Adc16ConversionReturnValue |= (message_data[1]<<8);

}

static void	message_2_callback_master(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Master got response to message 2 %d,%d,%d,%d\r\n", message_data[0], message_data[1], message_data[2], message_data[3]);
}

static void	message_3_callback_master(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Master got response to message 3 %d,%d,%d,%d,%d,%d,%d,%d\r\n",
			message_data[0], message_data[1], message_data[2], message_data[3],
			message_data[4], message_data[5], message_data[6], message_data[7]);
}

static void	message_1_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 1 request\r\n");
	// Send to update ADC
	if(true == g_Adc16ConversionDoneFlag)
	{
		ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
		g_Adc16ConversionDoneFlag = false;
	}
	message_data[0] = g_Adc16ConversionValue & 0xFF;
	message_data[1] = (g_Adc16ConversionValue & 0xFF00)>>8;
}

static void	message_2_callback_slave(void* message)
{
	PRINTF("Slave got message 2 request\r\n");
}

static void	message_3_callback_slave(void* message)
{
	uint8_t* message_data = (uint8_t*)message;
	PRINTF("Slave got message 3 request\r\n");
	message_data[0] = 79;
	message_data[1] = 80;
	message_data[2] = 81;
	message_data[3] = 82;
	message_data[4] = 83;
	message_data[5] = 84;
	message_data[6] = 85;
	message_data[7] = 86;
}





