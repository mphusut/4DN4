/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>

#include "hello_world.h"

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_ftm.h"
#include "fsl_uart.h"
#include "fsl_dspi.h" //todo: download dspi driver

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define FTM_MOTOR FTM0
#define FTM_CHANNEL_DC_MOTOR kFTM_Chnl_0
#define FTM_CHANNEL_SERVO_MOTOR kFTM_Chnl_3
#define TARGET_UART UART4

#define SERIAL_INPUT_DC_SIGN_INDEX 0
#define SERIAL_INPUT_DC_START_INDEX 1
#define SERIAL_INPUT_DC_END_INDEX 4
#define SERIAL_INPUT_SERVO_SIGN_INDEX 5
#define SERIAL_INPUT_SERVO_START_INDEX 6
#define SERIAL_INPUT_SERVO_END_INDEX 8

#define speed_to_dutycycle(dc_speed) dc_speed * 0.025f/100.0f + 0.0615;
#define	angle_to_dutycycle(servo_angle) servo_angle * 0.025f/45.0f + 0.075 ;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */


volatile char ch;
volatile int ch_index = 0;
char* uart_input;
volatile int new_char = 0;
 
int main(void){
	return problem2();
}

/*******************************************************************************
 * Problems
 ******************************************************************************/

int problem1(void)
{
	 uint8_t ch;
	 int dc_speed , servo_angle;
	 float  dc_dutyCycle;
	 float servo_dutyCycle;
	 BOARD_InitBootPins();
	 BOARD_InitBootClocks();

	 setupPWM(FTM_CHANNEL_SERVO_MOTOR);
	 setupPWM(FTM_CHANNEL_DC_MOTOR);

	 /******* Delay *******/
	 for(volatile int i = 0U; i < 1000000; i++)
	 __asm("NOP");

	 //updatePWM_dutyCycle(FTM_CHANNEL_DC_MOTOR, 0.0615);
	 //FTM_SetSoftwareTrigger(FTM_MOTOR, true);

	 scanf("%d %d", &dc_speed, &servo_angle);
	 dc_dutyCycle = dc_speed * 0.025f/100.0f + 0.0615;
	 servo_dutyCycle = servo_angle * 0.025f/45.0f + 0.075 ;

	 updatePWM_dutyCycle(FTM_CHANNEL_SERVO_MOTOR, servo_dutyCycle);
	 updatePWM_dutyCycle(FTM_CHANNEL_DC_MOTOR, dc_dutyCycle);


	 FTM_SetSoftwareTrigger(FTM_MOTOR, true);
	 while(1)
	 {

	 }
}

int problem2(void){
	int dc_speed , servo_angle;
	float dc_dutyCycle, servo_dutyCycle;
	char txbuff[] = "Connected\r\n";
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitDebugConsole();
	
	setupUART(false);
	
	setupPWM(FTM_CHANNEL_SERVO_MOTOR);
	setupPWM(FTM_CHANNEL_DC_MOTOR);

	/******* Delay *******/
	for(volatile int i = 0U; i < 10000000; i++)
		__asm("NOP");
	PRINTF("%s", txbuff);
	
	UART_WriteBlocking(TARGET_UART, txbuff, sizeof(txbuff) - 1);
	FTM_SetSoftwareTrigger(FTM_MOTOR, true);
	
	while (1)
	{
		
		UART_ReadBlocking(TARGET_UART, uart_input, 8);
		drive_dc_and_servo(uart_input);
	}

}

int problem3(){
	char txbuff[] = "Connected\r\n";
	/* Init board hardware. */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitDebugConsole();
	
	setupUART(true);
	
	setupPWM(FTM_CHANNEL_SERVO_MOTOR);
	setupPWM(FTM_CHANNEL_DC_MOTOR);

	/******* Delay *******/
	for(volatile int i = 0U; i < 10000000; i++)
		__asm("NOP");
	PRINTF("%s", txbuff);
	
	UART_WriteBlocking(TARGET_UART, txbuff, sizeof(txbuff) - 1);
	FTM_SetSoftwareTrigger(FTM_MOTOR, true);
	
	while (1)
	{
		if(new_char){
			new_char = 0;
			PRINTF("%c\n", ch);
			strcat(uart_input, &ch);
			if(ch == '\0'){
				drive_dc_and_servo(uart_input);
			}
		}
		
		
	}

	return 0;
}

int problem4(){
	// SPI_write();
	return 0;
}

/*******************************************************************************
 * PWM
 ******************************************************************************/

void setupPWM(ftm_chnl_t chnlNumber)
{
	ftm_config_t ftmInfo;
	ftm_chnl_pwm_signal_param_t ftmParam;
	ftm_pwm_level_select_t pwmLevel = kFTM_HighTrue;
	ftmParam.chnlNumber = chnlNumber;
	ftmParam.level = pwmLevel;
	ftmParam.dutyCyclePercent = 7;
	ftmParam.firstEdgeDelayPercent = 0U;
	ftmParam.enableComplementary = false;
	ftmParam.enableDeadtime = false;
	FTM_GetDefaultConfig(&ftmInfo);
	ftmInfo.prescale = kFTM_Prescale_Divide_128;
	FTM_Init(FTM_MOTOR, &ftmInfo);
	FTM_SetupPwm(FTM_MOTOR, &ftmParam, 1U, kFTM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(
	kCLOCK_BusClk));
	FTM_StartTimer(FTM_MOTOR, kFTM_SystemClock);
}

void updatePWM_dutyCycle(ftm_chnl_t channel, float dutyCycle)
{
	uint32_t cnv, cnvFirstEdge = 0, mod;
	/* The CHANNEL_COUNT macro returns -1 if it cannot match the FTM instance */
	assert(-1 != FSL_FEATURE_FTM_CHANNEL_COUNTn(FTM_MOTOR));
	mod = FTM_MOTOR->MOD;
	if (dutyCycle == 0U)
	{
	/* Signal stays low */
	cnv = 0;
	}
	else
	{
	cnv = mod * dutyCycle;
	/* For 100% duty cycle */
	if (cnv >= mod)
	{
	cnv = mod + 1U;
	}
	}
	FTM_MOTOR->CONTROLS[channel].CnV = cnv;
}

/*******************************************************************************
 * UART utilities
 ******************************************************************************/

void setupUART(bool interrupt_flag)
{
	uart_config_t config;
	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = 57600;
	config.enableTx = true;
	config.enableRx = true;
	config.enableRxRTS = true;
	config.enableTxCTS = true;
	UART_Init(TARGET_UART, &config, CLOCK_GetFreq(kCLOCK_BusClk));

	uart_input = malloc(10*sizeof(char));

	/******** Enable Interrupts *********/
	if(interrupt_flag){
		UART_EnableInterrupts(TARGET_UART, kUART_RxDataRegFullInterruptEnable);
		EnableIRQ(UART4_RX_TX_IRQn);
	}
	

}

void parse_UARTInput(int* dc_speed_ptr, int* servo_angle_ptr, const char* serial_input){
	dc_speed_ptr = malloc(sizeof(int));
	servo_angle_ptr = malloc(sizeof(int));

	*dc_speed_ptr = parse_UARTInput_DC(serial_input);
	*servo_angle_ptr = parse_UARTInput_Servo(serial_input);
}

int parse_UARTInput_DC(const char* serial_input){
	return parseSignedStringToInt(
		SERIAL_INPUT_DC_SIGN_INDEX,
		SERIAL_INPUT_DC_START_INDEX,
		SERIAL_INPUT_DC_END_INDEX,
		serial_input
	);
}

int parse_UARTInput_Servo(const char* serial_input){
	return parseSignedStringToInt(
		SERIAL_INPUT_SERVO_SIGN_INDEX,
		SERIAL_INPUT_SERVO_START_INDEX,
		SERIAL_INPUT_SERVO_END_INDEX,
		serial_input
	);
}

/*******************************************************************************
 * SPI Utilities
 ******************************************************************************/

status_t SPI_write(uint8_t regAddress, uint8_t value){
    dspi_transfer_t masterXfer;
    uint8_t *masterTxData = (uint8_t*)malloc(1+2);
    uint8_t *masterRxData = (uint8_t*)malloc(1 + 2);

    masterTxData[0] = regAddress & 0x7F | 0x80; //Set the most significant bit to '1' for write operations
    masterTxData[1] = regAddress & 0x80; //Clear the least significant 7 bits
    masterTxData[2] = value;
    
    masterXfer.txData = masterTxData;
    masterXfer.rxData = masterRxData;
    
    masterXfer.dataSize = 1 + 2;
    masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    status_t ret = DSPI_MasterTransferBlocking(SPI1, &masterXfer); // ?: Is this writing

    free(masterTxData);
    free(masterRxData);

    return ret;
}

status_t SPI_read(uint8_t regAddress, uint8_t *rxBuff, uint8_t rxBuffSize)
{
    dspi_transfer_t masterXfer;
    uint8_t *masterTxData = (uint8_t*)malloc(rxBuffSize + 2);
    uint8_t *masterRxData = (uint8_t*)malloc(rxBuffSize + 2);
    masterTxData[0] = regAddress & 0x7F; //Clear the most significant bit
    masterTxData[1] = regAddress & 0x80; //Clear the least significant 7 bits
    masterXfer.txData = masterTxData;
    masterXfer.rxData = masterRxData;
    masterXfer.dataSize = rxBuffSize + 2;
    masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;
    status_t ret = DSPI_MasterTransferBlocking(SPI1, &masterXfer);
    memcpy(rxBuff, &masterRxData[2], rxBuffSize);
    free(masterTxData);
    free(masterRxData);
    return ret;
}

/*******************************************************************************
 * Parsing utilities
 ******************************************************************************/

int parseSignedStringToInt(int signIndex, int start, int end, const char* serial_input){
	 int rslt = 0;
	for(int i = start; i < end; i++){
		rslt = 10 * rslt + atoi(serial_input[i]);
	}
	rslt *= serial_input[signIndex] == '+' ? 1 : -1;
	
	return rslt;
}

void UART4_RX_TX_IRQHandler()
{
	UART_GetStatusFlags(TARGET_UART);
	ch = UART_ReadByte(TARGET_UART);
	new_char = 1;
}

void drive_dc_and_servo(char* formatted_vals){
	int dc_speed , servo_angle;
	float dc_dutyCycle, servo_dutyCycle;

	PRINTF("%c\r\n", *formatted_vals);

	// parsing function
	parse_UARTInput(&dc_speed, &servo_angle, formatted_vals);

	//conversion to dutycycle
	dc_dutyCycle = speed_to_dutycycle(dc_speed);
	servo_dutyCycle = angle_to_dutycycle(servo_angle);

	// drive here
	updatePWM_dutyCycle(FTM_CHANNEL_SERVO_MOTOR, servo_dutyCycle);
	updatePWM_dutyCycle(FTM_CHANNEL_DC_MOTOR, dc_dutyCycle);
	
	printf("%d %d", &dc_speed, &servo_angle);
}