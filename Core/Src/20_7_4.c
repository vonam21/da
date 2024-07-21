/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define HIGH_LOAD 1
#define LOW_LOAD 0
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <LCD.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

uint8_t dataRX[1];
uint8_t dataTX[10] = "da gui\n";
uint8_t dataTX1[10] = "trung roi\n";
uint8_t save_data_chuoi_OK[2] = {0, 0};
uint8_t save_data_chuoi_CMQTT[5] = {0, 0, 0, 0, 0};
uint8_t so_sanh_chuoi_OK[2] = "OK";
uint8_t so_sanh_chuoi_CMQTT[5] = "CMQTT";
uint8_t save_data_chuoi_PB_DONE[7] = {0, 0, 0, 0, 0, 0, 0};
uint8_t so_sanh_chuoi_PB_DONE[7] = "PB DONE";
uint8_t save_data_chuoi_RX_PAYLOAD[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t so_sanh_chuoi_RX_PAYLOAD[15] = "CMQTTRXPAYLOAD:";
uint8_t so_sanh_chuoi_xuong_dong[1] = "\n";
uint8_t save_data_chuoi_RX_PAYLOAD_DATA[1] = {0};

uint8_t data_SIM_RESET[20] = "AT+CRESET\r\n";
uint8_t data_SIM_TEST[20] = "AT\r\n";
uint8_t data_SIM0[40] = "ATE0\r\n";
uint8_t data_SIM1[40] = "AT+CGEREP=0\r\n";
uint8_t data_SIM_baud[40] = "AT+IPR=9600\r\n";
uint8_t data_SIM2[40] = "AT+CMQTTSTART\r\n";
uint8_t data_SIM3[40] = "AT+CMQTTACCQ=0,\"clientID\"\r\n";
uint8_t data_SIM4[70] = "AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",60,0\r\n";
uint8_t data_SIM5[40] = "AT+CMQTTTOPIC=0,5\r\n";
//uint8_t data_SIM5[40] = "AT+CMQTTSUB=0,5,1\r\n";
uint8_t data_SIM6[40] = "vonam";
uint8_t data_SIM7[40] = "AT+CMQTTPAYLOAD=0,48\r\n";
uint8_t data_SIM8[40] = "nhietdo 24 do am 67\r\n";
uint8_t data_SIM9[40] = "AT+CMQTTPUB=0,1,60,0,0\r\n";
//uint8_t data_SIM10[40] = "AT+CMQTTSUB=0,7,1\r\n";
uint8_t data_SIM10[40] = "AT+CMQTTTOPIC=0,7\r\n";
uint8_t data_SIM11[40] = "nhietdo";
uint8_t cmd_SIM_UNSUBTOPIC[40] = "AT+CMQTTUNSUBTOPIC=0,5\r\n";
uint8_t input_SIM_UNSUBTOPIC[40] = "vonam";
uint8_t cmd_unsub_topic[40] = "AT+CMQTTUNSUB=0,1\r\n";

uint8_t cmd_check_mqtt[40] = "AT+CMQTTCONNECT?\r\n";




uint8_t data_RX[400];
uint8_t data_RX_0[60];
uint8_t data_RX_1[60];
uint8_t data_RX_2[60];
uint8_t data_RX_3[60];
uint8_t data_RX_4[60];
uint8_t data_RX_5[60];
uint8_t data_RX_6[60];
uint8_t data_RX_7[60];
uint8_t data_RX_8[60];
uint8_t data_RX_9[60];
uint8_t data_RX_10[60];
uint8_t data_RX_11[60];
uint8_t data_RX_12[60];
uint8_t data_RX_13[60];
uint8_t data_RX_14[60];
uint8_t cmd_nhap_topic[1] = {'>'};
uint8_t cmd_OK[2] = {'O', 'K'};
uint8_t cmd_MQTT[16] = {'+', 'C', 'M', 'Q', 'T', 'T', 'R', 'X', 'P', 'A', 'Y', 'L', 'O', 'A', 'D', ':'};

volatile int count_test_re = 0;
int cout_0 = 0;
int cout_1 = 0;
int cout_2 = 0;
int cout_3 = 0;
int cout_4 = 0;
int cout_5 = 0;
int cout_6 = 0;
int cout_7 = 0;
int cout_8 = 0;
int cout_9 = 0;
int cout_10 = 0;
int cout_11 = 0;
int cout_12 = 0;
int check_true = 0;
uint8_t match_buf[50];
int flag_pass = 0;
int flag_nhap_topic = 0;
int flag_cho_phep_gan = 0;
int count_flag_gan = 0;

int count_again_trans = 0;

int flag_check_OK = 0;
int flag_check_CMQTT = 0;
int flag_check_PB_DONE = 0;
int flag_check_RX_PAYLOAD = 0;
int flag_pass_RX_PAYLOAD = 0;
int flag_phan_biet_xuong_dong_data = 0;
int flag_pass_RX_PAYLOAD_next = 0;
int count_data_pass_RX_PAYLOAD = 0;
int flag_pass_all_RX_PAYLOAD = 0;
int du_time_to_send_data = 0;

uint8_t message[40];
volatile int led = 0, quat = 0, bom = 0;
int nguong_adc_quang_tro = 3000, nguong_adc_do_am_dat = 3000;
int nguong_nhiet_do = 30;
int mode = 1;
int flag_mode_bang_tay = 0;
int flag_mode_tu_dong = 1;

uint32_t debounceDelay_mode = 50;        // Th�?i gian ch�? debounce (miliseconds)
uint32_t debounceDelay = 150;            // Th�?i gian ch�? debounce (miliseconds)
volatile uint32_t lastDebounceTime = 0;  // Th�?i gian debounce cuối cùng
uint32_t currentTime = 0;
volatile uint32_t lastDebounceTime2 = 0;  // Th�?i gian debounce cuối cùng
uint32_t currentTime2 = 0;
volatile uint32_t lastDebounceTime3 = 0;  // Th�?i gian debounce cuối cùng
uint32_t currentTime3 = 0;
volatile uint32_t lastDebounceTime4 = 0;  // Th�?i gian debounce cuối cùng
uint32_t currentTime4 = 0;

int num1 = 0;
int num2 = 0;
int num3 = 0;
int num4 = 0;
int num5 = 0;
int num6 = 0;
int num7 = 0;
int num8 = 0;
int num9 = 0;
int num0 = 0;
int num_sao = 0;
int num_thang = 0;
volatile int a = 0, b = 0, c = 0, d = 0;

volatile uint32_t tick_lan_truoc = 0;
volatile uint32_t tick_hien_tai = 0;
volatile uint32_t thoi_gian_tick = 0;
uint8_t tick[100];
uint8_t data_dht11[4] = {0x00, 0x00, 0x00, 0x00};
int count_tick = 0;

uint64_t count = 0;
int count_state_lcd = 1;
char nhiet_do[20], do_am[20];
int adc_quang_tro = 0, adc_do_am_dat = 0;

int flag_send_data = 0;
int flag_ngat_keypad = 1;
int flag_number = 0;
int flag_chuyen_lcd = 1;
int flag_ngat_uart_xong_message = 0;
int flag_lan_dau_nhap_mat_khau = 0;
uint8_t message_gui_server[75];
uint8_t message_gui_server1[2];

char mat_khau_dung[6] = {1, 2, 3, 4, 5, 6};
char mat_khau_user[6] = {0, 0, 0, 0, 0, 0};
int count_mat_khau = 0;
int count_may_chu_nhap_vao = 0;
int flag_di_qua_nhap_mat_khau = 0;
int flag_timer4 = 0;

bool flag_disable_nut_nhan = 0;
int status_previous_led = 0;
int status_previous_quat = 0;
int status_previous_bom = 0;

int flag_start_button = 1;

int flag_number_state1 = 0;
int num1_state1 = 0;
int num2_state1 = 0;
int num3_state1 = 0;
int led_state1 = 0;
int num4_state1 = 0;
int num5_state1 = 0;
int num6_state1 = 0;
int num7_state1 = 0;
int num8_state1 = 0;
int num9_state1 = 0;
int num0_state1 = 0;
int quat_state1 = 0;
int bom_state1 = 0;
int num_sao_state1 = 0;
int num_thang_state1 = 0;
int mode_state1 = 1;

int state_ngat1 = 1;
int state_ngat2 = 0;

int count_timer1 = 0;
int flag_ngat_nut_nhan = 0;
int flag_ngat_uart = 0;

int flag_switch_timer3 = 0;

int FLAG_CHECK_SEND_DATA = 0;
int FLAG_CHECK_SEND_DATA1 = 0;

int count_err = 0;

int flag_k_cho_phep_nhan =0;

int flag_pass_input =0;

volatile int data_avail_mqtt = 0;
void reconnect_mqtt(void);
void update_lcd(void);
void xu_ly_tick_dht11(uint8_t *tick, uint8_t *data_dht11);
void doc_dht11(void);
void update(void);
void init_sim(void);

int check_buffer_RX(void)
{
	for (int i = 0; i < sizeof(data_RX); i++) {
		if (data_RX[i] != 0) {
			return 1;
		}
	}
	return 0;
}

int compare_n_char(uint8_t *buffer_1, int leng_buf_1, uint8_t *buffer_2, int leng_buf_2, uint8_t *match_buf)
{
	int cout = 0, i_global = 0;
	for (int i = 0; i < leng_buf_2; i++) {
		if (buffer_1[i] != buffer_2[i]) {
			return -1;
		}
		i_global++;
	}
	while (i_global < leng_buf_2) {
		match_buf[cout] = buffer_1[i_global];
		cout++;
		i_global++;
	}
	return 0;
}

void unsub_topic(void)
{
	int err;
	uint8_t cmd_SIM_UNSUBTOPIC[40] = "AT+CMQTTUNSUBTOPIC=0,5\r\n";
	uint8_t input_SIM_UNSUBTOPIC[40] = "vonam\r\n";
UNSUB_AGAIN:
	//	  flag_pass=0;
	//      count_again_trans=0;
	//	  err = check_buffer_RX();
	//	  if(err){
	//		  HAL_Delay(300);
	//		  goto UNSUB_AGAIN;
	//	  }

	HAL_UART_Transmit(&huart1, cmd_SIM_UNSUBTOPIC, strlen((const char *)cmd_SIM_UNSUBTOPIC), 1000);
	HAL_Delay(1000);
	update();
	update_lcd();
CHECK_UNSUB_RX:
	flag_pass = 0;
	count_again_trans = 0;
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(300);
		goto CHECK_UNSUB_RX;
	}
	HAL_UART_Transmit(&huart1, input_SIM_UNSUBTOPIC, strlen((const char *)input_SIM_UNSUBTOPIC), 1000);
	while (flag_pass == 0) {
		HAL_Delay(500);
		count_again_trans++;
		if (count_again_trans >= 10) {
			//			  goto UNSUB_AGAIN;
//			goto CHECK_UNSUB_RX2;
			break;
		}
	}
//CHECK_UNSUB_RX2:
//	flag_pass = 0;
//	count_again_trans = 0;
//	err = check_buffer_RX();
//	if (err) {
//		HAL_Delay(300);
//		goto CHECK_UNSUB_RX2;
//	}
//	HAL_UART_Transmit(&huart1, cmd_unsub_topic, strlen((const char *)cmd_unsub_topic), 1000);
//	while (flag_pass == 0) {
//		HAL_Delay(500);
//		count_again_trans++;
//		if (count_again_trans >= 10) {
//			goto UNSUB_AGAIN;
//		}
//	}
}
int leng,leng1;
void send_data_to_server(void)
{
	FLAG_CHECK_SEND_DATA = 1;
	flag_check_RX_PAYLOAD = 0;
	flag_check_OK = 1;
	int err;
	count_err=0;
	flag_pass_input =0;
UART6:

	flag_pass = 0;
	count_again_trans = 0;
	flag_nhap_topic = 0;
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(300);
		goto UART6;
	}
	if(HAL_UART_Transmit(&huart1, data_SIM5, strlen((const char *)data_SIM5), 3000) != HAL_OK){
		return;
	}
	HAL_Delay(2000);
	update();
	update_lcd();
	////////////////////////////////debug///////////////////
//	HAL_UART_Transmit(&huart1, cmd_check_mqtt, strlen((const char *)cmd_check_mqtt), 1000);

	////////////////////////////////debug///////////////////
	while (flag_pass_input == 0) {
		HAL_Delay(500);
		count_again_trans++;
		if (count_err >= 10){
			count_err=0;
			init_sim();
			goto UART6;
		}
		if (count_again_trans >= 8) {
			count_err++;
//			unsub_topic();
//			init_sim();
			goto UART6;
		}
	}
	flag_pass_input = 0;
	count_again_trans = 0;

BUFFER6:
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(300);
		goto BUFFER6;
	}
	HAL_UART_Transmit(&huart1, data_SIM6, strlen((const char *)data_SIM6), 3000);
	while (flag_pass == 0) {
		HAL_Delay(500);
		count_again_trans++;
		if (count_err >= 10){
			count_err=0;
			init_sim();
			goto UART6;
		}
		if (count_again_trans >= 8) {
			count_err++;
			goto UART6;
		}
	}

//	  while(flag_pass==0);
////////////////////// 5 6//////////////////
	count_err=0;
UART7:
	FLAG_CHECK_SEND_DATA1 = 1;
	flag_pass = 0;
	count_again_trans = 0;
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(300);
		goto UART7;
	}
	leng1 = strlen((const char *)data_SIM7);
	HAL_UART_Transmit(&huart1, data_SIM7, strlen((const char *)data_SIM7), 3000);
	HAL_Delay(1000);
	update();
	update_lcd();
	while (flag_pass_input == 0) {
		HAL_Delay(500);
		count_again_trans++;
		if (count_err >= 10){
			count_err=0;
			init_sim();
			goto UART6;
		}
		if (count_again_trans >= 8) {
			count_err++;
			goto UART7;
		}
	}
	flag_pass_input = 0;
	count_again_trans = 0;
	/////////////////////////////////////////////////////////////
DELAY_SEND_DATA:
	data_avail_mqtt = 0;
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(300);
		goto DELAY_SEND_DATA;
	}
	update();
	update_lcd();
	a = led % 2;
	b = quat % 2;
	c = bom % 2;
	d = mode % 2;
	int check_led = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	int check_quat = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	int check_bom = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
	sprintf((char *)message_gui_server,
	    "%02d.%02d %02d.%02d %04d %04d %04d %04d %02d %d %d %d %d %d %d %d",
	    data_dht11[2], data_dht11[3], data_dht11[0], data_dht11[1], adc_quang_tro, adc_do_am_dat, nguong_adc_quang_tro, nguong_adc_do_am_dat, nguong_nhiet_do, a, b, c, d, check_led, check_quat,
	    check_bom);
	//////////////////////////////////////////////////////////////////////
	leng = strlen((const char *)message_gui_server);
	HAL_UART_Transmit(&huart1, message_gui_server,leng, 3000);
	while (flag_pass == 0) {
		HAL_Delay(500);
		count_again_trans++;
		if (count_err >= 10)
		{
			init_sim();
			goto UART6;
		}
		if (count_again_trans >= 8) {
//			unsub_topic();
			count_err++;
			goto UART7;
		}
	}
	FLAG_CHECK_SEND_DATA1 = 0;

//	  //////////////////// 7 8//////////////////////////////
//
	count_err=0;
UART9:
	flag_pass = 0;
	count_again_trans = 0;
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(300);
		goto UART9;
	}
	if (data_avail_mqtt) {
		data_avail_mqtt=0;
//		count_err++;
		//		  unsub_topic();
		goto UART6;
	}
//	if (count_err >= 10)
//		reconnect_mqtt();
	HAL_UART_Transmit(&huart1, data_SIM9, strlen((const char *)data_SIM9), 3000);
	flag_k_cho_phep_nhan =1;
	while (flag_pass == 0) {
		HAL_Delay(500);
		count_again_trans++;
//		if (data_avail_mqtt) {
//			data_avail_mqtt=0;
//			//			  unsub_topic();
//			goto UART6;
//		}
		if(count_err >=10)
		{
			count_err=0;
			init_sim();
			goto UART6;
		}
		if (count_again_trans >= 8) {
			count_err++;
			goto UART9;
		}
	}
	flag_k_cho_phep_nhan=0;
	count_err = 0;
	flag_check_RX_PAYLOAD = 1;
	flag_check_OK = 0;
	FLAG_CHECK_SEND_DATA = 0;
}

void reconnect_mqtt(void)
{
	count_err = 0;
	int err;
UART4_MQTT:
	flag_pass = 0;
	count_again_trans = 0;
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(1000);
		goto UART4_MQTT;
	}
	HAL_UART_Transmit(&huart1, data_SIM4, strlen((const char *)data_SIM4), 3000);  // 60bytes
	while (flag_pass == 0) {
		HAL_Delay(500);
		count_again_trans++;
		if (flag_pass == 1) {
			break;
		}
		if (count_again_trans >= 10) {
			goto UART4_MQTT;
		}
	}
	flag_pass = 0;
/////////////////////	  /////////////////////////////////////////////////////////
UART5_MQTT:
	flag_pass = 0;
	count_again_trans = 0;
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(1000);
		goto UART5_MQTT;
	}
	HAL_UART_Transmit(&huart1, data_SIM10, (uint16_t)strlen((const char *)data_SIM10), 3000);
	HAL_Delay(1000);
BUFFER5_MQTT:
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(1000);
		goto BUFFER5_MQTT;
	}
	HAL_UART_Transmit(&huart1, data_SIM11, (uint16_t)strlen((const char *)data_SIM11), 3000);
	while (flag_pass == 0) {
		HAL_Delay(500);
		count_again_trans++;
		if (flag_pass == 1) {
			break;
		}
		if (count_again_trans >= 10) {
			goto UART5_MQTT;
		}
	}
}

void init_sim(void)
{
	flag_check_RX_PAYLOAD = 0;
	flag_check_OK = 1;
	int err = 0;

//	  flag_pass = 0;
UART_SIM_RESET:
		flag_pass = 0;
		count_again_trans = 0;
		err = check_buffer_RX();
		if (err) {
			HAL_Delay(1000);
			goto UART_SIM_RESET;
		}
	  HAL_UART_Transmit(&huart1, data_SIM_RESET, strlen((const char*)data_SIM_RESET), 3000);
		while (flag_pass == 0) {
			HAL_Delay(500);
			count_again_trans++;
			if (flag_pass == 1) {
				break;
			}
			if (count_again_trans >= 10) {
				goto UART_SIM_RESET;
			}
		}
	  HAL_Delay(13000);
UART_SIM_TEST:
	flag_pass = 0;
	count_again_trans = 0;
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(1000);
		goto UART_SIM_TEST;
	}
	HAL_UART_Transmit(&huart1, data_SIM_TEST, strlen((const char *)data_SIM_TEST), 3000);
	while (flag_pass == 0) {
		HAL_Delay(500);
		count_again_trans++;
		if (flag_pass == 1) {
			break;
		}
		if (count_again_trans >= 10) {
			goto UART_SIM_TEST;
		}
	}
//	  HAL_Delay(10000);
//UART0:
//	flag_pass = 0;
//	count_again_trans = 0;
//	err = check_buffer_RX();
//	if (err) {
//		HAL_Delay(1000);
//		goto UART0;
//	}
//	HAL_UART_Transmit(&huart1, data_SIM0, strlen((const char *)data_SIM0), 1000);
//	while (flag_pass == 0) {
//		HAL_Delay(500);
//		count_again_trans++;
//		if (flag_pass == 1) {
//			break;
//		}
//		if (count_again_trans >= 10) {
//			goto UART0;
//		}
//	}
///////////////////////////////////////////////////////////////////
UART1:
	flag_pass = 0;
	count_again_trans = 0;
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(1000);
		goto UART1;
	}
	HAL_UART_Transmit(&huart1, data_SIM1, strlen((const char *)data_SIM1), 3000);
	while (flag_pass == 0) {
		HAL_Delay(500);
		count_again_trans++;
		if (flag_pass == 1) {
			break;
		}
		if (count_again_trans >= 10) {
			goto UART1;
		}
	}
////////////////////////////////////////////1//////////////////
UART2:
	flag_pass = 0;
	count_again_trans = 0;
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(1000);
		goto UART2;
	}
	HAL_UART_Transmit(&huart1, data_SIM2, strlen((const char *)data_SIM2), 3000);
	while (flag_pass == 0) {
		HAL_Delay(500);
		count_again_trans++;
		if (flag_pass == 1) {
			break;
		}
		if (count_again_trans >= 10) {
			goto UART2;
		}
	}
	////////////////2///////////////////
UART3:
	flag_pass = 0;
	count_again_trans = 0;
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(1000);
		goto UART3;
	}
	HAL_UART_Transmit(&huart1, data_SIM3, strlen((const char *)data_SIM3), 3000);
	while (flag_pass == 0) {
		HAL_Delay(500);
		count_again_trans++;
		if (flag_pass == 1) {
			break;
		}
		if (count_again_trans >= 10) {
			goto UART3;
		}
	}
	////////////////////////3/////////////////////
UART4:
	flag_pass = 0;
	count_again_trans = 0;
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(1000);
		goto UART4;
	}
	HAL_UART_Transmit(&huart1, data_SIM4, strlen((const char *)data_SIM4), 3000);  // 60bytes
	while (flag_pass == 0) {
		HAL_Delay(500);
		count_again_trans++;
		if (flag_pass == 1) {
			break;
		}
		if (count_again_trans >= 10) {
			goto UART4;
		}
	}
	flag_pass_input = 0;
/////////////////////	  /////////////////////////////////////////////////////////
UART5:
	flag_pass = 0;
	count_again_trans = 0;
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(1000);
		goto UART5;
	}
	HAL_UART_Transmit(&huart1, data_SIM10, (uint16_t)strlen((const char *)data_SIM10), 3000);
	HAL_Delay(2000);
	while (flag_pass_input == 0) {
		HAL_Delay(500);
		count_again_trans++;
		if (count_err >= 10){
			count_err=0;
			init_sim();
			goto UART5;
		}
		if (count_again_trans >= 8) {
			count_err++;
//			unsub_topic();
			goto UART5;
		}
	}
	flag_pass_input = 0;
BUFFER5:
	err = check_buffer_RX();
	if (err) {
		HAL_Delay(1000);
		goto BUFFER5;
	}
	HAL_UART_Transmit(&huart1, data_SIM11, (uint16_t)strlen((const char *)data_SIM11), 3000);
	while (flag_pass == 0) {
		HAL_Delay(500);
		count_again_trans++;
		if (flag_pass == 1) {
			break;
		}
		if (count_again_trans >= 10) {
			goto UART5;
		}
	}

	/////////// kich hoat nhan res tu command sim topic
	/*flag_pass = 0;
	count_again_trans = 0;
	HAL_UART_Transmit(&huart1, data_SIM5, strlen((const char *)data_SIM5), 1000);
		HAL_Delay(1000);
		update();
		update_lcd();
ABC:		err = check_buffer_RX();
		if (err) {
			HAL_Delay(1000);
			goto ABC;
		}
		HAL_UART_Transmit(&huart1, data_SIM6, strlen((const char *)data_SIM6), 1000);
		while (flag_pass == 0) {
			HAL_Delay(500);
			count_again_trans++;
			if (count_err >= 10){
				reconnect_mqtt();
			}
			if (count_again_trans >= 10) {
				//			  unsub_topic();
				count_err++;
			}
		}*/
	////////////

	flag_pass_input=0;
	flag_check_RX_PAYLOAD = 1;
	flag_check_OK = 0;
	flag_pass_all_RX_PAYLOAD = 0;

}
void update(void)
{
	switch (mode % 2) {
		case 0: {
			flag_disable_nut_nhan = 0;
			flag_mode_bang_tay = 1;
			flag_mode_tu_dong = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
			status_previous_led = led;
			status_previous_quat = quat;
			status_previous_bom = bom;
			if ((led % 2) == 1) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, HIGH_LOAD);
			} else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, LOW_LOAD);
			}
			if ((quat % 2) == 1) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, HIGH_LOAD);
			} else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, LOW_LOAD);
			}
			if ((bom % 2) == 1) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, HIGH_LOAD);
			} else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, LOW_LOAD);
			}
			break;
		}
		case 1: {
			if (flag_di_qua_nhap_mat_khau == 1) {
				flag_disable_nut_nhan = 1;
			} else {
				flag_disable_nut_nhan = 0;
			}
			flag_mode_tu_dong = 1;
			flag_mode_bang_tay = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
			int check_status_pre = ((status_previous_led % 2) != (led % 2)) || ((status_previous_quat % 2) != (quat % 2)) || ((status_previous_bom % 2) != (bom % 2));
			status_previous_led = led;
			status_previous_quat = quat;
			status_previous_bom = bom;
			if (check_status_pre) {
				if ((count_state_lcd % 4) != 1) {
					flag_ngat_keypad = 1;
				}
			}
			if (adc_quang_tro > nguong_adc_quang_tro) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, HIGH_LOAD);
				led = 1;
			} else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, LOW_LOAD);
				led = 0;
			}
			if (data_dht11[2] > nguong_nhiet_do) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, HIGH_LOAD);
				quat = 1;
			} else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, LOW_LOAD);
				quat = 0;
			}
			if (adc_do_am_dat > nguong_adc_do_am_dat) {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, HIGH_LOAD);
				bom = 1;
			} else {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, LOW_LOAD);
				bom = 0;
			}
			break;
		}
	}
}

void update_lcd(void)
{
	if (flag_ngat_keypad) {
		flag_ngat_keypad = 0;
		switch (count_state_lcd % 4) {
			case 0: {
				lcd_clear();
				lcd_put_cur(0, 0);
				lcd_send_string(nhiet_do);
				lcd_put_cur(1, 0);
				lcd_send_string(do_am);
				break;
			}
			case 1: {
				if (flag_chuyen_lcd) {
					count_mat_khau = -1;
					count_may_chu_nhap_vao = 0;
					flag_lan_dau_nhap_mat_khau = 1;
					flag_chuyen_lcd = 0;
					lcd_clear();
					lcd_put_cur(0, 1);
					lcd_send_string("Nhap mat khau:");
					lcd_put_cur(1, 0);
				}
				count_mat_khau++;
				count_may_chu_nhap_vao++;
				if (count_may_chu_nhap_vao >= 2) {
					if (count_may_chu_nhap_vao == 2) {
						count_mat_khau = 0;
					}
					if ((flag_number <= 9) && (flag_number >= 0)) {
						mat_khau_user[count_mat_khau] = flag_number;
					}
					switch (flag_number) {
						case 1: {
							lcd_send_string("*");
							break;
						}
						case 2: {
							lcd_send_string("*");
							break;
						}
						case 3: {
							lcd_send_string("*");
							break;
						}
						case 4: {
							lcd_send_string("*");
							break;
						}
						case 5: {
							lcd_send_string("*");
							break;
						}
						case 6: {
							lcd_send_string("*");
							break;
						}
						case 7: {
							lcd_send_string("*");
							break;
						}
						case 8: {
							lcd_send_string("*");
							break;
						}
						case 9: {
							lcd_send_string("*");
							break;
						}
						case 0: {
							lcd_send_string("*");
							break;
						}
						case 11: {
							volatile int value_bang = 1;
							for (int i = 0; i < 6; i++) {
								if (mat_khau_user[i] != mat_khau_dung[i]) {
									value_bang = 0;
									break;
								}
							}
							for (int i = 0; i < 6; i++) {
								mat_khau_user[i] = 0;
							}
							if (value_bang) {
								count_state_lcd = 2;
								flag_ngat_keypad = 1;
								lcd_clear();
								lcd_put_cur(0, 1);
								lcd_send_string("Mat khau dung ->>");
								flag_di_qua_nhap_mat_khau = 1;
								HAL_Delay(2000);
							} else {
								count_state_lcd = 1;
								flag_chuyen_lcd = 1;
								flag_ngat_keypad = 1;
								lcd_clear();
								lcd_put_cur(0, 1);
								lcd_send_string("Mat khau sai ");
								lcd_put_cur(1, 1);
								lcd_send_string("Moi nhap lai  ");
								HAL_Delay(2000);
							}
							break;
						}
						default: {
							lcd_send_string("0");
							break;
						}
					}
				}
				break;
			}
			case 2: {
				lcd_clear();
				lcd_put_cur(0, 0);
				lcd_send_string("Den  Quat  Bom");
				lcd_put_cur(1, 0);
				if (led % 2 == 1) {
					lcd_send_string("Bat  ");
				} else {
					lcd_send_string("Tat  ");
				}
				if (quat % 2 == 1) {
					lcd_send_string("Bat  ");
				} else {
					lcd_send_string("Tat  ");
				}
				if (bom % 2 == 1) {
					lcd_send_string(" Bat  ");
				} else {
					lcd_send_string(" Tat  ");
				}
				break;
			}
			case 3: {
				lcd_clear();
				lcd_put_cur(0, 5);
				lcd_send_string("Mode ");
				lcd_put_cur(1, 5);
				if (mode % 2) {
					lcd_send_string("Tu Dong");
				} else {
					lcd_send_string("Bang Tay");
				}
				break;
			}
		}
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Stop_IT(&htim3);
	//  HAL_UART_Receive_IT(&huart1, dataRX, 1);
	HAL_UART_Receive_DMA(&huart1, data_RX, sizeof(data_RX));
	HAL_TIM_Base_Start_IT(&htim4);
	lcd_init();
	lcd_clear();
	lcd_put_cur(0, 3);
	lcd_send_string("HELLO LCD");
	init_sim();
	HAL_TIM_Base_Start_IT(&htim1);

	//  send_data_to_server();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//	  flag_check_RX_PAYLOAD =0;
		//	  flag_check_OK =1;
		//	  flag_pass = 0;
		//	  HAL_UART_Transmit(&huart1, data_SIM1, strlen(data_SIM1), 10);
		//	  while(flag_pass==0);
		//	  //////////////1//////////////////
		//	  flag_pass = 0;
		//	  HAL_UART_Transmit(&huart1, data_SIM2, strlen(data_SIM2), 10);
		//	  while(flag_pass==0);
		//	  ////////////////2///////////////////
		//	  flag_pass = 0;
		//	  HAL_UART_Transmit(&huart1, data_SIM3, strlen(data_SIM3), 10);
		//	  while(flag_pass==0);
		//	  ////////////////////////3/////////////////////
		//	  flag_pass = 0;
		//	  HAL_UART_Transmit(&huart1, data_SIM4, strlen(data_SIM4), 10);
		//	  while(flag_pass==0);
		//////////////////////4//////////////////////////
		//	  flag_pass = 0;
		//	  HAL_UART_Transmit(&huart1, data_SIM5, strlen(data_SIM5), 10);
		//	  HAL_Delay(100);
		//	  HAL_UART_Transmit(&huart1, data_SIM6, strlen(data_SIM6), 10);
		//	  while(flag_pass==0);
		//	  ////////////////////// 5 6//////////////////
		//	  flag_pass = 0;
		//	  HAL_UART_Transmit(&huart1, data_SIM7, strlen(data_SIM7), 10);
		//	  HAL_Delay(100);
		//	  HAL_UART_Transmit(&huart1, data_SIM8, strlen(data_SIM8), 10);
		//	  while(flag_pass==0);
		//	  //////////////////// 7 8//////////////////////////////
		//	  flag_pass = 0;
		//	  HAL_UART_Transmit(&huart1, data_SIM9, strlen(data_SIM9), 10);
		//	  while(flag_pass==0);
		///////////////////9 ///////////////////////

		//	  flag_check_RX_PAYLOAD =1;
		//	  flag_check_OK =0;
		//	  flag_pass_all_RX_PAYLOAD = 0;
		//	  HAL_UART_Transmit(&huart1, data_SIM10, strlen(data_SIM10), 10);
		//	  HAL_Delay(100);
		//	  HAL_UART_Transmit(&huart1, data_SIM11, strlen(data_SIM11), 10);
		update();
		if (flag_ngat_uart_xong_message) {
			flag_ngat_uart_xong_message = 0;
			for (int i = 0; i < sizeof(message); i++) {
				message[i] = 0;
			}
		}

		update();
		update_lcd();

		if (flag_send_data) {
			flag_send_data = 0;
			doc_dht11();
			HAL_Delay(10);
			xu_ly_tick_dht11(tick, data_dht11);

			sprintf(nhiet_do, "Nhiet do: %d.%doC", data_dht11[2], data_dht11[3]);
			sprintf(do_am, "Do am:  %d.%d ", data_dht11[0], data_dht11[1]);
			HAL_ADC_Start(&hadc1);
			HAL_Delay(50);
			HAL_ADC_Start(&hadc2);
			HAL_Delay(50);
			adc_quang_tro = HAL_ADC_GetValue(&hadc1);  // đ ?c adc quang trở
			HAL_Delay(50);
			adc_do_am_dat = HAL_ADC_GetValue(&hadc2);
			HAL_Delay(50);
			HAL_ADC_Stop(&hadc1);
			HAL_ADC_Stop(&hadc2);
			//			HAL_NVIC_DisableIRQ(EXTI2_IRQn);
			//			HAL_NVIC_DisableIRQ(EXTI3_IRQn);
			//			HAL_NVIC_DisableIRQ(EXTI4_IRQn);
			//			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
			update();
			update_lcd();
			if (FLAG_CHECK_SEND_DATA == 0)
				send_data_to_server();
			update();
			update_lcd();
			//			HAL_NVIC_EnableIRQ(EXTI2_IRQn);
			//			HAL_NVIC_EnableIRQ(EXTI3_IRQn);
			//			HAL_NVIC_EnableIRQ(EXTI4_IRQn);
			//			HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		}
		//		HAL_Delay(5000);
		//	  if (du_time_to_send_data) {
		//		  du_time_to_send_data=0;
		//		  send_data_to_server();
		//	  }
		//	  send_data_to_server();
		//	  HAL_Delay(7000);
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 63999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 30000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 63999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB12 PB13
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
uint8_t save_data_test[20];
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
////	if(huart->Instance == huart1.Instance)
////	{
////		if( *dataRX == 'F'){
////			HAL_UART_Transmit(huart, dataTX1, strlen(dataTX1), 10);
////		}
////		HAL_UART_Transmit(huart, dataTX, strlen(dataTX), 10);
////		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
////		HAL_UART_Receive_IT(huart, dataRX, sizeof(dataRX));
////	}
//	if(huart->Instance == huart1.Instance)
//	{
//		if(flag_check_OK)
//		{
//			save_data_chuoi_OK[0] = save_data_chuoi_OK[1];
//			save_data_chuoi_OK[1]=*dataRX;
//			HAL_UART_Receive_IT(huart, dataRX, 1);
////			for(int i=0; i < 19; i++){
////				save_data_test[i] = save_data_test[i+1];
////			}
////			save_data_test[19] = *dataRX;
//			if(*dataRX == 'K') {
//				if(strcmp((const char*)save_data_chuoi_OK,(const char*)so_sanh_chuoi_OK)==0){
//					flag_pass=1;
//				}
//			}
////			HAL_UART_Transmit(huart, dataTX, strlen(dataTX), 10);
////			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		}
//		if(flag_check_CMQTT)
//		{
//			for(int i=0; i < 4; i++){
//				save_data_chuoi_CMQTT[i] = save_data_chuoi_CMQTT[i+1];
//			}
//			save_data_chuoi_CMQTT[4]=*dataRX;
//			if(strcmp((const char*)save_data_chuoi_CMQTT,(const char*)so_sanh_chuoi_CMQTT)==0){
//				HAL_UART_Transmit(huart, dataTX1, strlen((const char*)dataTX1), 10);
//			}
////			HAL_UART_Transmit(huart, dataTX, strlen(dataTX), 10);
////			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//			HAL_UART_Receive_IT(huart, dataRX, sizeof(dataRX));
//		}
//		if(flag_check_PB_DONE)
//		{
//			for(int i=0; i < 6; i++){
//				save_data_chuoi_PB_DONE[i] = save_data_chuoi_PB_DONE[i+1];
//			}
//			save_data_chuoi_PB_DONE[6]=*dataRX;
//			if(strcmp((const char*)save_data_chuoi_PB_DONE,(const char*)so_sanh_chuoi_PB_DONE)==0){
//				HAL_UART_Transmit(huart, dataTX1, strlen((const char*)dataTX1), 10);
//			}
//			HAL_UART_Receive_IT(huart, dataRX, sizeof(dataRX));
//		}
//		if(flag_check_RX_PAYLOAD)
//		{
//			if(flag_pass_RX_PAYLOAD==0)
//			{
//				for(int i=0; i < 14; i++){
//					save_data_chuoi_RX_PAYLOAD[i] = save_data_chuoi_RX_PAYLOAD[i+1];
//				}
//				save_data_chuoi_RX_PAYLOAD[14]=*dataRX;
//				if(strcmp((const char*)save_data_chuoi_RX_PAYLOAD,(const char*)so_sanh_chuoi_RX_PAYLOAD)==0){
//					flag_pass_RX_PAYLOAD = 1;
//					__HAL_TIM_SET_COUNTER(&htim1, 0);
//	//				HAL_UART_Transmit(huart, dataTX1, strlen(dataTX1), 10);
//				}
//			}
//			if(flag_pass_RX_PAYLOAD) {
//				if(flag_pass_RX_PAYLOAD_next)
//				{
//					message[count_data_pass_RX_PAYLOAD] = *dataRX;
//					count_data_pass_RX_PAYLOAD++;
//					if(*dataRX == '\n'){
//						count_data_pass_RX_PAYLOAD=0;
//						flag_phan_biet_xuong_dong_data = 1;
//						flag_pass_RX_PAYLOAD_next =0;
//						flag_pass_all_RX_PAYLOAD =1;
//						flag_pass_RX_PAYLOAD =0;
//						flag_ngat_uart_xong_message=1;
//						sscanf((const char*)message, "%d %d %d %d %d %d %d", &led, &quat, &bom,&mode,&nguong_adc_quang_tro,&nguong_adc_do_am_dat,&nguong_nhiet_do);
//						for(int i=0 ; i < sizeof(message); i++ )
//						{
//							message[i] = 0;
//						}
//						if(flag_di_qua_nhap_mat_khau==1)
//						{
//							flag_ngat_keypad=1;
//						}
//						__HAL_TIM_SET_COUNTER(&htim1, 0);
//						__HAL_TIM_SET_COUNTER(&htim3, 0);
//						HAL_TIM_Base_Start_IT(&htim3);
//						flag_ngat_uart=1;
//
//
//					}
//
//				}
//				save_data_chuoi_RX_PAYLOAD_DATA[0] = *dataRX;
//				if(strcmp((const char*)save_data_chuoi_RX_PAYLOAD_DATA,(const char*)so_sanh_chuoi_xuong_dong)==0)
//				{
//					if(!flag_phan_biet_xuong_dong_data) {
//						flag_pass_RX_PAYLOAD_next =1;
//					}
//
//				}
//			}
//			flag_phan_biet_xuong_dong_data = 0;
//			HAL_UART_Receive_IT(huart, dataRX, 1);
//		}
//
//	}
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1) {
		//		send_data_to_server();
		flag_send_data = 1;
		du_time_to_send_data = 1;
		if (flag_di_qua_nhap_mat_khau == 1) {
			count_timer1++;
			if (count_timer1 > 150) {
				flag_di_qua_nhap_mat_khau = 0;
				count_state_lcd = 1;
				count_timer1 = 0;
				flag_ngat_keypad = 1;
				flag_chuyen_lcd = 1;
			}
		}
	}

	if (htim->Instance == TIM2) {
		count++;
	}
	if (htim->Instance == TIM3) {
		if (flag_ngat_uart == 1) {
			flag_ngat_uart = 0;
//			flag_send_data = 1;
			HAL_TIM_Base_Stop_IT(&htim3);
		}
		if (flag_ngat_nut_nhan == 1) {
			flag_ngat_nut_nhan = 0;
			switch (flag_switch_timer3) {
				case 1: {
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1) {
						num1 = num1_state1;
						num2 = num2_state1;
						num3 = num3_state1;
						led = led_state1;
						flag_number = flag_number_state1;
						flag_ngat_keypad = 1;
						state_ngat2 = 0;
						state_ngat1 = 1;
						flag_start_button = 1;
						GPIO_InitTypeDef GPIO_InitStruct = {0};
						GPIO_InitStruct.Pin = GPIO_PIN_2;
						GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
						GPIO_InitStruct.Pull = GPIO_PULLUP;
						HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					} else {
						state_ngat2 = 0;
						state_ngat1 = 1;
						GPIO_InitTypeDef GPIO_InitStruct = {0};
						GPIO_InitStruct.Pin = GPIO_PIN_2;
						GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
						GPIO_InitStruct.Pull = GPIO_PULLUP;
						HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					}
					break;
				}
				case 2: {
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1) {
						num4 = num4_state1;
						num5 = num5_state1;
						num6 = num6_state1;
						quat = quat_state1;
						flag_number = flag_number_state1;
						flag_ngat_keypad = 1;
						state_ngat2 = 0;
						state_ngat1 = 1;
						GPIO_InitTypeDef GPIO_InitStruct = {0};
						GPIO_InitStruct.Pin = GPIO_PIN_3;
						GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
						GPIO_InitStruct.Pull = GPIO_PULLUP;
						HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					} else {
						state_ngat2 = 0;
						state_ngat1 = 1;
						GPIO_InitTypeDef GPIO_InitStruct = {0};
						GPIO_InitStruct.Pin = GPIO_PIN_3;
						GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
						GPIO_InitStruct.Pull = GPIO_PULLUP;
						HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					}
					break;
				}
				case 3: {
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 1) {
						num7 = num7_state1;
						num8 = num8_state1;
						num9 = num9_state1;
						bom = bom_state1;
						flag_number = flag_number_state1;
						flag_ngat_keypad = 1;
						state_ngat2 = 0;
						state_ngat1 = 1;
						GPIO_InitTypeDef GPIO_InitStruct = {0};
						GPIO_InitStruct.Pin = GPIO_PIN_4;
						GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
						GPIO_InitStruct.Pull = GPIO_PULLUP;
						HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					} else {
						state_ngat2 = 0;
						state_ngat1 = 1;
						GPIO_InitTypeDef GPIO_InitStruct = {0};
						GPIO_InitStruct.Pin = GPIO_PIN_4;
						GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
						GPIO_InitStruct.Pull = GPIO_PULLUP;
						HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					}
					break;
				}
				case 4: {
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1) {
						if (num_sao != num_sao_state1) {
							count_state_lcd++;
							flag_chuyen_lcd = 1;
						}
						num_sao = num_sao_state1;
						num0 = num0_state1;
						num_thang = num_thang_state1;
						//				mode = mode_state1;
						flag_number = flag_number_state1;
						flag_ngat_keypad = 1;
						state_ngat2 = 0;
						state_ngat1 = 1;
						GPIO_InitTypeDef GPIO_InitStruct = {0};
						GPIO_InitStruct.Pin = GPIO_PIN_5;
						GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
						GPIO_InitStruct.Pull = GPIO_PULLUP;
						HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					} else {
						state_ngat2 = 0;
						state_ngat1 = 1;
						GPIO_InitTypeDef GPIO_InitStruct = {0};
						GPIO_InitStruct.Pin = GPIO_PIN_5;
						GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
						GPIO_InitStruct.Pull = GPIO_PULLUP;
						HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
					}
					break;
				}
				case 5: {
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 1) {
						if (flag_di_qua_nhap_mat_khau == 1) {
							mode = 1;
						}
					}
					break;
				}
				case 6: {
					if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11)) {
						if (flag_di_qua_nhap_mat_khau == 1) {
							mode = 0;
						}
					}
					break;
				}
			}
			HAL_TIM_Base_Stop_IT(&htim3);
		}
	}
	if (htim->Instance == TIM4) {
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		for (int i = 0; i < sizeof(data_RX); i++) {
			if (data_RX[i] != 0)
				break;
			if (i >= (sizeof(data_RX) - 1));
//				return;
		}
		HAL_UART_DMAStop(&huart1);
		HAL_UART_Receive_DMA(&huart1, data_RX, sizeof(data_RX));
		count_flag_gan = 0;
		for (int i = 0; i < sizeof(data_RX); i++) {
			if ((data_RX[i] == 13) && (data_RX[i + 1] == 10)) {
				count_flag_gan++;
			}

			if ((data_RX[i] == 13) || (data_RX[i] == 10)) {
				continue;
			}

			switch (count_flag_gan) {
				case 0:
					data_RX_0[cout_0] = data_RX[i];
					cout_0++;
					break;
				case 1:
					data_RX_1[cout_1] = data_RX[i];
					cout_1++;
					break;
				case 2:
					data_RX_2[cout_2] = data_RX[i];
					cout_2++;
					break;
				case 3:
					data_RX_3[cout_3] = data_RX[i];
					cout_3++;
					break;
				case 4:
					data_RX_4[cout_4] = data_RX[i];
					cout_4++;
					break;
				case 5:
					data_RX_5[cout_5] = data_RX[i];
					cout_5++;
					break;
				case 6:
					data_RX_6[cout_6] = data_RX[i];
					cout_6++;
					break;
				case 7:
					data_RX_7[cout_7] = data_RX[i];
					cout_7++;
					break;
				case 8:
					data_RX_8[cout_8] = data_RX[i];
					cout_8++;
					break;
				case 9:
					data_RX_9[cout_9] = data_RX[i];
					cout_9++;
					break;
				case 10:
					data_RX_10[cout_10] = data_RX[i];
					cout_10++;
					break;
				case 11:
					data_RX_11[cout_11] = data_RX[i];
					cout_11++;
					break;
				default:
					data_RX_12[cout_12] = data_RX[i];
					cout_12++;
					break;
			}
		}
		///////////////////////////////
		if (data_RX_0[0] == '>') {
			flag_pass_input =1;
		}
		if (data_RX_1[0] == '>') {
			flag_pass_input =1;
		}
		if (data_RX_2[0] == '>') {
			flag_pass_input =1;
		}
		if (data_RX_3[0] == '>') {
			flag_pass_input =1;
		}
		if (data_RX_4[0] == '>') {
			flag_pass_input =1;
		}
		if (data_RX_5[0] == '>') {
			flag_pass_input =1;
		}
		if (data_RX_6[0] == '>') {
			flag_pass_input =1;
		}
		if (data_RX_7[0] == '>') {
			flag_pass_input =1;
		}
		if (data_RX_8[0] == '>') {
			flag_pass_input =1;
		}
		if (data_RX_9[0] == '>') {
			flag_pass_input =1;
		}
		if (data_RX_10[0] == '>') {
			flag_pass_input =1;
		}
		if (data_RX_11[0] == '>') {
			flag_pass_input =1;
		}
		if (data_RX_12[0] == '>') {
			flag_pass_input =1;
		}
		//////////////////////////////
		if (!strcmp((const char *)data_RX_0, (const char *)cmd_OK)) {
			flag_pass = 1;
			flag_nhap_topic = 1;
		}
		if (!strcmp((const char *)data_RX_1, (const char *)cmd_OK)) {
			flag_pass = 1;
			flag_nhap_topic = 1;
		}
		if (!strcmp((const char *)data_RX_2, (const char *)cmd_OK)) {
			flag_pass = 1;
			flag_nhap_topic = 1;
		}
		if (!strcmp((const char *)data_RX_3, (const char *)cmd_OK)) {
			flag_pass = 1;
			flag_nhap_topic = 1;
		}
		if (!strcmp((const char *)data_RX_4, (const char *)cmd_OK)) {
			flag_pass = 1;
			flag_nhap_topic = 1;
		}
		if (!strcmp((const char *)data_RX_5, (const char *)cmd_OK)) {
			flag_pass = 1;
			flag_nhap_topic = 1;
		}
		if (!strcmp((const char *)data_RX_6, (const char *)cmd_OK)) {
			flag_pass = 1;
			flag_nhap_topic = 1;
		}
		if (!strcmp((const char *)data_RX_7, (const char *)cmd_OK)) {
			flag_pass = 1;
			flag_nhap_topic = 1;
		}
		if (!strcmp((const char *)data_RX_8, (const char *)cmd_OK)) {
			flag_pass = 1;
			flag_nhap_topic = 1;
		}
		if (!strcmp((const char *)data_RX_9, (const char *)cmd_OK)) {
			flag_pass = 1;
			flag_nhap_topic = 1;
		}
		if (!strcmp((const char *)data_RX_10, (const char *)cmd_OK)) {
			flag_pass = 1;
			flag_nhap_topic = 1;
		}
		if (!strcmp((const char *)data_RX_11, (const char *)cmd_OK)) {
			flag_pass = 1;
			flag_nhap_topic = 1;
		}
		if (!strcmp((const char *)data_RX_12, (const char *)cmd_OK)) {
			flag_pass = 1;
			flag_nhap_topic = 1;
		}
		int err = compare_n_char(data_RX_0, sizeof(data_RX_0), cmd_MQTT, sizeof(cmd_MQTT), match_buf);
		if (!err) {
			sscanf((const char *)data_RX_1, "%d %d %d %d %d %d %d", &led, &quat, &bom, &mode, &nguong_adc_quang_tro, &nguong_adc_do_am_dat, &nguong_nhiet_do);
			if(flag_k_cho_phep_nhan == 0)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 29999);
				data_avail_mqtt = 1;
			}
			count_test_re++;
		}
		err = compare_n_char(data_RX_1, sizeof(data_RX_1), cmd_MQTT, sizeof(cmd_MQTT), match_buf);
		if (!err) {
			sscanf((const char *)data_RX_2, "%d %d %d %d %d %d %d", &led, &quat, &bom, &mode, &nguong_adc_quang_tro, &nguong_adc_do_am_dat, &nguong_nhiet_do);
			if(flag_k_cho_phep_nhan == 0)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 29999);
				data_avail_mqtt = 1;
			}
			count_test_re++;
		}
		err = compare_n_char(data_RX_2, sizeof(data_RX_2), cmd_MQTT, sizeof(cmd_MQTT), match_buf);
		if (!err) {
			sscanf((const char *)data_RX_3, "%d %d %d %d %d %d %d", &led, &quat, &bom, &mode, &nguong_adc_quang_tro, &nguong_adc_do_am_dat, &nguong_nhiet_do);
			if(flag_k_cho_phep_nhan == 0)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 29999);
				data_avail_mqtt = 1;
			}
			count_test_re++;
		}
		err = compare_n_char(data_RX_3, sizeof(data_RX_3), cmd_MQTT, sizeof(cmd_MQTT), match_buf);
		if (!err) {
			sscanf((const char *)data_RX_4, "%d %d %d %d %d %d %d", &led, &quat, &bom, &mode, &nguong_adc_quang_tro, &nguong_adc_do_am_dat, &nguong_nhiet_do);
			if(flag_k_cho_phep_nhan == 0)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 29999);
				data_avail_mqtt = 1;
			}
			count_test_re++;
		}
		err = compare_n_char(data_RX_4, sizeof(data_RX_4), cmd_MQTT, sizeof(cmd_MQTT), match_buf);
		if (!err) {
			sscanf((const char *)data_RX_5, "%d %d %d %d %d %d %d", &led, &quat, &bom, &mode, &nguong_adc_quang_tro, &nguong_adc_do_am_dat, &nguong_nhiet_do);
			if(flag_k_cho_phep_nhan == 0)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 29999);
				data_avail_mqtt = 1;
			}
			count_test_re++;
		}
		err = compare_n_char(data_RX_5, sizeof(data_RX_5), cmd_MQTT, sizeof(cmd_MQTT), match_buf);
		if (!err) {
			sscanf((const char *)data_RX_6, "%d %d %d %d %d %d %d", &led, &quat, &bom, &mode, &nguong_adc_quang_tro, &nguong_adc_do_am_dat, &nguong_nhiet_do);
			if(flag_k_cho_phep_nhan == 0)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 29999);
				data_avail_mqtt = 1;
			}
			count_test_re++;
		}

		err = compare_n_char(data_RX_6, sizeof(data_RX_6), cmd_MQTT, sizeof(cmd_MQTT), match_buf);
		if (!err) {
			sscanf((const char *)data_RX_7, "%d %d %d %d %d %d %d", &led, &quat, &bom, &mode, &nguong_adc_quang_tro, &nguong_adc_do_am_dat, &nguong_nhiet_do);
			if(flag_k_cho_phep_nhan == 0)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 29999);
				data_avail_mqtt = 1;
			}
			count_test_re++;
		}

		err = compare_n_char(data_RX_7, sizeof(data_RX_7), cmd_MQTT, sizeof(cmd_MQTT), match_buf);
		if (!err) {
			sscanf((const char *)data_RX_8, "%d %d %d %d %d %d %d", &led, &quat, &bom, &mode, &nguong_adc_quang_tro, &nguong_adc_do_am_dat, &nguong_nhiet_do);
			if(flag_k_cho_phep_nhan == 0)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 29999);
				data_avail_mqtt = 1;
			}
			count_test_re++;
		}

		err = compare_n_char(data_RX_8, sizeof(data_RX_8), cmd_MQTT, sizeof(cmd_MQTT), match_buf);
		if (!err) {
			sscanf((const char *)data_RX_9, "%d %d %d %d %d %d %d", &led, &quat, &bom, &mode, &nguong_adc_quang_tro, &nguong_adc_do_am_dat, &nguong_nhiet_do);
			if(flag_k_cho_phep_nhan == 0)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 29999);
				data_avail_mqtt = 1;
			}
			count_test_re++;
		}

		err = compare_n_char(data_RX_9, sizeof(data_RX_9), cmd_MQTT, sizeof(cmd_MQTT), match_buf);
		if (!err) {
			sscanf((const char *)data_RX_10, "%d %d %d %d %d %d %d", &led, &quat, &bom, &mode, &nguong_adc_quang_tro, &nguong_adc_do_am_dat, &nguong_nhiet_do);
			if(flag_k_cho_phep_nhan == 0)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 29999);
				data_avail_mqtt = 1;
			}
			count_test_re++;
		}

		err = compare_n_char(data_RX_9, sizeof(data_RX_9), cmd_MQTT, sizeof(cmd_MQTT), match_buf);
		if (!err) {
			sscanf((const char *)data_RX_10, "%d %d %d %d %d %d %d", &led, &quat, &bom, &mode, &nguong_adc_quang_tro, &nguong_adc_do_am_dat, &nguong_nhiet_do);
			if(flag_k_cho_phep_nhan == 0)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 29999);
				data_avail_mqtt = 1;
			}
			count_test_re++;
		}

		err = compare_n_char(data_RX_10, sizeof(data_RX_10), cmd_MQTT, sizeof(cmd_MQTT), match_buf);
		if (!err) {
			sscanf((const char *)data_RX_11, "%d %d %d %d %d %d %d", &led, &quat, &bom, &mode, &nguong_adc_quang_tro, &nguong_adc_do_am_dat, &nguong_nhiet_do);
			if(flag_k_cho_phep_nhan == 0)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 29999);
				data_avail_mqtt = 1;
			}
			count_test_re++;
		}
		err = compare_n_char(data_RX_11, sizeof(data_RX_11), cmd_MQTT, sizeof(cmd_MQTT), match_buf);
		if (!err) {
			sscanf((const char *)data_RX_12, "%d %d %d %d %d %d %d", &led, &quat, &bom, &mode, &nguong_adc_quang_tro, &nguong_adc_do_am_dat, &nguong_nhiet_do);
			if(flag_k_cho_phep_nhan == 0)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 29999);
				data_avail_mqtt = 1;
			}
			count_test_re++;
		}
		err = compare_n_char(data_RX_12, sizeof(data_RX_12), cmd_MQTT, sizeof(cmd_MQTT), match_buf);
		if (!err) {
			sscanf((const char *)data_RX_13, "%d %d %d %d %d %d %d", &led, &quat, &bom, &mode, &nguong_adc_quang_tro, &nguong_adc_do_am_dat, &nguong_nhiet_do);
			if(flag_k_cho_phep_nhan == 0)
			{
				__HAL_TIM_SET_COUNTER(&htim1, 29999);
				data_avail_mqtt = 1;
			}
			count_test_re++;
		}
		cout_0 = 0;
		cout_1 = 0;
		cout_2 = 0;
		cout_3 = 0;
		cout_4 = 0;
		cout_5 = 0;
		cout_6 = 0;
		cout_7 = 0;
		cout_8 = 0;
		cout_9 = 0;
		cout_10 = 0;
		cout_11 = 0;
		cout_12 = 0;
		flag_cho_phep_gan = 0;
		count_flag_gan = 0;

		//////////
		for (int i = 0; i < sizeof(data_RX); i++) {
			data_RX[i] = 0;
		}
		for (int i = 0; i < sizeof(data_RX_0); i++) {
			data_RX_0[i] = 0;
		}
		for (int i = 0; i < sizeof(data_RX_1); i++) {
			data_RX_1[i] = 0;
		}
		for (int i = 0; i < sizeof(data_RX_2); i++) {
			data_RX_2[i] = 0;
		}
		for (int i = 0; i < sizeof(data_RX_3); i++) {
			data_RX_3[i] = 0;
		}
		for (int i = 0; i < sizeof(data_RX_4); i++) {
			data_RX_4[i] = 0;
		}
		for (int i = 0; i < sizeof(data_RX_5); i++) {
			data_RX_5[i] = 0;
		}
		for (int i = 0; i < sizeof(data_RX_6); i++) {
			data_RX_6[i] = 0;
		}
		for (int i = 0; i < sizeof(data_RX_7); i++) {
			data_RX_7[i] = 0;
		}
		for (int i = 0; i < sizeof(data_RX_8); i++) {
			data_RX_8[i] = 0;
		}
		for (int i = 0; i < sizeof(data_RX_9); i++) {
			data_RX_9[i] = 0;
		}
		for (int i = 0; i < sizeof(data_RX_10); i++) {
			data_RX_10[i] = 0;
		}
		for (int i = 0; i < sizeof(data_RX_11); i++) {
			data_RX_11[i] = 0;
		}
		for (int i = 0; i < sizeof(data_RX_12); i++) {
			data_RX_12[i] = 0;
		}
	}
}
int count_so_lan_nhan = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_2) {
		if (state_ngat1 == 1) {
			currentTime = HAL_GetTick();  // Lấy th�?i gian hiện tại
			if ((currentTime - lastDebounceTime) > debounceDelay || (flag_start_button == 1)) {
				// Cập nhật trạng thái nút nhấn chỉ khi đã qua th�?i gian debounce
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0) {
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
					if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0) && (flag_disable_nut_nhan == 0)) {
						GPIO_InitTypeDef GPIO_InitStruct = {0};
						GPIO_InitStruct.Pin = GPIO_PIN_2;
						GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
						GPIO_InitStruct.Pull = GPIO_PULLUP;
						HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
						num1_state1 = num1;
						flag_number_state1 = flag_number;
						num1_state1++;
						flag_number_state1 = 1;
						state_ngat2 = 1;
						state_ngat1 = 0;
						//					num1++;
						//					flag_number=1;
						//					flag_ngat_keypad=1;

					} else {
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
						if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0) && (flag_disable_nut_nhan == 0)) {
							GPIO_InitTypeDef GPIO_InitStruct = {0};
							GPIO_InitStruct.Pin = GPIO_PIN_2;
							GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
							GPIO_InitStruct.Pull = GPIO_PULLUP;
							HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
							num2_state1 = num2;
							flag_number_state1 = flag_number;
							num2_state1++;
							flag_number_state1 = 2;
							state_ngat2 = 1;
							state_ngat1 = 0;
						} else {
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
							if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0) && (flag_disable_nut_nhan == 0)) {
								GPIO_InitTypeDef GPIO_InitStruct = {0};
								GPIO_InitStruct.Pin = GPIO_PIN_2;
								GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
								GPIO_InitStruct.Pull = GPIO_PULLUP;
								HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
								num3_state1 = num3;
								flag_number_state1 = flag_number;
								num3_state1++;
								flag_number_state1 = 3;
								state_ngat2 = 1;
								state_ngat1 = 0;
							} else {
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
								if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 0) && (flag_disable_nut_nhan == 0)) {
									if (flag_di_qua_nhap_mat_khau) {
										GPIO_InitTypeDef GPIO_InitStruct = {0};
										GPIO_InitStruct.Pin = GPIO_PIN_2;
										GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
										GPIO_InitStruct.Pull = GPIO_PULLUP;
										HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
										led_state1 = led;
										flag_number_state1 = flag_number;
										led_state1++;
										flag_number_state1 = 10;
										state_ngat2 = 1;
										state_ngat1 = 0;
									}
								}
							}
						}
					}

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
				}

				lastDebounceTime = currentTime;  // Cập nhật th�?i gian debounce cuối cùng
			}
		}

		if (state_ngat2 == 1) {
			//				currentTime = HAL_GetTick(); // Lấy th�?i gian hiện tại
			//				if ((currentTime - lastDebounceTime) > debounceDelay_mode)
			//				{
			//					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == 1)
			//					{
			//						num1=num1_state1;
			//						num2=num2_state1;
			//						num3=num3_state1;
			//						led=led_state1;
			//						flag_number=flag_number_state1;
			//						flag_ngat_keypad=1;
			//						state_ngat2=0;
			//						state_ngat1=1;
			//						GPIO_InitTypeDef GPIO_InitStruct = {0};
			//						GPIO_InitStruct.Pin = GPIO_PIN_2;
			//						GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
			//						GPIO_InitStruct.Pull = GPIO_PULLUP;
			//						HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			//					}
			//				lastDebounceTime = currentTime; // Cập nhật th�?i gian debounce cuối cùng
			//				}
			//			}
			__HAL_TIM_SET_COUNTER(&htim3, 0);
			HAL_TIM_Base_Start_IT(&htim3);
			flag_ngat_nut_nhan = 1;
			flag_switch_timer3 = 1;
		}
	}

	if (GPIO_Pin == GPIO_PIN_3) {
		if (state_ngat1 == 1) {
			currentTime = HAL_GetTick();  // Lấy th�?i gian hiện tại
			if ((currentTime - lastDebounceTime) > debounceDelay || (flag_start_button == 1)) {
				// Cập nhật trạng thái nút nhấn chỉ khi đã qua th�?i gian debounce
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0) {
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
					if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0) && (flag_disable_nut_nhan == 0)) {
						GPIO_InitTypeDef GPIO_InitStruct = {0};
						GPIO_InitStruct.Pin = GPIO_PIN_3;
						GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
						GPIO_InitStruct.Pull = GPIO_PULLUP;
						HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
						num4_state1 = num4;
						flag_number_state1 = flag_number;
						num4_state1++;
						flag_number_state1 = 4;
						state_ngat2 = 1;
						state_ngat1 = 0;
					} else {
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
						if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0) && (flag_disable_nut_nhan == 0)) {
							GPIO_InitTypeDef GPIO_InitStruct = {0};
							GPIO_InitStruct.Pin = GPIO_PIN_3;
							GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
							GPIO_InitStruct.Pull = GPIO_PULLUP;
							HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
							num5_state1 = num5;
							flag_number_state1 = flag_number;
							num5_state1++;
							flag_number_state1 = 5;
							state_ngat2 = 1;
							state_ngat1 = 0;
						} else {
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
							if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0) && (flag_disable_nut_nhan == 0)) {
								GPIO_InitTypeDef GPIO_InitStruct = {0};
								GPIO_InitStruct.Pin = GPIO_PIN_3;
								GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
								GPIO_InitStruct.Pull = GPIO_PULLUP;
								HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
								num6_state1 = num6;
								flag_number_state1 = flag_number;
								num6_state1++;
								flag_number_state1 = 6;
								state_ngat2 = 1;
								state_ngat1 = 0;
							} else {
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
								if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 0) && (flag_disable_nut_nhan == 0)) {
									if (flag_di_qua_nhap_mat_khau) {
										GPIO_InitTypeDef GPIO_InitStruct = {0};
										GPIO_InitStruct.Pin = GPIO_PIN_3;
										GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
										GPIO_InitStruct.Pull = GPIO_PULLUP;
										HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
										quat_state1 = quat;
										flag_number_state1 = flag_number;
										quat_state1++;
										flag_number_state1 = 10;
										state_ngat2 = 1;
										state_ngat1 = 0;
									}
								}
							}
						}
					}

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
				}

				lastDebounceTime = currentTime;  // Cập nhật th�?i gian debounce cuối cùng
			}
		}
		if (state_ngat2 == 1) {
			//			currentTime = HAL_GetTick(); // Lấy th�?i gian hiện tại
			//			if ((currentTime - lastDebounceTime) > debounceDelay_mode)
			//			{
			//				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == 1)
			//				{
			//					num4=num4_state1;
			//					num5=num5_state1;
			//					num6=num6_state1;
			//					quat = quat_state1;
			//					flag_number=flag_number_state1;
			//					flag_ngat_keypad=1;
			//					state_ngat2=0;
			//					state_ngat1=1;
			//					GPIO_InitTypeDef GPIO_InitStruct = {0};
			//					GPIO_InitStruct.Pin = GPIO_PIN_3;
			//					GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
			//					GPIO_InitStruct.Pull = GPIO_PULLUP;
			//					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			//				}
			//			lastDebounceTime = currentTime; // Cập nhật th�?i gian debounce cuối cùng
			//			}
			__HAL_TIM_SET_COUNTER(&htim3, 0);
			HAL_TIM_Base_Start_IT(&htim3);
			flag_ngat_nut_nhan = 1;
			flag_switch_timer3 = 2;
		}
	}

	if (GPIO_Pin == GPIO_PIN_4) {
		if (state_ngat1 == 1) {
			currentTime = HAL_GetTick();  // Lấy th�?i gian hiện tại
			if ((currentTime - lastDebounceTime) > debounceDelay || (flag_start_button == 1)) {
				// Cập nhật trạng thái nút nhấn chỉ khi đã qua th�?i gian debounce
				if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0) && (flag_disable_nut_nhan == 0)) {
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
					if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0) && (flag_disable_nut_nhan == 0)) {
						GPIO_InitTypeDef GPIO_InitStruct = {0};
						GPIO_InitStruct.Pin = GPIO_PIN_4;
						GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
						GPIO_InitStruct.Pull = GPIO_PULLUP;
						HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
						num7_state1 = num7;
						flag_number_state1 = flag_number;
						num7_state1++;
						flag_number_state1 = 7;
						state_ngat2 = 1;
						state_ngat1 = 0;
					} else {
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
						if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0) && (flag_disable_nut_nhan == 0)) {
							GPIO_InitTypeDef GPIO_InitStruct = {0};
							GPIO_InitStruct.Pin = GPIO_PIN_4;
							GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
							GPIO_InitStruct.Pull = GPIO_PULLUP;
							HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
							num8_state1 = num8;
							flag_number_state1 = flag_number;
							num8_state1++;
							flag_number_state1 = 8;
							state_ngat2 = 1;
							state_ngat1 = 0;
						} else {
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
							if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0) && (flag_disable_nut_nhan == 0)) {
								GPIO_InitTypeDef GPIO_InitStruct = {0};
								GPIO_InitStruct.Pin = GPIO_PIN_4;
								GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
								GPIO_InitStruct.Pull = GPIO_PULLUP;
								HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
								num9_state1 = num9;
								flag_number_state1 = flag_number;
								num9_state1++;
								flag_number_state1 = 9;
								state_ngat2 = 1;
								state_ngat1 = 0;
							} else {
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
								if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 0) && (flag_disable_nut_nhan == 0)) {
									if (flag_di_qua_nhap_mat_khau) {
										GPIO_InitTypeDef GPIO_InitStruct = {0};
										GPIO_InitStruct.Pin = GPIO_PIN_4;
										GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
										GPIO_InitStruct.Pull = GPIO_PULLUP;
										HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
										bom_state1 = bom;
										flag_number_state1 = flag_number;
										bom_state1++;
										flag_number_state1 = 10;
										state_ngat2 = 1;
										state_ngat1 = 0;
									}
								}
							}
						}
					}

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
				}

				lastDebounceTime = currentTime;  // Cập nhật th�?i gian debounce cuối cùng
			}
		}
		if (state_ngat2 == 1) {
			//			currentTime = HAL_GetTick(); // Lấy th�?i gian hiện tại
			//			if ((currentTime - lastDebounceTime) > debounceDelay_mode)
			//			{
			//				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == 1)
			//				{
			//					num7=num7_state1;
			//					num8=num8_state1;
			//					num9=num9_state1;
			//					bom = bom_state1;
			//					flag_number=flag_number_state1;
			//					flag_ngat_keypad=1;
			//					state_ngat2=0;
			//					state_ngat1=1;
			//					GPIO_InitTypeDef GPIO_InitStruct = {0};
			//					GPIO_InitStruct.Pin = GPIO_PIN_4;
			//					GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
			//					GPIO_InitStruct.Pull = GPIO_PULLUP;
			//					HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			//				}
			//			lastDebounceTime = currentTime; // Cập nhật th�?i gian debounce cuối cùng
			//			}
			__HAL_TIM_SET_COUNTER(&htim3, 0);
			HAL_TIM_Base_Start_IT(&htim3);
			flag_ngat_nut_nhan = 1;
			flag_switch_timer3 = 3;
		}
	}

	if (GPIO_Pin == GPIO_PIN_5) {
		if (state_ngat1 == 1) {
			currentTime = HAL_GetTick();  // Lấy th�?i gian hiện tại
			if ((currentTime - lastDebounceTime) > debounceDelay || (flag_start_button == 1)) {
				// Cập nhật trạng thái nút nhấn chỉ khi đã qua th�?i gian debounce
				if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0) {
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0) {
						if (flag_di_qua_nhap_mat_khau) {
							//							num_sao++;
							//							count_state_lcd ++;
							//							flag_chuyen_lcd=1;
							//							flag_number=10;
							//							flag_ngat_keypad=1;

							GPIO_InitTypeDef GPIO_InitStruct = {0};
							GPIO_InitStruct.Pin = GPIO_PIN_5;
							GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
							GPIO_InitStruct.Pull = GPIO_PULLUP;
							HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
							num_sao_state1 = num_sao;
							flag_number_state1 = flag_number;
							num_sao_state1++;
							flag_number_state1 = 10;
							state_ngat2 = 1;
							state_ngat1 = 0;
						}
					} else {
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
						if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0) && (flag_disable_nut_nhan == 0)) {
							GPIO_InitTypeDef GPIO_InitStruct = {0};
							GPIO_InitStruct.Pin = GPIO_PIN_5;
							GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
							GPIO_InitStruct.Pull = GPIO_PULLUP;
							HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
							num0_state1 = num0;
							flag_number_state1 = flag_number;
							num0_state1++;
							flag_number_state1 = 0;
							state_ngat2 = 1;
							state_ngat1 = 0;
						} else {
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 1);
							if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0) && (flag_disable_nut_nhan == 0)) {
								GPIO_InitTypeDef GPIO_InitStruct = {0};
								GPIO_InitStruct.Pin = GPIO_PIN_5;
								GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
								GPIO_InitStruct.Pull = GPIO_PULLUP;
								HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
								num_thang_state1 = num_thang;
								flag_number_state1 = flag_number;
								num_thang_state1++;
								flag_number_state1 = 11;
								state_ngat2 = 1;
								state_ngat1 = 0;
							} else {
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
								HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
								if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 0) {
									if (flag_di_qua_nhap_mat_khau) {
										//										GPIO_InitTypeDef GPIO_InitStruct = {0};
										//										GPIO_InitStruct.Pin = GPIO_PIN_5;
										//										GPIO_InitStruct.Mode =
										//GPIO_MODE_IT_RISING; 										GPIO_InitStruct.Pull = GPIO_PULLUP; 										HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); 										mode_state1=mode;
										//										flag_number_state1=flag_number;
										//										mode_state1++;
										//										flag_number_state1=10;
										//										state_ngat2 =1;
										//										state_ngat1=0;
									}
								}
							}
						}
					}

					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, 0);
				}

				lastDebounceTime = currentTime;  // Cập nhật th�?i gian debounce cuối cùng
			}
		}
		if (state_ngat2 == 1) {
			//		currentTime = HAL_GetTick(); // Lấy th�?i gian hiện tại
			//		if ((currentTime - lastDebounceTime) > debounceDelay_mode)
			//		{
			//			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == 1)
			//			{
			//				if(num_sao!=num_sao_state1) {
			//					count_state_lcd ++;
			//					flag_chuyen_lcd=1;
			//				}
			//				num_sao=num_sao_state1;
			//				num0=num0_state1;
			//				num_thang=num_thang_state1;
			////				mode = mode_state1;
			//				flag_number=flag_number_state1;
			//				flag_ngat_keypad=1;
			//				state_ngat2=0;
			//				state_ngat1=1;
			//				GPIO_InitTypeDef GPIO_InitStruct = {0};
			//				GPIO_InitStruct.Pin = GPIO_PIN_5;
			//				GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
			//				GPIO_InitStruct.Pull = GPIO_PULLUP;
			//				HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			//			}
			//		lastDebounceTime = currentTime; // Cập nhật th�?i gian debounce cuối cùng
			//		}
			__HAL_TIM_SET_COUNTER(&htim3, 0);
			HAL_TIM_Base_Start_IT(&htim3);
			flag_ngat_nut_nhan = 1;
			flag_switch_timer3 = 4;
		}
	}
	if (GPIO_Pin == GPIO_PIN_1) {
		tick_hien_tai = __HAL_TIM_GET_COUNTER(&htim2);
		thoi_gian_tick = tick_hien_tai - tick_lan_truoc;
		tick_lan_truoc = tick_hien_tai;
		tick[count_tick] = thoi_gian_tick;
		count_tick++;
		if (count_tick > 84) {
			HAL_TIM_Base_Stop_IT(&htim2);
			HAL_NVIC_DisableIRQ(EXTI1_IRQn);
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pin = GPIO_PIN_1;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		}
	}
	if (GPIO_Pin == GPIO_PIN_8) {
		//		currentTime = HAL_GetTick(); // Lấy th�?i gian hiện tại
		//		if ((currentTime - lastDebounceTime) > debounceDelay_mode)
		//		{
		//			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8))
		//			{
		//				if(flag_di_qua_nhap_mat_khau==1)
		//				{
		//					mode =1;
		//				}
		//			}
		//		lastDebounceTime = currentTime;
		//		}
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		HAL_TIM_Base_Start_IT(&htim3);
		flag_ngat_nut_nhan = 1;
		flag_switch_timer3 = 5;
	}
	if (GPIO_Pin == GPIO_PIN_11) {
		//		currentTime = HAL_GetTick(); // Lấy th�?i gian hiện tại
		//		if ((currentTime - lastDebounceTime) > debounceDelay_mode)
		//		{
		//			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11))
		//			{
		//				if(flag_di_qua_nhap_mat_khau==1)
		//				{
		//					mode =0;
		//				}
		//			}
		//		lastDebounceTime = currentTime;
		//		}
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		HAL_TIM_Base_Start_IT(&htim3);
		flag_ngat_nut_nhan = 1;
		flag_switch_timer3 = 6;
	}
}

void xu_ly_tick_dht11(uint8_t *tick, uint8_t *data_dht11)
{
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	int count_data_dht11 = 0;
	for (int i = 4; i <= 82; i += 2) {
		data_dht11[count_data_dht11 / 8] <<= 1;
		if (tick[i] > tick[i + 1]) {
			data_dht11[count_data_dht11 / 8] |= 0;
		} else {
			data_dht11[count_data_dht11 / 8] |= 1;
		}
		count_data_dht11++;
	}
}

void doc_dht11(void)
{
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	count_tick = 0;
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_Delay(1);

	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, 0);

	HAL_Delay(20);
	if (count_tick == 0) {
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		tick_lan_truoc = 0;
	}
	GPIO_InitStruct.Pin = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
