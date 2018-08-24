/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 * This file contains the source code for a sample application to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf.h"
#include "boards.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_pwm.h"
#include "app_uart.h"
#include "app_util_platform.h"

#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "sdk_errors.h"

#include "nrf_drv_i2s.h"    //i2s

//BLE
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "ble_nus.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

uint8_t always_on_mode=0; uint8_t fast_blinky_mode=1; uint8_t normal_blinky_mode=2; uint8_t soft_blinky_mode=3; 
uint8_t circle_mode=4;  uint8_t flower_mode=5;  uint8_t slee_mode=6;
bool compare_rgbw;

//button
#include "app_button.h"
#include "nrf_gpiote.h"
#include "nrf_drv_gpiote.h"

//adc
#include "nrf_drv_ppi.h"
#include "nrf_drv_saadc.h"
#define SAMPLES_IN_BUFFER 1 //adc 5개 값을 평균냄  (sample[0]~sample[4]/5)=averaged adc output value
static const nrf_drv_timer_t adc_timer=NRF_DRV_TIMER_INSTANCE(0);
static nrf_saadc_value_t adc_buffer_pool[1];
static nrf_ppi_channel_t adc_ppi_channel;
static uint32_t m_adc_evt_counter;
volatile static uint16_t battery_level;
bool saadc_done=false;


//RGBW
#define LED_NUMBER 8
//한 개의 배열이 unsigned int 32bit 이고 이는 0x00000000이므로 0.31us*32=10ms가 된다
//때문에 reset에 필요한 50ms를 만들기 위해선 최소 5개의 리셋 배열이 필요하게 된다.
#define RESET_BITS 6
#define I2S_BUFFER_SIZE 4*LED_NUMBER+RESET_BITS

static uint32_t m_buffer_tx[I2S_BUFFER_SIZE];
static uint32_t m_buffer_rx[I2S_BUFFER_SIZE];
volatile bool g_i2s_start=true;
volatile bool g_i2s_running=false;

struct cRGBW	{uint8_t g;uint8_t r;uint8_t b;uint8_t w;};
struct cRGBW led[LED_NUMBER]={0};
static volatile uint8_t LED_Mode=0;

//Button
APP_TIMER_DEF(m_button_tick_timer);
static volatile uint32_t button_tick=0;
static volatile uint32_t button_tick_release=0;
static volatile uint32_t total_ticks=0;

//UART
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define MAX_TEST_DATA_BYTES (15U)
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 256

//PWM
APP_PWM_INSTANCE(PWM1, 1);  //name, timer id
//APP_PWM_INSTANCE(PWM2, 2);
static volatile bool pwm_ready_flag;

//TIMER
//const nrf_drv_timer_t timer_test=NRF_DRV_TIMER_INSTANCE(0); 
static volatile uint32_t LED_Stop_time=0;
APP_TIMER_DEF(led_hold_timer);
APP_TIMER_DEF(blank_blinky_timer);

//BLE
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */
#define DEVICE_NAME                     "nRF52832_LED_BOARD"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */
#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_DURATION                0                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. 0 to unlimited advertising */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}
};
/**
 * @brief Function for application main entry.
 */


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

 static void timers_init(void)
 {
     ret_code_t err_code=app_timer_init();
     APP_ERROR_CHECK(err_code);
 }
 
 /**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
 static void gap_params_init(void)
 {
     uint32_t err_code;
     ble_gap_conn_params_t gap_conn_params;
     ble_gap_conn_sec_mode_t sec_mode;
     
     BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
     //BLE_GAP_CONN_SEC_MODE_SET_ENC_WITH_MITM(&sec_mode);
     
     err_code=sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *)DEVICE_NAME,strlen(DEVICE_NAME));
     APP_ERROR_CHECK(err_code);
     
     memset(&gap_conn_params,0,sizeof(gap_conn_params));
     
     gap_conn_params.min_conn_interval=MIN_CONN_INTERVAL;
     gap_conn_params.max_conn_interval=MAX_CONN_INTERVAL;
     gap_conn_params.slave_latency=SLAVE_LATENCY;
     gap_conn_params.conn_sup_timeout=CONN_SUP_TIMEOUT;
     
     err_code=sd_ble_gap_ppcp_set(&gap_conn_params);
     APP_ERROR_CHECK(err_code);
 }
 
 /**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}
 
 
 void  mode_set(uint8_t *ble_data);
 /**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt) //used to receive data sent from the phone
{
    uint8_t rx_buff[255];
    
    if (p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        uint32_t err_code;
        NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
        NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);
        for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
        {
            do
            {
                err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
                rx_buff[i]=p_evt->params.rx_data.p_data[i];
                if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
                {
                    NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
                    APP_ERROR_CHECK(err_code);
                }
            } while (err_code == NRF_ERROR_BUSY);
        }
        if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
        {
            while (app_uart_put('\n') == NRF_ERROR_BUSY);
        }
    }printf("\r\n");
    for(int i=3; i<p_evt->params.rx_data.length;i++)   //Char to Int
        rx_buff[i]=(int)(rx_buff[i]-48);
    mode_set(rx_buff);

}

void set_color(struct cRGBW *color, uint8_t LEDindex);
void  mode_set(uint8_t *ble_rx_data)
 {
    uint8_t i=0;
     char mode[3]={};
     for(int i=0;i<3;i++)
     {
         mode[i]+=ble_rx_data[i];
     }i=3;

     if(strcmp(mode,"RGB")==0)
     {
         //printf("\r\nRGB START\r\n");
         led[0].r=ble_rx_data[i]*100+ble_rx_data[i+1]*10+ble_rx_data[i+2];
         printf("\r\nR : %d\r\n",led[0].r);
         led[0].g=ble_rx_data[i*2]*100+ble_rx_data[i*2+1]*10+ble_rx_data[i*2+2];
         printf("\r\nG : %d\r\n",led[0].g);
         led[0].b=ble_rx_data[i*3]*100+ble_rx_data[i*3+1]*10+ble_rx_data[i*3+2];
         printf("\r\nB : %d\r\n",led[0].b);
         led[0].w=ble_rx_data[i*4]*100+ble_rx_data[i*4+1]*10+ble_rx_data[i*4+2];
         printf("\r\nW : %d\r\n",led[0].w);
         
         if(LED_Mode==always_on_mode)
         {
             for(i=0;i<LED_NUMBER;i++)
             {
                 set_color(&led[0],i);
             }
         }
         if(LED_Mode==circle_mode)
            compare_rgbw=false;
     }
 }
 
 /**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}
 
 /**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    //memset은 특정 메모리블럭에서 원하는 크기만큼을 특정 문자(1개)로 셋팅한다.(보통 초기화의 역할을 한다)
    memset(&cp_init, 0, sizeof(cp_init));   //address, value, size
    
    //Pointer to the connection parameters desired by the application. When calling ble_conn_params_init, if this parameter is set to NULL, the connection parameters will be fetched from host. 
    cp_init.p_conn_params                  = NULL;
    //Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (in number of timer ticks). 
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    //Time between each call to sd_ble_gap_conn_param_update after the first (in number of timer ticks). Recommended value 30 seconds as per BLUETOOTH SPECIFICATION Version 4.0. 
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    //Number of attempts before giving up the negotiation. 
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    //If procedure is to be started when notification is started, set this to the handle of the corresponding CCCD. Set to BLE_GATT_HANDLE_INVALID if procedure is to be started on connect event. 
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    //Set to TRUE if a failed connection parameters update shall cause an automatic disconnection, set to FALSE otherwise. 
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void blank_led(void);
static void sleep_mode_enter(void)
{
    uint32_t err_code;
//    // Prepare wakeup buttons.
//    err_code = bsp_btn_ble_sleep_mode_prepare();
//    APP_ERROR_CHECK(err_code);

    blank_led();
    app_pwm_disable(&PWM1);
    nrf_delay_ms(100);
//    app_pwm_disable(&PWM2);
//    nrf_delay_ms(100);
    nrf_gpio_pin_clear(29);
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:  //Fast advertising will connect to any peer device, or filter with a whitelist if one exists.
//            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
//            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:  //Idle; no connectable advertising is ongoing.
            //sleep_mode_enter();
            break;
        default:
            break;
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
//            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}
    
 /**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
    
    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);
    
    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
    //nrf_delay_ms(100);    문제점?????
}

/**@brief Function for handling events from the GATT library. */
//GATT (Generic Attribute Profile) : GATT는 두 BLE 장치간에 Service, Characteristic 을 이용해서 데이터를 주고 받는 방법을 정의한 것입니다.
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

    
/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)    //send data to the phone
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
            {
                NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                NRF_LOG_HEXDUMP_DEBUG(data_array, index);
                do
                {
                    uint16_t length = (uint16_t)index;
                    err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                    if ( (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_BUSY) &&
                         (err_code != NRF_ERROR_NOT_FOUND) )
                    {
                        APP_ERROR_CHECK(err_code);
                    }
                } while (err_code == NRF_ERROR_BUSY);
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}

/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;  //BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE; 
    //advertising 시간 제한을 없애려면 GENERAL_DISC_MODE 설정과 adv_fast(or slow)_timeout=0 설정이 필요함
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;  /**< Event handler that will be called upon advertising events. */

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

 //static void i2s_data_handler(uint32_t const * p_data_received, uint32_t * p_data_to_send,uint16_t number_of_words)
 static void i2s_data_handler(nrf_drv_i2s_buffers_t const * p_released, uint32_t status)
 {
     if (!(status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED))
    {
        return;
    }
//     if(p_data_to_send!=NULL)
//     {
//     }
 }

 void pwm_ready_callback(uint32_t pwm_id)   //pwm callback function
 {
     pwm_ready_flag=true;
 }

    
static void i2s_init(void)
{
    uint32_t err_code;
    nrf_drv_i2s_config_t i2s_config=NRF_DRV_I2S_DEFAULT_CONFIG;
    //i2s_config.sdout_pin=I2S_SDIN_PIN;
    i2s_config.sdin_pin  = I2S_SDIN_PIN;
    i2s_config.sdout_pin=I2S_SDOUT_PIN;
    i2s_config.mck_setup=NRF_I2S_MCK_32MDIV10;
    i2s_config.ratio=NRF_I2S_RATIO_32X;
    i2s_config.channels=NRF_I2S_CHANNELS_STEREO;
    err_code=nrf_drv_i2s_init(&i2s_config,i2s_data_handler);
    APP_ERROR_CHECK(err_code);
    
    //err_code=nrf_drv_i2s_start(&initial_buffers,I2S_BUFFER_SIZE,0);
    //APP_ERROR_CHECK(err_code);
    //nrf_drv_i2s_stop();
    //nrf_drv_i2s_uninit();
    //printf("\r\ni2s_end\r\n");
 }

//  saadc
void adc_timer_handler(nrf_timer_event_t event_type, void * p_context)  //timer event 발생시 호출됨 
{
}
//void saadc_sampling_event_init(void)
//{
//    ret_code_t err_code;
//
//    err_code = nrf_drv_ppi_init();  //Function for initializing PPI module.
//    APP_ERROR_CHECK(err_code);
//
//    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;  //timer config=default
//    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32; //32bit
//    //Function for initializing the timer, timer number, config, timer event handler 
//    err_code = nrf_drv_timer_init(&adc_timer, &timer_cfg, adc_timer_handler); 
//    APP_ERROR_CHECK(err_code);
//
//    /* setup m_timer for compare event every 400ms */
//    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&adc_timer, 400);  //timer to ticks
//    nrf_drv_timer_extended_compare(&adc_timer,
//                                   NRF_TIMER_CC_CHANNEL0,
//                                   ticks,
//                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, //Shortcut between the compare event on the channel and the timer task (STOP or CLEAR)
//                                   false);  //설정한 시간마다 타이머 호출 
//    nrf_drv_timer_enable(&adc_timer); //timer enable
//
//    //Function for returning the address of a specific timer compare event.
//    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&adc_timer,
//                                                                                NRF_TIMER_CC_CHANNEL0);
//    //Function for getting the address of a SAMPLE SAADC task.(sample -> sampling)
//    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();
//
//    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
//    //Function for allocating a PPI channel, This function allocates the first unused PPI channel.
//    err_code = nrf_drv_ppi_channel_alloc(&adc_ppi_channel);
//    APP_ERROR_CHECK(err_code);
//
//    //Function for assigning task and event endpoints to the PPI channel.
//    err_code = nrf_drv_ppi_channel_assign(adc_ppi_channel,
//                                          timer_compare_event_addr,
//                                          saadc_sample_task_addr);
//    APP_ERROR_CHECK(err_code);
//}

void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(adc_ppi_channel);

    APP_ERROR_CHECK(err_code);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)  //Event generated when the buffer is filled with samples.
    {
        //ret_code_t - Indicates success or failure of an API procedure
        ret_code_t err_code;
        /*
        Function for issuing conversion of data to the buffer.
        This function is non-blocking. The application is notified about filling the buffer by the event handler. 
        Conversion will be done on all enabled channels. 
        If the ADC is in idle state, the function will set up Easy DMA for the conversion. 
        The ADC will be ready for sampling and wait for the SAMPLE task. 
        It can be triggered manually by the nrfx_saadc_sample function or by PPI using the NRF_SAADC_TASK_SAMPLE task. 
        If one buffer is already set and the conversion is ongoing, calling this function will result in queuing the given buffer. 
        The driver will start filling the issued buffer when the first one is completed. 
        If the function is called again before the first buffer is filled or calibration is in progress, it will return with error.
        */
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER); //result buffer, buffer size
        APP_ERROR_CHECK(err_code);

        int i;

        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            battery_level+=p_event->data.done.p_buffer[i];
        }
        m_adc_evt_counter++;
    }
    battery_level=(((battery_level*(3.6*(1/16384.0))*100)-140)/70.0)*100;   //max 4.2(2.1), min 2.8(1.4)
    saadc_done=true;
}

void saadc_init(void)
{
    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
     NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN6); //p30
     //NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(PIN_P), PIN_P=Analog Input
    channel_config.burst=NRF_SAADC_BURST_ENABLED;

    //Function for initializing the SAADC, (p_config(If NULL, the default one is used.), event_handler)
    //buffer가 다 차면 driver callback 호출
    err_code = nrf_drv_saadc_init(NULL, saadc_callback);  
    APP_ERROR_CHECK(err_code);

    //Function for initializing an SAADC channel. This function configures and enables the channel.    
    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    //(nrf_saadc_value_t *buffer, uint16_t size)
    err_code = nrf_drv_saadc_buffer_convert(adc_buffer_pool, SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

//    err_code = nrf_drv_saadc_buffer_convert(adc_buffer_pool[1], SAMPLES_IN_BUFFER);
//    APP_ERROR_CHECK(err_code);

}


static void send_rgbw_i2s(void);

void set_color(struct cRGBW *color, uint8_t LEDindex)
 {
    uint8_t i=0;
    struct cRGBW temp_rgbw;
    uint8_t red,green,blue,white;
    uint32_t buffer[4]={0};
    uint32_t err_code;
    nrf_drv_i2s_buffers_t const initial_buffers={.p_tx_buffer=m_buffer_tx, .p_rx_buffer=m_buffer_rx};

    for(i=0;i<8;i++)
    {
        if((color->g<<i)&0x80)  //green
            buffer[0]|=(0xe<<(7-i)*4);
        else
            buffer[0]|=(0x8<<(7-i)*4);
        if((color->r<<i)&0x80)  //red
            buffer[1]|=(0xe<<(7-i)*4);
        else
            buffer[1]|=(0x8<<(7-i)*4);
        if((color->b<<i)&0x80)  //blue
            buffer[2]|=(0xe<<(7-i)*4);
        else
            buffer[2]|=(0x8<<(7-i)*4);
        if((color->w<<i)&0x80)  //white
            buffer[3]|=(0xe<<(7-i)*4);
        else
            buffer[3]|=(0x8<<(7-i)*4);
    }

    for(i=0;i<4;i++)
        m_buffer_tx[4*LEDindex+i]=buffer[i]>>16 | buffer[i]<<16;
    for(i=4*LED_NUMBER;i<I2S_BUFFER_SIZE;i++)
        m_buffer_tx[i]=0;

    if(g_i2s_start && !g_i2s_running)
    {
        err_code=nrf_drv_i2s_start(&initial_buffers,I2S_BUFFER_SIZE,0); //p_initial_buffers, buffer_size, flags
        APP_ERROR_CHECK(err_code);
        g_i2s_running=true;
    }
    //printf("\r\nCurrent Mode : %d\r\n",LED_Mode);
 }

static void button_evt_handler(uint8_t pin_num, uint8_t button_action)
{
    uint32_t err_code;
    total_ticks=0;
    switch(button_action)
    {
        case APP_BUTTON_PUSH :
            app_timer_start(m_button_tick_timer,65535,NULL);
            button_tick=app_timer_cnt_get();
            APP_ERROR_CHECK(err_code);
            break;
        case APP_BUTTON_RELEASE :
            button_tick_release=app_timer_cnt_get();
            err_code=app_timer_stop(m_button_tick_timer);
            APP_ERROR_CHECK(err_code);
            total_ticks=app_timer_cnt_diff_compute(button_tick_release,button_tick);
            APP_ERROR_CHECK(err_code);
            break;
        default : 
            break;
    }
    if(1000<total_ticks && total_ticks<10000)
    {
        LED_Mode++;
//        if(LED_Mode==4)
//            LED_Mode=0;
    }
    else if(10000<total_ticks)
        LED_Mode=6;
}
    
static const app_button_cfg_t button_cfg={NRF_GPIO_PIN_MAP(0,6),APP_BUTTON_ACTIVE_LOW, NRF_GPIO_PIN_PULLUP,button_evt_handler};

 static void blank_led(void)
 {
    uint8_t i=0;
    led[1].r=0; led[1].g=0; led[1].b=0; led[1].w=0;
    app_pwm_disable(&PWM1);
    for(int i=0;i<LED_NUMBER;i++)
    {
        set_color(&led[1],i);
    }
 }

static void always_on(void)
{
    app_pwm_enable(&PWM1);
    pwm_ready_flag=false;
    while(app_pwm_channel_duty_set(&PWM1,0,100)==NRF_ERROR_BUSY);
    led[0].r=0;led[0].g=0;led[0].b=0,led[0].w=255;
    for(int i=0;i<LED_NUMBER;i++)
    {
        set_color(&led[0],i);
    }
    while(LED_Mode==always_on_mode)
    {
    }
}

static void fast_blinky(void)
{
    //led[1]=led[0];
    led[1].r=0; led[1].g=0; led[1].b=0; led[1].w=0;
    uint32_t err_code;
    uint8_t hold_time=10;
    printf("Fast Blinky Mode\r\n");
    while(LED_Mode==fast_blinky_mode)
    {
        app_pwm_enable(&PWM1);
        pwm_ready_flag=false;
        while(app_pwm_channel_duty_set(&PWM1,0,100)==NRF_ERROR_BUSY);
        for(int i=0;i<LED_NUMBER;i++)
        {
            set_color(&led[0],i);
        }
        
        app_timer_start(led_hold_timer,APP_TIMER_TICKS(10),NULL);
        while(LED_Stop_time<hold_time)
        {
        }LED_Stop_time=0;
        app_timer_stop(led_hold_timer);
        
        app_pwm_disable(&PWM1);
//        pwm_ready_flag=false;
//        while(app_pwm_channel_duty_set(&PWM1,0,0)==NRF_ERROR_BUSY);
        for(int i=0;i<LED_NUMBER;i++)
        {
            set_color(&led[1],i);
        }
        app_timer_start(blank_blinky_timer,APP_TIMER_TICKS(10),NULL);
        while(LED_Stop_time<hold_time)
        {
        }LED_Stop_time=0;
        app_timer_stop(blank_blinky_timer);
    }
}

static void normal_blinky(void)
{
    //led[1]=led[0];
    led[1].r=0; led[1].g=0; led[1].b=0; led[1].w=0;
    uint32_t err_code;
    uint8_t hold_time=50;
    printf("Normal Blinky Mode\r\n");
    while(LED_Mode==normal_blinky_mode)
    {
        app_pwm_enable(&PWM1);
        pwm_ready_flag=false;
        while(app_pwm_channel_duty_set(&PWM1,0,100)==NRF_ERROR_BUSY);
        for(int i=0;i<LED_NUMBER;i++)
        {
            set_color(&led[0],i);
        }
        app_timer_start(led_hold_timer,APP_TIMER_TICKS(10),NULL);
        while(LED_Stop_time<hold_time)
        {
        }LED_Stop_time=0;
        app_timer_stop(led_hold_timer);        
//        app_timer_start(blank_blinky_timer,APP_TIMER_TICKS(10),NULL);
//        while(LED_Stop_time<hold_time)
//        {
//        }LED_Stop_time=0;
//        app_timer_stop(blank_blinky_timer);
        
        app_pwm_disable(&PWM1);
//        pwm_ready_flag=false;
//        while(app_pwm_channel_duty_set(&PWM1,0,0)==NRF_ERROR_BUSY);
        for(int i=0;i<LED_NUMBER;i++)
        {
            set_color(&led[1],i);
        }
        app_timer_start(blank_blinky_timer,APP_TIMER_TICKS(10),NULL);
        while(LED_Stop_time<hold_time)
        {
        }LED_Stop_time=0;
        app_timer_stop(blank_blinky_timer);
    }
}
static void soft_blinky(void)
{
    uint8_t i,j;
    uint8_t value=0;    uint8_t step=4;    uint8_t hold_time=70;
    printf("Soft Blinky Mode\r\n");
    while(LED_Mode==soft_blinky_mode)
    {
        app_pwm_enable(&PWM1);
        for(j=0;j<255;j++)
        {
            led[1].r=floor((led[0].r/255.0)*j);
            led[1].g=floor((led[0].g/255.0)*j);
            led[1].b=floor((led[0].b/255.0)*j);
            led[1].w=floor((led[0].w/255.0)*j);
            value=floor((j/255.0)*100);
            for (i=0; i<LED_NUMBER; i++)
            {
                set_color(&led[1],i);
            }
            pwm_ready_flag=false;
            while(app_pwm_channel_duty_set(&PWM1,0,value)==NRF_ERROR_BUSY);
            //app_pwm_channel_duty_set(&PWM1,0,value);
            nrf_delay_ms(step);
            if(LED_Mode!=soft_blinky_mode)
                break;
        }
        app_timer_start(led_hold_timer,APP_TIMER_TICKS(10),NULL);
        while(LED_Stop_time<hold_time)
        {
        }LED_Stop_time=0;
        app_timer_stop(led_hold_timer);
        
        for(j=255;0<j;j--)
        {
            led[1].r=floor((led[0].r/255.0)*j);
            led[1].g=floor((led[0].g/255.0)*j);
            led[1].b=floor((led[0].b/255.0)*j);
            led[1].w=floor((led[0].w/255.0)*j);
            value=floor((j/255.0)*100);
            for (i=0; i<LED_NUMBER; i++)
            {
                set_color(&led[1],i);
            }
            pwm_ready_flag=false;
            while(app_pwm_channel_duty_set(&PWM1,0,value)==NRF_ERROR_BUSY);                
            nrf_delay_ms(step);
            if(LED_Mode!=soft_blinky_mode)
                break;
        }
        blank_led();
        nrf_delay_ms(100);
    }
}

static void circle_blinky()
{
    struct cRGBW circle_array[9];
    uint8_t i;
    uint8_t hold_time=10;
    app_pwm_enable(&PWM1);
    pwm_ready_flag=false;
    while(app_pwm_channel_duty_set(&PWM1,0,100)==NRF_ERROR_BUSY);
    for(i=1;i<=LED_NUMBER;i++)
    {
        circle_array[i-1].r=floor((led[0].r*i)/8);
        circle_array[i-1].g=floor((led[0].g*i)/8);
        circle_array[i-1].b=floor((led[0].b*i)/8);
        circle_array[i-1].w=floor((led[0].w*i)/8);
    }
    for(i=0;i<LED_NUMBER;i++)
    {
        set_color(&circle_array[i],i);
    }
    
    while(LED_Mode==circle_mode)
    {
        circle_array[LED_NUMBER]=circle_array[LED_NUMBER-1];
        for(i=LED_NUMBER;1<i;i--)
            circle_array[i-1]=circle_array[i-2];
        circle_array[0]=circle_array[LED_NUMBER];
        for(i=0;i<LED_NUMBER;i++)
        {
            set_color(&circle_array[i],i);
        }
        
        if(compare_rgbw==false)
        {
            for(i=1;i<=LED_NUMBER;i++)
            {
                circle_array[i-1].r=floor((led[0].r*i)/8);
                circle_array[i-1].g=floor((led[0].g*i)/8);
                circle_array[i-1].b=floor((led[0].b*i)/8);
                circle_array[i-1].w=floor((led[0].w*i)/8);
            }
            compare_rgbw=true;
        }      
        app_timer_start(blank_blinky_timer,APP_TIMER_TICKS(10),NULL);
        while(LED_Stop_time<hold_time)
        {
        }LED_Stop_time=0;
        app_timer_stop(blank_blinky_timer);
        if(LED_Mode!=circle_mode)
                break;
        for(int i=0;i<8;i++)        
            printf("%d  ",circle_array[i].w);
        printf("\r\n");
    }
}

static void flower_blinky()
{
    uint8_t i,j;
    uint8_t center=128; uint8_t brightness=0;
    //led[0],2,4,8-짝
    //led[1],3,5,7-홀
    //led[1].r=0;led[1].g=0;led[1].b=0;led[1].w=center;
    struct cRGBW flower[LED_NUMBER]={0};
    flower[1].r=0;flower[1].g=0;flower[1].b=0;flower[1].w=center;
    
    printf("Flower Mode\r\n");
    
    while(LED_Mode==flower_mode)
    {
        while(brightness<255)
        {
            flower[0].r=floor((led[0].r/255.0)*brightness);
            flower[0].g=floor((led[0].g/255.0)*brightness);
            flower[0].b=floor((led[0].b/255.0)*brightness);
            flower[0].w=floor((led[0].w/255.0)*brightness);
            
            for(j=2;j<LED_NUMBER;j+=2)  //rgb
            {
                flower[j]=flower[0];
            }
            
            if(center<=brightness)
            {
                flower[1].w+=1;
            }
            else if(brightness<center)
            {
                flower[1].w-=1;
            }
            for(j=1;j<LED_NUMBER;j+=2)  //w
                flower[j].w=flower[1].w;
            for(j=0;j<LED_NUMBER;j++)
            {
                set_color(&flower[j],j);
            }
            brightness++;
            nrf_delay_ms(5);
            if(LED_Mode!=flower_mode)
                break;
        }
//        if(LED_Mode!=flower_mode)
//            break;
        while(0<brightness)
        {
            flower[0].r=floor((led[0].r/255.0)*brightness);
            flower[0].g=floor((led[0].g/255.0)*brightness);
            flower[0].b=floor((led[0].b/255.0)*brightness);
            flower[0].w=floor((led[0].w/255.0)*brightness);
        
            for(j=2;j<LED_NUMBER;j+=2)  //rgb
            {
                flower[j]=flower[0];
            }
        
            if(center<=brightness)
            {
                flower[1].w+=1;
            }
            else if(brightness<center)
            {
                flower[1].w-=1;
            }
            for(j=1;j<LED_NUMBER;j+=2)  //w
                flower[j].w=flower[1].w;
        
            for(j=0;j<LED_NUMBER;j++)
            {
                set_color(&flower[j],j);
            }
            nrf_delay_ms(5);
            brightness--;
            if(LED_Mode!=flower_mode)
                break;
        }
//        if(LED_Mode!=flower_mode)
//            break;
    }
}           


static void timer_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);
}
 
static void blank_blinky_timer_handler(void * p_context)
{
    LED_Stop_time++;
}
    
static void led_hold_timer_handler(void * p_context)
{
    LED_Stop_time++;
    for (int i=0; i<LED_NUMBER; i++)
    {
            set_color(&led[0],i);
    }
}
    
 static void button_init(void)
 {
    uint32_t err_code;
    uint32_t timer;
    err_code = app_timer_create(&m_button_tick_timer, APP_TIMER_MODE_REPEATED, timer_handler);
    err_code = app_timer_create(&led_hold_timer, APP_TIMER_MODE_REPEATED, led_hold_timer_handler);
    err_code = app_timer_create(&blank_blinky_timer, APP_TIMER_MODE_REPEATED, blank_blinky_timer_handler);
    APP_ERROR_CHECK(err_code);    
    err_code=app_button_init(&button_cfg,1,APP_TIMER_TICKS(50));
    APP_ERROR_CHECK(err_code);
    app_button_enable();
}

static void pwm_init()
{
    uint32_t err_code;
    app_pwm_config_t pwm1_cfg=APP_PWM_DEFAULT_CONFIG_1CH(5000L,NRF_GPIO_PIN_MAP(0, 8));
    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
    err_code=app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);
    
//    app_pwm_config_t pwm2_cfg=APP_PWM_DEFAULT_CONFIG_1CH(5000L,NRF_GPIO_PIN_MAP(0, 29));
//    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
//    err_code=app_pwm_init(&PWM2,&pwm2_cfg,pwm_ready_callback);
//    APP_ERROR_CHECK(err_code);
//    app_pwm_enable(&PWM2);
//    pwm_ready_flag=false;
//    while(app_pwm_channel_duty_set(&PWM2,0,100)==NRF_ERROR_BUSY);
}

static void measure_battery()
{
    uint32_t err_code;
    err_code=nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
    nrf_delay_ms(1000);
    while(saadc_done==false)
    {
        err_code=nrf_drv_saadc_sample();
        APP_ERROR_CHECK(err_code);
    }
    //printf("Battery : %d\r\n",battery_level);
    
	if(90<=battery_level)
	{
		led[0].r=0;led[0].g=0;led[0].b=0,led[0].w=255;
	}
	else if(70<=battery_level)
	{
		led[0].r=10;led[0].g=10;led[0].b=255,led[0].w=0;
	}
	else if(50<=battery_level)
	{
		led[0].r=10;led[0].g=255;led[0].b=10,led[0].w=0;
	}
	else if(30<=battery_level)
	{
		led[0].r=255;led[0].g=200;led[0].b=10,led[0].w=0;
	}
	else if(10<=battery_level)
	{
		led[0].r=255;led[0].g=100;led[0].b=10,led[0].w=0;
	}
	else if(battery_level<=10)
	{
		led[0].r=255;led[0].g=1;led[0].b=1,led[0].w=0;
	}
    pwm_ready_flag=false;
    while(app_pwm_channel_duty_set(&PWM1,0,100)==NRF_ERROR_BUSY);
    for(int i=0;i<LED_NUMBER;i++)
        set_color(&led[0],i);
    nrf_delay_ms(3000);
}

int main(void)
{
    //uint32_t err_code=NRF_SUCCESS;
    uint32_t err_code;
    bool erase_bonds;
    //struct cRGBW led[LED_NUMBER];
    bool saadc_chk;
    uint32_t battery;
    double test;
    
    uart_init();
    log_init();
    timers_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    i2s_init();
    nrf_drv_i2s_buffers_t const initial_buffers={.p_tx_buffer=m_buffer_tx, .p_rx_buffer=m_buffer_rx};
    /* Configure board. */
    //bsp_board_init(BSP_INIT_LEDS);  
    
    
    button_init();
    app_button_disable();
    pwm_init();
    saadc_init();
    
    nrf_gpio_cfg_output(29);
    nrf_gpio_pin_set(29);
//    err_code=nrf_drv_saadc_sample();
//    APP_ERROR_CHECK(err_code);
//    nrf_saadc_enable();
    
    //i2s 
    //MCK=L0+R0+L1+R1
    //LRCK=MCK/32   (ex:L0=8*sck)
    //SCK=2*LRCK*16
    //set led color에서 한 개의 비트를 보내는데 걸리는 시간  = 1/(MCK/32)=0.31us (=L0 or R0 or L1 or R1)
    //즉, L or R 의 크기는 1/(MCK/32)가 되고 L0,R0,L1,R1이 4bit가 한 묶음이 되어 0 or 1을 표현하고
    //그 각각의 값은 R or G or B 32bit 배열의 한 bit가 된다.
    //버퍼의 값이 e일 경우, e는 1110이므로 1이 되고  8은 1000이므로 0으로 표현된다.
    //ex)0xeeee8888 의 경우(4bit*8=32bit), RGB LED 에선 0x11110000으로 인식된다고 볼 수 있고
    //이는 (0111(=L0+R0+L1+R1(=MCK/32))*4+(1000)*4 이므로 총 8개의  L0,R0,L1,R1 묶음(4*8=32bit)으로 이루어졌다고 볼 수 있다.
    //따라서 2진수 8비트의 경우, 최대값은 255
        
//    // PWM
//    app_pwm_config_t pwm1_cfg=APP_PWM_DEFAULT_CONFIG_1CH(19L,27);  //period_in_us, pin name
//    pwm1_cfg.pin_polarity[1]=APP_PWM_POLARITY_ACTIVE_HIGH;
//    err_code=app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
//    APP_ERROR_CHECK(err_code);
//    app_pwm_enable(&PWM1);
//    

    
    advertising_start();
    
    led[0].w=255;
    
    //nrf_drv_saadc_sample_task_get();
    
    //nrfx_saadc_sample();

    
//    for(int i=0;i<5;i++)
//    {
//        err_code=nrf_drv_saadc_sample();
//        APP_ERROR_CHECK(err_code);
//        while(saadc_done==false)
//        {
//            err_code=nrf_drv_saadc_sample();
//            APP_ERROR_CHECK(err_code);
//        }
//        printf("Battery : %d\r\n",battery_level);
//    }
    
    measure_battery();
    app_button_enable();
    LED_Mode=0;
    printf("\r\nMain Start\r\n");
    while (true)
    {
        printf("\r\nMode : %d\r\n",LED_Mode);
        switch(LED_Mode)
        {
            case 0 : always_on();
            break;
            case 1 : fast_blinky();
            break;
            case 2 : normal_blinky();
            break;
            case 3 : soft_blinky();
            break;
            case 4 : circle_blinky();
            break;
            case 5 : flower_blinky();
            break;
            case 6 : sleep_mode_enter();
            break;
        }
    }
}

/**
 *@}
 **/
