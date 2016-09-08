/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_drv_twi.h"
#include "config.h"
#include "MPU6050.h"
#include "LSM6DS3.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define DEVICE_NAME                     "nRF52 ZIKTO"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            (2 + BSP_APP_TIMERS_NUMBER)                 /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

const char MSG_LINE[] = "===========================================\r\n";
const char MID_LINE[] =   "-------------------------------------------\r\n\n";
#define START_STRING     "          **   LSM6DS3 MOCKUP  **\n"                                /**< The string that will be sent over the UART when the application starts. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

//#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
//#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */


static const nrf_drv_twi_t m_twi_master = NRF_DRV_TWI_INSTANCE(MASTER_TWI_INST);
void printHelp();
void func1(void);
static ret_code_t write_register(size_t addr, uint8_t pdata);
static ret_code_t read_register(size_t addr, uint8_t * pdata);
ret_code_t I2CwriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
ret_code_t I2CwriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
static ret_code_t twi_master_init(void);
void readData();
void fifoBegin();
uint16_t fifoGetStatus(void);
uint16_t fifoRead( void );

typedef struct{
	struct{
			float x;
			float y;
			float z;
	} accel;
	
	struct{
			float x;
			float y;
			float z;
	} gyro;
	
}data_t;



static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */
static bool rx_received = false;
static uint8_t rx_data[20];
data_t data = {{0,0,0},{0,0,0}};





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


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    printf("rx : ") ;
		rx_received = true;
	for (uint32_t i = 0; i < length; i++)
    {
      while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
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
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
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
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
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
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
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
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in] p_ble_evt S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

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

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a S110 SoftDevice event to all modules with a S110 SoftDevice 
 *        event handler.
 *
 * @details This function is called from the S110 SoftDevice event interrupt handler after a S110 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    
}


/**@brief Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);

    // Enable BLE stack.
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#if (defined(S130) || defined(S132))
    ble_enable_params.gatts_enable_params.attr_tab_size   = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
#endif
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
        if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }

}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_ENABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud38400
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Application main function.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
    uint8_t  start_string[] = START_STRING;
    
    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
    uart_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    
	 
		printf("%s",MSG_LINE);
		printf("%s",start_string);
		printf("\n");
		printf("%s",MID_LINE);

    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    
	  err_code = twi_master_init();
    APP_ERROR_CHECK(err_code);
    printHelp();
    // Enter main loop.
    for (;;)
    {
      uint8_t c = 0;
			while(c == 0)
        {
          c = getchar();
        }
        switch((char)c)
        {
        default:
					printf("You selected %c\n", (char)c);
				
				case '\n':
        case '\r':
            break;
        
				case '1':
            func1();
            break;
        
				case '2':
            readData();
            break;
				
				case '?':
						printHelp();
				
        }
				//power_manage();
        }
		
 
}// end main


/** 
 * @}
 */

void printHelp(){
	printf("     COMMAND LIST  \n");
	printf("1 :  Initialise Accel Gyro\n");
	printf("2 :  Get Instantial Data\n");
	printf("? :  Print Command List\n");
						
}

void func1(void){
	printf("mpu init()\n");
	printf("LMS6DS3 Default Address : 0x%.4x\n",LSM6DS3_SLAVE_ADDR);
	
	uint8_t rd;
	printf("Testing Connection... ");
	read_register(LSM6DS3_ACC_GYRO_WHO_AM_I_REG,  &rd);
	//printf("who_am_i reg : 0x%.2x\n",rd);
	printf("%s\n", (rd==(LSM6DS3_SLAVE_ADDR-0x2))?"Successful":"Failed" );
	
	
	uint8_t dataToWrite = 0;
	
	
	printf("Set Accelerometer..\n");
	dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz; // BandWith = 100hz
	dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_4g;
	dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_208Hz;
	write_register(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);
	read_register(LSM6DS3_ACC_GYRO_CTRL4_C,  &rd);
	dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);
	if(true) dataToWrite |= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED; // supposed to be if(settings.accelODRoff)
	write_register(LSM6DS3_ACC_GYRO_CTRL4_C, dataToWrite);
	
	printf("Set Gyrometer..\n");
	dataToWrite = 0;
	dataToWrite |= LSM6DS3_ACC_GYRO_FS_G_2000dps;
	dataToWrite |= LSM6DS3_ACC_GYRO_ODR_G_208Hz;
	write_register(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);
	
	printf("FIFO Setup..\n");
	fifoBegin();
	//readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
	
	//write_register(MPU6050_RA_GYRO_CONFIG,  MPU6050_GYRO_FS_250<<3); // Gyro Config Addr : 0x1B
  //	write_register(MPU6050_RA_ACCEL_CONFIG,  MPU6050_ACCEL_FS_2<<3); // Accel Config Addr : 0x1C
	
	//I2CwriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, false);
	
	// Test Connection
	
	
	
	
}



void fifoBegin(){
	
	int threshold = 3000;
	
	uint8_t thresholdLByte = threshold & 0x00FF;
	uint8_t thresholdHByte = (threshold & 0x0F00) >> 8;
	//Pedo bits not configured (ctl2)

	//CONFIGURE FIFO_CTRL3
	uint8_t tempFIFO_CTRL3 = 0;
	
	if (true) // settings.gyroFifoEnabled == 1
	{
		//Set up gyro stuff
		//Build on FIFO_CTRL3
		//Set decimation
		tempFIFO_CTRL3 |= (1 & 0x07) << 3;

	}
	if (true) // settings.accelFifoEnabled == 1
	{
		//Set up accelerometer stuff
		//Build on FIFO_CTRL3
		//Set decimation
		tempFIFO_CTRL3 |= ( 1 & 0x07); // 1 = settings.accelFifoDecimation
	}

	//CONFIGURE FIFO_CTRL4  (nothing for now-- sets data sets 3 and 4
	uint8_t tempFIFO_CTRL4 = 0;

	//CONFIGURE FIFO_CTRL5
	uint8_t tempFIFO_CTRL5 = 0;
	tempFIFO_CTRL5 |= LSM6DS3_ACC_GYRO_ODR_FIFO_10Hz;

	//Hard code the fifo mode here:
	//tempFIFO_CTRL5 |= settings.fifoModeWord = 6;  //set mode:

	//Write the data
	write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL1, thresholdLByte);
	//Serial.println(thresholdLByte, HEX);
	write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL2, thresholdHByte);
	//Serial.println(thresholdHByte, HEX);
	write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL3, tempFIFO_CTRL3);
	write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL4, tempFIFO_CTRL4);
	write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL5, tempFIFO_CTRL5);

}


uint16_t fifoGetStatus(){
	uint8_t tempReadByte = 0;
	uint16_t tempAccumulator = 0;
	read_register(LSM6DS3_ACC_GYRO_FIFO_STATUS1,&tempReadByte);
	tempAccumulator = tempReadByte;
	read_register(LSM6DS3_ACC_GYRO_FIFO_STATUS2,&tempReadByte);
	tempAccumulator |= (tempReadByte << 8);
}

void readData(){
	int16_t fifoData = 0;
	uint8_t tempReadByte;

	// Accel x
	if(read_register(LSM6DS3_ACC_GYRO_OUTX_L_XL,&tempReadByte)) return;
	fifoData = tempReadByte;
	if(read_register(LSM6DS3_ACC_GYRO_OUTX_H_XL,&tempReadByte)) return;
	fifoData |= (tempReadByte << 8);
	
	data.accel.x = (float) fifoData/835.918367;
	
	// Accel y
	if(read_register(LSM6DS3_ACC_GYRO_OUTY_L_XL,&tempReadByte)) return;
	fifoData = tempReadByte;
	if(read_register(LSM6DS3_ACC_GYRO_OUTY_H_XL,&tempReadByte)) return;
	fifoData |= (tempReadByte << 8);
	
	data.accel.y = (float) fifoData/835.918367;
	
	// Accel z
	if(read_register(LSM6DS3_ACC_GYRO_OUTZ_L_XL,&tempReadByte)) return;
	fifoData = tempReadByte;
	if(read_register(LSM6DS3_ACC_GYRO_OUTZ_H_XL,&tempReadByte)) return;
	fifoData |= (tempReadByte << 8);
	
	data.accel.z = (float) fifoData/835.918367;
	
	char data_wr[20];
	sprintf(data_wr,"%2.2f,%2.2f,%2.2f ",data.accel.x, data.accel.y, data.accel.z);
	printf("accel data : %s\n",data_wr);
	ble_nus_string_send(&m_nus, data_wr, 20);

}


static ret_code_t read_register(size_t addr, uint8_t * pdata){
	 ret_code_t ret;
    int i = 0;
		do
    {
       uint8_t addr8 = (uint8_t)addr;
       ret = nrf_drv_twi_tx(&m_twi_master, LSM6DS3_SLAVE_ADDR, &addr8, 1, true);
       if(NRF_SUCCESS != ret)
       {
				 break;
       }
       ret = nrf_drv_twi_rx(&m_twi_master, LSM6DS3_SLAVE_ADDR, pdata,1, false);
			 i++;
    }while(0);
		
    return ret;
}

static ret_code_t write_register(size_t addr, uint8_t pdata)
{
    ret_code_t ret;
		printf("writing 0x%02x on REG 0x%02x..\n",pdata,addr);
    do
    {
        uint8_t addr8 = (uint8_t)addr;
        ret = nrf_drv_twi_tx(&m_twi_master, LSM6DS3_SLAVE_ADDR, &addr8, 1, true);
        if(NRF_SUCCESS != ret)
        {
          break;
        }
        ret = nrf_drv_twi_tx(&m_twi_master, LSM6DS3_SLAVE_ADDR, &pdata, 1, false);
    }while(0);
    return ret;
}

ret_code_t I2CwriteBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {

		uint8_t b;
	
    if (read_register(regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte
        return write_register(regAddr, b);
    } else {
        return NRF_ERROR_NULL;
    }
}

ret_code_t I2CwriteBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    read_register(regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return write_register(regAddr, b);
}



static ret_code_t twi_master_init(void)
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
    {
       .scl                = TWI_SCL_M,
       .sda                = TWI_SDA_M,
       .frequency          = NRF_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    do
    {
        ret = nrf_drv_twi_init(&m_twi_master, &config, NULL);
        if(NRF_SUCCESS != ret)
        {
            break;
        }
        nrf_drv_twi_enable(&m_twi_master);
    }while(0);
    return ret;
}