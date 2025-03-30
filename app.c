/***************************************************************************//**
 * @file
 * @brief Core application logic.
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************/
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "app.h"
#include <stdint.h>
#include <stdbool.h>
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_iadc.h"
#include "gatt_db.h"

#define NUM_SAMPLES         7

#define LE_MONITOR_SIGNAL     0x01
/***************************************************************************//**
 * @brief
 *    IADC Configuration Definitions.
 ******************************************************************************/
// Set CLK_ADC to 10 MHz
#define CLK_SRC_ADC_FREQ    40000000  // CLK_SRC_ADC
#define CLK_ADC_FREQ        10000000  // CLK_ADC - 100 kHz max in normal mode

// When changing GPIO port/pins above, make sure to change xBUSALLOC macro's
// accordingly.
#define IADC_INPUT_0_PORT_PIN     iadcPosInputPortAPin4;
#define IADC_INPUT_1_PORT_PIN     iadcPosInputPortAPin5;
#define IADC_INPUT_2_PORT_PIN     iadcPosInputPortAPin6;
#define IADC_INPUT_3_PORT_PIN     iadcPosInputPortAPin0;
#define IADC_INPUT_4_PORT_PIN     iadcPosInputPortBPin0;
#define IADC_INPUT_5_PORT_PIN     iadcPosInputPortCPin0;
#define IADC_INPUT_6_PORT_PIN     iadcPosInputPortCPin1;

#define IADC_INPUT_0_BUS          ABUSALLOC
#define IADC_INPUT_0_BUSALLOC     GPIO_ABUSALLOC_AEVEN0_ADC0

#define IADC_INPUT_1_BUS          ABUSALLOC
#define IADC_INPUT_1_BUSALLOC     GPIO_ABUSALLOC_AODD0_ADC0

#define IADC_INPUT_2_BUS          ABUSALLOC
#define IADC_INPUT_2_BUSALLOC     GPIO_ABUSALLOC_AEVEN0_ADC0

#define IADC_INPUT_3_BUS          ABUSALLOC
#define IADC_INPUT_3_BUSALLOC     GPIO_ABUSALLOC_AEVEN0_ADC0

#define IADC_INPUT_4_BUS          BBUSALLOC
#define IADC_INPUT_4_BUSALLOC     GPIO_BBUSALLOC_BEVEN0_ADC0

#define IADC_INPUT_5_BUS          CDBUSALLOC
#define IADC_INPUT_5_BUSALLOC     GPIO_CDBUSALLOC_CDEVEN0_ADC0

#define IADC_INPUT_6_BUS          CDBUSALLOC
#define IADC_INPUT_6_BUSALLOC     GPIO_CDBUSALLOC_CDODD0_ADC0




uint8_t flag=1;

// Buffer to store IADC samples
volatile uint16_t scanResult[NUM_SAMPLES+2];

static uint8_t connection_handle;

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;





  #if defined(BSP_WSTK_BRD4181A) || defined(BSP_WSTK_BRD4182A)
  // HFXO frequency set for BRD4181A and BRD4182A
  #define HFXO_FREQ               38400000
  #else
  // HFXO frequency for rest of radio boards
  #define HFXO_FREQ               39000000
  #endif


  void initCMU(void)
  {
    // Initialization structure for HFXO configuration
    CMU_HFXOInit_TypeDef hfxoInit = CMU_HFXOINIT_DEFAULT;

    // Check if device has HFXO configuration information in DEVINFO page
    if (DEVINFO->MODULEINFO & DEVINFO_MODULEINFO_HFXOCALVAL) {
      // Use the DEVINFO page's CTUNE values
      hfxoInit.ctuneXoAna = (DEVINFO->MODXOCAL & DEVINFO_MODXOCAL_HFXOCTUNEXOANA_DEFAULT)
          >> _DEVINFO_MODXOCAL_HFXOCTUNEXOANA_SHIFT;
      hfxoInit.ctuneXiAna = (DEVINFO->MODXOCAL & DEVINFO_MODXOCAL_HFXOCTUNEXIANA_DEFAULT)
          >> _DEVINFO_MODXOCAL_HFXOCTUNEXIANA_SHIFT;
    }

    // Configure HFXO settings
    CMU_HFXOInit(&hfxoInit);

    // Set system HFXO frequency
    SystemHFXOClockSet(HFXO_FREQ);

    // Enable HFXO oscillator, and wait for it to be stable
    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

    // Select HFXO as the EM01GRPA clock
    CMU_ClockSelectSet(cmuClock_EM01GRPACLK, cmuSelect_HFXO);
  }


  /**************************************************************************//**
   * @brief  IADC initialization
   *****************************************************************************/
  void initIADC(void)
  {
    // Declare initialization structures
    IADC_Init_t init = IADC_INIT_DEFAULT;
    IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
    IADC_InitScan_t initScan = IADC_INITSCAN_DEFAULT;

    // Scan table structure
    IADC_ScanTable_t scanTable = IADC_SCANTABLE_DEFAULT;

    CMU_ClockEnable(cmuClock_IADC0, true);

    // Use the EM01GRPACLK as the IADC clock
    CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_EM01GRPACLK);

    // Shutdown between conversions to reduce current
    init.warmup = iadcWarmupNormal;

    // Set the HFSCLK prescale value here
    init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0, CLK_SRC_ADC_FREQ, 0);

    /*
     * The IADC local timer runs at CLK_SRC_ADC_FREQ, which is at least
     * 2x CLK_ADC_FREQ. CLK_SRC_ADC_FREQ in this example is equal to the
     * HFXO frequency. Dividing the frequency of the HFXO by 1000 will give
     * the tick count for 1 ms trigger rate.
     * For example - if HFXO freq = 38.4 MHz,
     *
     * ticks for 1 ms trigger = 38400000 / 1000
     * ticks =  38400
     * for 50Hz: 38400000/768000=50 Hz
     */
    init.timerCycles = CMU_ClockFreqGet(cmuClock_IADCCLK)/8;

    /*
     * Configuration 0 is used by both scan and single conversions by
     * default.  Use internal bandgap as the reference and specify the
     * reference voltage in mV.
     *
     * Resolution is not configurable directly but is based on the
     * selected oversampling ratio (osrHighSpeed), which defaults to
     * 2x and generates 12-bit results.
     */
    initAllConfigs.configs[0].reference = iadcCfgReferenceVddx;
    initAllConfigs.configs[0].vRef = 3300;
    initAllConfigs.configs[0].osrHighSpeed = iadcCfgOsrHighSpeed2x;
    initAllConfigs.configs[0].analogGain = iadcCfgAnalogGain1x;

    /*
     * CLK_SRC_ADC must be prescaled by some value greater than 1 to
     * derive the intended CLK_ADC frequency.
     *
     * Based on the default 2x oversampling rate (OSRHS)...
     *
     * conversion time = ((4 * OSRHS) + 2) / fCLK_ADC
     *
     * ...which results in a maximum sampling rate of 833 ksps with the
     * 2-clock input multiplexer switching time is included.
     */
    initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                       CLK_ADC_FREQ,
                                                                       0,
                                                                       iadcCfgModeNormal,
                                                                       init.srcClkPrescale);

    /*
     * The IADC local timer triggers conversions.
     *
     * Set the SCANFIFODVL flag when scan FIFO holds 2 entries.  In this
     * example, the interrupt associated with the SCANFIFODVL flag in
     * the IADC_IF register is not used.
     *
     * Tag each FIFO entry with scan table entry ID.
     */
    initScan.triggerSelect = iadcTriggerSelTimer;
    initScan.dataValidLevel = iadcFifoCfgDvl2;
    initScan.showId = true;
    /*
     * Trigger continuously once scan is started.  Note that
     * initScan.start = false by default, so conversions must be started
     * manually with IADC_command(IADC0, iadcCmdStartScan).
     *
     * Configure entries in the scan table.  CH0 is single-ended from
     * input 0; CH1 is single-ended from input 1.
     */
    scanTable.entries[0].posInput = IADC_INPUT_0_PORT_PIN;
    scanTable.entries[0].negInput = iadcNegInputGnd;
    scanTable.entries[0].includeInScan = true;

    scanTable.entries[1].posInput = IADC_INPUT_1_PORT_PIN;
    scanTable.entries[1].negInput = iadcNegInputGnd;
    scanTable.entries[1].includeInScan = true;

    scanTable.entries[2].posInput = IADC_INPUT_2_PORT_PIN;
    scanTable.entries[2].negInput = iadcNegInputGnd;
    scanTable.entries[2].includeInScan = true;

    scanTable.entries[3].posInput = IADC_INPUT_3_PORT_PIN;
    scanTable.entries[3].negInput = iadcNegInputGnd;
    scanTable.entries[3].includeInScan = true;

    scanTable.entries[4].posInput = IADC_INPUT_4_PORT_PIN;
    scanTable.entries[4].negInput = iadcNegInputGnd;
    scanTable.entries[4].includeInScan = true;

    scanTable.entries[5].posInput = IADC_INPUT_5_PORT_PIN;
    scanTable.entries[5].negInput = iadcNegInputGnd;
    scanTable.entries[5].includeInScan = true;

    scanTable.entries[6].posInput = IADC_INPUT_6_PORT_PIN;
    scanTable.entries[6].negInput = iadcNegInputGnd;
    scanTable.entries[6].includeInScan = true;

    // Initialize IADC
    IADC_init(IADC0, &init, &initAllConfigs);

    // Initialize scan
    IADC_initScan(IADC0, &initScan, &scanTable);

    // Enable the IADC timer (must be after the IADC is initialized)
    IADC_command(IADC0, iadcCmdEnableTimer);

    // Allocate the analog bus for ADC0 inputs

    GPIO->IADC_INPUT_0_BUS |= IADC_INPUT_0_BUSALLOC;
    GPIO->IADC_INPUT_1_BUS |= IADC_INPUT_1_BUSALLOC;
    GPIO->IADC_INPUT_2_BUS |= IADC_INPUT_2_BUSALLOC;
    GPIO->IADC_INPUT_3_BUS |= IADC_INPUT_3_BUSALLOC;
    GPIO->IADC_INPUT_4_BUS |= IADC_INPUT_4_BUSALLOC;
    GPIO->IADC_INPUT_5_BUS |= IADC_INPUT_5_BUSALLOC;
    GPIO->IADC_INPUT_6_BUS |= IADC_INPUT_6_BUSALLOC;

    // Enable scan interrupts
    IADC_enableInt(IADC0, IADC_IEN_SCANFIFODVL);

    // Enable ADC interrupts
    NVIC_ClearPendingIRQ(IADC_IRQn);
    NVIC_EnableIRQ(IADC_IRQn);
  }


  void IADC_IRQHandler(void)
    {
      IADC_Result_t sample;


      // While the FIFO count is non-zero...
      while (IADC_getScanFifoCnt(IADC0))
      {
        // Pull a scan result from the FIFO
        sample = IADC_pullScanFifoResult(IADC0);

        /*
         * Calculate the voltage converted as follows:
         *
         * For single-ended conversions, the result can range from 0 to
         * +Vref, i.e., for Vref = VBGR = 3V, and with analog gain = 1,
         * 0xFFF represents the full scale value of 3.3V.
         */

        scanResult[sample.id+1] = sample.data ; //* 3.3 / 0xFFF
        scanResult[0] =  101011;
        scanResult[8] = 100011 ;
      }
      sl_bt_external_signal(LE_MONITOR_SIGNAL);

      /*
       * Clear the scan table complete interrupt.  Reading from the FIFO
       * does not do this automatically.
       */
      IADC_clearInt(IADC0, IADC_IF_SCANFIFODVL);
    }




/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{

  initCMU();

  initIADC();


  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{



  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application code here!                              //
  // This is called infinitely.                                              //
  // Do not call blocking functions from here!                               //
  /////////////////////////////////////////////////////////////////////////////
}

/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{
  sl_status_t sc;

  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&advertising_set_handle);
      app_assert_status(sc);

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Set advertising interval to 100ms.
      sc = sl_bt_advertiser_set_timing(
        advertising_set_handle,
        160, // min. adv. interval (milliseconds * 1.6)
        160, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      app_assert_status(sc);
      // Start advertising and enable connections.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      connection_handle = evt->data.evt_connection_opened.connection;
      break;

    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Restart advertising after client has disconnected.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_legacy_advertiser_connectable);
      app_assert_status(sc);
      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////
    case sl_bt_evt_gatt_server_characteristic_status_id:
          // Check if Average Voltage Characteristic changed
          if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_voltage_array) {

            // client characteristic configuration changed by remote GATT client
            if(sl_bt_gatt_server_client_config == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags) {

              // Check if EFR Connect App enabled notifications
              if(sl_bt_gatt_disable != evt->data.evt_gatt_server_characteristic_status.client_config_flags) {
                // Start sampling data
                  IADC_command(IADC0, iadcCmdStartScan);
              }
              // indication and notifications disabled
              else {
                  IADC_command(IADC0, iadcCmdStopScan);

              }
            }
          }
          break;


    case sl_bt_evt_system_external_signal_id:

          // External signal triggered from iADC interrupt
          if(evt->data.evt_system_external_signal.extsignals & LE_MONITOR_SIGNAL) {


            // Notify connected user
            sc = sl_bt_gatt_server_send_notification(connection_handle,
                                                     gattdb_voltage_array,
                                                     sizeof(scanResult),
                                                     (uint8_t *)&scanResult);


          }
          break;
    // -------------------------------
    // Default event handler.
    default:
      break;
  }


  /**************************************************************************//**
   * @brief  IADC interrupt handler
   *****************************************************************************/


}
