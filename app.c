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
#include "app_log.h"
#include "sl_sensor_rht.h"
#include "temperature.h"
#include "sl_bt_api.h"
#include "gatt_db.h"

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

/**************************************************************************//**
 * Application Initialization Function
 *****************************************************************************/
SL_WEAK void app_init(void)
{
    // Add your additional application initialization code here!
    // This function is called once during startup.

    // Log a message indicating the function entry
    app_log_info("%s\n", __FUNCTION__);
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
  void myTimerCallback(sl_sleeptimer_timer_handle_t *handle, void *data) {
      static int step = 1;
      app_log_info("Timer step %d\n", step++);
  }
  void displayTemperature(int16_t temperature) {
      printf("Temperature: %d\n", temperature);
      // Your code to display the temperature in your application
  }


  // Define your timer callback function
  sl_sleeptimer_timer_callback_t myCallback = myTimerCallback; // Rename to myCallback
  sl_sleeptimer_timer_handle_t timer_handle; // Declaration of timer_handle
  uint32_t timeout_ms = 1000;  // Some constant value for the timeout
  void *callbackData = NULL;  // No specific callback data required
  uint8_t priority = 0;  // Some constant value for priority
  uint16_t option_flags = 0;  // Some constant value
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
                                         sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      app_log_info("%s: connection_opened!\n", __FUNCTION__);
      sc = sl_sensor_rht_init();


      break;
    case sl_bt_evt_gatt_server_user_read_request_id:
    {
        app_log_info("%s: Read button opened!\n", __FUNCTION__);
        int16_t bleTemperature;
        sl_status_t status = getAndConvertTemperatureToBLE(&bleTemperature);

        if (status == SL_STATUS_OK) {
            app_log_info("Temperature (BLE format): %d\n", bleTemperature);
        } else {
            app_log_info("Failed to get temperature (Status: %lu)\n", (unsigned long)status);
        }

        // Check for gattdb_temperature_0 characteristic
        const sl_bt_evt_gatt_server_user_read_request_t *read_request = &evt->data.evt_gatt_server_user_read_request;

        if (read_request->characteristic == gattdb_temperature_0) {
            app_log_info("Read request for gattdb_temperature_0 detected.\n");
            // Handle gattdb_temperature_0 read request here if needed

            // Prepare to send response
            uint16_t sent_len = 0;
            uint8_t att_errorcode = 0; // Assuming no error
            uint8_t response_value[] = {0x00, 0x01, 0x02}; // Example response value
            size_t response_value_len = sizeof(response_value) / sizeof(response_value[0]);

            // Send user read response
            sl_status_t response_status = sl_bt_gatt_server_send_user_read_response(
                read_request->connection,
                read_request->characteristic,
                att_errorcode,
                response_value_len,
                response_value,
                &sent_len
            );

            // Check if response is sent successfully and log accordingly
            if (response_status == SL_STATUS_OK) {
                app_log_info("User read response sent successfully.\n");
            } else {
                app_log_info("Failed to send user read response (Status: %lu)\n", (unsigned long)response_status);
            }
        }
    }
    break;
    case sl_bt_evt_gatt_server_characteristic_status_id: {
        const sl_bt_evt_gatt_server_characteristic_status_t *status_evt = &evt->data.evt_gatt_server_characteristic_status;

        if (status_evt->characteristic == gattdb_temperature_0) {
            if (status_evt->client_config_flags & gatt_notification) {
                app_log_info("Notify received for temperature characteristic. Starting timer...\n");

                // Start the timer when Notify is received
                sl_status_t status = sl_sleeptimer_start_periodic_timer(
                    &timer_handle,
                    timeout_ms,
                    myTimerCallback,
                    callbackData,  // Pass the address of callback data
                    priority,
                    option_flags
                );

                // Check status and handle errors if any
                if (status != SL_STATUS_OK) {
                    app_log_info("Failed to start the timer.\n");
                    // Handle the failure to start the timer
                } else {
                    // Get and display the temperature when the notification is received
                    int16_t bleTemperature;
                    sl_status_t tempStatus = getAndConvertTemperatureToBLE(&bleTemperature);
                    if (tempStatus == SL_STATUS_OK) {
                        displayTemperature(bleTemperature);
                    } else {
                        app_log_info("Failed to get temperature (Status: %lu)\n", (unsigned long)tempStatus);
                        // Handle the failure to get temperature
                    }
                }
            } else {
                app_log_info("Notify not received for temperature characteristic. Stopping timer...\n");

                // Stop the timer if Notify is not received
                sl_status_t stop_timer_status = sl_sleeptimer_stop_timer(&timer_handle);

                // Check status and handle errors if any
                if (stop_timer_status != SL_STATUS_OK) {
                    app_log_info("Failed to stop the timer.\n");
                    // Handle the failure to stop the timer
                }
            }
        }
    }
    break;
    case sl_bt_evt_gatt_server_user_write_request_id: {
        const sl_bt_evt_gatt_server_user_write_request_t *write_request = &evt->data.evt_gatt_server_user_write_request;

        // Check if the write request is for the Digital characteristic
        if (write_request->characteristic == gattdb_digital_characteristic) {
            app_log_info("Write request for digital characteristic detected.\n");

            // Access the written value and its length
            const uint8_t *data = write_request->value.data;
            size_t data_len = write_request->value.len;

            // Process the written value
            // Here you can use 'data' and 'data_len' to handle the written value as needed
        }
    }
    break;







    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      app_log_info("%s: connection_closed!\n", __FUNCTION__);
      sl_sensor_rht_deinit();

      // Generate data for advertising
      sc = sl_bt_legacy_advertiser_generate_data(advertising_set_handle,
                                                 sl_bt_advertiser_general_discoverable);
      app_assert_status(sc);

      // Restart advertising after client has disconnected.
      sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                         sl_bt_advertiser_connectable_scannable);
      app_assert_status(sc);
      break;



    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
