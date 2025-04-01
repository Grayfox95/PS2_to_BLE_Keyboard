/* PS2 interface file

*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

#include "soc/gpio_struct.h"
#include "ps2_dev.h"
#include "hid_dev.h"

/* Local defines*/
#define PS2_TAG "PS2"
#define PS2_COFFEE_LED  0
#define PS2_CLK_GPIO_4     4
#define PS2_DATA_GPIO_5     5
#define PS2_COFFEE_GPIO_0_SEL  (1ULL<<PS2_COFFEE_LED)
#define PS2_CLK_GPIO_4_SEL  (1ULL<<PS2_CLK_GPIO_4)
#define PS2_DATA_GPIO_5_SEL  (1ULL<<PS2_DATA_GPIO_5)

#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t ps2_tx_event_queue = NULL; // Contains the events for the transmission.
static QueueHandle_t ps2_rx_code_queue = NULL; // Contains each byte achieved from ps2 data signal.
QueueHandle_t ps2_tx_code_queue = NULL; // Contains each ps2 code arrived from keyboard.
// QueueHandle_t ps2_led_code_queue = NULL; // Contains each ps2 led code arrived from PC.

static PS2_STATE_TYPE ps2_connection_f = PS2_IDLE;

static uint8_t ps2_tx_code[8] = {0};
static uint8_t ps2_tx_parity = 0;
static uint8_t ps2_curr_led_status = 0;
static uint8_t ps2_new_led_status = 0;
static bool ps2_update_led_f = false;
static uint8_t ps2_coffee_f = false;


/* This function is the callback for the CLK line. This is called at each CLK transition    */
/* and works in transmission and reception mode. When the keyboard send a scan code, it     */
/* reads each byte on the data line and save it in a specific queue.                        */

static void IRAM_ATTR ps2_clk_isr_handler(void* arg)
{
    static uint8_t n = 1;
    static uint8_t k = 0;
    PS2_TX_EVENT_TYPE ps2_tx_event;

    uint8_t value = 0;
    int64_t time = 0;

    static uint8_t code = 0;
    static uint8_t parity = 1;
    static uint8_t parity_read = 1;
    static int64_t prev_time = 0;

    // Check time for prevent glitches. 
    time = esp_timer_get_time();

    // Check if we are sending data.
    if(ps2_connection_f == PS2_HOST_RTS)
    {
        if(time - prev_time < 60)
        {
            // This is to avoid clock glitches.
            return;
        }
        // Sending data.
        switch(n)
        {
            case 0:
                break;
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
                // Send bit in data line.
                gpio_set_level(PS2_DATA_GPIO_5, ps2_tx_code[n-1]);
                break;
            case 9:
                // Send parity bit in data line.
                gpio_set_level(PS2_DATA_GPIO_5, ps2_tx_parity);
                break;
            case 10:
                // Send Stop bit.
                gpio_set_level(PS2_DATA_GPIO_5, 1);
                // Release data line.
                gpio_set_direction(PS2_DATA_GPIO_5, GPIO_MODE_DEF_INPUT);
                break;
            case 11:
                // ACK bit.
                // Release data line.
                gpio_set_direction(PS2_DATA_GPIO_5, GPIO_MODE_DEF_INPUT);
                ps2_connection_f = PS2_IDLE;
                ps2_tx_event = PS2_TX_COMPLETE;
                
                n = 1;
                xQueueSendFromISR(ps2_tx_event_queue, &ps2_tx_event, NULL);
                prev_time = time;
                return;
            default:
                ps2_connection_f = PS2_IDLE;
                ps2_tx_event = PS2_TX_FAILED;
                n=1;
                prev_time = time;
                return;
        }
        prev_time = time;
        n++;
    }

    // Check if we are reading data.
    else if(ps2_connection_f != PS2_HOST_RTS || ps2_connection_f != PS2_HOST_RETRANSMISSION)
    {
        // Read data from the GPIO5.
        value = (uint8_t) gpio_get_level(PS2_DATA_GPIO_5);
        

        // Check the delta time from previous bit.
        if (time - prev_time > 200)
        {
            // Set that we are starting a reception phase.
            ps2_connection_f = PS2_HOST_RECEPTION;
            // Reset counters.
            k = 0;
            code = 0x00;
            parity = 0x01;
        }
        else if(time - prev_time < 60)
        {
            // This is to avoid clock glitches.
            return;
        }
        switch(k)
        {
            case 0:
                break;
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
                // It is a data bit, update the code value.
                code = code | (value << (k-1));
                if (value)
                {
                    parity ^= 0x01;
                }
                break;
            case 9:
                parity_read = value;
                break;
            case 10:
                if(parity_read != parity)
                {
                    // Parity error. Set clock as output.
                    gpio_set_direction(PS2_CLK_GPIO_4, GPIO_MODE_DEF_OUTPUT);

                    // Set the clock LOW for at least 100 us.
                    gpio_set_level(PS2_CLK_GPIO_4, 0);

                    // Set flag to retransmission.
                    ps2_connection_f = PS2_HOST_RETRANSMISSION;
                }
                else
                {
                    // Data has been received correctly, send it to the queue.
                    xQueueSendFromISR(ps2_rx_code_queue, &code, NULL);
                    ps2_connection_f = PS2_IDLE;
                }
                // fallthrough;
            default:
                // Reset counters.
                k = 0x00;
                code = 0x00;
                parity = 0x01;
                prev_time = time;
                return;
        }
        prev_time = time;
        k++;
    }
}


PS2_RESULT_E ps2_retransmission(void)
{
    esp_err_t res = ESP_FAIL;
    PS2_TX_EVENT_TYPE ps2_tx_event;
    
    // Build the command to be sent. 
    uint8_t i = 0;
    for(i=0; i<8; i++)
    {
        ps2_tx_code[i] = (0xFE >> i) & 0x01;
    }
    ps2_tx_parity = 0x00;

    // The clock should be already OFF. Set data as output.
    res = gpio_set_direction(PS2_DATA_GPIO_5, GPIO_MODE_DEF_OUTPUT);
    if(res != ESP_OK)
    {
        ESP_LOGE(PS2_TAG, "%s clock line gpio_set_direction() failed", __func__);
        return PS2_RESULT_FAIL;
    }

    // Set DATA LOW.
    res = gpio_set_level(PS2_DATA_GPIO_5, 0);

    // Release the CLK line.
    res = gpio_set_direction(PS2_CLK_GPIO_4, GPIO_MODE_DEF_INPUT);

    // Set the Request-to-Send status.
    ps2_connection_f = PS2_HOST_RTS;
    //ESP_LOGI(PS2_TAG, "Waiting event\n");

    // Wait until the data has been sent.
    if (xQueueReceive(ps2_tx_event_queue, &ps2_tx_event, portMAX_DELAY))
    {
        if(ps2_tx_event == PS2_TX_COMPLETE)
        {
            // ESP_LOGI(PS2_TAG, "Transmission completed\n");
            // Now we are ready to read the new data.
            ps2_connection_f = PS2_HOST_RECEPTION;
            return PS2_RESULT_SUCCESS;
        }
    }
    ESP_LOGE(PS2_TAG, "Error during Transmission...\n");
    // Something wrong happened. Release data line.
    gpio_set_direction(PS2_DATA_GPIO_5, GPIO_MODE_DEF_INPUT);
    // Release the CLK line.
    res = gpio_set_direction(PS2_CLK_GPIO_4, GPIO_MODE_DEF_INPUT);
    // Set the IDLE status.
    ps2_connection_f = PS2_IDLE;
    // Enable again the CLK interrupt.
    // res = gpio_isr_handler_add(PS2_CLK_GPIO_4, ps2_clk_isr_handler, (void*) PS2_CLK_GPIO_4);
    return PS2_RESULT_FAIL;
}

PS2_RESULT_E ps2_send_code(uint8_t tx_data)
{
    esp_err_t res = ESP_FAIL;
    ps2_connection_f = PS2_NONE;
    PS2_TX_EVENT_TYPE ps2_tx_event;
    uint8_t i = 0;
    
    // Prepare data to be sent
    ps2_tx_parity = 0x01;
    for (i = 0; i < 8; i++)
    {
        ps2_tx_code[i] = tx_data & 0x01;
        if(ps2_tx_code[i] == 1)
        {
            ps2_tx_parity = !ps2_tx_parity;
        }

        tx_data = tx_data >> 1;
    }
    // Set clock as output.
    res = gpio_set_direction(PS2_CLK_GPIO_4, GPIO_MODE_DEF_OUTPUT);
    if(res != ESP_OK)
    {
        ESP_LOGE(PS2_TAG, "%s clock line gpio_set_direction() failed", __func__);
        return PS2_RESULT_FAIL;
    }
    // Set the clock LOW for at least 100 us.
    res = gpio_set_level(PS2_CLK_GPIO_4, 0);

    ets_delay_us(100);

    // Set data as output.
    res = gpio_set_direction(PS2_DATA_GPIO_5, GPIO_MODE_DEF_OUTPUT);
    if(res != ESP_OK)
    {
        ESP_LOGE(PS2_TAG, "%s clock line gpio_set_direction() failed", __func__);
        return PS2_RESULT_FAIL;
    }

    // Set DATA LOW.
    res = gpio_set_level(PS2_DATA_GPIO_5, 0);

    // Release the CLK line.
    res = gpio_set_direction(PS2_CLK_GPIO_4, GPIO_MODE_DEF_INPUT);

    // Set the Request-to-Send status.
    ps2_connection_f = PS2_HOST_RTS;

    // Wait until the data has been sent.
    if (xQueueReceive(ps2_tx_event_queue, &ps2_tx_event, portMAX_DELAY))
    {
        if(ps2_tx_event == PS2_TX_COMPLETE)
        {
            return PS2_RESULT_SUCCESS;
        }

    }
    ESP_LOGE(PS2_TAG, "Error during Transmission...\n");
    // Something wrong happened. Release data line.
    gpio_set_direction(PS2_DATA_GPIO_5, GPIO_MODE_DEF_INPUT);
    // Release the CLK line.
    res = gpio_set_direction(PS2_CLK_GPIO_4, GPIO_MODE_DEF_INPUT);
    // Set the IDLE status.
    ps2_connection_f = PS2_IDLE;
    // Enable again the CLK interrupt.
    // res = gpio_isr_handler_add(PS2_CLK_GPIO_4, ps2_clk_isr_handler, (void*) PS2_CLK_GPIO_4);
    return PS2_RESULT_FAIL;
}

PS2_RESULT_E ps2_code_sorting(uint8_t code, ps2_code_t * ps2_code)
{
    static uint8_t extended_f = 0;
    static uint8_t break_f = 0;
    PS2_RESULT_E res = PS2_RESULT_FAIL;

    switch(code)
    {
        case 0xFA:
            // ACK code has been received.
            ps2_code->type = ACK_CODE;
            // Set the correct code value.
            ps2_code->code = code;
            res = PS2_RESULT_ACK;
        break;
        case 0xE0:
            // Extended code has been received.
            extended_f = 1;
            res = PS2_RESULT_WAITING;
        break;
        case 0xF0:
            // Break code has been received.
            break_f = 1;
            res = PS2_RESULT_WAITING;
        break;
        case 0xFE:
            // Invalid argument.
            ps2_code->type = ERROR_CODE;
        break;
        default:
            // Set the correct code type.
            if(break_f)
            {
                ps2_code->type = BREAK_CODE;
            }
            else
            {
                ps2_code->type = MAKE_CODE;
            }
            // Set the extended_f.
            ps2_code->extended_f = extended_f;

            // Set the correct code value.
            ps2_code->code = code;

            res = PS2_RESULT_SUCCESS;
            // Reset the flags.
            extended_f = 0;
            break_f = 0;
        break;
    }
    return res;
}

PS2_RESULT_E ps2_receive_code(ps2_code_t * ps2_code)
{
    static uint8_t code = 0;

    ps2_code_t ps2_special_code;
    PS2_RESULT_E res = PS2_RESULT_FAIL;
    esp_err_t res2;

    // Check for new data from the queue.
    while (xQueueReceive(ps2_rx_code_queue, &code, 10 / portTICK_PERIOD_MS))
    {
        // Print the received code.
        //printf("0x%X\n", code);

        // Convert the received data to ps2_code
        res = ps2_code_sorting(code, ps2_code);
        if(res == PS2_RESULT_SUCCESS)
        {
            // Check if it is a special code (codes that replicated shortcuts).
            if(ps2_code->extended_f == true)
            {
                switch(ps2_code->code)
                {
                    case 0X42:  // Sleep key -> WIN+L.
                        if(ps2_code->type == MAKE_CODE)
                        {
                            ps2_special_code = (ps2_code_t) {MAKE_CODE, true, 0x1F}; // WIN key
                            xQueueSend(ps2_tx_code_queue, &ps2_special_code, pdMS_TO_TICKS(5));
                            ps2_special_code = (ps2_code_t) {MAKE_CODE, false, 0x4B};   // L key
                            xQueueSend(ps2_tx_code_queue, &ps2_special_code, pdMS_TO_TICKS(5));
                        }
                        else if(ps2_code->type == BREAK_CODE)
                        {
                            ps2_special_code = (ps2_code_t) {BREAK_CODE, true, 0x1F}; // WIN key
                            xQueueSend(ps2_tx_code_queue, &ps2_special_code, pdMS_TO_TICKS(5));
                            ps2_special_code = (ps2_code_t) {BREAK_CODE, false, 0x4B};   // L key
                            xQueueSend(ps2_tx_code_queue, &ps2_special_code, pdMS_TO_TICKS(5));
                        }
                    return PS2_RESULT_SUCCESS;
                    case 0X43:  // Close key -> ALT+F4.
                        if(ps2_code->type == MAKE_CODE)
                        {
                            ps2_special_code = (ps2_code_t) {MAKE_CODE, false, 0x11}; // Left ALT key
                            xQueueSend(ps2_tx_code_queue, &ps2_special_code, pdMS_TO_TICKS(5));
                            ps2_special_code = (ps2_code_t) {MAKE_CODE, false, 0x0C};   // F4 key
                            xQueueSend(ps2_tx_code_queue, &ps2_special_code, pdMS_TO_TICKS(5));
                        }
                        else if(ps2_code->type == BREAK_CODE)
                        {
                            ps2_special_code = (ps2_code_t) {BREAK_CODE, false, 0x11}; // Left ALT key
                            xQueueSend(ps2_tx_code_queue, &ps2_special_code, pdMS_TO_TICKS(5));
                            ps2_special_code = (ps2_code_t) {BREAK_CODE, false, 0x0C};   // F4 key
                            xQueueSend(ps2_tx_code_queue, &ps2_special_code, pdMS_TO_TICKS(5));
                        }
                    return PS2_RESULT_SUCCESS;
                    case 0X4B:  // Menu key -> CTRL+ALT+CANC.
                        if(ps2_code->type == MAKE_CODE)
                        {
                            ps2_special_code = (ps2_code_t) {MAKE_CODE, false, 0x14}; // Left CTRL key
                            xQueueSend(ps2_tx_code_queue, &ps2_special_code, pdMS_TO_TICKS(5));
                            ps2_special_code = (ps2_code_t) {MAKE_CODE, false, 0x11}; // Left ALT key
                            xQueueSend(ps2_tx_code_queue, &ps2_special_code, pdMS_TO_TICKS(5));
                            ps2_special_code = (ps2_code_t) {MAKE_CODE, true, 0x71};   // Del key
                            xQueueSend(ps2_tx_code_queue, &ps2_special_code, pdMS_TO_TICKS(5));
                        }
                        else if(ps2_code->type == BREAK_CODE)
                        {
                            ps2_special_code = (ps2_code_t) {BREAK_CODE, false, 0x14}; // Left CTRL key
                            xQueueSend(ps2_tx_code_queue, &ps2_special_code, pdMS_TO_TICKS(5));
                            ps2_special_code = (ps2_code_t) {BREAK_CODE, false, 0x11}; // Left ALT key
                            xQueueSend(ps2_tx_code_queue, &ps2_special_code, pdMS_TO_TICKS(5));
                            ps2_special_code = (ps2_code_t) {BREAK_CODE, true, 0x71};   // Del key
                            xQueueSend(ps2_tx_code_queue, &ps2_special_code, pdMS_TO_TICKS(5));
                        }
                    return PS2_RESULT_SUCCESS;
                    case 0X65:  // Coffee button. 
                        if(ps2_code->type == BREAK_CODE)
                        {
                            ps2_coffee_f = !ps2_coffee_f;
                             res2 = gpio_set_level(PS2_COFFEE_LED, !ps2_coffee_f);
                                if(res2 != ESP_OK)
                                {
                                    // GPIO configuration wrong
                                    ESP_LOGE(PS2_TAG, "%s clock line gpio_config() failed", __func__);
                                    return PS2_RESULT_FAIL;
                                }

                            //gpio_set_level(PS2_COFFEE_LED, 1);
                             ESP_LOGI(PS2_TAG, "Coffee: %d", ps2_coffee_f);
                            
                        }
                    return PS2_RESULT_SUCCESS;
                }
            }
            // PS2 code can be sent to the HID queue.
            xQueueSend(ps2_tx_code_queue, ps2_code, pdMS_TO_TICKS(5));
        }
        else if(res == PS2_RESULT_ACK)
        {
            // Received code is ACK, return success in this case.
            return PS2_RESULT_SUCCESS;
        }
    }
    return res;
}

PS2_RESULT_E ps2_lines_init(void)
{
    esp_err_t res = ESP_FAIL;
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};


    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    // io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO5 here
    io_conf.pin_bit_mask = PS2_CLK_GPIO_4_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;

    res = gpio_config(&io_conf);
    if(res != ESP_OK)
    {
        // GPIO configuration wrong
        ESP_LOGE(PS2_TAG, "%s clock line gpio_config() failed", __func__);
        return PS2_RESULT_FAIL;
    }

    // Disable the interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4 here
    io_conf.pin_bit_mask = PS2_DATA_GPIO_5_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;

    res = gpio_config(&io_conf);
    if(res != ESP_OK)
    {
        // GPIO configuration wrong
        ESP_LOGE(PS2_TAG, "%s data line gpio_config() failed", __func__);
        return PS2_RESULT_FAIL;
    }

    // Disable the interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO0 here
    io_conf.pin_bit_mask = PS2_COFFEE_GPIO_0_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //enable pull-up mode
     io_conf.pull_down_en = 1;

    res = gpio_config(&io_conf);
    if(res != ESP_OK)
    {
        // GPIO configuration wrong
        ESP_LOGE(PS2_TAG, "%s coffee gpio_config() failed", __func__);
        return PS2_RESULT_FAIL;
    }
    return PS2_RESULT_SUCCESS;
}

// Da provare qual è la funzione che inizializza l'interrupt e quali funzioni posso essere richiamate nel main
PS2_RESULT_E ps2_clk_interrupt_init(void)
{
        esp_err_t res = ESP_FAIL;
        // Install gpio isr service.
        res = gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
        if(res != ESP_OK)
        {
            ESP_LOGE(PS2_TAG, "%s gpio_install_isr_service() failed", __func__);
            return PS2_RESULT_FAIL;
        }
        // Hook isr handler for specific gpio pin.
        res = gpio_isr_handler_add(PS2_CLK_GPIO_4, ps2_clk_isr_handler, (void*) PS2_CLK_GPIO_4);
        if(res != ESP_OK)
        {
            ESP_LOGE(PS2_TAG, "%s gpio_isr_handler_add() failed", __func__);
            return PS2_RESULT_FAIL;
        }
        // Remove isr handler for gpio number.
        res = gpio_isr_handler_remove(PS2_CLK_GPIO_4);
        // Hook isr handler for specific gpio pin again.
        res = gpio_isr_handler_add(PS2_CLK_GPIO_4, ps2_clk_isr_handler, (void*) PS2_CLK_GPIO_4);
        if(res != ESP_OK)
        {
            ESP_LOGE(PS2_TAG, "%s gpio_isr_handler_add() failed", __func__);
            return PS2_RESULT_FAIL;
        }
        return PS2_RESULT_SUCCESS;
}


void ps2_update_led(uint8_t ps2_led_status)
{
    // Conversion from HID LED code to PS2.
    uint8_t scroll_lock = (ps2_led_status >> 2) & 0X01;
    ps2_led_status = (ps2_led_status << 1) & 0x07;
    ps2_led_status |=  scroll_lock;

    // Check if the status needs to be updated.
    if(ps2_curr_led_status != ps2_led_status)
    {   
        ps2_new_led_status = ps2_led_status;
        ps2_update_led_f = true;
    }
}
    

PS2_RESULT_E ps2_set_led(uint8_t ps2_led_val)
{
    PS2_RESULT_E res = PS2_RESULT_FAIL;
    ps2_code_t ps2_code;

    // Send SET LED command 0xED.
    res = ps2_send_code(0xED);
    if(res == PS2_RESULT_SUCCESS)
    {
        // Wait for the reception of PS2 ACK code.
        res = ps2_receive_code(&ps2_code);
        if(ps2_code.type == ACK_CODE)
        {
            // Send the LED value.
            res = ps2_send_code(ps2_led_val);
            if(res == PS2_RESULT_SUCCESS )
            {
                // Wait for the reception of PS2 ACK code.
                res = ps2_receive_code(&ps2_code);
                if(ps2_code.type == ACK_CODE)
                {
                    // Update the new led status.
                    ps2_curr_led_status = ps2_led_val;
                    return PS2_RESULT_SUCCESS;
                }
                else
                {
                    ESP_LOGE(PS2_TAG, "ERROR sending led status");
                    return PS2_RESULT_FAIL; 
                }
            }
        }
        else
        {
            ESP_LOGE(PS2_TAG, "ERROR sending 0xED command");
            return PS2_RESULT_FAIL;
        }
    }
    return PS2_RESULT_FAIL;
}

void ps2_coffee_mode_start(void)
{
    ps2_code_t ps2_coffee_code;
    ESP_LOGI(PS2_TAG, "Coffee mode start");
    ps2_coffee_code = (ps2_code_t) {MAKE_CODE, true, 0x7C}; // Print Screen
    xQueueSend(ps2_tx_code_queue, &ps2_coffee_code, pdMS_TO_TICKS(5));
    ps2_coffee_code = (ps2_code_t) {BREAK_CODE, true, 0x7C}; // Print Screen
    xQueueSend(ps2_tx_code_queue, &ps2_coffee_code, pdMS_TO_TICKS(5));
}

void ps2_low_power_mode_start(void)
{
    // Set GPIO5 (PS2 CLK) as external source for deep sleep wakeup
    esp_sleep_enable_ext1_wakeup( 1ULL << PS2_CLK_GPIO_4, ESP_EXT1_WAKEUP_ALL_LOW); 
    // Set pullup and disable pulldown to prevent unwanted gpio triggers.
    rtc_gpio_pullup_en(PS2_CLK_GPIO_4);
    rtc_gpio_pulldown_dis(PS2_CLK_GPIO_4);

    ESP_LOGI(PS2_TAG, "Enter in deep sleep mode");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    // Enter in deep sleep mode.
    esp_deep_sleep_start();

    ESP_LOGI(PS2_TAG, "Exit from deep sleep mode");
    
    // Reset status of GPIO4.
    rtc_gpio_deinit(PS2_CLK_GPIO_4);

}

void ps2_data_task(void* arg)
{
    static int64_t inactivity_time = 0;
    static int64_t coffee_time = 0;
    int64_t curr_time = 0;

    PS2_RESULT_E res = PS2_RESULT_FAIL;
    ps2_code_t ps2_code;

    // Queue code creation. 
    ps2_rx_code_queue = xQueueCreate(500, sizeof(uint8_t)); // Queue for the hex code from the keyboard
    ps2_tx_code_queue = xQueueCreate(300, sizeof(ps2_code_t)); // Queue for ps2 converted code for HID task
    ps2_tx_event_queue = xQueueCreate(2, sizeof(PS2_TX_EVENT_TYPE)); // Queue for PS2 TX events

    // Initialize GPIO for ps2 lines.
    res = ps2_lines_init();
    if(res != PS2_RESULT_SUCCESS)
    {
        ESP_LOGE(PS2_TAG, "%s ps2_lines_init() failed", __func__);
        return;
    }

    // Initialize the clk interrupt
    res = ps2_clk_interrupt_init();
    if(res != PS2_RESULT_SUCCESS)
    {
        ESP_LOGE(PS2_TAG, "%s ps2_clk_interrupt_init() failed", __func__);
        return;
    }

    // Turn off coffee led
    gpio_set_level(PS2_COFFEE_LED, 1);
    // ESP_LOGI(PS2_TAG, "Starting PS2 Task");
    for (;;) {
        
        if(hid_dev_is_connected() == true )
        {
            // Save current time for inactivity.
            curr_time = esp_timer_get_time();
            // inactivity_time = curr_time - prev_time;

            // Check for a new received PS2 code.
            res = ps2_receive_code(&ps2_code);
            if(res == PS2_RESULT_SUCCESS)
            {
                // If a new code has been pressed, reset inactivity time.
                inactivity_time = curr_time;
            }

            if(ps2_connection_f == PS2_HOST_RETRANSMISSION)
            {
                // There is a parity error, let's execute a retransmission.
                ESP_LOGE(PS2_TAG, "PS2 parity error.");
                ps2_retransmission();
            }
            // Check if we need to update led status.
            if((res != PS2_RESULT_WAITING) && (ps2_update_led_f == true) && (ps2_connection_f == PS2_IDLE))
            {
                // Try to turn on the LED for at least 3 times.
                for(uint8_t i = 0; i < 3; i++)
                {
                    vTaskDelay(2 / portTICK_PERIOD_MS);
                    res = ps2_set_led(ps2_new_led_status);
                    if(res == PS2_RESULT_SUCCESS)
                    {
                        break;
                    }
                }
                ps2_update_led_f = false;
            }
            // Check for coffee time.
            if((ps2_coffee_f == true) && (curr_time-coffee_time ) > COFFEE_TIMEOUT)
            {
                // Start coffee procedure.
                ps2_coffee_mode_start();
                //Reset coffee time.
                coffee_time = curr_time;
                // Reset also the inacitivity timeout
                inactivity_time = curr_time;
            }

            // Check if the inactivity time elapsed. 
            if((curr_time-inactivity_time ) > INACTIVITY_TIMEOUT)
            {
                // Enter in low power mode and reset timer.
                ps2_low_power_mode_start();
                inactivity_time = esp_timer_get_time();
            }
        }   
    }
}

